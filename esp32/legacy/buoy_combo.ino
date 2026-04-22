/* buoy_combo.ino */
#include "BotleticsSIM7000.h"
#include <HardwareSerial.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#if defined(ARDUINO_ARCH_ESP32)
#include "base64.h"
#else
#include <Base64.h>
#endif
#include <Arduino.h>
#include "BluetoothSerial.h"

// Check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Check Serial Port Profile
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;

#include "secrets.h" 
#include "buoy_combo.h"

// Pin Definitions
#define SIMCOM_7000
#define BOTLETICS_PWRKEY 18
#define RST 5
#define TX_MODEM 17  // ESP32 TX1 to Modem RX
#define RX_MODEM 16  // ESP32 RX1 to Modem TX
// #define TX_GPS 12    // ESP32 TX2 to GPS RX
// #define RX_GPS 27    // ESP32 RX2 to GPS TX


// Global Objects
HardwareSerial modemSS(1);     // UART1 to modem
HardwareSerial gpsSerial(2);   // UART2 to GPS
Botletics_modem_LTE modem = Botletics_modem_LTE();
SFE_UBLOX_GNSS myGNSS;

// Flags
bool networkConnected = false;
bool gprsEnabled = false;
bool gpsEnabled = false;
bool ntripConnected = false;
bool gpsUARTOnline = false;
volatile bool shutdownRequested = false;

// Timing
long lastReceivedRTCM_ms = 0;
int maxTimeBeforeHangup_ms = 100000;
const unsigned long ntripRetryInterval = 30000;
unsigned long lastNTRIPAttempt = 0;
long lastGPSPrint = 0;

// Configuration
uint8_t type;
char replybuffer[255];
char imei[16] = {0};

// Bluetooth Name
String device_name = "RTK-GPS-BT";

void setup() {
  // USB Debug Serial
  Serial.begin(9600);
  SerialBT.begin(device_name);
  Serial.println("Connecting to bluetooth...");
  
  // Use if bluetooth seems to be acting up
  // SerialBT.deleteAllBondedDevices(); // Uncomment this to delete paired devices; Must be called after begin
  delay(2000);
  Serial.println(F("\n=== Buoy Combo - UART GPS ==="));
  SerialBT.println("\n=== Buoy Combo - UART GPS ===");

  // Setting Pins
  pinMode(STATUS_LED, OUTPUT);
  pinMode(SHUTDOWN_BTN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SHUTDOWN_BTN), shutdownISR, FALLING);


  initialize_gnss_uart_f();

  // Initialize Modem
  pinMode(RST, OUTPUT);
  digitalWrite(RST, HIGH);
  
  Serial.println(F("Powering on modem..."));
  SerialBT.println(F("Powering on modem..."));
  modem.powerOn(BOTLETICS_PWRKEY);

  // Start modem serial
  modemSS.begin(115200, SERIAL_8N1, TX_MODEM, RX_MODEM);
  Serial.println(F("Configuring modem to 9600 baud"));
  SerialBT.println(F("Configuring modem to 9600 baud"));
  modemSS.println("AT+IPR=9600");
  delay(1000);
  modemSS.begin(9600, SERIAL_8N1, TX_MODEM, RX_MODEM);

  if (!modem.begin(modemSS)) {
    Serial.println(F("Couldn't find modem"));
    SerialBT.println(F("Couldn't find modem"));
    while (1);
  }

  type = modem.type();
  Serial.println(F("SIM7000 detected"));
  SerialBT.println(F("SIM7000 detected"));
  
  uint8_t imeiLen = modem.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print(F("Module IMEI: ")); 
    SerialBT.println(F("Module IMEI: "));
    Serial.println(imei);
    SerialBT.println(imei);
  }

  modem.setFunctionality(1);
  modem.setNetworkSettings(F("hologram"));
  
  Serial.println(F("Setup complete!\n"));
  SerialBT.println(F("Setup complete\n"));
}

void loop() {
  // Handle user AT commands
  if (Serial.available()) {
    Serial.print(F("modem> "));
    SerialBT.println(F("modem>" ));
    while (Serial.available()) {
      modemSS.write(Serial.read());
    }
    delay(100);
    while (modemSS.available()) {
      Serial.write(modemSS.read());
    }
    return;
  }
  
  // Network management
  network_status_check_f();
  enable_gprs_f();
  
  // NTRIP connection management
  if (gprsEnabled && !ntripConnected && 
      (millis() - lastNTRIPAttempt > ntripRetryInterval)) {
    beginNTRIPClient();
    lastNTRIPAttempt = millis();
  }

  // Handle NTRIP data (receives RTCM and sends to GPS via UART)
  if (ntripConnected) {
    handleNTRIPData();
  }

  // May cause problems and its not really needed unless debugging, if needed maybe use i2c for print_gps_status_f() - Amara
  //  Print GPS status periodically
  // if (gpsUARTOnline && (millis() - lastGPSPrint > 1000)) {
  //   lastGPSPrint = millis();
  //   print_gps_status_f();
  // }
  
  monitor_connection_health();
  
  updateStatusLED();

  // Check for shutdown request
  if (shutdownRequested) {
    gracefulShutdown();
  }

  delay(10);  // Reduced from 1000ms for better responsiveness
}