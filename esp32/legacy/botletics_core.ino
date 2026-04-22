/*
  ESP32 + SIM7000 LTE + GPS
  This is the version that just uses the botletics code needed to connect to the LTE network
  and get GPS data.
  It does not include the NTRIP client code, which is in esp32_rtk.ino
*/

#include "BotleticsSIM7000.h"
#define SIMCOM_7000

// For botletics SIM7000 shield with ESP32
#define BOTLETICS_PWRKEY 18
#define RST 5
#define TX 17 // ESP32 hardware serial RX2 (GPIO16)
#define RX 16 // ESP32 hardware serial TX2 (GPIO17)

// For ESP32 hardware serial
#include <HardwareSerial.h>
HardwareSerial modemSS(1);
Botletics_modem_LTE modem = Botletics_modem_LTE();

// Global Variables
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
uint8_t type;
char replybuffer[255];
char imei[16] = {0};

bool networkConnected = false;
bool gprsEnabled = false;
bool gpsEnabled = false;

void setup() {
  // Initialize Serial FIRST
  Serial.begin(9600);
  Serial.println(F("Starting - Amara"));
  
  pinMode(RST, OUTPUT);
  digitalWrite(RST, HIGH); // Default state

  // Turn on the module by pulsing PWRKEY low
  Serial.println(F("Powering on modem..."));
  modem.powerOn(BOTLETICS_PWRKEY);
  
  Serial.println(F("ESP32 SIMCom Basic Test"));
  Serial.println(F("Initializing....(May take several seconds)"));

  // Start at default SIM7000 shield baud rate
  modemSS.begin(115200, SERIAL_8N1, TX, RX);

  Serial.println(F("Configuring to 9600 baud"));
  modemSS.println("AT+IPR=9600");
  delay(1000); // Give more time for baud rate change
  modemSS.begin(9600, SERIAL_8N1, TX, RX);
  
  if (!modem.begin(modemSS)) {
    Serial.println(F("Couldn't find modem"));
    while (1); // Don't proceed if it couldn't find the device
  }
  
  type = modem.type();
  Serial.println(F("Modem is OK"));
  Serial.println(F("SIM7000 detected"));
  
  // Print module IMEI number
  uint8_t imeiLen = modem.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("Module IMEI: "); 
    Serial.println(imei);
  }
  
  modem.setFunctionality(1); // AT+CFUN=1
  modem.setNetworkSettings(F("hologram")); // For Hologram SIM card
  
  Serial.println(F("Setup complete!"));
}

void loop() {
  // Check if user wants to send AT commands
  if (Serial.available()) {
    Serial.print(F("modem> "));
    // Read user input and send to modem (you'd need to implement this)
    while (Serial.available()) {
      modemSS.write(Serial.read());
    }
    delay(100);
    // Print modem response
    while (modemSS.available()) {
      Serial.write(modemSS.read());
    }
    return; // Skip the rest of the loop if user is interacting
  }

  // Network Status Check
  if (!networkConnected) {
    Serial.println(F("Checking network status..."));
    uint8_t n = modem.getNetworkStatus();
    Serial.print(F("Network status "));
    Serial.print(n);
    Serial.print(F(": "));
    
    if (n == 0) Serial.println(F("Not registered"));
    if (n == 1) Serial.println(F("Registered (home)"));
    if (n == 2) Serial.println(F("Not registered (searching)"));
    if (n == 3) Serial.println(F("Denied"));
    if (n == 4) Serial.println(F("Unknown"));
    if (n == 5) Serial.println(F("Registered roaming"));
    
    if (n == 1 || n == 5) {
      networkConnected = true;
      Serial.println(F("Network connected!"));
    } else {
      delay(5000); // Wait 5 seconds before checking again
      return;
    }
  }

  // Enable GPRS
  if (networkConnected && !gprsEnabled) {
    Serial.println(F("Enabling GPRS..."));
    if (modem.enableGPRS(true)) {
      gprsEnabled = true;
      Serial.println(F("GPRS enabled!"));
    } else {
      Serial.println(F("Failed to enable GPRS"));
    }
    delay(3000);
  }

  // Enable GPS
  if (!gpsEnabled) {
    Serial.println(F("Enabling GPS..."));
    if (modem.enableGPS(true)) {
      gpsEnabled = true;
      Serial.println(F("GPS enabled!"));
    } else {
      Serial.println(F("Failed to enable GPS"));
    }
    delay(3000);
  }

  // GPS Status Check
  if (gpsEnabled) {
    int8_t stat = modem.GPSstatus();
    Serial.print(F("GPS status: "));
    if (stat < 0) Serial.println(F("Failed to query"));
    if (stat == 0) Serial.println(F("GPS off"));
    if (stat == 1) Serial.println(F("No fix"));
    if (stat == 2) Serial.println(F("2D fix"));
    if (stat == 3) Serial.println(F("3D fix"));

    // Get GPS data if we have a fix
    if (stat >= 2) {
      Serial.println(F("Getting GPS coordinates..."));
      float latitude, longitude, speed_kph, heading, altitude;
      if (modem.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude)) {
        Serial.println(F("---------------------"));
        Serial.print(F("Latitude: ")); Serial.println(latitude, 6);
        Serial.print(F("Longitude: ")); Serial.println(longitude, 6);
        Serial.print(F("Speed: ")); Serial.println(speed_kph);
        Serial.print(F("Heading: ")); Serial.println(heading);
        Serial.print(F("Altitude: ")); Serial.println(altitude);
        Serial.println(F("---------------------"));
      } else {
        Serial.println(F("Failed to get GPS data"));
      }
    }
  }

  delay(10000); // Wait 10 seconds before next iteration
}