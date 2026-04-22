/*
  ESP32 WiFi NTRIP Client - RTK Wave Buoy (Point One Nav Polaris)
  Based on SparkFun example by Nathan Seidle, November 18th, 2021
  Modified for UCSD MAE223 Ocean Technology class

  Connects to the Point One Nav Polaris NTRIP network over WiFi and
  forwards RTCM corrections to the ZED-F9P GPS receiver via UART (Serial2).

  Polaris uses Virtual Reference Station (VRS) — the rover must send its
  NMEA GGA position back to the caster so Polaris knows where to compute
  corrections. This sketch sends GGA on connect and refreshes every 5s.

  BLE (Bluetooth Low Energy) allows wireless monitoring and configuration
  using a BLE terminal app (e.g. nRF Connect - free on iOS/Android/desktop).
  Connect to device named "RTK-Buoy" and use the Nordic UART Service (NUS).

  Button (GPIO 0) starts/stops the NTRIP client.
  LED (GPIO 13) indicates NTRIP running status.

  Hardware Connections:
  - ZED-F9P UART RX -> ESP32 GPIO 12 (TX_GPS)
  - ZED-F9P UART TX -> ESP32 GPIO 27 (RX_GPS)
  - Button on GPIO 0 (built-in on SparkFun Thing Plus)
  - LED on GPIO 13 (built-in)

  BLE Commands:
  STATUS              - system status summary
  GPS                 - current position and RTK fix info
  RESET               - restart NTRIP connection
  SLEEP               - enter light sleep (button to wake)
  GET HOST|PORT|MOUNT|USER  - read current NTRIP config value
  SET HOST <value>    - set NTRIP caster hostname or IP
  SET PORT <value>    - set NTRIP caster port
  SET MOUNT <value>   - set NTRIP mount point
  SET USER <value>    - set NTRIP username
  SET PASS <value>    - set NTRIP password (case sensitive)

  Config changes are saved to flash and persist across reboots.
  Defaults fall back to values in secrets.h if never set.
*/

#include <WiFi.h>
#include "secrets.h"
#include <Preferences.h>

// UART to ZED-F9P
#include <HardwareSerial.h>
HardwareSerial gpsSerial(2);
#define TX_GPS 12
#define RX_GPS 27

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
SFE_UBLOX_GNSS myGNSS;

// Base64 encoding for NTRIP auth
#if defined(ARDUINO_ARCH_ESP32)
#include "base64.h"
#else
#include <Base64.h>
#endif

// BLE - Nordic UART Service (NUS), compatible with nRF Connect
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_mac.h"

#define BLE_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_CHAR_RX_UUID  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // phone -> buoy
#define BLE_CHAR_TX_UUID  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // buoy -> phone

BLEServer* pServer = NULL;
BLECharacteristic* pTxChar = NULL;
bool bleConnected = false;
char bleRxBuf[256] = {0};
volatile bool bleDataReady = false;

// ============================================================
// Runtime NTRIP config - loaded from NVS, falls back to secrets.h
// ============================================================
Preferences prefs;
char ntripHost[64];
uint16_t ntripPort;
char ntripMount[32];
char ntripUser[64];
char ntripPass[64];

// ============================================================
// Button / LED
// ============================================================
const int BUTTON_PIN = 0;
const int LED_PIN = 13;
bool lastButtonState = HIGH;
bool currentButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
bool ntripRunning = false;

// ============================================================
// Timing
// ============================================================
long lastReceivedRTCM_ms = 0;
int maxTimeBeforeHangup_ms = 100000;
unsigned long lastGPSBroadcast_ms = 0;
const unsigned long gpsBroadcastInterval_ms = 5000;  // GPS update over BLE every 5s
unsigned long lastGGASent_ms = 0;
const unsigned long ggaInterval_ms = 300000;         // Resend GGA every 5 min — buoy drift is slow relative to VRS correction scale (~km)

// ============================================================
// BLE output helper - sends to Serial and BLE simultaneously
// Chunks messages into 20-byte BLE packets automatically
// ============================================================
void buoyPrint(const String& msg) {
  Serial.print(msg);
  if (bleConnected && pTxChar != NULL) {
    int len = msg.length();
    int offset = 0;
    while (offset < len) {
      int chunkSize = min(20, len - offset);
      String chunk = msg.substring(offset, offset + chunkSize);
      pTxChar->setValue(chunk.c_str());
      pTxChar->notify();
      offset += chunkSize;
      delay(10);
    }
  }
}

void buoyPrintln(const String& msg) {
  buoyPrint(msg + "\n");
}

// ============================================================
// Build NMEA GGA sentence from current ZED-F9P position.
// Sent to the Polaris caster so VRS knows where to compute corrections.
// Must be sent on connect and refreshed every ~5s.
// ============================================================
String buildGGA() {
  double lat = myGNSS.getLatitude()    / 10000000.0;
  double lon = myGNSS.getLongitude()   / 10000000.0;
  double alt = myGNSS.getAltitudeMSL() / 1000.0;
  uint8_t fix     = myGNSS.getFixType();
  uint8_t siv     = myGNSS.getSIV();
  uint8_t carrier = myGNSS.getCarrierSolutionType();
  uint8_t h = myGNSS.getHour();
  uint8_t m = myGNSS.getMinute();
  uint8_t s = myGNSS.getSecond();

  // GGA quality: 0=invalid, 1=GPS, 4=RTK Fixed, 5=RTK Float
  int quality = 0;
  if (fix >= 2) {
    if      (carrier == 2) quality = 4;
    else if (carrier == 1) quality = 5;
    else                   quality = 1;
  }

  char latDir = (lat >= 0) ? 'N' : 'S';
  double absLat = fabs(lat);
  int latDeg    = (int)absLat;
  double latMin = (absLat - latDeg) * 60.0;

  char lonDir = (lon >= 0) ? 'E' : 'W';
  double absLon = fabs(lon);
  int lonDeg    = (int)absLon;
  double lonMin = (absLon - lonDeg) * 60.0;

  char body[128];
  snprintf(body, sizeof(body),
    "GPGGA,%02d%02d%02d.00,%02d%07.4f,%c,%03d%07.4f,%c,%d,%02d,1.0,%.2f,M,0.0,M,,",
    h, m, s, latDeg, latMin, latDir, lonDeg, lonMin, lonDir, quality, siv, alt);

  uint8_t checksum = 0;
  for (int i = 0; body[i] != '\0'; i++) checksum ^= (uint8_t)body[i];

  char sentence[140];
  snprintf(sentence, sizeof(sentence), "$%s*%02X\r\n", body, checksum);
  return String(sentence);
}

// ============================================================
// GPS broadcast - sent over BLE periodically while NTRIP running
// ============================================================
void broadcastGPS() {
  if (!bleConnected) return;
  float lat     = myGNSS.getLatitude()        / 10000000.0;
  float lon     = myGNSS.getLongitude()       / 10000000.0;
  float alt     = myGNSS.getAltitudeMSL()     / 1000.0;
  float hAcc    = myGNSS.getHorizontalAccEst()/ 1000.0;
  uint8_t carrier = myGNSS.getCarrierSolutionType();
  String rtk = (carrier == 2) ? "FIX" : (carrier == 1) ? "FLT" : "NON";
  buoyPrintln("GPS " + String(lat, 7) + " " + String(lon, 7) +
              " alt:" + String(alt, 1) + "m RTK:" + rtk +
              " hAcc:" + String(hAcc, 3) + "m");
}

// ============================================================
// BLE command handler
// ============================================================
void handleBLECommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  // Uppercase copy for keyword matching; preserve case for values (passwords, hostnames)
  String cmdUpper = cmd;
  cmdUpper.toUpperCase();

  // --- STATUS ---
  if (cmdUpper == "STATUS") {
    buoyPrintln("=== STATUS ===");
    buoyPrintln("WiFi: " + String(WiFi.status() == WL_CONNECTED ? "connected" : "disconnected"));
    buoyPrintln("IP: " + WiFi.localIP().toString());
    buoyPrintln("NTRIP: " + String(ntripRunning ? "running" : "stopped"));
    buoyPrintln("Host: " + String(ntripHost) + ":" + String(ntripPort));
    buoyPrintln("Mount: " + String(ntripMount));
    buoyPrintln("==============");

  // --- GPS ---
  } else if (cmdUpper == "GPS") {
    buoyPrintln("=== GPS ===");
    float lat    = myGNSS.getLatitude()         / 10000000.0;
    float lon    = myGNSS.getLongitude()        / 10000000.0;
    float alt    = myGNSS.getAltitudeMSL()      / 1000.0;
    float hAcc   = myGNSS.getHorizontalAccEst() / 1000.0;
    uint8_t fix      = myGNSS.getFixType();
    uint8_t carrier  = myGNSS.getCarrierSolutionType();
    uint8_t siv      = myGNSS.getSIV();
    String rtk = (carrier == 2) ? "Fixed" : (carrier == 1) ? "Float" : "None";
    buoyPrintln("Lat:  " + String(lat, 7));
    buoyPrintln("Lon:  " + String(lon, 7));
    buoyPrintln("Alt:  " + String(alt, 2) + " m");
    buoyPrintln("Fix:  " + String(fix));
    buoyPrintln("RTK:  " + rtk);
    buoyPrintln("hAcc: " + String(hAcc, 3) + " m");
    buoyPrintln("SIV:  " + String(siv));
    buoyPrintln("===========");

  // --- RESET ---
  } else if (cmdUpper == "RESET") {
    buoyPrintln("Resetting NTRIP connection...");
    ntripRunning = false;

  // --- SLEEP ---
  } else if (cmdUpper == "SLEEP") {
    buoyPrintln("Entering light sleep. Press button to wake.");
    ntripRunning = false;
    digitalWrite(LED_PIN, LOW);
    delay(500);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);
    esp_light_sleep_start();
    buoyPrintln("Woke from sleep.");

  // --- GET ---
  } else if (cmdUpper.startsWith("GET ")) {
    String param = cmdUpper.substring(4);
    param.trim();
    if      (param == "HOST")  buoyPrintln("HOST: "  + String(ntripHost));
    else if (param == "PORT")  buoyPrintln("PORT: "  + String(ntripPort));
    else if (param == "MOUNT") buoyPrintln("MOUNT: " + String(ntripMount));
    else if (param == "USER")  buoyPrintln("USER: "  + String(ntripUser));
    else buoyPrintln("Unknown param. Options: HOST PORT MOUNT USER");

  // --- SET ---
  } else if (cmdUpper.startsWith("SET ")) {
    String rest      = cmd.substring(4);   // preserve original case for value
    String restUpper = rest;
    restUpper.toUpperCase();

    if (restUpper.startsWith("HOST ")) {
      String val = rest.substring(5); val.trim();
      val.toCharArray(ntripHost, sizeof(ntripHost));
      prefs.putString("ntripHost", val);
      buoyPrintln("HOST set: " + val);

    } else if (restUpper.startsWith("PORT ")) {
      String val = rest.substring(5); val.trim();
      ntripPort = (uint16_t)val.toInt();
      prefs.putUShort("ntripPort", ntripPort);
      buoyPrintln("PORT set: " + val);

    } else if (restUpper.startsWith("MOUNT ")) {
      String val = rest.substring(6); val.trim();
      val.toCharArray(ntripMount, sizeof(ntripMount));
      prefs.putString("ntripMount", val);
      buoyPrintln("MOUNT set: " + val);

    } else if (restUpper.startsWith("USER ")) {
      String val = rest.substring(5); val.trim();
      val.toCharArray(ntripUser, sizeof(ntripUser));
      prefs.putString("ntripUser", val);
      buoyPrintln("USER set: " + val);

    } else if (restUpper.startsWith("PASS ")) {
      String val = rest.substring(5); val.trim();  // keep original case
      val.toCharArray(ntripPass, sizeof(ntripPass));
      prefs.putString("ntripPass", val);
      buoyPrintln("PASS updated.");

    } else {
      buoyPrintln("Unknown SET param. Options: HOST PORT MOUNT USER PASS");
    }

  } else {
    buoyPrintln("Unknown command: " + cmd);
    buoyPrintln("Commands: STATUS GPS RESET SLEEP GET SET");
  }
}

// ============================================================
// BLE server callbacks
// ============================================================
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    bleConnected = true;
    buoyPrintln("BLE connected. Commands: STATUS GPS RESET SLEEP GET SET");
  }
  void onDisconnect(BLEServer* pServer) {
    bleConnected = false;
    Serial.println(F("BLE disconnected - restarting advertising"));
    pServer->startAdvertising();
  }
};

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) {
    strncpy(bleRxBuf, pChar->getValue().c_str(), sizeof(bleRxBuf) - 1);
    bleRxBuf[sizeof(bleRxBuf) - 1] = '\0';
    bleDataReady = true;
  }
};

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println(F("RTK Wave Buoy - WiFi NTRIP Client"));

  // Button and LED
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Load NTRIP config from NVS, fall back to secrets.h defaults
  prefs.begin("ntrip", false);
  prefs.getString("ntripHost",  ntripHost,  sizeof(ntripHost));
  if (strlen(ntripHost)  == 0) strncpy(ntripHost,  casterHost,  sizeof(ntripHost));
  ntripPort = prefs.getUShort("ntripPort", casterPort);
  prefs.getString("ntripMount", ntripMount, sizeof(ntripMount));
  if (strlen(ntripMount) == 0) strncpy(ntripMount, mountPoint,  sizeof(ntripMount));
  prefs.getString("ntripUser",  ntripUser,  sizeof(ntripUser));
  if (strlen(ntripUser)  == 0) strncpy(ntripUser,  casterUser,  sizeof(ntripUser));
  prefs.getString("ntripPass",  ntripPass,  sizeof(ntripPass));
  if (strlen(ntripPass)  == 0) strncpy(ntripPass,  casterUserPW,sizeof(ntripPass));

  // BLE setup - unique name using last 2 bytes of MAC address
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_BT);
  char bleName[20];
  snprintf(bleName, sizeof(bleName), "RTK-Buoy-%02X%02X", mac[4], mac[5]);

  BLEDevice::init(bleName);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  BLEService* pService = pServer->createService(BLE_SERVICE_UUID);

  // TX characteristic: buoy -> phone (notify)
  pTxChar = pService->createCharacteristic(BLE_CHAR_TX_UUID,
              BLECharacteristic::PROPERTY_NOTIFY);
  pTxChar->addDescriptor(new BLE2902());

  // RX characteristic: phone -> buoy (write)
  BLECharacteristic* pRxChar = pService->createCharacteristic(BLE_CHAR_RX_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pRxChar->setCallbacks(new RxCallbacks());

  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("BLE advertising as: " + String(bleName));

  // ZED-F9P over UART
  gpsSerial.begin(115200, SERIAL_8N1, RX_GPS, TX_GPS);
  Serial.println(F("Connecting to ZED-F9P over UART..."));
  int gpsAttempts = 0;
  while (!myGNSS.begin(gpsSerial) && gpsAttempts < 5) {
    Serial.println(F("ZED-F9P not detected, retrying..."));
    delay(1000);
    gpsAttempts++;
  }
  if (gpsAttempts >= 5) {
    Serial.println(F("ERROR: ZED-F9P not found. Check wiring."));
  } else {
    Serial.println(F("ZED-F9P connected!"));
    // Use default 1Hz nav rate — 10Hz starves the RTK engine and prevents fix
    // Don't call setPortInput — ZED-F9P accepts RTCM3 on all ports by default
  }

  // WiFi
  Serial.print(F("Connecting to WiFi"));
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println();
  buoyPrintln("WiFi connected: " + WiFi.localIP().toString());
  buoyPrintln("Press button to start NTRIP. BLE: " + String(bleName));
}

// ============================================================
// Main loop
// ============================================================
void loop() {
  // Process incoming BLE commands
  if (bleDataReady) {
    bleDataReady = false;
    String cmd = String(bleRxBuf);
    handleBLECommand(cmd);
  }

  // Periodic GPS broadcast over BLE
  if (bleConnected && (millis() - lastGPSBroadcast_ms > gpsBroadcastInterval_ms)) {
    lastGPSBroadcast_ms = millis();
    broadcastGPS();
  }

  // Button debounce
  int reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonState) lastDebounceTime = millis();
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != currentButtonState) {
      currentButtonState = reading;
      if (currentButtonState == LOW) {
        if (!ntripRunning) {
          buoyPrintln("Button - Starting NTRIP");
          ntripRunning = true;
          digitalWrite(LED_PIN, HIGH);
          beginClient();
        } else {
          buoyPrintln("Button - Stopping NTRIP");
          ntripRunning = false;
          digitalWrite(LED_PIN, LOW);
        }
      }
    }
  }
  lastButtonState = reading;

  if (!ntripRunning) {
    Serial.println(F("Press button to start NTRIP. LED OFF = stopped"));
    delay(2000);
  }
}

// ============================================================
// NTRIP client - connect, receive RTCM, push to ZED-F9P
// BLE commands and GPS broadcasts are handled inside this loop
// ============================================================
void beginClient() {
  WiFiClient ntripClient;
  long rtcmCount = 0;

  buoyPrintln("Connecting to NTRIP caster...");

  while (ntripRunning) {

    // --- BLE command processing ---
    if (bleDataReady) {
      bleDataReady = false;
      String cmd = String(bleRxBuf);
      handleBLECommand(cmd);
      if (!ntripRunning) break;  // RESET command sets ntripRunning = false
    }

    // --- Periodic GPS broadcast ---
    if (bleConnected && (millis() - lastGPSBroadcast_ms > gpsBroadcastInterval_ms)) {
      lastGPSBroadcast_ms = millis();
      broadcastGPS();
    }

    // --- Button check ---
    int reading = digitalRead(BUTTON_PIN);
    if (reading != lastButtonState) lastDebounceTime = millis();
    if ((millis() - lastDebounceTime) > debounceDelay) {
      if (reading != currentButtonState) {
        currentButtonState = reading;
        if (currentButtonState == LOW) {
          buoyPrintln("Button - stopping NTRIP");
          ntripRunning = false;
          digitalWrite(LED_PIN, LOW);
          break;
        }
      }
    }
    lastButtonState = reading;

    // --- Connect to caster if not already connected ---
    if (!ntripClient.connected()) {
      if (!ntripClient.connect(ntripHost, ntripPort)) {
        buoyPrintln("Connection to caster failed");
        ntripRunning = false;
        digitalWrite(LED_PIN, LOW);
        return;
      }

      buoyPrintln("Connected to " + String(ntripHost) + ":" + String(ntripPort));
      buoyPrintln("Requesting mount: " + String(ntripMount));

      // Build HTTP request
      const int SERVER_BUFFER_SIZE = 512;
      char serverRequest[SERVER_BUFFER_SIZE];
      snprintf(serverRequest, SERVER_BUFFER_SIZE,
        "GET /%s HTTP/1.0\r\nUser-Agent: NTRIP SparkFun u-blox Client v1.0\r\n", ntripMount);

      char credentials[512];
      if (strlen(ntripUser) == 0) {
        strncpy(credentials, "Accept: */*\r\nConnection: close\r\n", sizeof(credentials));
      } else {
        char userCredentials[128];
        snprintf(userCredentials, sizeof(userCredentials), "%s:%s", ntripUser, ntripPass);
#if defined(ARDUINO_ARCH_ESP32)
        base64 b;
        String strEncoded = b.encode(userCredentials);
        char encodedCredentials[strEncoded.length() + 1];
        strEncoded.toCharArray(encodedCredentials, sizeof(encodedCredentials));
        snprintf(credentials, sizeof(credentials), "Authorization: Basic %s\r\n", encodedCredentials);
#else
        int encodedLen = base64_enc_len(strlen(userCredentials));
        char encodedCredentials[encodedLen];
        base64_encode(encodedCredentials, userCredentials, strlen(userCredentials));
#endif
      }
      strncat(serverRequest, credentials, SERVER_BUFFER_SIZE);
      strncat(serverRequest, "\r\n", SERVER_BUFFER_SIZE);
      ntripClient.write(serverRequest, strlen(serverRequest));

      // Wait for response
      unsigned long timeout = millis();
      while (ntripClient.available() == 0) {
        if (millis() - timeout > 5000) {
          buoyPrintln("Caster timed out!");
          ntripClient.stop();
          ntripRunning = false;
          digitalWrite(LED_PIN, LOW);
          return;
        }
        delay(10);
      }

      // Check response
      bool connectionSuccess = false;
      char response[512];
      int responseSpot = 0;
      while (ntripClient.available()) {
        if (responseSpot == sizeof(response) - 1) break;
        response[responseSpot++] = ntripClient.read();
        if (strstr(response, "200") > (char *)0) connectionSuccess = true;
        if (strstr(response, "401") > (char *)0) {
          buoyPrintln("Bad credentials! Check user/pass.");
          connectionSuccess = false;
        }
      }
      response[responseSpot] = '\0';
      Serial.print(F("Caster: ")); Serial.println(response);

      if (!connectionSuccess) {
        buoyPrintln("NTRIP connection failed.");
        ntripRunning = false;
        digitalWrite(LED_PIN, LOW);
        return;
      }
      buoyPrintln("NTRIP streaming RTCM...");
      lastReceivedRTCM_ms = millis();

      // Send initial GGA so Polaris VRS knows our position
      String gga = buildGGA();
      ntripClient.print(gga);
      lastGGASent_ms = millis();
      Serial.print(F("Sent GGA: ")); Serial.print(gga);
    } // end connect

    // --- Refresh GGA every 5s to keep Polaris VRS position current ---
    if (ntripClient.connected() && (millis() - lastGGASent_ms > ggaInterval_ms)) {
      String gga = buildGGA();
      ntripClient.print(gga);
      lastGGASent_ms = millis();
    }

    // --- Read RTCM and write directly to ZED-F9P serial ---
    if (ntripClient.connected()) {
      uint8_t rtcmData[512];
      rtcmCount = 0;
      while (ntripClient.available()) {
        int bytesToRead = min((int)ntripClient.available(), (int)sizeof(rtcmData));
        int bytesRead = ntripClient.read(rtcmData, bytesToRead);
        if (bytesRead > 0) {
          gpsSerial.write(rtcmData, bytesRead);
          rtcmCount += bytesRead;
        }
      }
      if (rtcmCount > 0) {
        lastReceivedRTCM_ms = millis();
        Serial.print(F("RTCM -> ZED: ")); Serial.println(rtcmCount);
      }
    }

    // --- RTCM timeout ---
    if (millis() - lastReceivedRTCM_ms > maxTimeBeforeHangup_ms) {
      buoyPrintln("RTCM timeout. Disconnecting...");
      if (ntripClient.connected()) ntripClient.stop();
      ntripRunning = false;
      digitalWrite(LED_PIN, LOW);
      return;
    }

    delay(10);
  } // end while ntripRunning

  buoyPrintln("NTRIP stopped.");
  ntripClient.stop();
  ntripRunning = false;
  digitalWrite(LED_PIN, LOW);
}
