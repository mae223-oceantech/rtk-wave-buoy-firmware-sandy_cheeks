/*
  ESP32 WiFi NTRIP Client — RTK Wave Buoy (Point One Nav Polaris)
  Based on SparkFun example by Nathan Seidle
  Modified for UCSD MAE223 Ocean Technology class

  Merges the best of esp32_polaris.ino and esp32_rtk_mae.ino:
    - Correct ZED-F9P init: no nav-rate override (stays at 1 Hz default),
      no setPortInput (ZED accepts RTCM3 on all ports by default)
    - Direct gpsSerial.write() for RTCM — bypasses SparkFun library overhead
    - HTTP/1.1 with Ntrip-Version: Ntrip/2.0 + chunked transfer decoder
      required for Polaris responses
    - GGA sent on connect, refreshed every 5 min — appropriate for a
      slow-moving buoy (drift is small relative to VRS correction scale)
    - Reconnect counter + session timer for field diagnostics
    - No BLE

  Hardware:
    SparkFun ESP32 Thing Plus + u-blox ZED-F9P

  Wiring:
    ESP32 GPIO 27 (RX2) → ZED-F9P TX2
    ESP32 GPIO 12 (TX2) → ZED-F9P RX2
    ESP32 GND → ZED-F9P GND

  Button (GPIO 0): press to start NTRIP, press again to stop
  LED (GPIO 13):   blinking = WiFi connected, ready
                   solid on = NTRIP running

  Before flashing, create a secrets.h tab in this sketch folder:
    const char ssid[]           = "<wifi name>";
    const char password[]       = "<wifi password>";
    const char casterHost[]     = "polaris.pointonenav.com";
    const uint16_t casterPort   = 2101;
    const char casterUser[]     = "<your email>";
    const char casterUserPW[]   = "<your api key>";
    const char mountPoint[]     = "POLARIS";
*/

#include <WiFi.h>
#include "/Users/Alexander/Repositories/rtk-wave-buoy-firmware-sandy_cheeks/esp32/secrets.h"
#include <HardwareSerial.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <SPI.h>
#include <SD.h>

#if defined(ARDUINO_ARCH_ESP32)
#include "base64.h"
#else
#include <Base64.h>
#endif

HardwareSerial gpsSerial(2);
#define RX_GPS 17
#define TX_GPS 16

SFE_UBLOX_GNSS myGNSS;

// Button / LED
const int BUTTON_PIN = 0;
const int LED_PIN    = 13;
bool lastButtonState    = HIGH;
bool currentButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
bool ntripRunning  = false;
bool ledBlinkState = false;
unsigned long lastBlinkTime = 0;

// RTCM / GGA timing
long lastReceivedRTCM_ms           = 0;
const int maxTimeBeforeHangup_ms   = 100000;
unsigned long lastGGASent_ms       = 0;
const unsigned long ggaInterval_ms = 300000; // 5 min — buoy drift slow vs VRS scale

// Session diagnostics
int reconnectCount        = 0;
unsigned long sessionStart_ms = 0;

// SD card logging (SparkFun ESP32 Thing Plus: onboard microSD on GPIO 5)
const int SD_CS_PIN = 5;
File gpsLogFile;
bool sdAvailable = false;
unsigned long lastFlushTime = 0;
const unsigned long flushInterval_ms = 1000;

// Drain ZED-F9P UART so the hardware buffer doesn't overflow.
// Logging is OFF — set LOG_GPS=true to write NMEA/UBX to SD (or Serial as fallback).
void drainGpsSerial() {
  const bool LOG_GPS = false;
  bool wrote = false;
  while (gpsSerial.available()) {
    uint8_t b = gpsSerial.read();
    if (LOG_GPS) {
      if (sdAvailable) {
        gpsLogFile.write(b);
        wrote = true;
      } else {
        Serial.write(b);
      }
    }
  }
  if (sdAvailable && wrote && (millis() - lastFlushTime > flushInterval_ms)) {
    gpsLogFile.flush();
    lastFlushTime = millis();
  }
}

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println(F("RTK Wave Buoy — WiFi/Polaris NTRIP Client"));

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize SD card for GPS data logging
  if (SD.begin(SD_CS_PIN)) {
    gpsLogFile = SD.open("/gps.log", FILE_APPEND);
    if (gpsLogFile) {
      sdAvailable = true;
      Serial.println(F("SD: logging GPS data to /gps.log"));
      gpsLogFile.println();
      gpsLogFile.print(F("=== Session start, millis="));
      gpsLogFile.print(millis());
      gpsLogFile.println(F(" ==="));
    } else {
      Serial.println(F("SD: failed to open /gps.log — GPS will go to Serial"));
    }
  } else {
    Serial.println(F("SD: init FAILED (no card?) — GPS will go to Serial"));
  }

  gpsSerial.begin(38400, SERIAL_8N1, RX_GPS, TX_GPS);
  Serial.println(F("Connecting to ZED-F9P..."));
  int attempts = 0;
  while (!myGNSS.begin(gpsSerial) && attempts < 5) {
    Serial.println(F("ZED-F9P not detected, retrying..."));
    delay(1000);
    attempts++;
  }
  if (attempts >= 5) {
    Serial.println(F("ERROR: ZED-F9P not found. Check wiring."));
  } else {
    Serial.println(F("ZED-F9P connected."));
    // No setPortInput — ZED-F9P accepts RTCM3 on all UARTs by default.
    // No setNavigationFrequency — 1 Hz default keeps the RTK engine fed.
  }
WiFi.mode(WIFI_STA);
WiFi.disconnect(true, true);
delay(1000);

Serial.println("Scanning...");
int n = WiFi.scanNetworks();

for (int i=0; i<n; i++) {
  Serial.print(i);
  Serial.print(": ");
  Serial.print(WiFi.SSID(i));
  Serial.print(" RSSI=");
  Serial.print(WiFi.RSSI(i));
  Serial.print(" CH=");
  Serial.print(WiFi.channel(i));
  Serial.print(" ENC=");
  Serial.println(WiFi.encryptionType(i));
}

Serial.println("Connecting...");
WiFi.begin(ssid, password);

int tries = 0;
while (WiFi.status() != WL_CONNECTED && tries < 30) {
  Serial.print(".");
  Serial.print(WiFi.status());
  delay(500);
  tries++;
}

Serial.println();
Serial.print("Final status=");
Serial.println(WiFi.status());
Serial.print("Local IP: ");
Serial.println(WiFi.localIP());
Serial.print("DNS server: ");
Serial.println(WiFi.dnsIP());

// DNS sanity check — resolves the caster host name independently of the TCP connect
IPAddress casterIP;
bool dnsOk = WiFi.hostByName(casterHost, casterIP);
if (dnsOk && casterIP != IPAddress(0, 0, 0, 0)) {
  Serial.print(F("DNS OK: "));
  Serial.print(casterHost);
  Serial.print(F(" -> "));
  Serial.println(casterIP);
} else {
  Serial.print(F("DNS FAILED for "));
  Serial.print(casterHost);
  Serial.print(F(" (returned "));
  Serial.print(casterIP);
  Serial.println(F(") — not connected to a working network"));
}
  sessionStart_ms = millis();
  

  Serial.println(F("Press GPIO 0 button to start/stop NTRIP. Blinking LED = ready."));
}

// ============================================================
// Main loop
// ============================================================
void loop() {
  drainGpsSerial();

  int reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonState) lastDebounceTime = millis();

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != currentButtonState) {
      currentButtonState = reading;
      if (currentButtonState == LOW) {
        if (!ntripRunning) {
          Serial.println(F("Button — starting NTRIP"));
          ntripRunning = true;
          digitalWrite(LED_PIN, HIGH);
          beginClient();
        } else {
          Serial.println(F("Button — stopping NTRIP"));
          ntripRunning = false;
          digitalWrite(LED_PIN, LOW);
        }
      }
    }
  }
  lastButtonState = reading;

  // Heartbeat blink while waiting — indicates WiFi connected and ready
  if (!ntripRunning) {
    if (millis() - lastBlinkTime >= 1000) {
      lastBlinkTime = millis();
      ledBlinkState = !ledBlinkState;
      digitalWrite(LED_PIN, ledBlinkState);
    }
  }
}

// ============================================================
// Build NMEA GGA from current ZED-F9P position.
// Polaris VRS uses this to place the virtual base station near the buoy.
// quality=0 if no fix yet — Polaris will wait until a valid position arrives.
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

  // GGA quality indicator: 0=no fix, 1=GPS, 4=RTK Fixed, 5=RTK Float
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
  for (int i = 0; body[i]; i++) checksum ^= (uint8_t)body[i];

  char sentence[140];
  snprintf(sentence, sizeof(sentence), "$%s*%02X\r\n", body, checksum);
  return String(sentence);
}

// ============================================================
// NTRIP client — connect to Polaris, receive RTCM, push to ZED-F9P
// ============================================================
void beginClient() {
  WiFiClient ntripClient;
  long rtcmCount = 0;
  unsigned long rtcmBytesTotal       = 0;
  unsigned long rtcmBytesSinceStatus = 0;
  unsigned long lastStatusPrint_ms   = 0;
  const unsigned long statusInterval_ms = 3000;

  Serial.println(F("Subscribing to Polaris caster..."));

  while (ntripRunning) {
    drainGpsSerial();

    // Button check inside NTRIP loop
    int reading = digitalRead(BUTTON_PIN);
    if (reading != lastButtonState) lastDebounceTime = millis();
    if ((millis() - lastDebounceTime) > debounceDelay) {
      if (reading != currentButtonState) {
        currentButtonState = reading;
        if (currentButtonState == LOW) {
          Serial.println(F("Button — stopping NTRIP"));
          ntripRunning = false;
          digitalWrite(LED_PIN, LOW);
          break;
        }
      }
    }
    lastButtonState = reading;

    // Connect (or reconnect) to caster
    if (!ntripClient.connected()) {
      reconnectCount++;
      float elapsedMin = (millis() - sessionStart_ms) / 60000.0;
      Serial.print(F("Connect attempt #")); Serial.print(reconnectCount);
      Serial.print(F(" at ")); Serial.print(elapsedMin, 1); Serial.println(F(" min"));
      Serial.print(F("Opening socket to ")); Serial.println(casterHost);

      // Resolve at attempt time — distinguishes DNS failure from TCP failure
      IPAddress casterIP;
      if (!WiFi.hostByName(casterHost, casterIP)) {
        Serial.println(F("  DNS lookup FAILED — caster host did not resolve"));
      } else {
        Serial.print(F("  DNS -> ")); Serial.println(casterIP);
      }

      if (!ntripClient.connect(casterHost, casterPort)) {
        Serial.println(F("Connection to caster failed"));
        ntripRunning = false;
        digitalWrite(LED_PIN, LOW);
        return;
      }

      Serial.print(F("Connected to ")); Serial.print(casterHost);
      Serial.print(F(":")); Serial.println(casterPort);

      // HTTP/1.1 with Ntrip-Version: Ntrip/2.0 — Polaris uses chunked transfer encoding
      const int SERVER_BUFFER_SIZE = 512;
      char serverRequest[SERVER_BUFFER_SIZE];
      snprintf(serverRequest, SERVER_BUFFER_SIZE,
        "GET /%s HTTP/1.1\r\n"
        "Host: %s\r\n"
        "Ntrip-Version: Ntrip/2.0\r\n"
        "User-Agent: NTRIP SparkFun u-blox Client v1.0\r\n",
        mountPoint, casterHost);

      char credentials[512];
      if (strlen(casterUser) == 0) {
        strncpy(credentials, "Accept: */*\r\nConnection: close\r\n", sizeof(credentials));
      } else {
        char userCredentials[128];
        snprintf(userCredentials, sizeof(userCredentials), "%s:%s", casterUser, casterUserPW);
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
      strncat(serverRequest, credentials, SERVER_BUFFER_SIZE - strlen(serverRequest) - 1);
      strncat(serverRequest, "\r\n",     SERVER_BUFFER_SIZE - strlen(serverRequest) - 1);

      Serial.print(F("Request size: ")); Serial.print(strlen(serverRequest));
      Serial.print(F(" / ")); Serial.print(SERVER_BUFFER_SIZE); Serial.println(F(" bytes"));
      ntripClient.write(serverRequest, strlen(serverRequest));

      // Wait for HTTP response
      unsigned long timeout = millis();
      while (ntripClient.available() == 0) {
        if (millis() - timeout > 5000) {
          Serial.println(F("Caster timed out"));
          ntripClient.stop();
          ntripRunning = false;
          digitalWrite(LED_PIN, LOW);
          return;
        }
        delay(10);
      }

      // Parse HTTP response — look for 200 OK
      bool connectionSuccess = false;
      char response[512];
      int responseSpot = 0;
      while (ntripClient.available()) {
        if (responseSpot == sizeof(response) - 1) break;
        response[responseSpot++] = ntripClient.read();
        if (strstr(response, "200") > (char *)0) connectionSuccess = true;
        if (strstr(response, "401") > (char *)0) {
          Serial.println(F("401 Unauthorized — check casterUser/casterUserPW in secrets.h"));
          connectionSuccess = false;
        }
      }
      response[responseSpot] = '\0';
      Serial.print(F("Caster response: ")); Serial.println(response);

      if (!connectionSuccess) {
        Serial.println(F("NTRIP connection failed"));
        ntripRunning = false;
        digitalWrite(LED_PIN, LOW);
        return;
      }

      lastReceivedRTCM_ms = millis();
      lastStatusPrint_ms  = millis();

      // Send initial GGA so Polaris VRS knows where to synthesize the virtual base station
      String gga = buildGGA();
      ntripClient.print(gga);
      lastGGASent_ms = millis();
      Serial.print(F("Sent GGA: ")); Serial.print(gga);
    }

    // Refresh GGA every 5 min to keep Polaris VRS position current
    if (ntripClient.connected() && millis() - lastGGASent_ms > ggaInterval_ms) {
      String gga = buildGGA();
      ntripClient.print(gga);
      lastGGASent_ms = millis();
      Serial.println(F("GGA refreshed"));
    }

    // Read chunked RTCM and write directly to ZED-F9P UART
    if (ntripClient.connected()) {
      rtcmCount = 0;
      while (ntripClient.available()) {
        // Decode HTTP/1.1 chunked transfer: read hex chunk size line
        char chunkSizeBuf[10];
        int idx = 0;
        while (ntripClient.available()) {
          char c = ntripClient.read();
          if (c == '\n') break;
          if (c != '\r') chunkSizeBuf[idx++] = c;
        }
        chunkSizeBuf[idx] = '\0';
        int chunkSize = strtol(chunkSizeBuf, NULL, 16);
        if (chunkSize == 0) break; // zero-size chunk = end of stream

        // Read chunkSize bytes of RTCM data
        uint8_t rtcmData[512 * 4];
        int bytesRead = 0;
        while (bytesRead < chunkSize && ntripClient.available()) {
          rtcmData[bytesRead++] = ntripClient.read();
          if (bytesRead == sizeof(rtcmData)) break;
        }
        // Consume trailing CRLF after chunk payload
        while (ntripClient.available()) {
          char c = ntripClient.read();
          if (c == '\n') break;
        }

        if (bytesRead > 0) {
          gpsSerial.write(rtcmData, bytesRead); // direct to ZED-F9P UART
          rtcmCount += bytesRead;
        }
        break; // one chunk per loop iteration — keeps button/GGA checks responsive
      }

      if (rtcmCount > 0) {
        lastReceivedRTCM_ms = millis();
        rtcmBytesTotal       += rtcmCount;
        rtcmBytesSinceStatus += rtcmCount;
      }
    }

    // Periodic "corrections flowing" heartbeat
    if (rtcmBytesSinceStatus > 0 && millis() - lastStatusPrint_ms >= statusInterval_ms) {
      float seconds = (millis() - lastStatusPrint_ms) / 1000.0;
      Serial.print(F("Polaris corrections OK — "));
      Serial.print((unsigned long)(rtcmBytesSinceStatus / seconds));
      Serial.print(F(" B/s, "));
      Serial.print(rtcmBytesTotal);
      Serial.println(F(" B total"));
      rtcmBytesSinceStatus = 0;
      lastStatusPrint_ms = millis();
    }

    // Disconnect if no RTCM received for 100s
    if (millis() - lastReceivedRTCM_ms > maxTimeBeforeHangup_ms) {
      Serial.println(F("RTCM timeout — disconnecting"));
      if (ntripClient.connected()) ntripClient.stop();
      ntripRunning = false;
      digitalWrite(LED_PIN, LOW);
      return;
    }

    delay(10);
  }

  Serial.println(F("NTRIP stopped"));
  ntripClient.stop();
  ntripRunning = false;
  digitalWrite(LED_PIN, LOW);
}
