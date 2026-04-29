/*
  ESP32 WiFi NTRIP Client — RTK Wave Buoy (Point One Nav Polaris)
  Based on SparkFun example by Nathan Seidle
  Modified for UCSD MAE223 Ocean Technology class

  Merges the best of esp32_polaris.ino and esp32_rtk_mae.ino:
    - ZED-F9P init: nav rate set to 5 Hz (captures wave motion ~5–20 s
      periods, well under the F9P's RTK ceiling), no setPortInput
      (ZED accepts RTCM3 on all UARTs by default)
    - Direct gpsSerial.write() for RTCM — bypasses SparkFun library overhead
    - HTTP/1.1 with Ntrip-Version: Ntrip/2.0 + chunked transfer decoder
      required for Polaris responses
    - GGA sent on connect, refreshed every 10 s — Polaris VRS expects
      regular GGA so the synthesized base stays near the rover
    - Reconnect counter + session timer for field diagnostics
    - No BLE

  Hardware:
    SparkFun ESP32 Thing Plus + u-blox ZED-F9P

  Wiring (ESP32 UART2 ↔ F9P UART1 — the pads labeled TX1/MISO and
  RX1/MOSI on the SparkFun breakout are F9P UART1; MISO/MOSI are the
  shared SPI names on the same pads):
    ESP32 GPIO 27 (RX2) → ZED-F9P TX1/MISO   (F9P UART1)
    ESP32 GPIO 12 (TX2) → ZED-F9P RX1/MOSI   (F9P UART1)
    ESP32 GND           → ZED-F9P GND
  Verified by u-center MON-COMMS: RTCM3 arrives on F9P UART1, not UART2.

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
#include "secrets.h"
#include <HardwareSerial.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

#if defined(ARDUINO_ARCH_ESP32)
#include "base64.h"
#else
#include <Base64.h>
#endif

HardwareSerial gpsSerial(2);
#define RX_GPS 27
#define TX_GPS 12

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
// Polaris VRS expects GGA every 10–30 s so the synthesized base stays near the rover;
// 5 min was too sparse and let the VRS solution drift relative to actual buoy position.
const unsigned long ggaInterval_ms = 10000;

// Session diagnostics
int reconnectCount        = 0;
unsigned long sessionStart_ms = 0;
unsigned long rtcmTotalBytes  = 0;
int desyncSuspectCount    = 0;
int trailingCRLFMismatch  = 0;

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println(F("RTK Wave Buoy — WiFi/Polaris NTRIP Client"));

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  gpsSerial.begin(115200, SERIAL_8N1, RX_GPS, TX_GPS);
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
    // 5 Hz captures wave motion (periods ~5–20 s) while staying well under
    // the F9P's RTK nav-rate ceiling.
    myGNSS.setNavigationFrequency(5);
  }

  Serial.print(F("Connecting to WiFi"));
  WiFi.begin(ssid, password);
  unsigned long wifiStart_ms = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - wifiStart_ms > 30000) {
      Serial.println();
      Serial.println(F("ERROR: WiFi connect timed out after 30 s."));
      Serial.println(F("Check ssid/password in secrets.h, or that the AP is in range."));
      Serial.println(F("Reset the board after fixing."));
      while (true) { delay(5000); }
    }
    delay(500);
    Serial.print(F("."));
  }
  Serial.println();
  Serial.print(F("WiFi connected: "));
  Serial.println(WiFi.localIP());
  sessionStart_ms = millis();

  Serial.println(F("Press GPIO 0 button to start/stop NTRIP. Blinking LED = ready."));
}

// ============================================================
// Main loop
// ============================================================
void loop() {
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

// Blocking single-byte read with timeout. Returns -1 on timeout or disconnect.
// Required for HTTP/1.1 chunked decoder: chunk-size lines and payloads can straddle
// TCP packet boundaries under WiFi jitter, and a non-blocking read on a momentarily
// drained socket buffer permanently desyncs the decoder.
int readByteBlocking(WiFiClient &client, uint32_t timeout_ms) {
  uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    if (client.available()) return client.read();
    if (!client.connected()) return -1;
    delay(1);
  }
  return -1;
}

// ============================================================
// Build NMEA GGA from current ZED-F9P position.
// Polaris VRS uses this to place the virtual base station near the buoy.
// quality=0 if no fix yet — Polaris will wait until a valid position arrives.
// ============================================================
String buildGGA() {
  // One PVT poll caches every NAV-PVT field used below; subsequent getters return
  // cached values instead of issuing eight separate UART polls that each block
  // RTCM injection to the F9P.
  myGNSS.getPVT();

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
  unsigned long lastGGASent_ms = 0;
  bool prevConnected = false;
  unsigned long lastDiag_ms = 0;

  Serial.println(F("Subscribing to Polaris caster..."));

  while (ntripRunning) {

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

    // Socket state-transition log — catches silent TCP closes that the
    // reconnect block alone wouldn't surface as a distinct event.
    bool nowConnected = ntripClient.connected();
    if (nowConnected != prevConnected) {
      Serial.print(F("[socket] "));
      Serial.print(prevConnected ? F("connected") : F("disconnected"));
      Serial.print(F(" -> "));
      Serial.println(nowConnected ? F("connected") : F("disconnected"));
      prevConnected = nowConnected;
    }

    // 1 Hz diagnostics — distinguishes decoder desync from stream outage from
    // F9P-side RTK loss when a session goes bad.
    if (millis() - lastDiag_ms > 1000) {
      lastDiag_ms = millis();
      myGNSS.getPVT();
      Serial.print(F("[diag] carrier="));   Serial.print(myGNSS.getCarrierSolutionType());
      Serial.print(F(" siv="));             Serial.print(myGNSS.getSIV());
      Serial.print(F(" rtcmTotal="));       Serial.print(rtcmTotalBytes);
      Serial.print(F(" desyncSuspect="));   Serial.print(desyncSuspectCount);
      Serial.print(F(" crlfMismatch="));    Serial.println(trailingCRLFMismatch);
    }

    // Connect (or reconnect) to caster
    if (!ntripClient.connected()) {
      reconnectCount++;
      float elapsedMin = (millis() - sessionStart_ms) / 60000.0;
      Serial.print(F("Connect attempt #")); Serial.print(reconnectCount);
      Serial.print(F(" at ")); Serial.print(elapsedMin, 1); Serial.println(F(" min"));
      Serial.print(F("Opening socket to ")); Serial.println(casterHost);

      if (!ntripClient.connect(casterHost, casterPort)) {
        // Match the post-connect reconnect path: don't kill the session on a
        // single transient WiFi/TCP blip — back off and let the loop retry.
        Serial.println(F("Connection to caster failed — retrying in 1 s"));
        delay(1000);
        continue;
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
      bool casterTimedOut = false;
      while (ntripClient.available() == 0) {
        if (millis() - timeout > 5000) {
          Serial.println(F("Caster timed out — retrying in 1 s"));
          ntripClient.stop();
          casterTimedOut = true;
          break;
        }
        delay(10);
      }
      if (casterTimedOut) {
        delay(1000);
        continue;  // back to top of while(ntripRunning) — try again
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
        // 401 means credentials are wrong — don't retry forever, exit so you notice
        if (strstr(response, "401") > (char *)0) {
          Serial.println(F("401 Unauthorized — exiting (check secrets.h)"));
          ntripRunning = false;
          digitalWrite(LED_PIN, LOW);
          return;
        }
        // Anything else is probably transient (caster hiccup, weird response) — retry
        Serial.println(F("NTRIP connection failed — retrying in 2 s"));
        ntripClient.stop();
        delay(2000);
        continue;
      }

      lastReceivedRTCM_ms = millis();

      // Send initial GGA so Polaris VRS knows where to synthesize the virtual base station
      String gga = buildGGA();
      ntripClient.print(gga);
      lastGGASent_ms = millis();
      Serial.print(F("Sent GGA: ")); Serial.print(gga);
    }

    // Refresh GGA at ggaInterval_ms so Polaris VRS keeps the synthesized base near the rover
    if (ntripClient.connected() && millis() - lastGGASent_ms > ggaInterval_ms) {
      String gga = buildGGA();
      ntripClient.print(gga);
      lastGGASent_ms = millis();
      Serial.println(F("GGA refreshed"));
    }

    // Read chunked RTCM and write directly to ZED-F9P UART.
    // All reads are blocking with timeout: chunk-size lines and payloads can
    // straddle TCP packet boundaries, and a non-blocking read on a momentarily
    // drained socket buffer permanently desyncs the decoder.
    if (ntripClient.connected() && ntripClient.available()) {
      rtcmCount = 0;

      // Read hex chunk-size line, terminated by CRLF. Bounded write to chunkSizeBuf.
      char chunkSizeBuf[12];
      int idx = 0;
      bool sizeReadOk = true;
      while (idx < (int)sizeof(chunkSizeBuf) - 1) {
        int b = readByteBlocking(ntripClient, 5000);
        if (b < 0) { sizeReadOk = false; break; }
        if (b == '\n') break;
        if (b != '\r') chunkSizeBuf[idx++] = (char)b;
      }
      chunkSizeBuf[idx] = '\0';

      if (!sizeReadOk) {
        Serial.println(F("[desync?] timeout reading chunk size — dropping socket"));
        ntripClient.stop();
      } else {
        long chunkSize = strtol(chunkSizeBuf, NULL, 16);

        if (chunkSize == 0) {
          // HTTP/1.1 zero-size chunk = end of stream. For a Polaris RTCM
          // stream this only happens on caster termination or decoder desync;
          // either way, drop the socket and let the reconnect path run rather
          // than waiting ~100 s for the RTCM-silence timer to fire.
          Serial.println(F("[stream] chunkSize=0 (end of stream) — dropping socket"));
          ntripClient.stop();
        } else if (chunkSize < 0 || chunkSize > 4096) {
          // Polaris RTCM chunks are well under 4 KB; an oversized parse means
          // we are reading payload bytes as ASCII hex — decoder is desynced.
          desyncSuspectCount++;
          Serial.print(F("[desync?] chunkSize=")); Serial.print(chunkSize);
          Serial.print(F(" hex='"));               Serial.print(chunkSizeBuf);
          Serial.println(F("' — dropping socket"));
          ntripClient.stop();
        } else {
          // Consume exactly chunkSize bytes, flushing in buffer-sized passes
          // so chunks larger than rtcmData still pass through intact.
          uint8_t rtcmData[1024];
          long remaining = chunkSize;
          bool payloadOk = true;
          while (remaining > 0) {
            int want = remaining > (long)sizeof(rtcmData) ? (int)sizeof(rtcmData) : (int)remaining;
            int got = 0;
            while (got < want) {
              int b = readByteBlocking(ntripClient, 5000);
              if (b < 0) { payloadOk = false; break; }
              rtcmData[got++] = (uint8_t)b;
            }
            if (got > 0) {
              gpsSerial.write(rtcmData, got);
              rtcmCount      += got;
              rtcmTotalBytes += got;
            }
            if (!payloadOk) break;
            remaining -= got;
          }

          if (!payloadOk) {
            Serial.println(F("[desync?] short read on chunk payload — dropping socket"));
            ntripClient.stop();
          } else {
            // Trailing CRLF after chunk payload — blocking, so it can't be
            // skipped just because the TCP buffer drained between bytes.
            int b1 = readByteBlocking(ntripClient, 5000);
            int b2 = readByteBlocking(ntripClient, 5000);
            if (b1 != '\r' || b2 != '\n') {
              trailingCRLFMismatch++;
              Serial.print(F("[desync?] trailing CRLF mismatch: 0x"));
              if (b1 < 0) Serial.print(F("--")); else Serial.print((uint8_t)b1, HEX);
              Serial.print(F(" 0x"));
              if (b2 < 0) Serial.print(F("--")); else Serial.print((uint8_t)b2, HEX);
              Serial.println(F(" — dropping socket"));
              ntripClient.stop();
            }
          }
        }
      }

      if (rtcmCount > 0) {
        lastReceivedRTCM_ms = millis();
        Serial.print(F("RTCM → ZED: ")); Serial.println(rtcmCount);
      }
    }

    // RTCM silent too long → drop the socket and let the outer loop reconnect.
    // Keeps the buoy recovering on its own through WiFi blips / caster hiccups
    // without needing a physical button press.
    if (millis() - lastReceivedRTCM_ms > maxTimeBeforeHangup_ms) {
      Serial.println(F("RTCM timeout — reconnecting"));
      if (ntripClient.connected()) ntripClient.stop();
      lastReceivedRTCM_ms = millis(); // reset so we don't immediately re-trigger
      delay(1000);                    // brief backoff before reconnect
      continue;                       // back to top of while(ntripRunning)
    }

    delay(10);
  }

  Serial.println(F("NTRIP stopped"));
  ntripClient.stop();
  ntripRunning = false;
  digitalWrite(LED_PIN, LOW);
}
