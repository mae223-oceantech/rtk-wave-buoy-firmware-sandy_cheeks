/*
  ESP32 WiFi/Cellular NTRIP Client — RTK Wave Buoy
  Based on esp32_polaris_wifi.ino (WiFi/Polaris path)
  and buoy_combo.ino (cellular/SIM7000 path)

  Toggle transport with the define below:
    Commented out → WiFi, connects to Polaris (HTTP/1.1 + chunked transfer)
    Defined        → Cellular (LTE), connects to SIO caster (HTTP/1.0, no chunking)

  Eventually the cellular path will support Polaris too (requires GGA sending
  over TCP), but for Phase 1 it targets a simpler fixed-base NTRIP caster.

  WiFi hardware (same for both modes):
    ESP32 GPIO 27 (RX2) → ZED-F9P TX1/MISO   (F9P UART1)
    ESP32 GPIO 12 (TX2) → ZED-F9P RX1/MOSI   (F9P UART1)
    Button: GPIO 0 — press to start/stop NTRIP
    LED:    GPIO 13 — blink = ready, solid = NTRIP active

  Additional wiring for cellular mode:
    ESP32 GPIO 16 (RX1) → SIM7000 TX
    ESP32 GPIO 17 (TX1) → SIM7000 RX
    ESP32 GPIO 18       → SIM7000 PWRKEY
    ESP32 GPIO 5        → SIM7000 RST
    LiPo battery connected to SIM7000 JST connector (required)
*/

// ============================================================
// TRANSPORT TOGGLE — uncomment for cellular, leave commented for WiFi
// ============================================================
// #define USE_CELLULAR

// ============================================================
// Common includes
// ============================================================
#include "secrets.h"
#include <HardwareSerial.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

#if defined(ARDUINO_ARCH_ESP32)
#include "base64.h"
#else
#include <Base64.h>
#endif

// ============================================================
// Transport-specific includes and globals
// ============================================================
#ifdef USE_CELLULAR

#include "BotleticsSIM7000.h"
#define SIMCOM_7000

#define BOTLETICS_PWRKEY 18
#define RST_MODEM        5
#define TX_MODEM         17   // ESP32 TX1 → SIM7000 RX
#define RX_MODEM         16   // ESP32 RX1 ← SIM7000 TX

HardwareSerial modemSS(1);
Botletics_modem_LTE modem = Botletics_modem_LTE();

bool networkConnected = false;
bool gprsEnabled      = false;
char imei[16]         = {0};

#else  // WiFi

#include <WiFi.h>

int desyncSuspectCount   = 0;
int trailingCRLFMismatch = 0;

// Polaris VRS needs GGA every 10–30 s so the virtual base stays near the rover
const unsigned long ggaInterval_ms = 10000;

#endif  // USE_CELLULAR

// ============================================================
// GPS — shared by both transports
// ============================================================
HardwareSerial gpsSerial(2);
#define RX_GPS 27
#define TX_GPS 12
SFE_UBLOX_GNSS myGNSS;

// ============================================================
// Button / LED
// ============================================================
const int BUTTON_PIN = 0;
const int LED_PIN    = 13;

bool lastButtonState    = HIGH;
bool currentButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

bool ntripRunning  = false;
bool ledBlinkState = false;
unsigned long lastBlinkTime = 0;

// ============================================================
// RTCM / session tracking — shared
// ============================================================
long lastReceivedRTCM_ms         = 0;
const int maxTimeBeforeHangup_ms = 100000;

int reconnectCount        = 0;
unsigned long sessionStart_ms = 0;
unsigned long rtcmTotalBytes  = 0;

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);

#ifdef USE_CELLULAR
  Serial.println(F("RTK Wave Buoy — Cellular/SIO NTRIP Client"));
#else
  Serial.println(F("RTK Wave Buoy — WiFi/Polaris NTRIP Client"));
#endif

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // --- ZED-F9P init (both modes) ---
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
    // 5 Hz captures wave motion (periods ~5-20 s) without hitting F9P's RTK ceiling
    myGNSS.setNavigationFrequency(5);
  }

#ifdef USE_CELLULAR

  // --- SIM7000 init ---
  pinMode(RST_MODEM, OUTPUT);
  digitalWrite(RST_MODEM, HIGH);

  Serial.println(F("Powering on modem..."));
  modem.powerOn(BOTLETICS_PWRKEY);

  // SIM7000 boots at 115200; drop to 9600 for stability
  modemSS.begin(115200, SERIAL_8N1, TX_MODEM, RX_MODEM);
  Serial.println(F("Configuring modem to 9600 baud"));
  modemSS.println("AT+IPR=9600");
  delay(1000);
  modemSS.begin(9600, SERIAL_8N1, TX_MODEM, RX_MODEM);

  if (!modem.begin(modemSS)) {
    Serial.println(F("ERROR: SIM7000 not found — check wiring and LiPo battery"));
    while (1);
  }
  Serial.println(F("SIM7000 detected."));

  uint8_t imeiLen = modem.getIMEI(imei);
  if (imeiLen > 0) { Serial.print(F("IMEI: ")); Serial.println(imei); }

  modem.setFunctionality(1);                       // AT+CFUN=1 — full RF on
  modem.setNetworkSettings(F(CELLULAR_APN));       // APN from secrets.h

  // Wait for LTE registration (up to 60 s)
  Serial.println(F("Waiting for network registration..."));
  uint8_t netStatus = 0;
  unsigned long netStart = millis();
  while (netStatus != 1 && netStatus != 5) {
    if (millis() - netStart > 60000) {
      Serial.println(F("ERROR: Network registration timed out. Check SIM card."));
      break;
    }
    netStatus = modem.getNetworkStatus();
    Serial.print(F("Network status: ")); Serial.println(netStatus);
    delay(3000);
  }

  if (netStatus == 1 || netStatus == 5) {
    networkConnected = true;
    Serial.println(F("Network registered."));
  }

  // Enable GPRS/data
  if (networkConnected) {
    modem.enableGPRS(false);   // clean slate before enabling
    delay(1000);
    Serial.println(F("Enabling GPRS..."));
    for (int i = 1; i <= 3 && !gprsEnabled; i++) {
      Serial.print(F("GPRS attempt ")); Serial.print(i); Serial.println(F("/3"));
      if (modem.enableGPRS(true)) {
        gprsEnabled = true;
        Serial.println(F("GPRS enabled."));
      } else {
        delay(i * 3000);   // back off: 3 s, 6 s, 9 s
      }
    }
    if (!gprsEnabled) {
      Serial.println(F("WARNING: GPRS failed. Check SIM card and APN in secrets.h."));
    }
  }

#else  // WiFi

  Serial.print(F("Connecting to WiFi"));
  WiFi.begin(ssid, password);
  unsigned long wifiStart_ms = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - wifiStart_ms > 30000) {
      Serial.println();
      Serial.println(F("ERROR: WiFi timed out. Check ssid/password in secrets.h."));
      while (true) { delay(5000); }
    }
    delay(500);
    Serial.print(F("."));
  }
  Serial.println();
  Serial.print(F("WiFi connected: "));
  Serial.println(WiFi.localIP());

#endif  // USE_CELLULAR

  sessionStart_ms = millis();
  Serial.println(F("Press GPIO 0 button to start/stop NTRIP. Blinking LED = ready."));
}

// ============================================================
// Main loop — button debounce + LED heartbeat (same for both transports)
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

  // Blink while idle — indicates transport is connected and ready
  if (!ntripRunning) {
    if (millis() - lastBlinkTime >= 1000) {
      lastBlinkTime = millis();
      ledBlinkState = !ledBlinkState;
      digitalWrite(LED_PIN, ledBlinkState);
    }
  }
}

// ============================================================
// buildGGA — construct NMEA GGA from current ZED-F9P position.
// Used by the WiFi/Polaris path to keep the VRS base near the rover.
// Included in the cellular build too, ready for Phase 2 Polaris support.
// ============================================================
String buildGGA() {
  // Single getPVT() caches all NAV-PVT fields; subsequent getters return
  // cached values rather than issuing separate blocking UART polls.
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

  // GGA quality: 0=no fix, 1=GNSS, 4=RTK fixed, 5=RTK float
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
// CELLULAR beginClient
// Connects to SIO (or any NTRIP/1.0 fixed-base caster) via SIM7000 TCP.
// Uses HTTP/1.0 — no chunked transfer encoding, no mandatory GGA.
// ============================================================
#ifdef USE_CELLULAR

void beginClient() {
  Serial.println(F("Subscribing to NTRIP caster via cellular..."));
  bool tcpConnected = false;
  unsigned long lastDiag_ms = 0;

  while (ntripRunning) {

    // --- Button check (mirrors loop()) ---
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

    // --- 1 Hz diagnostics ---
    if (millis() - lastDiag_ms > 1000) {
      lastDiag_ms = millis();
      myGNSS.getPVT();
      Serial.print(F("[diag] carrier="));  Serial.print(myGNSS.getCarrierSolutionType());
      Serial.print(F(" siv="));            Serial.print(myGNSS.getSIV());
      Serial.print(F(" rtcmTotal="));      Serial.print(rtcmTotalBytes);
      Serial.print(F(" reconnects="));     Serial.println(reconnectCount);
    }

    // --- Connect / reconnect ---
    if (!tcpConnected || !modem.TCPconnected()) {
      if (tcpConnected) {
        // Close the dead socket before reconnecting
        modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
        tcpConnected = false;
        delay(1000);
      }

      // Re-check GPRS — can drop if signal is lost
      uint8_t netStat = modem.getNetworkStatus();
      if (!gprsEnabled || (netStat != 1 && netStat != 5)) {
        Serial.println(F("GPRS lost — re-enabling..."));
        modem.enableGPRS(false);
        delay(1000);
        gprsEnabled = modem.enableGPRS(true);
        if (!gprsEnabled) {
          Serial.println(F("GPRS re-enable failed — retrying in 5 s"));
          delay(5000);
          continue;
        }
        Serial.println(F("GPRS restored."));
      }

      reconnectCount++;
      float elapsedMin = (millis() - sessionStart_ms) / 60000.0;
      Serial.print(F("TCP connect attempt #")); Serial.print(reconnectCount);
      Serial.print(F(" at ")); Serial.print(elapsedMin, 1); Serial.println(F(" min"));
      Serial.print(F("Connecting to ")); Serial.print(casterHost);
      Serial.print(F(":")); Serial.println(casterPort);

      if (!modem.TCPconnect((char*)casterHost, casterPort)) {
        Serial.println(F("TCP connect failed — retrying in 2 s"));
        delay(2000);
        continue;
      }
      delay(500);   // let the connection settle before sending

      // Build HTTP/1.0 NTRIP request (no chunked encoding)
      char credentials[200] = "";
      if (strlen(casterUser) > 0) {
        char userCreds[128];
        snprintf(userCreds, sizeof(userCreds), "%s:%s", casterUser, casterUserPW);
        base64 b;
        String encoded = b.encode(userCreds);
        snprintf(credentials, sizeof(credentials),
          "Authorization: Basic %s\r\n", encoded.c_str());
      }

      char serverRequest[512];
      snprintf(serverRequest, sizeof(serverRequest),
        "GET /%s HTTP/1.0\r\n"
        "User-Agent: NTRIP ESP32 Client v1.0\r\n"
        "%s"
        "\r\n",
        mountPoint, credentials);

      Serial.println(F("Sending NTRIP request..."));
      if (!modem.TCPsend((char*)serverRequest, strlen(serverRequest))) {
        Serial.println(F("Send failed — retrying"));
        modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
        delay(2000);
        continue;
      }

      // Read HTTP response header.
      // Scan byte-by-byte for \r\n\r\n (end of header). Any bytes that arrive
      // after the header boundary are the start of RTCM — forward them immediately.
      char headerBuf[512] = {0};
      int  headerLen      = 0;
      bool headerDone     = false;
      unsigned long headerTimeout = millis();

      while (!headerDone && millis() - headerTimeout < 5000) {
        uint16_t avail = modem.TCPavailable();
        if (avail > 0) {
          uint8_t chunk[128];
          uint16_t got = modem.TCPread(chunk, min((uint16_t)sizeof(chunk), avail));
          for (int i = 0; i < (int)got && !headerDone; i++) {
            if (headerLen < (int)sizeof(headerBuf) - 1) {
              headerBuf[headerLen++] = (char)chunk[i];
            }
            // Detect \r\n\r\n
            if (headerLen >= 4 &&
                headerBuf[headerLen-4] == '\r' && headerBuf[headerLen-3] == '\n' &&
                headerBuf[headerLen-2] == '\r' && headerBuf[headerLen-1] == '\n') {
              headerDone = true;
              // Forward any bytes that arrived after the header in this same read
              int trailing = (int)got - (i + 1);
              if (trailing > 0) {
                gpsSerial.write(chunk + i + 1, trailing);
                rtcmTotalBytes += trailing;
              }
            }
          }
        }
        delay(10);
      }

      Serial.print(F("Caster response: ")); Serial.println(headerBuf);

      // Check for auth failure first — wrong credentials, don't loop forever
      if (strstr(headerBuf, "401") != NULL) {
        Serial.println(F("401 Unauthorized — check casterUser/casterUserPW in secrets.h"));
        ntripRunning = false;
        digitalWrite(LED_PIN, LOW);
        return;
      }

      bool connectionSuccess = (strstr(headerBuf, "200") != NULL ||
                                strstr(headerBuf, "ICY") != NULL);
      if (!connectionSuccess) {
        Serial.println(F("NTRIP connection failed — retrying in 2 s"));
        modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
        delay(2000);
        continue;
      }

      Serial.println(F("NTRIP connected via cellular!"));
      tcpConnected = true;
      lastReceivedRTCM_ms = millis();
    }

    // --- Read RTCM and push to ZED-F9P ---
    uint16_t avail = modem.TCPavailable();
    if (avail > 0) {
      uint8_t buf[256];
      uint16_t got = modem.TCPread(buf, min((uint16_t)sizeof(buf), avail));
      if (got > 0) {
        gpsSerial.write(buf, got);
        rtcmTotalBytes += got;
        lastReceivedRTCM_ms = millis();
        Serial.print(F("RTCM → ZED: ")); Serial.println(got);
      }
    }

    // --- RTCM silence timeout → drop socket and reconnect ---
    if (millis() - lastReceivedRTCM_ms > maxTimeBeforeHangup_ms) {
      Serial.println(F("RTCM timeout — reconnecting"));
      modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
      tcpConnected = false;
      lastReceivedRTCM_ms = millis();
      delay(1000);
      continue;
    }

    delay(10);
  }

  // Clean up on exit
  if (tcpConnected) modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
  Serial.println(F("NTRIP stopped"));
  ntripRunning = false;
  digitalWrite(LED_PIN, LOW);
}

#else  // WiFi

// ============================================================
// WiFi beginClient
// Connects to Polaris (or any NTRIP/2.0 caster) over WiFi.
// Uses HTTP/1.1 with chunked transfer encoding.
// Sends GGA on connect and refreshes every ggaInterval_ms.
// ============================================================

// Blocking single-byte read with timeout. Required for the HTTP/1.1 chunked
// decoder: chunk-size lines and payloads can straddle TCP packet boundaries
// under WiFi jitter, and a non-blocking read on a drained buffer desyncs
// the decoder permanently.
int readByteBlocking(WiFiClient &client, uint32_t timeout_ms) {
  uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    if (client.available()) return client.read();
    if (!client.connected()) return -1;
    delay(1);
  }
  return -1;
}

void beginClient() {
  WiFiClient ntripClient;
  long rtcmCount = 0;
  unsigned long lastGGASent_ms = 0;
  bool prevConnected = false;
  unsigned long lastDiag_ms = 0;

  Serial.println(F("Subscribing to Polaris caster..."));

  while (ntripRunning) {

    // --- Button check ---
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

    // Log socket state transitions
    bool nowConnected = ntripClient.connected();
    if (nowConnected != prevConnected) {
      Serial.print(F("[socket] "));
      Serial.print(prevConnected ? F("connected") : F("disconnected"));
      Serial.print(F(" -> "));
      Serial.println(nowConnected ? F("connected") : F("disconnected"));
      prevConnected = nowConnected;
    }

    // --- 1 Hz diagnostics ---
    if (millis() - lastDiag_ms > 1000) {
      lastDiag_ms = millis();
      myGNSS.getPVT();
      Serial.print(F("[diag] carrier="));   Serial.print(myGNSS.getCarrierSolutionType());
      Serial.print(F(" siv="));             Serial.print(myGNSS.getSIV());
      Serial.print(F(" rtcmTotal="));       Serial.print(rtcmTotalBytes);
      Serial.print(F(" desyncSuspect="));   Serial.print(desyncSuspectCount);
      Serial.print(F(" crlfMismatch="));    Serial.println(trailingCRLFMismatch);
    }

    // --- Connect / reconnect ---
    if (!ntripClient.connected()) {
      reconnectCount++;
      float elapsedMin = (millis() - sessionStart_ms) / 60000.0;
      Serial.print(F("Connect attempt #")); Serial.print(reconnectCount);
      Serial.print(F(" at ")); Serial.print(elapsedMin, 1); Serial.println(F(" min"));
      Serial.print(F("Opening socket to ")); Serial.println(casterHost);

      if (!ntripClient.connect(casterHost, casterPort)) {
        Serial.println(F("Connection to caster failed — retrying in 1 s"));
        delay(1000);
        continue;
      }

      Serial.print(F("Connected to ")); Serial.print(casterHost);
      Serial.print(F(":")); Serial.println(casterPort);

      // HTTP/1.1 with Ntrip-Version: Ntrip/2.0 — required for Polaris chunked stream
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
        base64 b;
        String strEncoded = b.encode(userCredentials);
        char encodedCredentials[strEncoded.length() + 1];
        strEncoded.toCharArray(encodedCredentials, sizeof(encodedCredentials));
        snprintf(credentials, sizeof(credentials),
          "Authorization: Basic %s\r\n", encodedCredentials);
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
      if (casterTimedOut) { delay(1000); continue; }

      // Parse HTTP response — look for 200 OK or 401
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
        if (strstr(response, "401") > (char *)0) {
          Serial.println(F("401 Unauthorized — exiting (check secrets.h)"));
          ntripRunning = false;
          digitalWrite(LED_PIN, LOW);
          return;
        }
        Serial.println(F("NTRIP connection failed — retrying in 2 s"));
        ntripClient.stop();
        delay(2000);
        continue;
      }

      lastReceivedRTCM_ms = millis();

      // Send initial GGA so Polaris VRS places the virtual base near the buoy
      String gga = buildGGA();
      ntripClient.print(gga);
      lastGGASent_ms = millis();
      Serial.print(F("Sent GGA: ")); Serial.print(gga);
    }

    // --- Refresh GGA at ggaInterval_ms ---
    if (ntripClient.connected() && millis() - lastGGASent_ms > ggaInterval_ms) {
      String gga = buildGGA();
      ntripClient.print(gga);
      lastGGASent_ms = millis();
      Serial.println(F("GGA refreshed"));
    }

    // --- Read HTTP/1.1 chunked RTCM and write to ZED-F9P ---
    if (ntripClient.connected() && ntripClient.available()) {
      rtcmCount = 0;

      // Read hex chunk-size line (terminated by CRLF)
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
          // Zero chunk = end of HTTP/1.1 stream — caster terminated or desync
          Serial.println(F("[stream] chunkSize=0 — dropping socket"));
          ntripClient.stop();
        } else if (chunkSize < 0 || chunkSize > 4096) {
          // Payload bytes read as hex → decoder desynced
          desyncSuspectCount++;
          Serial.print(F("[desync?] chunkSize=")); Serial.print(chunkSize);
          Serial.print(F(" hex='")); Serial.print(chunkSizeBuf);
          Serial.println(F("' — dropping socket"));
          ntripClient.stop();
        } else {
          // Consume exactly chunkSize bytes, passing in 1 KB blocks
          uint8_t rtcmData[1024];
          long remaining = chunkSize;
          bool payloadOk = true;
          while (remaining > 0) {
            int want = remaining > (long)sizeof(rtcmData) ? (int)sizeof(rtcmData) : (int)remaining;
            int got  = 0;
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
            // Consume trailing CRLF after chunk payload
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

    // --- RTCM silence timeout → reconnect ---
    if (millis() - lastReceivedRTCM_ms > maxTimeBeforeHangup_ms) {
      Serial.println(F("RTCM timeout — reconnecting"));
      if (ntripClient.connected()) ntripClient.stop();
      lastReceivedRTCM_ms = millis();
      delay(1000);
      continue;
    }

    delay(10);
  }

  Serial.println(F("NTRIP stopped"));
  ntripClient.stop();
  ntripRunning = false;
  digitalWrite(LED_PIN, LOW);
}

#endif  // USE_CELLULAR / WiFi beginClient
