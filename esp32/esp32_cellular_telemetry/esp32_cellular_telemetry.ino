/*
  ESP32 Cellular NTRIP Client — RTK Wave Buoy  (+ live telemetry)
  SIM7000 LTE + ZED-F9P

  Two NTRIP modes, selected by the define below:

    USE_POLARIS_CELLULAR commented out (default):
      Cellular + SIO / fixed-base caster
      HTTP/1.0, no chunked decoder, no GGA required.

    USE_POLARIS_CELLULAR defined:
      Cellular + Point One Nav Polaris
      HTTP/1.1 chunked stream, GGA sent every 10 s.

  Telemetry (NEW):
    Every TELEMETRY_INTERVAL_MS the NTRIP connection is paused (~10 s),
    a JSON snapshot is HTTP-POSTed to TELEMETRY_HOST/data, then NTRIP
    reconnects automatically.  Run tools/buoy_server.py on your laptop
    and expose it with:  ngrok http --domain=<your-static-domain> 5000
    Put your ngrok domain in secrets.h as TELEMETRY_HOST.

  Wiring:
    ESP32 GPIO 27 (RX2) → ZED-F9P TX1/MISO   (F9P UART1)
    ESP32 GPIO 12 (TX2) → ZED-F9P RX1/MOSI   (F9P UART1)
    ESP32 GPIO 16 (RX1) → SIM7000 TX
    ESP32 GPIO 17 (TX1) → SIM7000 RX
    ESP32 GPIO 18       → SIM7000 PWRKEY
    ESP32 GPIO 5        → SIM7000 RST
    Button: GPIO 0 — press to start/stop NTRIP
    LED:    GPIO 13 — blink = ready, solid = NTRIP active
    LiPo battery connected to SIM7000 JST connector (required)
*/

// ============================================================
// MODE TOGGLE
// Commented out = SIO / fixed-base (HTTP/1.0, no GGA)
// Uncommented   = Polaris (HTTP/1.1 chunked, GGA every 10 s)
// ============================================================
// #define USE_POLARIS_CELLULAR

// ============================================================
// Includes
// ============================================================
#include "secrets.h"
#include <HardwareSerial.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include "BotleticsSIM7000.h"

#if defined(ARDUINO_ARCH_ESP32)
#include "base64.h"
#else
#include <Base64.h>
#endif

#define SIMCOM_7000

// ============================================================
// SIM7000 pins and objects
// ============================================================
#define BOTLETICS_PWRKEY 18
#define RST_MODEM        5
#define TX_MODEM         17   // ESP32 TX1 → SIM7000 RX
#define RX_MODEM         16   // ESP32 RX1 ← SIM7000 TX

HardwareSerial modemSS(1);
Botletics_modem_LTE modem = Botletics_modem_LTE();

bool networkConnected = false;
bool gprsEnabled      = false;
char imei[16]         = {0};

// ============================================================
// ZED-F9P
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
unsigned long lastDebounceTime  = 0;
const unsigned long debounceDelay = 50;

bool ntripRunning  = false;
bool ledBlinkState = false;
unsigned long lastBlinkTime = 0;

// ============================================================
// RTCM / session tracking
// ============================================================
long lastReceivedRTCM_ms         = 0;
const int maxTimeBeforeHangup_ms = 100000;

int reconnectCount        = 0;
unsigned long sessionStart_ms = 0;
unsigned long rtcmTotalBytes  = 0;

#ifdef USE_POLARIS_CELLULAR
int desyncSuspectCount   = 0;
int trailingCRLFMismatch = 0;
const unsigned long ggaInterval_ms = 10000;
#endif

// ============================================================
// [TELEMETRY] Interval between telemetry posts (ms).
// The NTRIP connection is closed for ~10 s each time this fires.
// ============================================================
const unsigned long telemetryInterval_ms = 60000;
unsigned long lastTelemetry_ms = 0;

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);

#ifdef USE_POLARIS_CELLULAR
  Serial.println(F("RTK Wave Buoy — Cellular/Polaris NTRIP Client + Telemetry"));
#else
  Serial.println(F("RTK Wave Buoy — Cellular/SIO NTRIP Client + Telemetry"));
#endif

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // --- ZED-F9P ---
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
    myGNSS.setNavigationFrequency(5);
  }

  // --- SIM7000 ---
  pinMode(RST_MODEM, OUTPUT);
  digitalWrite(RST_MODEM, HIGH);

  Serial.println(F("Powering on modem..."));
  modem.powerOn(BOTLETICS_PWRKEY);

  // NOTE: ESP32 HardwareSerial::begin signature is begin(baud, config, rxPin, txPin),
  // so the canonical call would be (..., RX_MODEM, TX_MODEM). The order below
  // (TX_MODEM, RX_MODEM) appears swapped, but it matches the TA's working setup —
  // do not "fix" without confirming on hardware first.
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

  modem.setFunctionality(1);
  modem.setNetworkSettings(F(CELLULAR_APN));

  // [TELEMETRY] Enable SSL so postData() can reach the ngrok HTTPS URL.
  modem.setHTTPSRedirect(true);

  // Wait for LTE registration
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

  // Enable GPRS
  if (networkConnected) {
    modem.enableGPRS(false);
    delay(1000);
    Serial.println(F("Enabling GPRS..."));
    for (int i = 1; i <= 3 && !gprsEnabled; i++) {
      Serial.print(F("GPRS attempt ")); Serial.print(i); Serial.println(F("/3"));
      if (modem.enableGPRS(true)) {
        gprsEnabled = true;
        Serial.println(F("GPRS enabled."));
      } else {
        delay(i * 3000);
      }
    }
    if (!gprsEnabled)
      Serial.println(F("WARNING: GPRS failed. Check SIM card and APN in secrets.h."));
  }

  sessionStart_ms    = millis();
  lastTelemetry_ms   = millis();  // [TELEMETRY] don't post immediately on boot
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

  if (!ntripRunning) {
    if (millis() - lastBlinkTime >= 1000) {
      lastBlinkTime = millis();
      ledBlinkState = !ledBlinkState;
      digitalWrite(LED_PIN, ledBlinkState);
    }
  }
}

// ============================================================
// Helpers
// ============================================================

bool ensureGPRS() {
  uint8_t netStat = modem.getNetworkStatus();
  if (gprsEnabled && (netStat == 1 || netStat == 5)) return true;
  Serial.println(F("GPRS lost — re-enabling..."));
  modem.enableGPRS(false);
  delay(1000);
  gprsEnabled = modem.enableGPRS(true);
  if (gprsEnabled) Serial.println(F("GPRS restored."));
  else             Serial.println(F("GPRS re-enable failed."));
  return gprsEnabled;
}

// ============================================================
// [TELEMETRY] postTelemetry
// Reads the current ZED-F9P state and POSTs a JSON snapshot to
// TELEMETRY_HOST/data.  Call this only after closing the NTRIP
// TCP socket — the modem's HTTP layer and active TCP socket
// cannot run simultaneously.
// ============================================================
void postTelemetry() {
  myGNSS.getPVT();
  uint8_t carrier = myGNSS.getCarrierSolutionType();
  uint8_t siv     = myGNSS.getSIV();
  double  lat     = myGNSS.getLatitude()    / 10000000.0;
  double  lon     = myGNSS.getLongitude()   / 10000000.0;
  double  alt     = myGNSS.getAltitudeMSL() / 1000.0;

  char latBuf[14], lonBuf[14], altBuf[10];
  dtostrf(lat, 1, 6, latBuf);
  dtostrf(lon, 1, 6, lonBuf);
  dtostrf(alt, 1, 1, altBuf);

  char url[100];
  snprintf(url, sizeof(url), "https://" TELEMETRY_HOST "/data");

  char body[220];
  snprintf(body, sizeof(body),
    "{\"lat\":%s,\"lon\":%s,\"alt\":%s,"
    "\"carrier\":%d,\"siv\":%d,"
    "\"rtcm_bytes\":%lu,\"reconnects\":%d,\"uptime_s\":%lu}",
    latBuf, lonBuf, altBuf,
    carrier, siv,
    rtcmTotalBytes, reconnectCount, millis() / 1000UL);

  Serial.print(F("[telemetry] POST → ")); Serial.println(url);
  Serial.print(F("[telemetry] ")); Serial.println(body);

  if (!modem.postData("POST", url, body)) {
    Serial.println(F("[telemetry] POST failed"));
  } else {
    Serial.println(F("[telemetry] OK"));
  }
}

#ifdef USE_POLARIS_CELLULAR
String buildGGA() {
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

  int quality = 0;
  if (fix >= 2) {
    if      (carrier == 2) quality = 4;
    else if (carrier == 1) quality = 5;
    else                   quality = 1;
  }

  char latDir = (lat >= 0) ? 'N' : 'S'; double absLat = fabs(lat);
  char lonDir = (lon >= 0) ? 'E' : 'W'; double absLon = fabs(lon);
  int latDeg = (int)absLat; double latMin = (absLat - latDeg) * 60.0;
  int lonDeg = (int)absLon; double lonMin = (absLon - lonDeg) * 60.0;

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

static uint8_t _cellBuf[256];
static int     _cellBufLen = 0;
static int     _cellBufIdx = 0;

int readByteCellular(uint32_t timeout_ms) {
  uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    if (_cellBufIdx < _cellBufLen) return (uint8_t)_cellBuf[_cellBufIdx++];
    _cellBufLen = 0; _cellBufIdx = 0;
    uint16_t avail = modem.TCPavailable();
    if (avail > 0) {
      int got = modem.TCPread(_cellBuf, min((uint16_t)sizeof(_cellBuf), avail));
      if (got > 0) { _cellBufLen = got; continue; }
    }
    if (!modem.TCPconnected()) return -1;
    delay(1);
  }
  return -1;
}
#endif  // USE_POLARIS_CELLULAR

// ============================================================
// Cellular + Polaris beginClient
// ============================================================
#ifdef USE_POLARIS_CELLULAR

void beginClient() {
  Serial.println(F("Subscribing to Polaris via cellular (HTTP/1.1 + chunked + GGA)..."));
  bool tcpConnected    = false;
  unsigned long lastGGASent_ms = 0;
  unsigned long lastDiag_ms    = 0;

  while (ntripRunning) {

    // Button check
    int reading = digitalRead(BUTTON_PIN);
    if (reading != lastButtonState) lastDebounceTime = millis();
    if ((millis() - lastDebounceTime) > debounceDelay) {
      if (reading != currentButtonState) {
        currentButtonState = reading;
        if (currentButtonState == LOW) {
          Serial.println(F("Button — stopping NTRIP"));
          ntripRunning = false; digitalWrite(LED_PIN, LOW); break;
        }
      }
    }
    lastButtonState = reading;

    // 1 Hz diagnostics
    if (millis() - lastDiag_ms > 1000) {
      lastDiag_ms = millis();
      myGNSS.getPVT();
      Serial.print(F("[diag] carrier="));   Serial.print(myGNSS.getCarrierSolutionType());
      Serial.print(F(" siv="));             Serial.print(myGNSS.getSIV());
      Serial.print(F(" rtcmTotal="));       Serial.print(rtcmTotalBytes);
      Serial.print(F(" desyncSuspect="));   Serial.print(desyncSuspectCount);
      Serial.print(F(" crlfMismatch="));    Serial.println(trailingCRLFMismatch);
    }

    // Connect / reconnect
    if (!tcpConnected || !modem.TCPconnected()) {
      if (tcpConnected) {
        modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
        tcpConnected = false;
        _cellBufLen = _cellBufIdx = 0;
        delay(1000);
      }

      if (!ensureGPRS()) { delay(5000); continue; }

      reconnectCount++;
      float elapsedMin = (millis() - sessionStart_ms) / 60000.0;
      Serial.print(F("TCP connect attempt #")); Serial.print(reconnectCount);
      Serial.print(F(" at ")); Serial.print(elapsedMin, 1); Serial.println(F(" min"));
      Serial.print(F("Connecting to ")); Serial.print(casterHost);
      Serial.print(F(":")); Serial.println(casterPort);

      if (!modem.TCPconnect((char*)casterHost, casterPort)) {
        Serial.println(F("TCP connect failed — retrying in 2 s"));
        delay(2000); continue;
      }
      delay(500);

      const int REQ_SIZE = 512;
      char serverRequest[REQ_SIZE];
      snprintf(serverRequest, REQ_SIZE,
        "GET /%s HTTP/1.1\r\nHost: %s\r\nNtrip-Version: Ntrip/2.0\r\n"
        "User-Agent: NTRIP ESP32 Client v1.0\r\n",
        mountPoint, casterHost);

      char credentials[256];
      if (strlen(casterUser) == 0) {
        strncpy(credentials, "Accept: */*\r\nConnection: close\r\n", sizeof(credentials));
      } else {
        char userCreds[128];
        snprintf(userCreds, sizeof(userCreds), "%s:%s", casterUser, casterUserPW);
        base64 b;
        String encoded = b.encode(userCreds);
        snprintf(credentials, sizeof(credentials),
          "Authorization: Basic %s\r\n", encoded.c_str());
      }
      strncat(serverRequest, credentials, REQ_SIZE - strlen(serverRequest) - 1);
      strncat(serverRequest, "\r\n",     REQ_SIZE - strlen(serverRequest) - 1);

      if (!modem.TCPsend((char*)serverRequest, strlen(serverRequest))) {
        Serial.println(F("Send failed — retrying in 2 s"));
        modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
        delay(2000); continue;
      }

      unsigned long timeout = millis();
      bool timedOut = false;
      while (modem.TCPavailable() == 0) {
        if (millis() - timeout > 5000) {
          Serial.println(F("Caster timed out — retrying in 1 s"));
          modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
          timedOut = true; break;
        }
        delay(10);
      }
      if (timedOut) { delay(1000); continue; }

      char headerBuf[512] = {0};
      int  headerLen  = 0;
      bool headerDone = false;
      _cellBufLen = _cellBufIdx = 0;
      unsigned long headerTimeout = millis();

      while (!headerDone && millis() - headerTimeout < 5000) {
        uint16_t avail = modem.TCPavailable();
        if (avail > 0) {
          uint8_t chunk[128];
          uint16_t got = modem.TCPread(chunk, min((uint16_t)sizeof(chunk), avail));
          for (int i = 0; i < (int)got; i++) {
            if (!headerDone) {
              if (headerLen < (int)sizeof(headerBuf) - 1)
                headerBuf[headerLen++] = (char)chunk[i];
              if (headerLen >= 4 &&
                  headerBuf[headerLen-4] == '\r' && headerBuf[headerLen-3] == '\n' &&
                  headerBuf[headerLen-2] == '\r' && headerBuf[headerLen-1] == '\n') {
                headerDone = true;
                int trailing = (int)got - (i + 1);
                if (trailing > 0 && trailing <= (int)sizeof(_cellBuf)) {
                  memcpy(_cellBuf, chunk + i + 1, trailing);
                  _cellBufLen = trailing; _cellBufIdx = 0;
                }
              }
            }
          }
        }
        delay(10);
      }

      Serial.print(F("Caster response: ")); Serial.println(headerBuf);

      if (strstr(headerBuf, "401") != NULL) {
        Serial.println(F("401 Unauthorized — check casterUser/casterUserPW in secrets.h"));
        ntripRunning = false; digitalWrite(LED_PIN, LOW); return;
      }
      if (strstr(headerBuf, "200") == NULL) {
        Serial.println(F("NTRIP connection failed — retrying in 2 s"));
        modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
        _cellBufLen = _cellBufIdx = 0;
        delay(2000); continue;
      }

      String gga = buildGGA();
      modem.TCPsend((char*)gga.c_str(), gga.length());
      lastGGASent_ms = millis();
      Serial.print(F("Sent GGA: ")); Serial.print(gga);

      Serial.println(F("NTRIP connected via cellular/Polaris!"));
      tcpConnected = true;
      lastReceivedRTCM_ms = millis();
    }

    // Refresh GGA
    if (tcpConnected && millis() - lastGGASent_ms > ggaInterval_ms) {
      String gga = buildGGA();
      modem.TCPsend((char*)gga.c_str(), gga.length());
      lastGGASent_ms = millis();
      Serial.println(F("GGA refreshed"));
    }

    // HTTP/1.1 chunked decoder
    if (modem.TCPavailable() > 0 || _cellBufIdx < _cellBufLen) {
      long rtcmCount = 0;

      char chunkSizeBuf[12];
      int  idx = 0; bool sizeReadOk = true;
      while (idx < (int)sizeof(chunkSizeBuf) - 1) {
        int b = readByteCellular(5000);
        if (b < 0) { sizeReadOk = false; break; }
        if (b == '\n') break;
        if (b != '\r') chunkSizeBuf[idx++] = (char)b;
      }
      chunkSizeBuf[idx] = '\0';

      if (!sizeReadOk) {
        Serial.println(F("[desync?] timeout reading chunk size — dropping socket"));
        modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
        tcpConnected = false; _cellBufLen = _cellBufIdx = 0;
      } else {
        long chunkSize = strtol(chunkSizeBuf, NULL, 16);

        if (chunkSize == 0) {
          Serial.println(F("[stream] chunkSize=0 — caster closed stream"));
          modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
          tcpConnected = false; _cellBufLen = _cellBufIdx = 0;
        } else if (chunkSize < 0 || chunkSize > 4096) {
          desyncSuspectCount++;
          Serial.print(F("[desync?] chunkSize=")); Serial.print(chunkSize);
          Serial.print(F(" hex='")); Serial.print(chunkSizeBuf);
          Serial.println(F("' — dropping socket"));
          modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
          tcpConnected = false; _cellBufLen = _cellBufIdx = 0;
        } else {
          uint8_t rtcmData[1024];
          long remaining = chunkSize; bool payloadOk = true;
          while (remaining > 0) {
            int want = remaining > (long)sizeof(rtcmData) ? (int)sizeof(rtcmData) : (int)remaining;
            int got  = 0;
            while (got < want) {
              int b = readByteCellular(5000);
              if (b < 0) { payloadOk = false; break; }
              rtcmData[got++] = (uint8_t)b;
            }
            if (got > 0) { gpsSerial.write(rtcmData, got); rtcmCount += got; rtcmTotalBytes += got; }
            if (!payloadOk) break;
            remaining -= got;
          }

          if (!payloadOk) {
            Serial.println(F("[desync?] short read on payload — dropping socket"));
            modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
            tcpConnected = false; _cellBufLen = _cellBufIdx = 0;
          } else {
            int b1 = readByteCellular(5000);
            int b2 = readByteCellular(5000);
            if (b1 != '\r' || b2 != '\n') {
              trailingCRLFMismatch++;
              Serial.print(F("[desync?] trailing CRLF mismatch: 0x"));
              if (b1 < 0) Serial.print(F("--")); else Serial.print((uint8_t)b1, HEX);
              Serial.print(F(" 0x"));
              if (b2 < 0) Serial.print(F("--")); else Serial.print((uint8_t)b2, HEX);
              Serial.println(F(" — dropping socket"));
              modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
              tcpConnected = false; _cellBufLen = _cellBufIdx = 0;
            }
          }
        }
      }

      if (rtcmCount > 0) {
        lastReceivedRTCM_ms = millis();
        Serial.print(F("RTCM → ZED: ")); Serial.println(rtcmCount);
      }
    }

    // [TELEMETRY] Close NTRIP, post snapshot, let loop reconnect.
    if (tcpConnected && millis() - lastTelemetry_ms > telemetryInterval_ms) {
      Serial.println(F("[telemetry] Pausing NTRIP to post telemetry..."));
      modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
      tcpConnected = false;
      _cellBufLen = _cellBufIdx = 0;
      postTelemetry();
      lastTelemetry_ms = millis();
      // outer loop reconnects NTRIP on the next iteration
    }

    if (millis() - lastReceivedRTCM_ms > maxTimeBeforeHangup_ms) {
      Serial.println(F("RTCM timeout — reconnecting"));
      modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
      tcpConnected = false; _cellBufLen = _cellBufIdx = 0;
      lastReceivedRTCM_ms = millis(); delay(1000); continue;
    }

    delay(10);
  }

  if (tcpConnected) modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
  Serial.println(F("NTRIP stopped"));
  ntripRunning = false; digitalWrite(LED_PIN, LOW);
}

// ============================================================
// Cellular + SIO beginClient
// ============================================================
#else

void beginClient() {
  Serial.println(F("Subscribing to NTRIP caster via cellular..."));
  bool tcpConnected = false;
  unsigned long lastDiag_ms = 0;

  while (ntripRunning) {

    // Button check
    int reading = digitalRead(BUTTON_PIN);
    if (reading != lastButtonState) lastDebounceTime = millis();
    if ((millis() - lastDebounceTime) > debounceDelay) {
      if (reading != currentButtonState) {
        currentButtonState = reading;
        if (currentButtonState == LOW) {
          Serial.println(F("Button — stopping NTRIP"));
          ntripRunning = false; digitalWrite(LED_PIN, LOW); break;
        }
      }
    }
    lastButtonState = reading;

    // 1 Hz diagnostics
    if (millis() - lastDiag_ms > 1000) {
      lastDiag_ms = millis();
      myGNSS.getPVT();
      Serial.print(F("[diag] carrier="));  Serial.print(myGNSS.getCarrierSolutionType());
      Serial.print(F(" siv="));            Serial.print(myGNSS.getSIV());
      Serial.print(F(" rtcmTotal="));      Serial.print(rtcmTotalBytes);
      Serial.print(F(" reconnects="));     Serial.println(reconnectCount);
    }

    // Connect / reconnect
    if (!tcpConnected || !modem.TCPconnected()) {
      if (tcpConnected) {
        modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
        tcpConnected = false; delay(1000);
      }

      if (!ensureGPRS()) { delay(5000); continue; }

      reconnectCount++;
      float elapsedMin = (millis() - sessionStart_ms) / 60000.0;
      Serial.print(F("TCP connect attempt #")); Serial.print(reconnectCount);
      Serial.print(F(" at ")); Serial.print(elapsedMin, 1); Serial.println(F(" min"));
      Serial.print(F("Connecting to ")); Serial.print(casterHost);
      Serial.print(F(":")); Serial.println(casterPort);

      if (!modem.TCPconnect((char*)casterHost, casterPort)) {
        Serial.println(F("TCP connect failed — retrying in 2 s"));
        delay(2000); continue;
      }
      delay(500);

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
        "GET /%s HTTP/1.0\r\nUser-Agent: NTRIP ESP32 Client v1.0\r\n%s\r\n",
        mountPoint, credentials);

      Serial.println(F("Sending NTRIP request..."));
      if (!modem.TCPsend((char*)serverRequest, strlen(serverRequest))) {
        Serial.println(F("Send failed — retrying in 2 s"));
        modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
        delay(2000); continue;
      }

      char headerBuf[512] = {0};
      int  headerLen = 0; bool headerDone = false;
      unsigned long headerTimeout = millis();

      while (!headerDone && millis() - headerTimeout < 5000) {
        uint16_t avail = modem.TCPavailable();
        if (avail > 0) {
          uint8_t chunk[128];
          uint16_t got = modem.TCPread(chunk, min((uint16_t)sizeof(chunk), avail));
          for (int i = 0; i < (int)got && !headerDone; i++) {
            if (headerLen < (int)sizeof(headerBuf) - 1)
              headerBuf[headerLen++] = (char)chunk[i];
            if (headerLen >= 4 &&
                headerBuf[headerLen-4] == '\r' && headerBuf[headerLen-3] == '\n' &&
                headerBuf[headerLen-2] == '\r' && headerBuf[headerLen-1] == '\n') {
              headerDone = true;
              int trailing = (int)got - (i + 1);
              if (trailing > 0) { gpsSerial.write(chunk + i + 1, trailing); rtcmTotalBytes += trailing; }
            }
          }
        }
        delay(10);
      }

      Serial.print(F("Caster response: ")); Serial.println(headerBuf);

      if (strstr(headerBuf, "401") != NULL) {
        Serial.println(F("401 Unauthorized — check casterUser/casterUserPW in secrets.h"));
        ntripRunning = false; digitalWrite(LED_PIN, LOW); return;
      }
      if (strstr(headerBuf, "200") == NULL && strstr(headerBuf, "ICY") == NULL) {
        Serial.println(F("NTRIP connection failed — retrying in 2 s"));
        modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
        delay(2000); continue;
      }

      Serial.println(F("NTRIP connected via cellular!"));
      tcpConnected = true;
      lastReceivedRTCM_ms = millis();
    }

    // Read RTCM and push to ZED-F9P
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

    // [TELEMETRY] Close NTRIP, post snapshot, let loop reconnect.
    if (tcpConnected && millis() - lastTelemetry_ms > telemetryInterval_ms) {
      Serial.println(F("[telemetry] Pausing NTRIP to post telemetry..."));
      modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
      tcpConnected = false;
      postTelemetry();
      lastTelemetry_ms = millis();
      // outer loop reconnects NTRIP on the next iteration
    }

    if (millis() - lastReceivedRTCM_ms > maxTimeBeforeHangup_ms) {
      Serial.println(F("RTCM timeout — reconnecting"));
      modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
      tcpConnected = false;
      lastReceivedRTCM_ms = millis(); delay(1000); continue;
    }

    delay(10);
  }

  if (tcpConnected) modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
  Serial.println(F("NTRIP stopped"));
  ntripRunning = false; digitalWrite(LED_PIN, LOW);
}

#endif  // USE_POLARIS_CELLULAR
