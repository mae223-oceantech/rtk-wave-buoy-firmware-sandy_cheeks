/*
  ESP32 WiFi/Cellular NTRIP Client — RTK Wave Buoy
  Based on esp32_polaris_wifi.ino (WiFi/Polaris path)
  and buoy_combo.ino (cellular/SIM7000 path)

  Three operating modes, selected by the defines below:

    1. WiFi + Polaris (default — both defines commented out)
       HTTP/1.1 chunked stream, GGA sent every 10 s for VRS placement.

    2. Cellular + SIO / fixed-base caster (#define USE_CELLULAR only)
       HTTP/1.0 plain stream, no GGA required, no chunked decoder.
       Works with any NTRIP/1.0 caster (SIO, RTK2Go, etc.).

    3. Cellular + Polaris (#define USE_CELLULAR + #define USE_POLARIS_CELLULAR)
       HTTP/1.1 chunked stream over SIM7000 TCP, GGA sent every 10 s.
       The challenge: Polaris is a VRS — it synthesizes a virtual base
       station near your rover and needs your GPS position to do it.
       GGA is sent upstream over the same TCP connection as RTCM comes down.
       The chunked decoder uses a buffered byte reader on top of TCPread().

  Hardware — same for all modes:
    ESP32 GPIO 27 (RX2) → ZED-F9P TX1/MISO   (F9P UART1)
    ESP32 GPIO 12 (TX2) → ZED-F9P RX1/MOSI   (F9P UART1)
    Button: GPIO 0 — press to start/stop NTRIP
    LED:    GPIO 13 — blink = ready, solid = NTRIP active

  Additional wiring for cellular modes:
    ESP32 GPIO 16 (RX1) → SIM7000 TX
    ESP32 GPIO 17 (TX1) → SIM7000 RX
    ESP32 GPIO 18       → SIM7000 PWRKEY
    ESP32 GPIO 5        → SIM7000 RST
    LiPo battery connected to SIM7000 JST connector (required)
*/

// ============================================================
// MODE TOGGLES
// Step 1: uncomment for cellular transport (comment out = WiFi)
// Step 2: if cellular AND using Polaris, also uncomment USE_POLARIS_CELLULAR
// ============================================================
// #define USE_CELLULAR
// #define USE_POLARIS_CELLULAR   // only meaningful when USE_CELLULAR is defined

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

#endif  // USE_CELLULAR

// Chunked-decoder diagnostics and GGA interval are needed for WiFi Polaris
// and for cellular Polaris, but not for cellular SIO.
#if !defined(USE_CELLULAR) || defined(USE_POLARIS_CELLULAR)
int desyncSuspectCount   = 0;
int trailingCRLFMismatch = 0;
// Polaris VRS needs GGA every 10–30 s so the synthesized base stays near the rover
const unsigned long ggaInterval_ms = 10000;
#endif

// ============================================================
// GPS — shared by all modes
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

#if defined(USE_CELLULAR) && defined(USE_POLARIS_CELLULAR)
  Serial.println(F("RTK Wave Buoy — Cellular/Polaris NTRIP Client"));
#elif defined(USE_CELLULAR)
  Serial.println(F("RTK Wave Buoy — Cellular/SIO NTRIP Client"));
#else
  Serial.println(F("RTK Wave Buoy — WiFi/Polaris NTRIP Client"));
#endif

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // --- ZED-F9P init (all modes) ---
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

  modem.setFunctionality(1);                  // AT+CFUN=1 — full RF on
  modem.setNetworkSettings(F(CELLULAR_APN));  // APN from secrets.h

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
// Main loop — button debounce + LED heartbeat (same for all modes)
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

  // Blink while idle — indicates transport connected and waiting
  if (!ntripRunning) {
    if (millis() - lastBlinkTime >= 1000) {
      lastBlinkTime = millis();
      ledBlinkState = !ledBlinkState;
      digitalWrite(LED_PIN, ledBlinkState);
    }
  }
}

// ============================================================
// buildGGA — NMEA GGA sentence from ZED-F9P position.
// Polaris VRS uses this to place the virtual base near the rover.
// quality=0 if no fix — Polaris accepts it and waits for a real position.
// ============================================================
String buildGGA() {
  // One getPVT() caches all NAV-PVT fields; subsequent getters use the cache
  // instead of issuing separate blocking UART polls that would delay RTCM injection.
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
// Helper — reconnect GPRS if signal was lost (cellular modes only)
// Returns true if GPRS is ready, false if it couldn't be restored.
// ============================================================
#ifdef USE_CELLULAR
bool ensureGPRS() {
  uint8_t netStat = modem.getNetworkStatus();
  if (gprsEnabled && (netStat == 1 || netStat == 5)) return true;

  Serial.println(F("GPRS lost — re-enabling..."));
  modem.enableGPRS(false);
  delay(1000);
  gprsEnabled = modem.enableGPRS(true);
  if (gprsEnabled) {
    Serial.println(F("GPRS restored."));
  } else {
    Serial.println(F("GPRS re-enable failed."));
  }
  return gprsEnabled;
}
#endif

// ============================================================
// MODE 3: Cellular + Polaris
// HTTP/1.1 chunked stream over SIM7000 TCP.
// GGA sent on connect and refreshed every ggaInterval_ms.
//
// Key challenge: modem.TCPread() returns byte batches, not a stream.
// The chunked decoder needs one byte at a time, so readByteCellular()
// maintains a local cache and refills it from TCPread() when empty.
// ============================================================
#if defined(USE_CELLULAR) && defined(USE_POLARIS_CELLULAR)

// Byte cache for readByteCellular(). File-scope so beginClient() can
// reset it on each reconnect, keeping the decoder state clean.
static uint8_t _cellBuf[256];
static int     _cellBufLen = 0;
static int     _cellBufIdx = 0;

// Read one byte from the modem TCP stream, blocking up to timeout_ms.
// Serves from the local cache; refills via TCPread() when empty.
// Returns -1 on timeout or disconnection.
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

void beginClient() {
  Serial.println(F("Subscribing to Polaris via cellular (HTTP/1.1 + chunked + GGA)..."));
  bool tcpConnected = false;
  unsigned long lastGGASent_ms = 0;
  unsigned long lastDiag_ms    = 0;

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
    if (!tcpConnected || !modem.TCPconnected()) {
      if (tcpConnected) {
        modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
        tcpConnected = false;
        _cellBufLen = _cellBufIdx = 0;   // flush decoder cache on disconnect
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
        delay(2000);
        continue;
      }
      delay(500);

      // HTTP/1.1 request — Polaris requires Ntrip-Version: Ntrip/2.0
      const int SERVER_BUFFER_SIZE = 512;
      char serverRequest[SERVER_BUFFER_SIZE];
      snprintf(serverRequest, SERVER_BUFFER_SIZE,
        "GET /%s HTTP/1.1\r\n"
        "Host: %s\r\n"
        "Ntrip-Version: Ntrip/2.0\r\n"
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
      strncat(serverRequest, credentials, SERVER_BUFFER_SIZE - strlen(serverRequest) - 1);
      strncat(serverRequest, "\r\n",     SERVER_BUFFER_SIZE - strlen(serverRequest) - 1);

      Serial.print(F("Request size: ")); Serial.print(strlen(serverRequest));
      Serial.print(F(" / ")); Serial.println(SERVER_BUFFER_SIZE);

      if (!modem.TCPsend((char*)serverRequest, strlen(serverRequest))) {
        Serial.println(F("Send failed — retrying in 2 s"));
        modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
        delay(2000);
        continue;
      }

      // Wait for response
      unsigned long timeout = millis();
      bool timedOut = false;
      while (modem.TCPavailable() == 0) {
        if (millis() - timeout > 5000) {
          Serial.println(F("Caster timed out — retrying in 1 s"));
          modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
          timedOut = true;
          break;
        }
        delay(10);
      }
      if (timedOut) { delay(1000); continue; }

      // Read HTTP response header — scan for \r\n\r\n.
      // Any bytes arriving after the header boundary in the same TCPread batch
      // are pre-loaded into _cellBuf so the chunked decoder finds them immediately.
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
                // Pre-load post-header bytes into the decoder cache
                int trailing = (int)got - (i + 1);
                if (trailing > 0 && trailing <= (int)sizeof(_cellBuf)) {
                  memcpy(_cellBuf, chunk + i + 1, trailing);
                  _cellBufLen = trailing;
                  _cellBufIdx = 0;
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
        ntripRunning = false;
        digitalWrite(LED_PIN, LOW);
        return;
      }

      if (strstr(headerBuf, "200") == NULL) {
        Serial.println(F("NTRIP connection failed — retrying in 2 s"));
        modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
        _cellBufLen = _cellBufIdx = 0;
        delay(2000);
        continue;
      }

      // Send initial GGA so Polaris places the VRS near the buoy
      String gga = buildGGA();
      modem.TCPsend((char*)gga.c_str(), gga.length());
      lastGGASent_ms = millis();
      Serial.print(F("Sent GGA: ")); Serial.print(gga);

      Serial.println(F("NTRIP connected via cellular/Polaris!"));
      tcpConnected = true;
      lastReceivedRTCM_ms = millis();
    }

    // --- Refresh GGA so Polaris VRS keeps tracking the rover ---
    if (tcpConnected && millis() - lastGGASent_ms > ggaInterval_ms) {
      String gga = buildGGA();
      modem.TCPsend((char*)gga.c_str(), gga.length());
      lastGGASent_ms = millis();
      Serial.println(F("GGA refreshed"));
    }

    // --- HTTP/1.1 chunked RTCM decoder using readByteCellular() ---
    // Trigger when either the modem buffer or the local cache has data.
    if (modem.TCPavailable() > 0 || _cellBufIdx < _cellBufLen) {
      long rtcmCount = 0;

      // Read hex chunk-size line (terminated by CRLF)
      char chunkSizeBuf[12];
      int  idx       = 0;
      bool sizeReadOk = true;
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
        tcpConnected = false;
        _cellBufLen = _cellBufIdx = 0;
      } else {
        long chunkSize = strtol(chunkSizeBuf, NULL, 16);

        if (chunkSize == 0) {
          Serial.println(F("[stream] chunkSize=0 — caster closed stream, dropping socket"));
          modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
          tcpConnected = false;
          _cellBufLen = _cellBufIdx = 0;
        } else if (chunkSize < 0 || chunkSize > 4096) {
          // Payload bytes mis-read as hex size → decoder desynced
          desyncSuspectCount++;
          Serial.print(F("[desync?] chunkSize=")); Serial.print(chunkSize);
          Serial.print(F(" hex='")); Serial.print(chunkSizeBuf);
          Serial.println(F("' — dropping socket"));
          modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
          tcpConnected = false;
          _cellBufLen = _cellBufIdx = 0;
        } else {
          // Consume exactly chunkSize bytes and forward to ZED-F9P
          uint8_t rtcmData[1024];
          long remaining = chunkSize;
          bool payloadOk = true;
          while (remaining > 0) {
            int want = remaining > (long)sizeof(rtcmData) ? (int)sizeof(rtcmData) : (int)remaining;
            int got  = 0;
            while (got < want) {
              int b = readByteCellular(5000);
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
            modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
            tcpConnected = false;
            _cellBufLen = _cellBufIdx = 0;
          } else {
            // Consume trailing CRLF after chunk payload
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
              tcpConnected = false;
              _cellBufLen = _cellBufIdx = 0;
            }
          }
        }
      }

      if (rtcmCount > 0) {
        lastReceivedRTCM_ms = millis();
        Serial.print(F("RTCM → ZED: ")); Serial.println(rtcmCount);
      }
    }

    // --- RTCM silence timeout → drop socket and reconnect ---
    if (millis() - lastReceivedRTCM_ms > maxTimeBeforeHangup_ms) {
      Serial.println(F("RTCM timeout — reconnecting"));
      modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
      tcpConnected = false;
      _cellBufLen = _cellBufIdx = 0;
      lastReceivedRTCM_ms = millis();
      delay(1000);
      continue;
    }

    delay(10);
  }

  if (tcpConnected) modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
  Serial.println(F("NTRIP stopped"));
  ntripRunning = false;
  digitalWrite(LED_PIN, LOW);
}

// ============================================================
// MODE 2: Cellular + SIO / fixed-base caster
// HTTP/1.0 plain stream — no chunked decoder, no GGA required.
// ============================================================
#elif defined(USE_CELLULAR)

void beginClient() {
  Serial.println(F("Subscribing to NTRIP caster via cellular..."));
  bool tcpConnected = false;
  unsigned long lastDiag_ms = 0;

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
        modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
        tcpConnected = false;
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
        delay(2000);
        continue;
      }
      delay(500);

      // HTTP/1.0 NTRIP request — no chunked encoding
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
        Serial.println(F("Send failed — retrying in 2 s"));
        modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
        delay(2000);
        continue;
      }

      // Read HTTP response header — scan for \r\n\r\n.
      // Bytes after the header boundary are forwarded to ZED-F9P immediately
      // (the F9P syncs to the RTCM3 0xD3 preamble and ignores leading text).
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
            if (headerLen < (int)sizeof(headerBuf) - 1)
              headerBuf[headerLen++] = (char)chunk[i];
            if (headerLen >= 4 &&
                headerBuf[headerLen-4] == '\r' && headerBuf[headerLen-3] == '\n' &&
                headerBuf[headerLen-2] == '\r' && headerBuf[headerLen-1] == '\n') {
              headerDone = true;
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

  if (tcpConnected) modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
  Serial.println(F("NTRIP stopped"));
  ntripRunning = false;
  digitalWrite(LED_PIN, LOW);
}

// ============================================================
// MODE 1: WiFi + Polaris
// HTTP/1.1 chunked stream, GGA sent on connect and every ggaInterval_ms.
// ============================================================
#else

// Blocking single-byte read with timeout. Required for the chunked decoder:
// chunk-size lines and payloads can straddle TCP packet boundaries under
// WiFi jitter, and a non-blocking read on a drained buffer desyncs the
// decoder permanently.
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

    // --- HTTP/1.1 chunked RTCM decoder ---
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
          Serial.println(F("[stream] chunkSize=0 — dropping socket"));
          ntripClient.stop();
        } else if (chunkSize < 0 || chunkSize > 4096) {
          desyncSuspectCount++;
          Serial.print(F("[desync?] chunkSize=")); Serial.print(chunkSize);
          Serial.print(F(" hex='")); Serial.print(chunkSizeBuf);
          Serial.println(F("' — dropping socket"));
          ntripClient.stop();
        } else {
          // Consume exactly chunkSize bytes in 1 KB passes
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

#endif  // mode selection
