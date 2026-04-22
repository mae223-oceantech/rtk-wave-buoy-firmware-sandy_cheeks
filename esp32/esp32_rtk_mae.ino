/*

  ESP32 Wifi NTRIP Client & RTK GPS
  Based off of SparkFun example by Nathan Siedle 
  Modified by Amara Ihekwoeme

  Essentially, it connects to wifi network, which gives it the
  ability to pull RTCM correction data from an NTRIP caster
  (rtk2go.com), then pushes it to the u-blox ZED-F9P over UART
  to get centimeter level RTK positioning

  How?:
  The ESP32 acts as an NTRIP client which requests correction
  data from a caster and the ESP32 funnels that into the RTK GPS
  The caster being rtk2go.com. 

  Hardware:
    Sparkfun ESP32 Thing Plus + u-blox ZED-F9P

  Wiring (matches buoy_combo / polaris — avoids GPIO 12 strapping conflict
  because GPIO 12 is used as ESP32 TX, not RX, so F9P never drives it at boot):
    ESP32 GPIO 27 (RX2) to ZED-F9P TX2
    ESP32 GPIO 12 (TX2) to ZED-F9P RX2
    ESP32 GND to ZED-F9P GND

  ESP32 mechanics:
    Button (GPIO 0): hold to start NTRIP client, press again to stop
    LED (GPIO 13): blink = Wifi connected aka ready
                   solid on = NTRIP running
  
  Serial monitor: 115200 baud rate

  Befor flashing, make a secrets.h tab with these values
    const char ssid[] = "<wifi name>";
    const char password[] =  "<wifi password>";

    //RTK2Go works well and is free
    const char casterHost[] = "<ip address>"; 
    const uint16_t casterPort = <port>;
    const char casterUser[] = "<caster user name>";
    const char casterUserPW[] = "<caster password>";
    const char mountPoint[] = "<mount_point>";
*/

#include <WiFi.h>
#include "secrets.h"

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

//The ESP32 core has a built in base64 library but not every platform does
//We'll use an external lib if necessary.
#if defined(ARDUINO_ARCH_ESP32)
#include "base64.h" //Built-in ESP32 library
#else
#include <Base64.h> //nfriendly library from https://github.com/adamvr/arduino-base64, will work with any platform
#endif

//Button configuration
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
const int BUTTON_PIN = 0; // GPIO 0 is the button on SparkFun Thing Plus ESP32
const int LED_PIN = 13;
bool lastButtonState = HIGH;
bool currentButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
bool ntripRunning = false;
unsigned long lastBlinkTime = 0;
bool ledBlinkState = false;
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Global variables
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
long lastReceivedRTCM_ms = 0; //5 RTCM messages take approximately ~300ms to arrive at 115200bps
int maxTimeBeforeHangup_ms = 100000; //If we fail to get a complete RTCM frame after 10s, then disconnect from caster
int reconnectCount = 0;
unsigned long sessionStart_ms = 0;
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setup()
{
  Serial.begin(115200);
  Serial.println(F("NTRIP testing"));

  // Initialize button
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  // initioalizing led
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial2.begin(115200, SERIAL_8N1, 27, 12); // RX=GPIO27, TX=GPIO12 (matches buoy_combo)
  if (myGNSS.begin(Serial2) == false) {
    Serial.println("ZED-F9P not detected on UART2. Check wiring/baud.");
    while(1);
  }

  myGNSS.setPortInput(COM_PORT_UART1, COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3); //Be sure RTCM3 input is enabled. UBX + RTCM3 is not a valid state.
  myGNSS.setUART1Output(COM_TYPE_UBX); // Turn off NMEA noise on UART2

  myGNSS.setNavigationFrequency(10); //Set output in Hz.
  
  Serial.print(F("Connecting to local WiFi"));
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println();

  Serial.print(F("WiFi connected with IP: "));
  Serial.println(WiFi.localIP());
  sessionStart_ms = millis();

  Serial.println(F("Press the button (GPIO 0) to start/stop NTRIP Client."));
  Serial.println(F("LED will be ON when NTRIP is running, OFF when stopped."));
}

void loop()
{
  // Read button with debouncing
  int reading = digitalRead(BUTTON_PIN);
  
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != currentButtonState) {
      currentButtonState = reading;
      
      // Button pressed (LOW because of pull-up)
      if (currentButtonState == LOW) {
        if (!ntripRunning) {
          Serial.println(F("Button pressed - Starting NTRIP Client"));
          ntripRunning = true;
          ledBlinkState = false; // stop blink, go solid
          digitalWrite(LED_PIN, HIGH);
          beginClient();
        } else {
          Serial.println(F("Button pressed - Stopping NTRIP Client"));
          ntripRunning = false;
          digitalWrite(LED_PIN, LOW);
        }
      }
    }
  }
  
  lastButtonState = reading;
  
  if (!ntripRunning) {
    // slow heartbeat blink (~1Hz) to indicate wifi connected, ready to go
    if (millis() - lastBlinkTime >= 1000) {
      lastBlinkTime = millis();
      ledBlinkState = !ledBlinkState;
      digitalWrite(LED_PIN, ledBlinkState);
    }
  }
}


// Build NMEA GGA sentence from current ZED-F9P position.
// Sent to the Polaris caster so VRS knows where to compute corrections.
// If F9P has no fix yet, quality=0 is reported and Polaris will wait.
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

//Connect to NTRIP Caster, receive RTCM, and push to ZED module over I2C
void beginClient()
{
  WiFiClient ntripClient;
  long rtcmCount = 0;
  unsigned long lastGGASent_ms = 0;

  Serial.println(F("Subscribing to Caster. Press button to stop. LED ON = running"));

  while (ntripRunning)
  {
    // Check button state for stop command
    int reading = digitalRead(BUTTON_PIN);
    if (reading != lastButtonState) {
      lastDebounceTime = millis();
    }
    
    if ((millis() - lastDebounceTime) > debounceDelay) {
      if (reading != currentButtonState) {
        currentButtonState = reading;
        if (currentButtonState == LOW) {
          Serial.println(F("Button pressed - stopping NTRIP client"));
          ntripRunning = false;
          digitalWrite(LED_PIN, LOW);
          break;
        }
      }
    }
    lastButtonState = reading;

    //Connect if we are not already. Limit to 5s between attempts.
    if (ntripClient.connected() == false)
    {
      
      reconnectCount++;
      float elapsedMin = (millis() - sessionStart_ms) / 60000.0;
      Serial.print(F("NTRIP disconnect #"));
      Serial.print(reconnectCount);
      Serial.print(F(" at "));
      Serial.print(elapsedMin, 1);
      Serial.println(F(" min into session"));

      Serial.print(F("Opening socket to "));
      Serial.println(casterHost);

      if (ntripClient.connect(casterHost, casterPort) == false) //Attempt connection
      {
        Serial.println(F("Connection to caster failed"));
        ntripRunning = false;
        digitalWrite(LED_PIN, LOW);
        return;
      }
      else
      {
        Serial.print(F("Connected to "));
        Serial.print(casterHost);
        Serial.print(F(": "));
        Serial.println(casterPort);

        Serial.print(F("Requesting NTRIP Data from mount point "));
        Serial.println(mountPoint);

        const int SERVER_BUFFER_SIZE  = 512;
        char serverRequest[SERVER_BUFFER_SIZE];

        snprintf(serverRequest, SERVER_BUFFER_SIZE,
          "GET /%s HTTP/1.1\r\n"
          "Host: %s\r\n"
          "Ntrip-Version: Ntrip/2.0\r\n"
          "User-Agent: NTRIP SparkFun u-blox Client v1.0\r\n",
          mountPoint, casterHost);

        char credentials[512];
        if (strlen(casterUser) == 0)
        {
          strncpy(credentials, "Accept: */*\r\nConnection: close\r\n", sizeof(credentials));
        }
        else
        {
          //Pass base64 encoded user:pw
          char userCredentials[sizeof(casterUser) + sizeof(casterUserPW) + 1]; //The ':' takes up a spot
          snprintf(userCredentials, sizeof(userCredentials), "%s:%s", casterUser, casterUserPW);

          Serial.print(F("Sending credentials: "));
          Serial.println(userCredentials);

#if defined(ARDUINO_ARCH_ESP32)
          //Encode with ESP32 built-in library
          base64 b;
          String strEncodedCredentials = b.encode(userCredentials);
          char encodedCredentials[strEncodedCredentials.length() + 1];
          strEncodedCredentials.toCharArray(encodedCredentials, sizeof(encodedCredentials)); //Convert String to char array
          snprintf(credentials, sizeof(credentials), "Authorization: Basic %s\r\n", encodedCredentials);
#else
          //Encode with nfriendly library
          int encodedLen = base64_enc_len(strlen(userCredentials));
          char encodedCredentials[encodedLen]; //Create array large enough to house encoded data
          base64_encode(encodedCredentials, userCredentials, strlen(userCredentials)); //Note: Input array is consumed
#endif
        }
        strncat(serverRequest, credentials, SERVER_BUFFER_SIZE);
        strncat(serverRequest, "\r\n", SERVER_BUFFER_SIZE);

        Serial.print(F("serverRequest size: "));
        Serial.print(strlen(serverRequest));
        Serial.print(F(" of "));
        Serial.print(sizeof(serverRequest));
        Serial.println(F(" bytes available"));

        Serial.println(F("Sending server request:"));
        Serial.println(serverRequest);
        ntripClient.write(serverRequest, strlen(serverRequest));

        //Wait for response
        unsigned long timeout = millis();
        while (ntripClient.available() == 0)
        {
          if (millis() - timeout > 5000)
          {
            Serial.println(F("Caster timed out!"));
            ntripClient.stop();
            ntripRunning = false;
            digitalWrite(LED_PIN, LOW);
            return;
          }
          delay(10);
        }

        //Check reply
        bool connectionSuccess = false;
        char response[512];
        int responseSpot = 0;
        while (ntripClient.available())
        {
          if (responseSpot == sizeof(response) - 1) break;

          response[responseSpot++] = ntripClient.read();
          if (strstr(response, "200") > (char *)0) //Look for 'ICY 200 OK'
            connectionSuccess = true;
          if (strstr(response, "401") > (char *)0) //Look for '401 Unauthorized'
          {
            Serial.println(F("Hey - your credentials look bad! Check you caster username and password."));
            connectionSuccess = false;
          }
        }
        response[responseSpot] = '\0';

        Serial.print(F("Caster responded with: "));
        Serial.println(response);

        if (connectionSuccess == false)
        {
          Serial.print(F("Failed to connect to "));
          Serial.print(casterHost);
          Serial.print(F(": "));
          Serial.println(response);
          ntripRunning = false;
          digitalWrite(LED_PIN, LOW);
          return;
        }
        else
        {
          Serial.print(F("Connected to "));
          Serial.println(casterHost);
          lastReceivedRTCM_ms = millis(); //Reset timeout

          // Send initial GGA so Polaris VRS knows our position
          String gga = buildGGA();
          ntripClient.print(gga);
          lastGGASent_ms = millis();
          Serial.print(F("Sent initial GGA: "));
          Serial.print(gga);
        }
      } //End attempt to connect
    } //End connected == false

    if (ntripClient.connected() == true)
    {
      uint8_t rtcmData[512 * 4]; //Most incoming data is around 500 bytes but may be larger
      rtcmCount = 0;

      // Refresh GGA every 10s to keep Polaris VRS position current
      if (millis() - lastGGASent_ms > 10000)
      {
        String gga = buildGGA();
        ntripClient.print(gga);
        lastGGASent_ms = millis();
        Serial.println(F("GGA sent to caster"));
      }

      //Print any available RTCM data
      while (ntripClient.available())
      {
        // Read chunk size line (hex digits followed by \r\n)
        char chunkSizeBuf[10];
        int idx = 0;
        while (ntripClient.available()) {
          char c = ntripClient.read();
          if (c == '\n') break;
          if (c != '\r') chunkSizeBuf[idx++] = c;
        }
        chunkSizeBuf[idx] = '\0';
        int chunkSize = strtol(chunkSizeBuf, NULL, 16);
        if (chunkSize == 0) break; // last chunk

        // Read exactly chunkSize bytes of RTCM
        int bytesRead = 0;
        while (bytesRead < chunkSize && ntripClient.available()) {
          rtcmData[rtcmCount++] = ntripClient.read();
          bytesRead++;
          if (rtcmCount == sizeof(rtcmData)) break;
        }
        // Consume trailing \r\n after chunk data
        while (ntripClient.available()) {
          char c = ntripClient.read();
          if (c == '\n') break;
        }
        break; // process what we have, come back next loop iteration
      }

      if (rtcmCount > 0)
      {
        lastReceivedRTCM_ms = millis();

        //Push RTCM to GNSS module over I2C
        myGNSS.pushRawData(rtcmData, rtcmCount, false);
        Serial.print(F("RTCM pushed to ZED: "));
        Serial.println(rtcmCount);
      }
    }

    //Close socket if we don't have new data for 10s
    if (millis() - lastReceivedRTCM_ms > maxTimeBeforeHangup_ms)
    {
      Serial.println(F("RTCM timeout. Disconnecting..."));
      if (ntripClient.connected() == true)
        ntripClient.stop();
      ntripRunning = false;
      digitalWrite(LED_PIN, LOW);
      return;
    }

    delay(10);
  }

  Serial.println(F("NTRIP Client stopped"));
  Serial.println(F("Disconnecting..."));
  ntripClient.stop();
  ntripRunning = false;
  digitalWrite(LED_PIN, LOW);
}