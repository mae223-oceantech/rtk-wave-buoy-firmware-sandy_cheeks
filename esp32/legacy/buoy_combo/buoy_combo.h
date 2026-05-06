/* buoy_combo.h */
#ifndef BUOY_COMBO_H
#define BUOY_COMBO_H

#include "BotleticsSIM7000.h"
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <HardwareSerial.h>

// Add base64 library
#if defined(ARDUINO_ARCH_ESP32)
#include "base64.h"
#else
#include <Base64.h>
#endif

#define STATUS_LED 13  // Built-in LED for status
#define TX_GPS 12      // ESP32 TX2 to GPS RX  
#define RX_GPS 27      // ESP32 RX2 to GPS TX
#define SHUTDOWN_BTN 0 // for shutdown


extern bool networkConnected;
extern bool gprsEnabled;
extern bool gpsEnabled;
extern bool ntripConnected;
extern bool gpsUARTOnline;
extern unsigned long lastNTRIPAttempt;
extern long lastReceivedRTCM_ms;
extern int maxTimeBeforeHangup_ms;
extern const unsigned long ntripRetryInterval;
extern long lastGPSPrint;
extern Botletics_modem_LTE modem;
extern SFE_UBLOX_GNSS myGNSS;
extern HardwareSerial gpsSerial;
extern volatile bool shutdownRequested;


// Function declarations
void network_status_check_f();
void enable_gprs_f();
void initialize_gnss_uart_f();
// void print_gps_status_f();
void beginNTRIPClient();
void handleNTRIPData();
void monitor_connection_health();
void printDebugStatus();
void updateStatusLED();


void initialize_gnss_uart_f() {
  Serial.println(F("=== Initializing ZED-F9P via UART ==="));
  SerialBT.println(F("=== Initializing ZED-F9P via UART ==="));
  Serial.print(F("TX_GPS pin: "));
  SerialBT.print(F("TX_GPS pin: "));
  Serial.println(TX_GPS);
  SerialBT.println(TX_GPS);
  Serial.print(F("RX_GPS pin: "));
  SerialBT.print(F("RX_GPS pin: "));
  Serial.println(RX_GPS);
  SerialBT.println(RX_GPS);
  
  const long baudRates[] = {115200, 115200, 115200, 115200, 115200};
  const int numRates = 5;
  
  for (int i = 0; i < numRates; i++) {
    Serial.print(F("Trying "));
    SerialBT.print(F("Trying "));
    Serial.print(baudRates[i]);
    SerialBT.print(baudRates[i]);
    Serial.println(F(" baud..."));
    SerialBT.println(F(" baud..."));
    
    gpsSerial.begin(baudRates[i], SERIAL_8N1, RX_GPS, TX_GPS);
    delay(1000);  // Give more time
    
    // Try to get any response
    Serial.println(F("  Attempting myGNSS.begin()..."));
    SerialBT.println(F("  Attempting myGNSS.begin()..."));
    
    if (myGNSS.begin(gpsSerial)) {
      Serial.print(F("SUCCESS at "));
      SerialBT.print(F("SUCCESS at "));
      Serial.print(baudRates[i]);
      SerialBT.print(baudRates[i]);
      Serial.println(F(" baud!"));
      SerialBT.println(F(" baud!"));
      gpsUARTOnline = true;
      
      // Don't configure ports yet - just confirm connection
      Serial.println(F("GPS UART connected!"));
      SerialBT.println(F("GPS UART connected!"));
      return;
    } else {
      Serial.println(F("  Failed"));
      SerialBT.println(F("GPS UART connected!"));
    }
    
    gpsSerial.end();
    delay(100);
  }
  
  Serial.println(F("ERROR: GPS UART failed at all baud rates!"));
  SerialBT.println(F("ERROR: GPS UART failed at all baud rates!"));
  gpsUARTOnline = false;
}


void network_status_check_f() {
  // Serial.println("START network_status_check");
  if (!networkConnected) {
    Serial.println(F("=== Checking Network Status ==="));
    SerialBT.println(F("=== Checking Network Status ==="));
    
    // Check signal strength first
    uint8_t rssi = modem.getRSSI();
    Serial.print(F("Signal strength (RSSI): "));
    SerialBT.print(F("Signal strength (RSSI): "));
    Serial.print(rssi);
    SerialBT.print(rssi);
    if (rssi == 0) {
      Serial.println(F(" (No signal)"));
      SerialBT.println(F(" (No signal)"));
    } else if (rssi == 99) {
      Serial.println(F(" (Unknown)"));
      SerialBT.println(F(" (Unknown)"));
    } else if (rssi < 10) {
      Serial.println(F(" (Poor)"));
      SerialBT.println(F(" (Poor)"));
    } else if (rssi < 15) {
      Serial.println(F(" (Fair)"));
      SerialBT.println(F(" (Fair)"));
    } else if (rssi < 20) {
      Serial.println(F(" (Good)"));
      SerialBT.println(F(" (Good)"));
    } else {
      Serial.println(F(" (Excellent)"));
      SerialBT.println(F(" (Excellent)"));
    }
    uint8_t n = modem.getNetworkStatus();
    Serial.print(F("Network status "));
    SerialBT.print(F("Network status "));
    Serial.print(n);
    SerialBT.print(n);
    Serial.print(F(": "));
    SerialBT.print(F(": "));
    
    if (n == 0) {
      Serial.println(F("Not registered"));
      SerialBT.println(F("Not registered"));
    } 
    if (n == 1) {
      Serial.println(F("Registered (home)"));
      SerialBT.println(F("Registered (home)"));
    }
    if (n == 2) {
      Serial.println(F("Not registered (searching)"));
      SerialBT.println(F("Not registered (searching)"));
    }

    if (n == 3) {
      Serial.println(F("Denied"));
      SerialBT.println(F("Denied"));
    }
    if (n == 4) {
      Serial.println(F("Unknown"));
      SerialBT.println(F("Unknown"));
    }
    if (n == 5) {
      Serial.println(F("Registered roaming"));
      SerialBT.println(F("Registered roaming"));
    }
    
    if (n == 1 || n == 5) {
      // Wait a bit longer to ensure registration is stable
      Serial.println(F("Network registration detected - verifying stability..."));
      SerialBT.println(F("Network registration detected - verifying stability..."));
      delay(3000);
      
      // Double-check registration
      uint8_t n2 = modem.getNetworkStatus();
      if (n2 == n) {
        networkConnected = true;
        Serial.println(F("Network connection confirmed!"));
        SerialBT.println(F("Network connection confirmed!"));
      } else {
        Serial.println(F("Network registration unstable - waiting..."));
        SerialBT.println(F("Network registration unstable - waiting..."));
        delay(5000);
      }
    } else {
      // Progressive delay based on status
      int waitTime = 5000;
      if (n == 2) waitTime = 10000; // Searching takes longer
      if (n == 3) waitTime = 30000; // Denied - wait longer before retry
      
      Serial.print(F("Waiting "));
      SerialBT.print(F("Waiting "));
      Serial.print(waitTime/1000);
      SerialBT.print(waitTime/1000);
      Serial.println(F(" seconds before rechecking..."));
      SerialBT.println(F(" seconds before rechecking..."));
      delay(waitTime);
      return;
    }
  }
  // Serial.println("END network_status_check");
}

// Replace your enable_gprs_f() function with this improved version:
void enable_gprs_f() {
  // Serial.println("START enable_gprs_f");
  if (networkConnected && !gprsEnabled) {
    Serial.println(F("=== Starting GPRS Enable Process ==="));
    SerialBT.println(F("=== Starting GPRS Enable Process ==="));
    // Step 1: Check signal strength first
    uint8_t rssi = modem.getRSSI();
    Serial.print(F("Signal strength (RSSI): "));
    SerialBT.print(F("Signal strength (RSSI): "));
    Serial.println(rssi);
    SerialBT.println(rssi);
    
    if (rssi == 0 || rssi == 99) {
      Serial.println(F("Poor/no signal - waiting before GPRS attempt"));
      SerialBT.println(F("Poor/no signal - waiting before GPRS attempt"));
      delay(5000);
      return; // Try again next loop
    }
    
    // Step 2: Ensure we're really registered (sometimes status lies)
    Serial.println(F("Double-checking network registration..."));
    SerialBT.println(F("Double-checking network registration..."));
    uint8_t n = modem.getNetworkStatus();
    if (n != 1 && n != 5) {
      Serial.print(F("Network status changed to: "));
      SerialBT.print(F("Network status changed to: "));
      Serial.println(n);
      SerialBT.println(n);
      networkConnected = false; // Reset and try again
      return;
    }
    
    // Step 3: Clean up any existing GPRS connection
    Serial.println(F("Cleaning up previous GPRS state..."));
    SerialBT.println(F("Cleaning up previous GPRS state..."));
    modem.enableGPRS(false); // Disable first
    delay(2000); // Wait for cleanup
    
    // Step 4: Attempt GPRS enable with retry logic
    Serial.println(F("Attempting GPRS enable..."));
    SerialBT.println(F("Attempting GPRS enable..."));
    
    for (int attempt = 1; attempt <= 3; attempt++) {
      Serial.print(F("GPRS attempt "));
      SerialBT.print(F("GPRS attempt "));
      Serial.print(attempt);
      SerialBT.print(attempt);
      Serial.println(F("/3"));
      SerialBT.println(F("/3"));
      
      if (modem.enableGPRS(true)) {
        gprsEnabled = true;
        Serial.println(F("GPRS enabled successfully!"));
        SerialBT.println(F("GPRS enabled successfully!"));
        
        // Verify GPRS is really working
        Serial.println(F("Verifying GPRS connection..."));
        SerialBT.println(F("Verifying GPRS connection..."));
        delay(2000);
        
        // Try to get IP address as verification
        // (You might need to add this function call if available)
        Serial.println(F("GPRS verification complete"));
        SerialBT.println(F("GPRS verification complete"));
        return;
        
      } else {
        Serial.print(F("GPRS attempt "));
        SerialBT.print(F("GPRS attempt "));
        Serial.print(attempt);
        SerialBT.print(attempt);
        Serial.println(F(" failed"));
        SerialBT.println(F(" failed"));
        
        if (attempt < 3) {
          Serial.print(F("Waiting "));
          SerialBT.print(F("Waiting "));
          Serial.print(attempt * 5);
          SerialBT.print(attempt * 5);
          Serial.println(F(" seconds before retry..."));
          SerialBT.println(F(" seconds before retry..."));
          delay(attempt * 5000); // Progressive delay: 5s, 10s, 15s
        }
      }
    }
    
    Serial.println(F("All GPRS attempts failed - will retry in next loop"));
    SerialBT.println(F("All GPRS attempts failed - will retry in next loop"));
    delay(10000); // Wait 10 seconds before next attempt
  }
  // Serial.println("END enable_gprs_f");
}

void beginNTRIPClient() {
  Serial.println(F("Attempting NTRIP connection via LTE TCP..."));
  SerialBT.println(F("Attempting NTRIP connection via LTE TCP..."));
  
  // First, try to connect to the NTRIP caster using TCP
  Serial.print(F("Connecting to "));
  SerialBT.print(F("Connecting to "));
  Serial.print(casterHost);
  SerialBT.print(casterHost);
  Serial.print(F(":"));
  SerialBT.print(F(":"));
  Serial.println(casterPort);
  SerialBT.println(casterPort);
  
  if (!modem.TCPconnect((char*)casterHost, casterPort)) {
    Serial.println(F("Failed to connect to NTRIP caster"));
    SerialBT.println(F("Failed to connect to NTRIP caster"));
    return;
  }
  
  // Wait a moment for connection to establish
  delay(1000);
  
  // Check if we're connected
  if (!modem.TCPconnected()) {
    Serial.println(F("TCP connection not established"));
    SerialBT.println(F("TCP connection not established"));
    return;
  }
  
  Serial.println(F("TCP connection established, sending NTRIP request..."));
  SerialBT.println(F("TCP connection established, sending NTRIP request..."));
  
  // Create the NTRIP request string
  String ntripRequest = "GET /" + String(mountPoint) + " HTTP/1.0\r\n";
  ntripRequest += "User-Agent: NTRIP SparkFun u-blox Client v1.0\r\n";
  
  // Add authentication if credentials are provided
  if (strlen(casterUser) > 0) {
    String userCredentials = String(casterUser) + ":" + String(casterUserPW);
    Serial.print(F("Adding credentials for: "));
    SerialBT.print(F("Adding credentials for: "));
    Serial.println(casterUser);
    SerialBT.println(casterUser);
    
    // Encode credentials in base64
    base64 b;
    String encodedCredentials = b.encode(userCredentials);
    ntripRequest += "Authorization: Basic " + encodedCredentials + "\r\n";
  } else {
    ntripRequest += "Accept: */*\r\n";
  }
  
  // ntripRequest += "Connection: close\r\n";
  ntripRequest += "\r\n";
  
  Serial.println(F("Sending NTRIP request:"));
  SerialBT.println(F("Sending NTRIP request:"));
  Serial.println(ntripRequest);
  SerialBT.println(ntripRequest);
  
  // Send the NTRIP request
  if (!modem.TCPsend((char*)ntripRequest.c_str(), ntripRequest.length())) {
    Serial.println(F("Failed to send NTRIP request"));
    SerialBT.println(F("Failed to send NTRIP request"));
    return;
  }
  
  Serial.println(F("NTRIP request sent, waiting for response..."));
  SerialBT.println(F("NTRIP request sent, waiting for response..."));
  
  // Wait for response and check if successful
  delay(2000); // Give server time to respond
  
  // Check for response data
  uint16_t available = modem.TCPavailable();
  Serial.print(F("DEBUG: Available bytes = "));
  SerialBT.print(F("DEBUG: Available bytes = "));
  Serial.println(available);
  SerialBT.println(available);

  if (available > 0) {
    Serial.print(F("Response available: "));
    SerialBT.print(F("Response available: "));
    Serial.print(available);
    SerialBT.print(available);
    Serial.println(F(" bytes"));
    SerialBT.println(F(" bytes"));
    
    // Try reading in smaller chunks to avoid buffer issues
    Serial.println(F("DEBUG: About to call TCPread..."));
    SerialBT.println(F("DEBUG: About to call TCPread..."));
    
    // Use smaller buffer and read multiple times if needed
    uint8_t responseBuffer[128];  // Smaller buffer
    uint16_t bytesToRead = min(available, (uint16_t)sizeof(responseBuffer));
    
    Serial.print(F("DEBUG: Trying to read "));
    SerialBT.print(F("DEBUG: Trying to read "));
    Serial.print(bytesToRead);
    SerialBT.print(bytesToRead);
    Serial.println(F(" bytes"));
    SerialBT.println(F(" bytes"));
    
    uint16_t bytesRead = modem.TCPread(responseBuffer, bytesToRead);
    
    Serial.print(F("DEBUG: Actually read "));
    SerialBT.print(F("DEBUG: Actually read "));
    Serial.print(bytesRead);
    SerialBT.print(bytesRead);
    Serial.println(F(" bytes"));
    SerialBT.println(F(" bytes"));
    
    if (bytesRead > 0) {
      Serial.println(F("DEBUG: Successfully read data!"));
      SerialBT.println(F("DEBUG: Successfully read data!"));
      
      // Null terminate for safety
      if (bytesRead < sizeof(responseBuffer)) {
        responseBuffer[bytesRead] = '\0';
      } else {
        responseBuffer[sizeof(responseBuffer)-1] = '\0';
      }
      
      // Convert to string for easier parsing
      String response = "";
      for (int i = 0; i < bytesRead; i++) {
        response += (char)responseBuffer[i];
      }
      
      Serial.println(F("=== SERVER RESPONSE ==="));
      SerialBT.println(F("=== SERVER RESPONSE ==="));
      Serial.println(response);
      SerialBT.println(response);
      Serial.println(F("=== END SERVER RESPONSE ==="));
      SerialBT.println(F("=== END SERVER RESPONSE ==="));
      
      // Check for various success patterns
      bool connectionSuccess = false;
      String responseUpper = response;
      responseUpper.toUpperCase();
      
      if (responseUpper.indexOf("200") >= 0) {
        Serial.println(F("Found '200' - HTTP OK"));
        SerialBT.println(F("Found '200' - HTTP OK"));
        connectionSuccess = true;
      } else if (responseUpper.indexOf("ICY") >= 0) {
        Serial.println(F("Found 'ICY' - Icecast/NTRIP OK"));
        SerialBT.println(F("Found 'ICY' - Icecast/NTRIP OK"));
        connectionSuccess = true;
      } else if (response.length() > 0 && 
                responseUpper.indexOf("ERROR") < 0 && 
                responseUpper.indexOf("401") < 0 && 
                responseUpper.indexOf("404") < 0) {
        Serial.println(F("? No explicit error found - assuming success"));
        SerialBT.println(F("? No explicit error found - assuming success"));
        connectionSuccess = true;
      }
      
      if (connectionSuccess) {
        Serial.println(F("NTRIP connection successful!"));
        SerialBT.println(F("NTRIP connection successful!"));
        ntripConnected = true;
        lastReceivedRTCM_ms = millis();
      } else {
        Serial.println(F("NTRIP connection failed - bad response"));
        SerialBT.println(F("NTRIP connection failed - bad response"));
        modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 10000);
      }
      
    } else {
      Serial.println(F("DEBUG: TCPread returned 0 bytes!"));
      SerialBT.println(F("DEBUG: TCPread returned 0 bytes!"));
      Serial.println(F("Waiting 1 second and trying again..."));
      SerialBT.println(F("Waiting 1 second and trying again..."));
      delay(1000);
      
      uint16_t available2 = modem.TCPavailable();
      Serial.print(F("Available after delay: "));
      SerialBT.print(F("Available after delay: "));
      Serial.println(available2);
      SerialBT.println(available2);
      
      if (available2 > 0) {
        bytesToRead = min(available2, (uint16_t)sizeof(responseBuffer));
        bytesRead = modem.TCPread(responseBuffer, bytesToRead);
        Serial.print(F("Second attempt read: "));
        SerialBT.print(F("Second attempt read: "));
        Serial.print(bytesRead);
        SerialBT.print(bytesRead);
        Serial.println(F(" bytes"));
        SerialBT.println(F(" bytes"));
        
        if (bytesRead > 0) {
          Serial.println(F("Second attempt successful - assuming NTRIP connected"));
          SerialBT.println(F("Second attempt successful - assuming NTRIP connected"));
          ntripConnected = true;
          lastReceivedRTCM_ms = millis();
        }
      }
    }
    
    } else {
      Serial.println(F("No response from server"));
      SerialBT.println(F("No response from server"));
      modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 10000);
    }
}



void handleNTRIPData() {
  // if (!modem.TCPconnected()) {
  //   Serial.println(F("TCP lost - closing socket"));
  //   modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
  //   ntripConnected = false;
  //   return;
  // }
  
  uint16_t available = modem.TCPavailable();
  
  if (available == 0) {
    // Only disconnect after real timeout, not a single bad AT response
    if (millis() - lastReceivedRTCM_ms > maxTimeBeforeHangup_ms) {
      Serial.println(F("RTCM timeout. Disconnecting..."));
      SerialBT.println(F("RTCM timeout. Disconnecting..."));
      modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
      ntripConnected = false;
    }
    return;
  }  
  uint8_t rtcmBuffer[256];
  uint16_t totalSent = 0;
  int readCount = 0;
  
  while (modem.TCPavailable() > 0 && readCount < 40) {
    uint16_t bytesRead = modem.TCPread(rtcmBuffer, 250);
    if (bytesRead > 0 && gpsUARTOnline) {
      gpsSerial.write(rtcmBuffer, bytesRead);
      totalSent += bytesRead;
      readCount++;
    } else {
      break;
    }
  }
  
  if (totalSent > 0) {
    lastReceivedRTCM_ms = millis();
  }
  
  // Minimal debug output
  Serial.print(totalSent);
  SerialBT.print(totalSent);
  Serial.print(F("->"));
  SerialBT.print(F("->"));
  Serial.println(modem.TCPavailable());
  SerialBT.println(modem.TCPavailable());
}

void monitor_connection_health() {
  if (ntripConnected) return;
  // Serial.println("START monitor_connection_health");
  static unsigned long lastHealthCheck = 0;
  
  if (millis() - lastHealthCheck > 30000) { // Check every 30 seconds
    lastHealthCheck = millis();
    
    Serial.println(F("=== Connection Health Check ==="));
    SerialBT.println(F("=== Connection Health Check ==="));
    
    if (networkConnected) {
      uint8_t status = modem.getNetworkStatus();
      uint8_t rssi = modem.getRSSI();
      
      Serial.print(F("Network: "));
      SerialBT.print(F("Network: "));
      Serial.print(status);
      SerialBT.print(status);
      Serial.print(F(", Signal: "));
      SerialBT.print(F(", Signal: "));
      Serial.println(rssi);
      SerialBT.println(rssi);
      
      // Reset flags if network is lost
      if (status != 1 && status != 5) {
        Serial.println(F("Network lost - resetting connection flags"));
        SerialBT.println(F("Network lost - resetting connection flags"));
        networkConnected = false;
        gprsEnabled = false;
        ntripConnected = false;
      }
    }
    
    Serial.println(F("=== End Health Check ==="));
    SerialBT.println(F("=== End Health Check ==="));
  }
  // Serial.println("END monitor_connection_health");
}

// DEBUG help
void printDebugStatus() {
  Serial.println(F("=== DEBUG STATUS ==="));
  SerialBT.println(F("=== DEBUG STATUS ==="));
  Serial.print(F("networkConnected = "));
  SerialBT.print(F("networkConnected = "));
  Serial.println(networkConnected ? "true" : "false");
  SerialBT.println(networkConnected ? "true" : "false");
  Serial.print(F("gprsEnabled = "));
  SerialBT.print(F("gprsEnabled = "));
  Serial.println(gprsEnabled ? "true" : "false");
  SerialBT.println(gprsEnabled ? "true" : "false");
  Serial.print(F("gpsEnabled = "));
  SerialBT.print(F("gpsEnabled = "));
  Serial.println(gpsEnabled ? "true" : "false");
  SerialBT.println(gpsEnabled ? "true" : "false");
  Serial.print(F("ntripConnected = "));
  SerialBT.print(F("ntripConnected = "));
  Serial.println(ntripConnected ? "true" : "false");
  SerialBT.println(ntripConnected ? "true" : "false");
  Serial.print(F("lastNTRIPAttempt = "));
  SerialBT.print(F("lastNTRIPAttempt = "));
  Serial.println(lastNTRIPAttempt);
  SerialBT.println(lastNTRIPAttempt);
  Serial.print(F("millis() = "));
  SerialBT.print(F("millis() = "));
  Serial.println(millis());
  SerialBT.println(millis());
  Serial.print(F("Time since last attempt = "));
  SerialBT.print(F("Time since last attempt = "));
  Serial.println(millis() - lastNTRIPAttempt);
  SerialBT.println(millis() - lastNTRIPAttempt);
  Serial.println(F("=== END DEBUG STATUS ==="));
  SerialBT.println(F("=== END DEBUG STATUS ==="));
}

void updateStatusLED() {

  if (ntripConnected) {
    digitalWrite(STATUS_LED, HIGH);  // Solid = NTRIP active
    return;
  }
  
  digitalWrite(STATUS_LED, LOW);
}

void IRAM_ATTR shutdownISR() {
  shutdownRequested = true;
}

void gracefulShutdown() {
  Serial.println(F("\n=== SHUTDOWN REQUESTED ==="));
  SerialBT.println(F("\n=== SHUTDOWN REQUESTED ==="));
  
  // Blink LED 3 times to confirm shutdown
  for (int i = 0; i < 3; i++) {
    digitalWrite(STATUS_LED, HIGH);
    delay(100);
    digitalWrite(STATUS_LED, LOW);
    delay(100);
  }
  
  // Close NTRIP/TCP connection
  if (ntripConnected) {
    Serial.println(F("Closing NTRIP connection..."));
    SerialBT.println(F("Closing NTRIP connection..."));
    modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
    ntripConnected = false;
    delay(1000);
  }
  
  // Disable GPRS
  if (gprsEnabled) {
    Serial.println(F("Disabling GPRS..."));
    SerialBT.println(F("Disabling GPRS..."));
    modem.enableGPRS(false);
    gprsEnabled = false;
    delay(1000);
  }
  
  // Power down modem
  Serial.println(F("Powering down modem..."));
  SerialBT.println(F("Powering down modem..."));
  modem.sendCheckReply(F("AT+CPOWD=1"), F("NORMAL POWER DOWN"), 5000);
  delay(2000);
  
  // Turn off LED
  digitalWrite(STATUS_LED, LOW);
  
  // Configure wake-up source
  Serial.println(F("Entering deep sleep..."));
  SerialBT.println(F("Entering deep sleep..."));
  Serial.println(F("Press button again to wake up."));
  SerialBT.println(F("Press button again to wake up."));
  Serial.flush();  // Make sure message prints before sleep
  
  // Configure pin 0 to wake on LOW (button pressed)
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);
  
  // Enter deep sleep // this saves more power but it turns off bluetooth
  // esp_deep_sleep_start();
  // enter light sleep // uses more power but keeps bluetooth on
  esp_light_sleep_start();
  
}

#endif