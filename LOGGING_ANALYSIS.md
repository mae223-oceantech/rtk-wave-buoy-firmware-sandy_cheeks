# Logging Analysis: Accel and UBX in OpenLog_Artemis_GNSS_Logging_Modified

## Overview

The firmware logs two independent data streams to the SD card:

| Stream | File | Format |
|--------|------|--------|
| UBX/NMEA from ZED-F9P | `dataLog#####.ubx` | Binary UBX + optional NMEA |
| IMU data from ICM-20948 | `imuLog#####.csv` | CSV text |

Both streams share the same file number counter (`settings.nextDataLogNumber`), so matching file numbers (e.g. `dataLog00001.ubx` + `imuLog00001.csv`) correspond to the same logging session.

---

## UBX Logging

### 1. Buffer Setup — `Sensors.ino: beginSensors()`

```cpp
gpsSensor_ublox.setFileBufferSize(FILE_BUFFER_SIZE); // 32768 bytes
```

The SparkFun u-blox library allocates an internal ring buffer before `.begin()` is called. All incoming UBX/NMEA bytes from the ZED-F9P (via I2C) land in this buffer first.

### 2. Message Enable/Disable — `Sensors.ino: enableMessages()` / `disableMessages()`

`enableMessages()` is called once at startup (and again on file rotation). For each UBX message type it does three things:

```cpp
gpsSensor_ublox.setAutoRXMRAWXcallbackPtr(&callbackRXMRAWX, ...);   // register terminal-print callback
gpsSensor_ublox.logRXMRAWX(settings.sensor_uBlox.logUBXRXMRAWX > 0 ? true : false); // enable file buffering
gpsSensor_ublox.setAutoRXMRAWXrate(settings.sensor_uBlox.logUBXRXMRAWX, ...);       // set output rate
```

Key messages for RTK/RTKLIB post-processing:

| Message | Setting field | Purpose |
|---------|--------------|---------|
| RXM-RAWX | `logUBXRXMRAWX` | Raw pseudoranges/carrier phase — required for RTKLIB |
| RXM-SFRBX | `logUBXRXMSFRBX` | Navigation subframes — required for RTKLIB |
| NAV-PVT | `logUBXNAVPVT` | Position/velocity/time — also used for RTC sync |
| NAV-HPPOSLLH | `logUBXNAVHPPOSLLH` | High-precision lat/lon |
| NAV-RELPOSNED | `logUBXNAVRELPOSNED` | Relative position (RTK baseline) |

`disableMessages()` mirrors the same list, setting all rates to 0 and `logXxx(false)`. It is called before file rotation and before sleep.

### 3. Data Collection + Write Loop — `storeData.ino: storeData()`

Called every `loop()` iteration:

```cpp
void storeData(void)
{
  // 1. Poll the ZED-F9P and fill the library's ring buffer
  gpsSensor_ublox.checkUblox();
  gpsSensor_ublox.checkCallbacks();  // triggers terminal-print callbacks

  // 2. Drain the ring buffer to SD in 512-byte aligned chunks (up to 5120 bytes/call)
  while (fileBytesAvailable >= packetLength) {
    gpsSensor_ublox.extractFileBufferData(&myBuffer, bytesToWrite);
    gnssDataFile.write(myBuffer, bytesToWrite);
  }

  // 3. Sync the file to SD every second
  if (millis() > lastDataLogSyncTime + 1000)
    gnssDataFile.sync();

  // 4. Read IMU (see below)
  readIMUData();
}
```

Write guard at `storeData.ino:31`:
```cpp
if (settings.logData && settings.sensor_uBlox.log && online.microSD && online.dataLogging)
    gnssDataFile.write(myBuffer, bytesToWrite);
```

### 4. Terminal Callbacks — `callbacks.ino`

The callbacks registered in `enableMessages()` only print abbreviated labels to serial (e.g. `" RXM-RAWX"`). They do **not** write to SD — that is handled entirely by the library's internal buffer + `logXxx(true)`.

Exception: `callbackNAVPVT` also syncs the Apollo3 RTC from the GNSS UTC time when `rtcNeedsSync` is set.

### 5. File Lifecycle

| Event | Location | Action |
|-------|----------|--------|
| Startup | `OpenLog_Artemis_GNSS_Logging_Modified.ino:272` | `beginDataLogging()` opens `dataLog#####.ubx` |
| New file requested (`f` key) | `Sensors.ino: openNewLogFile()` | Flush, sync, close, open next |
| Sleep | `lowerPower.ino:398` | `beginDataLogging()` reopens after wake |
| Stop logging / shutdown | `Sensors.ino: closeLogFile()` | `storeFinalData()` drains remainder, then close |

`storeFinalData()` (`storeData.ino:65`) does a final drain of any bytes left in the ring buffer before the file is closed.

---

## Accelerometer (IMU) Logging

### 1. Hardware Init — `OpenLog_Artemis_GNSS_Logging_Modified.ino:453 beginIMU()`

```cpp
void beginIMU() {
  if (settings.sensor_IMU.log == false) { disableIMU(); return; }
  imuPowerOn();           // assert PIN_IMU_POWER (pin 27)
  SPI.begin();
  enableCIPOpullUp();
  while (!initialized && attempts < 5) {
    myICM.begin(PIN_IMU_CHIP_SELECT, SPI);  // CS = pin 44
    if (myICM.status == ICM_20948_Stat_Ok) { online.imu = true; break; }
  }
}
```

The ICM-20948 is on the OLA's internal SPI bus (not Qwiic). If all 5 attempts fail, `online.imu` stays false and the IMU is powered off.

### 2. File Creation + CSV Header — `OpenLog_Artemis_GNSS_Logging_Modified.ino:687 beginDataLogging()`

```cpp
if (settings.sensor_IMU.log) {
  strcpy(imuDataFileName, findNextAvailableIMULog(settings.nextDataLogNumber, "imuLog"));
  imuDataFile.open(imuDataFileName, O_CREAT | O_APPEND | O_WRITE);

  if (imuDataFile.fileSize() == 0) {
    // Write header row based on which sensors are enabled:
    // "Timestamp,Sensor,[AccX,AccY,AccZ,][GyrX,GyrY,GyrZ,][MagX,MagY,MagZ,][Temp]"
    imuDataFile.print(header);
    imuDataFile.sync();
  }
}
```

The header columns are dynamically built from four boolean settings: `enableAccel`, `enableGyro`, `enableMag`, `enableTemp`.

### 3. Read Loop — `OpenLog_Artemis_GNSS_Logging_Modified.ino:522 readIMUData()`

Called at the end of every `storeData()` call, which is called every `loop()`. Gating:

```cpp
if (!online.imu || !settings.sensor_IMU.log) return;
if (millis() - lastIMUReadTime < settings.sensor_IMU.logRateMs) return;
if (!myICM.dataReady()) return;

myICM.getAGMT();  // fetch accel + gyro + mag + temp from ICM-20948
String imuDataString = createIMUDataString();
imuDataFile.print(imuDataString);
imuDataFile.sync();  // immediate flush after each sample
lastIMUReadTime = millis();
```

### 4. Data String Construction — `OpenLog_Artemis_GNSS_Logging_Modified.ino:594 createIMUDataString()`

```
<timestamp>,IMU,<AccX>,<AccY>,<AccZ>,<GyrX>,<GyrY>,<GyrZ>,<MagX>,<MagY>,<MagZ>,<Temp>\r\n
```

Accel values use a manual float-to-string conversion to work around an Apollo3 `String(float)` precision issue:

```cpp
float accX = myICM.accX();
int accX_int = (int)(accX * 100);          // e.g. -981 for -9.81 m/s²
dataString += String(accX_int / 100);      // integer part
dataString += ".";
if (abs(accX_int % 100) < 10) dataString += "0";  // zero-pad
dataString += String(abs(accX_int % 100)); // fractional part
```

Gyro, mag, and temp are cast directly to `int`.

### 5. Settings Persistence — `nvm.ino:229`

IMU settings are written to `OLA_GNSS_settings.cfg` and loaded back on boot:

```
IMU:log=1
IMU:enableAccel=1
IMU:enableGyro=1
IMU:enableMag=0
IMU:enableTemp=0
IMU:logRateMs=100
```

---

## Data Flow Diagram

```
loop()
  └─ storeData()
        │
        ├─ [UBX path]
        │    gpsSensor_ublox.checkUblox()        ← polls ZED-F9P via I2C
        │    gpsSensor_ublox.checkCallbacks()    ← fires terminal-print callbacks
        │    extractFileBufferData()             ← drains library ring buffer (32 KB)
        │    gnssDataFile.write()                ← writes to dataLog#####.ubx on SD
        │    gnssDataFile.sync()  (every 1s)
        │
        └─ [IMU path]
             readIMUData()
               myICM.getAGMT()                  ← reads ICM-20948 via SPI
               createIMUDataString()             ← builds CSV row
               imuDataFile.print()              ← writes to imuLog#####.csv on SD
               imuDataFile.sync()               ← immediate flush
```

---

## Key Design Notes

- **UBX buffering is library-managed**: the SparkFun u-blox library accumulates bytes from the ZED-F9P into its 32 KB ring buffer. The firmware drains it in `storeData()` in 512-byte aligned chunks. This decouples GNSS output rate from SD write timing.
- **IMU has no buffer**: each `readIMUData()` call does a single SPI transaction and writes one CSV row. Samples are rate-limited by `logRateMs` (default 100 ms = 10 Hz).
- **IMU `sync()` on every write** (`storeData.ino:577`) prioritizes data integrity at the cost of SD throughput. The UBX path only syncs every second.
- **ACKs/NACKs are stripped**: the u-blox library automatically excludes ACK/NACK packets from the file buffer, so only UBX data messages appear in the `.ubx` file.
- **Float precision workaround**: the Apollo3 Arduino core has a known issue with `String(float)`. Accel values use integer arithmetic to format to 2 decimal places (see `createIMUDataString()`).
