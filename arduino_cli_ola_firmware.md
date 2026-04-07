# Arduino CLI on Apple Silicon

Command-line Arduino workflow for this project — no IDE required. Covers setup, compiling, uploading, and reading serial output on macOS Apple Silicon (M1/M2/M3).

---

## Installation

```bash
brew install arduino-cli
arduino-cli version  # should show 1.4.1 or later
```

### Apple Silicon Note

Some Arduino toolchains (including SparkFun Apollo3) ship x86_64 binaries only. Install Rosetta 2 once to allow them to run:

```bash
softwareupdate --install-rosetta --agree-to-license
```

---

## Configuration

Initialize the config and add board manager URLs for both boards used in this project:

```bash
arduino-cli config init
arduino-cli config add board_manager.additional_urls https://raw.githubusercontent.com/sparkfun/Arduino_Apollo3/main/package_sparkfun_apollo3_index.json
arduino-cli config add board_manager.additional_urls https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
arduino-cli core update-index
```

Config file: `~/Library/Arduino15/arduino-cli.yaml`

---

## Install Board Cores

```bash
arduino-cli core install "SparkFun:apollo3@2.2.1"
arduino-cli core install "esp32:esp32"
```

> **Important:** Apollo3 must be version 2.2.1 exactly — v2.1.1 has a known I2C bug that breaks sensor communication on the OLA.

To verify installed cores:
```bash
arduino-cli core list
```

---

## Install Libraries

```bash
arduino-cli lib install "SparkFun u-blox GNSS Arduino Library"
arduino-cli lib install "SparkFun u-blox GNSS v3"
arduino-cli lib install "SparkFun 9DoF IMU Breakout - ICM 20948 - Arduino Library"
arduino-cli lib install "SdFat"
arduino-cli lib install "BotleticsSIM7000"
```

| Library | Version |
|---------|---------|
| SparkFun u-blox GNSS Arduino Library | 2.2.28 |
| SparkFun u-blox GNSS v3 | 3.1.13 |
| SparkFun 9DoF IMU Breakout - ICM 20948 - Arduino Library | 1.3.2 |
| SdFat | 2.3.0 |
| BotleticsSIM7000 | 1.0.6 |

---

## Finding the Serial Port

```bash
ls /dev/cu.usb*
```

> **Important (macOS):** Always use `/dev/cu.*` — never `/dev/tty.*`. The `tty` variant blocks on hardware handshake and reads will hang indefinitely. The port suffix may change between sessions (e.g. `cu.usbserial-10`, `cu.usbserial-110`) — always verify before connecting.

---

## Board FQBNs

| Board | FQBN |
|-------|------|
| OpenLog Artemis (OLA) | `SparkFun:apollo3:sfe_artemis_atp` |
| SparkFun ESP32 Thing Plus | `esp32:esp32:adafruit_feather_esp32` |

To find any board's FQBN:
```bash
arduino-cli board listall | grep -i <boardname>
```

---

## Compiling a Sketch

> **Note:** `arduino-cli` requires the main `.ino` filename to match its containing folder name exactly.

For the OpenLog Artemis:
```bash
arduino-cli compile --fqbn SparkFun:apollo3:sfe_artemis_atp OpenLog_Artemis_GNSS_Logging_Modified/
```

For the ESP32 (buoy_combo):
```bash
arduino-cli compile --fqbn esp32:esp32:adafruit_feather_esp32 buoy_combo/
```

For `esp32_rtk_wifi` (WiFi + BLE — requires larger partition):
```bash
arduino-cli compile --fqbn esp32:esp32:adafruit_feather_esp32 \
  --build-property build.partitions=huge_app \
  --build-property upload.maximum_size=3145728 \
  esp32_rtk_wifi/
```

---

## Uploading Firmware

### OLA — Recommended: use the upload script

[`upload_ola_firmware.py`](upload_ola_firmware.py) handles port detection, compilation, and upload in one step:

```bash
python3 upload_ola_firmware.py
```

**Successful output:**
```
Found port: /dev/cu.usbserial-10

--- Compiling sketch ---
Sketch uses 254764 bytes (25%) of program storage space. Maximum is 983040 bytes.
Global variables use 51640 bytes (13%) of dynamic memory, leaving 341576 bytes for local variables.

--- Uploading firmware ---
Artemis SVL Bootloader
Got SVL Bootloader Version: 5
[##################################################] Upload complete

Done. Firmware uploaded successfully.
```

### Manual upload

```bash
arduino-cli upload -p /dev/cu.usbserial-10 --fqbn SparkFun:apollo3:sfe_artemis_atp OpenLog_Artemis_GNSS_Logging_Modified/
```

---

## Reading Serial Output

`arduino-cli monitor` does not work non-interactively. Use pyserial instead:

```bash
pip3 install pyserial  # one-time install
```

Read output for 10 seconds:
```bash
python3 -c "
import serial, time
s = serial.Serial('/dev/cu.usbserial-10', 115200, timeout=1)
start = time.time()
while time.time() - start < 10:
    data = s.read(256)
    if data:
        print(data.decode('utf-8', errors='replace'), end='', flush=True)
s.close()
"
```

Expected OLA boot output:
```
Artemis OpenLog v2.11
SD card online
Data logging online
IMU online
...
```

---

## Troubleshooting

| Error | Cause | Fix |
|-------|-------|-----|
| `bad CPU type in executable` | SparkFun toolchain is x86_64, Rosetta not installed | `softwareupdate --install-rosetta --agree-to-license` |
| `main file missing from sketch` | `.ino` filename doesn't match folder name | Rename `.ino` to match folder |
| `board not found` / invalid FQBN | Wrong FQBN string | Run `arduino-cli board listall \| grep -i <name>` |
| Port not found | OLA not connected or wrong device path | `ls /dev/cu.usb*`; replug if needed |
| `Resource busy` / port locked | Previous serial process still holding port | Kill any open `cat` or `screen` sessions on the port |
| Serial reads hang forever | Using `/dev/tty.*` instead of `/dev/cu.*` | Switch to `/dev/cu.*` |
