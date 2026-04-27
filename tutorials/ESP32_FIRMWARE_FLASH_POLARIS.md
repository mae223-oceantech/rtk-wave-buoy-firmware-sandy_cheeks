# ESP32 Firmware: Compile, Flash, and Connect to Polaris

This guide covers everything from a fresh install to a live RTK correction stream from
Point One Navigation's Polaris network, using either Arduino IDE or arduino-cli.

**Firmware:** `esp32/esp32_polaris_wifi.ino`

---

## What you need

- SparkFun ESP32 Thing Plus
- USB-C cable (data, not charge-only)
- WiFi network credentials (2.4 GHz)
- Point One Navigation Polaris account (app.pointonenav.com)

---

## Step 1 — Get the firmware

Clone the repo on the computer connected to the ESP32:

```bash
git clone git@github.com:missioncreep11/rtk-wave-buoy.git
cd rtk-wave-buoy
```

---

## Step 2 — Create secrets.h

`secrets.h` holds WiFi and Polaris credentials. It is **not committed to git** — you must create it on each machine you flash from.

**Where to get Polaris credentials:** log in at app.pointonenav.com → Devices or API Keys → create a new key. The username is your email; the password is the API key.

---

## Method A — Arduino IDE

### A1 — Install the ESP32 board package

1. **File → Preferences** → paste into "Additional boards manager URLs":
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
2. **Tools → Board → Boards Manager** → search `esp32` → install **"esp32 by Espressif Systems"** (version 2.x)

### A2 — Install required libraries

**Tools → Manage Libraries** → install:

| Library | Author |
|---|---|
| SparkFun u-blox GNSS Arduino Library | SparkFun Electronics |

WiFi and base64 are built into the ESP32 board package — no separate install needed.

### A3 — Open the firmware

1. **File → Open** → navigate to `esp32/esp32_polaris_wifi.ino`
2. Arduino IDE will prompt to move it into its own folder — click **OK**

### A4 — Create secrets.h in the sketch folder

Arduino IDE will have moved the sketch to a new folder (e.g. `~/Documents/Arduino/esp32_polaris_wifi/`). Create `secrets.h` there:

```cpp
const char ssid[]           = "YourWiFiName";
const char password[]       = "YourWiFiPassword";

const char casterHost[]     = "polaris.pointonenav.com";
const uint16_t casterPort   = 2101;
const char casterUser[]     = "your@email.com";
const char casterUserPW[]   = "your-api-key";
const char mountPoint[]     = "POLARIS";
```

### A5 — Select board and port

1. **Tools → Board → ESP32 Arduino → SparkFun ESP32 Thing Plus**
   (If not listed, use **ESP32 Dev Module**)
2. Plug in the ESP32 via USB-C
3. **Tools → Port** → select the port that appeared

### A6 — Compile

Click the **checkmark (Verify)** button.

Expected output:
```
Compilation complete.
Sketch uses XXXXX bytes (XX%) of program storage space.
```

Common errors:

| Error | Fix |
|---|---|
| `'casterHost' was not declared` | `secrets.h` missing from sketch folder |
| `'base64' was not declared` | Wrong board — must be ESP32, not AVR |
| ESP32 board package not found | Re-add the boards manager URL in Step A1 |

### A7 — Flash

Click the **right-arrow (Upload)** button.

```
Connecting......
Chip is ESP32-D0WDQ6
Uploading...
Hash of data verified.
Hard resetting via RTS pin...
```

If it hangs on `Connecting......`: hold the **BOOT button** (GPIO 0) while upload starts, release when you see `Uploading`.

---

## Method B — arduino-cli

### B1 — Install the ESP32 board package

```bash
arduino-cli core update-index
arduino-cli core install esp32:esp32
```

### B2 — Install required libraries

```bash
arduino-cli lib install "SparkFun u-blox GNSS Arduino Library"
```

### B3 — Set up the sketch folder

arduino-cli requires the folder name to match the `.ino` filename:

```bash
mkdir -p ~/sketches/esp32_polaris_wifi
cp esp32/esp32_polaris_wifi.ino ~/sketches/esp32_polaris_wifi/
```

Create `secrets.h` in the same folder:

```bash
nano ~/sketches/esp32_polaris_wifi/secrets.h
```

Paste the secrets block from Step 2, save and exit.

### B4 — Find the board FQBN

```bash
arduino-cli board listall | grep -i "thing plus"
```

Look for `esp32:esp32:esp32thing_plus`. If not listed, use `esp32:esp32:esp32` (generic ESP32 Dev Module).

### B5 — Find the port

```bash
arduino-cli board list
```

The ESP32 will appear with a port (`/dev/cu.usbserial-XXXX` on Mac, `/dev/ttyUSB0` on Linux, `COM#` on Windows).

### B6 — Compile

```bash
arduino-cli compile --fqbn esp32:esp32:esp32thing_plus ~/sketches/esp32_polaris_wifi
```

### B7 — Upload

```bash
arduino-cli upload -p /dev/cu.usbserial-XXXX --fqbn esp32:esp32:esp32thing_plus ~/sketches/esp32_polaris_wifi
```

Replace `/dev/cu.usbserial-XXXX` with your actual port.

If upload hangs on `Connecting......`: hold **BOOT** (GPIO 0) while uploading.

### B8 — Monitor serial output

```bash
arduino-cli monitor -p /dev/cu.usbserial-XXXX --config baudrate=115200
```

---

## Step 3 — Verify boot output

Open the serial monitor at **115200 baud**. On boot:

```
RTK Wave Buoy — WiFi/Polaris NTRIP Client
Connecting to ZED-F9P...
ZED-F9P connected.
Connecting to WiFi.....
WiFi connected: 192.168.X.X
Press GPIO 0 button to start/stop NTRIP. Blinking LED = ready.
```

**If WiFi fails:** check `ssid`/`password` in `secrets.h`. ESP32 only connects to 2.4 GHz.

**If ZED-F9P not detected:**
- Wiring: ESP32 GPIO 27 → ZED-F9P TX2, ESP32 GPIO 12 → ZED-F9P RX2
- Confirm ZED-F9P is powered and its UART2 baud rate is 115200

---

## Step 4 — Start the Polaris correction stream

Once the LED is **blinking (~1 Hz)**, press the **BOOT button** (GPIO 0).

LED goes **solid** and serial monitor shows:

```
Button — starting NTRIP
Subscribing to Polaris caster...
Connect attempt #1 at 0.0 min
Opening socket to polaris.pointonenav.com
Connected to polaris.pointonenav.com:2101
Request size: XXX / 512 bytes
Caster response: HTTP/1.1 200 OK ...
Sent GGA: $GPGGA,......
RTCM → ZED: 312
RTCM → ZED: 498
...
```

The ZED-F9P will begin receiving RTCM corrections. Within 30–90 seconds expect RTK Float; within a few minutes RTK Fixed under open sky.

Press the button again to stop.

---

## LED status reference

| LED state | Meaning |
|---|---|
| OFF | Booting or WiFi not connected |
| Blinking ~1 Hz | WiFi connected, ready — press button to start |
| Solid ON | NTRIP running, RTCM flowing to ZED-F9P |

---

## Why GGA matters for Polaris

Polaris uses the **Virtual Reference Station (VRS)** method — it synthesizes corrections as if a physical base station existed ~1 km from your position. To do this it needs to know where you are. The firmware sends an NMEA GGA sentence immediately on connect and refreshes it every 5 minutes. Without GGA, Polaris cannot compute corrections and will not stream RTCM.

See `tutorials/POLARIS_NETWORK_RTK.md` for the full technical explanation.
