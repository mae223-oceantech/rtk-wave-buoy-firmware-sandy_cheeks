# ESP32 Firmware: Compile, Flash, and Connect to Polaris

This guide covers everything from a fresh Arduino IDE install to a live RTK correction stream from Point One Navigation's Polaris network.

**Firmware to use:** `esp32/esp32_polaris.ino`

> Note: Amara's `esp32_rtk_mae.ino` also works, but `esp32_polaris.ino` is more complete:
> it adds BLE monitoring, fixes a nav-rate bug that prevents RTK fix, and uses a 5-min GGA
> refresh interval appropriate for a slow-moving buoy. Use `esp32_polaris.ino`.

---

## What you need

- SparkFun ESP32 Thing Plus
- USB-C cable (data, not charge-only)
- Arduino IDE 2.x installed
- WiFi network credentials
- Point One Navigation Polaris account (get one at app.pointonenav.com)

---

## Step 1 — Install the ESP32 board package

1. Open Arduino IDE → **File → Preferences**
2. In "Additional boards manager URLs" paste:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
3. Click OK
4. Go to **Tools → Board → Boards Manager**
5. Search `esp32`, install **"esp32 by Espressif Systems"** (version 2.x)

---

## Step 2 — Install required libraries

Go to **Tools → Manage Libraries** and install:

| Library | Author |
|---|---|
| SparkFun u-blox GNSS Arduino Library | SparkFun Electronics |

The BLE and WiFi libraries are built into the ESP32 board package — no separate install needed.

---

## Step 3 — Create secrets.h

The firmware reads credentials from a `secrets.h` file that is **not committed to git** (it's in `.gitignore`). You must create it yourself.

In the `esp32/` folder, create a new file called `secrets.h` with this content:

```cpp
// WiFi
const char ssid[]     = "YourWiFiName";
const char password[] = "YourWiFiPassword";

// Polaris NTRIP caster
const char     casterHost[]   = "polaris.pointonenav.com";
const uint16_t casterPort     = 2101;
const char     casterUser[]   = "your@email.com";       // Polaris account email
const char     casterUserPW[] = "your-api-key-here";    // Polaris API key
const char     mountPoint[]   = "POLARIS";
```

**Where to get Polaris credentials:**
- Log in at app.pointonenav.com
- Go to **Devices** or **API Keys**
- Create a new device/key — the username is your email, the password is the API key shown there

---

## Step 4 — Open the firmware

1. In Arduino IDE: **File → Open**
2. Navigate to `esp32/esp32_polaris.ino`
3. Arduino IDE will ask if you want to create a folder — click **OK**

You should see two tabs: `esp32_polaris` and `secrets` (the `.h` file you just created).

---

## Step 5 — Select the board and port

1. **Tools → Board → ESP32 Arduino → SparkFun ESP32 Thing Plus**
   - If that exact option isn't listed, use **ESP32 Dev Module**
2. Plug in the ESP32 via USB-C
3. **Tools → Port** — select the port that appeared (usually `/dev/cu.usbserial-XXXX` on Mac or `COM#` on Windows)

---

## Step 6 — Compile

Click the **checkmark (Verify)** button in the top left.

Expected output in the console:
```
Compilation complete.
Sketch uses XXXXX bytes (XX%) of program storage space.
```

If you get errors:

| Error | Fix |
|---|---|
| `'casterHost' was not declared` | `secrets.h` is missing or not in the same folder as the `.ino` |
| `BLEDevice.h: No such file` | Wrong board selected — must be ESP32, not AVR |
| `fatal error: esp_mac.h` | ESP32 board package is too old — update to 2.x |

---

## Step 7 — Flash

Click the **right-arrow (Upload)** button.

The console will show:
```
Connecting......
Chip is ESP32-D0WDQ6
Uploading...
Hash of data verified.
Leaving...
Hard resetting via RTS pin...
```

If it hangs on `Connecting......`:
- Hold the **BOOT button** (GPIO 0) on the ESP32 while upload starts, release once you see "Uploading"
- Some boards need this on first flash

---

## Step 8 — Open the serial monitor

**Tools → Serial Monitor**, set baud rate to **115200**.

On boot you should see:
```
RTK Wave Buoy - WiFi NTRIP Client
BLE advertising as: RTK-Buoy-XXXX
Connecting to ZED-F9P over UART...
ZED-F9P connected!
Connecting to WiFi.....
WiFi connected: 192.168.X.X
Press button to start NTRIP. BLE: RTK-Buoy-XXXX
```

**If WiFi fails to connect:** Double-check `ssid` and `password` in `secrets.h`. The ESP32 only connects to 2.4 GHz networks.

**If ZED-F9P not detected:**
- Check wiring: ESP32 GPIO 27 → ZED-F9P TX2, ESP32 GPIO 12 → ZED-F9P RX2
- Confirm ZED-F9P is powered and UART2 baud is 115200

---

## Step 9 — Start the Polaris correction stream

Once WiFi is connected and the LED is **blinking (~1 Hz)**, press the **BOOT button** (GPIO 0) on the ESP32.

LED goes **solid** and the serial monitor shows:
```
Button - Starting NTRIP
Connecting to NTRIP caster...
Connected to polaris.pointonenav.com:2101
Requesting mount: POLARIS
NTRIP streaming RTCM...
Sent GGA: $GPGGA,......
RTCM -> ZED: 312
RTCM -> ZED: 498
...
```

The ZED-F9P will begin receiving RTCM corrections. Within 30–90 seconds you should see it achieve RTK Float, and within a few minutes RTK Fixed under good sky conditions.

Press the button again to stop the NTRIP stream.

---

## LED status reference

| LED state | Meaning |
|---|---|
| OFF | Booting or WiFi not connected |
| Blinking ~1 Hz | WiFi connected, ready — press button to start |
| Solid ON | NTRIP running, RTCM flowing to ZED-F9P |

---

## Optional: BLE monitoring

The firmware advertises as `RTK-Buoy-XXXX` over Bluetooth. Connect with **nRF Connect** (free, iOS/Android/desktop) using the Nordic UART Service.

Once connected, type commands:

| Command | What it shows |
|---|---|
| `STATUS` | WiFi, NTRIP, and caster info |
| `GPS` | Current lat/lon/alt, RTK fix type, SIV, hAcc |
| `RESET` | Restart NTRIP connection |
| `SET HOST polaris.pointonenav.com` | Change caster host (saved to flash) |
| `SET PORT 2101` | Change port (saved to flash) |
| `SET MOUNT POLARIS` | Change mount point (saved to flash) |
| `SET USER email@x.com` | Change username (saved to flash) |
| `SET PASS yourkey` | Change API key (saved to flash) |

Config changes persist across reboots — you can update Polaris credentials wirelessly without reflashing.

---

## Why GGA matters for Polaris

Polaris uses the **Virtual Reference Station (VRS)** method. Instead of sending corrections from a physical base station, it synthesizes corrections as if a virtual base station existed 1–2 km from your position. To do this, it needs to know where you are — which is why the firmware sends an NMEA GGA sentence immediately on connect and refreshes it every 5 minutes. Without GGA, Polaris has no rover position and will not stream RTCM.

See `tutorials/POLARIS_NETWORK_RTK.md` for the full technical explanation.
