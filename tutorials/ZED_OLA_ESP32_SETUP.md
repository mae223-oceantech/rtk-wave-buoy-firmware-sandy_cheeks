# OLA Setup: Field Logging with ESP32 + Polaris

This tutorial covers configuring the ZED-F9P and OpenLog Artemis to work together in the field deployment, where the **ESP32 delivers Polaris RTCM corrections** to the ZED-F9P and the **OLA logs high-precision position data** to SD card over Qwiic.

---

## What You Need

- SparkFun ZED-F9P with u-blox ANN-MB1 antenna
- SparkFun OpenLog Artemis with microSD card
- SparkFun ESP32 Thing Plus (flashed with `esp32_polaris_wifi.ino`)
- Qwiic cable (ZED-F9P → OLA)
- 2× jumper wires (ESP32 UART2 → ZED-F9P UART1)
- PC with u-center installed (Windows) for one-time ZED-F9P configuration
- USB-C cable (ZED-F9P → PC, for u-center)

---

## Physical Wiring

```
[Antenna] ──── [ZED-F9P] ──── Qwiic ──── [OLA] ──── SD card
                   │
              UART1 (TX1/MISO, RX1/MOSI)
                   │
              [ESP32 GPIO27/12]
                   │
                 WiFi → Polaris → RTCM corrections
```

### UART1 connections (ESP32 ↔ ZED-F9P)

| ESP32 pin | ZED-F9P pad | Direction |
|-----------|-------------|-----------|
| GPIO 27 (RX2) | TX1 / MISO | ZED-F9P → ESP32 |
| GPIO 12 (TX2) | RX1 / MOSI | ESP32 → ZED-F9P |
| GND | GND | Common ground — required |

> The pads labelled **TX1/MISO** and **RX1/MOSI** on the SparkFun ZED-F9P breakout are UART1. Cross TX to RX as shown.

### Qwiic connection (OLA ↔ ZED-F9P)

Connect any Qwiic port on the ZED-F9P to any Qwiic port on the OLA. The OLA powers the ZED-F9P over Qwiic — no separate power wire needed.

---

## Step 1 — Configure the ZED-F9P in u-center (one time)

Connect the ZED-F9P to your PC via USB-C and open u-center.

### Set UART1 baud rate and protocols

**View → Configuration View → PRT (Ports) → UART1:**

- Baud rate: `115200`
- Protocol in: check **UBX** and **RTCM3**
- Protocol out: check **UBX**

Click **Send**.

> The ZED-F9P ships at 9600 baud on UART1. The ESP32 firmware expects 115200 — this step is required or the ESP32 will not detect the ZED-F9P.

### Enable required messages on I2C

**View → Configuration View → CFG-MSG**

For each message below, select it, check the **I2C** column, and click **Send**:

| Message | Purpose |
|---------|---------|
| NAV-PVT | Standard position/velocity/time — used as fallback |
| NAV-HPPOSLLH | High-precision lat/lon/alt — **required for cm-level OLA logging** |

### Save to non-volatile memory

**View → Configuration View → CFG-CFG:**

- Click **Save current configuration**
- Check **BBR**, **Flash**, and **I2C-EEPROM**
- Click **Send**

Settings now survive power cycles.

---

## Step 2 — Configure the OLA SD card

The OLA reads `OLA_GNSS_settings.cfg` from the SD card on every boot. Ensure these settings are present:

| Setting | Value | Purpose |
|---------|-------|---------|
| `GNSS:logUBXNAVPVT` | `1` | Log standard position messages |
| `GNSS:logUBXNAVHPPOSLLH` | `1` | Log high-precision position — **required for cm-level data** |
| `useGPIO32ForStopLogging` | `1` | Enable clean log file close via GPIO32 |

If the config file does not exist, the OLA will create one with defaults on first boot. You can then edit it on your PC and reinsert the card.

---

## Step 3 — Power up and verify

The OLA and ESP32 share a LiPo battery through a Y cable. Power-up order matters — follow this sequence exactly:

1. Insert the SD card into the OLA
2. Confirm all wiring is complete (Qwiic, UART jumpers, shared GND)
3. Connect the battery's **first output to the OLA** — this also powers the ZED-F9P over Qwiic
4. Wait a few seconds for the OLA and ZED-F9P to finish booting
5. Connect the battery's **second output to the ESP32**
6. Last, connect your **computer to the ESP32 via USB-C** to monitor serial output

> **Do not connect USB-C to the OLA or ZED-F9P.** Both boards are powered entirely through the battery Y cable. A USB-C connection on either board can interfere with power sequencing and cause a brownout when the ZED-F9P starts up.

Open a serial terminal on the OLA port at **115200 baud**. You should see:

```
Config file read complete
Artemis OpenLog GNSS v3.2
Created log file: dataLog00000.ubx
SD card online
Data logging online
GNSS online
2026/04/28 23:00:32.67 NAV-PVT
2026/04/28 23:00:32.68 NAV-HPPOSLLH
2026/04/28 23:00:32.70,IMU,...
```

`NAV-HPPOSLLH` lines confirm the OLA is receiving high-precision positions.

On the ESP32 serial monitor (115200 baud), press the **BOOT button** to start the NTRIP connection. Once Polaris corrections are flowing you should see:

```
RTCM → ZED: 312
RTCM → ZED: 498
```

RTK Float typically arrives within 30–90 seconds; RTK Fixed within a few minutes under open sky.

---

## Step 4 — Stop logging and retrieve data

To close the log file cleanly before removing the SD card:

**Momentarily short GPIO32 to GND on the OLA** — a jumper wire from the GPIO32 pad to any GND pin works. The OLA will flush and close the file, then stop logging.

Do not remove the SD card while the STAT LED is actively blinking (mid-write).

---

## Analysing the data

Parse the UBX log file with the notebook at `ubx_parsers/parse_and_plot.ipynb`. Update `UBX_FILE` to point to your file and run all cells.

RTK Fixed epochs will show `carrier_solution = 2` and horizontal accuracy of 1–5 cm.

---

## Troubleshooting

**System doesn't come up reliably / OLA or ZED-F9P appears to reset on ESP32 power-on**
- Follow the power-up sequence in Step 3 exactly — connect OLA first, ESP32 second
- Do not connect USB-C to the OLA or ZED-F9P while on battery power

**OLA shows "GNSS offline"**
- Check the Qwiic cable at both ends
- Confirm the ZED-F9P is powered

**NAV-HPPOSLLH not appearing in OLA serial output**
- Confirm `GNSS:logUBXNAVHPPOSLLH=1` in the config file
- Confirm NAV-HPPOSLLH is enabled on the I2C port in u-center (Step 1)

**ESP32 shows "ZED-F9P not detected"**
- Confirm UART1 baud rate is 115200 (Step 1)
- Check GPIO27/12 wiring and that GND is shared

**RTK never reaches Fixed**
- Confirm RTCM bytes are flowing on ESP32 serial (`RTCM → ZED: ...`)
- Check antenna has clear sky view
- Confirm `UART1 Protocol in` includes RTCM3 (Step 1)

**No new UBX file on reboot**
- Each power cycle creates a new numbered file (`dataLog00001.ubx`, etc.)
- If files stop incrementing, reformat the SD card as FAT32 or exFAT
