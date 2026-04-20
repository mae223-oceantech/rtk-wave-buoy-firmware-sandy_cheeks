# ZED-F9P Setup: u-center NTRIP + OLA Logging

In this lab you will connect the ZED-F9P directly to a PC running u-center to configure it, connect to the **Point One Nav Polaris** RTK correction network, and observe the system reach an RTK fix. The OpenLog Artemis logs the resulting high-precision GPS data to SD card over Qwiic at the same time.

This is the same configuration the firmware (`esp32_polaris.ino`) handles automatically in the field — doing it manually first helps you understand what the firmware is doing and lets you watch each step in real time.

---

## What You Need

- SparkFun ZED-F9P with u-blox ANN-MB1 antenna
- PC running Windows with **u-center** installed (one per group is fine)
- SparkFun OpenLog Artemis with microSD card inserted
- Qwiic cable (ZED-F9P → OLA)
- USB-C cable (ZED-F9P → PC)
- USB-C cable (OLA → PC, for serial monitoring — optional but recommended)
- Clear sky view outside, or near a window with sky exposure

---

## Physical Setup

```
[Antenna] ──── [ZED-F9P] ──── USB-C ──── [PC / u-center]
                   │
                 Qwiic
                   │
              [OpenLog Artemis] ──── SD card logging
```

1. Screw the ANN-MB1 antenna onto the ZED-F9P SMA port
2. Connect the ZED-F9P to the OLA with a Qwiic cable
3. Connect the ZED-F9P to your PC with a USB-C cable
4. If monitoring OLA output, connect the OLA to your PC with a second USB-C cable
5. Place the antenna somewhere with a clear view of the sky

---

## Part 1 — Connect u-center to the ZED-F9P

1. Open u-center
2. Click **Receiver → Connection → COMx** and select the ZED-F9P port
   - In Windows Device Manager it will appear as something like `USB Serial Device (COM5)`
   - Try each port if unsure — when connected, the status bar at the bottom of u-center will show satellite data moving
3. Set the baud rate to **115200** (Receiver → Baudrate → 115200)

Once connected you should see satellites appearing in the satellite signal bar and the fix type updating in the bottom status bar.

---

## Part 2 — Explore the Interface

Before setting anything up, spend a few minutes exploring what u-center shows you.

Open these views from the **View** menu:

| View | What it shows |
|------|--------------|
| Satellite Level | Signal strength for each visible satellite |
| Fix Status | Current fix type: No Fix / 3D / Float / Fixed |
| Data View | Raw UBX message fields |
| Text Console | Raw text output from the receiver |
| Google Map View | Position plotted on a map (requires internet) |

At this point you likely have a standard 3D GPS fix — not RTK. Note the horizontal accuracy shown in the Data View (typically 1–3 metres without RTK).

---

## Part 3 — Configure Key Parameters

Open the **UBX Configuration** panel: **View → Configuration View** (or F9).

### Navigation Rate

Under **CFG-RATE:**
- Set **Measurement Period** to `1000 ms` (1 Hz)
- Set **Navigation Rate** to `1`

> **Why 1 Hz?** Higher rates (e.g. 10 Hz) can starve the RTK engine — the receiver spends so much time computing navigation solutions that it can't process incoming RTCM correction data fast enough to converge on a fix.

Click **Send** after changing values.

### Enable Raw Measurement Messages

Under **CFG-MSG**, enable the following messages on the **USB** port:

| Message | Purpose |
|---------|---------|
| RXM-RAWX | Raw pseudorange and carrier phase — needed for RTKLIB post-processing |
| RXM-SFRBX | Satellite navigation data — needed for RTKLIB |
| NAV-PVT | Position, velocity, time — the main output used by the OLA |

For each message: select it from the list, check the **USB** column, and click **Send**.

### Save Configuration

Under **CFG-CFG:**
- Click **Save current configuration**
- Check all three memory layers: **BBR**, **Flash**, **I2C-EEPROM**
- Click **Send**

This writes your settings to the ZED-F9P's non-volatile memory so they survive a power cycle.

---

## Part 4 — Connect to the Polaris NTRIP Network

NTRIP delivers RTK correction data to your rover (the ZED-F9P). Polaris uses a **Virtual Reference Station (VRS)** — rather than streaming corrections from a single fixed base station, it uses your rover's position to synthesize optimal corrections from the nearest stations in its nationwide network. To do this, your NTRIP client must send its GPS position (as an NMEA GGA sentence) back to the server after connecting.

u-center handles this automatically. In the field deployment, `esp32_polaris.ino` does the same thing in firmware.

1. Go to **Receiver → NTRIP Client...**
2. Fill in the connection details:

   | Field | Value |
   |-------|-------|
   | Caster | `polaris.pointonenav.com` |
   | Port | `2101` |
   | Mount Point | `POLARIS` |
   | Username | *(provided by instructor)* |
   | Password | *(provided by instructor)* |

3. Click **OK** — u-center will connect to the caster and begin streaming RTCM correction data to the ZED-F9P

Watch the **NTRIP** indicator in the status bar. Once corrections are flowing you should see it turn green and the fix status begin to change.

---

## Part 5 — Watch the Fix Converge

RTK convergence takes a few minutes after corrections start flowing. Watch the fix status progress:

| Fix Type | Meaning | Typical accuracy |
|----------|---------|-----------------|
| 3D | Standard GPS, no corrections | 1–3 m |
| RTK Float | Receiving corrections, ambiguities not resolved | 0.3–1 m |
| RTK Fixed | Ambiguities resolved, full RTK | 1–3 cm |

You can also watch the ZED-F9P LED:
- **Solid on** — 3D fix
- **Blinking** — RTK Float
- **Off** — RTK Fixed

> If the fix stays at Float for more than 5–10 minutes, check that the antenna has a clear sky view, the RTCM byte counter in u-center is incrementing, and the signal bars show strong signals on multiple constellations.

Note the horizontal accuracy value in the Data View as it improves from metres down to centimetres.

---

## Part 6 — Confirm OLA is Logging

While u-center is running, the OLA is simultaneously reading position data from the ZED-F9P over Qwiic (I2C). These two connections use separate ports on the ZED-F9P and do not interfere with each other.

If you have a serial terminal open on the OLA (see `tutorials/SERIAL_MONITOR_SETUP.md`):

```
2026/04/20 15:42:11.00,GNSS,32.87012,-117.25341,20.412,2,2,0.012,0.018,...
```

The `carrier_solution` column (second-to-last) will show:
- `0` — no RTK
- `1` — RTK Float
- `2` — RTK Fixed

Once you see `2` in the log, you are recording centimetre-level positions.

---

## What to Record in Your Lab Notebook

1. How long did it take to go from 3D → Float → Fixed?
2. What was the horizontal accuracy before and after RTK?
3. How many satellites were visible? Which constellations?
4. What was the RSSI / signal quality of the NTRIP connection?
5. Did the fix drop at any point? What happened to accuracy when it did?

---

## Troubleshooting

**u-center shows no satellites**
- Check that you selected the correct COM port
- Verify the antenna is connected and has sky view
- Give it 1–2 minutes for the receiver to acquire signals from cold start

**Fix stays at 3D, never reaches Float**
- Confirm the NTRIP connection is active (status bar should show green, RTCM byte count should be increasing)
- Check your credentials and mount point
- Verify network connectivity on the PC

**Fix reaches Float but never Fixed**
- Antenna may be partially obstructed — move it to a location with a cleaner sky view
- Wait longer — Float to Fixed can take up to 10 minutes in challenging conditions
- Check that signals are strong on multiple constellations in the satellite view

**OLA shows "GNSS offline"**
- Check the Qwiic cable connection at both ends
- Confirm the ZED-F9P is powered (connected to PC via USB-C)
- Try pressing reset on the OLA

**OLA logs show carrier_solution = 0 even after u-center shows Fixed**
- The OLA polls position over I2C — it may lag u-center by a few seconds
- Check that NAV-PVT is enabled on the I2C port in u-center (CFG-MSG → NAV-PVT → check I2C column)
