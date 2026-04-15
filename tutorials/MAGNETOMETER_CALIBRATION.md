# Magnetometer Calibration

## Overview

The ICM-20948 on the OpenLog Artemis includes an AK09916 3-axis magnetometer. Raw magnetometer data is distorted by permanent magnetic fields nearby — from the PCB, battery, enclosure hardware, and local environment. This is called **hard iron distortion** and causes the sensor's output to trace an ellipsoid offset from the origin rather than a sphere centered at zero.

Before using heading (compass direction) in your wave analysis, you must collect calibration data and compute offsets to correct for this distortion.

---

## Background

When you rotate the sensor through all orientations, the magnetometer traces a sphere in 3D space. In an ideal sensor with no distortion, that sphere is centered at the origin. Hard iron distortion shifts the center of the sphere away from the origin by a fixed offset on each axis.

The correction is simple:

$$\text{corrected} = \text{raw} - \text{offset}$$

where each offset is the midpoint of that axis's measurement range:

$$\text{offset}_x = \frac{\max(X) + \min(X)}{2}$$

---

## Step 1 — Flash the firmware

Make sure the latest OLA firmware is loaded. Follow `tutorials/IMU_STUDENT_GUIDE.md` if you have not already done this today.

---

## Step 2 — Set the RTC time

The OLA coin cell may be dead. Before collecting data, set the RTC manually so your log files have correct timestamps.

1. Open a serial terminal (see `tutorials/SERIAL_MONITOR_SETUP.md`) at 115200 baud
2. Press any key to open the main menu
3. Press `t` for **Set UTC Time**
4. Enter current UTC date and time when prompted

---

## Step 3 — Collect calibration data

You need to rotate the sensor through as many orientations as possible so the magnetometer samples the full sphere.

1. Insert a formatted microSD card into the OLA
2. Power on — logging starts automatically
3. Slowly rotate the buoy through all axes:
   - Roll 360° (rotate around the long axis)
   - Pitch 360° (nose up and over)
   - Yaw 360° (spin flat on a table)
   - Repeat each rotation 2–3 times, moving slowly
4. Aim for **at least 5 minutes** of rotation data
5. Power off and retrieve the SD card

The log file will be named `imuLog00000.csv` (or incrementing number). Rename it `magcaldata.csv` and place it in the `ola_data/` folder at the repo root.

> **Note:** Keep the sensor away from metal tables, laptops, and phone chargers during collection — these add extra hard iron distortion that won't be present on the buoy at sea.

---

## Step 4 — Run the calibration notebook

Open `accelerometer/magcal_notebook.ipynb` in VS Code using the `mae223` kernel.

Run the cells in order:

1. **Load data** — reads your CSV and extracts mag, accel, and gyro columns
2. **Plot raw data** — you should see an off-center ellipsoid in 3D
3. **Compute offsets** — fill in the `min()` and `max()` calls for each axis, then compute the midpoints
4. **Apply calibration** — subtract the offsets from the raw data
5. **Plot calibrated data** — the scatter should now form a sphere near the origin
6. **Record your values** — write down the three offsets in the table

---

## What good calibration looks like

| Raw | Calibrated |
|-----|------------|
| Ellipsoid offset from origin | Sphere centered near origin |
| XY projection: off-center circle or ellipse | XY projection: circle centered at (0, 0) |

If your calibrated data is still an ellipse (not a circle), the axes have different scale factors — this is soft iron distortion and requires a more advanced correction matrix. Flag this to the instructor.

---

## Step 5 — Record offsets for sensor fusion

You will use these offsets in the sensor fusion notebook and eventually in the firmware. Record them in your lab notebook in the format:

```
Hard iron offsets (µT):
  offset_x = ___
  offset_y = ___
  offset_z = ___
```

---

## Troubleshooting

**Data looks like a flat disk, not a sphere**
You did not rotate through all orientations — pitch and roll coverage is missing. Re-collect with more complete rotation.

**Scatter has a gap or hole**
Some orientations were not reached. Not a problem for a basic hard iron correction, but note the gap.

**Values look wrong / all zeros**
Check that the magnetometer is enabled in the OLA firmware settings (main menu → sensor settings). The AK09916 is a separate sensor on the I2C bus and must be explicitly enabled.

**Timestamp is 2000/01/01**
You forgot to set the RTC (Step 2). Re-collect with the time set, or note in your lab notebook that the timestamp is approximate.
