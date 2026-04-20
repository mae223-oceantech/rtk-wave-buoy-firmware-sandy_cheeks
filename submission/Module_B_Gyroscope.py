"""
Module B — Gyroscope Analysis
Deliverables:
  1. Time-series of all three axes during static test
  2. ZRO bias per axis in dps and deg/min
  3. Integrated heading from rotation test (raw vs bias-corrected)
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# ── file paths ────────────────────────────────────────────────────────────────
STILL_CSV = "imuLog2MinDrift_gyroscope_still_120.csv"
ROT_CSV   = "imuLogGyro90degrees.csv"

# ── helpers ───────────────────────────────────────────────────────────────────
def load(path):
    df = pd.read_csv(path, parse_dates=["Timestamp"])
    df["t"] = (df["Timestamp"] - df["Timestamp"].iloc[0]).dt.total_seconds()
    return df

def integrate(gyr_dps, t_s):
    dt = np.diff(t_s, prepend=t_s[0])
    return np.cumsum(gyr_dps * dt)

# ── load and trim still log to clean window only ──────────────────────────────
still_raw = load(STILL_CSV)
still = still_raw[(still_raw["t"] >= 15) & (still_raw["t"] <= 216)].copy()
still["t"] = still["t"] - still["t"].iloc[0]   # re-zero time

# ── load rotation log ─────────────────────────────────────────────────────────
rot = load(ROT_CSV)

# ── ZRO bias per axis ─────────────────────────────────────────────────────────
bias = {ax: still[ax].mean() for ax in ("GyrX", "GyrY", "GyrZ")}

print("=" * 55)
print("B1 — Zero-Rate Output (ZRO) Bias  [clean window only]")
print("=" * 55)
print(f"  Clean window : t=15 s to t=216 s  ({216-15} s)")
print(f"  {'Axis':<6}  {'Bias (dps)':>10}  {'Bias (deg/min)':>14}  {'In +-5 dps?':>11}")
for ax in ("GyrX", "GyrY", "GyrZ"):
    deg_min = bias[ax] * 60
    ok = "YES" if abs(bias[ax]) <= 5.0 else "NO"
    print(f"  {ax:<6}  {bias[ax]:>10.4f}  {deg_min:>14.3f}  {ok:>11}")

# ── Plot 1 — Static time-series (all three axes, clean window) ────────────────
fig1, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
fig1.suptitle("B1 — Gyroscope Static Test (clean window: 15–216 s)", fontsize=13, fontweight="bold")

for i, ax_name in enumerate(("GyrX", "GyrY", "GyrZ")):
    axes[i].plot(still["t"], still[ax_name], lw=0.5, alpha=0.7, color="steelblue", label="raw")
    axes[i].axhline(bias[ax_name], color="red", lw=1.5, ls="--",
                    label=f"bias = {bias[ax_name]:.4f} dps  ({bias[ax_name]*60:.3f} deg/min)")
    axes[i].axhline( 5, color="gray", lw=0.8, ls=":", label="+-5 dps spec")
    axes[i].axhline(-5, color="gray", lw=0.8, ls=":")
    axes[i].set_ylabel(f"{ax_name} (dps)")
    axes[i].legend(fontsize=8, loc="upper right")

axes[2].set_xlabel("Time (s)")
plt.tight_layout()
plt.savefig("B1_static_timeseries.png", dpi=150)
print("\n  Saved: B1_static_timeseries.png")

# ── Plot 2 — Integrated heading from rotation test ────────────────────────────
raw_angle  = integrate(rot["GyrZ"].values, rot["t"].values)
corr_angle = integrate(rot["GyrZ"].values - bias["GyrZ"], rot["t"].values)
excursion_raw  = raw_angle.min()
excursion_corr = corr_angle.min()

print("\n" + "=" * 55)
print("B2 — 90 deg Rotation Test (integrated heading)")
print("=" * 55)
print(f"  Peak angle raw            : {excursion_raw:+.2f} deg  (error {excursion_raw-(-90):+.2f} deg vs -90)")
print(f"  Peak angle bias-corrected : {excursion_corr:+.2f} deg  (error {excursion_corr-(-90):+.2f} deg vs -90)")

fig2, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
fig2.suptitle("B2 — Integrated Heading: Rotation Test", fontsize=13, fontweight="bold")

ax1.plot(rot["t"], rot["GyrZ"], lw=0.8, color="steelblue")
ax1.axhline(0, color="k", lw=0.5, ls="--")
ax1.set_ylabel("GyrZ (dps)")
ax1.set_title("Raw gyro rate during rotation")

ax2.plot(rot["t"], raw_angle,  color="steelblue", label=f"Raw -> {excursion_raw:.1f} deg")
ax2.plot(rot["t"], corr_angle, color="orange", ls="--", label=f"Bias-corrected -> {excursion_corr:.1f} deg")
ax2.axhline(-90, color="red", lw=1.2, ls=":", label="Target -90 deg")
ax2.set_ylabel("Integrated angle (deg)")
ax2.set_xlabel("Time (s)")
ax2.set_title("Integrated heading before and after bias correction")
ax2.legend(fontsize=9)

plt.tight_layout()
plt.savefig("B2_integrated_heading.png", dpi=150)
print("  Saved: B2_integrated_heading.png")

plt.show()
print("\nDone.")
