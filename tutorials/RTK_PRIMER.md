# Real-Time Kinematic (RTK) GNSS — Technical Primer

This primer is written for graduate-level engineers. It covers the signal model, error sources, the differencing approach that makes RTK work, integer ambiguity resolution, and the practical performance limits relevant to coastal instrumentation.

---

## 1. The GNSS Measurement Model

A GNSS receiver makes two fundamental measurements per satellite per epoch:

### Pseudorange (code measurement)

$$\rho = r + c(\delta t_r - \delta t^s) + I + T + \epsilon_\rho$$

Where:
- $r$ — true geometric range between receiver and satellite
- $c\,\delta t_r$, $c\,\delta t^s$ — receiver and satellite clock offsets
- $I$ — ionospheric delay (positive, signal slowed)
- $T$ — tropospheric delay
- $\epsilon_\rho$ — code noise + multipath (~0.3–3 m)

### Carrier phase measurement

$$\lambda \phi = r + c(\delta t_r - \delta t^s) - I + T + \lambda N + \epsilon_\phi$$

Where:
- $\lambda$ — carrier wavelength (L1: 19.0 cm, L2: 24.4 cm)
- $\phi$ — measured phase in cycles
- $N$ — **integer ambiguity** — unknown integer number of full cycles between satellite and receiver at lock-on
- $\epsilon_\phi$ — phase noise (~1–3 mm)

Note the sign flip on $I$: the ionosphere advances the carrier phase while delaying the code. This is exploited in dual-frequency receivers to directly estimate and remove ionospheric delay.

The carrier phase is roughly **100× more precise** than pseudorange but carries an unknown integer offset $N$ that must be resolved before it can be used for positioning.

---

## 2. Standalone GNSS Error Budget

| Error source | Typical magnitude | Spatial correlation |
|-------------|------------------|-------------------|
| Satellite clock | ~1 m (broadcast) / ~3 cm (precise) | Global (same for all receivers) |
| Satellite orbit | ~1–2 m broadcast | Global |
| Ionospheric delay | 1–10 m | Correlated over ~100–300 km |
| Tropospheric delay | 2–4 m zenith | Correlated over ~50–100 km |
| Receiver clock | Eliminated by 4th satellite | Local |
| Multipath | 0.1–1 m | Completely local |
| Receiver noise | ~0.3 m (code), ~3 mm (phase) | Local |

Standalone pseudorange positioning (SPS) achieves ~3–5 m accuracy after satellite clock and orbit errors are reduced by the broadcast navigation message. Carrier phase positioning without resolving $N$ is meaningless for absolute position.

---

## 3. Differential GNSS and the Differencing Hierarchy

The key insight behind high-accuracy GNSS is that **spatially-correlated errors cancel when you difference observations between receivers**. The closer two receivers are, the more complete the cancellation.

### Single difference (between receivers, one satellite $s$)

$$\Delta \phi^s = \phi^s_r - \phi^s_b$$

Satellite clock error cancels exactly. Orbit error, ionospheric and tropospheric delays cancel to the degree they are spatially correlated (i.e., the baseline is short relative to the correlation length).

### Double difference (between receivers, between satellites $s$ and $r$)

$$\nabla\Delta \phi^{sr} = \Delta\phi^s - \Delta\phi^r$$

Receiver clock offsets cancel exactly. The integer ambiguity becomes $\nabla\Delta N^{sr}$, which is also an integer — and now has no nuisance clock parameters mixed in. The double-difference observation model is:

$$\lambda \,\nabla\Delta\phi^{sr} = \nabla\Delta r^{sr} - \nabla\Delta I^{sr} + \nabla\Delta T^{sr} + \lambda\,\nabla\Delta N^{sr} + \epsilon$$

For a short baseline (<10 km), $\nabla\Delta I \approx 0$ and $\nabla\Delta T \approx 0$, leaving only geometry and the integer ambiguity. **This is the RTK measurement model.**

---

## 4. Integer Ambiguity Resolution

Resolving $\nabla\Delta N$ to its correct integer value is what converts RTK from "float" (~0.1–0.5 m) to "fixed" (~1–3 cm). This is the central computational challenge in RTK.

### The search problem

With $n$ satellites you have $n-1$ double-difference ambiguities. From pseudorange and Doppler you can compute a **float solution** — a real-valued estimate $\hat{N}$ with a covariance $Q_{\hat{N}}$. The task is to find the integer vector $N$ that is most consistent with the data:

$$\hat{N}_{fix} = \arg\min_{N \in \mathbb{Z}^{n-1}} (\hat{N} - N)^T Q_{\hat{N}}^{-1} (\hat{N} - N)$$

This is an **integer least squares** problem. The search space is an ellipsoid in $\mathbb{Z}^{n-1}$.

### LAMBDA method

The standard algorithm is **LAMBDA** (Least-squares AMBiguity Decorrelation Adjustment, Teunissen 1995). It applies a $Z$-transformation to decorrelate the ambiguities (making the search space more spherical), searches efficiently in the transformed space, then maps back. LAMBDA dominates RTK implementations including u-blox.

### Validation: ratio test

After finding the best candidate $N_1$ and second-best $N_2$, the **ratio test** checks:

$$\text{ratio} = \frac{(N_2 - \hat{N})^T Q^{-1} (N_2 - \hat{N})}{(N_1 - \hat{N})^T Q^{-1} (N_1 - \hat{N})} > \text{threshold}$$

A ratio > 3 (often > 5 in practice) indicates the correct integer is unambiguous. Below threshold, the receiver reports RTK Float rather than Fixed. The ZED-F9P uses an adaptive threshold based on satellite geometry and signal quality.

### Wide-lane / narrow-lane bootstrapping

On dual-frequency receivers, a common strategy is:
1. Form the **wide-lane** combination (L1 − L2, $\lambda_{WL} \approx 86$ cm) — large wavelength makes integer search easy
2. Use the resolved wide-lane ambiguity to constrain the **narrow-lane** search (effective wavelength ~10.7 cm)

This dramatically speeds up convergence, which is why dual-frequency receivers like the ZED-F9P reach fixed solutions in seconds to minutes rather than the tens of minutes needed by single-frequency receivers.

---

## 5. Distance-Dependent Error Growth

The RTK fixed solution relies on the residual errors after double-differencing being negligible. As baseline length increases:

**Ionosphere:** The differential ionospheric delay grows roughly linearly with baseline at mid-latitudes under quiet conditions — approximately 1–3 ppm (mm/km). At 30 km this is 3–9 cm, enough to corrupt ambiguity resolution. Under active conditions (solar storm, post-sunset equatorial plasma bubbles) gradients can be 10× larger.

**Troposphere:** The wet component (water vapor) is poorly modeled. Differential tropospheric delay grows at roughly 0.2–1 mm/km baseline.

**Orbit:** Differential orbit error is ~0.1 mm/km for broadcast ephemeris.

The practical limit for reliable RTK with a single base station is **~30–50 km** under typical conditions. Network RTK (Polaris VRS) addresses this by interpolating the error field from multiple nearby stations — see `tutorials/POLARIS_NETWORK_RTK.md` for details.

---

## 6. RTCM — The Correction Protocol

RTK corrections are transmitted from base to rover using the **RTCM SC-104** binary protocol. Key message types:

| Message | Content |
|---------|---------|
| RTCM 1004 | L1/L2 GPS observations (pseudorange + phase) |
| RTCM 1012 | L1/L2 GLONASS observations |
| RTCM 1074/1084/1094/1124 | MSM4 — GPS/GLONASS/Galileo/BeiDou observations (modern, preferred; MSM7 variants are 1077/1087/1097/1127) |
| RTCM 1005/1006 | Base station antenna position |
| RTCM 1033 | Receiver/antenna descriptor |

The rover receives these, reconstructs the base observations, forms double differences, and runs LAMBDA. This is what the ESP32 is forwarding when it writes RTCM bytes to the ZED-F9P's UART.

RTCM is a one-way stream — the base has no knowledge of the rover. The bidirectional link in VRS (GGA back to server) is an NTRIP-layer concept, not part of RTCM itself.

---

## 7. Practical Performance Summary

| Condition | Expected accuracy |
|-----------|-----------------|
| Standalone GNSS (SPS) | 3–5 m CEP |
| SBAS (WAAS/EGNOS) | 1–3 m |
| RTK Float | 0.1–0.5 m |
| RTK Fixed, short baseline (<5 km) | 1–3 cm horizontal, 2–5 cm vertical |
| RTK Fixed, 30 km baseline | 2–5 cm horizontal (ionosphere-dependent) |
| RTK Fixed, >50 km baseline | Unreliable without network corrections |

Vertical accuracy is typically 1.5–2× worse than horizontal due to the geometry of overhead-only satellite coverage (poor vertical dilution of precision).

**Time to first fix:**
- Cold start to 3D: 30–60 s
- 3D to RTK Float: 10–60 s after corrections arrive
- Float to Fixed (dual-frequency, short baseline): 5–30 s
- Float to Fixed (single-frequency): minutes to tens of minutes

---

## 8. The ZED-F9P in This System

The ZED-F9P is a **concurrent dual-band** (L1 + L2) multi-constellation receiver. Relevant specifications:

- Tracks GPS L1C/A + L2C, GLONASS L1OF + L2OF, Galileo E1B/C + E5b, BeiDou B1I + B2I
- Moving baseline RTK accuracy: 1 cm + 1 ppm CEP (horizontal)
- RTK convergence: typically < 10 s on short baselines with good geometry
- Outputs UBX-NAV-PVT (position/velocity/time), UBX-RXM-RAWX (raw measurements for post-processing)
- Communicates with the OpenLog Artemis over I2C (address 0x42) using the SparkFun u-blox GNSS library
- Receives RTCM corrections from the ESP32 over UART

The `carrier_solution` field in UBX-NAV-PVT (and in the OLA CSV log) directly reflects the ambiguity state: 0 = none, 1 = float, 2 = fixed.

---

## Further Reading

- Teunissen, P.J.G. (1995). *The least-squares ambiguity decorrelation adjustment.* Journal of Geodesy, 70(1–2), 65–82. — Original LAMBDA paper
- Leick, A. (2015). *GPS Satellite Surveying*, 4th ed. Wiley. — Standard graduate reference
- Misra, P., Enge, P. (2006). *Global Positioning System: Signals, Measurements and Performance*, 2nd ed. — Signal model depth
- u-blox ZED-F9P Integration Manual — hardware-specific implementation details
