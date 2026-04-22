# How Polaris Models RTK Corrections Across the Network

## The core problem network RTK solves

Single-baseline RTK works well within ~30 km of a physical base. Beyond that, **distance-dependent errors** grow:

- **Ionospheric delay** — free electrons in the ionosphere slow the signal by an amount that varies spatially. Two receivers 50 km apart see meaningfully different delays, so the base station's correction becomes wrong for the rover.
- **Tropospheric delay** — water vapor and pressure vary with location and altitude, adding a few cm of range error that also does not cancel at distance.
- **Orbit/clock errors** — mostly common-mode, largely handled by precise ephemeris.

Network RTK's job is to estimate these error fields continuously across the coverage area and synthesize corrections that are valid *at the rover's location*, not at some base station kilometers away.

---

## What each reference station contributes

Each Polaris station has a **known position** and continuously observes carrier phase and pseudorange. Since the position is known, the station can compute its own measurement residuals — the difference between what it observes and what physics predicts. Those residuals are dominated by the spatially-varying ionospheric and tropospheric delays.

The network software ingests all of these simultaneously and builds a **spatial model of the error field** across the coverage area.

---

## The interpolation: from N stations to continuous coverage

At 30–40 km spacing, the ionospheric gradient across any single baseline is small (typically 1–5 cm). This means **linear interpolation** between surrounding stations is sufficient for most conditions:

```
error at rover ≈ weighted combination of residuals at surrounding stations
                  (weights = inverse distance or plane fit)
```

More rigorously, the ionospheric delay is modeled as a thin shell at ~350 km altitude (the **single-layer model**). The network estimates the **Vertical Total Electron Content (VTEC)** gradient across the coverage area and maps it to each satellite's line of sight using an obliquity factor. The troposphere uses an empirical model (Saastamoinen or similar) corrected by the network residuals.

The ambiguity resolution piece is what separates good network RTK from naive interpolation — the network must resolve **integer carrier-phase ambiguities across all baselines simultaneously** (wide-lane first, then narrow-lane) before the error field estimates are reliable. This is computationally expensive and is why these are cloud-hosted systems.

---

## Why VRS specifically — and why GGA is required

Point One uses the **Virtual Reference Station (VRS)** approach rather than alternatives like FKP or MAC:

| Method | How it works | Needs GGA from rover? |
|--------|-------------|----------------------|
| **VRS** | Server synthesizes fake RTCM as if a physical base exists 1–2 km from the rover | **Yes** — server needs rover position to place the virtual station |
| **FKP** | Server sends correction gradients; rover applies them locally | No |
| **MAC** | Server sends master + auxiliary differences; rover reconstructs the error field | No |

VRS is dominant because the rover sees it as a standard single-baseline problem — no special firmware required, works with any RTK receiver out of the box. The trade-off is the mandatory bidirectional link: **the rover must send an NMEA GGA sentence to the server**, which tells Polaris where to synthesize the virtual base station.

This is why `esp32_polaris.ino` sends GGA immediately after connecting and refreshes it every 5 minutes. For a wave buoy, more frequent updates are unnecessary — the buoy drifts slowly relative to the spatial scale of the VRS correction field (several kilometres), so the virtual base station position does not need to move on a sub-minute timescale. Without the initial GGA, however, Polaris has no rover position and cannot compute corrections at all.

---

## What the 30–40 km station spacing means in practice

At that density, linear interpolation of the ionospheric field introduces errors of roughly **1–3 mm** under typical conditions — well below the carrier-phase noise floor. The network density is genuinely sufficient for 1–3 cm RTK accuracy anywhere in the coverage footprint, not just near physical stations.

Sparser networks (e.g. 100 km spacing) require higher-order ionospheric models and degrade under active ionospheric conditions (solar storms, equatorial anomaly). Polaris's dense self-owned infrastructure is what makes their accuracy claim hold up across the US.

---

## Relevance to the wave buoy

Polaris synthesizes a virtual base station near the buoy's current position. Its real advantages over a single campus base station (like the UCSD/SIO server) are:

- **Denser network** — 30–40 km station spacing means the nearest physical stations are closer, so the interpolated corrections are more accurate for a buoy operating in the Southern California nearshore zone
- **Resilience** — not dependent on a single campus server being online
- **Portability** — works for deployments anywhere in the US, not just near UCSD

However, **all Polaris reference stations are on land**. For a buoy well offshore (>50–100 km into the Pacific), there are no stations to the west and the interpolation becomes extrapolation — accuracy degrades just as it does with single-baseline RTK. Neither approach is well-suited to truly offshore open-ocean deployments.

| Scenario | UCSD/SIO base | Polaris |
|----------|--------------|---------|
| Nearshore (<15 km from Scripps) | Fine | Fine |
| Moderate offshore (15–50 km) | Degrading | Better |
| Far offshore (>50–100 km Pacific) | Poor | Also poor |

For far offshore work, **PPP (Precise Point Positioning)** using satellite-delivered corrections (e.g. u-blox PointPerfect, Fugro StarFire) or a vessel-based local reference station would be the appropriate approach.
