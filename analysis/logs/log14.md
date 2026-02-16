# log14 Analysis

## Metadata
- **Date**: 2026-02-16
- **Vehicle**: TD-Matek-5 outdoor (MatekH743-bdshot)
- **Firmware**: V4.6.3v2-SFD (5282cc3d)
- **Branch**: SmallFastDrone-4.6-AltHold
- **Log file**: ./log14.bin
- **Sensors**: GPS (u-blox SAM-M10Q), baro, dual IMU, rangefinder, optical flow (no data)
- **Flight #**: 8

## Parameters — Changes from Log11

Only learned values changed — no manual config changes:

| Parameter | Log11 | Log14 | Notes |
|-----------|-------|-------|-------|
| MOT_THST_HOVER | 0.1338 | **0.1426** | +6.6% (learned up) |
| INS_ACC_VRFB_Z | -0.3948 | **-0.3744** | IMU0 converging toward zero |
| INS_ACC2_VRFB_Z | -0.2221 | **-0.2401** | IMU1 moved slightly away |

Compass NOT relocated — same offsets, same DEV_ID=855305.

## Flight Timeline

| Offset (from ARM) | Event |
|--------------------|-------|
| -2.6s | EKF3 IMU0/IMU1 initialized |
| **+0.0s** | **ARMED** (Stabilize) |
| +1.1s | Takeoff |
| +4.0s | Ground mag anomaly, yaw re-aligned |
| **+9.7s** | **Mode: AltHold** |
| +88.3s | GPS 3D fix acquired (4 sats, HDop=3.05) |
| **+97.8s** | **Mode: Stabilize** (landing) |
| **+104.1s** | **DISARMED** |

Total flight: 104s. AltHold: 88s.

**No EKF lane switch** — stayed on Core 0 the entire flight (improvement over log11 which
switched C0→C1 at +10s).

**GPS nearly absent**: no 3D fix for the first 88.3s. Even after fix, only 4 sats with
HDop=3.05 — GPS quality too marginal for EKF to use. Velocity innovations frozen for 95
of 104 seconds. Much worse than log11's ~51.5s delay.

## Altitude Hold Performance

### CTUN Statistics (stable AltHold, 802 samples)

| Metric | Mean | Std | Min | Max |
|--------|------|-----|-----|-----|
| Alt (EKF) | 1.898 m | 0.077 | 1.704 | 1.993 |
| DAlt (desired) | 2.013 m | — | — | — |
| BAlt (baro) | 1.817 m | 0.242 | — | — |
| ThO | 0.141 | 0.002 | 0.136 | 0.145 |
| ThH | 0.140 | 0.002 | 0.128 | 0.134 |
| **Alt Error mean** | **+0.115 m** | — | — | — |
| **Alt Error std** | **0.077 m** | — | 0.020 | 0.309 |

**7.7cm alt error std — 50% improvement over log11 (15.5cm).** Mean bias of +11.5cm
(vehicle consistently ~11.5cm below target). Excellent control with very tight ThO
variation (std=0.002).

## Vibrations

| IMU | VibeX Mean/Max | VibeY Mean/Max | VibeZ Mean/Max | Clip |
|-----|----------------|----------------|----------------|------|
| IMU0 | 3.82 / 6.11 | 5.38 / 6.94 | 2.63 / 3.73 | **0** |
| IMU1 | 2.60 / 4.63 | 3.10 / 4.35 | 1.64 / 3.00 | **0** |

Zero clipping. Low vibration. IMU1 quieter across all axes.

## XKF3 Innovations

### Velocity Innovations — FROZEN (no GPS)

Both cores frozen from ~T+6s to T+101s:

| Core | IVN (frozen) | IVE (frozen) | IVD (frozen) |
|------|-------------|-------------|-------------|
| C0 | -0.25 | +0.37 | +0.07 |
| C1 | -0.26 | +0.43 | -0.01 |

GPS had no fix for 88.3s. Even after 3D fix (4 sats, HDop=3.05), EK3_GPS_CHECK quality
gates prevented fusion. Innovations only unfroze at T+101.2s during landing descent.

### Magnetometer Innovations — Still Stuck

| Field | Log11 | Log14 | Change |
|-------|-------|-------|--------|
| IMX | +103 | **+108** | +5 |
| IMY | -43 | **+14** | **+57 (huge shift)** |
| IMZ | -263 | **-252** | +11 |
| Total | 286 mG | **275 mG** | -4% |

All stuck at constant values for entire hover (std=0.0). The IMY shift of +57 mGauss
between flights is unexplained since the compass wasn't relocated. Total residual still
enormous at 275 mGauss.

## XKF4 Variance Ratios

| Core | SV mean/max | SP mean/max | SH mean/max | SM mean/max |
|------|------------|------------|------------|------------|
| C0 | 0.060 / 0.060 | 0.006 / — | 0.015 / 0.110 | 0.014 / — |
| C1 | 0.060 / 0.060 | 0.009 / — | 0.020 / — | 0.014 / — |

**SH > 1.0: zero samples.** Height variance excellent.

SV locked at 0.060 — no GPS velocity fusion reducing it. When GPS finally kicked in at
T+101.2s, SV dropped to 0.010.

## Rangefinder

| Metric | Value |
|--------|-------|
| Hover samples | 1603 |
| Status Good | **100%** |
| Dist mean | 1.622 m |
| Dist std | 0.240 m |

## Temperature

| Sensor | Min | Max | Range |
|--------|-----|-----|-------|
| BARO | 39.1 C | 50.1 C | 11.0 C |
| IMU0 | 27.4 C | 35.6 C | 8.2 C |
| IMU1 | 28.9 C | 37.6 C | 8.7 C |

Cold start — baro started at 39°C vs 53°C in log11. Larger thermal drift.

## Motor Outputs (DShot)

| Motor | Mean | Std | Offset |
|-------|------|-----|--------|
| C1 | 1345 | 22 | -0.1% |
| C2 | 1336 | 20 | -0.7% |
| C3 | 1390 | 19 | **+3.3%** |
| C4 | 1314 | 25 | **-2.4%** |

C3/C4 asymmetry slightly more pronounced than log11. Same CG offset pattern.

## ESC RPM

| ESC | Mean RPM | Std |
|-----|----------|-----|
| ESC0 | 11,923 | 500 |
| ESC1 | 11,481 | 537 |
| ESC2 | 12,799 | 547 |
| ESC3 | 10,974 | 547 |

Mean hover RPM: 11,794 (197 Hz fundamental). Notch at 150 Hz base tracking via ESC RPM.

## AccZ Bias

| Core | AZ start | AZ settled | VRFB |
|------|----------|-----------|------|
| C0 | -0.040 | +0.020 | -0.3744 |
| C1 | -0.040 | -0.010 | -0.2401 |

Both cores settled quickly. Bias learning working correctly.

## Post-Landing

- EKF alt at disarm: -0.918m
- **Post-disarm drift: +1.59m in 3.3s** (worse than log11's -0.67m)
- BAlt spike to -9.26m during landing (severe baro transient)
- GPS fix lost at disarm (T+103.7s)

## Key Findings

1. **Alt error std improved 50%**: 7.7cm vs 15.5cm in log11 — best measured performance
2. **No EKF lane switch** — Core 0 stable the entire flight
3. **GPS nearly absent** — no fix for 88s, velocity innovations frozen for 95s
4. **Compass innovations still stuck** — total residual 275 mGauss, same pattern as log11
5. **Post-landing drift worse** — 1.59m vs 0.67m, driven by severe baro transient (-9.26m)
6. **Optical flow still not working** — zero samples despite FLOW_TYPE=6

## See Also
- [log11](log11.md) — previous flight
- [log15](log15.md) — next flight (compass improvement)
