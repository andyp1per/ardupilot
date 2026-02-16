# logm2_5 Analysis

## Metadata
- **Date**: 2026-02-16
- **Vehicle**: TD-MicoAir-2 (MicoAir743v2)
- **Firmware**: V4.6.3v2-SFD (883e416a)
- **Branch**: SmallFastDrone-4.6-AltHold
- **Log file**: ./log5.bin
- **Sensors**: Optical flow, baro, dual IMU, rangefinder (MAVLink, 7m max)
- **No GPS, no compass** — fully GPS-denied
- **Flight #**: 2

## Parameters

Same as [logm2_4](logm2_4.md) — same vehicle and firmware.

Key values:
| Parameter | Value |
|-----------|-------|
| EK3_IMU_MASK | 1 (IMU0 only) |
| EK3_RNG_USE_HGT | -1 |
| BARO1_THST_SCALE | -147 |
| ACC_ZBIAS_LEARN | 0 (disabled) |
| INS_ACC_VRFB_Z | 0.041 |
| MOT_THST_HOVER | 0.3814 |
| MOT_HOVER_LEARN | 0 |

## Flight Timeline

Two armed segments. First brief (~11s in STABILIZE), second is the main flight.

| Offset (from ARM #2) | Event |
|-----------------------|-------|
| -12.7s | ARM #1 (STABILIZE, brief ground test) |
| -1.2s | DISARM #1, field elevation 232m |
| **+0.0s** | **ARM #2** (ALT_HOLD) |
| +0.7s | AUTO_ARMED |
| +2.3s | EKF yaw reset |
| +29.0s | EKF3 stopped aiding, restarted, fusing optical flow |
| **+47.7s** | **Mode: STABILIZE** (landing) |
| +52.5s | LAND_COMPLETE |
| **+54.8s** | **DISARMED** |
| +55.8s | EKF variance: position lost |

Total flight: ~55s. AltHold: ~47s.

## Altitude Hold Performance

### CTUN Statistics — Stable Hover Phase (45-70s)

| Metric | Mean | Std | Min | Max |
|--------|------|-----|-----|-----|
| Alt (EKF) | 3.749 m | **0.191 m** | — | — |
| ThO | 0.372 | — | — | — |
| Alt Error mean | +0.042 m | — | — | — |
| **Alt Error std** | **0.191 m** | — | — | — |

### Full Hover (28-75s)

| Metric | Mean | Std | Min | Max |
|--------|------|-----|-----|-----|
| Alt (EKF) | 4.080 m | 0.729 | 2.391 | 5.402 |
| DAlt (desired) | 3.865 m | 1.197 | 0.000 | 5.257 |
| BAlt (baro) | 3.809 m | 1.783 | **-7.420** | 5.840 |
| ThO | 0.355 | 0.069 | 0.000 | 0.514 |
| ThH | 0.3814 | 0.000 | — | — |

**BAlt excursion to -7.42m during climb** — severe baro-thrust coupling artifact despite
BARO1_THST_SCALE=-147. The compensation is not sufficient for this vehicle's thrust transients.

## Vibrations — CRITICAL FINDING

| IMU | VibeX Mean/Max | VibeY Mean/Max | VibeZ Mean/Max | Clip Events |
|-----|----------------|----------------|----------------|-------------|
| IMU0 | 3.74 / 6.31 | 3.87 / 6.71 | **10.42 / 15.03** | **9,870** |
| IMU1 | 0.85 / 5.36 | 0.99 / 5.61 | 1.45 / 8.39 | **0** |

**IMU0 has severe Z-axis vibration (mean 10.42 m/s², max 15.03) and massive clipping
(9,870 events).** IMU1 is dramatically better: Z mean=1.45, zero clips.

**The EKF is running on the worst IMU.** EK3_IMU_MASK=1 selects IMU0 only. Switching to
IMU1 (mask=2) or enabling both (mask=3) would significantly improve estimation quality.

## XKF3 Innovations

All velocity innovations stuck at constant values during hover:

| Innovation | Value |
|-----------|-------|
| IVN | +0.10 (constant) |
| IVE | -0.01 (constant) |
| IVD | -0.03 (constant) |
| IPD | -0.259 mean (varies, range -10.8 to +1.43) |

IVD stuck near zero — with no Z velocity source (EK3_SRC1_VELZ=0), there's nothing to
drive velocity innovations. IPD (baro height innovation) does vary and reaches -10.8m
during the worst baro-thrust transient.

## XKF4 Variance Ratios

SH (height test ratio) exceeded 1.0 in **21 samples** during hover, meaning baro innovations
were being rejected periodically. This indicates the EKF is struggling with baro data quality.

## Rangefinder

| Metric | Hover |
|--------|-------|
| Dist mean | 1.584 m |
| Dist std | 0.785 m |
| Status=4 (good) | 94.4% |

XKF5: HAGL=1.697m, offset=-2.368m (baro-to-rangefinder offset), Herr=0.111m.
Despite being available, rangefinder is not used for EKF height (EK3_RNG_USE_HGT=-1).

## Temperature

| Sensor | Min | Max | Range |
|--------|-----|-----|-------|
| BARO | 52.1 C | 57.1 C | 5.0 C |
| IMU0 | 48.8 C | 53.8 C | 5.0 C |
| IMU1 | 52.0 C | 57.7 C | 5.7 C |

Vehicle was already warm from log4 — no cold-start issue.

## Motor Outputs

| Motor | Mean | Std | Offset |
|-------|------|-----|--------|
| C1 | 1574 | 31 | +0.8% |
| C2 | 1521 | 79 | **-2.5%** |
| C3 | 1592 | 27 | **+2.0%** |
| C4 | 1558 | 50 | -0.2% |

Same CG offset pattern as log4. Motor 2 has highest std (79) and occasionally drops to
idle (1150), suggesting it's on the less loaded arm.

ESC RPM: 30-33k mean. ESC1 (Motor 2) has lowest mean (29,690) and highest std (4,110).

## Post-Landing

- PD drifts from -1.00 to -1.77 in 1.2s (0.64 m/s drift rate)
- "EKF variance: position lost" error
- BAlt reads ~1.8m on ground (persistent baro offset from flight thermal effects)

## Key Findings

1. **IMU0 Z-axis clipping is critical** — 9,870 clip events during hover, 2x recommended
   vibration limit. IMU1 is 6x better with zero clips. EKF should use IMU1.

2. **Baro-thrust coupling still severe** — BAlt excursion to -7.42m during climb despite
   THST_SCALE=-147. Compensation insufficient for thrust transients.

3. Altitude hold performance is reasonable in stable hover (19.1cm std) but significantly
   worse than log4 (2.5cm std). The IMU0 clipping likely contributes.

4. Height innovation test (SH) fails periodically — baro data quality is marginal.

5. Same post-disarm EKF collapse as log4.

## Recommendations

1. **Switch EK3_IMU_MASK to 2 (IMU1 only) or 3 (both)** — IMU0 is severely compromised
   by Z-axis vibration and clipping.

2. **Enable ACC_ZBIAS_LEARN** — the VRF learned value (0.041) is far too small given the
   actual vibration-induced bias visible in the clipping data.

3. Investigate IMU0 mounting — why is it so much worse than IMU1?

## See Also
- [logm2_4](logm2_4.md) — same vehicle, flight 1 (excellent performance)
- [logm2_6](logm2_6.md) — same vehicle, flight 3 (altitude failure)
