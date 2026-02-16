# log11 Analysis

## Metadata
- **Date**: 2026-02-16
- **Vehicle**: TD-Matek-5 outdoor (MatekH743-bdshot)
- **Firmware**: V4.6.3v2-SFD (5282cc3d)
- **Branch**: SmallFastDrone-4.6-AltHold
- **Log file**: ./log11.bin
- **Sensors**: GPS (u-blox SAM-M10Q), baro, dual IMU, rangefinder (Orient=25, downward)
- **Flight #**: 6 (first with GNDEFF_TMO and ARMING_CHECK applied)

## Parameters — Changes from Previous Flights

**Applied (from pending list):**
| Parameter | Old | New | Status |
|-----------|-----|-----|--------|
| TKOFF_GNDEFF_TMO | 0 | **3.0** | Applied |
| ARMING_CHECK | 64 | **1** | Applied (all checks) |

**NOT applied:**
| Parameter | Value | Expected |
|-----------|-------|----------|
| INS_ACC_VRFB_Z | -0.3948 | 0 (but see note below) |
| INS_ACC2_VRFB_Z | -0.2221 | 0 |

Note: The VRFB values are now productively learned values from ACC_ZBIAS_LEARN=3. Zeroing
them would regress — they should be left as-is. The original "zero and re-learn" advice was
for the corrupted -0.343 value on log3/log4; these are genuine learned biases.

**Other changes:**
| Parameter | Previous | log11 |
|-----------|----------|-------|
| MOT_THST_HOVER | 0.146 | **0.1338** (learned down) |

**Key config (unchanged):**
| Parameter | Value |
|-----------|-------|
| EK3_RNG_USE_HGT | -1 |
| BARO1_THST_SCALE | -100 |
| EK3_MAG_CAL | 7 |
| COMPASS_MOTCT | 2 |
| ACC_ZBIAS_LEARN | 3 |
| EK3_SRC1_VELZ | 3 (GPS) |
| EK3_IMU_MASK | 3 (dual) |
| INS_HNTCH_MODE | 3 (ESC RPM) |
| INS_HNTCH_FREQ | 150 Hz |

**Compass motor compensation:** COMPASS_MOT_X=-61.4, MOT_Y=1.0, MOT_Z=89.5

## Flight Timeline

| Offset (from ARM) | Event |
|--------------------|-------|
| -17.9s | Boot, Frame QUAD/X, Lua scripts loaded |
| -5.6s | EKF3 IMU0/IMU1 initialized, AHRS active |
| -3.9s | Tilt alignment complete |
| **+0.0s** | **ARMED** (AltHold) |
| +2.4s | Takeoff |
| +4.5s | EKF3 origin set |
| **+10.0s** | **EKF3 lane switch 0→1** |
| +17.0s | Ground mag anomaly, yaw re-aligned (both cores) |
| +51.5s | EKF3 IMU0/IMU1 using GPS |
| **+100.3s** | **Mode: STABILIZE** (landing) |
| +104.2s | **EKF3 lane switch 1→0** |
| +104.9s | Landing detected |
| **+105.3s** | **DISARMED**, Field Elevation 145m |

Total flight: ~105s. AltHold: ~100s.

Key observations:
- EKF lane switch C0→C1 at +10s — Core 0 had early issues (likely compass-related)
- Ground mag anomaly at +17s — compass interference from motors detected
- GPS velocity fusion delayed to +51.5s — very late
- Lane switch back C1→C0 just before disarm
- No EKF failsafe (improvement over log5)

## Altitude Hold Performance

### CTUN Statistics (AltHold, ARM+10s to STABILIZE, ~89s, 878 samples)

| Metric | Mean | Std | Min | Max |
|--------|------|-----|-----|-----|
| Alt (EKF) | 1.869 m | 0.608 | -1.119 | 2.275 |
| DAlt (desired) | 1.883 m | 0.601 | -0.739 | 2.036 |
| BAlt (baro) | 1.759 m | 0.898 | **-6.720** | 2.760 |
| ThO | 0.130 | 0.008 | 0.052 | 0.151 |
| ThH | 0.130 | 0.002 | 0.128 | 0.134 |
| **Alt Error mean** | **+0.014 m** | — | — | — |
| **Alt Error std** | **0.155 m** | — | -0.551 | +0.380 |

**15.5cm alt error std — good outdoor performance.** Mean error of +14mm.

ThH learning: 0.128→0.134 during flight (+0.006). MOT_THST_HOVER saved as 0.1338.

BAlt excursion to -6.72m during early arm/takeoff — baro-thrust coupling artifact.

## Vibrations

| IMU | VibeX Mean/Max | VibeY Mean/Max | VibeZ Mean/Max | Clip |
|-----|----------------|----------------|----------------|------|
| IMU0 | 4.00 / 8.45 | 5.08 / 7.07 | 2.60 / 11.12 | **0** |
| IMU1 | 2.79 / 6.60 | 3.17 / 5.00 | 1.88 / 11.90 | **0** |

**Zero clipping on both IMUs.** Vibrations within acceptable range. IMU1 lower across all axes.

## XKF3 Innovations

### Velocity Innovations (hover)

| Core | IVN mean/std | IVE mean/std | IVD mean/std |
|------|-------------|-------------|-------------|
| C0 | +0.032 / 0.178 | +0.007 / 0.206 | -0.146 / 0.311 |
| C1 | +0.042 / 0.181 | +0.020 / 0.207 | -0.151 / 0.292 |

Velocity innovations are **not stuck** — they vary with GPS updates (~3 Hz update rate
accounts for the ~70% "constant" samples between updates). This is NOT the Core 0 velocity
fusion freeze seen in log7.

IVD mean of -0.15 — slight negative bias, not pathological.

IPD range: -7.77 to +1.89 (one large spike during takeoff baro-GPS disagreement).

### Magnetometer Innovations — STUCK

| Innovation | C0 Value | C1 Value | Notes |
|-----------|----------|----------|-------|
| IMX | **+103** (constant) | +103 | Stuck |
| IMY | **-43** (constant) | -43 | Stuck |
| IMZ | **-263** (constant) | -263 | **Stuck — enormous** |

Magnetometer innovations are stuck at large constant values throughout hover. The motor
compensation (MOX=-448, MOZ=+653 mGauss mean) is applying corrections of 400-650 mGauss
but the residual interference is still 100-263 mGauss. The EKF cannot reconcile compass
readings with its state estimate.

## XKF4 Variance Ratios

| Core | SV mean/max | SP mean/max | SH mean/max | SM mean/max |
|------|------------|------------|------------|------------|
| C0 | 0.079 / 0.670 | 0.018 / 0.080 | 0.045 / 1.240 | 0.035 / 0.460 |
| C1 | 0.079 / 0.670 | 0.019 / 0.080 | 0.045 / 1.240 | 0.035 / 0.270 |

**SH > 1.0: 1/900 samples (0.1%)** — excellent height variance control.

Post-disarm: SV peaks 0.29, SM peaks 0.52. No failsafe thresholds approached.

## Rangefinder

| Metric | Value |
|--------|-------|
| Hover samples | 1800 |
| Dist mean | 1.844 m |
| Dist std | 0.623 m |
| Status Good | 99.4% |
| OutRangeLow | 0.6% |

With EK3_RNG_USE_HGT=-1, rangefinder is contributing to AltHold height — consistent with
good altitude hold performance.

## Temperature

| Sensor | Min | Max | Range |
|--------|-----|-----|-------|
| BARO | 53.1 C | 58.6 C | 5.5 C |
| IMU0 | 36.6 C | 43.8 C | 7.3 C |
| IMU1 | 38.5 C | 45.3 C | 6.8 C |

Baro runs notably hotter than IMUs (53-59°C vs 37-45°C). 5°C baro drift during flight.

## Motor Outputs (DShot)

| Motor | Mean | Std | Offset |
|-------|------|-----|--------|
| C1 | 1344 | 33 | +1.1% |
| C2 | 1304 | 33 | -1.8% |
| C3 | 1356 | 41 | +2.0% |
| C4 | 1311 | 34 | -1.3% |

Same asymmetry pattern as previous flights (C1/C3 high, C2/C4 low). Within 2%.

## ESC RPM

| ESC | Mean RPM | Std |
|-----|----------|-----|
| ESC0 | 12,256 | 825 |
| ESC1 | 11,013 | 902 |
| ESC2 | 12,317 | 1,108 |
| ESC3 | 11,228 | 825 |

**Overall mean: 11,703 RPM (195 Hz fundamental).** INS_HNTCH_FREQ=150 Hz base with ESC
telemetry tracking (MODE=3) — notch tracks correctly upward from base.

## AccZ Bias (XKF2)

| Core | AZ mean | AZ start→end | Range |
|------|---------|-------------|-------|
| C0 | -0.009 | -0.069 → -0.002 | [-0.15, +0.02] |
| C1 | -0.012 | -0.059 → -0.026 | [-0.13, +0.01] |

**AccZ bias learning is working.** Both cores converging toward zero during hover. The
VRFB values (-0.395 IMU0, -0.222 IMU1) are being used productively at boot.

## Compass Analysis

**Yaw consistency (hover):**
- C0 yaw mean: 320.9°, std 2.91°
- C1 yaw mean: 322.8°, std 2.83°
- **Difference: 1.97° mean, max 2.51°** — excellent in-flight convergence

**Boot yaw inconsistency: 29°** — resolves during flight as EKF learns offsets.

**Motor compensation applied:** MOX=-448 mGauss, MOZ=+653 mGauss mean — corrections of
400-650 mGauss with residuals of 100-263 mGauss. The interference exceeds compensation
capability.

## Post-Landing

- Position drift: dD=-0.56 to -0.67m over 20 post-disarm samples
- Velocity: VD=-0.35 to -0.75 m/s post-disarm
- SM peaks at 0.52 (compass variance highest after motor stop)
- **Significantly milder** than previous flights — no EKF failsafe triggered

## Key Findings

**Improvements from previous flights:**
1. TKOFF_GNDEFF_TMO=3.0 and ARMING_CHECK=1 successfully applied
2. No EKF failsafe (log5 had failsafe)
3. No velocity fusion freeze (log7 had Core 0 freeze)
4. AccZ bias learning working with genuine VRFB values
5. Post-landing EKF behavior much milder
6. Zero IMU clipping on both IMUs

**Persistent issues:**
1. **Compass interference remains #1 issue** — IMX/IMY/IMZ innovations stuck at +103/-43/-263
   mGauss throughout hover. Motor compensation applies 400-650 mGauss but residual is still
   enormous. Ground mag anomaly triggered at +17s.
2. EKF lane switch at +10s (C0→C1) — compass-related early instability
3. GPS velocity fusion delayed to +51.5s
4. Boot yaw inconsistency of 29° (resolves in flight)
5. Optical flow configured (FLOW_TYPE=6) but **zero flow samples** — sensor not providing data

## See Also
- [log7](log7.md) — previous flight (best outdoor TD-Matek-5)
- [log12](log12.md) — ground session following this flight
