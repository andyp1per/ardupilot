# logm2_6 Analysis

## Metadata
- **Date**: 2026-02-16
- **Vehicle**: TD-MicoAir-2 (MicoAir743v2)
- **Firmware**: V4.6.3v2-SFD (883e416a)
- **Branch**: SmallFastDrone-4.6-AltHold
- **Log file**: ./log6.bin
- **Sensors**: Optical flow, baro, dual IMU, rangefinder (MAVLink, 7m max)
- **No GPS, no compass** — fully GPS-denied
- **Flight #**: 3

## Parameters

Same as [logm2_4](logm2_4.md) — no parameter changes across flights.

## Flight Timeline

Two short flights in this log. First is the critical one.

| Offset (from ARM #1) | Event |
|-----------------------|-------|
| -3.0s | Boot, EKF initialized |
| **+0.0s** | **ARM #1** (STABILIZE) |
| +0.7s | LAND_COMPLETE |
| +1.0s | LANDED_STATE |
| +3.8s | EKF3 started relative aiding |
| **+8.4s** | **Mode: ALT_HOLD** |
| +18.0s | EKF origin set, "GPS Denied Environment" |
| **+30.3s** | **Mode: STABILIZE** (pilot aborted) |
| +35.7s | Brief ALT_HOLD (0.9s) |
| +36.6s | Back to STABILIZE |
| **+43.5s** | **Brief ALT_HOLD — FULL THROTTLE** (0.8s) |
| +43.5s | EKF thinks vehicle is 25m underground → commands max climb |
| +48.5s | LAND_COMPLETE |
| **+49.0s** | **DISARM #1** |
| +50.0s | EKF variance: position lost |
| — | ARM #2 (brief STABILIZE, 8.8s, no altitude issues) |

Total AltHold hover: ~22s. Flight terminated early due to altitude estimation failure.

## Altitude Hold Performance — SEVERE FAILURE

### CTUN Statistics (AltHold, +8.4s to +30.3s)

219 samples, ~22s.

| Metric | Mean | Std | Min | Max |
|--------|------|-----|-----|-----|
| Alt (EKF) | 0.168 m | 0.401 | -1.350 | 0.719 |
| DAlt (desired) | -1.015 m | 0.336 | -1.775 | 0.000 |
| BAlt (baro) | 3.436 m | 1.211 | 2.200 | **10.750** |
| ThO | 0.383 | 0.038 | 0.000 | 0.617 |
| CRt (climb rate) | **-167 cm/s** | 42.0 | -362 | -84 |

**Alt Error mean = +1.183 m, std = 0.318 m**

**The EKF believes the vehicle is descending at 1.67 m/s while actually hovering at 2-3m.**
The rangefinder confirms the vehicle is stable at ~2.35m AGL, but the EKF's vertical
velocity estimate is completely wrong.

## Root Cause: Cascading Altitude Estimation Failure

### 1. Massive AccZ Bias (-0.62 m/s²)

XKF2 AZ (accel Z bias estimate) during hover:
- Starts at 0.0 at boot
- Rapidly grows to -0.54 by t=9s
- Stabilizes at **-0.62 m/s²** during hover

This means the EKF corrects the IMU reading by -0.62, making the effective AccZ =
-9.78 + 0.62 = -9.16 m/s² — less than gravity. The EKF concludes the vehicle must be
in net downward acceleration (falling), producing the false descent velocity.

The INS_ACC_VRFB_Z correction is only 0.041 m/s² — **15x smaller than the actual bias**.

### 2. Stuck IVD Innovation (0.64)

| Innovation | Value | Notes |
|-----------|-------|-------|
| IVN | -0.19 (constant) | Stuck |
| IVE | -0.55 (constant) | Stuck |
| **IVD** | **+0.64 (constant)** | **Stuck — saturated, can't correct VD** |
| IPD | +3.23 mean (varies) | Baro tries to correct, peaks at +8.52 |

The IVD innovation is saturated/gated at 0.64 and cannot drive the velocity estimate
back toward reality. The baro IPD innovation averages +3.2m (baro says higher than EKF
estimates) but the correction is insufficient against the persistent AccZ bias.

### 3. Baro Thermal Drift (+3.5m in 20s)

BAlt rises from 2.3m to 5.8m during the 22s hover (baro temp: 51.9→55.5°C, +3.6°C).
This 3.5m drift fights against the EKF's belief that the vehicle is descending.

### 4. No Rangefinder Height Source

EK3_RNG_USE_HGT=-1 means the working rangefinder (2.35m mean, 95% good status) is
**not used for height estimation**. If enabled, it would have prevented this failure.

### 5. No VelZ Source

EK3_SRC1_VELZ=0 means there is no independent velocity-Z measurement. The EKF relies
entirely on integrating biased accelerometers and correcting with thermally-drifting baro.

### 6. Single Core on Worst IMU

EK3_IMU_MASK=1 uses only IMU0, which has **10.6 m/s² mean Z vibration** (2x limit).
IMU1 has 6x less vibration and zero clipping.

## What Happened After AltHold

After the pilot switched to STABILIZE at +30.3s:
- EKF PD continued integrating the false descent velocity
- PD reached **+52.6m** (EKF thinks vehicle is 52m underground) by t=50.7s
- IPD innovation reached **+54.9m**
- Brief AltHold at +43.5s: ThO hit **1.000 (FULL THROTTLE)** — EKF commanded max climb
  because it believed the vehicle was 25m underground
- The pilot saved the situation by immediately switching back to STABILIZE

## XKF4 Variance Ratios

SH (height test) during hover: mean=0.77, rising from 0.47 to **1.59** by end of hover.
SH exceeded 1.0 in **54/219 samples (25%)** — a quarter of baro innovations were failing.
After AltHold, SH reached **8.49** — complete height estimation failure.

## Vibrations

| IMU | VibeZ Mean | VibeZ Max | Clip |
|-----|-----------|-----------|------|
| IMU0 | **10.64** | **29.09** | 29 (during landing) |
| IMU1 | 1.72 | 27.53 | 0 |

IMU0 Z vibration consistently >10 m/s² during hover. Same pattern as log5.

## Rangefinder

During AltHold hover (438 samples):
| Metric | Value |
|--------|-------|
| Dist mean | 2.35 m |
| Dist std | 0.82 m |
| Status=4 (good) | 95% |

**Rangefinder confirms stable hover at ~2-3m** while EKF reports continuous descent.
This is the most damning evidence — the rangefinder shows the truth but the EKF ignores it.

## Temperature

| Sensor | Min | Max | Range |
|--------|-----|-----|-------|
| BARO | 51.2 C | 57.5 C | 6.4 C |
| IMU0 | 47.5 C | 54.3 C | 6.8 C |
| IMU1 | 51.7 C | 58.1 C | 6.4 C |

Both IMUs operating 4-5°C below their TCAL calibration temperatures.

## Motor Outputs

| Motor | Mean | Std | Offset |
|-------|------|-----|--------|
| C1 | 1587 | 35 | +0.4% |
| C2 | 1547 | 34 | -2.1% |
| C3 | 1605 | 36 | +1.6% |
| C4 | 1581 | 41 | 0.0% |

Same asymmetry pattern. ESC RPM: 30-33k mean. Notch center at 475 Hz is 61 Hz below
actual fundamental (~536 Hz) but within the 235 Hz bandwidth.

## Comparison: logm2_4 → logm2_5 → logm2_6

| Metric | logm2_4 | logm2_5 | logm2_6 |
|--------|---------|---------|---------|
| AltHold duration | 108 s | 47 s | **22 s** |
| Alt error std | **0.025 m** | 0.191 m | **FAILED** |
| IMU0 Z vibe mean | 7.23 | **10.42** | **10.64** |
| IMU0 clips | 0 | **9,870** | 29 |
| CRt mean | +2.2 cm/s | -0.1 cm/s | **-167 cm/s** |
| EKF AccZ bias | — | — | **-0.62 m/s²** |
| IVD | -0.43 stuck | -0.03 stuck | **+0.64 stuck** |
| SH failures | — | 21 | **54 (25%)** |
| Outcome | **Excellent** | OK | **FAILURE** |

The progressive degradation from log4 → log5 → log6 suggests either increasing IMU0
vibration sensitivity or accumulating thermal/calibration drift between flights.

## Recommendations

**Immediate (critical):**
1. **EK3_IMU_MASK = 2 or 3** — stop using IMU0 as primary; IMU1 has 6x less Z vibration
2. **Enable rangefinder for height** — EK3_RNG_USE_HGT=70 (use below 70% of max range)
3. **ACC_ZBIAS_LEARN = 2** — enable Z-bias learning (the 0.041 VRF is 15x too small)

**Important:**
4. Investigate IMU0 Z-axis vibration source — mechanical mounting issue?
5. Consider EK3_SRC1_VELZ=3 (GPS) — oh wait, no GPS. Consider if rangefinder rate
   can provide velocity Z.
6. Re-center notch filter: actual RPM fundamental ~536 Hz, notch center 475 Hz

**Nice to have:**
7. Fix Turtle_Mode.lua error (repeats every 10s in all logs)
8. ARMING_CHECK should be more than 64 (RC only)

## See Also
- [logm2_4](logm2_4.md) — same vehicle, flight 1 (excellent performance)
- [logm2_5](logm2_5.md) — same vehicle, flight 2 (clipping discovered)
