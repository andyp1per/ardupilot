# TD-MicoAir-2 — Vehicle Notes

## Vehicle Description

- **Frame**: Quad X (FRAME_CLASS=1, FRAME_TYPE=1)
- **FC**: MicoAir743v2 (STM32H743, APJ=1179)
- **Firmware**: V4.6.3v2-SFD (latest: 33d99563)
- **Board orientation**: YAW45 (AHRS_ORIENTATION=1)
- **IMU**: Dual IMU — EK3_IMU_MASK=3 (both IMUs), EK3_PRIMARY=1 (Core 1 / IMU1 forced)
- **Baro**: On-board, BARO1_THST_SCALE=-147
- **Rangefinder**: MAVLink type (RNGFND1_TYPE=10), 7m max, Orient=25 (downward)
- **GPS**: Disabled (GPS1_TYPE=0) — fully GPS-denied
- **Compass**: Disabled (COMPASS_ENABLE=0) — yaw from EKF GSF
- **Optical flow**: EK3_SRC1_VELXY=5 (primary XY velocity source)
- **MOT_THST_HOVER**: 0.3814 (~38% hover throttle)
- **Motor RPM**: ~30-33k (high KV)
- **Notch filter**: INS_HNTCH_FREQ=475 Hz, BW=235 Hz, MODE=1 (throttle), OPTS=6

## Flight Log Summary

| # | Log | Date | AltHold (s) | Alt Err Std | Key Finding |
|---|-----|------|-------------|-------------|-------------|
| 1 | [logm2_4](../logs/logm2_4.md) | Feb 16 | 108 | **2.5 cm** | Excellent — rangefinder aiding height despite RNG_USE_HGT=-1 |
| 2 | [logm2_5](../logs/logm2_5.md) | Feb 16 | 47 | 19.1 cm | IMU0 clipping (9,870 events), Z vibe 10.4 m/s²; IMU1 6x better |
| 3 | [logm2_6](../logs/logm2_6.md) | Feb 16 | 22 | **FAILED** | AccZ bias -0.62 m/s², EKF thinks 25m underground, full throttle panic |
| 4 | [logm2_log4](../logs/logm2_log4.md) | Feb 17 | ~30 | **CEILING** | Terrain lockout on Core 1 — flow lost 4s after takeoff, 38s dead reckoning, ceiling hit |

Flights 1-3: no parameter changes. Flight 4: major parameter changes (see logm2_log4).

## Current Configuration (as of logm2_log4)

```
# Height source
EK3_SRC1_POSZ        = 1        (baro)
EK3_SRC1_VELZ        = 0        (NONE — no Z velocity source)
EK3_RNG_USE_HGT     = 10       (rangefinder below 10% max = 0.7m — PROBLEM)
EK3_ALT_M_NSE       = 1.5
EK3_HGT_I_GATE      = 150
EK3_ACC_P_NSE       = 0.05     (7x below default 0.35 — too low)

# Position/velocity source
EK3_SRC1_POSXY       = 0        (none)
EK3_SRC1_VELXY       = 5        (optical flow)
EK3_SRC1_YAW         = 0        (none — GSF yaw)

# EKF config
EK3_IMU_MASK         = 3        (both IMUs)
EK3_PRIMARY          = 1        (forced Core 1 / IMU1 — PROBLEM)
EK3_GND_EFF_DZ       = 7

# Baro compensation
BARO1_THST_SCALE    = -147

# Position controller
PSC_POSZ_P          = 1.25
PSC_VELZ_P          = 6.5
PSC_JERK_Z          = 30       (6x default — PROBLEM)

# Accel bias
INS_ACC_VRFB_Z      = 0.019
ACC_ZBIAS_LEARN     = 3        (enabled)

# Ground effect
TKOFF_GNDEFF_ALT    = 0.5
TKOFF_GNDEFF_TMO    = 3.0

# Motor
MOT_THST_HOVER      = 0.3814
MOT_HOVER_LEARN     = 0        (disabled)

# Notch
INS_HNTCH_FREQ      = 475
INS_HNTCH_BW        = 235
INS_HNTCH_MODE      = 1        (throttle)
INS_HNTCH_OPTS      = 6        (multi-source + loop rate)

# Arming
ARMING_CHECK        = 64       (RC only)
COMPASS_ENABLE      = 0
GPS1_TYPE           = 0
```

## Flight History

```
logm2_4      [=]          2.5 cm  — EXCELLENT, 108s hover
logm2_5      [======]    19.1 cm  — degraded, 47s, IMU0 clipping discovered
logm2_6      [XXXXXXXXX] FAILED   — 22s, cascading altitude failure (AccZ bias)
logm2_log4   [XXXXXXXXX] CEILING  — terrain lockout on Core 1, 38s dead reckoning
```

### Progressive Degradation (logm2_4 → logm2_5 → logm2_6)

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

The degradation from flight to flight suggests increasing IMU0 vibration sensitivity,
possibly from thermal effects on the IMU mounting or progressive loosening.

### logm2_log4 — New Failure Mode

After parameter changes to address the above issues (EK3_IMU_MASK=3, EK3_PRIMARY=1,
ACC_ZBIAS_LEARN=3, EK3_RNG_USE_HGT=10), a **different** failure occurred: the per-core
terrain estimator on Core 1 locked out within 4 seconds of takeoff, leaving Core 1 flying
dead reckoning for 38 seconds. When the EKF reset aiding, the altitude jump caused a
ceiling hit. See [logm2_log4](../logs/logm2_log4.md) for full analysis.

Root cause: `EK3_RNG_USE_HGT=10` triggers the terrain estimator lockout feedback loop
(see [ekf_rng_use_hgt_feedback](../topics/ekf_rng_use_hgt_feedback.md)). This was
compounded by 105s of pre-ARM jogging which corrupted baro and caused Core 1 to develop
a different altitude estimate from Core 0.

## Known Issues

1. **EK3_RNG_USE_HGT terrain lockout** — With EK3_RNG_USE_HGT=10, the rangefinder stops
   being the primary height source above 0.7m HAGL. When the terrain estimator loses track
   (e.g. during takeoff height reset), `gndOffsetValid` expires after 5s, permanently
   locking out optical flow fusion. **This caused the logm2_log4 ceiling hit.** Must be set
   to -1. See [ekf_rng_use_hgt_feedback](../topics/ekf_rng_use_hgt_feedback.md).

2. **EK3_PRIMARY=1 prevents core failover** — Forcing Core 1 as primary means the altitude
   controller cannot fall back to Core 0 even when Core 1 is flying dead reckoning. In
   logm2_log4, Core 0 maintained correct flow fusion the entire flight but was never used.

3. **PSC_JERK_Z=30 allows dangerous altitude spikes** — 6x default. When the EKF resets
   aiding on the primary core, the altitude target can spike unconstrained, causing rapid
   ceiling/floor hits.

4. **No Z velocity source** — EK3_SRC1_VELZ=0 means no independent vertical velocity
   measurement. The EKF relies entirely on integrating (biased) accelerometers corrected by
   (thermally-drifting) baro.

5. **EK3_ACC_P_NSE=0.05 too low** — 7x below default (0.35). Makes EKF trust IMU more and
   correct from observations slower, slowing recovery from transient errors.

6. **Baro thermal drift** — BAlt rose 3.5m in 20s during logm2_6 hover (baro temp
   51.9→55.5°C). The 32°C cold-start rise in logm2_4 shows how much the baro is affected.

7. **Notch filter miscentered** — Actual motor fundamental ~536 Hz (from ESC RPM 30-33k).
   INS_HNTCH_FREQ=475 Hz is 61 Hz low, but within the 235 Hz bandwidth so not critical.

8. **CG offset** — Motor outputs show C2 consistently low (-2.1 to -2.5%), C3 consistently
   high (+1.6 to +2.0%). Forward CG bias.

9. **Turtle_Mode.lua error** — Script error at line 14 repeats every 10s. Non-functional.

### Resolved (from logm2_4/5/6)

10. ~~**EKF running on worst IMU**~~ — Fixed: EK3_IMU_MASK changed from 1 to 3,
    EK3_PRIMARY=1 forces IMU1 (the good one). IMU0 had 10+ m/s² Z vibe and 9,870 clips.

11. ~~**ACC_ZBIAS_LEARN disabled**~~ — Fixed: ACC_ZBIAS_LEARN changed from 0 to 3.

## Recommended Changes (for next flight)

### Critical (must fix)

| Parameter | Current | Target | Reason |
|-----------|---------|--------|--------|
| **EK3_RNG_USE_HGT** | 10 | **-1** | Always use rangefinder — prevents terrain lockout (logm2_log4) |
| **EK3_PRIMARY** | 1 | **0** | Let EKF choose healthiest core — forced primary caused ceiling hit |
| **PSC_JERK_Z** | 30 | **5** (default) | Prevent dangerous altitude target spikes |

### Important

| Parameter | Current | Target | Reason |
|-----------|---------|--------|--------|
| EK3_ACC_P_NSE | 0.05 | **0.35** (default) | Allow EKF to correct from observations faster |
| ARMING_CHECK | 64 | **1** | Enable all arming checks |
| INS_HNTCH_FREQ | 475 | **536** | Re-center notch on actual motor fundamental |

### Investigate

- **Terrain state not reset on height datum change** — EKF3 resets PD at takeoff but not
  terrainState, creating instantaneous HAGL error. This is an EKF3 code issue.
- IMU0 Z-axis vibration source — mechanical mounting issue? Why is IMU0 so much worse?
- Consider ESC telemetry for notch tracking (MODE=3) instead of throttle-based (MODE=1)
- Turtle_Mode.lua script error — remove or fix

## See Also
- [logm2_4](../logs/logm2_4.md) — flight 1, excellent performance
- [logm2_5](../logs/logm2_5.md) — flight 2, clipping discovered
- [logm2_6](../logs/logm2_6.md) — flight 3, altitude failure (AccZ bias)
- [logm2_log4](../logs/logm2_log4.md) — flight 4, ceiling hit (terrain lockout)
- [ekf_rng_use_hgt_feedback](../topics/ekf_rng_use_hgt_feedback.md) — the RNG_USE_HGT feedback loop
