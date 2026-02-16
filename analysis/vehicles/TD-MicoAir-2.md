# TD-MicoAir-2 — Vehicle Notes

## Vehicle Description

- **Frame**: Quad X (FRAME_CLASS=1, FRAME_TYPE=1)
- **FC**: MicoAir743v2 (STM32H743, APJ=1179)
- **Firmware**: V4.6.3v2-SFD (883e416a)
- **Board orientation**: YAW45 (AHRS_ORIENTATION=1)
- **IMU**: Dual IMU — EK3_IMU_MASK=1 (**IMU0 only**, IMU1 unused)
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

No parameter changes between flights — same config for all three.

## Current Configuration

```
# Height source
EK3_SRC1_POSZ        = 1        (baro)
EK3_SRC1_VELZ        = 0        (NONE — no Z velocity source)
EK3_RNG_USE_HGT     = -1       (rangefinder height disabled)
EK3_ALT_M_NSE       = 1.5
EK3_HGT_I_GATE      = 150

# Position/velocity source
EK3_SRC1_POSXY       = 0        (none)
EK3_SRC1_VELXY       = 5        (optical flow)
EK3_SRC1_YAW         = 0        (none — GSF yaw)

# EKF config
EK3_IMU_MASK         = 1        (IMU0 only — PROBLEM)
EK3_GND_EFF_DZ       = 7

# Baro compensation
BARO1_THST_SCALE    = -147

# Position controller
PSC_POSZ_P          = 1.25
PSC_VELZ_P          = 6.5

# Accel bias
INS_ACC_VRFB_Z      = 0.041    (too small — real bias is 15x larger)
ACC_ZBIAS_LEARN     = 0        (DISABLED)

# Ground effect
TKOFF_GNDEFF_ALT    = (default)
TKOFF_GNDEFF_TMO    = 0

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

## Progressive Degradation (logm2_4 → logm2_5 → logm2_6)

```
logm2_4  [=]          2.5 cm  — EXCELLENT, 108s hover
logm2_5  [======]    19.1 cm  — degraded, 47s, IMU0 clipping discovered
logm2_6  [XXXXXXXXX] FAILED   — 22s, cascading altitude failure
```

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

## Known Issues

1. **EKF running on worst IMU** — EK3_IMU_MASK=1 selects IMU0, which has severe Z-axis
   vibration (mean 10+ m/s², 2x recommended limit) and massive clipping (9,870 events in
   logm2_5). IMU1 has 6x less vibration and zero clips. **This is the primary issue.**

2. **No Z velocity source** — EK3_SRC1_VELZ=0 means no independent vertical velocity
   measurement. The EKF relies entirely on integrating (biased) accelerometers corrected by
   (thermally-drifting) baro. When the AccZ bias grows large, there's nothing to pull the
   velocity estimate back to reality.

3. **Rangefinder not used for height** — EK3_RNG_USE_HGT=-1 disables the working rangefinder
   (2.35m mean, 95% good status in logm2_6) as a height source. The rangefinder data
   confirms stable hover while the EKF reports continuous descent.

4. **ACC_ZBIAS_LEARN disabled** — INS_ACC_VRFB_Z=0.041 is 15x smaller than the actual bias
   visible in logm2_6 (-0.62 m/s²). With learning disabled, the correction can never adapt.

5. **Stuck velocity innovations** — IVD stuck at constant values in all three flights
   (-0.43, -0.03, +0.64). With no Z velocity source (VELZ=0), there's nothing to drive
   velocity innovations, so they saturate/gate without correcting.

6. **Baro thermal drift** — BAlt rose 3.5m in 20s during logm2_6 hover (baro temp
   51.9→55.5°C). The 32°C cold-start rise in logm2_4 shows how much the baro is affected.

7. **Notch filter miscentered** — Actual motor fundamental ~536 Hz (from ESC RPM 30-33k).
   INS_HNTCH_FREQ=475 Hz is 61 Hz low, but within the 235 Hz bandwidth so not critical.

8. **CG offset** — Motor outputs show C2 consistently low (-2.1 to -2.5%), C3 consistently
   high (+1.6 to +2.0%). Forward CG bias.

9. **Turtle_Mode.lua error** — Script error at line 14 repeats every 10s. Non-functional.

## Recommended Changes (for next flight)

### Critical (must fix)

| Parameter | Current | Target | Reason |
|-----------|---------|--------|--------|
| EK3_IMU_MASK | 1 | **3** (or 2) | Use both IMUs (or IMU1 only); IMU0 is severely vibration-compromised |
| EK3_RNG_USE_HGT | -1 | **70** | Enable rangefinder for height below 70% max range (~4.9m) |
| ACC_ZBIAS_LEARN | 0 | **2** | Enable Z-bias learning with ground inhibit |
| INS_ACC_VRFB_Z | 0.041 | **0** | Zero and let re-learn from clean start |

### Important

| Parameter | Current | Target | Reason |
|-----------|---------|--------|--------|
| ARMING_CHECK | 64 | **1** | Enable all arming checks |
| INS_HNTCH_FREQ | 475 | **536** | Re-center notch on actual motor fundamental |

### Investigate

- IMU0 Z-axis vibration source — mechanical mounting issue? Why is IMU0 so much worse?
- Consider ESC telemetry for notch tracking (MODE=3) instead of throttle-based (MODE=1)
- Turtle_Mode.lua script error — remove or fix

## See Also
- [logm2_4](../logs/logm2_4.md) — flight 1, excellent performance
- [logm2_5](../logs/logm2_5.md) — flight 2, clipping discovered
- [logm2_6](../logs/logm2_6.md) — flight 3, altitude failure
