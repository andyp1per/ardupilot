# log4 Analysis

## Metadata
- **Date**: 2026-02-16
- **Vehicle**: TD outdoor (MatekH743-bdshot)
- **Firmware**: V4.6.3v2-SFD (5282cc3d)
- **Branch**: SmallFastDrone-4.6-AltHold
- **Log file**: ./log4.bin
- **Sensors**: GPS, baro, dual IMU, rangefinder (Orient=25, downward)
- **Note**: Different from earlier SFD development "log4" (ground effect gap analysis)

## Parameters

Same as [log3](log3.md) — identical vehicle and firmware, consecutive flight.

| Parameter | Value | Notes |
|-----------|-------|-------|
| EK3_RNG_USE_HGT | -1 | Rangefinder disabled for EKF height |
| BARO1_THST_SCALE | -100 | Active |
| EK3_MAG_CAL | 3 | AFTER_CLIMB (not recommended 7) |
| COMPASS_MOTCT | 0 | No motor compensation (not recommended 2) |
| INS_ACC_VRFB_Z | -0.343 | **Corrupted** (should be small positive) |
| ACC_ZBIAS_LEARN | 0 | Disabled (not recommended 2) |
| TKOFF_GNDEFF_TMO | 0 | No timeout (not recommended 3.0) |
| MOT_THST_HOVER | 0.132 | Good |
| PSC_POSZ_P | 1.0 | Good |
| PSC_VELZ_P | 5.0 | Good |

## Flight Timeline

| Time (s) | Event |
|-----------|-------|
| 5.2 | Boot, Stabilize → AltHold |
| 19.2 | EV 62 (yaw alignment on ground) |
| 27.5 | EV 25 (Set home) |
| 30.2 | AltHold mode confirmed |
| 35.8 | **Armed** (EV 10) |
| 39.1 | **Takeoff** (EV 28) |
| 43.0 | BAlt spike to -6.8m (prop wash) |
| 50.9 | EV 62 (yaw alignment IMU0) |
| 131.5 | **Disarmed** (EV 11), land complete |

Total hover: ~92s (39s to 131s). Target altitude: 3.05m.

## Altitude Hold Performance

### CTUN Statistics (hover, 39s-131s)

| Metric | Mean | Std | Min | Max |
|--------|------|-----|-----|-----|
| Alt (EKF) | 2.46 m | — | -1.89 | 3.35 |
| DAlt (desired) | — | — | — | 3.05 |
| BAlt (baro) | 2.57 m | — | **-6.80** | 4.20 |
| ThO | 0.127 | — | 0.049 | 0.172 |
| Alt Error (mean) | **-0.001 m** | — | -0.646 | +1.516 |
| **Alt Error std** | **0.291 m** | — | — | — |

Alt error mean is essentially zero — the controller holds target on average. But
std of 29.1 cm is significantly worse than log3's 14.2 cm. The higher hover altitude
(3m vs 2m) and the massive takeoff baro spike contribute to the increased variance.

### Steady-State Performance (60-120s, excluding takeoff transients)

During stable hover at 3.05m target, the rangefinder provides independent truth:

| Time | EKF Alt | BAlt | RFND | BAlt-EKF |
|------|---------|------|------|----------|
| 60s | 3.14 | 3.59 | 3.35 | +0.45 |
| 65s | 3.31 | 4.04 | 3.81 | +0.73 |
| 70s | 3.33 | 3.97 | 3.75 | +0.64 |
| 80s | — | — | 2.84 | — |
| 90s | — | — | 2.04 | — |
| 100s | — | — | 3.02 | — |
| 110s | — | — | 3.22 | — |
| 120s | — | — | 2.62 | — |

The rangefinder shows actual altitude varies from 2.0 to 3.8m — the vehicle is not holding
altitude as tightly as the EKF reports. The BAlt-EKF divergence oscillates +/- 0.7m
throughout the flight, driven by thermal drift.

## Baro Anomaly at Takeoff (42.9-43.5s)

Massive baro pressure spike immediately after motor ramp:

| Time | BARO Alt | Press (Pa) |
|------|----------|------------|
| 42.82s | +0.26 m | 100317.5 |
| 42.92s | **-3.00 m** | 100357.3 |
| 43.02s | **-6.72 m** | 100403.0 |
| 43.12s | **-6.80 m** | 100405.1 |
| 43.52s | -5.65 m | 100391.8 |
| 43.92s | +0.41 m | 100317.7 (recovered) |

88 Pa spike (7.5m equivalent) lasting ~1 second. The EKF correctly rejected this
(EK3_HGT_I_GATE=300 helped), and altitude only wobbled ~0.3m.

## Temperature

| Sensor | Boot | Peak | Hover Mean | End | Range |
|--------|------|------|-----------|-----|-------|
| BARO | 48.6 C | **57.1 C** (t=42s) | 53.0 C | 51.7 C | **8.5 C** |
| IMU | 37.6 C | **42.4 C** (t=37s) | 39.5→37.1 C | 37.1 C | 6.8 C |

Unlike log3 (where baro only heated), log4 shows a full thermal cycle: baro heats
8.5 C from boot to arm, then **cools 5.3 C** during the hover from prop airflow. Both
heating and cooling phases drive baro altitude drift.

## Vibrations

| IMU | VibeX Mean/Max | VibeY Mean/Max | VibeZ Mean/Max |
|-----|----------------|----------------|----------------|
| IMU0 | 4.51 / 17.92 | 5.01 / 12.24 | 2.09 / 23.14 |
| IMU1 | 3.22 / 12.25 | 3.27 / 6.11 | 1.58 / 27.72 |

Vibrations higher than log3, with occasional Z-axis spikes to 23-28 m/s^2. No clipping.
The higher hover altitude (3m vs 2m) may contribute to different vibration modes. Notch
filter (INS_HNTCH_ENABLE=1) is active and helping.

## EKF Dual-Core Divergence

### Peak Divergence: 0.69m at 110s

| Time | Alt_C0 | Alt_C1 | Difference |
|------|--------|--------|------------|
| 65s | 3.31 | 3.59 | 0.28 m |
| 85s | 2.74 | 2.60 | 0.14 m |
| 100s | 3.09 | 3.54 | 0.45 m |
| **110s** | **3.16** | **3.84** | **0.69 m** |
| 120s | 2.93 | 2.78 | 0.15 m |
| 125s | 2.85 | 2.26 | 0.59 m |

Core 1 shows more altitude variation than Core 0. The 0.69m peak divergence at 110s
is significant — the two IMUs are disagreeing on altitude by nearly 70cm during hover.
This is worse than log3's 20cm divergence. The corrupted INS_ACC_VRFB_Z=-0.343 likely
affects the two IMUs differently, driving the divergence.

## XKF3 Innovations — Persistent VD Innovation

**IVD (velocity down innovation) is stuck at -0.54 m/s from 50s onward for Core 0.**

| Time | IVD | IPD | IVN | IVE |
|------|-----|-----|-----|-----|
| 40s | -0.14 | +2.46 | +0.07 | +0.14 |
| 50s | **-0.54** | -0.50 | +0.09 | -0.16 |
| 60-130s | **-0.54** | oscillating | +0.09 | -0.16 |

The persistent -0.54 m/s VD innovation means the EKF consistently sees a disagreement
between predicted and measured vertical velocity. This indicates an **unmodeled Z-axis
acceleration bias** that the EKF cannot track because ACC_ZBIAS_LEARN=0.

The IVN and IVE values are also frozen at 0.09 and -0.16 respectively, suggesting the
EKF is not fusing GPS velocity updates after the initial alignment.

## XKFS MAG_FUSION State Transitions

| Time | MAG_FUSION | Meaning |
|------|-----------|---------|
| 6s | 0 | Not fusing compass |
| 40s | 1 | Fusing yaw only (offsets frozen) |
| 60s | 2 | Fully fusing (offsets learning) |
| 60-130s | 2 | Steady full fusion |

Proper MAG_CAL=3 behavior: compass transitions from not fusing → yaw only → full fusion
after first in-flight yaw alignment at 50.9s.

## Rangefinder Data

Rangefinder active throughout (Orient=25, downward-facing, EK3_RNG_USE_HGT=-1 so not
used by EKF for height):

| Phase | RFND Range |
|-------|-----------|
| Pre-takeoff (40s) | 0.64 m |
| Climb (45s) | 0.32 m (still near ground) |
| Rising (55s) | 1.91 m |
| Peak (65s) | **3.81 m** |
| Mid-hover (85s) | 1.83 m |
| End hover (125s) | 2.47 m |
| Landing (130s) | 0.19 m |

The rangefinder shows the vehicle oscillated between ~1.8m and ~3.8m during the hover,
while the EKF reported a tighter 2.5-3.35m range. This suggests the EKF may be
over-smoothing altitude, or that baro thermal drift is causing the EKF to slowly
diverge from truth.

## Post-Landing EKF Divergence

| Time | Alt | BAlt | CRt (cm/s) | Event |
|------|-----|------|------------|-------|
| 131.5s | -1.15 | +0.30 | -69 | Disarm |
| 131.6s | -1.20 | -0.12 | -142 | EKF still descending |
| 132.2s | **-1.89** | +0.03 | -160 | Peak descent |
| 133.2s | **-2.32** | +0.13 | -114 | **Maximum divergence** |
| 134.8s | -1.75 | +0.06 | -39 | Slowly recovering |

After disarm, the EKF believes the vehicle is **descending at 142-160 cm/s** while on the
ground. Altitude drops from -1.15 to -2.32m (1.17m excursion below actual ground). Then
slowly recovers toward -1.75m as baro corrects.

The baro reads correctly near 0m throughout — this is purely an EKF state divergence from
residual velocity and accel bias after the flight.

**Contrast with log3:** Log3 EKF drifted **upward** (+0.94m above ground). Log4 EKF drifts
**downward** (-2.32m below ground). The direction depends on the residual velocity state
at the moment of disarm.

## Motor Outputs (RCOU)

| Time | C1 | C2 | C3 | C4 |
|------|------|------|------|------|
| 50s | 1312 | 1335 | 1376 | 1292 |
| 70s | 1303 | 1340 | 1389 | 1290 |
| 90s | 1331 | 1329 | 1382 | 1298 |
| 110s | 1283 | 1366 | 1411 | 1255 |
| 120s | 1315 | 1344 | 1383 | 1297 |

Motor outputs are stable. C3 consistently highest (~1380-1410), C4 lowest (~1255-1300),
indicating a slight CG offset toward motor 3.

## GPS Quality

| Metric | Mean | Min | Max |
|--------|------|-----|-----|
| NSats | 14.6 | 11 | 15 |
| HDop | 0.84 | 0.79 | 1.69 |
| Status | 3 | 3 | 3 |

Excellent GPS throughout. Brief HDop spike to 1.69 at 99.5s, no impact.

## Comparison: log3 vs log4

| Metric | Log3 | Log4 |
|--------|------|------|
| Hover altitude | 2.0 m | 3.0 m |
| Mode | Stabilize→AltHold | AltHold from start |
| Alt error std | **14.2 cm** | **29.1 cm** |
| Baro temp range | 9.4 C (rising) | 8.5 C (rise then fall) |
| Max BAlt spike | -6.15 m | **-6.80 m** |
| Core divergence (max) | 20 cm | **69 cm** |
| Post-landing drift | +0.94 m (up) | **-2.32 m (down)** |
| VD innovation | — | -0.54 m/s (stuck) |
| Rangefinder | Not analyzed | 1.8-3.8 m range |

Log4 performs worse on every metric. The higher altitude may contribute (more thermal
drift, more baro variability), but the persistent VD innovation and core divergence
suggest the corrupted VRFB_Z and disabled ZBIAS_LEARN are the primary factors.

## Recommendations

Same as [log3](log3.md) — all five parameter changes apply:

```
ACC_ZBIAS_LEARN = 2          # Fixes stuck VD innovation
INS_ACC_VRFB_Z = 0           # Zero corrupted value, re-learn
TKOFF_GNDEFF_TMO = 3.0       # Protect takeoff baro transient
EK3_MAG_CAL = 7              # Faster yaw convergence, takeoff protection
COMPASS_MOTCT = 2            # Motor compass compensation
```

Additional log4-specific observations:

1. The rangefinder shows actual altitude variation (1.8-3.8m) is larger than what the
   EKF reports (2.5-3.35m). Consider enabling rangefinder height fusion at lower altitudes
   (EK3_RNG_USE_HGT=70 with appropriate RNGFND_MAX_CM).

2. The persistent IVD=-0.54 innovation is the strongest evidence that ACC_ZBIAS_LEARN
   must be enabled. The EKF sees a consistent 0.54 m/s velocity error it cannot correct.

3. The 0.69m dual-core divergence is concerning — suggests IMU-specific accel biases that
   differ significantly between the two sensors.

## See Also
- [log3](log3.md) — same vehicle, Stabilize→AltHold, lower altitude
- [logtd6](logtd6.md) — earlier TD outdoor with 17.9cm alt error
- [logtd8](logtd8.md) — VRFB clamp finding on TD vehicle
- [Baro Thermal Drift](../topics/baro_thermal_drift.md)
