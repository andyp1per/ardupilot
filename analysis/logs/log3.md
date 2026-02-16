# log3 Analysis

## Metadata
- **Date**: 2026-02-16
- **Vehicle**: TD outdoor (MatekH743-bdshot)
- **Firmware**: V4.6.3v2-SFD (5282cc3d)
- **Branch**: SmallFastDrone-4.6-AltHold
- **Log file**: ./log3.bin
- **Sensors**: GPS (no optical flow), baro, dual IMU
- **Note**: Different from earlier SFD development "log3" (hover bias validation)

## Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| EK3_RNG_USE_HGT | -1 | Rangefinder disabled for EKF height |
| BARO1_THST_SCALE | -100 | Active |
| EK3_MAG_CAL | 3 | AFTER_CLIMB (not recommended 7) |
| COMPASS_MOTCT | 0 | No motor compensation (not recommended 2) |
| INS_ACC_VRFB_Z | -0.343 | **Corrupted** (should be small positive) |
| ACC_ZBIAS_LEARN | 0 | Disabled (not recommended 2) |
| TKOFF_GNDEFF_TMO | 0 | No timeout (not recommended 3.0) |
| MOT_THST_HOVER | 0.126 | Well-calibrated (actual ThO=0.131) |
| PSC_POSZ_P | 1.0 | Good |
| PSC_VELZ_P | 5.0 | Good |
| EK3_ALT_M_NSE | 2.0 | Wide baro noise allowance |
| EK3_HGT_I_GATE | 300 | Very wide innovation gate |

## Flight Timeline

| Time (s) | Event |
|-----------|-------|
| 5.2 | Boot, Stabilize mode |
| 19.5 | Armed (EV 10) |
| 20.7 | AutoArmed (EV 15) |
| 21.0 | TakeoffComplete (EV 28) |
| 28.2 | **AltHold mode** |
| 117.9 | EKF yaw reset (EV 62) |
| 129.5 | Stabilize mode |
| 132.1 | Land complete (EV 17 + EV 18) |
| 132.8 | Disarmed (EV 11) |

Total flight: ~113s. AltHold hover: ~101s (28.2s to 129.5s).

## Altitude Hold Performance

### CTUN Statistics (AltHold, 28s-129s)

| Metric | Mean | Std | Min | Max |
|--------|------|-----|-----|-----|
| Alt (EKF) | 2.008 m | 0.096 | 1.782 | 2.289 |
| DAlt (desired) | 2.076 m | 0.114 | 0.000 | 2.083 |
| BAlt (baro) | 2.100 m | 0.294 | 1.420 | 2.940 |
| ThO | 0.131 | 0.002 | 0.114 | 0.138 |
| CRt (cm/s) | 0.81 | 9.37 | -27 | 40 |
| **Alt Error** | **-0.068 m** | **0.142** | -0.301 | 1.869 |

**Alt error std = 14.2 cm** during steady hover. This is good performance for baro-only
height hold outdoors. The 1.87m max error is a transient at AltHold entry (DAlt jumps from
0 to 2.08m).

ThO extremely stable at 0.131 (std=0.002) confirming MOT_THST_HOVER is well-calibrated.

### Late-Flight Drift

After yaw reset at 117.9s, altitude drifts upward ~25cm (2.04m to 2.29m). BAlt climbs
more dramatically from ~2.5m to ~2.9m. Root cause: baro thermal drift (see Temperature).

## Temperature

| Sensor | Boot | Takeoff | Peak | End | Range |
|--------|------|---------|------|-----|-------|
| BARO | 40.8 C | 47.6 C | 50.2 C | 50.2 C | **9.4 C** |
| IMU0 | 31.8 C | — | 36.6 C | — | 4.8 C |
| IMU1 | 33.2 C | — | 38.0 C | — | 4.8 C |

The baro temperature **rose continuously** throughout the entire flight at ~0.1 C/10s and
never thermally stabilized. This is opposite to the SmallFastDronev1 behavior (which cools
from prop airflow). The TD vehicle baro is heating from board/MCU thermal soak rather than
cooling from prop wash.

BAlt drifted from ~1.7m to ~2.9m as the baro warmed — a 1.2m total baro drift over 100s.

### BAlt-EKF Divergence

| Period | BAlt - EKF Alt (mean) | Notes |
|--------|----------------------|-------|
| 100-110s | -0.18 to +0.21 | Small, oscillating |
| 110-118s | +0.11 to +0.63 | Diverging — baro rising from thermal |
| 118-128s | +0.19 to +0.65 | Peak divergence, sustained |

Mean BAlt-EKF = +0.21m, std = 0.29m. The EKF correctly rejected most of the baro thermal
drift, keeping actual altitude error much smaller than the baro drift.

## Vibrations

| IMU | VibeX Mean/Max | VibeY Mean/Max | VibeZ Mean/Max |
|-----|----------------|----------------|----------------|
| IMU0 | 2.55 / 3.61 | 4.77 / 5.91 | 1.27 / 1.64 |
| IMU1 | 1.76 / 2.63 | 2.96 / 3.97 | 0.80 / 1.37 |

Vibrations are **very low** — well within acceptable limits (threshold ~15 m/s^2). IMU0
slightly higher than IMU1. VibeY is the dominant axis for both IMUs, suggesting lateral
resonance mode.

## EKF Dual-Core Divergence

### PD (Altitude) Divergence

| Phase | PD0-PD1 (m) | Notes |
|-------|-------------|-------|
| 100-116s | -0.03 to -0.08 | IMU0 slightly deeper |
| 116.5s | 0.00 | **Crossover** |
| 117.9s (yaw reset) | +0.06 | Cores begin diverging |
| 120s | +0.13 | IMU1 now deeper |
| 125s | +0.16 | Peak divergence |
| Post-landing | +0.08 | Slowly converging |

The yaw alignment event at 117.9s triggers the divergence. IMU1 tracks the baro drift
more aggressively than IMU0 after yaw alignment changes compass fusion state. Max divergence
of 20cm between cores during hover.

## Compass Behavior (EK3_MAG_CAL=3)

- Earth field: MN=224, ME=-34, MD=431 mG (constant after init at ~30s)
- Body offsets: MX=MY=MZ=0 (no offset learning with MAG_CAL=3 + MOTCT=0)
- Initial alignment: MN=737 at t=22s, settling to 224 by t=30s — very large initial error
- **98 seconds** from takeoff to yaw reset (very slow convergence)

Compare: With EK3_MAG_CAL=7 on SmallFastDronev1, yaw divergence was 5.3 deg vs 24 deg with
MAG_CAL=4. This vehicle would benefit from MAG_CAL=7 + COMPASS_MOTCT=2.

## GPS Quality

| Metric | Mean | Min | Max |
|--------|------|-----|-----|
| NSats | 14.7 | 7 | 15 |
| HDop | 0.84 | 0.79 | 1.68 |
| Status | 3 | 3 | 3 |

Excellent GPS throughout.

## Takeoff Behavior

| Time | ThO | Alt (m) | BAlt (m) | CRt (cm/s) | Event |
|------|-----|---------|----------|------------|-------|
| 21.0s | 0.005 | -0.38 | -6.15 | — | Motors start, massive baro spike |
| 22.0s | 0.115 | -0.38 | — | — | Liftoff |
| 24.0s | 0.131 | +0.33 | — | +87 | Peak climb rate |
| 26.0s | 0.137 | +2.33 | — | — | Overshoot peak |
| 28.2s | 0.127 | +2.00 | — | — | AltHold engaged, settling |

Takeoff overshoot: ~25cm (12%). BAlt swings -6.15m to +0.72m during takeoff (6.9m range
in 0.6s) — baro essentially useless during this phase.

## Post-Landing EKF Divergence

**The most significant finding in this log.**

| Time | Event | EKF Alt | BAlt | VD (m/s) |
|------|-------|---------|------|----------|
| 129.5s | Stabilize, descending | 2.08 | 2.22 | — |
| 131.4s | **Baro slam** | 0.26 | **-4.53** | +0.86 |
| 131.5-131.9s | Oscillation | bouncing | -4.5 to -1.9 | reversing |
| 132.0-132.8s | Recovery | 0.38 | -3.4 to -0.5 | — |
| 132.8s | **Disarmed** | **0.38** | -0.5 | — |
| 133-135.6s | **Post-disarm drift** | **0.94** | — | -0.58 |

After disarm, the EKF believes the vehicle is **climbing at 0.58 m/s** while sitting on
the ground. Altitude drifts from +0.38m to +0.94m in 3 seconds. Both EKF cores agree on
this divergent behavior.

**Root cause:** The -4.5m baro pressure spike at landing corrupted the EKF height state.
With no rangefinder (EK3_RNG_USE_HGT=-1), there is no independent height reference. After
the baro recovers, the EKF has a residual velocity state that continues integrating.

**Baro thermal shift:** Post-landing baro pressure is ~100333 Pa vs pre-takeoff ~100318 Pa —
a permanent 15 Pa offset (~1.3m) from the 9.4 C thermal drift during flight.

## Recommendations

### Immediate (parameter changes for this vehicle)

```
# Enable Z-bias learning (fixes persistent accel bias)
ACC_ZBIAS_LEARN = 2

# Fix corrupted VRFB (zero and re-learn)
INS_ACC_VRFB_Z = 0

# Ground effect timeout (protects takeoff baro transient)
TKOFF_GNDEFF_TMO = 3.0

# Compass improvements (proven on SmallFastDronev1)
EK3_MAG_CAL = 7
COMPASS_MOTCT = 2
```

### Observations

1. Baro thermal drift (9.4 C, never stabilized) is the primary altitude error source
2. Altitude performance is still good (14.2 cm std) despite suboptimal parameters
3. Post-landing EKF divergence is the most concerning behavior for re-arm scenarios
4. Compass convergence at 98s is very slow — MAG_CAL=7 would improve this significantly

## See Also
- [log4](log4.md) — same vehicle, AltHold from start, higher altitude hover
- [logtd5](logtd5.md) — earlier TD outdoor flight, baro thermal drift -22 C
- [logtd6](logtd6.md) — TD outdoor with improved PSC gains (17.9cm)
- [Baro Thermal Drift](../topics/baro_thermal_drift.md)
