# log6 Analysis

## Metadata
- **Date**: 2026-02-16
- **Vehicle**: TD-Matek-5 outdoor (MatekH743-bdshot)
- **Firmware**: V4.6.3v2-SFD (5282cc3d)
- **Branch**: SmallFastDrone-4.6-AltHold
- **Log file**: ./log6.bin
- **Sensors**: GPS, baro, dual IMU, rangefinder (Orient=25, downward)
- **Flight #**: 4

## Parameters

Same as log5 — consecutive flight, no parameter changes between flights.

| Parameter | Value | Notes |
|-----------|-------|-------|
| ACC_ZBIAS_LEARN | 3 | Always learn (set more aggressively than planned 2) |
| INS_ACC_VRFB_Z | -0.011 (boot) | Not zeroed; learned to **-0.390** during flight |
| TKOFF_GNDEFF_TMO | 0 | NOT APPLIED (should be 3.0) |
| EK3_MAG_CAL | 7 | OK — GROUND_AND_INFLIGHT |
| COMPASS_MOTCT | 2 | OK — current-based motor compensation |
| ARMING_CHECK | 64 | NOT APPLIED — GPS bitmask only (should be 1) |
| MOT_THST_HOVER | 0.140 | Learned from previous flights |
| BARO1_THST_SCALE | -100 | Active |
| PSC_POSZ_P | 1.0 | Good |
| PSC_VELZ_P | 5.0 | Good |
| EK3_RNG_USE_HGT | -1 | Rangefinder disabled for EKF height |

## INS_ACC_VRFB_Z Evolution

| Phase | Value |
|-------|-------|
| Boot | -0.011 m/s^2 |
| Disarm save | **-0.390 m/s^2** |

With ACC_ZBIAS_LEARN=3, the VRFB learned from -0.011 to -0.390 during this single flight.
This is a very large Z-accel bias being captured — moving further from zero rather than
toward it.

## Flight Timeline

| Offset (from arm) | Event |
|--------------------|-------|
| -15.7s | Boot, STABILIZE mode |
| -14.9s | Hover Z-bias: IMU0=-0.010, IMU1=+0.020 m/s^2 |
| -3.5s | EKF3 both cores initialized, EKF3 active |
| -3.4s | Initial yaw alignment complete |
| **+0.0s** | **ARMED** (ARMING_CHECK=64) |
| +0.0s | COMPASS_RESET, EKF_SPEED_RESET |
| +1.1s | TAKEOFF event |
| +2.9s | **Ground mag anomaly** — yaw re-aligned (both cores) |
| +6.6s | EKF origin set, SET_HOME |
| +7.4s | In-flight yaw alignment (C0), +7.9s (C1) |
| **+12.7s** | **Mode: ALT_HOLD** |
| +36.9s | EKF3 IMU1 is using GPS |
| +41.7s | EKF3 IMU0 is using GPS |
| **+96.0s** | **Mode: STABILIZE** (landing) |
| +97.7s | EKF3 lane switch to Core 1, yaw reset |
| +99.0s | LAND_COMPLETE_MAYBE |
| **+99.7s** | **DISARMED**, LAND_COMPLETE, EKF_ALT_RESET |
| +101.1s | PreArm: EKF3 yaw inconsistent **61 deg** |

Total armed: ~100s. ALT_HOLD hover: ~83s (+12.7s to +96.0s).

**Key difference from log5:** No EKF Failsafe — GPS incorporated smoothly at +37-42s
without triggering compass errors.

## Altitude Hold Performance

### CTUN Statistics (ALT_HOLD, +18s to +95s)

770 samples.

| Metric | Mean | Std | Min | Max |
|--------|------|-----|-----|-----|
| Alt (EKF) | 1.873 m | 0.326 | 1.299 | 3.053 |
| DAlt (desired) | 1.934 m | 0.219 | 1.876 | 2.867 |
| BAlt (baro) | 1.854 m | 0.645 | 0.110 | 3.910 |
| ThO | 0.138 | 0.005 | 0.107 | 0.151 |
| Alt Error mean | **-0.061 m** | — | -0.577 | +0.739 |
| **Alt Error std** | **0.235 m** | — | — | — |

23.5 cm altitude error std. BAlt is very noisy (std=0.645m) but the EKF smooths this
to much tighter tracking. ThO averaging 0.138 matches MOT_THST_HOVER=0.14.

## EKF Dual-Core Divergence

| Time (from hover start) | C0 PD | C1 PD | Divergence |
|--------------------------|-------|-------|------------|
| +0s | -2.53 | -2.74 | 0.21 m |
| +10s | -2.11 | -1.84 | 0.27 m |
| +20s | -1.85 | -3.21 | **1.36 m** |
| +30s | -1.75 | -1.91 | 0.16 m |
| +40s | -1.92 | -1.94 | 0.02 m |
| +50s | -2.03 | -2.37 | 0.34 m |
| +60s | -1.86 | -2.12 | 0.26 m |
| +70s | -1.33 | -1.17 | 0.15 m |
| +75s | -1.65 | -1.88 | 0.23 m |

**Max hover divergence: 1.40 m** (at ~20s hover, coincides with GPS incorporation at +37-42s)
After GPS settled (~35s hover), divergence dropped below 0.25m consistently.

## XKF3 Innovations — IVD

| Time (from arm) | IVD Core 0 | IVD Core 1 |
|------------------|-----------|-----------|
| +0s | -0.13 | -0.10 |
| +10s | -0.95 | -0.82 |
| +20s | -0.95 | -0.82 |
| +30s | -0.95 | -0.82 |
| +40s | -0.95 | -0.52 |
| +50s | -0.49 | -0.35 |
| +60s | -0.07 | -0.06 |
| +70s | +0.12 | +0.04 |
| +80s | -0.09 | -0.01 |
| +90s | -0.92 | -0.90 |
| +99s | +0.53 | +0.23 |

| Core | Full flight mean | Std |
|------|-----------------|-----|
| Core 0 | -0.39 | 0.58 |
| Core 1 | -0.33 | 0.52 |

**IVD is no longer stuck at -0.54.** It oscillates and reaches near-zero during mid-flight
(+60-80s) when GPS is actively providing corrections. The pre-GPS period (+10-40s) still
shows strong negative bias (-0.95 / -0.82), and the late-flight period returns to -0.92.
Mid-flight improvement confirms GPS velocity fusion is correcting the bias.

## XKFS MAG_FUSION State Transitions

| Time | C0 State | C1 State | Event |
|------|----------|----------|-------|
| -3.3s | 2 (learning) | 2 (learning) | EKF init on ground |
| +2.8s | 1 (frozen) | 1 (frozen) | Ground mag anomaly |
| +7.5s | 2 (learning) | — | After in-flight yaw alignment |
| +8.0s | — | 2 (learning) | After in-flight yaw alignment |
| +8s→end | 2 (learning) | 2 (learning) | Continuous learning |

MAG_CAL=7 working correctly with proper state transitions:
ground learning → freeze on anomaly → resume after yaw init → continuous learning.

## XKF2 Compass Offsets

| Time (from arm) | C0 MX/MY/MZ | C1 MX/MY/MZ |
|------------------|-------------|-------------|
| +0s | 0/0/-1 | 0/0/-1 |
| +10s | 40/-4/5 | 34/-22/7 |
| +20s | 77/186/-32 | 103/168/-39 |
| +30s | -17/295/-89 | 9/282/-160 |
| +40s | 35/261/-129 | 106/234/-275 |
| +50s | 74/291/-239 | 121/256/-293 |
| +60s | 81/298/-228 | 125/259/-283 |
| +70s | 83/301/-228 | 126/261/-283 |
| +80s | 83/301/-226 | 125/260/-282 |
| +90s | 82/300/-226 | 124/260/-282 |
| +99s | 81/296/-220 | 124/267/-285 |

**Very large compass offsets** — MY reaches ~300 mGauss, MZ reaches ~-280 mGauss.
Convergence stabilizes by ~50s. The two cores disagree substantially (especially MZ:
-220 vs -285), contributing to the 61-degree yaw inconsistency post-disarm.

The offsets are ~50% larger than log5, suggesting the learning had more time to converge
on the true interference values.

## Temperature

| Sensor | Min | Max | Range |
|--------|-----|-----|-------|
| BARO | 51.3 C | 52.9 C | **1.6 C** |
| IMU0 | 35.6 C | 39.0 C | 3.4 C |
| IMU1 | 37.6 C | 40.5 C | 2.9 C |

Baro temperature range only 1.6 C — much better thermal stability than log3's 9.4 C.
The vehicle was already thermally soaked from log5.

## Vibrations

| IMU | VibeX Mean/Max | VibeY Mean/Max | VibeZ Mean/Max | Clip |
|-----|----------------|----------------|----------------|------|
| IMU0 | 2.80 / 6.88 | 5.02 / 6.94 | 1.57 / 3.67 | 0 |
| IMU1 | 1.83 / 4.94 | 3.20 / 5.03 | 1.06 / 3.44 | 0 |

Low vibrations, no clipping.

## GPS Quality

| Metric | Mean | Min | Max |
|--------|------|-----|-----|
| NSats | 20.5 | 13 | 21 |
| HDop | 0.57 | 0.53 | 0.97 |
| Status | 3 | 3 | 3 |

Excellent GPS throughout.

## Post-Landing

4.6s of post-disarm data available.

| Rel Disarm | Alt (m) | BAlt (m) |
|------------|---------|----------|
| +0.0s | -0.11 | -0.53 |
| +0.4s | -1.00 | +0.35 |
| +1.0s | -0.83 | +0.39 |
| +2.0s | -0.63 | +0.38 |
| +4.0s | -0.68 | +0.38 |
| +4.6s | -0.87 | +0.35 |

Post-disarm XKF1 PD: Core 0=+0.87m, Core 1=+0.07m — inter-core divergence ~0.80m.
Post-disarm yaw inconsistency: **61 degrees** between cores.

## Comparison: log5 → log6

| Metric | log5 | log6 |
|--------|------|------|
| Hover duration | 30 s | **83 s** |
| Alt error std | 37.5 cm* | **23.5 cm** |
| IVD behavior | Gone (positive bias) | Oscillating (mean -0.39) |
| Core divergence (max) | 0.89 m (2.79 total) | **1.40 m** (GPS transient) |
| EKF Failsafe | **YES** | No |
| Compass offsets (MY) | ~174 | **~300** |
| Post-disarm yaw | — | **61 deg** |
| Baro temp range | 11.8 C | **1.6 C** |

*log5 was cut short by failsafe

## Recommendations

Same as log5 — the primary issue remains compass interference. Additionally:

1. INS_ACC_VRFB_Z learned from -0.011 to -0.390 during this flight. If the intent
   was to zero it and re-learn, it should be explicitly reset to 0.
2. The IVD during baro-only periods (pre-GPS) is still biased at -0.95, suggesting
   BARO1_THST_SCALE may need tuning or the baro-only altitude solution has residual bias.
3. The 61-degree yaw inconsistency post-disarm is concerning for quick re-arm scenarios.

## See Also
- [log5](log5.md) — same vehicle, previous flight (EKF failsafe)
- [log7](log7.md) — same vehicle, next flight (best performance)
- [log3](log3.md) — same vehicle, pre-update baseline
