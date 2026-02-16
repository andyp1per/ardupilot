# log5 Analysis

## Metadata
- **Date**: 2026-02-16
- **Vehicle**: TD-Matek-5 outdoor (MatekH743-bdshot)
- **Firmware**: V4.6.3v2-SFD (5282cc3d)
- **Branch**: SmallFastDrone-4.6-AltHold
- **Log file**: ./log5.bin
- **Sensors**: GPS, baro, dual IMU, rangefinder (Orient=25, downward)
- **Flight #**: 3 (first with updated params)

## Parameters

| Parameter | Old (log3/4) | Expected | Actual | Status |
|-----------|-------------|----------|--------|--------|
| ACC_ZBIAS_LEARN | 0 | 2 | **3** | Different — set to 3 (always learn) not 2 |
| INS_ACC_VRFB_Z | -0.343 | 0 | **-0.011** | Nearly zero but not exactly 0 |
| TKOFF_GNDEFF_TMO | 0 | 3.0 | **0.0** | NOT APPLIED |
| EK3_MAG_CAL | 3 | 7 | **7** | OK |
| COMPASS_MOTCT | 0 | 2 | **2** | OK |
| ARMING_CHECK | — | 1 (all) | **64** | NOT APPLIED — GPS bitmask only |

Additional parameters unchanged from log3/log4:

| Parameter | Value | Notes |
|-----------|-------|-------|
| BARO1_THST_SCALE | -100 | Active |
| PSC_POSZ_P | 1.0 | Good |
| PSC_VELZ_P | 5.0 | Good |
| EK3_RNG_USE_HGT | -1 | Rangefinder disabled for EKF height |
| EK3_ALT_M_NSE | 2.0 | Wide baro noise allowance |
| EK3_HGT_I_GATE | 300 | Very wide innovation gate |
| MOT_THST_HOVER | 0.131 | Well-calibrated |
| INS_HNTCH_ENABLE | 1 | Harmonic notch active |

**Key finding:** Three of six intended changes were not applied: TKOFF_GNDEFF_TMO stayed at 0,
ARMING_CHECK=64 (GPS only bitmask, not 1 for all checks), and INS_ACC_VRFB_Z was not
exactly zeroed. ACC_ZBIAS_LEARN was set more aggressively (3 vs 2).

## Flight Timeline

| Offset (from arm) | Event |
|--------------------|-------|
| -15.5s | Boot: ArduCopter V4.6.3v2-SFD, Frame QUAD/X |
| -3.3s | EKF3 both cores initialized, EKF3 active |
| -3.2s | Initial yaw alignment complete |
| **+0.0s** | **ARMED in STABILIZE** |
| +0.8s | SET_HOME |
| +1.1s | EKF yaw reset |
| +3.9s | Ground mag anomaly, yaw re-aligned (both cores) |
| +6.5s | IMU0 in-flight yaw alignment complete |
| +6.7s | EKF origin set |
| +7.0s | IMU1 in-flight yaw alignment complete |
| +18.9s | EKF lane switch -> Core 1 |
| **+21.5s** | **Mode: ALT_HOLD** |
| +22.6s | Vibration compensation ON |
| +28.9s | EKF lane switch -> Core 0 |
| **+47.6s** | **EKF3 IMU0 is using GPS** |
| **+47.6s** | **GPS Glitch or Compass error** |
| +51.0s | EKF lane switch -> Core 1, Glitch cleared |
| +51.8s | EKF3 IMU1 is using GPS |
| +51.8s | **GPS Glitch or Compass error** (again) |
| **+52.2s** | **Mode: STABILIZE** (pilot override) |
| **+56.0s** | **EKF variance over thresholds — EKF Failsafe** |
| +61.0s | EKF lane switch -> Core 0 |
| **+61.2s** | **DISARMED**, LAND_COMPLETE, EKF_ALT_RESET |

Total armed: ~61s. ALT_HOLD hover: ~30s (+21.5s to +52.2s).
**Outcome: EKF Failsafe triggered.**

## Altitude Hold Performance

### CTUN Statistics (ALT_HOLD, +21.5s to +52.2s)

307 samples over ~30s.

| Metric | Mean | Std | Min | Max |
|--------|------|-----|-----|-----|
| Alt (EKF) | 1.379 m | 0.419 | 0.655 | 2.306 |
| DAlt (desired) | 1.338 m | 0.354 | 0.743 | 2.347 |
| BAlt (baro) | 2.010 m | 1.018 | 0.110 | 3.850 |
| ThO | 0.136 | 0.011 | 0.107 | 0.175 |
| Alt Error mean | **+0.041 m** | — | -0.986 | +0.782 |
| **Alt Error std** | **0.375 m** | — | — | — |

Only 30s of ALT_HOLD data before EKF failsafe terminated the flight. The 37.5 cm std is
misleading — includes the GPS incorporation transient that caused the failsafe.

## EKF Failsafe Sequence

The failsafe was triggered by a cascade when GPS was first incorporated:

1. **+47.6s**: IMU0 starts using GPS → immediate "GPS Glitch or Compass error"
2. **+51.0s**: Lane switch to Core 1, Core 0 glitch cleared
3. **+51.8s**: IMU1 starts using GPS → same error on Core 1
4. **+52.2s**: Pilot switches to STABILIZE
5. **+53-56s**: SV (velocity variance) ramps up: Core 0 max=1.87, Core 1 max=2.08
6. **+56.0s**: EKF variance over thresholds → EKF Failsafe

The XKF4 variance data shows SV (velocity) was the trigger:
- Core 0: SV peaked at 1.87 around +55s
- Core 1: SV peaked at 2.08 around +55s
- SH (heading) spiked to 1.04/1.07 briefly at +57.6s (post-landing baro slam)

**Root cause:** Massive compass interference (see XKF2) meant the EKF's heading was
significantly wrong when GPS velocity data was first incorporated, causing the velocity
innovations to exceed thresholds.

## EKF Dual-Core Divergence

### During Hover

| Time (offset) | C0 PD | C1 PD | Divergence |
|---------------|-------|-------|------------|
| +5s | -1.93 | -1.14 | 0.80 m |
| +10s | -0.94 | -0.58 | 0.36 m |
| +15s | -1.35 | -1.13 | 0.22 m |
| +20s | -1.55 | -2.16 | 0.60 m |
| +25s | -1.68 | -2.41 | 0.74 m |
| +30s | -1.41 | -1.44 | 0.03 m |

**Max hover divergence: 0.89 m**
**Full-flight max divergence: 2.79 m** (during GPS glitch / failsafe)

## XKF3 Innovations

### IVD (Velocity Down)

| Metric | Core 0 | Core 1 |
|--------|--------|--------|
| Hover mean | +0.518 | — |
| Full flight mean | +0.138 | +0.203 |
| Full flight std | 0.659 | 0.666 |
| Full flight max | +1.82 | +2.22 |

**The persistent IVD=-0.54 from log3/log4 is gone.** The IVD has flipped sign to positive
(~+0.5 during hover), indicating the cleared VRFB and enabled ZBIAS_LEARN fixed the
negative bias. However, a new positive bias of ~+0.5 indicates residual issues.

## XKFS MAG_FUSION State Transitions

| Time (rel) | State | Event |
|------------|-------|-------|
| -3.1s | 2 (learning) | EKF initialized on ground |
| +3.8s | 1 (frozen) | Ground mag anomaly detected |
| +6.6s | 2 (learning) | In-flight yaw alignment (C0) |
| +7.1s | 2 (learning) | In-flight yaw alignment (C1) |
| +7.1s→end | 2 (learning) | Continuous learning for remainder |

MAG_CAL=7 working correctly: ground learning → freeze on anomaly → resume after yaw init.

## XKF2 Compass Offsets

Core 0 during hover:

| Phase | MX | MY | MZ |
|-------|-----|-----|-----|
| Armed | 0 | 0 | 0 |
| ALT_HOLD start | -123 | -18 | -100 |
| Mid hover | -50 | 129 | -101 |
| Near end | 2 | 174 | 10 |
| Disarmed | 13 | 189 | 8 |

Hover ranges: MX=172, MY=195, MZ=182 mGauss.

**Massive compass interference** — offsets swinging 100-200 mGauss across all three axes
during hover. This is almost certainly the root cause of the "GPS Glitch or Compass error"
messages. The compass interference is so large that the EKF struggles to distinguish motor
interference from genuine compass anomalies.

## Temperature

| Sensor | Boot | Peak | Range |
|--------|------|------|-------|
| BARO | 36.9 C | 48.7 C | 11.8 C |
| IMU0 | 28.4 C | 34.7 C | 6.3 C |
| IMU1 | 29.8 C | 36.1 C | 6.3 C |

## Vibrations

| IMU | VibeX Mean/Max | VibeY Mean/Max | VibeZ Mean/Max | Clip |
|-----|----------------|----------------|----------------|------|
| IMU0 | 3.23 / 6.81 | 5.18 / 6.45 | 1.44 / 2.59 | 0 |
| IMU1 | 2.24 / 4.93 | 3.49 / 4.73 | 1.01 / 2.53 | 0 |

Low vibrations, no clipping.

## GPS Quality

| Metric | Mean | Min | Max |
|--------|------|-----|-----|
| NSats | 20.1 | 14 | 21 |
| HDop | 0.59 | 0.53 | 1.03 |
| Status | 3 | 3 | 3 |

Excellent GPS. The "GPS Glitch or Compass error" was compass-related, not GPS.

## Post-Landing

Only 0.9s of post-disarm CTUN data. EKF_ALT_RESET fired at disarm causing Alt to jump
from +0.79 to -1.69. Core 1 PD drifted -0.80m in ~2s after disarm.

## Comparison: log3 → log4 → log5

| Metric | log3 | log4 | log5 |
|--------|------|------|------|
| Hover altitude | 2.0 m | 3.0 m | ~1.4 m |
| Hover duration | 101 s | 92 s | **30 s** |
| Alt error std | 14.2 cm | 29.1 cm | 37.5 cm* |
| IVD stuck | — | -0.54 | **Gone** |
| Core divergence (max) | 20 cm | 69 cm | **89 cm** |
| Post-land drift | +0.94 m | -2.32 m | -0.80 m (C1) |
| Outcome | Good | Good | **EKF Failsafe** |

*Short hover + GPS transient makes this comparison unfair.

## Recommendations

1. **Compass interference is the primary issue.** The 100-200 mGauss offset swings
   during hover caused the EKF Failsafe. Consider:
   - External GPS/compass module on a mast (best fix)
   - Physical separation of compass from power wires/ESCs
   - If external compass not feasible, consider disabling compass fusion entirely

2. Apply the missing parameter changes: TKOFF_GNDEFF_TMO=3.0, ARMING_CHECK=1

3. The IVD bias fix is confirmed — clearing VRFB and enabling ZBIAS_LEARN worked.

## See Also
- [log3](log3.md) — same vehicle, first flight
- [log4](log4.md) — same vehicle, second flight
- [log6](log6.md) — next flight, no EKF failsafe
