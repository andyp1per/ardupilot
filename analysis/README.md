# EKF3 Altitude Hold Analysis

Flight log analysis for the SmallFastDrone-4.6-AltHold branch. Three vehicles tested:
- **SFD indoor** — small fast drone (MambaH743v4), indoor optical flow, logjk series
- **TD** — optical flow copter (indoor/outdoor), logtd series
- **SmallFastDronev1** — BF_X quad, indoor optical flow + rangefinder, log197/198/201/202/208 series

## Master Summary Table

| Log | Date | Vehicle | Key Params Changed | Alt Err Std | Key Finding |
|-----|------|---------|--------------------|-------------|-------------|
| [logjk1](logs/logjk1.md) | Jan 25 | SFD indoor | Optical flow, no VELZ | — | Z-bias drifted to +0.92 m/s² while disarmed; led to stationary zero-vel fusion fix |
| [logjk2](logs/logjk2.md) | Jan 27 | SFD indoor | Same | — | THST_SCALE=-556 Pa estimated (consistent with logjk1) |
| [logjk3](logs/logjk3.md) | Jan 28 | SFD indoor | Same | — | Worst baro noise (2.71m std), THST_SCALE=-280 Pa |
| [logjk4](logs/logjk4.md) | Jan 28 | SFD indoor | VRFB_Z=-0.199, GNDEFF=0.8 | **FAILED** | Frozen correction + ground effect conflict prevents takeoff |
| [logjk5](logs/logjk5.md) | Jan 28 | SFD indoor | Same | — | Best baro noise of logjk series (1.01m std) |
| [logjk6](logs/logjk6.md) | Jan 30 | SFD indoor | RNG_USE_HGT=2.0 | **67.4cm** | Unflyable — rangefinder feedback loop discovered |
| [logjk7](logs/logjk7.md) | Jan 30 | SFD indoor | **RNG_USE_HGT=-1** | **10.4cm** | 6.5x improvement — confirmed feedback loop fix |
| [logjk8](logs/logjk8.md) | Jan 30 | SFD indoor | Same as logjk7 | — | Best throttle→baro correlation (-0.800) |
| [logjk9](logs/logjk9.md) | Jan 30 | SFD indoor | Same | — | Not yet analyzed in detail |
| [logtd1](logs/logtd1.md) | Jan 30 | TD indoor | GNDEFF=1.0, no THST | — | Severe baro propwash (+4.8m offset); THST_SCALE=-147 calculated |
| [logtd2](logs/logtd2.md) | Jan 30 | TD indoor | **RNG_USE_HGT=-1**, THST=-147 | **6.2cm** | Best altitude hold achieved across all logs |
| [logtd3](logs/logtd3.md) | Jan 30 | TD indoor | THST=-147, RNG NoData | **CEILING** | Ceiling hit — rangefinder had no data; motivated THST_FILT |
| [logtd4](logs/logtd4.md) | Feb 12 | TD outdoor | Same | — | Throttle vs current comparison data |
| [logtd5](logs/logtd5.md) | Feb 12 | TD outdoor | PSC_P=0.3, V=1.8 | **47.3cm** | PSC gains too low; baro thermal drift -22°C; VRFB_Z on wrong IMU |
| [logtd6](logs/logtd6.md) | Feb 12 | TD outdoor | **PSC_P=1.0, V=5.0** | **17.9cm** | 2.6x improvement from PSC tuning alone |
| [logtd7](logs/logtd7.md) | Feb 12 | TD outdoor | Same as logtd6 | 23-26cm | Rangefinder at 6m didn't beat baro-only at 27m |
| [logtd8](logs/logtd8.md) | Feb 12 | TD outdoor | GPS+baro, VRFB=-0.57 | **20-36cm** | VRFB clamp finding: true bias ~0.57; clamp raised to ±0.6 |
| [log197](logs/log197.md) | Feb 13 | SmallFastDronev1 indoor | Stabilize, THST=0, TCAL on | — | VRF=+0.089; TCAL Z-axis overcorrects 3.8x (upside-down cal) |
| [log198](logs/log198.md) | Feb 13 | SmallFastDronev1 indoor | Loiter, RNG_USE_HGT=3 | **CRASH** | Terrain offset feedback loop → flow velocity 2-4x → backward lean |
| [log201](logs/log201.md) | Feb 14 | SmallFastDronev1 indoor | Stabilize, TCAL recal | — | TCAL fix confirmed; yaw inconsistency 24° (MOTCT=0 + MAG_CAL=4 root cause found) |
| [log202](logs/log202.md) | Feb 14 | SmallFastDronev1 indoor | Loiter, PSC_P=1.0 | — | Alt hold improved (0.39m mean); yaw twitchy (ANG_YAW_P=17.8); terrain offset ±1.7m |
| [log208](logs/log208.md) | Feb 14 | SmallFastDronev1 outdoor | MAG_CAL=7, MOTCT=2 | — | MAG_CAL=7 verified (5.3° vs 24° divergence); takeoff overshoot; roll 8-10 Hz oscillation |

## Earlier Development Logs (log1-log12)

These logs document the initial development of ground effect compensation and Z-bias
inhibition on the SFD vehicle. Key milestones:

| Log | Purpose | Outcome |
|-----|---------|---------|
| log4 | Original altitude hold analysis | Ground effect gap identified; TKOFF_GNDEFF_ALT created |
| log5 | TKOFF_GNDEFF_ALT=0.8 test | Innovation clamping fixed; velocity drift still present |
| log6 | Ground motor test | +0.084 m/s² AccZ shift from motors confirmed |
| log7 | Temperature vs motor analysis | Motor effect 20x larger than temperature effect |
| log8 | Z-bias inhibition flight test | Dramatic improvement: ±0.10m hover, 46s stable |
| log1 | AID_NONE mode fix | Zero velocity fusion for pre-arm stability |
| log2 | Hover bias calibration flight | Learned INS_ACC_VRFB_Z: IMU0=0.073, IMU1=0.048 |
| log3 | Hover bias validation flight | Frozen correction working; EKF residual converged to zero |

## Topic Files

Cross-cutting analysis across multiple logs:

| Topic | Description |
|-------|-------------|
| [Throttle vs Current](topics/throttle_vs_current.md) | 17-log comparison: throttle wins 9-2 for baro compensation |
| [Baro Thermal Drift](topics/baro_thermal_drift.md) | logtd5-7: 21°C temp drop causes 1.5m drift |
| [EK3_RNG_USE_HGT Feedback](topics/ekf_rng_use_hgt_feedback.md) | logjk6→7: feedback loop discovery and fix |
| [BARO1_THST_FILT](topics/baro_thrust_filter.md) | Throttle filter implementation, calibration guide |
| [Indoor Loiter — SFDv1](topics/indoor_loiter_sfdv1.md) | Crash analysis, TCAL upside-down discovery, VRF estimation, thermal drift |
| [Outdoor Tuning — SFDv1](topics/outdoor_tuning_sfdv1.md) | Aggressive takeoff (MOT_THST_HOVER mismatch), roll 8-10 Hz limit cycle, gain recommendations |
| [MAG_CAL=7 Verification](topics/mag_cal_7_verification.md) | GROUND_AND_INFLIGHT mode verified: all state transitions correct, 5.3° vs 24° yaw divergence |

## Key Implementation Commits

Development commits on this branch (unsquashed). Squashed PR is on `pr-z-bias-squashed`.

**EKF3 core changes:**

| Commit | Description |
|--------|-------------|
| `a0f30dad8f` | Inhibit Z-bias learning during ground effect |
| `51232123fa` | Inhibit Z-bias learning without Z velocity source |
| `0625e3264e` | Check actual Z velocity availability for bias inhibition |
| `2661900e57` | Fuse zero velocity when stationary on ground |
| `be3f38b595` | Use frozen correction for hover Z-bias |
| `2b2b439a9b` | Increase hover Z-bias correction clamp to ±0.6 m/s² |

**Parameter storage and abstraction:**

| Commit | Description |
|--------|-------------|
| `7d1c3ab2c3` | Per-IMU vibration rectification bias storage (INS_ACCx_VRFB_Z) |
| `4eff5193dc` | AHRS hover Z-bias accessor methods |

**Copter-level integration:**

| Commit | Description |
|--------|-------------|
| `35a7f215dc` | TKOFF_GNDEFF_ALT parameter for ground effect threshold |
| `b417228a68` | TKOFF_GNDEFF_TMO parameter for ground effect timeout |
| `78463d24c5` | Hover Z-bias learning in ArduCopter |
| `b9595ea59a` | ACC_ZBIAS_LEARN bitmask with ground inhibit option |
| `1e3bca8f8f` | Inhibit accel bias learning during acro flight |

**Tests:**

| Commit | Description |
|--------|-------------|
| `2439587c5a` | Autotests for ground effect compensation active in EKF |

## Open Issues

1. **Ground effect + frozen correction conflict** — [logjk4](logs/logjk4.md) shows
   frozen correction creates phantom acceleration during ground effect; fix proposed
   but not yet implemented
2. **Post-landing EKF divergence** — ground effect protection accumulates error that
   causes drift after disarm
3. **Ground effect flags clear too early** — uses EKF altitude (which can be wrong)
   instead of rangefinder
4. **Roll oscillation on SFDv1** — [log208](logs/log208.md) shows 8-10 Hz limit cycle from
   high autotune gains + asymmetric airframe. See [outdoor tuning topic](topics/outdoor_tuning_sfdv1.md)
   for recommended gain reductions (ANG_RLL_P: 27→20, ANG_PIT_P: 31→27).
5. **Aggressive takeoff on SFDv1** — MOT_THST_HOVER=0.125 vs actual 0.069; 5.5x acceleration
   overshoot at takeoff. Fix: MOT_THST_HOVER=0.07, TKOFF_SLEW_TIME=1.0-1.5.

## Resolved Issues

1. **VRFB clamp raised to ±0.6** — [logtd8](logs/logtd8.md) showed true vibration
   rectification of ~0.57 m/s² exceeded the original ±0.3 clamp. MAX_HOVER_BIAS_CORRECTION
   raised to 0.6f in both InitialiseFilter and setHoverZBiasCorrection.
2. **Zero velocity fusion CI failures** — Fusing at IMU rate (400Hz) overconstrained the filter;
   fusing before tiltAlignComplete caused "EKF attitude is bad". Fixed by gating behind
   `fuseHgtData` and `tiltAlignComplete`. See [EKF3 CLAUDE.md](../libraries/AP_NavEKF3/CLAUDE.md).
3. **Ground effect default threshold** — TKOFF_GNDEFF_ALT=0 changed default from 0.50m to 0m;
   fixed with `is_positive()` fallback to original 0.50m.
4. **TCAL upside-down calibration** — [log201](logs/log201.md) confirmed recalibration right-side
   up fixed the 3.8x Z-axis overcorrection. ACC1_Z went from 55844 to -3597.
5. **Yaw inconsistency from motor interference** — [log201](logs/log201.md) traced 24° divergence
   to COMPASS_MOTCT=0 + MAG_CAL=4. Fixed with COMPASS_MOTCT=2 + new EK3_MAG_CAL=7
   ([verified in log208](topics/mag_cal_7_verification.md): divergence reduced to 5.3°).

## Best Known Configuration

### Indoor (small drone, significant propwash)
```
EK3_RNG_USE_HGT = -1
BARO1_THST_SCALE = -550    (vehicle-specific, calibrate per airframe)
BARO1_THST_FILT = 1.0
TKOFF_GNDEFF_ALT = 5
INS_ACC_VRFB_Z = 0         (let it re-learn)
ACC_ZBIAS_LEARN = 2
```

### Indoor (low-throttle quad, thermal-dominated baro)
```
EK3_RNG_USE_HGT = -1
BARO1_THST_SCALE = -20         (vehicle-specific, low throttle = less propwash)
TKOFF_GNDEFF_TMO = 3.0
PSC_POSZ_P = 1.0
INS_ACC_VRFB_Z = 0.09          (measured VRF for this airframe)
ACC_ZBIAS_LEARN = 3
INS_TCAL1_ACC*_Z = 0           (zero until recalibrated right-side up)
```

### Outdoor (moderate propwash)
```
PSC_POSZ_P = 1.0
PSC_VELZ_P = 5.0
BARO1_THST_SCALE = -100 to -150
EK3_RNG_USE_HGT = -1
```
