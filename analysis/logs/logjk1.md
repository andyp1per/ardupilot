# logjk1 Analysis

## Metadata
- **Date**: 2025-01-25
- **Vehicle**: SFD indoor (small fast drone, MambaH743v4)
- **Branch**: SmallFastDrone-4.6-AltHold
- **Commit**: Pre-stationary zero velocity fix
- **Log file**: ./logjk1.bin

## Parameters
- `EK3_SRC1_VELXY = 5` (Optical Flow) — provides XY velocity
- `EK3_SRC1_VELZ = 0` (None) — NO Z velocity source
- `EK3_SRC1_POSZ = 1` (Baro) — Z position only

## Flight Profile
- External tester with optical flow
- Multi-flight session (arm/disarm/rearm)

## Performance Metrics
- Baro motor-on noise std: **2.64m** (9.4x motors-off ratio)

## Sensor Status
- Optical flow providing XY velocity
- No Z velocity source
- Rangefinder present

## Analysis

### Problem: Z-bias drifted to +0.92 m/s² during disarmed period

**Root Cause:**
1. Optical flow provides XY velocity → `PV_AidingMode != AID_NONE`
2. Original code: `horizInhibit = (PV_AidingMode == AID_NONE)` → FALSE
3. So bias learning was NOT inhibited by `horizInhibit`
4. Ground effect inhibit only works when armed (`takeoff_expected` or `touchdown_expected`)
5. When **disarmed**, Z-bias learning was unrestricted
6. Without Z velocity, bias is unobservable → drifted to +0.92 m/s²

**Timeline (85-92s):**
```
t=85.9s: Disarm
t=86.2s: Z-bias starts rapid change (-0.06 → -0.31)
t=89.1s: Z-bias jumps to 0.00 (reset?)
t=89.7s-91.1s: Z-bias climbs from +0.06 to +1.00
t=92.1s: Rearm with Z-bias = +0.92 m/s²
```

### Fixes Applied

1. **Initial fix (commit `49187dac64`)**: Inhibit Z-bias learning when `EK3_SRC1_VELZ = 0`, regardless of XY aiding status.
2. **Better fix — Stationary Zero Velocity Fusion (commit `0ff35c20b4`)**: Fuse synthetic zero velocity when `onGround && !motorsArmed`, making bias observable in all aiding modes.

### Replay Validation

| Phase | Time | Original Z-bias | Fixed Z-bias |
|-------|------|-----------------|--------------|
| Pre-arm (stationary) | 40s | -0.20 m/s² | 0.00 m/s² |
| Armed on ground | 48s | -0.06 m/s² | 0.00 m/s² |
| Early flight | 55s | -0.10 m/s² | 0.00 m/s² |
| Disarm period | 86-90s | -0.10 → -0.29 m/s² | 0.00 m/s² |
| Second flight | 100s | +0.92 m/s² | 0.00 m/s² |

**Total improvement:** 0.75 m/s² less Z-bias drift (from ±0.75 to 0.00)

### BARO_THST_SCALE Analysis
- Thrust→Baro correlation: -0.89 (strong)
- Estimated THST_SCALE: -551 Pa
- Raw baro error: 2.70m → After compensation: 1.21m (55% improvement)

## Recommendations
- Enable stationary zero velocity fusion (applied)
- Consider BARO1_THST_SCALE = -550 for this vehicle

## See Also
- [EK3 RNG_USE_HGT feedback loop](../topics/ekf_rng_use_hgt_feedback.md)
- [Throttle vs Current analysis](../topics/throttle_vs_current.md)
