# log17 Analysis

## Metadata
- **Date**: 2026-02-16
- **Vehicle**: TD-Matek-5 outdoor (MatekH743-bdshot)
- **Firmware**: V4.6.3v2-SFD (5282cc3d)
- **Branch**: SmallFastDrone-4.6-AltHoldv2
- **Log file**: /mnt/c/support/bragg/AltHold-SP/TD-Matek-5Inch/log17.bin
- **Purpose**: Test EKF height datum reset — land, reset, re-arm

## Summary

**Test failed: pilot could not re-arm after landing.** The blocking issue was the compass
earth-field consistency check, not the altitude reset.

## Flight Timeline

| Time | Event |
|------|-------|
| +5.1s | Boot |
| +24.7s | **ARMED** (no GPS origin — mag field check skipped) |
| +24.7s | EKF_ALT_RESET (Id=60), EKF_POS_RESET (Id=57) |
| +27.5s | EKF3 origin set, SET_HOME |
| +32.4s | In-flight yaw alignment complete |
| +61.8s | LAND_COMPLETE_MAYBE |
| +62.6s | LAND_COMPLETE |
| +62.7s | **DISARMED** — "Field Elevation Set: 139m" triggers full EKF reinit |
| +63.7s | EKF3 IMU0/IMU1 reinitialised |
| +65.2s | Tilt alignment complete |
| +67.0s | **PreArm: Check mag field (xy diff:153>100)** |
| +67→129s | 13 consecutive arm rejections, all "Check mag field (xy diff:~150>100)" |
| +81.9s | Also briefly: "EKF attitude is bad", "core 0 unhealthy" → auto reinit |
| — | Log ends, never re-armed |

## Root Cause: Compass Earth-Field Check

The first arm succeeded because **GPS origin was not set yet**. The compass earth-field
XY check (`check_mag_field_strength()`) requires `ahrs.get_location()` to compute the
WMM reference field. Without a location, the check is skipped entirely.

After the EKF reinitialised on disarm and GPS origin was re-established, the check became
active. The measured compass XY field (~382 mGauss) was ~150 mGauss above the expected
WMM horizontal field (~220 mGauss), exceeding the 100 mGauss `ARMING_MAGTHRESH` threshold.

The compass calibration was poor — field magnitude varied nearly 2x with heading, indicating
bad hard-iron/soft-iron correction. (This was before the compass was recalibrated.)

## Secondary Issue: Altitude Offset After EKF Reinit

The "Field Elevation Set" feature triggers a **full EKF reinit** (not just `resetHeightDatum()`).
After reinit, the new GPS origin altitude (143.5m AMSL) differed from the original home
(139.15m AMSL), creating a ~4.4m altitude offset — the controller thought it was 4.4m above
home while on the ground.

This would have been dangerous in AltHold if arming had succeeded, as the controller would
have tried to descend 4.4m into the ground.

## Observations

1. **Not a resetHeightDatum() test** — the full EKF reinit path was triggered, not just
   the height datum reset we fixed
2. **Compass cal was the blocker** — recalibration needed before retesting
3. **"Field Elevation Set" needs investigation** — triggers full reinit with GPS origin
   mismatch risk, rather than a clean height datum reset

## See Also
- [log15](log15.md) — previous flight with -0.44m altitude at ARM (the bug resetHeightDatum fix targets)
- [log26](log26.md) — subsequent test of resetHeightDatum() fix (with recalibrated compass)
