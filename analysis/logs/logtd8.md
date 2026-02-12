# logtd8 Analysis

## Metadata
- **Date**: 2025-02-12
- **Vehicle**: TD outdoor (user drone, Quad BetaFlight-X)
- **Branch**: SmallFastDrone-4.6-AltHold
- **Log file**: ./logtd8.bin

## Parameters
- `EK3_SRC1_POSXY = 3` (GPS), `EK3_SRC1_VELXY = 3` (GPS), `EK3_SRC1_VELZ = 3` (GPS)
- `EK3_SRC1_POSZ = 1` (Baro)
- `EK3_RNG_USE_HGT = -1` (disabled)
- `EK3_IMU_MASK = 3` (both IMUs)
- `BARO1_THST_SCALE = -100`, `BARO1_THST_FILT = 1.0`
- `INS_ACC_VRFB_Z = -0.568` (IMU0), `INS_ACC2_VRFB_Z = -0.565` (IMU1) — **exceeds ±0.3 clamp!**
- `ACC_ZBIAS_LEARN = 3` (bitmask: learn+save, use saved, disable ground learning)
- `TKOFF_GNDEFF_ALT = 1.0`
- `MOT_THST_HOVER = 0.137` (very overpowered)
- `PSC_POSZ_P = 1.0`, `PSC_VELZ_P = 5.0`
- `RNGFND1_TYPE = 24` (TOF), `RNGFND1_MAX_CM = 3000`

## Flight Profile
- t=31.4s: Boot (LowBattery event at startup)
- t=45.3s: AutoArm in STABILIZE
- t=54.5s: Switch to ALT_HOLD
- t=54-80s: Climb from ground to ~8m (no GPS)
- t=80s: GPS 3D fix (5 sats)
- t=80-118s: Hover ~7-11m (GPS 3D)
- t=118s: DGPS (9+ sats)
- t=118-181s: Continue climbing to ~14m, hover
- t=181s: Switch to STABILIZE, descend and land
- t=195.7s: EKF lane switch (core 1 → core 0 → back)
- t=197.4s: Disarm, VRFB bias saved
- **Total ALT_HOLD: ~126 seconds**

## Performance Metrics

### Overall ALT_HOLD (54.5-181s)
- Alt error: mean **-0.05m**, std **0.55m**
- Max undershoot: -1.52m, Max overshoot: +3.45m

### By GPS Phase
| Phase | Duration | Alt Err Mean | Alt Err Std | Notes |
|-------|----------|-------------|-------------|-------|
| No GPS (54-80s) | 26s | -0.32m | **0.76m** | Climbing, no velocity aiding |
| GPS 3D (80-118s) | 38s | -0.14m | **0.28m** | Good — GPS velocity helping |
| DGPS (118-181s) | 63s | +0.11m | **0.51m** | Slightly worse — higher altitude, more wind? |

### Stable Hover Segments
| Period | Altitude | Alt Err Std | ThO |
|--------|----------|-------------|-----|
| 75-90s (~7.5m) | 7.1-7.7m | **0.20m** | 0.139 |
| 105-120s (~11m) | 10.3-11.5m | **0.35m** | 0.145 |
| 140-160s (~14.4m) | 13.7-15.1m | **0.36m** | 0.158 |

## Sensor Status

### GPS
- NoFix until t=80s, 3D at 80s (5 sats), DGPS at 118s (9+ sats)
- Good coverage for most of the flight

### Rangefinder
- Status distribution: NoData=898, OutOfRangeLow=546, Good=2087
- Effective range ~12m; NoData above that
- Good readings at 3-12m altitude

### Baro
- Temperature: 44.8°C at arm → 27.4°C at end of flight (**-17.3°C drop**)
- Baro-RFND offset (where RFND good): mean **+1.17m**, std 1.11m
- Baro consistently reads higher than rangefinder

### Vibration
- VibeX: 2-3 m/s², VibeY: 4-5 m/s², VibeZ: 3-4 m/s²
- All within acceptable limits

### XKF4 Status
- TS=50 (0b110010) before GPS: velocity + airspeed + drag timeout
- TS=58 (0b111010) after GPS: adds height timeout (baro-only for Z)
- FS=0 throughout (no faults)

## Analysis

### BUG FOUND: VRFB Clamp Mismatch — Stored Values Exceed Applied Correction

The VRFB values stored in parameters (-0.568, -0.565) far exceed the ±0.3 clamp
applied when the correction is set in the EKF. This creates a permanent gap:

```
Boot:  VRFB loaded = -0.568 (IMU0), -0.565 (IMU1)
Apply: Correction clamped to -0.300 (both IMUs)
Gap:   -0.268 (IMU0), -0.265 (IMU1) — must be re-learned by EKF every flight

During hover:
  EKF residual AZ ≈ -0.27 (learning the uncorrected portion)
  Total = -0.27 + -0.30 = -0.57
  Saved on disarm: -0.568, -0.565 (matches total)

Next flight: same cycle repeats — only -0.3 applied, -0.27 re-learned
```

**The learning converges** (doesn't diverge), but the parameter stores a value
that can never be fully used. Each flight wastes time re-learning the same
~0.27 m/s² residual.

**Root cause:** The ±0.3 clamp is too small for this vehicle's vibration
rectification. The true bias is ~0.57 m/s² — nearly 2x the clamp.

**Options:**
1. Increase clamp to ±0.6 m/s² (covers this vehicle)
2. Also clamp the value at save time (so parameter doesn't grow misleadingly)
3. Add per-flight GCS warning when VRFB exceeds clamp

### EKF Accel Bias Behavior

| Phase | Core 0 AZ | Core 1 AZ | Notes |
|-------|----------|----------|-------|
| Pre-arm | -0.25 | -0.24 | Stable (zero-vel fusion + ground learning disabled) |
| Early hover (60s) | -0.10 | -0.07 | Ground effect just cleared, bias adjusting |
| Stable hover (90-180s) | -0.25 to -0.28 | -0.18 to -0.31 | Settling around -0.27 |

EKF is successfully learning the residual vibration rectification bias during
hover, even with only baro for Z position.

### Baro Thermal Drift

17.3°C temperature drop during flight, consistent with logtd5-7 pattern.
Baro reads ~1.2m higher than rangefinder on average — likely thermal drift
plus residual propwash at hover throttle.

### EKF Lane Switch at Landing

At t=195.7s during landing, EKF switched from core 0 to core 1 and back.
Possibly triggered by landing transients. Core 0 went "unhealthy" briefly
(PreArm message at t=206s). Worth monitoring but not a flight-critical issue.

### Hover Bias Saved on Disarm

| Parameter | Boot Value | Saved Value | Change |
|-----------|-----------|------------|--------|
| INS_ACC_VRFB_Z (IMU0) | -0.443 | **-0.568** | -0.125 |
| INS_ACC2_VRFB_Z (IMU1) | -0.519 | **-0.565** | -0.046 |
| MOT_THST_HOVER | 0.131 | 0.137 | +0.006 |

IMU0 value changed significantly; IMU1 was already close to converged.

## Recommendations

1. **Increase VRFB clamp** from ±0.3 to ±0.6 m/s² for vehicles with large
   vibration rectification. This vehicle's true bias (~0.57 m/s²) exceeds
   the current clamp.
2. **Add save-time clamping** so the VRFB parameter doesn't store values
   that can never be fully applied.
3. **Flight is working well** — altitude hold std of 0.20-0.36m during
   stable hover is good for an outdoor flight with baro + GPS.
4. **Baro thermal drift** remains the largest error source at altitude.
   Consider TCAL recalibration.

## See Also
- [logtd5](logtd5.md), [logtd6](logtd6.md), [logtd7](logtd7.md) — same vehicle
- [Baro thermal drift](../topics/baro_thermal_drift.md)
