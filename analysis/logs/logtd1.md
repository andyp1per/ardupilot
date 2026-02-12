# logtd1 Analysis

## Metadata
- **Date**: 2025-01-30
- **Vehicle**: TD outdoor (optical flow copter)
- **Branch**: SmallFastDrone-4.6-AltHold
- **Log file**: ./logtd1.bin

## Parameters
- `ACC_ZBIAS_LEARN = 3` (learn+save+use)
- `INS_ACC_VRFB_Z = 0.0567` (previously learned)
- `TKOFF_GNDEFF_ALT = 1.0`
- `EK3_RNG_USE_HGT = -1` (rangefinder not used for height)
- `EK3_SRC1_VELXY = 5` (optical flow)
- `RNGFND1_MAX_CM = 700`

## Flight Profile
- 73 second hover at ~2m actual altitude (per rangefinder)

## Performance Metrics
- Severe baro propwash: +4.8m steady-state error during hover
- Baro reads +7.0m while rangefinder shows 2.2m

## Sensor Status
- Rangefinder active (showing 2.2m at hover)
- Optical flow active
- Baro severely affected by propwash

## Analysis

### Severe Baro Propwash Error

| Phase | Throttle | Baro | RFND | Comment |
|-------|----------|------|------|---------|
| Pre-arm (motors off) | 0.00 | -0.6m | 0.01m | Correct |
| Motors spinning (on ground) | 0.08 | **-3.1m** | 0.02m | Propwash increases pressure |
| Peak propwash | 0.37 | **-6.2m** | 0.01m | Maximum pressure distortion |
| Liftoff transition | 0.37 | **+4.2m** | 0.06m | Exits propwash zone |
| Stable hover | 0.38 | +7.0m | 2.2m | **+4.8m error locked in** |

**Root cause:** When the copter lifts off, it exits the propwash pressure zone, causing a **12m altitude jump in <0.5 seconds** (from -6m to +6m).

### Ground Effect Working Correctly

During ground effect (takeoff_expected=1), the EKF correctly ignores the baro. But when ground effect clears, EKF has no choice but to accept the (wrong) baro reading since `EK3_RNG_USE_HGT=-1`.

### BARO1_THST_SCALE Can Fix This

**Calculation:**
```
BARO1_THST_SCALE = -(error_m × 12 Pa/m) / throttle
                 = -(4.8 × 12) / 0.392
                 = -147 Pa
```

**Result with BARO1_THST_SCALE = -147:**

| Phase | Throttle | Raw Baro | Correction | Result |
|-------|----------|----------|------------|--------|
| Ground (motors off) | 0.00 | 0m | 0m | 0m |
| Ground (motors on) | 0.08 | -3m | -1m | -4m (ignored by GndEff) |
| **Hover** | 0.38 | +7m | **-5m** | **+2m** (matches RFND!) |

### Alternative: Enable rangefinder for height

Set `EK3_RNG_USE_HGT=70` (use rangefinder below 70% of max range):
- RNGFND1_MAX_CM=700 → uses RFND below 4.9m
- Combined with `TKOFF_GNDEFF_TMO=3` keeps ground effect active during transient

## Recommendations

**Option A (simple):** `BARO1_THST_SCALE = -147`
- Directly compensates propwash-induced offset
- Calibration specific to this airframe

**Option B (robust):** `EK3_RNG_USE_HGT=70` + `TKOFF_GNDEFF_TMO=3`
- Uses rangefinder as truth source
- More robust to varying conditions

## See Also
- [logtd2](logtd2.md) — flight with EK3_RNG_USE_HGT=-1
- [logtd3](logtd3.md) — ceiling hit when rangefinder failed
- [Baro thrust filter](../topics/baro_thrust_filter.md)
- [Throttle vs Current analysis](../topics/throttle_vs_current.md)
