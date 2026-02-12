# logtd2 Analysis

## Metadata
- **Date**: 2025-01-30
- **Vehicle**: TD indoor (optical flow copter)
- **Branch**: SmallFastDrone-4.6-AltHold
- **Log file**: ./logtd2.bin

## Parameters
- `EK3_RNG_USE_HGT = -1` (disabled)
- `BARO1_THST_SCALE = -147`
- `INS_ACC_VRFB_Z = 0.044`
- `RNGFND1_MAX_CM = 700`

## Flight Profile
- Indoor flight, **GOOD** results
- 188 second flight duration (longest of the series)

## Performance Metrics
- Alt std dev: **6.2cm** (best of all analyzed logs)
- Alt error mean: 4.7cm
- Alt error max: 26.7cm

## Sensor Status
- Rangefinder active
- Optical flow active
- BARO1_THST_SCALE compensating propwash

## Analysis

### Excellent Altitude Hold Performance

Comparison with logjk series:

| Metric | logjk6 (worst) | logjk7 | logtd2 |
|--------|-----------------|---------|--------|
| Flight duration | 22s | 74s | 188s |
| Alt std dev | 67.4cm | 10.4cm | **6.2cm** |
| Alt error mean | 39.4cm | 14.0cm | **4.7cm** |
| Alt error max | 129.4cm | 34.5cm | **26.7cm** |

### Why logtd2 is Better Than logjk7

1. `INS_ACC_VRFB_Z = 0.044` (reasonable) vs -0.4 (exceeded clamp)
2. `BARO1_THST_SCALE = -147` (calibrated) vs -200 (estimated)
3. Different vehicle characteristics

### BARO_THST_SCALE Findings
- Thrust→Baro correlation at 0-3m: -0.106
- Current→Baro correlation at 0-3m: -0.112
- Both weak — thrust compensation is already removing the correlation

## Recommendations
- This configuration represents the best achieved indoor altitude hold
- Recommended as baseline settings for similar vehicles

## See Also
- [logjk7](logjk7.md) — comparison with EK3_RNG_USE_HGT=-1 on SFD vehicle
- [EK3 RNG_USE_HGT feedback loop](../topics/ekf_rng_use_hgt_feedback.md)
