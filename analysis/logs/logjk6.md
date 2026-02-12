# logjk6 Analysis

## Metadata
- **Date**: 2025-01-30
- **Vehicle**: SFD indoor (small fast drone, MambaH743v4)
- **Branch**: SmallFastDrone-4.6-AltHold
- **Log file**: ./logjk6.bin

## Parameters
- `EK3_RNG_USE_HGT = 2.0` (rangefinder height switching enabled)
- `BARO1_THST_SCALE = -200`
- `INS_ACC_VRFB_Z = -0.4`
- `RNGFND1_MAX_CM = 3000`

## Flight Profile
- Indoor flight, **UNFLYABLE**
- 22 second flight duration

## Performance Metrics
- Alt std dev: **67.4cm** (worst of the series)
- Alt error mean: 39.4cm
- Alt error max: 129.4cm

## Sensor Status
- Rangefinder active but EK3_RNG_USE_HGT=2.0 caused feedback loop

## Analysis

### EK3_RNG_USE_HGT Feedback Loop Discovery

With `EK3_RNG_USE_HGT > 0`, the rangefinder height threshold uses EKF altitude. When baro corrupts EKF altitude above the threshold, the rangefinder gets locked out — creating a feedback loop:

1. Baro noise corrupts EKF altitude → reports higher than actual
2. EKF altitude exceeds `EK3_RNG_USE_HGT` threshold
3. Rangefinder height source disabled
4. EKF relies entirely on corrupted baro
5. EKF altitude stays wrong → rangefinder stays locked out

### BARO_THST_SCALE Findings
- Thrust→Baro correlation at 0-3m: -0.615
- Current→Baro correlation at 0-3m: -0.495

### INS_ACC_VRFB_Z Issue
- Value was -0.4, which exceeds the ±0.3 clamp
- Learned during problematic flights — should be reset to 0

## Recommendations
- Switch to `EK3_RNG_USE_HGT = -1` (disable rangefinder height switching)
- Reset `INS_ACC_VRFB_Z = 0`
- See logjk7 for results of these changes

## See Also
- [logjk7](logjk7.md) — improved results with EK3_RNG_USE_HGT=-1
- [EK3 RNG_USE_HGT feedback loop](../topics/ekf_rng_use_hgt_feedback.md)
