# logjk7 Analysis

## Metadata
- **Date**: 2025-01-30
- **Vehicle**: SFD indoor (small fast drone, MambaH743v4)
- **Branch**: SmallFastDrone-4.6-AltHold
- **Log file**: ./logjk7.bin

## Parameters
- `EK3_RNG_USE_HGT = -1` (disabled — key change from logjk6)
- `BARO1_THST_SCALE = -200`
- `INS_ACC_VRFB_Z = -0.4` (still too large, not yet reset)
- `RNGFND1_MAX_CM = 3000`

## Flight Profile
- Indoor flight, **DRAMATICALLY IMPROVED** over logjk6
- 74 second flight duration

## Performance Metrics
- Alt std dev: **10.4cm** (6.5x improvement over logjk6's 67.4cm)
- Alt error mean: 14.0cm
- Alt error max: 34.5cm

## Sensor Status
- Rangefinder present but not used for height switching (EK3_RNG_USE_HGT=-1)

## Analysis

### EK3_RNG_USE_HGT=-1 Validation

Switching from `EK3_RNG_USE_HGT=2.0` (logjk6) to `-1` (logjk7) eliminated the rangefinder height switching feedback loop:

| Metric | logjk6 (HGT=2.0) | logjk7 (HGT=-1) | Change |
|--------|-------------------|-------------------|--------|
| Flight duration | 22s | 74s | 3.4x |
| Alt std dev | 67.4cm | **10.4cm** | 6.5x better |
| Alt error mean | 39.4cm | 14.0cm | 2.8x better |
| Alt error max | 129.4cm | 34.5cm | 3.8x better |

### Remaining Issues
- `INS_ACC_VRFB_Z = -0.4` exceeds ±0.3 clamp — should be reset to 0
- Still room for improvement (compare to logtd2's 6.2cm std)

### Temperature vs Motor AccZ Analysis

Investigation of whether AccZ shift was caused by prop cooling or motor thrust/vibration:

| Factor | Correlation | Effect Size |
|--------|-------------|-------------|
| Motor output | **+0.954** | +0.10 m/s² |
| Temperature | -0.404 | ~0.005 m/s² |

**AccZ by motor state:**
- Low motor (≤1080 PWM): AccZ = -9.828 m/s²
- High motor (>1080 PWM): AccZ = -9.726 m/s²
- Difference: +0.10 m/s² (1% of gravity)

**Conclusion:** AccZ shift is motor thrust/vibration (20x larger than temperature effect).

## Recommendations
- Keep `EK3_RNG_USE_HGT = -1` for indoor flight
- Reset `INS_ACC_VRFB_Z = 0` for further improvement
- Confirmed: BARO1_THST_SCALE is the right approach for baro correction

## See Also
- [logjk6](logjk6.md) — comparison baseline
- [logtd2](logtd2.md) — similar configuration, better results
- [EK3 RNG_USE_HGT feedback loop](../topics/ekf_rng_use_hgt_feedback.md)
