# BARO1_THST_FILT Implementation

## Summary

Low-pass filter on throttle before baro thrust compensation to reduce altitude
transients during rapid throttle changes.

## Problem Identified

From [logtd3](../logs/logtd3.md) analysis: during rapid throttle changes, the
instantaneous thrust compensation causes step changes in corrected baro altitude,
contributing to altitude transients.

- Throttle ramped 0% → 33% in 0.6 seconds
- Without filter: Instant ~4m altitude correction from thrust scaling alone
- This adds to the already chaotic baro behavior during transients

## Implementation

**New parameter: BARO1_THST_FILT**
- Default: 1.0 Hz cutoff
- Set to 0 to disable (original behavior)
- Filter smooths throttle input to reduce altitude transients

**Code location:** `libraries/AP_Baro/AP_Baro.cpp:thrust_pressure_correction()`

## Effect of 1Hz Filter (logtd3 scenario)

| Time | Raw Throttle | Filtered | Raw Correction | Filtered Correction |
|------|--------------|----------|----------------|---------------------|
| 0.0s | 0% | 0% | 0m | 0m |
| 0.1s | 5% | 1.9% | -0.6m | -0.2m |
| 0.3s | 23% | 12.7% | -2.8m | -1.6m |
| 0.4s | 33% | 20.5% | -4.1m | -2.5m |
| 0.6s | 35% | 30% | -4.3m | -3.7m |

The filter reduces peak altitude transients by ~40% during rapid throttle changes.

## Trade-offs

| Setting | Transient Behavior | Steady-State | Best For |
|---------|-------------------|--------------|----------|
| FILT = 0 (disabled) | Sharp step changes | Immediate | Outdoor/aggressive |
| FILT = 0.5 Hz | Very smooth | ~2s lag | Indoor/gentle |
| FILT = 1.0 Hz | Smooth | ~1s lag | Indoor/general |
| FILT = 2.0+ Hz | Mild smoothing | Fast | Outdoor/responsive |

## Recommended Settings for Indoor Flight

```
EK3_RNG_USE_HGT = -1          # Disable rangefinder height switching
BARO1_THST_SCALE = -147       # Calibrated thrust compensation (vehicle-specific)
BARO1_THST_FILT = 1.0         # Filter throttle transients
INS_ACC_VRFB_Z = 0            # Reset if previously corrupted
TKOFF_GNDEFF_ALT = 5          # Keep ground effect protection to 5m
```

## How to Calibrate BARO1_THST_SCALE

1. Fly a stable hover with rangefinder and log data
2. Compare baro altitude vs rangefinder altitude during steady hover
3. Calculate: `THST_SCALE = -(baro_error_m × 12 Pa/m) / hover_throttle`
4. Verify across multiple throttle levels

### Vehicle-Specific Values Found

| Vehicle | BARO1_THST_SCALE | Hover Throttle | Notes |
|---------|-----------------|----------------|-------|
| SFD indoor | -550 Pa | ~0.10 | Small, high prop wash |
| TD optical flow | -147 Pa | ~0.39 | Larger, less prop wash |
| TD outdoor | -100 Pa | ~0.125 | High altitude, conservative |

## See Also
- [logtd3](../logs/logtd3.md) — incident that motivated this feature
- [logtd1](../logs/logtd1.md) — THST_SCALE calibration example
- [Throttle vs Current analysis](throttle_vs_current.md) — why throttle, not current
