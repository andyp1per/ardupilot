# logtd4 Analysis

## Metadata
- **Date**: 2025-02-12
- **Vehicle**: TD outdoor (optical flow copter)
- **Branch**: SmallFastDrone-4.6-AltHold
- **Log file**: ./logtd4.bin

## Parameters
- Referenced in throttle vs current analysis

## Flight Profile
- Outdoor flight

## Performance Metrics
- Referenced in throttle vs current comparison

## Sensor Status
- Optical flow active
- Rangefinder present

## Analysis

### BARO_THST_SCALE Findings
- Thrust→Baro correlation at 0-3m: -0.387
- Current→Baro correlation at 0-3m: -0.302
- Thrust-Current correlation: 0.988 (extremely high)
- Winner: Throttle

## Recommendations
- Throttle remains better predictor than current for this vehicle

## See Also
- [Throttle vs Current analysis](../topics/throttle_vs_current.md)
