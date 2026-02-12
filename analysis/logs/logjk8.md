# logjk8 Analysis

## Metadata
- **Date**: 2025-01-30
- **Vehicle**: SFD indoor (small fast drone, MambaH743v4)
- **Branch**: SmallFastDrone-4.6-AltHold
- **Log file**: ./logjk8.bin

## Parameters
- Same configuration as logjk7 series

## Flight Profile
- Indoor flight

## Performance Metrics
- Referenced in throttle vs current analysis

## Sensor Status
- Optical flow active
- Rangefinder present

## Analysis

### BARO_THST_SCALE Findings
- Thrust→Baro correlation at 0-3m: -0.800 (strongest of logjk series)
- Current→Baro correlation at 0-3m: -0.516
- Thrust-Current correlation: 0.570

### Notable
- Strongest throttle→baro correlation of any logjk log at 0-3m
- Confirms throttle is a better predictor than current for baro pressure error

## Recommendations
- Good data supporting BARO1_THST_SCALE calibration

## See Also
- [Throttle vs Current analysis](../topics/throttle_vs_current.md)
