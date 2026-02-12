# logjk2 Analysis

## Metadata
- **Date**: 2025-01-27
- **Vehicle**: SFD indoor (small fast drone, MambaH743v4)
- **Branch**: SmallFastDrone-4.6-AltHold
- **Log file**: ./logjk2.bin

## Parameters
- Optical flow navigation (EK3_SRC1_VELXY=5)
- No Z velocity source (EK3_SRC1_VELZ=0)

## Flight Profile
- Indoor flight, successful

## Performance Metrics
- Baro motor-on noise std: **1.69m** (1.9x motors-off ratio — better than most logjk flights)

## Sensor Status
- Optical flow active
- Rangefinder present

## Analysis

### BARO_THST_SCALE Findings
- Thrust→Baro correlation at 0-3m: -0.491
- Current→Baro correlation at 0-3m: -0.413
- Thrust-Current correlation: 0.952 (very high)
- Estimated THST_SCALE: -556 Pa (consistent with logjk1)
- Raw baro error: 1.72m → After compensation: 1.32m (23% improvement)

### At 3-8m
- Thrust→Baro correlation: -0.740 (strong)
- Current→Baro correlation: -0.534

## Recommendations
- BARO1_THST_SCALE = -550 consistent with logjk1 data
- Baro noise lower than typical — may indicate better baro isolation conditions

## See Also
- [Throttle vs Current analysis](../topics/throttle_vs_current.md)
- [Baro thrust filter](../topics/baro_thrust_filter.md)
