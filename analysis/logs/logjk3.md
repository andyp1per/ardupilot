# logjk3 Analysis

## Metadata
- **Date**: 2025-01-28
- **Vehicle**: SFD indoor (small fast drone, MambaH743v4)
- **Branch**: SmallFastDrone-4.6-AltHold
- **Log file**: ./logjk3.bin

## Parameters
- Optical flow navigation (EK3_SRC1_VELXY=5)
- No Z velocity source (EK3_SRC1_VELZ=0)

## Flight Profile
- Indoor flight, partial success

## Performance Metrics
- Baro motor-on noise std: **2.71m** (8.0x motors-off ratio)
- Highest absolute baro noise of all logjk logs

## Sensor Status
- Optical flow active
- Rangefinder present

## Analysis

### BARO_THST_SCALE Findings
- Thrust→Baro correlation at 0-3m: -0.506
- Current→Baro correlation at 0-3m: -0.208
- Thrust-Current correlation: 0.432 (low — unusual)
- Estimated THST_SCALE: -280 Pa (lower than logjk1/2)
- Raw baro error: 2.69m → After compensation: 2.24m (17% improvement)

### At 3-8m
- Thrust→Baro correlation: -0.775 (strong)
- Current→Baro correlation: -0.151 (very weak)

## Recommendations
- Baro noise is severe — consider hardware baro relocation
- THST_SCALE value inconsistent with logjk1/2 (may indicate different flight conditions)

## See Also
- [Throttle vs Current analysis](../topics/throttle_vs_current.md)
