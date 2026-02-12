# logjk5 Analysis

## Metadata
- **Date**: 2025-01-28
- **Vehicle**: SFD indoor (small fast drone, MambaH743v4)
- **Branch**: SmallFastDrone-4.6-AltHold
- **Log file**: ./logjk5.bin

## Parameters
- Optical flow navigation (EK3_SRC1_VELXY=5)
- No Z velocity source (EK3_SRC1_VELZ=0)

## Flight Profile
- Indoor flight, partial success

## Performance Metrics
- Baro motor-on noise std: **1.01m** (2.3x motors-off ratio — best of logjk series)

## Sensor Status
- Optical flow active
- Rangefinder present

## Analysis

### BARO_THST_SCALE Findings
- Thrust→Baro correlation at 0-3m: -0.788 (strong)
- Current→Baro correlation at 0-3m: -0.480
- Thrust-Current correlation: 0.521
- Estimated THST_SCALE: -230 Pa
- Raw baro error: 1.03m → After compensation: 0.92m (11% improvement)

### Notable
- Lowest baro noise of all logjk logs (1.01m vs 2.7m typical)
- Lower THST_SCALE estimate than logjk1/2 — may reflect different hover conditions

## Recommendations
- Best baro performance of the logjk series — investigate what was different (baro isolation? flight conditions?)

## See Also
- [Throttle vs Current analysis](../topics/throttle_vs_current.md)
