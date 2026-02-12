# logtd7 Analysis

## Metadata
- **Date**: 2025-02-12
- **Vehicle**: TD outdoor (user drone)
- **Branch**: SmallFastDrone-4.6-AltHold
- **Log file**: ./logtd7.bin

## Parameters
- Same tuning as logtd6 (PSC_POSZ_P=1.0, PSC_VELZ_P=5.0)
- `BARO1_THST_SCALE = -100`
- `EK3_IMU_MASK = 2`

## Flight Profile
- Two-phase flight:
  - **LOITER1**: ~27m altitude (baro-only, rangefinder beyond range)
  - **LOITER2**: ~6m altitude (with rangefinder, all Stat=4)

## Performance Metrics
- LOITER1 (27m, baro-only): Alt error std **23.1cm**
- LOITER2 (6m, with rangefinder): Alt error std **25.7cm**

## Sensor Status
- Rangefinder: NoData at 27m, Good (Stat=4) at 6m
- Baro affected by thermal drift

## Analysis

### Rangefinder Didn't Dramatically Improve Over Baro-Only

Surprising finding: at 6m with a working rangefinder, altitude hold (25.7cm std) was *slightly worse* than at 27m baro-only (23.1cm std).

Possible explanations:
- Rangefinder provides height, not velocity — velocity drift still present
- Different wind/turbulence conditions at 6m vs 27m
- Rangefinder noise at near-max range

### Log Truncation
- Log was truncated before disarm
- Hover bias learning was NOT saved (no disarm event)

### BARO_THST_SCALE Findings
- Thrust→Baro correlation at 0-3m: -0.165
- Current→Baro correlation at 0-3m: -0.213
- Winner: Current (one of two logs where current beats throttle at low alt)

## Recommendations
- Investigate why rangefinder didn't improve altitude hold
- Ensure clean disarm to save hover bias learning
- Consider that at 6m, rangefinder may be near noise floor for this sensor

## See Also
- [logtd5](logtd5.md) — baseline
- [logtd6](logtd6.md) — improved tuning
- [Baro thermal drift](../topics/baro_thermal_drift.md)
- [Throttle vs Current analysis](../topics/throttle_vs_current.md)
