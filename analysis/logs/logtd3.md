# logtd3 Analysis

## Metadata
- **Date**: 2025-01-30
- **Vehicle**: TD indoor (optical flow copter)
- **Branch**: SmallFastDrone-4.6-AltHold
- **Log file**: ./logtd3.bin

## Parameters
- `BARO1_THST_SCALE = -147`
- `EK3_RNG_USE_HGT = -1`
- `RNGFND1_TYPE = 10` (MAVLink)
- `RNGFND1_MAX_CM = 700`

## Flight Profile
- Indoor flight, **CEILING HIT** — safety incident

## Performance Metrics
- N/A — flight aborted after ceiling impact

## Sensor Status
- **Rangefinder returning `Stat=1 (NoData)` throughout takeoff** — critical failure
- No valid height reference available

## Analysis

### Ceiling Hit Due to Baro Transients Without Rangefinder

**Timeline of incident:**

| Time | Baro | EKF Alt | Innovation | Throttle | Event |
|------|------|---------|------------|----------|-------|
| 13.08s | -3.1m | -0.69m | -0.5m | 0% | Pre-takeoff |
| 13.38s | -4.7m | -0.71m | -4.5m | 5% | Baro spike DOWN |
| 13.78s | **-10.5m** | -0.73m | -10.2m | 37% | Baro minimum |
| 13.98s | 0.0m | -0.68m | -0.5m | 41% | Baro recovers |
| 15.28s | +2.2m | -0.83m | +2.6m | 47% | Baro diverging UP |
| 15.88s | +3.5m | -1.21m | +4.1m | **100%** | **Full throttle** |
| 16.08s | -0.5m | -1.56m | -0.4m | 100% | **CEILING IMPACT** |
| 16.78s | +5.6m | -1.14m | — | 0% | Pilot switches to STABILIZE |

### Sequence of Events
1. Baro spiked negative (-10.5m) during motor spinup — EKF correctly rejected
2. Baro overcorrected positive (+3.5m) as motors sustained power
3. EKF altitude drifted to -1.21m while baro claimed +3.5m
4. Altitude controller saw 1.3m error → full throttle → ceiling impact

### Why Thrust Scaling Didn't Help During Transients
- `BARO1_THST_SCALE` provides **linear** compensation
- Actual pressure disturbance during spinup is **non-linear** with overshoot/undershoot
- At t=13.78s: throttle=37%, correction=-147×0.37=-54Pa (≈4.5m), but baro showed -10.5m
- The 6m difference cannot be compensated by linear scaling

### This Led to BARO1_THST_FILT Implementation
The step change from instantaneous thrust compensation motivated adding a low-pass filter on throttle before thrust compensation.

## Recommendations
1. **Rangefinder is critical for indoor flight** — baro alone is unreliable
2. Verify rangefinder returns valid data (`Stat=4`) before flight
3. `Stat=1` (NoData) means no height reference available
4. Use BARO1_THST_FILT to smooth throttle transients

## See Also
- [Baro thrust filter](../topics/baro_thrust_filter.md) — motivated by this incident
- [logtd1](logtd1.md) — same vehicle, working rangefinder
