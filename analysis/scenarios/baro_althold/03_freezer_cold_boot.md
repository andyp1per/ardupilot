# Baro AltHold: Freezer Cold Boot

## Purpose

Test altitude hold when the flight controller boots cold (e.g., stored in a freezer or
cold vehicle) and the barometer temperature rises rapidly during warmup. The baro pressure
reading drifts as the sensor warms, creating a Z-bias that the EKF must track.

## Real-World Procedure

1. Store drone in freezer at -20C for 1+ hour
2. Remove and immediately power on outdoors
3. Wait for GPS fix and arm
4. Take off to 15m in ALT_HOLD
5. Hover for 120s while board warms from -20C toward ambient
6. Monitor altitude for thermal drift
7. Land

## SITL Simulation

**Feasibility:** FULL - SITL thermal model simulates exponential warmup with configurable
start temperature, board offset, and time constant.

### Parameters

```
SIM_TEMP_START    = -20    # Start temperature -20C (freezer)
SIM_TEMP_BRD_OFF  = 40     # Board runs 40C above atmospheric
SIM_TEMP_TCONST   = 60     # 60s warmup time constant
SIM_TEMP_BARO_FAC = 2.0    # Temperature sensitivity factor
```

### Temperature Model

The SITL baro uses: `T(t) = T_sensor - (T_sensor - T0) * exp(-t/tconst)`

With these params:
- T0 = -20C (start)
- T_sensor = ambient + 40C offset
- Time constant = 60s
- Significant drift for first ~3 minutes (3*tau)

### Test Sequence

1. Set temperature simulation parameters
2. Arm in ALT_HOLD mode (early, while still cold)
3. Take off to 15m
4. Hover for 120s
5. Check altitude stays within +/-3m despite thermal drift
6. Verify EKF tracks the drift without failsafe
7. Land and disarm

## Pass/Fail Criteria

| Metric | Pass | Fail |
|--------|------|------|
| Alt hold drift (120s) | < 3m | > 5m |
| Height innovation ratio (SH) | < 1.0 | > 1.5 |
| No EKF failsafe | Yes | No |
| EKF Z-bias tracks trend | Monotonic correction | Erratic |

## Key Log Signals

- `BARO.Temp` - temperature rising from -20C
- `BARO.Alt` - raw baro altitude (expect drift as temp changes)
- `CTUN.Alt` vs `CTUN.DAlt` - actual vs desired altitude
- `NKF1.PD` - EKF position down (should compensate for baro drift)
- `NKF4.SH` - height innovation test ratio

## Related Logs

- Any cold-start flight log with visible BARO.Temp ramp
