# Baro AltHold: Open Space Baseline

## Purpose

Establish baseline altitude hold performance in ideal conditions: outdoors, no obstacles,
stable temperature, good GPS. This is the control scenario against which all other baro
scenarios are compared.

## Real-World Procedure

1. Power on in open field, wait for GPS 3D fix
2. Arm in STABILIZE, take off to 15m
3. Switch to ALT_HOLD, hover for 60s
4. Observe altitude drift on GCS
5. Land in LAND mode

## SITL Simulation

**Feasibility:** FULL - default SITL provides clean baro with no environmental disturbances.

### Parameters

```
# No special SIM parameters needed - default SITL environment
```

### Test Sequence

1. Arm in ALT_HOLD mode
2. Take off to 15m
3. Hover for 60s
4. Check altitude stays within +/-2m of target
5. Land and disarm

## Pass/Fail Criteria

| Metric | Pass | Fail |
|--------|------|------|
| Alt hold drift (60s) | < 2m | > 2m |
| Height innovation ratio (SH) | < 0.5 | > 1.0 |
| Throttle oscillation | < 10% pk-pk | > 20% pk-pk |

## Key Log Signals

- `CTUN.Alt` vs `CTUN.DAlt` - actual vs desired altitude
- `CTUN.ThO` - throttle output stability
- `NKF4.SH` - height innovation test ratio
- `BARO.Alt` - raw baro altitude

## Related Logs

- Any clean outdoor hover log serves as baseline
