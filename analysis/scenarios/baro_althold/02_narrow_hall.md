# Baro AltHold: Narrow Hall

## Purpose

Test altitude hold in an indoor confined space where prop wash reflects off walls and
ceiling, creating pressure disturbances on the barometer. This is a worst-case scenario
for baro-based altitude hold on small quads in tight environments.

## Real-World Procedure

1. Power on in narrow hallway (< 3m wide, < 3m ceiling)
2. Arm in STABILIZE, take off to ~1.5m (half ceiling height)
3. Switch to ALT_HOLD, hover for 30s
4. Observe altitude oscillation and any bobbing
5. Attempt slow forward flight in ALT_HOLD
6. Land in LAND mode

## SITL Simulation

**Feasibility:** PARTIAL - SITL can inject baro noise and wash coefficient but cannot
simulate true confined aerodynamics (ground effect + ceiling effect + wall reflections).

### Parameters

```
SIM_BARO1_RND    = 0.3      # Extra baro noise (meters) to simulate pressure disturbances
SIM_BARO1_WCF_DN = -0.005   # Stronger downwash-on-baro coefficient
```

### Test Sequence

1. Set baro noise and wash parameters
2. Arm in ALT_HOLD mode
3. Take off to 5m (scaled from real 1.5m)
4. Hover for 30s
5. Check altitude stays within +/-3m of target (relaxed due to noise)
6. Land and disarm

## Pass/Fail Criteria

| Metric | Pass | Fail |
|--------|------|------|
| Alt hold drift (30s) | < 3m | > 5m |
| Height innovation ratio (SH) | < 1.0 | > 1.5 |
| No EKF failsafe | Yes | No |
| Throttle oscillation | < 20% pk-pk | > 30% pk-pk |

## Key Log Signals

- `CTUN.Alt` vs `CTUN.DAlt` - actual vs desired altitude
- `CTUN.ThO` - throttle output (expect higher variance)
- `NKF4.SH` - height innovation test ratio
- `BARO.Alt` - raw baro altitude (expect noisy)

## Limitations

- SITL baro noise is Gaussian; real confined-space pressure is correlated with throttle
- No ceiling effect simulation in SITL
- Results are conservative - real-world may be worse

## Related Logs

- Indoor hover logs with visible baro oscillation
