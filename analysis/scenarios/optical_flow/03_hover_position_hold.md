# Optical Flow: Hover Position Hold

## Purpose

Test optical flow position hold accuracy during a sustained hover. This measures drift
rate and position hold precision when relying on flow sensor for velocity estimation.
Key metric for indoor navigation capability.

## Real-World Procedure

1. Enable optical flow sensor and rangefinder
2. Arm in LOITER mode (with flow as velocity source)
3. Take off to 2m (optimal rangefinder range)
4. Release sticks and hover for 60s
5. Measure total position drift
6. Land

## SITL Simulation

**Feasibility:** FULL - flow loiter with drift measurement.

### Parameters

```
# Optical flow
SIM_FLOW_ENABLE = 1
SIM_FLOW_RND    = 0.05    # Low flow noise (good conditions)
FLOW_TYPE       = 10
RNGFND1_TYPE    = 1
RNGFND1_MIN_CM  = 0
RNGFND1_MAX_CM  = 4000
RNGFND1_PIN     = 0
RNGFND1_SCALING = 12.12

# EKF sources (flow-based velocity)
EK3_SRC1_POSXY  = 0       # None (rely on flow integration)
EK3_SRC1_VELXY  = 5       # Optical flow
EK3_SRC1_VELZ   = 0       # None

# Disable GPS for pure flow test
SIM_GPS_DISABLE = 1
ARMING_CHECK    = 786390
```

### Test Sequence

1. Set flow parameters
2. Reboot SITL
3. Arm in LOITER
4. Take off to 5m (SITL-scaled from real 2m)
5. Release sticks (center), hover for 60s
6. Record starting and ending position
7. Calculate drift magnitude
8. Land and disarm

## Pass/Fail Criteria

| Metric | Pass | Fail |
|--------|------|------|
| Total position drift (60s) | < 3m | > 5m |
| Max velocity during hover | < 0.5m/s | > 1.0m/s |
| Altitude hold | < 1m drift | > 2m drift |
| Flow quality sustained | > 150 avg | < 100 avg |
| No EKF failsafe | Yes | No |

## Key Log Signals

- `NKF1.PN/PE` - position drift over time
- `NKF1.VN/VE` - velocity estimate (should be ~0 during hover)
- `OF.qual` - flow quality consistency
- `RFND.Dist1` - rangefinder altitude stability
- `CTUN.Alt` vs `CTUN.DAlt` - altitude hold

## Related Logs

- Indoor hover logs with flow sensor enabled
