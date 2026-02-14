# Optical Flow: Acro to Loiter with Good Light

## Purpose

Test mode transition from ACRO (no position control) to LOITER (flow-based position hold)
with good lighting conditions. This validates that the EKF can quickly establish position
control from optical flow when transitioning from a manual mode where flow data may have
been ignored or used differently.

## Real-World Procedure

1. Enable optical flow sensor and rangefinder
2. Good lighting (indoor with lights on, or outdoor)
3. Arm in ACRO mode
4. Fly manually for 10s (various attitudes and velocities)
5. Switch to LOITER
6. Verify vehicle stops and holds position within 5s
7. Hover for 30s
8. Land

## SITL Simulation

**Feasibility:** FULL - low flow noise simulates good lighting conditions.

### Parameters

```
# Optical flow (good conditions)
SIM_FLOW_ENABLE = 1
SIM_FLOW_RND    = 0.05    # Low noise = good lighting
FLOW_TYPE       = 10
RNGFND1_TYPE    = 1
RNGFND1_MIN_CM  = 0
RNGFND1_MAX_CM  = 4000
RNGFND1_PIN     = 0
RNGFND1_SCALING = 12.12

# EKF sources
EK3_SRC1_POSXY  = 0       # None
EK3_SRC1_VELXY  = 5       # Optical flow
EK3_SRC1_VELZ   = 0       # None

# GPS available for this test
```

### Test Sequence

1. Set flow parameters with low noise
2. Reboot SITL
3. Arm in LOITER, take off to 10m
4. Switch to ACRO
5. Apply RC inputs to create movement (pitch/roll commands for 5s)
6. Switch to LOITER
7. Wait for vehicle to stop (velocity < 0.3m/s)
8. Hover for 30s, measure drift
9. Land and disarm

## Pass/Fail Criteria

| Metric | Pass | Fail |
|--------|------|------|
| Transition settle time | < 5s to < 0.5m/s | > 10s |
| Position hold after settle | < 3m drift in 30s | > 5m drift |
| No EKF failsafe | Yes | No |
| Flow quality during transition | > 100 | < 50 |
| Altitude maintained | < 2m deviation | > 3m deviation |

## Key Log Signals

- `MODE` - mode transition timestamps
- `OF.qual` - flow quality through transition
- `NKF1.VN/VE` - velocity convergence after mode switch
- `NKF1.PN/PE` - position stability after transition
- `CTUN.Alt` - altitude during transition

## Related Logs

- Mode transition logs in well-lit environments
