# Optical Flow: Acro to Loiter without Light

## Purpose

Test mode transition from ACRO to LOITER in near-darkness where optical flow quality is
degraded. This is the worst-case for flow-based position control - the EKF must handle
noisy or missing flow data gracefully and avoid position runaway.

## Real-World Procedure

1. Enable optical flow sensor and rangefinder
2. Near-dark environment (minimal ambient light)
3. Arm in ACRO mode
4. Fly manually for 10s
5. Switch to LOITER
6. Observe whether vehicle achieves position hold or drifts
7. Be ready to switch back to ACRO if position control fails
8. Land

## SITL Simulation

**Feasibility:** PARTIAL - high flow noise approximates degraded flow quality but isn't a
true lighting model. Real darkness causes flow quality to drop to 0; SITL noise keeps
quality nonzero but inaccurate.

### Parameters

```
# Optical flow (poor conditions - high noise)
SIM_FLOW_ENABLE = 1
SIM_FLOW_RND    = 2.0     # High noise = near-darkness approximation
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
```

### Limitations

- `SIM_FLOW_RND` adds Gaussian noise to flow; real darkness is systematic (quality drops)
- SITL doesn't model flow quality degradation from low light
- Test is conservative - real darkness may be harder

### Test Sequence

1. Set flow parameters with high noise
2. Reboot SITL
3. Arm in LOITER, take off to 10m
4. Switch to ACRO
5. Apply RC inputs for 5s
6. Switch to LOITER
7. Wait up to 10s for vehicle to settle
8. Check if position hold is achieved (relaxed criteria)
9. Land and disarm

## Pass/Fail Criteria

| Metric | Pass | Fail |
|--------|------|------|
| No crash | Yes | No |
| No EKF failsafe | Yes | No |
| Position drift rate | < 2m/s | Uncontrolled runaway |
| Vehicle controllable | Yes | No |
| Altitude maintained | < 5m deviation | Altitude runaway |

## Key Log Signals

- `OF.qual` - flow quality (expect degraded)
- `OF.flowX/flowY` - flow rates (expect noisy)
- `NKF4.SP` - position innovation ratio (expect elevated)
- `NKF1.VN/VE` - velocity estimate noise
- `MODE` - mode transition timing

## Related Logs

- Low-light or night flight logs with flow sensor
