# Optical Flow: Interior Throw

## Purpose

Test throw-mode launch indoors with no GPS, relying entirely on optical flow and
rangefinder for position estimation. This is the most demanding optical flow scenario -
the EKF must recover from throw dynamics using only flow and baro.

## Real-World Procedure

1. Enable optical flow sensor and rangefinder
2. No GPS (indoor environment)
3. Arm in THROW mode
4. Throw drone upward
5. Vehicle catches itself and transitions to LOITER
6. Hover for 30s using flow-only navigation
7. Land

## SITL Simulation

**Feasibility:** FULL - GPS disabled, flow-only EKF sources.

### Parameters

```
# Throw parameters
THROW_NEXTMODE  = 5       # LOITER after throw catch

SIM_SHOVE_Z     = -30     # Upward throw velocity
SIM_SHOVE_X     = -20     # Forward throw velocity

# GPS disabled
SIM_GPS_DISABLE = 1

# Optical flow
SIM_FLOW_ENABLE = 1
FLOW_TYPE       = 10
RNGFND1_TYPE    = 1
RNGFND1_MIN_CM  = 0
RNGFND1_MAX_CM  = 4000
RNGFND1_PIN     = 0
RNGFND1_SCALING = 12.12

# EKF sources (flow-only, no GPS)
EK3_SRC1_POSXY  = 0       # None
EK3_SRC1_VELXY  = 5       # Optical flow
EK3_SRC1_VELZ   = 0       # None

# Arming check relaxed for no GPS
ARMING_CHECK    = 786390
```

### Test Sequence

1. Set flow-only + throw parameters
2. Reboot SITL
3. Arm in THROW mode
4. Trigger SIM_SHOVE (throw)
5. Wait for mode transition to LOITER
6. Hover for 30s
7. Check position hold stability (flow-only)
8. Land and disarm

## Pass/Fail Criteria

| Metric | Pass | Fail |
|--------|------|------|
| Throw catch time | < 10s | > 15s |
| Mode transition | THROW -> LOITER | Stuck or crash |
| Position hold drift | < 3m in 30s | > 5m |
| No EKF failsafe | Yes | No |
| Flow quality after catch | > 100 | < 50 |
| No GPS required | True | GPS dependency |

## Key Log Signals

- `OF.flowX/flowY` - flow rates (primary velocity source)
- `OF.qual` - flow quality
- `NKF1.PN/PE` - position estimate (flow-derived)
- `RFND.Dist1` - rangefinder (critical for flow scaling)
- `NKF4.SP` - position innovation ratio

## Related Logs

- Indoor throw-launch logs without GPS
