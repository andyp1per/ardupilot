# Optical Flow: Exterior Throw

## Purpose

Test throw-mode launch with optical flow enabled and GPS available. This validates that
the EKF can recover from the throw dynamics and fuse both GPS and optical flow for
position estimation after catch.

## Real-World Procedure

1. Enable optical flow sensor and rangefinder
2. GPS fix available (outdoor)
3. Arm in THROW mode
4. Throw drone upward with forward velocity
5. Vehicle catches itself and transitions to LOITER (or configured THROW_NEXTMODE)
6. Hover for 30s
7. RTL and land

## SITL Simulation

**Feasibility:** FULL - uses existing ThrowMode pattern with SIM_SHOVE plus flow sensor.

### Parameters

```
# Throw parameters
THROW_NEXTMODE  = 5       # LOITER after throw catch
SIM_SHOVE_Z     = -30     # Upward throw velocity
SIM_SHOVE_X     = -20     # Forward throw velocity

# Optical flow
SIM_FLOW_ENABLE = 1
FLOW_TYPE       = 10      # SITL flow sensor
RNGFND1_TYPE    = 1       # Analog rangefinder
RNGFND1_MIN_CM  = 0
RNGFND1_MAX_CM  = 4000
RNGFND1_PIN     = 0
RNGFND1_SCALING = 12.12

# EKF sources (GPS + flow)
EK3_SRC1_POSXY  = 1       # GPS
EK3_SRC1_VELXY  = 5       # Optical flow
EK3_SRC1_VELZ   = 0       # None (baro for height)
```

### Test Sequence

1. Set throw + flow parameters
2. Reboot SITL
3. Arm in THROW mode
4. Trigger SIM_SHOVE (throw)
5. Wait for mode transition to LOITER
6. Hover for 30s
7. Check position hold stability
8. RTL and disarm

## Pass/Fail Criteria

| Metric | Pass | Fail |
|--------|------|------|
| Throw catch time | < 10s | > 15s |
| Mode transition | THROW -> LOITER | Stuck or crash |
| Position hold drift | < 5m in 30s | > 10m |
| No EKF failsafe | Yes | No |
| Flow quality after catch | > 100 | < 50 |

## Key Log Signals

- `OF.flowX/flowY` - flow rates during and after throw
- `OF.qual` - flow quality (should recover after catch)
- `NKF1.PN/PE` - position estimate stability
- `RFND.Dist1` - rangefinder distance (invalid during throw, valid after)
- Mode transitions in `MODE` messages

## Related Logs

- Throw-launch flight logs with flow sensor enabled
