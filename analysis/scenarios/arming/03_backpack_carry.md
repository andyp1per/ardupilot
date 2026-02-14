# Arming: Backpack Carry

## Purpose

Test EKF behavior when the drone is armed and then carried on an operator's back before
launch. The vertical bouncing motion from walking can confuse the EKF's altitude estimate
if zero-velocity fusion or baro integration doesn't handle the movement correctly.

## Real-World Procedure

1. Arm drone on level surface
2. Carefully place in backpack (props not spinning - armed but throttle zero)
3. Walk/jog for 30-60s
4. Remove from backpack, place on ground
5. Take off to 10m in ALT_HOLD
6. Hover for 30s
7. Land

## SITL Simulation

**Feasibility:** PARTIAL - `SIM_SHOVE` provides a one-shot velocity impulse, not continuous
carry motion. Can approximate the altitude disturbance but not the ongoing oscillation.

### Parameters

```
SIM_SHOVE_Z    = -2      # Upward velocity (simulates being lifted to back)
SIM_SHOVE_TIME = 5000    # 5s shove duration
```

### Limitations

- `SIM_SHOVE` auto-resets after duration - cannot simulate continuous walking bounce
- No periodic vertical oscillation model in SITL
- Real backpack carry has ~1Hz vertical bounce of +/-0.3m
- SITL test captures the "moved while armed" case but not the oscillation

### Test Sequence

1. Arm in ALT_HOLD
2. Trigger SIM_SHOVE to simulate lifting/carrying
3. Wait for shove to complete
4. Take off to 10m
5. Hover for 30s
6. Check altitude stability after the disturbance
7. Land and disarm

## Pass/Fail Criteria

| Metric | Pass | Fail |
|--------|------|------|
| No EKF failsafe during carry | Yes | No |
| Alt hold after carry | < 3m drift | > 5m drift |
| Height innovation (SH) | < 1.0 | > 1.5 |
| Altitude estimate at launch | Within 2m of actual | > 3m error |

## Key Log Signals

- `IMU.AccZ` - accelerometer during carry (should show disturbance)
- `NKF1.PD` - EKF position down during carry
- `BARO.Alt` - baro altitude (should track carry altitude change)
- `NKF4.SH` - height innovation during and after carry

## Related Logs

- Field logs where drone was moved after arming before takeoff
