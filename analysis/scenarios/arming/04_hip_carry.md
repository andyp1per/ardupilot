# Arming: Hip Carry

## Purpose

Test EKF behavior when the drone is armed and carried at hip level while walking. This
introduces horizontal and vertical motion that the EKF must track. Unlike backpack carry,
hip carry has more lateral movement and tilting.

## Real-World Procedure

1. Arm drone on level surface
2. Pick up and carry at hip while walking for 30s
3. Place back on ground
4. Take off to 10m in ALT_HOLD
5. Hover for 30s
6. Land

## SITL Simulation

**Feasibility:** PARTIAL - `SIM_SHOVE` provides combined horizontal and vertical impulse.

### Parameters

```
SIM_SHOVE_X    = 1       # Forward velocity (walking speed ~1m/s)
SIM_SHOVE_Y    = 0.5     # Lateral sway
SIM_SHOVE_TIME = 3000    # 3s shove duration
```

### Limitations

- One-shot motion only, not periodic walking gait
- No tilt simulation from carry angle
- Real hip carry has complex 3-axis motion profile

### Test Sequence

1. Arm in ALT_HOLD
2. Trigger SIM_SHOVE with horizontal + lateral components
3. Wait for shove to complete
4. Take off to 10m
5. Hover for 30s
6. Check altitude and position stability
7. Land and disarm

## Pass/Fail Criteria

| Metric | Pass | Fail |
|--------|------|------|
| No EKF failsafe during carry | Yes | No |
| Position estimate recovers | Within 5m after settle | > 10m error |
| Alt hold after carry | < 3m drift | > 5m drift |
| Height innovation (SH) | < 1.0 | > 1.5 |

## Key Log Signals

- `IMU.AccX/AccY` - horizontal acceleration during carry
- `NKF1.PN/PE/PD` - EKF position estimate during carry
- `NKF4.SP/SH` - position and height innovation ratios

## Related Logs

- Field logs with pre-flight movement
