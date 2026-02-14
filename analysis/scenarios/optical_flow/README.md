# Optical Flow Scenarios

Optical flow navigation tests covering throw-launch, position hold, and mode transitions
under varying lighting conditions. These validate EKF3 optical flow fusion and fallback
behavior.

## Scenarios

1. [Exterior Throw](01_exterior_throw.md) - GPS+flow throw launch outdoors (FULL SITL)
2. [Interior Throw](02_interior_throw.md) - Flow-only throw launch indoors (FULL SITL)
3. [Hover Position Hold](03_hover_position_hold.md) - Flow loiter drift test (FULL SITL)
4. [Acro->Loiter with Light](04_acro_loiter_with_light.md) - Mode transition, good flow (FULL SITL)
5. [Acro->Loiter without Light](05_acro_loiter_without_light.md) - Mode transition, degraded flow (PARTIAL SITL)

## EKF Source Configuration

For flow-only (interior) scenarios:
```
EK3_SRC1_POSXY = 0   (none)
EK3_SRC1_VELXY = 5   (optical flow)
EK3_SRC1_VELZ  = 0   (none)
```

## Key Log Signals

- `OF.flowX/flowY` - raw optical flow rates
- `OF.bodyX/bodyY` - body-frame flow rates
- `OF.qual` - flow quality (0-255)
- `NKF1.PN/PE` - EKF position north/east
- `NKF4.SP` - position innovation test ratio
- `RFND.Dist1` - rangefinder distance
- `CTUN.Alt` - altitude tracking
