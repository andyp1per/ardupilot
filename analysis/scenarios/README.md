# Test Scenarios

Real-world test scenarios for validating EKF3 altitude hold changes before field testing.
Each scenario includes a description, parameter settings, pass/fail criteria, and SITL
autotest feasibility.

## Categories

- [Baro AltHold](baro_althold/) - Barometric altitude hold in various environments
- [Arming](arming/) - Arming and launch sequences with different prop sizes
- [Optical Flow](optical_flow/) - Optical flow navigation and throw-mode scenarios

## SITL Feasibility Summary

| Scenario | Feasibility | Mechanism |
|----------|------------|-----------|
| Baro: Open space | FULL | Default SITL, alt hold at 10-20m |
| Baro: Narrow hall | PARTIAL | `SIM_BARO1_RND`, `SIM_BARO1_WCF_DN` |
| Baro: Freezer cold boot | FULL | `SIM_TEMP_START=-20`, `SIM_TEMP_BRD_OFF=40` |
| Baro: Heat gun hot boot | FULL | `SIM_TEMP_START=50`, `SIM_TEMP_BRD_OFF=20` |
| Arm: Immediate launch | FULL | Arm + immediate takeoff per prop config |
| Arm: Delayed launch | FULL | Arm, wait 30-60s, then takeoff |
| Arm: Backpack carry | PARTIAL | `SIM_SHOVE_Z=-2, SIM_SHOVE_TIME=5000` |
| Arm: Hip carry | PARTIAL | `SIM_SHOVE_X=1, SIM_SHOVE_Y=0.5` |
| Arm: 100m infil run | PARTIAL | `SIM_SHOVE_X=3, SIM_SHOVE_TIME=10000` |
| Flow: Exterior throw | FULL | ThrowMode + `SIM_FLOW_ENABLE=1` |
| Flow: Interior throw | FULL | + `SIM_GPS_DISABLE=1`, flow-only EKF |
| Flow: Hover pos hold | FULL | Flow loiter, measure position drift |
| Flow: Acro->loiter (light) | FULL | `SIM_FLOW_RND=0.05` |
| Flow: Acro->loiter (no light) | PARTIAL | `SIM_FLOW_RND=2.0` |

## Running Autotests

```bash
Tools/autotest/autotest.py build.Copter test.Copter.ScenarioBaroAltHold_OpenSpace
Tools/autotest/autotest.py test.Copter.ScenarioBaroAltHold_ColdBoot
Tools/autotest/autotest.py test.Copter.ScenarioArm_ImmediateLaunch
Tools/autotest/autotest.py test.Copter.ScenarioFlow_ExteriorThrow
```
