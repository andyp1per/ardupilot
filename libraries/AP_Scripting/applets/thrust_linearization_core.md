# Thrust Linearization

This script calculates the optimal MOT_THST_EXPO parameter in-flight via
dynamic system identification. It measures body-frame vertical acceleration
versus motor output to characterize the thrust curve and find the expo value
that best linearizes motor response.

## Overview

The MOT_THST_EXPO parameter compensates for the non-linear relationship
between throttle input and motor thrust. An incorrect value causes:
- Altitude hold oscillations
- Inconsistent throttle response
- Suboptimal position controller performance

This script automates the tuning process by flying specific maneuvers and
analyzing the relationship between commanded throttle and measured acceleration.

## Test Modes

### Hover Test (Mode 0)

The default mode performs vertical velocity steps while maintaining horizontal
position:
1. Stabilize at current altitude
2. Climb at 1.5 m/s
3. Stabilize
4. Descend at 1.0 m/s
5. Stabilize
6. Fast climb at 2.0 m/s
7. Stabilize
8. Fast descend at 1.5 m/s
9. Stabilize

This sequence exercises the throttle range from hover to climb/descent,
collecting acceleration data at various throttle levels.

### Forward Flight Test (Mode 1)

For aircraft that primarily fly forward, this mode may provide more relevant
data:
1. Accelerate to cruise speed (TLIN_SPD)
2. Maintain speed while performing vertical oscillations
3. Turn around when approaching distance limit (TLIN_DIST)
4. Repeat until sufficient data collected

The vertical wave motion during cruise generates throttle variation while
maintaining realistic flight conditions.

## Parameters

### TLIN_ENABLE

Set to 1 to enable the script. When disabled, the script does not respond
to triggers or consume significant resources.

### TLIN_MODE

Test mode selection:
- 0: Hover test (default) - vertical steps while holding position
- 1: Forward flight test - cruise with vertical oscillation

### TLIN_SPD

Target forward speed for forward flight mode in m/s. Range: 2-20 m/s.
Default: 8 m/s.

### TLIN_DIST

Maximum distance from start position before aborting or turning around.
Range: 50-1000 m. Default: 300 m.

### TLIN_LEAN

Maximum lean angle allowed during test. If exceeded, the test aborts and
the aircraft switches to LOITER mode. Range: 10-60 degrees. Default: 45 deg.

### TLIN_START

Command trigger parameter. Set to 1 to start the test programmatically
(auto-resets to 0). This allows triggering from GCS or companion computer.

### TLIN_STATE

Read-only status parameter:
- 0: Idle - ready for new test
- 1: Running - test in progress
- 2: Complete - test finished successfully
- 3: Error - test aborted or failed

### TLIN_RESULT

Read-only result parameter containing the calculated MOT_THST_EXPO value
after successful test completion. Range: 0.0-1.0.

## Activation

The test can be triggered two ways:

### RC Switch

Assign an RC channel to Scripting1 (RCx_OPTION = 300). When the switch goes
high, the test starts. Setting the switch low during the test aborts it.

### Parameter Trigger

Set TLIN_START to 1 via GCS or MAVLink. The test starts immediately if
preconditions are met.

## Prerequisites

Before the test will start, the following conditions must be met:
- Aircraft must be armed
- Flight mode must be GUIDED
- Battery remaining > 20%
- GPS status >= 3D fix
- AHRS healthy

## Safety Features

The test includes multiple safety checks that will abort the test:
- Lean angle exceeds TLIN_LEAN
- Altitude drifts more than 5m below or 50m above start altitude
- Distance from start exceeds TLIN_DIST
- Vibration exceeds 30 m/s^2
- Pilot RC input detected (throttle stick > 10%)

When any safety limit is triggered, the aircraft automatically switches to
LOITER mode.

## Operation

1. Configure parameters:
   - Set TLIN_ENABLE = 1
   - Set RCx_OPTION = 300 for your preferred switch channel
   - Optionally adjust TLIN_MODE, TLIN_DIST, TLIN_LEAN

2. Arm and take off in a stable mode (LOITER or GUIDED)

3. Switch to GUIDED mode at a safe altitude (15-30m recommended)

4. Trigger the test via RC switch or TLIN_START parameter

5. Monitor GCS messages for progress:
   - "TLIN: Starting test"
   - "TLIN: Starting hover test" or "TLIN: Starting forward test"
   - Step progress messages
   - "TLIN: Result: Expo=X.XX"

6. After completion, the aircraft returns to LOITER mode

7. Review TLIN_RESULT and compare with current MOT_THST_EXPO

## Interpreting Results

The script reports both the calculated expo and current MOT_THST_EXPO:
```
TLIN: Result: Expo=0.65
TLIN: Current MOT_THST_EXPO=0.65
```

If the values differ significantly (>0.1), consider updating MOT_THST_EXPO:
```
param set MOT_THST_EXPO 0.65
```

Values typically range from 0.4 to 0.8 depending on motor/propeller
combination. Lower values indicate more linear motors; higher values
indicate more exponential thrust curves.

## Data Logging

Results are logged to the onboard dataflash log with the "TLIN" message type
containing:
- Calculated expo value
- Current MOT_THST_EXPO value
- Data sufficiency indicator

## Companion Script

The optional thrust_linearization_menu.lua script provides a CRSF (Crossfire)
menu interface for configuration and triggering. Both scripts communicate
via the TLIN_ parameters.

## Troubleshooting

**Test won't start:**
- Verify TLIN_ENABLE = 1
- Check you are armed and in GUIDED mode
- Check GCS messages for specific failure reason

**Test aborts immediately:**
- Check altitude is stable before triggering
- Verify GPS has good fix
- Reduce TLIN_LEAN if aircraft is tilting during wind gusts

**Result seems wrong:**
- Ensure calm conditions for best accuracy
- Try multiple runs and average results
- Verify vibration levels are acceptable

**No messages appear:**
- Confirm scripting is enabled (SCR_ENABLE = 1)
- Check script loaded without errors (look for lua errors on boot)
