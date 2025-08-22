# **Flip On Switch (Advanced)**

This applet allows a drone to perform complex flip maneuvers initiated by an RC switch. It utilizes the `vehicle_control.lua` module for the underlying flight control logic. The script supports two distinct operational modes for standard and spring-loaded switches, configurable via parameters.

## **How to Use**

1.  Ensure `vehicle_control.lua` is present in the `APM/scripts/modules` directory on your autopilot's SD card.
2.  Copy the `flip_on_a_switch.lua` script to the `APM/scripts/` directory.
3.  Enable Lua scripting by setting `SCR_ENABLE` to 1 and reboot the autopilot.
4.  Configure the script's parameters to match your vehicle and desired maneuver.
5.  Set up an RC switch to activate the script.

## **Parameters**

* **FLIP_ENABLE**: Set to 1 to enable the script. Default is 1.
* **FLIP_AXIS**: The axis for the flip. 1 for Roll, 2 for Pitch. Default is 1 (Roll).
* **FLIP_RATE**: The target rotation rate for the flip in degrees per second. Default is 720.
* **FLIP_THROTTLE**: The throttle level to use during the flip (0-1). A value of -1 will cut the throttle entirely. Default is 0.0.
* **FLIP_HOVER**: The vehicle's true hover throttle (0-1). This is critical for the script's physics calculations. Default is 0.125.
* **FLIP_SPRING**: Selects the control mode. 0 for standard switches, 1 for spring-loaded switches. Default is 0.
* **FLIP_FLICK_TO**: (Standard Mode Only) The time in seconds to differentiate a "flick" from a "hold". Any switch activation shorter than this is a flick. Default is 0.5.
* **FLIP_COMMIT_TO**: The timeout in seconds after the last switch input before the maneuver is committed and starts. Default is 0.75.

## **RC Switch Setup**

To activate the script, you need to assign an RC switch to the **"Scripting1"** auxiliary function.

* Set an `RCx_OPTION` parameter to **300**, where `x` is the RC channel number of your desired switch.

## **Operation**

The vehicle must be in a VTOL mode (like Loiter or Guided) and flying for the script to start. The behavior then depends on the `FLIP_SPRING` parameter.

### **Standard Mode (FLIP_SPRING = 0)**

This mode is for standard, non-momentary switches. It uses a commit-then-execute logic.

1.  **Define the Maneuver:**
    * **Flick-to-Count:** Quickly flick the switch high then low one or more times. The script will count the number of flicks.
    * **Hold-for-Duration:** Hold the switch high for a period longer than `FLIP_FLICK_TO`. The duration of the hold will define the duration of the flip.
2.  **Commit the Maneuver:** After defining the maneuver, leave the switch in the **HIGH** position. After the `FLIP_COMMIT_TO` timeout, the maneuver will begin and repeat until cancelled.
3.  **Cancel:** Move the switch to the **LOW** or **MIDDLE** position at any time to cancel the maneuver.

### **Spring-loaded Mode (FLIP_SPRING = 1)**

This mode is for momentary switches that naturally return to the low position. It uses an execute-after-timeout logic.

1.  **Define the Maneuver:**
    * Perform the same **Flick-to-Count** or **Hold-for-Duration** sequence as in Standard Mode. The switch will naturally return to the low position.
2.  **Automatic Commit:** The script will wait for the `FLIP_COMMIT_TO` timeout. If no new switch inputs are detected, the maneuver will start automatically.
3.  **Cancel:** While a maneuver is active, perform a single, quick flick (`low-high-low`) to cancel it.