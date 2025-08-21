# **Flip On Switch**

This applet allows a drone to perform continuous flips while an RC switch is held in the high position. It utilizes the vehicle\_control.lua module for the flip maneuver logic.

## **How to Use**

1. Ensure vehicle\_control.lua is present in the APM/scripts/ directory on your autopilot's SD card.  
2. Copy the flip\_on\_switch.lua script to the APM/scripts/ directory.  
3. Enable Lua scripting by setting SCR\_ENABLE to 1 and reboot the autopilot.  
4. Configure the script's parameters.  
5. Set up an RC switch to activate the script.

## **Parameters**

* FLIP\_ENABLE: Set to 1 to enable the script. Default is 1\.  
* FLIP\_AXIS: The axis for the flip. 1 for Roll, 2 for Pitch. Default is 1 (Roll).  
* FLIP\_RATE: The rotation rate for the flip in degrees per second. Default is 720\.  
* FLIP\_DURATION: The duration of a single 360-degree flip in seconds. Default is 1.0.  
* FLIP\_THROTTLE: The throttle level to use during the flip (0-1). Default is 0.5.

## **RC Switch Setup**

To activate the script, you need to assign an RC switch to the "Scripting1" auxiliary function.

* Set an RCx\_OPTION parameter to 300, where x is the RC channel number of your desired 3-position switch.  
  * **LOW/MIDDLE**: The script is inactive. The drone will stop flipping.  
  * **HIGH**: The drone will continuously flip until the switch is moved.

## **Operation**

The vehicle must be in a VTOL mode (like Loiter or Guided) and flying for the script to start. When the assigned RC switch is moved to the high position, the script will take control and initiate the flip sequence. The drone will continue to flip as long as the switch is held high. Releasing the switch will stop the flips and the drone will recover to a level attitude. The script is designed to maintain the drone's altitude during the maneuver.