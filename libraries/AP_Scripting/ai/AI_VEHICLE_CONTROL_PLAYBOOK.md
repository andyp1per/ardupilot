# **ArduPilot Lua Scripting: Functional Guides**

Welcome to the functional guides for ArduPilot Lua scripting. This resource organizes the scripting API by the *task you want to accomplish*, helping you discover the right functions for your needs. For complete details on each function, including all parameters and return values, please refer to the full **API Reference**.

### **1\. Reading Vehicle State & Telemetry**

This guide covers the essential functions for accessing the vehicle's real-time data, such as its position, attitude, and sensor readings. This is the foundation for any script that needs to be aware of the vehicle's status.

* **Primary Objects:** ahrs, gps, baro, battery

| Function/Method | Description |
| :---- | :---- |
| **ahrs:get\_location()** | Retrieves the vehicle's best-estimate position as a Location object. |
| **ahrs:get\_velocity\_NED()** | Gets the vehicle's current velocity in the North-East-Down frame. |
| **ahrs:get\_roll\_rad()** | Returns the current roll angle in radians. |
| **ahrs:get\_pitch\_rad()** | Returns the current pitch angle in radians. |
| **ahrs:get\_yaw\_rad()** | Returns the current yaw angle in radians. |
| **ahrs:healthy()** | Checks if the main attitude and heading reference system is providing reliable data. |
| **gps:status(instance)** | Gets the fix status for a specific GPS instance (e.g., 3D Fix, RTK Fixed). |
| **gps:num\_sats(instance)** | Returns the number of satellites being tracked by a specific GPS. |
| **baro:get\_altitude()** | Gets the current barometric altitude in meters. |
| **battery:voltage(instance)** | Gets the voltage of a specific battery. |
| **battery:capacity\_remaining\_pct(instance)** | Gets the remaining capacity of a battery as a percentage. |

### **2\. Controlling Vehicle Movement**

This guide focuses on functions that actively control the vehicle's movement. These commands are only effective when the vehicle is in a mode that accepts external guidance, such as **Guided** or **Auto** (during a NAV\_SCRIPT\_TIME command). The underlying implementation for these commands is the ModeGuided controller, and its behavior can be fine-tuned using the GUID\_OPTIONS parameter.

#### **Movement Control Types**

There are three main types of movement control, each with different levels of abstraction:

* **Position Control:** The safest method. You command a target position or velocity, and the autopilot's internal controllers manage the attitude and rates required to get there. The autopilot will not allow the vehicle to become inverted.  
* **Angle Control:** More direct than position control. You command a target attitude (roll, pitch, yaw), and the autopilot works to maintain it. The autopilot will still prevent the vehicle from becoming inverted.  
* **Rate Control:** The most direct method. You command specific rotational rates. This gives the script full control but also full responsibility. The script **must** monitor the vehicle's attitude and stop the rotation to achieve a desired outcome. This mode *can* allow the vehicle to go inverted if not managed carefully. If new rate commands are not received within a timeout period (set by GUID\_TIMEOUT), the vehicle will automatically stop rotating as a safety measure.

#### **Position Control**

* **Primary Object:** vehicle

| Function/Method | Description |
| :---- | :---- |
| **vehicle:set\_target\_location(Location\_ud)** | Commands the vehicle to fly to a specific geographic location. |
| **vehicle:set\_target\_velocity\_NED(Vector3f\_ud)** | Commands the vehicle to fly at a specific velocity in the North-East-Down frame. |
| **vehicle:set\_target\_pos\_NED(...)** | Commands the vehicle to fly to a position relative to the EKF origin, with optional yaw control. |

#### **Angle Control**

* **Primary Object:** vehicle

| Function/Method | Description |
| :---- | :---- |
| **vehicle:set\_target\_angle\_and\_climbrate(...)** | Sets a target roll, pitch, and yaw angle, along with a target climb rate. |

#### **Rate Control**

* **Primary Object:** vehicle

| Function/Method | Description |
| :---- | :---- |
| **vehicle:set\_target\_rate\_and\_throttle(...)** | Sets target roll, pitch, and yaw rates in degrees/second, along with a collective throttle value. |

#### **Combined Control (Advanced)**

These functions provide more granular control by allowing you to specify position, velocity, and acceleration targets simultaneously. This is useful for creating complex, smooth trajectories.

* **Primary Object:** vehicle

| Function/Method | Description |
| :---- | :---- |
| **vehicle:set\_target\_velaccel\_NED(...)** | Sets a target velocity and a feed-forward acceleration. Use this to command a smooth change in velocity. |
| **vehicle:set\_target\_posvel\_NED(...)** | Sets a target position and a target velocity. Use this to command the vehicle to arrive at a point with a specific speed and direction. |
| **vehicle:set\_target\_posvelaccel\_NED(...)** | Sets a target position, velocity, and acceleration. This provides the most complete control over the trajectory for advanced applications. |

#### **Advanced Control & Tuning**

The behavior of Guided mode can be modified with the GUID\_OPTIONS parameter. This is a bitmask, so you can combine options by adding their values together.

* **DoNotStabilizePositionXY (Bit 0, Value 1):** In velocity control mode, the autopilot will normally still try to hold its position and prevent drift. Setting this option disables that behavior, resulting in a "purer" velocity control where the vehicle may drift with wind.  
* **DoNotStabilizeVelocityXY (Bit 1, Value 2):** In acceleration control mode, the autopilot will normally try to hold velocity. Setting this option disables that behavior.  
* **WPNavUsedForPosControl (Bit 3, Value 8):** Changes the position controller from the simple pos\_control to the full wp\_nav controller. This allows for smoother pathing around corners but requires that set\_target\_location commands be sent at a lower rate.

### **3\. Managing Autonomous Missions**

This guide covers interaction with the onboard mission planner, allowing scripts to read, modify, and control the flow of pre-defined missions.

* **Primary Object:** mission

| Function/Method | Description |
| :---- | :---- |
| **mission:num\_commands()** | Returns the total number of commands in the current mission. |
| **mission:get\_item(index)** | Retrieves a single mission command at a specific index. |
| **mission:set\_item(index, item)** | Sets or updates a single mission command at a specific index. |
| **mission:set\_current\_cmd(index)** | Jumps the mission execution to the command at the specified index. |
| **mission:state()** | Returns the current status of the mission (e.g., Running, Complete, Stopped). |
| **mission:clear()** | Clears all commands from the current mission. |

### **4\. GCS Communication & Logging**

This guide explains how to send information back to the Ground Control Station (GCS) and how to write custom data to the vehicle's onboard logs for post-flight analysis.

* **Primary Objects:** gcs, logger

| Function/Method | Description |
| :---- | :---- |
| **gcs:send\_text(severity, text)** | Sends a text message to the GCS message console. |
| **gcs:send\_named\_float(name, value)** | Sends a named floating-point value for real-time graphing or display in the GCS. |
| **logger:write(name, labels, format, ...)** | Writes a custom entry to the onboard dataflash log. |

### **5\. Interacting with Parameters**

This guide covers how to read and write ArduPilot's configuration parameters from within a script.

* **Primary Objects:** param, Parameter()

| Function/Method | Description |
| :---- | :---- |
| **param:get(name)** | Gets a parameter's current value by its name. |
| **param:set(name, value)** | Sets a parameter's value for the current session only (does not persist). |
| **param:set\_and\_save(name, value)** | Sets a parameter's value and saves it to permanent storage. |
| **Parameter(name)** | Creates a more efficient helper object for a parameter that will be accessed frequently. |

### **6\. Controlling Peripherals & I/O**

This guide details how to control physical outputs like servos, relays, and GPIO pins, and how to read from inputs.

* **Primary Objects:** SRV\_Channels, relay, gpio

| Function/Method | Description |
| :---- | :---- |
| **SRV\_Channels:set\_output\_pwm\_chan(chan, pwm)** | Sets the raw PWM value for a specific servo output channel. |
| **SRV\_Channels:set\_output\_scaled(function, value)** | Sets a normalized output value (-1 to 1\) for a channel assigned a specific function. |
| **relay:on(instance)** | Turns a specific relay on. |
| **relay:toggle(instance)** | Toggles the state of a specific relay. |
| **gpio:write(pin\_number, value)** | Writes a high (1) or low (0) value to a GPIO pin configured as an output. |
| **gpio:read(pin\_number)** | Reads the state of a GPIO pin configured as an input. |

### **7\. Dealing with Radio Control (RC) Input**

This guide covers how to read the pilot's stick inputs from the RC transmitter and how to respond to auxiliary switch changes. This is essential for scripts that allow for manual override or mode changes based on pilot input.

Note: The specific channel numbers for the primary flight controls (roll, pitch, throttle, yaw) are mapped by the user. Your script should read the RCMAP\_ROLL, RCMAP\_PITCH, RCMAP\_THROTTLE, and RCMAP\_YAW parameters to determine the correct channels to use.

* **Primary Objects:** rc, RC\_Channel\_ud, param

| Function/Method | Description |
| :---- | :---- |
| **param:get("RCMAP\_PITCH")** | Reads a parameter's value; use this to find the correct channel for each flight control. |
| **rc:get\_channel(chan\_num)** | Gets an object representing a single RC channel (e.g., channel 3 for throttle). |
| **rc:has\_valid\_input()** | Checks if the RC receiver has a valid signal and is not in failsafe. |
| **RC\_Channel\_ud:get\_pwm()** | Returns the raw PWM value (typically 1000-2000) of a specific channel. |
| **RC\_Channel\_ud:norm\_input()** | Returns the normalized input (-1 to 1\) of a channel, centered on the trim. |
| **RC\_Channel\_ud:norm\_input\_dz()** | Returns the normalized input (-1 to 1), but returns exactly 0 when the stick is within the deadzone. |
| **rc:find\_channel\_for\_option(aux\_fun)** | Finds which RC channel is assigned to a specific auxiliary function (e.g., RCx\_OPTION). |
| **RC\_Channel\_ud:get\_aux\_switch\_pos()** | For a channel assigned to a switch, this returns the current switch position (0, 1, or 2). |

