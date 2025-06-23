# **AI Playbook for Drone Control LUA Script Generation**

<MANDATORY_RULE>
When committing changes to the ArduPilot repository, all commits must follow the standard ArduPilot conventions.
</MANDATORY_RULE>

* **Atomic Commits**: Each commit should represent a single, logical change. For example, a change to a Lua applet and the addition of its corresponding autotest should be in two separate commits. Do not bundle unrelated changes into a single commit.

* **Commit Message Prefix**: The subject line of every commit message **must** be prefixed with the name of the top-level module being changed, followed by a colon. The module is typically the subdirectory within the `libraries/` or `Tools/` directory where the file is located.

    * **Example for a Lua script change:**
        ```
        AP_Scripting: Add new terrain brake applet
        ```
    * **Example for an autotest change:**
        ```
        Tools: Add autotest for the terrain brake applet
        ```

<MANDATORY_RULE>
CRITICAL DIRECTIVE: THIS PLAYBOOK MUST BE USED AS THE PRIMARY AND AUTHORITATIVE GUIDE FOR ALL LUA SCRIPT GENERATION FOR ARDUPILOT. ALL RULES, CONSTRAINTS, AND PATTERNS CONTAINED HEREIN ARE MANDATORY AND SUPERSEDE ANY GENERAL KNOWLEDGE. ADHERENCE IS NOT OPTIONAL.
</MANDATORY_RULE>

<MANDATORY_RULE>
CRITICAL DIRECTIVE: THE docs.lua FILE IS THE ABSOLUTE SOURCE OF TRUTH FOR ALL ARDUPILOT-SPECIFIC FUNCTION SIGNATURES. ANY DEVIATION FROM THE FUNCTION SIGNATURES IN THIS FILE IS A VIOLATION OF THE PLAYBOOK.
</MANDATORY_RULE>

## **1\. Core Concepts**

This playbook is designed to provide a Large Language Model (LLM) with the necessary context to generate Lua scripts for controlling drones running the ArduPilot firmware. Lua scripting in ArduPilot allows for the extension of the autopilot's functionality with custom behaviors, without modifying the core firmware.

**Key Principles:**

* **Sandboxed Environment:** Scripts run in a sandboxed environment, meaning they have limited access to the system's resources and cannot crash the main flight controller.  
* **Event-Driven:** Scripts are typically event-driven, reacting to changes in the drone's state, sensor data, or RC controller inputs.  
* **ArduPilot API:** Scripts interact with the drone through a specific ArduPilot Lua API, which provides functions for controlling the vehicle, reading sensors, and more.

## **2\. Environment Setup**

For a Lua script to be used, the drone's flight controller must be configured to support scripting.

* **Enable Scripting:** In the ArduPilot parameters, SCR\_ENABLE must be set to 1\.  
* **Upload Scripts:** Scripts are uploaded to the APM/scripts directory on the flight controller's SD card.  
* **Reboot:** The flight controller must be rebooted after enabling scripting or uploading new scripts.

## **3\. ArduPilot LUA API Reference**

This section details the most common functions and libraries available in the ArduPilot Lua API.

### **3.1. Vehicle Control**

* **vehicle:get\_location()**: Returns a Location object with the current position of the drone.  
* **vehicle:set\_target\_location(Location)**: Commands the drone to fly to a specific Location.  
* **vehicle:set\_target\_velocity\_NED(Vector3f)**: Sets the drone's target velocity in a North-East-Down (NED) frame.  
* **vehicle:arm()**: Arms the drone.  
* **vehicle:disarm()**: Disarms the drone.  
* **vehicle:get\_mode()**: Returns the current flight mode of the drone.  
* **vehicle:set\_mode(mode)**: Sets the drone's flight mode (e.g., GUIDED, LOITER, RTL).  
* **vehicle:start\_takeoff(altitude)**: Initiates an auto-takeoff to the specified altitude.

**Example:**

\-- Fly to a GPS coordinate  
local target\_location \= Location()  
target\_location:lat(47.397742)  
target\_location:lng(8.545594)  
target\_location:alt(10) \-- 10 meters altitude  
vehicle:set\_target\_location(target\_location)

### **3.2. GPS**

* **gps:num\_sensors()**: Returns the number of connected GPS sensors.  
* **gps:status(instance)**: Returns the fix status of a specific GPS sensor.  
* **gps:location(instance)**: Returns a Location object with the position from a specific GPS sensor.  
* **gps:primary\_sensor()**: Returns the index of the primary GPS sensor.

### **3.3. Sensors**

* **rangefinder:num\_sensors()**: Returns the number of connected rangefinders.  
* **rangefinder:distance(instance)**: Returns the distance measured by a specific rangefinder.  
* **battery:num\_instances()**: Returns the number of connected batteries.  
* **battery:voltage(instance)**: Returns the voltage of a specific battery.  
* **battery:remaining\_capacity(instance)**: Returns the remaining capacity of a specific battery.

### **3.4. RC Channels**

* **rc:get\_channel(channel\_num)**: Returns the current PWM value of a specific RC channel.  
* **rc:get\_aux\_cached(aux\_channel)**: Returns the cached value of an auxiliary channel.

## **4\. Available Scripts**

The ArduPilot repository contains a wide variety of pre-written Lua scripts that can be used as-is or adapted for specific needs. These scripts are categorized into applets, drivers, examples, and tests.

### **4.1. Applets**

Applets are complete, ready-to-use scripts that require no user editing. They often provide high-level functionality and can be enabled through parameters. Each applet is accompanied by a markdown file (.md) that details its operation.

* BattEstimate  
* BatteryTag  
* Gimbal\_Camera\_Mode  
* Heli\_IM\_COL\_Tune  
* Heli\_idle\_control  
* Hexsoon LEDs  
* MissionSelector  
* ONVIF\_Camera\_Control  
* Param\_Controller  
* QuadPlane\_Low\_Alt\_FW\_mode\_prevention  
* RockBlock  
* Script\_Controller  
* SmartAudio  
* UniversalAutoLand  
* VTOL-quicktune  
* advance-wp  
* ahrs-set-origin  
* ahrs-source-extnav-optflow  
* camera-change-setting  
* copter-deadreckon-home  
* copter-slung-payload  
* copter\_terrain\_brake  
* crsf-calibrate  
* follow-target-send  
* forward\_flight\_motor\_shutdown  
* leds\_on\_a\_switch  
* motor\_failure\_test  
* mount-poi  
* net\_webserver  
* param-set  
* pelco\_d\_antennatracker  
* plane\_package\_place  
* plane\_precland  
* plane\_ship\_landing  
* repl  
* revert\_param  
* rover-quicktune  
* runcam\_on\_arm  
* video-stream-information  
* winch-control  
* x-quad-cg-allocation  
* **Aerobatics:**  
  * plane\_aerobatics  
  * sport\_aerobatics  
* **WebExamples:**  
  * test.lua  
  * test.shtml

### **4.2. Drivers**

Drivers provide support for specific hardware or protocols.

* BattMon\_ANX  
* EFI\_DLA  
* EFI\_HFE  
* EFI\_Halo6000  
* EFI\_NMEA2k  
* EFI\_SkyPower  
* Generator\_SVFFI  
* Hobbywing\_DataLink  
* INF\_Inject  
* LTE\_modem  
* UltraMotion  
* mount-djirs2-driver  
* mount-viewpro-driver  
* torqeedo-torqlink  
* **TOFSense-M:**  
  * TOFSENSE-M\_CAN  
  * TOFSENSE-M\_Serial

### **4.3. Examples**

Examples provide demonstrations of specific functionalities and can be used as a starting point for custom scripts.

* 6DoF\_roll\_pitch  
* AHRS\_switch  
* BQ40Z\_bms\_shutdown  
* CAN\_MiniCheetah\_drive  
* CAN\_logger  
* CAN\_read  
* CAN\_write  
* DroneCAN\_test  
* EFI\_tester  
* ESC\_slew\_rate  
* FenceBreach  
* FlexDebug  
* Flip\_Mode  
* LED\_matrix\_image  
* LED\_matrix\_text  
* LED\_poslight  
* LED\_roll  
* MAVLinkHL  
* MAVLink\_Commands  
* Mission\_test  
* MotorMatrix\_dotriaconta\_octaquad\_x  
* MotorMatrix\_fault\_tolerant\_hex  
* MotorMatrix\_hexadeca\_octa  
* MotorMatrix\_hexadeca\_octa\_cw\_x  
* MotorMatrix\_setup  
* Motor\_mixer\_dynamic\_setup  
* Motors\_6DoF  
* NMEA-decode  
* OOP\_example  
* RCIN\_test  
* RC\_override  
* RM3100\_self\_test  
* SN-GCJA5-particle-sensor  
* Safety\_States  
* Serial\_Dump  
* UART\_log  
* active\_source\_set  
* ahrs-print-angle-and-rates  
* ahrs-print-home-and-origin  
* ahrs-print-variances  
* ahrs-set-home-to-vehicle-location  
* ahrs-source-gps-optflow  
* ahrs-source-gps-wheelencoders  
* ahrs-source  
* analog\_input\_and\_GPIO  
* arming-check-batt-temp  
* arming-check-wp1-takeoff  
* aux\_cached  
* battery\_internal\_resistance\_check  
* benewakeH30\_can\_rangefinder  
* button\_test  
* camera-test  
* cell\_balance\_check  
* command\_int  
* copter-circle-speed  
* copter-fast-descent  
* copter-fly-vertical-circle  
* copter-nav-script-time  
* copter-posoffset  
* copter-wall-climber  
* copter\_alt\_offset  
* copter\_deploy  
* copter\_pingpong  
* copy\_userdata  
* crosstrack\_restore  
* crsf-menu  
* easter-egg  
* efi\_speed\_check  
* esc\_rpm\_scale  
* fault\_handling  
* frsky\_battery  
* frsky\_rpm  
* frsky\_wp  
* fw\_vtol\_failsafe  
* gen\_control  
* get\_notify\_RGB  
* glide\_into\_wind  
* gps\_synth  
* hello\_world  
* hello\_world\_display  
* i2c\_scan  
* jump\_tags\_calibrate\_agl  
* jump\_tags\_into\_wind\_landing  
* land\_hagl  
* lidar\_control  
* logging  
* mag\_heading  
* message\_interval  
* mission-edit-demo  
* mission-load  
* mission-save  
* mission\_spiral  
* motor\_lost  
* mount-test  
* net\_test  
* notch\_switch  
* opendog\_demo  
* orbit\_follow  
* param\_add  
* param\_get\_set\_test  
* plane-callout-alt  
* plane-doublets  
* plane-wind-failsafe  
* plane-wind-fs  
* plane\_guided\_follow  
* plane\_guided\_terrain  
* protected\_call  
* proximity\_test  
* qnh\_alt  
* quadruped  
* rangefinder\_quality\_test  
* rangefinder\_test  
* readstring\_test  
* relay\_control  
* rgb\_led\_test  
* rgb\_notify\_patterns  
* rover-MinFixType  
* rover-SaveTurns  
* rover-TerrainDetector  
* rover-motor-driver  
* rover-set-steering-and-throttle  
* rover-set-turn-rate  
* serial\_test  
* servo\_scan  
* servo\_set\_get  
* servo\_slew  
* set-angle  
* set-target-location  
* set-target-velocity  
* set\_CAMERA\_INFORMATION  
* set\_target\_posvel\_circle  
* ship\_vel\_match  
* sim\_arming\_pos  
* simple\_loop  
* sitl\_standby\_sim  
* smbus-check-cycles  
* sub\_test\_synthetic\_seafloor  
* temperature\_sensor  
* terrain\_warning  
* test\_get\_target\_location  
* test\_load  
* test\_script\_button  
* test\_update\_target\_location  
* trisonica-mini  
* wp\_test  
* wrap32\_test

## **5\. Code Generation Constraints**

The following constraints apply to all Lua code generation:

### **5.1. Lua Version and Libraries**

* **Lua Version:** All generated code must be compatible with Lua 5.3.  
* **API Source of Truth:** The docs.lua file is the **definitive source of truth** for all ArduPilot-specific function signatures. In cases of discrepancy between examples and this documentation, the docs.lua file takes precedence.  
* **Allowed Functions:** Functions are limited to:  
  * Standard Lua 5.3 language features.  
  * Functions documented in the provided docs.lua file.  
  * The standard io library for file operations.  
  * The require() function, for loading modules from the script's local APM/scripts directory.

### **5.2. Script Structure and Execution**

* **No Threads:** Scripts must not create or manage their own threads. The ArduPilot environment handles script execution in a single-threaded manner.  
* **Non-Blocking:** Scripts must not contain any blocking calls or long-running loops. Each execution of the script's main function should complete within a few milliseconds.  
* **Update Function Pattern:** The required structure depends on the script's purpose:  
  * **Applets (Continuous Tasks):** Must use a main update() function that performs its tasks and then reschedules itself to run again at a regular interval.  
  * **One-Shot Scripts:** Can execute logic sequentially and terminate without an update() loop.  
  * **Test Scripts:** Typically run a series of assert() checks and may optionally enter a simple loop to report a "tests passed" message.  
* **Error Handling:** A protected\_wrapper using pcall() is mandatory for all applets. It is also required for any example or test script where a runtime error is possible (e.g., interacting with hardware, complex calculations). For very simple, infallible example scripts (like hello\_world.lua), the wrapper can be omitted for clarity.  
  **Example protected\_wrapper:**  
  function protected\_wrapper()  
    local success, err \= pcall(update)  
    if not success then  
       gcs:send\_text(MAV\_SEVERITY.ERROR, "Internal Error: " .. err)  
       \-- Reschedule with a longer delay after an error  
       return protected\_wrapper, 1000  
    end  
    \-- Reschedule with the normal update interval  
    return protected\_wrapper, 100  
  end

### **5.3. Initial Condition Checks**

* **Use assert():** Scripts should use the assert() function at the beginning to validate essential preconditions. This ensures the script fails early and clearly if the environment is not correctly configured.  
  **Example assert() check:**  
  \-- Check that a required parameter is set  
  local my\_param \= assert(param:get('MYAPL\_ENABLE'), 'MYAPL\_ENABLE not set')

  \-- Check that the vehicle is the correct type  
  assert(vehicle:get\_frame\_class() \== 1, 'Script requires a Quad frame')

### **5.4. Default Applet Behavior**

* **RC Switch Activation:** By default, all applets must be activatable via an RC switch using a hardcoded Auxiliary Function.  
  1. The script should use a constant to define which scripting aux function to listen to (e.g., local SCRIPTING\_AUX\_FUNC \= 300).  
  2. Use rc:get\_aux\_cached(SCRIPTING\_AUX\_FUNC) to read the switch position (0=low, 1=middle, 2=high).  
  3. The documentation (.md file) must instruct the user to set their desired RCx\_OPTION to this number (e.g., "Set RC9\_OPTION to 300"). This removes a parameter from the script and simplifies user setup.

**Example 3-Position Switch Logic:**\-- Define a constant for the auxiliary function  
local SCRIPTING\_AUX\_FUNC \= 300 \-- Corresponds to "Scripting1"

\-- In the script's main logic  
function update()  
    \-- Directly use the hardcoded aux function number  
    local switch\_pos \= rc:get\_aux\_cached(SCRIPTING\_AUX\_FUNC)  
    if switch\_pos \== 0 then  
        \-- Handle LOW position  
    elseif switch\_pos \== 1 then  
        \-- Handle MIDDLE position  
    elseif switch\_pos \== 2 then  
        \-- Handle HIGH position  
    end  
    return update, 200 \-- reschedule  
end

### **5.5. Deliverable Format**

* **Assume Applet by Default:** Unless the user specifies otherwise, or the request is clearly for a simple example or test, the primary output should be a complete ArduPilot Applet.  
* **Applet Format:** An applet must include two files:  
  1. The Lua script file (.lua).  
  2. A corresponding Markdown documentation file (.md) that explains the applet's purpose, parameters, and usage.  
* **Example/Test Format:** If generating an example or test, follow the simpler structure observed in the repository (e.g., may omit headers, pcall wrappers, and documentation).  
* **Autotest Generation:** For every new applet, offer to generate a corresponding SITL autotest. The autotest must load the *actual* applet file and follow a two-stage parameter setup.  
  **Annotated Autotest Example:**  
  \# Import the test suite for the relevant vehicle  
  from rover import AutoTestRover

  class MyNewTest(AutoTestRover):  
      def MyNewAppletTest(self):  
          self.start\_subtest("Test MyNewApplet functionality")  
          \# The context manager handles installing the real script file  
          self.install\_applet\_script\_context("my\_new\_applet.lua")

          \# STAGE 1: Enable scripting and reboot.  
          \# This allows the script's parameters to be created.  
          self.set\_parameters({  
              "SCR\_ENABLE": 1,  
          })  
          self.reboot\_sitl()

          \# STAGE 2: Configure the script's parameters now that they exist.  
          \# A second reboot is required for the script to use these new values.  
          self.set\_parameters({  
              "MYAPL\_ENABLE": 1,  
              "RC9\_OPTION": 300, \# Scripting1  
          })  
          self.reboot\_sitl()

          \# The test can now proceed with the script fully configured  
          self.wait\_ready\_to\_arm()  
          self.arm\_vehicle()  
          self.context\_collect('STATUSTEXT')  
          self.set\_rc(9, 2000\)  
          self.wait\_statustext("MyNewApplet: State changed to HIGH", check\_context=True, timeout=5)  
          self.disarm\_vehicle()

### **5.6. Code Quality**

* **Header Comments (Applets Only):** Every applet script must start with a comment block that briefly describes its purpose and functionality. The style should be concise and consistent with other applets in the ArduPilot repository. This is not required for examples or tests.  
* **State Change Feedback:** Applets must provide brief, clear feedback via gcs:send\_text(severity, text) when significant state changes occur (e.g., activation, mode change, action completed). These messages are the primary mechanism for verification in autotests.  
* **Use Enums for Constants:** Avoid using hardcoded integers ("magic numbers") for values like modes, states, or options. Instead, define a local table at the start of the script to act as an enumeration.  
  **Example Enum for gcs:send\_text:**  
  \-- Enum for MAV\_SEVERITY levels. Using this is mandatory  
  \-- for gcs:send\_text() instead of hardcoded numbers.  
  local MAV\_SEVERITY \= {  
      EMERGENCY \= 0,  
      ALERT \= 1,  
      CRITICAL \= 2,  
      ERROR \= 3,  
      WARNING \= 4,  
      NOTICE \= 5,  
      INFO \= 6,  
      DEBUG \= 7  
  }

  \-- Usage:  
  gcs:send\_text(MAV\_SEVERITY.INFO, "Script initialized")

* **luacheck Compliance:** All generated Lua code must be free of errors and warnings when analyzed with the luacheck tool, using the standard ArduPilot configuration.

### **5.7. Parameter Creation**

* **Strict Syntax:** Parameter creation is a strict two-step process that must be followed exactly as documented in docs.lua.  
  1. **Declare Table:** Use param:add\_table(table\_key, prefix, num\_params) to declare the parameter group. The prefix must **NOT** have a trailing underscore.  
  2. **Add Parameters:** Iterate through a local table of parameter definitions and add each one using param:add\_param(table\_key, param\_num, name, default\_value).  
* **Naming Convention:**  
  * The prefix should be a short, uppercase string (e.g., MYAPL).  
  * The name in the parameter definition should be the suffix (e.g., ENABLE).  
  * The final parameter name seen by the user is PREFIX\_NAME (e.g., MYAPL\_ENABLE).  
  * The total length of this full name **must not exceed 16 characters**.  
  * The full name must be used when getting/setting the parameter in Lua and in autotests.  
* **Unique Table Key:** The table\_key must be an integer between 1 and 200 and must be unique across all existing scripts in the ArduPilot repository. Do not reuse any of the following keys: 7, 8, 9, 10, 11, 12, 14, 15, 16, 31, 36, 37, 39, 40, 41, 42, 43, 44, 45, 48, 49, 70, 71, 72, 73, 75, 76, 78, 79, 80, 81, 82, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 102, 104, 106, 109, 110, 111, 117, 136, 138, 139, 193\.  
  **Correct Parameter Creation Example:**  
  \-- This local table is for script organization only.  
  local parameter\_definitions \= {  
      { name \= "ENABLE", default \= 0 },  
      { name \= "VALUE", default \= 12.3 }  
  }  
  local PARAM\_TABLE\_KEY \= 101  
  local PARAM\_TABLE\_PREFIX \= "MYAPL" \-- Note: NO trailing underscore

  \-- Step 1: Declare the table with its key, prefix, and the number of parameters.  
  assert(param:add\_table(PARAM\_TABLE\_KEY, PARAM\_TABLE\_PREFIX, \#parameter\_definitions), "Could not add param table")

  \-- Step 2: Add each parameter individually using the correct signature.  
  for i, p\_def in ipairs(parameter\_definitions) do  
      assert(param:add\_param(PARAM\_TABLE\_KEY, i, p\_def.name, p\_def.default), "Could not add param "..p\_def.name)  
  end

  \-- Usage in code: use the full, concatenated name.  
  local my\_val \= param:get('MYAPL\_VALUE')

## **6\. Operational Constraints and Safety**

When generating Lua scripts, the following constraints must be strictly adhered to, to ensure the safety and stability of the drone.

* **Flight Mode Awareness:** Scripts must be aware of the vehicle's current flight mode. Actions should only be executed if the current flight mode is appropriate. For example, a script that controls the drone's position should only run in modes like GUIDED or AUTO.  
* **Arming Status:** Scripts must always check if the vehicle is armed before executing any commands that could result in motor activation or movement.  
* **Failsafe Integrity:** Scripts must not interfere with critical failsafe mechanisms, such as RC failsafe, battery failsafe, or geofence breaches.  
* **Resource Limits:** Scripts should be mindful of the flight controller's processing and memory limits. Avoid long-running, blocking operations and excessive memory allocation. Use short, efficient functions and schedule them to run periodically.  
* **Parameter Modification:** Scripts should exercise extreme caution when modifying parameters. Critical flight parameters should not be changed without a clear understanding of the consequences. If a script does modify parameters, it should restore them to their original values when the script is disabled or completes its task.  
* **User Control Priority:** The pilot's RC input should always have the highest priority. Scripts should be designed to relinquish control immediately if the pilot provides any input on the sticks or changes the flight mode.

## **7\. Best Practices for Prompting**

When requesting a Lua script, provide clear and concise instructions. Referencing the available scripts in the repository can help to improve the clarity and specificity of your requests.

**Prompting Template:**

"Create a Lua script that \[**action**\] when \[**trigger**\]. The script should \[**specific behavior**\]."

**Examples:**

* "Create a Lua script that **makes the drone fly in a 20-meter square pattern** when **the pilot flips RC channel 7 high**. The script should **return the drone to its original position after completing the pattern**."  
* "Create a Lua script that **logs the drone's altitude and battery voltage to a file every 5 seconds** when **the drone is armed**. The script should **stop logging when the drone is disarmed**."  
* "Create a Lua script that **activates a servo connected to output 10** when **the drone's altitude is above 50 meters**. The script should **deactivate the servo when the altitude is below 50 meters**."

## **8\. Example Use Cases**

Here are some common drone behaviors with their corresponding text prompts and expected Lua script outputs.

### **8.1. Simple Box Mission**

* **Prompt:** "Create a Lua script that makes the drone fly in a 15-meter box pattern when the pilot flips RC channel 8 to high."  
* **Expected Lua Script:**  
  local BOX\_SIZE \= 15 \-- meters  
  local original\_location \= nil

  function update()  
    if rc:get\_channel(8):get\_pwm() \> 1800 then  
      if original\_location \== nil then  
        original\_location \= vehicle:get\_location()  
        local target \= original\_location  
        target:offset(BOX\_SIZE, 0\)  
        vehicle:set\_target\_location(target)  
      end  
    else  
      original\_location \= nil  
    end  
    return update, 100  
  end

  return update()

  ## **9. Working with Aider and a Local ArduPilot Codebase**

  This section provides a guide for using a tool like `aider` to directly modify a local clone of the ArduPilot git repository. This workflow is ideal for making changes to existing scripts or autotests.

  ### **9.1. File Locations**

  To work with existing scripts, you need to know where they are located within the ArduPilot source tree. When creating new scripts, they should be placed in the appropriate subdirectory based on their function.

  * **Lua Scripts**: The source for all Lua scripts is located in the `ArduPilot/libraries/AP_Scripting/scripts/` directory. They are organized into subdirectories based on their type:
      * `applets/`
      * `drivers/`
      * `examples/`
  * **Autotests**: The Python-based SITL autotests for Lua scripts are located in `ArduPilot/Tools/autotest/`. The tests are vehicle-specific and should be added to the appropriate file (e.g., `arducopter.py`).

  ### **9.2. Aider Workflow**

  The general workflow involves launching `aider` from the root of the `ArduPilot` repository and adding the specific files you want to modify to the chat session.

  **Example Workflow:**

  Let's say you want to modify the `copter_terrain_brake.lua` applet and add a corresponding autotest.

  1.  **Start Aider**: Open your terminal and navigate to the root of your local `ardupilot` repository. Launch `aider` and add the relevant files:
      ```bash
      cd /path/to/your/ardupilot
      aider libraries/AP_Scripting/scripts/applets/copter_terrain_brake.lua Tools/autotest/arducopter.py
      ```
  2.  **Provide Instructions**: Once `aider` has loaded the files, you can provide instructions for the changes you want. `aider` will use the context of the files it has been given, along with the rules in this playbook, to generate the necessary modifications.
    
      **Example Prompt for `aider`:**
      > "In `copter_terrain_brake.lua`, add GCS messages for activation status. In `arducopter.py`, add a new autotest method to the `AutoTestCopter` class to verify this new functionality."
  3.  **Review and Apply**: `aider` will propose changes to the files. You can review the proposed changes and approve them to have them applied directly to your local files.

  By following this process, you can efficiently iterate on scripts and their tests within your local development environment.

  ### **9.3. Aider Output Format**

  <MANDATORY_RULE>
  When generating changes for existing files, the output must be a set of edits in a diff-style format, using Unix-style line endings (`\n`). Do not provide the entire file content unless the file is new. The edits should be clear and easy to apply.
  </MANDATORY_RULE>

### **8.2. Battery Failsafe**

* **Prompt:** "Create a Lua script that triggers a Return-to-Launch (RTL) when the battery voltage drops below 14.8 volts."  
* **Expected Lua Script:**  
  local LOW\_VOLTAGE \= 14.8

  function update()  
    if battery:voltage(0) \< LOW\_VOLTAGE then  
      vehicle:set\_mode("RTL")  
    end  
    return update, 1000  
  end

  return update()

### **8.3. Landing Gear Control**

* **Prompt:** "Create a Lua script that deploys the landing gear (servo on output 9\) when the drone's altitude is below 10 meters and retracts it above 10 meters."  
* **Expected Lua Script:**  
  local LANDING\_GEAR\_ALTITUDE \= 10 \-- meters  
  local SERVO\_OUTPUT \= 9  
  local DEPLOYED\_PWM \= 1900  
  local RETRACTED\_PWM \= 1100

  function update()  
    local current\_alt \= vehicle:get\_location():alt()  
    if current\_alt \< LANDING\_GEAR\_ALTITUDE then  
      servo:set\_output(SERVO\_OUTPUT, DEPLOYED\_PWM)  
    else  
      servo:set\_output(SERVO\_OUTPUT, RETRACTED\_PWM)  
    end  
    return update, 500  
  end

  return update()  
