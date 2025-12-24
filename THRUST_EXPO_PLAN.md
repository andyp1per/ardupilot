# **Project Plan: Dynamic Thrust Linearization Lua Script**

## **Current Session Context (for continuation)**

### Test Environment
- **Simulator:** RealFlight with FlightAxis interface to SITL
- **Test Aircraft:** Very overpowered - MOT_THST_HOVER = 0.12 (12% throttle to hover)
- **Branch:** `pr-lua-thst-expo`

### Current Problem
The test aircraft is extremely overpowered, making expo measurement difficult:
- Hover throttle is only 2-3%
- Even aggressive vertical maneuvers (15 m/s) barely change throttle (0.02-0.05 range)
- All data ends up in 1-2 bins, insufficient for expo curve fitting
- Result is always Expo=0.00

### What We've Tried
1. **Velocity-based hover test** - throttle doesn't vary enough on overpowered aircraft
2. **Direct throttle calibration** (`set_target_rate_and_throttle`) - causes rapid altitude changes, triggers safety limits
3. **Aggressive blips** - short duration up/down commands - still not enough variation
4. **Asymmetric maneuvers** - longer descents with hard brakes - helped slightly but altitude control issues

### Key Insight
On overpowered aircraft:
- **Climbing** uses very low throttle (gravity helps brake)
- **Descending** uses very low throttle (gravity accelerates)
- **Braking from descent** (commanding climb) uses highest throttle
- The position controller manages throttle so tightly that we don't see much variation

### Test Modes (TLIN_MODE parameter)
- **0 = Hover** - vertical velocity steps
- **1 = Forward** - cruise with vertical oscillation
- **2 = Calibration** - direct throttle control (skipped if hover < 25%)
- **3 = Full** - auto LOITER→GUIDED, then all tests in sequence

### How to Run Tests in RealFlight/SITL

```bash
# Start SITL with RealFlight
sim_vehicle.py -v ArduCopter -f flightaxis:192.168.x.x --console --map

# Once connected and armed in LOITER:
# Set parameters
param set TLIN_ENABLE 1
param set TLIN_MODE 3      # Full test mode
param set RC9_OPTION 300   # Scripting1 aux function

# Trigger test by setting RC9 high
rc 9 2000

# Watch output for:
# - "TLIN: Switching to GUIDED"
# - "TLIN: Starting test"
# - Step progress messages
# - "TLIN: Result: Expo=X.XX"
# - "TLIN: SpinMin=X.XX"

# Abort test
rc 9 1000
```

### Current Script Behavior

**File:** `libraries/AP_Scripting/applets/thrust_linearization_core.lua`

**Data Collection:**
- 10 bins for expo calculation, range from 1% to (hover * 2.5) throttle
- 20 spin bins for MOT_SPIN_MIN analysis (0-20% throttle range)
- `motors:get_throttle()` returns actual motor output
- `ins:get_accel(0).z` returns body-frame Z acceleration

**Binning Logic:**
```lua
local function get_throttle_range()
    local hover = MOT_THST_HOVER:get() or 0.5
    local min_thr = 0.01  -- Accept data from 1%
    local max_thr = math.min(hover * 2.5, 1.0)
    return min_thr, max_thr
end
```

**Safety Limits:**
- Altitude: -15m to +25m relative to start
- Lean angle: TLIN_LEAN parameter (default 45 deg)
- Distance: TLIN_DIST parameter
- Vibration: 30 m/s^2

### Ideas to Try Next

1. **Even more aggressive descent/brake cycles** - the brake phase is where we see highest throttle
2. **Sustained high-throttle climb** - command continuous climb to push throttle higher
3. **Different bin calculation** - maybe normalize bins relative to hover rather than absolute throttle
4. **Accept limited data** - for overpowered aircraft, maybe report "unable to measure" rather than Expo=0.00
5. **Forward flight mode** - cruise with oscillation might generate more throttle variation

### Commits on Branch (latest first)
```
2382011985 AP_Scripting: thrust linearization skip calibration for overpowered
dd9c6df3ad AP_Scripting: thrust linearization reduce calibration steps
5de329876c Tools: Add autotest for thrust linearization Full mode
7541360ba0 AP_Scripting: thrust linearization add Full test mode
cb63c2c241 AP_Scripting: thrust linearization longer descents for velocity
d1e2826885 AP_Scripting: thrust linearization balance altitude
04a501197e AP_Scripting: thrust linearization asymmetric blips for altitude
041c57c266 AP_Scripting: thrust linearization improve altitude safety
f3862c7a69 AP_Scripting: thrust linearization use aggressive blips
95cf951874 AP_Scripting: thrust linearization increase vertical speeds
253849cf4f AP_Scripting: thrust linearization lower min throttle threshold
aa38fe18e8 AP_Scripting: thrust linearization add MOT_SPIN_MIN analysis
1d1302720a AP_Scripting: thrust linearization use MOT_THST_HOVER for binning
```

---

## **Implementation Status**

| Component | Status | Notes |
|-----------|--------|-------|
| `thrust_linearization_core.lua` | ✅ Complete | All 4 test modes implemented |
| `thrust_linearization_menu.lua` | ✅ Complete | CRSF menu interface |
| `thrust_linearization_core.md` | ✅ Complete | User documentation |
| Autotest (hover mode) | ✅ Complete | `ScriptThrustLinearization` |
| Autotest (forward mode) | ✅ Complete | `ScriptThrustLinearizationForward` |
| Autotest (full mode) | ✅ Complete | `ScriptThrustLinearizationFull` |
| MOT_SPIN_MIN analysis | ✅ Complete | Suggests optimal spin_min value |
| **Overpowered aircraft tuning** | 🔄 In Progress | Need throttle variation for expo calc |

---

## **1. Objectives**

* **Goal:** Calculate the optimal MOT_THST_EXPO parameter in-flight.
* **Method:** Dynamic System Identification (Hover or Forward Flight).
* **Platform:** ArduCopter 4.x+ (Lua enabled).
* **Output:** A recommended parameter value displayed in GCS messages.

## **2. The Physics Model**

* **Assumption:** Body-Frame Vertical Acceleration (A_z,body) proportional to Physical Thrust (T).
* **The Curve:** T ≈ (1 - k) * u + k * u²
  * Where u is the input throttle (0.0 to 1.0).
  * Where k is the MOT_THST_EXPO.

## **3. Key Files**

- `libraries/AP_Scripting/applets/thrust_linearization_core.lua` - Main script
- `libraries/AP_Scripting/applets/thrust_linearization_core.md` - Documentation
- `libraries/AP_Scripting/applets/thrust_linearization_menu.lua` - CRSF menu
- `Tools/autotest/arducopter.py` - Contains 3 autotests

## **4. Parameters**

| Parameter | Description | Default |
|-----------|-------------|---------|
| TLIN_ENABLE | Enable script | 0 |
| TLIN_MODE | 0=Hover, 1=Forward, 2=Calibration, 3=Full | 0 |
| TLIN_SPD | Forward flight speed (m/s) | 8 |
| TLIN_DIST | Max distance from start (m) | 300 |
| TLIN_LEAN | Max lean angle (deg) | 45 |
| TLIN_START | Trigger (set to 1 to start) | 0 |
| TLIN_STATE | Status (0=Idle, 1=Running, 2=Complete, 3=Error) | 0 |
| TLIN_RESULT | Calculated expo result | 0 |

## **5. Current Hover Test Maneuver Sequence**

```lua
local HOVER_STEPS = {
    {vz = 0.0, duration_ms = 1000, name = "Hold"},
    {vz = -4.0, duration_ms = 400, name = "Up1"},
    {vz = 4.0, duration_ms = 500, name = "Dn1"},
    {vz = 0.0, duration_ms = 400, name = "Settle1"},
    {vz = -6.0, duration_ms = 300, name = "Up2"},
    {vz = 10.0, duration_ms = 800, name = "Dn2"},
    {vz = -8.0, duration_ms = 600, name = "Brake2"},
    -- ... more steps
}
```

Note: vz is NED, so negative = climb, positive = descend.

## **6. Remaining Work**

1. Get sufficient throttle variation on overpowered aircraft for expo calculation
2. Verify SpinMin analysis works correctly
3. Clean up debug output before final PR
4. Update documentation if maneuver sequence changes significantly
