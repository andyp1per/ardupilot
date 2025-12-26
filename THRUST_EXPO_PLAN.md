# **Project Plan: Dynamic Thrust Linearization Lua Script**

## **Current Session Context (for continuation)**

### Test Environment
- **Simulator:** RealFlight with FlightAxis interface to SITL
- **Test Aircraft:** Very overpowered - MOT_THST_HOVER = 0.12 (12% throttle to hover)
- **Branch:** `pr-lua-thst-expo`

### Latest Status (2024-12-26)
**Two-phase RealFlight autotest working:**
1. **Phase 1: Spin-min calibration (TLIN_MODE=2)** - PASSED
   - Found MOT_SPIN_MIN=0.04 (was 0.16)
   - 350 samples collected via descent/climb velocity pairs
   - Parameter automatically set

2. **Phase 2: Hover test for expo (TLIN_MODE=0)** - PASSED
   - 520 samples across 8 bins
   - Result: Expo=0.00 (linear model fits well for this aircraft)
   - Also found SpinMin=0.07 during hover maneuvers

**Key insight:** For overpowered aircraft, the spin-min calibration is more important than expo. With only 12% throttle to hover, most flight occurs in the 0-20% throttle range where linearity matters less than having the correct spin-min.

### Previous Problem (Solved)
The test aircraft had MOT_SPIN_MIN=0.16 but hover throttle was only 0.12, meaning the parameter was incorrectly set higher than the hover point. This made expo measurement difficult because:
- All data was compressed into a very narrow throttle range
- Only 2 bins would fill during standard testing
- Result was always Expo=0.00

**Solution:** Added dedicated spin-min calibration mode (TLIN_MODE=2) that runs first to find the correct MOT_SPIN_MIN value.

### Key Learnings
1. **Lua VM timeout is real** - Complex operations must be chunked (50 samples for rebin, 3 iterations for solve)
2. **Variable ordering matters in Lua** - All global state must be defined before any function that references it
3. **Flight mode transitions** - After spin-min calibration switches to LOITER, must explicitly switch back to GUIDED for phase 2
4. **Descent maneuvers need altitude margin** - Initial 8m takeoff was insufficient; now using 18-22m
5. **Descent/climb pairs maintain altitude** - Better than sequential descents which accumulate altitude loss

### Test Modes (TLIN_MODE parameter)
- **0 = Hover** - vertical velocity steps
- **1 = Forward** - cruise with vertical oscillation
- **2 = Calibration** - direct throttle control (skipped if hover < 25%)
- **3 = Full** - auto LOITER→GUIDED, then all tests in sequence

### How to Run Tests in RealFlight/SITL

**Manual method (MAVProxy console):**
```bash
# Start SITL with RealFlight
cd rise255
../Tools/autotest/sim_vehicle.py -v ArduCopter -f flightaxis:127.0.0.1 --console -N -D -G --no-wsl2-network

# Once connected and armed in LOITER:
param set TLIN_ENABLE 1
param set TLIN_MODE 3      # Full test mode
param set TLIN_START 1     # Trigger test
```

**Automated method (test runner script):**
```bash
cd rise255
python3 run_tlin_test.py --no-sitl --mode 0  # Connect to existing SITL, hover mode
python3 run_tlin_test.py --mode 3            # Auto-start SITL, full test mode
```

### Test Runner Script (`rise255/run_tlin_test.py`)

Automates the RealFlight testing workflow:
- Connects to SITL (or auto-starts it)
- Disables failsafes for testing (FS_THR_ENABLE, FS_CRASH_CHECK)
- Fixes RealFlight battery drain (SIM_BATT_CAP_AH=100)
- Arms in LOITER, takes off, switches to GUIDED
- Enables TLIN script and triggers test via TLIN_START parameter
- Monitors TLIN_STATE for completion

**Key issues discovered:**
1. Must arm in LOITER then switch to GUIDED (can't arm in GUIDED)
2. RealFlight battery drains over time - need to override with SIM_BATT params
3. Script checks battery level - fails if <20%
4. Need to disable FS_THR_ENABLE and FS_CRASH_CHECK for reliable arming
5. SCR_DEBUG_OPTS=16 prevents scripting pre-arm block
6. Direct TCP connection to SITL port 5762 works better than MAVProxy UDP

**When RealFlight freezes/crashes:**
- SITL load drops to 0%, commands go unresponsive
- Need to reset aircraft in RealFlight (Ctrl+R or menu)
- May need to restart SITL completely

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
| `thrust_linearization_core.lua` | ✅ Complete | All 4 test modes + spin-min calibration |
| `thrust_linearization_menu.lua` | ✅ Complete | CRSF menu interface |
| `thrust_linearization_core.md` | ✅ Complete | User documentation |
| Autotest (hover mode) | ✅ Complete | `ScriptThrustLinearization` |
| Autotest (forward mode) | ✅ Complete | `ScriptThrustLinearizationForward` |
| Autotest (full mode) | ✅ Complete | `ScriptThrustLinearizationFull` |
| Autotest (RealFlight) | ✅ Complete | `RealFlightThrustLinearization` - two-phase test |
| MOT_SPIN_MIN calibration | ✅ Complete | TLIN_MODE=2, sets parameter automatically |
| **Overpowered aircraft** | ✅ Solved | Spin-min calibration runs first, then expo |

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

### Completed This Session
1. ✅ Added TLIN_MODE=2 for dedicated spin-min calibration
2. ✅ Chunked processing to avoid Lua VM timeout (50 samples rebin, 3 iterations solve)
3. ✅ Two-phase RealFlight autotest (spin-min → expo)
4. ✅ Gentle descent/climb pairs for low-throttle data collection
5. ✅ Flight mode transition handling between phases

### Future Improvements
1. ⬜ Clean up debug output before final PR
2. ⬜ Update documentation with new TLIN_MODE=2 option
3. ⬜ Test on aircraft with more typical hover throttle (40-60%)
4. ⬜ Consider combining spin-min and expo into single test sequence
