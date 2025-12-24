--[[
  Thrust Linearization Core Script
  Calculates optimal MOT_THST_EXPO parameter in-flight via dynamic system identification.
  Uses body-frame vertical acceleration vs motor output to characterize thrust curve.

  Communication with menu script via TLIN_ parameters.
  Activation via RC switch (aux func 300) or TLIN_CMD_START parameter.
--]]

-- Parameter table configuration
local PARAM_TABLE_KEY = 74
local PARAM_TABLE_PREFIX = "TLIN_"

-- Flight modes (ArduCopter)
local MODE_GUIDED = 4
local MODE_LOITER = 5

-- MAVLink severity levels
local MAV_SEVERITY = {
    EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3,
    WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7
}

-- Scripting aux function for RC activation
local SCRIPTING_AUX_FUNC = 300

-- State machine states
local STATE = {
    IDLE = 0,
    RUNNING = 1,
    COMPLETE = 2,
    ERROR = 3
}

-- Test phases
local PHASE = {
    INIT = 0,
    STABILIZE = 1,
    COLLECT = 2,
    SOLVE = 3,
    DONE = 4
}

-- Bind to existing parameter
local function bind_param(name)
    local p = Parameter()
    if not p:init(name) then
        return nil
    end
    return p
end

-- Add new parameter to our table
local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value),
           string.format('could not add param %s', name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- Setup parameter table
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 10), 'could not add param table')

--[[
  // @Param: TLIN_ENABLE
  // @DisplayName: Thrust Linearization Enable
  // @Description: Enables the thrust linearization test script.
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local TLIN_ENABLE = bind_add_param("ENABLE", 1, 0)

--[[
  // @Param: TLIN_MODE
  // @DisplayName: Test Mode
  // @Description: Test mode selection. 0=Hover (vertical steps), 1=Forward flight.
  // @Values: 0:Hover,1:Forward
  // @User: Standard
--]]
local TLIN_MODE = bind_add_param("MODE", 2, 0)

--[[
  // @Param: TLIN_SPD
  // @DisplayName: Test Speed
  // @Description: Target forward speed for forward flight mode test.
  // @Units: m/s
  // @Range: 2 20
  // @User: Standard
--]]
local TLIN_SPD = bind_add_param("SPD", 3, 8)

--[[
  // @Param: TLIN_DIST
  // @DisplayName: Maximum Distance
  // @Description: Maximum distance from home before aborting test.
  // @Units: m
  // @Range: 50 1000
  // @User: Standard
--]]
local TLIN_DIST = bind_add_param("DIST", 4, 300)

--[[
  // @Param: TLIN_LEAN
  // @DisplayName: Maximum Lean Angle
  // @Description: Abort test if lean angle exceeds this value.
  // @Units: deg
  // @Range: 10 60
  // @User: Standard
--]]
local TLIN_LEAN = bind_add_param("LEAN", 5, 45)

--[[
  // @Param: TLIN_START
  // @DisplayName: Start Command
  // @Description: Set to 1 to start test (auto-resets to 0).
  // @Values: 0:Idle,1:Start
  // @User: Standard
--]]
local TLIN_START = bind_add_param("START", 6, 0)

--[[
  // @Param: TLIN_STATE
  // @DisplayName: Test State
  // @Description: Current test state (read-only). 0=Idle, 1=Running, 2=Complete, 3=Error.
  // @Values: 0:Idle,1:Running,2:Complete,3:Error
  // @User: Standard
--]]
local TLIN_STATE = bind_add_param("STATE", 7, 0)

--[[
  // @Param: TLIN_RESULT
  // @DisplayName: Result Expo
  // @Description: Calculated MOT_THST_EXPO value (read-only).
  // @Range: 0 1
  // @User: Standard
--]]
local TLIN_RESULT = bind_add_param("RESULT", 8, 0)

-- Bind to existing ArduPilot parameters
local MOT_THST_EXPO = bind_param("MOT_THST_EXPO")
local MOT_THST_HOVER = bind_param("MOT_THST_HOVER")
local MOT_SPIN_MIN = bind_param("MOT_SPIN_MIN")

-- Runtime state
local g_state = {
    phase = PHASE.INIT,
    start_time_ms = 0,
    start_alt_m = 0,
    start_loc = nil,
    last_switch_pos = 0,
    collect_start_ms = 0,
    hover_step = 0,
    hover_step_start_ms = 0,
    fwd_phase = 0,
    fwd_heading_rad = 0,
    turnaround_count = 0,
    sample_count = 0,
    last_diag_ms = 0,
    last_throttle = 0
}

-- Data collection bins (10 bins for expo calculation)
local NUM_BINS = 10
local g_bins = {}
for i = 1, NUM_BINS do
    g_bins[i] = {sum_accel = 0, sum_throttle = 0, count = 0}
end

-- Additional bins for spin_min analysis (20 bins covering 0-0.2 throttle range)
local NUM_SPIN_BINS = 20
local g_spin_bins = {}
for i = 1, NUM_SPIN_BINS do
    g_spin_bins[i] = {sum_accel = 0, sum_throttle = 0, count = 0}
end

-- Get throttle range based on MOT_THST_HOVER
local function get_throttle_range()
    local hover = MOT_THST_HOVER:get() or 0.5
    -- Range from 30% of hover to min(2.5x hover, 1.0)
    local min_thr = hover * 0.3
    local max_thr = math.min(hover * 2.5, 1.0)
    return min_thr, max_thr
end

-- Get throttle bin index (1-10) for given throttle value
local function get_bin_index(throttle)
    local min_thr, max_thr = get_throttle_range()
    if throttle < min_thr then return nil end
    if throttle > max_thr then throttle = max_thr end
    -- Map min_thr-max_thr to bins 1-10
    local range = max_thr - min_thr
    if range <= 0 then return 1 end
    local idx = math.floor((throttle - min_thr) / range * NUM_BINS) + 1
    if idx > NUM_BINS then idx = NUM_BINS end
    if idx < 1 then idx = 1 end
    return idx
end

-- Get spin bin index (1-20) for throttle values 0-0.2
local function get_spin_bin_index(throttle)
    if throttle < 0 or throttle > 0.2 then return nil end
    local idx = math.floor(throttle / 0.01) + 1
    if idx > NUM_SPIN_BINS then idx = NUM_SPIN_BINS end
    if idx < 1 then idx = 1 end
    return idx
end

-- Clear all data bins
local function clear_bins()
    for i = 1, NUM_BINS do
        g_bins[i].sum_accel = 0
        g_bins[i].sum_throttle = 0
        g_bins[i].count = 0
    end
    for i = 1, NUM_SPIN_BINS do
        g_spin_bins[i].sum_accel = 0
        g_spin_bins[i].sum_throttle = 0
        g_spin_bins[i].count = 0
    end
end

-- Add data point to appropriate bin
local function add_data_point(throttle, accel_z)
    g_state.last_throttle = throttle
    -- Add to expo bins (filtered by throttle range)
    local idx = get_bin_index(throttle)
    if idx then
        g_bins[idx].sum_accel = g_bins[idx].sum_accel + (-accel_z)
        g_bins[idx].sum_throttle = g_bins[idx].sum_throttle + throttle
        g_bins[idx].count = g_bins[idx].count + 1
        g_state.sample_count = g_state.sample_count + 1
    end
    -- Add to spin_min bins (always collect low throttle data)
    local spin_idx = get_spin_bin_index(throttle)
    if spin_idx then
        g_spin_bins[spin_idx].sum_accel = g_spin_bins[spin_idx].sum_accel + (-accel_z)
        g_spin_bins[spin_idx].sum_throttle = g_spin_bins[spin_idx].sum_throttle + throttle
        g_spin_bins[spin_idx].count = g_spin_bins[spin_idx].count + 1
    end
end

-- Check if we have enough data in bins
local function have_sufficient_data()
    local filled_bins = 0
    local total_samples = 0
    for i = 1, NUM_BINS do
        total_samples = total_samples + g_bins[i].count
        if g_bins[i].count >= 5 then
            filled_bins = filled_bins + 1
        end
    end
    gcs:send_text(MAV_SEVERITY.INFO, string.format("TLIN: Bins=%d samples=%d", filled_bins, total_samples))
    return filled_bins >= 3 or total_samples >= 50
end

-- Calculate linearity score for a given expo value
local function calc_linearity_score(expo)
    local total_error = 0
    local point_count = 0

    for i = 1, NUM_BINS do
        if g_bins[i].count > 0 then
            local avg_throttle = g_bins[i].sum_throttle / g_bins[i].count
            local avg_accel = g_bins[i].sum_accel / g_bins[i].count

            -- Expected linearized output: u_lin = (1-k)*u + k*u^2
            local u_lin = (1 - expo) * avg_throttle + expo * avg_throttle * avg_throttle

            -- In ideal case, accel should be proportional to linearized throttle
            -- Normalize by assuming hover at ~0.5 throttle gives ~9.8 m/s^2
            local expected_accel = u_lin * 19.6

            local err = avg_accel - expected_accel
            total_error = total_error + err * err
            point_count = point_count + 1
        end
    end

    if point_count < 3 then
        return 1e10
    end

    return total_error / point_count
end

-- Find optimal expo value
local function solve_expo()
    local best_expo = 0
    local best_score = 1e10

    -- Grid search from 0.0 to 1.0 in steps of 0.02
    for k = 0, 50 do
        local expo = k * 0.02
        local score = calc_linearity_score(expo)
        if score < best_score then
            best_score = score
            best_expo = expo
        end
    end

    -- Refine around best value
    local start_expo = math.max(0, best_expo - 0.02)
    local end_expo = math.min(1, best_expo + 0.02)
    for k = 0, 20 do
        local expo = start_expo + k * (end_expo - start_expo) / 20
        local score = calc_linearity_score(expo)
        if score < best_score then
            best_score = score
            best_expo = expo
        end
    end

    return math.floor(best_expo * 100 + 0.5) / 100
end

-- Analyze spin bins to estimate MOT_SPIN_MIN
-- Looks for transition from gravity-only (~9.8 m/s^2 down) to thrust
local function analyze_spin_min()
    -- Need at least a few data points
    local total_spin_samples = 0
    for i = 1, NUM_SPIN_BINS do
        total_spin_samples = total_spin_samples + g_spin_bins[i].count
    end
    if total_spin_samples < 10 then
        return nil  -- Not enough data
    end

    -- Gravity baseline: ~9.8 m/s^2 body-frame accel when no thrust
    local GRAVITY = 9.8
    local THRUST_THRESHOLD = 1.0  -- Accel must exceed gravity by this much

    -- Find first bin where average accel significantly exceeds gravity
    -- This indicates thrust is being generated
    for i = 1, NUM_SPIN_BINS do
        if g_spin_bins[i].count >= 3 then
            local avg_accel = g_spin_bins[i].sum_accel / g_spin_bins[i].count
            local avg_throttle = g_spin_bins[i].sum_throttle / g_spin_bins[i].count
            -- If accel > gravity + threshold, props are generating thrust
            if avg_accel > GRAVITY + THRUST_THRESHOLD then
                -- Return suggested spin_min (slightly below this throttle)
                local suggested = math.max(0, avg_throttle - 0.01)
                return math.floor(suggested * 100 + 0.5) / 100
            end
        end
    end

    -- If no clear transition found, check current MOT_SPIN_MIN
    local current_spin_min = MOT_SPIN_MIN:get() or 0.15
    return current_spin_min  -- No change suggested
end

-- Check pre-flight conditions
local function check_preconditions()
    if not arming:is_armed() then
        gcs:send_text(MAV_SEVERITY.WARNING, "TLIN: Must be armed")
        return false
    end

    if vehicle:get_mode() ~= MODE_GUIDED then
        gcs:send_text(MAV_SEVERITY.WARNING, "TLIN: Switch to GUIDED first")
        return false
    end

    local batt_pct = battery:capacity_remaining_pct(0)
    if batt_pct and batt_pct < 20 then
        gcs:send_text(MAV_SEVERITY.WARNING, "TLIN: Battery too low")
        return false
    end

    if gps:status(0) < 3 then
        gcs:send_text(MAV_SEVERITY.WARNING, "TLIN: GPS not healthy")
        return false
    end

    if not ahrs:healthy() then
        gcs:send_text(MAV_SEVERITY.WARNING, "TLIN: AHRS not healthy")
        return false
    end

    return true
end

-- Check safety limits during test
local function check_safety()
    -- Check lean angle
    local roll_rad = ahrs:get_roll_rad()
    local pitch_rad = ahrs:get_pitch_rad()
    if roll_rad and pitch_rad then
        local lean_deg = math.deg(math.sqrt(roll_rad * roll_rad + pitch_rad * pitch_rad))
        if lean_deg > TLIN_LEAN:get() then
            gcs:send_text(MAV_SEVERITY.WARNING, "TLIN: Lean angle exceeded")
            return false
        end
    end

    -- Check altitude limits
    local alt = baro:get_altitude()
    if alt then
        local rel_alt = alt - g_state.start_alt_m
        if rel_alt < -5 or rel_alt > 50 then
            gcs:send_text(MAV_SEVERITY.WARNING, "TLIN: Altitude limit exceeded")
            return false
        end
    end

    -- Check distance from start
    local pos = ahrs:get_location()
    if pos and g_state.start_loc then
        local dist = pos:get_distance(g_state.start_loc)
        if dist > TLIN_DIST:get() then
            gcs:send_text(MAV_SEVERITY.WARNING, "TLIN: Distance limit exceeded")
            return false
        end
    end

    -- Check vibration
    local vibe = ahrs:get_vibration()
    if vibe then
        local vibe_mag = math.sqrt(vibe:x() * vibe:x() + vibe:y() * vibe:y() + vibe:z() * vibe:z())
        if vibe_mag > 30 then
            gcs:send_text(MAV_SEVERITY.WARNING, "TLIN: Vibration too high")
            return false
        end
    end

    -- Check RC stick override (throttle channel)
    local thr_ch = rc:find_channel_for_option(0)
    if thr_ch then
        local norm = thr_ch:norm_input_dz()
        if norm and math.abs(norm) > 0.1 then
            gcs:send_text(MAV_SEVERITY.WARNING, "TLIN: RC override detected")
            return false
        end
    end

    return true
end

-- Abort test and return to loiter
local function abort_test(reason)
    gcs:send_text(MAV_SEVERITY.WARNING, "TLIN: Aborted - " .. reason)
    vehicle:set_mode(MODE_LOITER)
    TLIN_STATE:set(STATE.ERROR)
    g_state.phase = PHASE.DONE
end

-- Complete test successfully
local function complete_test(expo_result)
    gcs:send_text(MAV_SEVERITY.INFO, string.format("TLIN: Result: Expo=%.2f", expo_result))
    gcs:send_text(MAV_SEVERITY.INFO, string.format("TLIN: Current MOT_THST_EXPO=%.2f", MOT_THST_EXPO:get() or 0))

    -- Analyze spin_min data
    local spin_min_result = analyze_spin_min()
    local current_spin_min = MOT_SPIN_MIN:get() or 0.15
    if spin_min_result then
        gcs:send_text(MAV_SEVERITY.INFO, string.format("TLIN: SpinMin=%.2f (cur=%.2f)",
                      spin_min_result, current_spin_min))
    end

    TLIN_RESULT:set(expo_result)
    TLIN_STATE:set(STATE.COMPLETE)

    -- Log results
    logger:write("TLIN", "Expo,CurExpo,SpinMin,CurSpinMin", "ffff",
                 expo_result, MOT_THST_EXPO:get() or 0,
                 spin_min_result or 0, current_spin_min)

    vehicle:set_mode(MODE_LOITER)
    g_state.phase = PHASE.DONE
end

-- Hover test vertical step sequence
local HOVER_STEPS = {
    {vz = 0.0, duration_ms = 2000, name = "Hold"},
    {vz = -1.5, duration_ms = 3000, name = "Climb"},
    {vz = 0.0, duration_ms = 2000, name = "Hold"},
    {vz = 1.0, duration_ms = 3000, name = "Descend"},
    {vz = 0.0, duration_ms = 2000, name = "Hold"},
    {vz = -2.0, duration_ms = 2000, name = "FastClimb"},
    {vz = 0.0, duration_ms = 2000, name = "Hold"},
    {vz = 1.5, duration_ms = 2000, name = "FastDesc"},
    {vz = 0.0, duration_ms = 2000, name = "Hold"},
}

-- Run hover test mode
local function run_hover_test()
    local now_ms = millis():tofloat()

    if g_state.hover_step == 0 then
        g_state.hover_step = 1
        g_state.hover_step_start_ms = now_ms
        g_state.collect_start_ms = now_ms
        -- Show throttle range based on MOT_THST_HOVER
        local min_thr, max_thr = get_throttle_range()
        local hover = MOT_THST_HOVER:get() or 0.5
        gcs:send_text(MAV_SEVERITY.INFO, "TLIN: Starting hover test")
        gcs:send_text(MAV_SEVERITY.INFO, string.format("TLIN: hover=%.2f range=%.2f-%.2f",
                      hover, min_thr, max_thr))
    end

    -- Check if current step is complete
    local step = HOVER_STEPS[g_state.hover_step]
    if not step then
        return true  -- all steps complete
    end

    local step_elapsed = now_ms - g_state.hover_step_start_ms
    if step_elapsed >= step.duration_ms then
        g_state.hover_step = g_state.hover_step + 1
        g_state.hover_step_start_ms = now_ms
        step = HOVER_STEPS[g_state.hover_step]
        if not step then
            gcs:send_text(MAV_SEVERITY.INFO, "TLIN: Collecting done")
            return true  -- all steps complete
        end
        -- Show progress: step name, percentage, and current throttle/samples
        local pct = math.floor(g_state.hover_step * 100 / #HOVER_STEPS)
        gcs:send_text(MAV_SEVERITY.INFO, string.format("TLIN: %s %d%% t=%.2f n=%d",
                      step.name, pct, g_state.last_throttle, g_state.sample_count))
    end

    -- Command velocity (NED: positive Z is down)
    local vel = Vector3f()
    vel:x(0)
    vel:y(0)
    vel:z(step.vz)
    if not vehicle:set_target_velocity_NED(vel) then
        gcs:send_text(MAV_SEVERITY.WARNING, "TLIN: Velocity cmd failed")
    end

    -- Collect data
    local throttle = motors:get_throttle()
    local accel = ins:get_accel(0)
    if throttle and accel then
        add_data_point(throttle, accel:z())
        g_state.last_throttle = throttle
    end

    -- Periodic diagnostic every 5 seconds
    if now_ms - g_state.last_diag_ms > 5000 then
        g_state.last_diag_ms = now_ms
        gcs:send_text(MAV_SEVERITY.INFO, string.format("TLIN: thr=%.2f n=%d",
                      g_state.last_throttle, g_state.sample_count))
    end

    return false
end

-- Run forward flight test mode
local function run_forward_test()
    local now_ms = millis():tofloat()

    -- Forward flight phases: accelerate, cruise+wave, turnaround
    local FWD_PHASE = {ACCEL = 0, WAVE = 1, TURN = 2}

    if g_state.fwd_phase == 0 and g_state.collect_start_ms == 0 then
        -- Initialize forward flight
        g_state.collect_start_ms = now_ms
        g_state.fwd_phase = FWD_PHASE.ACCEL
        g_state.turnaround_count = 0

        -- Get current heading as flight direction
        g_state.fwd_heading_rad = ahrs:get_yaw_rad() or 0
        gcs:send_text(MAV_SEVERITY.INFO, "TLIN: Starting forward test")
    end

    local test_spd = TLIN_SPD:get()
    local vel = Vector3f()

    -- Check distance for turnaround
    local pos = ahrs:get_location()
    local dist = 0
    if pos and g_state.start_loc then
        dist = pos:get_distance(g_state.start_loc)
    end

    if g_state.fwd_phase == FWD_PHASE.ACCEL then
        -- Accelerate to cruise speed
        vel:x(test_spd * math.cos(g_state.fwd_heading_rad))
        vel:y(test_spd * math.sin(g_state.fwd_heading_rad))
        vel:z(0)

        local cur_vel = ahrs:get_velocity_NED()
        if cur_vel then
            local hspd = math.sqrt(cur_vel:x() * cur_vel:x() + cur_vel:y() * cur_vel:y())
            if hspd > test_spd * 0.9 then
                g_state.fwd_phase = FWD_PHASE.WAVE
                g_state.hover_step_start_ms = now_ms
            end
        end

    elseif g_state.fwd_phase == FWD_PHASE.WAVE then
        -- Cruise with vertical oscillation for data collection
        vel:x(test_spd * math.cos(g_state.fwd_heading_rad))
        vel:y(test_spd * math.sin(g_state.fwd_heading_rad))

        -- Sine wave vertical motion
        local wave_period_ms = 4000
        local wave_phase = ((now_ms - g_state.hover_step_start_ms) % wave_period_ms) / wave_period_ms
        local vz = 1.5 * math.sin(wave_phase * 2 * math.pi)
        vel:z(vz)

        -- Check for turnaround
        if dist > TLIN_DIST:get() * 0.8 then
            g_state.fwd_phase = FWD_PHASE.TURN
            g_state.turnaround_count = g_state.turnaround_count + 1
        end

    elseif g_state.fwd_phase == FWD_PHASE.TURN then
        -- Brake and reverse direction
        vel:x(0)
        vel:y(0)
        vel:z(0)

        local cur_vel = ahrs:get_velocity_NED()
        if cur_vel then
            local hspd = math.sqrt(cur_vel:x() * cur_vel:x() + cur_vel:y() * cur_vel:y())
            if hspd < 1.0 then
                -- Reverse heading
                g_state.fwd_heading_rad = g_state.fwd_heading_rad + math.pi
                g_state.fwd_phase = FWD_PHASE.ACCEL

                -- Check if we've done enough passes
                if g_state.turnaround_count >= 2 and have_sufficient_data() then
                    return true  -- test complete
                end
            end
        end
    end

    vehicle:set_target_velocity_NED(vel)

    -- Collect data during wave phase
    if g_state.fwd_phase == FWD_PHASE.WAVE then
        local throttle = motors:get_throttle()
        local accel = ins:get_accel(0)
        if throttle and accel then
            add_data_point(throttle, accel:z())
        end
    end

    -- Timeout after 120 seconds
    if now_ms - g_state.collect_start_ms > 120000 then
        return true
    end

    return false
end

-- Start the test
local function start_test()
    if not check_preconditions() then
        TLIN_STATE:set(STATE.ERROR)
        return
    end

    gcs:send_text(MAV_SEVERITY.INFO, "TLIN: Starting test")

    -- Initialize state
    g_state.phase = PHASE.STABILIZE
    g_state.start_time_ms = millis():tofloat()
    g_state.start_alt_m = baro:get_altitude() or 0
    g_state.start_loc = ahrs:get_location()
    g_state.hover_step = 0
    g_state.hover_step_start_ms = 0
    g_state.collect_start_ms = 0
    g_state.fwd_phase = 0
    g_state.turnaround_count = 0
    g_state.sample_count = 0
    g_state.last_diag_ms = 0
    g_state.last_throttle = 0

    clear_bins()

    TLIN_STATE:set(STATE.RUNNING)
    TLIN_START:set(0)  -- Reset trigger
end

-- Main update function
local function update()
    -- Check if enabled
    if TLIN_ENABLE:get() ~= 1 then
        return update, 500
    end

    -- Check for RC switch trigger
    local switch_pos = rc:get_aux_cached(SCRIPTING_AUX_FUNC)
    if switch_pos then
        if switch_pos == 2 and g_state.last_switch_pos ~= 2 then
            -- Switch went high - start test
            if g_state.phase == PHASE.INIT or g_state.phase == PHASE.DONE then
                g_state.phase = PHASE.INIT
                start_test()
            end
        elseif switch_pos == 0 and g_state.last_switch_pos ~= 0 then
            -- Switch went low during test - abort
            if g_state.phase ~= PHASE.INIT and g_state.phase ~= PHASE.DONE then
                abort_test("RC switch")
            end
        end
        g_state.last_switch_pos = switch_pos
    end

    -- Check for parameter trigger
    if TLIN_START:get() == 1 then
        if g_state.phase == PHASE.INIT or g_state.phase == PHASE.DONE then
            g_state.phase = PHASE.INIT
            start_test()
        end
        TLIN_START:set(0)
    end

    -- Run state machine
    if g_state.phase == PHASE.STABILIZE then
        -- Initial stabilization period
        local elapsed = millis():tofloat() - g_state.start_time_ms
        if elapsed > 2000 then
            g_state.phase = PHASE.COLLECT
        else
            -- Hold position
            local vel = Vector3f()
            vel:x(0)
            vel:y(0)
            vel:z(0)
            vehicle:set_target_velocity_NED(vel)
        end

    elseif g_state.phase == PHASE.COLLECT then
        -- Check safety
        if not check_safety() then
            abort_test("Safety limit")
            return update, 50
        end

        -- Run appropriate test mode
        local test_complete = false
        if TLIN_MODE:get() == 0 then
            test_complete = run_hover_test()
        else
            test_complete = run_forward_test()
        end

        if test_complete then
            g_state.phase = PHASE.SOLVE
        end

    elseif g_state.phase == PHASE.SOLVE then
        -- Calculate result
        if have_sufficient_data() then
            local expo = solve_expo()
            complete_test(expo)
        else
            abort_test("Insufficient data")
        end
    end

    -- Fast loop during active test
    if g_state.phase == PHASE.COLLECT or g_state.phase == PHASE.STABILIZE then
        return update, 20  -- 50Hz during test
    end

    return update, 100
end

-- Protected wrapper for error handling
local function protected_wrapper()
    local success, result, interval = pcall(update)
    if not success then
        gcs:send_text(MAV_SEVERITY.ERROR, "TLIN: Error - " .. tostring(result))
        TLIN_STATE:set(STATE.ERROR)
        return protected_wrapper, 1000
    end
    return protected_wrapper, interval or 100
end

gcs:send_text(MAV_SEVERITY.INFO, "TLIN: Core script loaded")

return protected_wrapper()
