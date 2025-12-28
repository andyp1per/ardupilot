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
  // @Description: Test mode selection. 0=Hover, 1=Forward, 2=Calibration, 3=Full (all three in sequence).
  // @Values: 0:Hover,1:Forward,2:Calibration,3:Full
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

--[[
  // @Param: TLIN_ACCEL
  // @DisplayName: Test Acceleration
  // @Description: Aggressiveness for test maneuvers. Controls vertical velocities in hover test and horizontal acceleration in forward flight. 5=baseline, 10=2x aggressive.
  // @Units: m/s/s
  // @Range: 1 15
  // @User: Standard
--]]
local TLIN_ACCEL = bind_add_param("ACCEL", 9, 5.0)

-- Bind to existing ArduPilot parameters
local MOT_THST_EXPO = bind_param("MOT_THST_EXPO")
local MOT_THST_HOVER = bind_param("MOT_THST_HOVER")
local MOT_SPIN_MIN = bind_param("MOT_SPIN_MIN")
local MOT_SPIN_MAX = bind_param("MOT_SPIN_MAX")

-- Rate PID parameters for compensation when spin range changes
local RATE_PID_PARAMS = {
    "ATC_RAT_RLL_P", "ATC_RAT_RLL_I", "ATC_RAT_RLL_D",
    "ATC_RAT_PIT_P", "ATC_RAT_PIT_I", "ATC_RAT_PIT_D",
    "ATC_RAT_YAW_P", "ATC_RAT_YAW_I", "ATC_RAT_YAW_D",
    "PSC_ACCZ_P", "PSC_ACCZ_I", "PSC_ACCZ_D"
}

-- Full test sub-phases
local FULL_SUBTEST = {
    CALIB = 1,
    HOVER = 2,
    FORWARD = 3
}

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
    last_throttle = 0,
    full_subtest = 0,  -- For full test mode: tracks which sub-test
    waiting_for_guided = false,  -- For auto LOITER->GUIDED transition
    expected_motor_out = nil,  -- Expected motor output at hover after spin_min compensation
    measured_hover_sum = 0,    -- Sum of throttle samples during hover hold
    measured_hover_count = 0   -- Count of hover samples
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

-- Raw data storage for dynamic binning (overpowered aircraft support)
local MAX_RAW_SAMPLES = 1000  -- Reduced for memory efficiency
local g_raw_data = {}
local g_observed_min_thr = 1.0
local g_observed_max_thr = 0.0

-- Get throttle range based on MOT_THST_HOVER
local function get_throttle_range()
    local hover = MOT_THST_HOVER:get() or 0.5
    -- Range from 1% (to accept overpowered vehicles)
    local min_thr = 0.01
    -- Upper limit: Allow collecting data up to 95% throttle or at least 0.85
    -- even for overpowered aircraft, to capture the curve tail if reached.
    -- However, practically limit based on hover to focus resolution where needed.
    local max_thr = math.min(math.max(hover * 3.0, 0.85), 0.95)
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

-- Re-bin state (must be defined before clear_bins)
local g_rebin_state = {
    in_progress = false,
    index = 1,
    min_thr = 0,
    max_thr = 0,
    range = 0,
    complete = false  -- Set when rebinning done, cleared when solve starts
}

-- Solve state for chunked processing (must be defined before clear_bins)
local g_solve_state = {
    in_progress = false,
    phase = 0,  -- 0 = grid search, 1 = refinement
    k = 0,
    best_expo = 0,
    best_score = 1e10,
    start_expo = 0,
    end_expo = 0
}

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
    -- Clear raw data storage
    g_raw_data = {}
    g_observed_min_thr = 1.0
    g_observed_max_thr = 0.0
    -- Reset rebin state
    g_rebin_state.in_progress = false
    g_rebin_state.index = 1
    g_rebin_state.complete = false
    -- Reset solve state
    g_solve_state.in_progress = false
    -- Reset hover measurement
    g_state.measured_hover_sum = 0
    g_state.measured_hover_count = 0
end

-- Add data point to appropriate bin
local function add_data_point(throttle, accel_z)
    g_state.last_throttle = throttle
    -- Track observed throttle range
    if throttle > 0.005 then  -- Ignore near-zero values
        if throttle < g_observed_min_thr then g_observed_min_thr = throttle end
        if throttle > g_observed_max_thr then g_observed_max_thr = throttle end
    end
    -- Store raw data for later dynamic binning
    if #g_raw_data < MAX_RAW_SAMPLES then
        table.insert(g_raw_data, {thr = throttle, acc = -accel_z})
    end
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

-- Re-bin raw data using observed throttle range (for overpowered aircraft)
-- Processes in chunks to avoid Lua timeout
local function rebin_with_observed_range()
    -- Start new rebinning process
    if not g_rebin_state.in_progress then
        -- Clear existing bins
        for i = 1, NUM_BINS do
            g_bins[i].sum_accel = 0
            g_bins[i].sum_throttle = 0
            g_bins[i].count = 0
        end

        g_rebin_state.min_thr = g_observed_min_thr
        g_rebin_state.max_thr = g_observed_max_thr
        g_rebin_state.range = g_rebin_state.max_thr - g_rebin_state.min_thr

        if g_rebin_state.range < 0.005 then
            gcs:send_text(MAV_SEVERITY.WARNING, string.format("TLIN: Range too small: %.3f-%.3f",
                          g_rebin_state.min_thr, g_rebin_state.max_thr))
            return true, false  -- done, failed
        end

        gcs:send_text(MAV_SEVERITY.INFO, string.format("TLIN: Re-binning %.3f-%.3f",
                      g_rebin_state.min_thr, g_rebin_state.max_thr))
        g_rebin_state.index = 1
        g_rebin_state.in_progress = true
    end

    -- Process a chunk of samples (limit per call to avoid timeout)
    local chunk_size = 50
    local end_idx = math.min(g_rebin_state.index + chunk_size - 1, #g_raw_data)
    local min_thr = g_rebin_state.min_thr
    local max_thr = g_rebin_state.max_thr
    local range = g_rebin_state.range

    for i = g_rebin_state.index, end_idx do
        local pt = g_raw_data[i]
        if pt and pt.thr >= min_thr and pt.thr <= max_thr then
            local idx = math.floor((pt.thr - min_thr) / range * NUM_BINS) + 1
            if idx > NUM_BINS then idx = NUM_BINS end
            if idx < 1 then idx = 1 end
            g_bins[idx].sum_accel = g_bins[idx].sum_accel + pt.acc
            g_bins[idx].sum_throttle = g_bins[idx].sum_throttle + pt.thr
            g_bins[idx].count = g_bins[idx].count + 1
        end
    end

    g_rebin_state.index = end_idx + 1

    -- Check if done
    if g_rebin_state.index > #g_raw_data then
        g_rebin_state.in_progress = false
        return true, true  -- done, success
    end

    return false, false  -- not done yet
end

-- Check if we have enough data in bins
-- For expo calculation, need data spread across multiple bins
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
    -- Need at least 3 bins with data for meaningful expo calculation
    return filled_bins >= 3 and total_samples >= 30
end

-- Calculate linearity score for a given expo value
-- Solves for the best slope (thrust scale) dynamically to handle any power-to-weight ratio
local function calc_linearity_score(expo)
    local sum_xy = 0
    local sum_xx = 0
    local point_count = 0

    -- 1. Calculate sums for linear regression (fitting Accel = m * Throttle_Lin)
    for i = 1, NUM_BINS do
        if g_bins[i].count > 0 then
            local avg_throttle = g_bins[i].sum_throttle / g_bins[i].count
            local avg_accel = g_bins[i].sum_accel / g_bins[i].count

            -- Calculate linearized throttle for this candidate expo
            local u_lin = (1 - expo) * avg_throttle + expo * avg_throttle * avg_throttle

            sum_xy = sum_xy + (u_lin * avg_accel)
            sum_xx = sum_xx + (u_lin * u_lin)
            point_count = point_count + 1
        end
    end

    if point_count < 3 or sum_xx < 0.0001 then
        return 1e10
    end

    -- 2. Calculate best fit slope 'm' (Scalar between linearized throttle and accel)
    local slope = sum_xy / sum_xx

    -- 3. Calculate residual error (R-squared-ish)
    local total_error = 0
    for i = 1, NUM_BINS do
        if g_bins[i].count > 0 then
            local avg_throttle = g_bins[i].sum_throttle / g_bins[i].count
            local avg_accel = g_bins[i].sum_accel / g_bins[i].count

            local u_lin = (1 - expo) * avg_throttle + expo * avg_throttle * avg_throttle
            local expected_accel = u_lin * slope

            local err = avg_accel - expected_accel
            total_error = total_error + (err * err)
        end
    end

    return total_error / point_count
end

-- Find optimal expo value (chunked to avoid timeout)
-- Returns (done, result) where result is only valid when done=true
local function solve_expo_chunked()
    local chunk_size = 3  -- iterations per call (keep small to avoid timeout)

    if not g_solve_state.in_progress then
        -- Start new solve
        g_solve_state.in_progress = true
        g_solve_state.phase = 0
        g_solve_state.k = 0
        g_solve_state.best_expo = 0
        g_solve_state.best_score = 1e10
    end

    if g_solve_state.phase == 0 then
        -- Grid search from 0.0 to 1.0 in steps of 0.02
        local end_k = math.min(g_solve_state.k + chunk_size - 1, 50)
        for k = g_solve_state.k, end_k do
            local expo = k * 0.02
            local score = calc_linearity_score(expo)
            if score < g_solve_state.best_score then
                g_solve_state.best_score = score
                g_solve_state.best_expo = expo
            end
        end
        g_solve_state.k = end_k + 1

        if g_solve_state.k > 50 then
            -- Move to refinement phase
            g_solve_state.phase = 1
            g_solve_state.k = 0
            g_solve_state.start_expo = math.max(0, g_solve_state.best_expo - 0.02)
            g_solve_state.end_expo = math.min(1, g_solve_state.best_expo + 0.02)
        end
        return false, 0  -- not done
    else
        -- Refine around best value
        local end_k = math.min(g_solve_state.k + chunk_size - 1, 20)
        for k = g_solve_state.k, end_k do
            local expo = g_solve_state.start_expo + k * (g_solve_state.end_expo - g_solve_state.start_expo) / 20
            local score = calc_linearity_score(expo)
            if score < g_solve_state.best_score then
                g_solve_state.best_score = score
                g_solve_state.best_expo = expo
            end
        end
        g_solve_state.k = end_k + 1

        if g_solve_state.k > 20 then
            -- Done
            g_solve_state.in_progress = false
            return true, math.floor(g_solve_state.best_expo * 100 + 0.5) / 100
        end
        return false, 0  -- not done
    end
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
                -- Clamp to 3% minimum to handle deadzone safety
                return math.max(0.03, math.floor(suggested * 100 + 0.5) / 100)
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

    -- In Full mode, auto-switch from LOITER to GUIDED
    if vehicle:get_mode() ~= MODE_GUIDED then
        if TLIN_MODE:get() == 3 and vehicle:get_mode() == MODE_LOITER then
            gcs:send_text(MAV_SEVERITY.INFO, "TLIN: Switching to GUIDED")
            if not vehicle:set_mode(MODE_GUIDED) then
                gcs:send_text(MAV_SEVERITY.WARNING, "TLIN: Failed to set GUIDED")
                return false
            end
            g_state.waiting_for_guided = true
            return false  -- Will retry next cycle
        end
        gcs:send_text(MAV_SEVERITY.WARNING, "TLIN: Switch to GUIDED first")
        return false
    end
    g_state.waiting_for_guided = false

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
    -- Check altitude limits (relative to start)
    -- Allow 15m below start (for descent maneuvers) and 25m above
    local alt = baro:get_altitude()
    if alt then
        local rel_alt = alt - g_state.start_alt_m
        if rel_alt < -15 then
            gcs:send_text(MAV_SEVERITY.WARNING, string.format("TLIN: Too low (%.0fm)", rel_alt))
            return false
        end
        if rel_alt > 25 then
            gcs:send_text(MAV_SEVERITY.WARNING, string.format("TLIN: Too high (%.0fm)", rel_alt))
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

-- Compensate PIDs and hover throttle when spin range changes to maintain identical performance
-- Scale factor = old_range / new_range
local function compensate_pids_for_spin_range(old_spin_min, old_spin_max, new_spin_min, new_spin_max)
    local old_range = old_spin_max - old_spin_min
    local new_range = new_spin_max - new_spin_min

    -- Sanity check ranges
    if old_range <= 0 or new_range <= 0 then
        gcs:send_text(MAV_SEVERITY.WARNING, "TLIN: Invalid spin range for PID compensation")
        return false
    end

    local scale = old_range / new_range

    -- Calculate new MOT_THST_HOVER based on measured hover output
    -- If we have measurement, use it. Otherwise use previous parameter
    local old_hover_param = MOT_THST_HOVER:get() or 0.5
    local old_motor_out

    if g_state.measured_hover_count > 10 then
        -- Use measured hover output directly
        old_motor_out = g_state.measured_hover_sum / g_state.measured_hover_count
        gcs:send_text(MAV_SEVERITY.INFO, string.format("TLIN: Measured Hover Out: %.3f", old_motor_out))
    else
        -- Fallback to existing parameter
        old_motor_out = old_spin_min + old_hover_param * old_range
    end

    -- Calculate new hover throttle parameter
    -- New motor_out = new_spin_min + new_hover * new_range
    -- We want New motor_out == Old motor_out (physical thrust same)
    local new_hover = (old_motor_out - new_spin_min) / new_range

    -- Clamp to valid range
    new_hover = math.max(0.01, math.min(0.9, new_hover))

    gcs:send_text(MAV_SEVERITY.INFO, string.format("TLIN: Hover %.2f -> %.2f", old_hover_param, new_hover))
    MOT_THST_HOVER:set(new_hover)

    -- Store expected motor output for later validation
    g_state.expected_motor_out = old_motor_out

    -- Only scale PIDs if change is significant (> 1%)
    if math.abs(scale - 1.0) < 0.01 then
        gcs:send_text(MAV_SEVERITY.INFO, "TLIN: Spin range change <1%, no PID scaling needed")
        return true
    end

    gcs:send_text(MAV_SEVERITY.INFO, string.format("TLIN: Scaling PIDs by %.3f", scale))

    local scaled_count = 0
    for _, name in ipairs(RATE_PID_PARAMS) do
        local p = bind_param(name)
        if p then
            local old_val = p:get()
            local new_val = old_val * scale
            p:set(new_val)
            scaled_count = scaled_count + 1
        end
    end

    gcs:send_text(MAV_SEVERITY.NOTICE, string.format("TLIN: Scaled %d PID params", scaled_count))

    return true
end

-- Validate hover throttle compensation by comparing measured vs expected motor output
-- Call this after hover test data collection to verify compensation was correct
local function validate_hover_compensation()
    if not g_state.expected_motor_out then
        return  -- No compensation was done, nothing to validate
    end

    -- Find the bin where acceleration is closest to gravity (9.8 m/s^2)
    -- This is where the vehicle was actually hovering, regardless of what
    -- throttle value we predicted for hover
    local GRAVITY = 9.8
    local best_bin = 0
    local best_accel_error = 1000
    local best_count = 0

    for i = 1, NUM_BINS do
        if g_bins[i].count > 10 then
            local avg_accel = g_bins[i].sum_accel / g_bins[i].count
            local accel_error = math.abs(avg_accel - GRAVITY)
            if accel_error < best_accel_error then
                best_accel_error = accel_error
                best_bin = i
                best_count = g_bins[i].count
            end
        end
    end

    if best_bin == 0 or best_count < 20 then
        gcs:send_text(MAV_SEVERITY.INFO, "TLIN: Not enough hover data to validate")
        g_state.expected_motor_out = nil
        return
    end

    -- Use the bin where we were actually hovering
    local avg_measured_throttle = g_bins[best_bin].sum_throttle / g_bins[best_bin].count
    local avg_accel = g_bins[best_bin].sum_accel / g_bins[best_bin].count

    -- Convert measured throttle to motor output
    local spin_min = MOT_SPIN_MIN:get() or 0.15
    local spin_max = MOT_SPIN_MAX:get() or 0.95
    local range = spin_max - spin_min
    local measured_motor_out = spin_min + avg_measured_throttle * range

    -- Compare to expected
    local error_pct = math.abs(measured_motor_out - g_state.expected_motor_out) / g_state.expected_motor_out * 100

    if error_pct > 10 then
        gcs:send_text(MAV_SEVERITY.WARNING, string.format("TLIN: Hover validation: %.1f%% error (accel=%.1f)",
                      error_pct, avg_accel))
        gcs:send_text(MAV_SEVERITY.WARNING, string.format("TLIN: Expected motor=%.3f, measured=%.3f",
                      g_state.expected_motor_out, measured_motor_out))

        -- Auto-adjust expo based on the error
        local current_expo = MOT_THST_EXPO:get() or 0
        if measured_motor_out < g_state.expected_motor_out and current_expo > 0.01 then
            -- Hovering at lower throttle = expo too high, reduce it
            -- Scale expo by the ratio of measured/expected
            local ratio = measured_motor_out / g_state.expected_motor_out
            local new_expo = current_expo * ratio
            new_expo = math.max(0, math.min(0.95, new_expo))
            gcs:send_text(MAV_SEVERITY.NOTICE, string.format("TLIN: Reducing expo %.2f -> %.2f", current_expo, new_expo))
            MOT_THST_EXPO:set(new_expo)
        elseif measured_motor_out > g_state.expected_motor_out then
            -- Hovering at higher throttle = expo too low, increase it
            -- But be conservative - only increase by half the error ratio
            local ratio = measured_motor_out / g_state.expected_motor_out
            local new_expo = current_expo + (1 - current_expo) * (ratio - 1) * 0.5
            new_expo = math.max(0, math.min(0.95, new_expo))
            if new_expo > current_expo + 0.05 then
                gcs:send_text(MAV_SEVERITY.NOTICE, string.format("TLIN: Increasing expo %.2f -> %.2f", current_expo, new_expo))
                MOT_THST_EXPO:set(new_expo)
            end
        end
    elseif error_pct > 5 then
        gcs:send_text(MAV_SEVERITY.INFO, string.format("TLIN: Hover validation: %.1f%% error (marginal)",
                      error_pct))
    else
        gcs:send_text(MAV_SEVERITY.INFO, string.format("TLIN: Hover validation OK (%.1f%% error)", error_pct))
    end

    -- Clear for next run
    g_state.expected_motor_out = nil
end

-- Complete spin-min calibration - analyze and set MOT_SPIN_MIN
local function complete_spinmin_calibration()
    local current_spin_min = MOT_SPIN_MIN:get() or 0.15
    local spin_min_result = analyze_spin_min()

    if not spin_min_result then
        gcs:send_text(MAV_SEVERITY.WARNING, "TLIN: Could not determine spin-min")
        gcs:send_text(MAV_SEVERITY.INFO, string.format("TLIN: Keeping MOT_SPIN_MIN=%.2f", current_spin_min))
        TLIN_STATE:set(STATE.ERROR)
        vehicle:set_mode(MODE_LOITER)
        g_state.phase = PHASE.DONE
        return
    end

    gcs:send_text(MAV_SEVERITY.INFO, string.format("TLIN: SpinMin Result=%.2f", spin_min_result))
    gcs:send_text(MAV_SEVERITY.INFO, string.format("TLIN: Previous MOT_SPIN_MIN=%.2f", current_spin_min))

    -- Set the new spin_min value and compensate PIDs
    if math.abs(spin_min_result - current_spin_min) > 0.01 then
        local current_spin_max = MOT_SPIN_MAX:get() or 0.95

        -- Compensate PIDs before changing spin_min to maintain identical performance
        compensate_pids_for_spin_range(current_spin_min, current_spin_max,
                                       spin_min_result, current_spin_max)

        MOT_SPIN_MIN:set(spin_min_result)
        gcs:send_text(MAV_SEVERITY.NOTICE, string.format("TLIN: Set MOT_SPIN_MIN=%.2f", spin_min_result))
    else
        gcs:send_text(MAV_SEVERITY.INFO, "TLIN: MOT_SPIN_MIN unchanged")
    end

    -- Store result and log
    TLIN_RESULT:set(spin_min_result)
    TLIN_STATE:set(STATE.COMPLETE)

    -- Calculate scale for logging
    local current_spin_max = MOT_SPIN_MAX:get() or 0.95
    local old_range = current_spin_max - current_spin_min
    local new_range = current_spin_max - spin_min_result
    local scale = (new_range > 0) and (old_range / new_range) or 1.0

    logger:write("TSPN", "SpinMin,OldSpinMin,Scale", "fff", spin_min_result, current_spin_min, scale)

    vehicle:set_mode(MODE_LOITER)
    g_state.phase = PHASE.DONE
end

-- Calibration mode throttle steps (relative to hover)
-- Steps through throttle levels and measures acceleration directly
-- Calibration steps - shorter duration and smaller increments for safety
-- On overpowered aircraft, large throttle increases cause rapid climb
local CALIB_STEPS = {
    {offset = 0.00, duration_ms = 1000, name = "Hover"},
    {offset = -0.01, duration_ms = 600, name = "Thr-1%"},
    {offset = 0.00, duration_ms = 400, name = "Return"},
    {offset = 0.01, duration_ms = 600, name = "Thr+1%"},
    {offset = 0.00, duration_ms = 400, name = "Return"},
    {offset = 0.02, duration_ms = 600, name = "Thr+2%"},
    {offset = 0.00, duration_ms = 400, name = "Return"},
    {offset = 0.03, duration_ms = 600, name = "Thr+3%"},
    {offset = 0.00, duration_ms = 400, name = "Return"},
    {offset = 0.04, duration_ms = 500, name = "Thr+4%"},
    {offset = 0.00, duration_ms = 400, name = "Return"},
    {offset = 0.05, duration_ms = 500, name = "Thr+5%"},
    {offset = 0.00, duration_ms = 1000, name = "Hover"},
}

-- Spin-min calibration steps - uses velocity commands to explore low throttle
-- Gentle descents followed by climbs to maintain altitude while getting low throttle data
local SPINMIN_STEPS = {
    {vz = 0.0, duration_ms = 1500, name = "Hold"},
    -- Descent 1: gentle
    {vz = 2.0, duration_ms = 1000, name = "Dn1"},
    {vz = -2.0, duration_ms = 1200, name = "Up1"},
    {vz = 0.0, duration_ms = 800, name = "Settle1"},
    -- Descent 2: moderate
    {vz = 3.0, duration_ms = 1000, name = "Dn2"},
    {vz = -3.0, duration_ms = 1200, name = "Up2"},
    {vz = 0.0, duration_ms = 800, name = "Settle2"},
    -- Descent 3: faster
    {vz = 4.0, duration_ms = 1000, name = "Dn3"},
    {vz = -4.0, duration_ms = 1200, name = "Up3"},
    {vz = 0.0, duration_ms = 800, name = "Settle3"},
    -- Descent 4: fastest (still safe)
    {vz = 5.0, duration_ms = 1000, name = "Dn4"},
    {vz = -5.0, duration_ms = 1200, name = "Up4"},
    {vz = 0.0, duration_ms = 1500, name = "Hold2"},
}

-- Run spin-min calibration using velocity commands (for overpowered aircraft)
local function run_spinmin_calibration()
    local now_ms = millis():tofloat()

    if g_state.hover_step == 0 then
        g_state.hover_step = 1
        g_state.hover_step_start_ms = now_ms
        g_state.collect_start_ms = now_ms
        gcs:send_text(MAV_SEVERITY.INFO, "TLIN: Starting spin-min calibration")
        gcs:send_text(MAV_SEVERITY.INFO, string.format("TLIN: Current MOT_SPIN_MIN=%.2f", MOT_SPIN_MIN:get() or 0.15))
        -- Reset measurement counters
        g_state.measured_hover_sum = 0
        g_state.measured_hover_count = 0
    end

    -- Check if current step is complete
    local step = SPINMIN_STEPS[g_state.hover_step]
    if not step then
        return true  -- all steps complete
    end

    local step_elapsed = now_ms - g_state.hover_step_start_ms
    if step_elapsed >= step.duration_ms then
        g_state.hover_step = g_state.hover_step + 1
        g_state.hover_step_start_ms = now_ms
        step = SPINMIN_STEPS[g_state.hover_step]
        if not step then
            gcs:send_text(MAV_SEVERITY.INFO, "TLIN: Spin-min data collection done")
            return true
        end
        local pct = math.floor(g_state.hover_step * 100 / #SPINMIN_STEPS)
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

    -- Collect data - focus on spin bins for low throttle analysis
    local throttle = motors:get_throttle()
    local accel = ins:get_accel(0)
    if throttle and accel then
        add_data_point(throttle, accel:z())
        g_state.last_throttle = throttle

        -- Measure true hover output during "Hold" and "Settle" phases
        -- This ensures we know the real thrust required to hover, regardless of parameters
        if (step.name:find("Hold") or step.name:find("Settle")) and step_elapsed > 500 then
            -- Calculate total motor output (PWM-equivalent 0-1)
            -- Current output = spin_min + throttle * (spin_max - spin_min)
            local s_min = MOT_SPIN_MIN:get() or 0.15
            local s_max = MOT_SPIN_MAX:get() or 0.95
            local total_out = s_min + throttle * (s_max - s_min)

            g_state.measured_hover_sum = g_state.measured_hover_sum + total_out
            g_state.measured_hover_count = g_state.measured_hover_count + 1
        end
    end

    -- Periodic diagnostic
    if now_ms - g_state.last_diag_ms > 3000 then
        g_state.last_diag_ms = now_ms
        local rel_alt = (baro:get_altitude() or 0) - g_state.start_alt_m
        gcs:send_text(MAV_SEVERITY.INFO, string.format("TLIN: thr=%.2f n=%d alt=%+.0fm",
                      g_state.last_throttle, g_state.sample_count, rel_alt))
    end

    return false
end

-- Run calibration test mode - direct throttle control or spin-min for overpowered
local function run_calibration_test()
    local now_ms = millis():tofloat()
    local hover = MOT_THST_HOVER:get() or 0.5

    -- For overpowered aircraft, use velocity-based spin-min calibration
    if hover < 0.25 then
        return run_spinmin_calibration()
    end

    if g_state.hover_step == 0 then
        g_state.hover_step = 1
        g_state.hover_step_start_ms = now_ms
        g_state.collect_start_ms = now_ms
        gcs:send_text(MAV_SEVERITY.INFO, "TLIN: Starting calibration test")
        gcs:send_text(MAV_SEVERITY.INFO, string.format("TLIN: hover=%.2f", hover))
    end

    -- Check if current step is complete
    local step = CALIB_STEPS[g_state.hover_step]
    if not step then
        return true  -- all steps complete
    end

    local step_elapsed = now_ms - g_state.hover_step_start_ms
    if step_elapsed >= step.duration_ms then
        g_state.hover_step = g_state.hover_step + 1
        g_state.hover_step_start_ms = now_ms
        step = CALIB_STEPS[g_state.hover_step]
        if not step then
            gcs:send_text(MAV_SEVERITY.INFO, "TLIN: Calibration done")
            return true
        end
        local pct = math.floor(g_state.hover_step * 100 / #CALIB_STEPS)
        local thr = hover + step.offset
        gcs:send_text(MAV_SEVERITY.INFO, string.format("TLIN: %s %d%% thr=%.2f n=%d",
                      step.name, pct, thr, g_state.sample_count))
    end

    -- Command direct throttle with zero rates (hold level)
    local thr = hover + step.offset
    thr = math.max(0.01, math.min(1.0, thr))  -- clamp to valid range
    if not vehicle:set_target_rate_and_throttle(0, 0, 0, thr) then
        gcs:send_text(MAV_SEVERITY.WARNING, "TLIN: Throttle cmd failed")
    end

    -- Collect data - skip first 300ms of each step for settling
    if step_elapsed > 300 then
        local throttle = motors:get_throttle()
        local accel = ins:get_accel(0)
        if throttle and accel then
            add_data_point(throttle, accel:z())
            g_state.last_throttle = throttle
        end
    end

    -- Periodic diagnostic
    if now_ms - g_state.last_diag_ms > 3000 then
        g_state.last_diag_ms = now_ms
        local rel_alt = (baro:get_altitude() or 0) - g_state.start_alt_m
        gcs:send_text(MAV_SEVERITY.INFO, string.format("TLIN: thr=%.2f n=%d alt=%+.0fm",
                      g_state.last_throttle, g_state.sample_count, rel_alt))
    end

    return false
end

-- Hover test vertical step sequences
-- Standard sequence for normal aircraft
local HOVER_STEPS_NORMAL = {
    {vz = 0.0, duration_ms = 1000, name = "Hold"},
    -- Set 1: gentle warmup
    {vz = -4.0, duration_ms = 400, name = "Up1"},
    {vz = 4.0, duration_ms = 500, name = "Dn1"},
    {vz = 0.0, duration_ms = 400, name = "Settle1"},
    -- Set 2: build descent velocity then brake hard
    {vz = -6.0, duration_ms = 300, name = "Up2"},
    {vz = 10.0, duration_ms = 800, name = "Dn2"},
    {vz = -8.0, duration_ms = 600, name = "Brake2"},
    {vz = 0.0, duration_ms = 400, name = "Settle2"},
    -- Set 3: aggressive - long descent, hard brake
    {vz = -8.0, duration_ms = 300, name = "Up3"},
    {vz = 15.0, duration_ms = 800, name = "Dn3"},
    {vz = -10.0, duration_ms = 600, name = "Brake3"},
    {vz = 0.0, duration_ms = 400, name = "Settle3"},
    -- Set 4: repeat aggressive pattern
    {vz = -8.0, duration_ms = 300, name = "Up4"},
    {vz = 15.0, duration_ms = 800, name = "Dn4"},
    {vz = -10.0, duration_ms = 600, name = "Brake4"},
    {vz = 0.0, duration_ms = 1000, name = "Hold"},
}

-- Gentle sequence for overpowered aircraft (hover < 25%)
-- Uses slower velocities and longer holds to avoid rapid altitude changes
local HOVER_STEPS_GENTLE = {
    {vz = 0.0, duration_ms = 1500, name = "Hold"},
    -- Set 1: very gentle
    {vz = -1.5, duration_ms = 600, name = "Up1"},
    {vz = 0.0, duration_ms = 500, name = "Settle1"},
    {vz = 2.0, duration_ms = 800, name = "Dn1"},
    {vz = -2.0, duration_ms = 600, name = "Brake1"},
    {vz = 0.0, duration_ms = 500, name = "Settle2"},
    -- Set 2: slightly more aggressive
    {vz = -2.0, duration_ms = 600, name = "Up2"},
    {vz = 0.0, duration_ms = 400, name = "Settle3"},
    {vz = 3.0, duration_ms = 1000, name = "Dn2"},
    {vz = -3.0, duration_ms = 800, name = "Brake2"},
    {vz = 0.0, duration_ms = 500, name = "Settle4"},
    -- Set 3: repeat with variation
    {vz = -2.5, duration_ms = 500, name = "Up3"},
    {vz = 0.0, duration_ms = 400, name = "Settle5"},
    {vz = 4.0, duration_ms = 1000, name = "Dn3"},
    {vz = -4.0, duration_ms = 800, name = "Brake3"},
    {vz = 0.0, duration_ms = 1500, name = "Hold2"},
}

-- Select appropriate step sequence based on hover throttle
local function get_hover_steps()
    local hover = MOT_THST_HOVER:get() or 0.5
    if hover < 0.25 then
        return HOVER_STEPS_GENTLE
    else
        return HOVER_STEPS_NORMAL
    end
end

-- Run hover test mode
local function run_hover_test()
    local now_ms = millis():tofloat()
    local hover_steps = get_hover_steps()

    -- Aggressiveness scaling: TLIN_ACCEL=5 is baseline (1.0x), TLIN_ACCEL=10 is 2x
    local accel_param = TLIN_ACCEL:get() or 5.0
    local aggr_scale = accel_param / 5.0

    if g_state.hover_step == 0 then
        g_state.hover_step = 1
        g_state.hover_step_start_ms = now_ms
        g_state.collect_start_ms = now_ms
        -- Show throttle range based on MOT_THST_HOVER
        local min_thr, max_thr = get_throttle_range()
        local hover = MOT_THST_HOVER:get() or 0.5
        gcs:send_text(MAV_SEVERITY.INFO, "TLIN: Starting hover test")
        if hover < 0.25 then
            gcs:send_text(MAV_SEVERITY.INFO, string.format("TLIN: hover=%.2f (gentle) accel=%.1f",
                          hover, accel_param))
        else
            gcs:send_text(MAV_SEVERITY.INFO, string.format("TLIN: hover=%.2f accel=%.1f",
                          hover, accel_param))
        end
    end

    -- Check if current step is complete
    local step = hover_steps[g_state.hover_step]
    if not step then
        return true  -- all steps complete
    end

    local step_elapsed = now_ms - g_state.hover_step_start_ms
    if step_elapsed >= step.duration_ms then
        g_state.hover_step = g_state.hover_step + 1
        g_state.hover_step_start_ms = now_ms
        step = hover_steps[g_state.hover_step]
        if not step then
            gcs:send_text(MAV_SEVERITY.INFO, "TLIN: Collecting done")
            validate_hover_compensation()
            return true  -- all steps complete
        end
        -- Show progress: step name, percentage, and current throttle/samples
        local pct = math.floor(g_state.hover_step * 100 / #hover_steps)
        gcs:send_text(MAV_SEVERITY.INFO, string.format("TLIN: %s %d%% t=%.2f n=%d",
                      step.name, pct, g_state.last_throttle, g_state.sample_count))
    end

    -- Scale velocity by aggressiveness factor
    local scaled_vz = step.vz * aggr_scale

    -- Command velocity with explicit acceleration for better response
    local vel = Vector3f()
    vel:x(0)
    vel:y(0)
    vel:z(scaled_vz)

    local accel = Vector3f()
    accel:x(0)
    accel:y(0)
    -- Vertical acceleration proportional to velocity change needed
    accel:z(accel_param * (scaled_vz > 0 and 1 or (scaled_vz < 0 and -1 or 0)))

    if not vehicle:set_target_velaccel_NED(vel, accel, false, 0, false, 0, false) then
        gcs:send_text(MAV_SEVERITY.WARNING, "TLIN: Velocity cmd failed")
    end

    -- Collect data
    local throttle = motors:get_throttle()
    local imu_accel = ins:get_accel(0)
    if throttle and imu_accel then
        add_data_point(throttle, imu_accel:z())
        g_state.last_throttle = throttle
    end

    -- Periodic diagnostic every 5 seconds - include relative altitude
    if now_ms - g_state.last_diag_ms > 5000 then
        g_state.last_diag_ms = now_ms
        local rel_alt = (baro:get_altitude() or 0) - g_state.start_alt_m
        gcs:send_text(MAV_SEVERITY.INFO, string.format("TLIN: thr=%.2f n=%d alt=%+.0fm",
                      g_state.last_throttle, g_state.sample_count, rel_alt))
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

    -- Get current velocity for diagnostics
    local cur_vel = ahrs:get_velocity_NED()
    local hspd = 0
    if cur_vel then
        hspd = math.sqrt(cur_vel:x() * cur_vel:x() + cur_vel:y() * cur_vel:y())
    end

    -- Periodic diagnostic (every 5 seconds)
    if now_ms - g_state.last_diag_ms > 5000 then
        g_state.last_diag_ms = now_ms
        local phase_names = {"ACCEL", "WAVE", "TURN"}
        local phase_name = phase_names[g_state.fwd_phase + 1] or "?"
        gcs:send_text(MAV_SEVERITY.INFO, string.format("TLIN: Fwd %s spd=%.1f dist=%.0f",
                      phase_name, hspd, dist))
    end

    -- Acceleration vector for velaccel control
    local accel = Vector3f()
    local fwd_accel = TLIN_ACCEL:get() or 5.0
    -- Vertical wave amplitude proportional to acceleration
    local wave_amp = fwd_accel * 0.3  -- ~1.5 m/s at 5 m/s^2 accel

    if g_state.fwd_phase == FWD_PHASE.ACCEL then
        -- Accelerate to cruise speed with explicit acceleration
        vel:x(test_spd * math.cos(g_state.fwd_heading_rad))
        vel:y(test_spd * math.sin(g_state.fwd_heading_rad))
        vel:z(0)
        -- Command acceleration in flight direction
        accel:x(fwd_accel * math.cos(g_state.fwd_heading_rad))
        accel:y(fwd_accel * math.sin(g_state.fwd_heading_rad))
        accel:z(0)

        if hspd > test_spd * 0.9 then
            gcs:send_text(MAV_SEVERITY.INFO, string.format("TLIN: Cruise at %.1f m/s", hspd))
            g_state.fwd_phase = FWD_PHASE.WAVE
            g_state.hover_step_start_ms = now_ms
        end

    elseif g_state.fwd_phase == FWD_PHASE.WAVE then
        -- Cruise with vertical oscillation for data collection
        vel:x(test_spd * math.cos(g_state.fwd_heading_rad))
        vel:y(test_spd * math.sin(g_state.fwd_heading_rad))

        -- Sine wave vertical motion
        local wave_period_ms = 4000
        local wave_phase = ((now_ms - g_state.hover_step_start_ms) % wave_period_ms) / wave_period_ms
        local vz = wave_amp * math.sin(wave_phase * 2 * math.pi)
        vel:z(vz)
        -- Zero horizontal accel during cruise, vertical accel for wave
        accel:x(0)
        accel:y(0)
        accel:z(wave_amp * 2 * math.pi / (wave_period_ms / 1000) * math.cos(wave_phase * 2 * math.pi))

        -- Check for turnaround (use 50% to leave room for braking)
        if dist > TLIN_DIST:get() * 0.5 then
            gcs:send_text(MAV_SEVERITY.INFO, string.format("TLIN: Turnaround %d at %.0fm",
                          g_state.turnaround_count + 1, dist))
            g_state.fwd_phase = FWD_PHASE.TURN
            g_state.turnaround_count = g_state.turnaround_count + 1
        end

    elseif g_state.fwd_phase == FWD_PHASE.TURN then
        -- Brake with explicit deceleration opposing current velocity
        vel:x(0)
        vel:y(0)
        vel:z(0)
        -- Command deceleration opposing current motion
        if cur_vel and hspd > 0.1 then
            local brake_scale = fwd_accel / hspd
            accel:x(-cur_vel:x() * brake_scale)
            accel:y(-cur_vel:y() * brake_scale)
        end
        accel:z(0)

        if hspd < 1.0 then
            -- Reverse heading
            gcs:send_text(MAV_SEVERITY.INFO, string.format("TLIN: Reversed, pass %d",
                          g_state.turnaround_count))
            g_state.fwd_heading_rad = g_state.fwd_heading_rad + math.pi
            g_state.fwd_phase = FWD_PHASE.ACCEL

            -- Check if we've done enough passes
            if g_state.turnaround_count >= 2 and have_sufficient_data() then
                return true  -- test complete
            end
        end
    end

    vehicle:set_target_velaccel_NED(vel, accel, false, 0, false, 0, false)

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
-- Reset step counters for next sub-test (keeps bin data)
local function reset_for_subtest()
    g_state.hover_step = 0
    g_state.hover_step_start_ms = 0
    g_state.collect_start_ms = 0
    g_state.fwd_phase = 0
    g_state.turnaround_count = 0
    -- Keep sample_count - accumulates across sub-tests
    -- Update start location/altitude for new sub-test
    g_state.start_alt_m = baro:get_altitude() or 0
    g_state.start_loc = ahrs:get_location()
end

local function start_test()
    if not check_preconditions() then
        -- In full mode waiting for GUIDED, don't set error
        if not g_state.waiting_for_guided then
            TLIN_STATE:set(STATE.ERROR)
        end
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
    g_state.full_subtest = 0
    g_state.waiting_for_guided = false

    -- In full mode, start with calibration
    if TLIN_MODE:get() == 3 then
        g_state.full_subtest = FULL_SUBTEST.CALIB
        gcs:send_text(MAV_SEVERITY.INFO, "TLIN: Full test - starting calibration")
    end

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
        if switch_pos == 2 then
            if g_state.last_switch_pos ~= 2 then
                -- Switch went high - start test
                if g_state.phase == PHASE.INIT or g_state.phase == PHASE.DONE then
                    g_state.phase = PHASE.INIT
                    start_test()
                end
            elseif g_state.waiting_for_guided then
                -- Keep retrying while waiting for GUIDED mode
                start_test()
            end
        elseif switch_pos == 0 and g_state.last_switch_pos ~= 0 then
            -- Switch went low during test - abort
            if g_state.phase ~= PHASE.INIT and g_state.phase ~= PHASE.DONE then
                abort_test("RC switch")
            end
            g_state.waiting_for_guided = false
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
        local mode = TLIN_MODE:get()

        if mode == 3 then
            -- Full test mode: run calibration -> hover -> forward in sequence
            if g_state.full_subtest == FULL_SUBTEST.CALIB then
                test_complete = run_calibration_test()
                if test_complete then
                    gcs:send_text(MAV_SEVERITY.INFO, "TLIN: Calibration done, starting hover test")
                    reset_for_subtest()
                    g_state.full_subtest = FULL_SUBTEST.HOVER
                    test_complete = false  -- Continue to next sub-test
                end
            elseif g_state.full_subtest == FULL_SUBTEST.HOVER then
                test_complete = run_hover_test()
                if test_complete then
                    gcs:send_text(MAV_SEVERITY.INFO, "TLIN: Hover done, starting forward test")
                    reset_for_subtest()
                    g_state.full_subtest = FULL_SUBTEST.FORWARD
                    test_complete = false  -- Continue to next sub-test
                end
            elseif g_state.full_subtest == FULL_SUBTEST.FORWARD then
                test_complete = run_forward_test()
                -- When forward completes, we're done
            end
        elseif mode == 0 then
            test_complete = run_hover_test()
        elseif mode == 1 then
            test_complete = run_forward_test()
        else
            test_complete = run_calibration_test()
        end

        if test_complete then
            g_state.phase = PHASE.SOLVE
        end

    elseif g_state.phase == PHASE.SOLVE then
        -- Check if this is calibration mode (mode 2) - just analyze spin_min
        local mode = TLIN_MODE:get()
        if mode == 2 then
            complete_spinmin_calibration()
            return update, 100
        end

        -- Solve is chunked - check if already in progress
        if g_solve_state.in_progress then
            -- Continue solving
            local done, expo = solve_expo_chunked()
            if done then
                -- Check if this was after rebinning (narrow range warning)
                local thr_range = g_observed_max_thr - g_observed_min_thr
                if thr_range < 0.05 then
                    gcs:send_text(MAV_SEVERITY.WARNING,
                        string.format("TLIN: Limited (%.1f%% range)", thr_range * 100))
                end
                complete_test(expo)
            end
            -- If not done, will continue on next update
        elseif g_rebin_state.complete then
            -- Rebinning finished, check data and start solve
            g_rebin_state.complete = false
            if have_sufficient_data() then
                gcs:send_text(MAV_SEVERITY.INFO, "TLIN: Solving...")
                solve_expo_chunked()  -- Start chunked solve
            else
                gcs:send_text(MAV_SEVERITY.WARNING,
                    string.format("TLIN: Range: %.3f-%.3f", g_observed_min_thr, g_observed_max_thr))
                abort_test("Insufficient throttle variation")
            end
        elseif have_sufficient_data() then
            -- Standard binning worked, start solve
            gcs:send_text(MAV_SEVERITY.INFO, "TLIN: Solving...")
            solve_expo_chunked()  -- Start chunked solve
        elseif not g_rebin_state.in_progress and g_rebin_state.index == 1 then
            -- Start dynamic binning with observed throttle range
            gcs:send_text(MAV_SEVERITY.INFO, "TLIN: Standard binning insufficient")
            local done, success = rebin_with_observed_range()
            if done and not success then
                abort_test("Throttle range too small")
            elseif done and success then
                -- Mark complete, start solve on next update
                g_rebin_state.complete = true
            end
        else
            -- Continue rebinning
            local done, success = rebin_with_observed_range()
            if done then
                if success then
                    g_rebin_state.complete = true
                else
                    gcs:send_text(MAV_SEVERITY.WARNING,
                        string.format("TLIN: Range: %.3f-%.3f", g_observed_min_thr, g_observed_max_thr))
                    abort_test("Insufficient throttle variation")
                end
            end
        end
    end

    -- Fast loop during active test or solve phase
    if g_state.phase == PHASE.COLLECT or g_state.phase == PHASE.STABILIZE or g_state.phase == PHASE.SOLVE then
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
