--[[
  flip_on_switch.lua: An applet to perform flips based on RC switch input.

  This script uses the vehicle_control module to execute a flip maneuver.
  It supports two modes via the FLIP_SPRING parameter:
  1. Standard Mode (FLIP_SPRING = 0):
     - Flick switch multiple times to set the number of flips.
     - Hold switch high to set the duration of the flip.
     - Maneuver starts when the switch is left high and is canceled if low.
  2. Spring-loaded Mode (FLIP_SPRING = 1):
     - Perform a flick or hold sequence (which ends with the switch low).
     - The maneuver starts automatically after a short timeout.
     - A subsequent flick cancels the active maneuver.
]]

local vehicle_control = require('vehicle_control')

-- Header Comment
-- This script allows a user to trigger a flip maneuver on a drone
-- using a 3-position RC switch with advanced control logic. The flip axis,
-- rate, and other parameters are configurable. The script ensures the drone
-- is in a safe state before starting the maneuver and maintains altitude.

-- Enum for MAV_SEVERITY levels
local MAV_SEVERITY = {
    EMERGENCY = 0,
    ALERT = 1,
    CRITICAL = 2,
    ERROR = 3,
    WARNING = 4,
    NOTICE = 5,
    INFO = 6,
    DEBUG = 7
}

-- Parameters
local PARAM_TABLE_KEY = 107
local PARAM_TABLE_PREFIX = "FLIP_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 8), 'flip_on_switch: could not add param table')

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

local FLIP_ENABLE = bind_add_param('ENABLE', 1, 1)
local FLIP_AXIS = bind_add_param('AXIS', 2, vehicle_control.axis.ROLL)
local FLIP_RATE = bind_add_param('RATE', 3, 720)
local FLIP_THROTTLE = bind_add_param('THROTTLE', 4, 0.0)
local FLIP_HOVER = bind_add_param('HOVER', 5, 0.125)
local FLIP_FLICK_TO = bind_add_param('FLICK_TO', 6, 0.5) -- Flick vs Hold timeout
local FLIP_COMMIT_TO = bind_add_param('COMMIT_TO', 7, 0.75) -- Commit timeout
local FLIP_SPRING = bind_add_param('SPRING', 8, 0) -- Spring-loaded switch mode

-- RC Function Constant
local SCRIPTING_AUX_FUNC = 300 -- Corresponds to "Scripting1"

-- State variables for the main maneuver
local flip_state = nil
local flip_active = false
local original_mode = nil

-- State variables for interpreting switch input
local switch_logic_state = {
    IDLE = 0,
    TIMING_HIGH = 1,
    WAITING_FOR_COMMIT = 2,
    COMMITTED = 3,
}
local current_switch_state = switch_logic_state.IDLE
local last_switch_pos = 0
local high_start_time = 0
local flick_count = 0
local last_flick_time = 0
local determined_num_flips = nil
local determined_duration_s = nil
local abort_cooldown_end_time = 0

-- Precondition checks
assert(FWVersion:type() == 2, 'Script requires a Copter frame')

-- Resets all state machines to a clean state
function reset_all_states(is_abort)
    if current_switch_state ~= switch_logic_state.IDLE then
        gcs:send_text(MAV_SEVERITY.DEBUG, "Switch Logic: Resetting to IDLE")
    end
    flip_active = false
    flip_state = nil
    current_switch_state = switch_logic_state.IDLE
    flick_count = 0
    high_start_time = 0
    last_flick_time = 0
    determined_num_flips = nil
    determined_duration_s = nil

    if is_abort then
        abort_cooldown_end_time = millis():tofloat() + 3000 -- 3 second cooldown
        gcs:send_text(MAV_SEVERITY.WARNING, "Maneuver aborted, cooldown active.")
    end

    -- Restore attitude and original flight mode if necessary
    if original_mode then
        vehicle:set_target_rate_and_throttle(0, 0, 0, 0.5)
        if vehicle:get_mode() ~= original_mode then
            vehicle:set_mode(original_mode)
        end
        original_mode = nil
    end
end

-- Function to start the flip maneuver once parameters are determined
function start_maneuver()
    -- Pre-flight checks
    if not arming:is_armed() or not vehicle:get_likely_flying() then
        gcs:send_text(MAV_SEVERITY.WARNING, "Flip: Vehicle must be armed and flying")
        reset_all_states(false)
        return
    end

    original_mode = vehicle:get_mode()
    if not (original_mode == vehicle_control.mode.LOITER or original_mode == vehicle_control.mode.GUIDED) then
        gcs:send_text(MAV_SEVERITY.WARNING, "Flip: Must be in Loiter or Guided mode to start")
        reset_all_states(false)
        return
    end

    -- Set to Guided mode for flip execution
    if original_mode ~= vehicle_control.mode.GUIDED then
        if not vehicle:set_mode(vehicle_control.mode.GUIDED) then
            gcs:send_text(MAV_SEVERITY.WARNING, "Flip: Failed to set Guided mode")
            reset_all_states(false)
            return
        end
    end

    local num_flips_arg = determined_num_flips
    local duration_s_arg = determined_duration_s
    
    if num_flips_arg then
        gcs:send_text(MAV_SEVERITY.INFO, string.format("Flip: Starting %d flips", num_flips_arg))
    elseif duration_s_arg then
        gcs:send_text(MAV_SEVERITY.INFO, string.format("Flip: Starting %.2fs flip", duration_s_arg))
    end

    flip_active = true
    flip_state = vehicle_control.maneuver.flip_start(
        FLIP_AXIS:get(),
        FLIP_RATE:get(),
        FLIP_THROTTLE:get(),
        duration_s_arg,
        num_flips_arg,
        nil,
        FLIP_HOVER:get()
    )
    if flip_state == nil then
        gcs:send_text(MAV_SEVERITY.WARNING, "Flip: Failed to start flip")
        reset_all_states(false)
    end
end

-- Handles switch logic for standard (non-spring-loaded) switches
function update_standard_mode(switch_pos, now)
    if current_switch_state == switch_logic_state.IDLE then
        if switch_pos == 2 and last_switch_pos ~= 2 then -- Rising edge to HIGH
            current_switch_state = switch_logic_state.TIMING_HIGH
            flick_count = 1
            gcs:send_text(MAV_SEVERITY.DEBUG, string.format("Switch Logic: -> TIMING_HIGH (Flick %d)", flick_count))
            high_start_time = now
        end
    elseif current_switch_state == switch_logic_state.TIMING_HIGH then
        local high_duration_s = (now - high_start_time) / 1000.0
        if switch_pos ~= 2 then -- Falling edge from HIGH
            current_switch_state = switch_logic_state.WAITING_FOR_COMMIT
            gcs:send_text(MAV_SEVERITY.DEBUG, "Switch Logic: -> WAITING_FOR_COMMIT")
            last_flick_time = now
        elseif high_duration_s >= FLIP_FLICK_TO:get() then
            gcs:send_text(MAV_SEVERITY.DEBUG, "Switch Logic: Hold detected -> COMMITTED")
            determined_duration_s = high_duration_s
            determined_num_flips = nil
            flick_count = 0
            current_switch_state = switch_logic_state.COMMITTED
        end
    elseif current_switch_state == switch_logic_state.WAITING_FOR_COMMIT then
        if switch_pos == 2 and last_switch_pos ~= 2 then -- Another flick detected
            flick_count = flick_count + 1
            current_switch_state = switch_logic_state.TIMING_HIGH
            gcs:send_text(MAV_SEVERITY.DEBUG, string.format("Switch Logic: -> TIMING_HIGH (Flick %d)", flick_count))
            high_start_time = now
        elseif (now - last_flick_time) / 1000.0 > FLIP_COMMIT_TO:get() then
            gcs:send_text(MAV_SEVERITY.DEBUG, "Switch Logic: Commit timeout -> COMMITTED")
            determined_num_flips = flick_count
            determined_duration_s = nil
            current_switch_state = switch_logic_state.COMMITTED
        end
    elseif current_switch_state == switch_logic_state.COMMITTED then
        if switch_pos == 2 then
            gcs:send_text(MAV_SEVERITY.DEBUG, "Switch Logic: Committed and high, starting maneuver")
            start_maneuver()
        else
            gcs:send_text(MAV_SEVERITY.DEBUG, "Switch Logic: Committed but not high, resetting")
            reset_all_states(false)
        end
    end
end

-- Handles switch logic for spring-loaded switches
function update_spring_mode(switch_pos, now)
    local rising_edge = (switch_pos == 2 and last_switch_pos ~= 2)
    local falling_edge = (switch_pos ~= 2 and last_switch_pos == 2)

    -- If a flip is active, a new flick cancels it
    if flip_active and falling_edge and (now - high_start_time)/1000.0 < FLIP_FLICK_TO:get() then
        gcs:send_text(MAV_SEVERITY.INFO, "Flip: Canceled by subsequent flick")
        reset_all_states(false)
        return
    end

    if current_switch_state == switch_logic_state.IDLE then
        if rising_edge then
            current_switch_state = switch_logic_state.TIMING_HIGH
            flick_count = 1
            gcs:send_text(MAV_SEVERITY.DEBUG, string.format("Spring Logic: -> TIMING_HIGH (Flick %d)", flick_count))
            high_start_time = now
        end
    elseif current_switch_state == switch_logic_state.TIMING_HIGH then
        if falling_edge then
            local high_duration_s = (now - high_start_time) / 1000.0
            if high_duration_s >= FLIP_FLICK_TO:get() then
                determined_duration_s = high_duration_s
                determined_num_flips = nil
                gcs:send_text(MAV_SEVERITY.DEBUG, "Spring Logic: Hold detected")
            else
                determined_num_flips = flick_count
                determined_duration_s = nil
                gcs:send_text(MAV_SEVERITY.DEBUG, "Spring Logic: Flick detected")
            end
            current_switch_state = switch_logic_state.WAITING_FOR_COMMIT
            last_flick_time = now
        end
    elseif current_switch_state == switch_logic_state.WAITING_FOR_COMMIT then
        if rising_edge then
            flick_count = flick_count + 1
            current_switch_state = switch_logic_state.TIMING_HIGH
            gcs:send_text(MAV_SEVERITY.DEBUG, string.format("Spring Logic: -> TIMING_HIGH (Flick %d)", flick_count))
            high_start_time = now
        elseif (now - last_flick_time) / 1000.0 > FLIP_COMMIT_TO:get() then
            gcs:send_text(MAV_SEVERITY.DEBUG, "Spring Logic: Commit timeout, starting maneuver")
            start_maneuver()
        end
    end
end

-- Main update function
function update()
    if FLIP_ENABLE:get() == 0 then
        return update, 1000
    end

    local switch_pos = rc:get_aux_cached(SCRIPTING_AUX_FUNC)
    local now = millis():tofloat()

    -- Check for abort cooldown
    if now < abort_cooldown_end_time then
        return update, 200
    end

    -- Global cancellation logic for standard mode
    if FLIP_SPRING:get() == 0 and switch_pos == 0 and flip_active then
        gcs:send_text(MAV_SEVERITY.INFO, "Flip: Canceled by user")
        reset_all_states(false)
        return update, 50
    end

    -- If a flip is active, update it
    if flip_active and flip_state then
        local status = vehicle_control.maneuver.flip_update(flip_state, reset_all_states)
        if status == vehicle_control.SUCCESS then
            if FLIP_SPRING:get() == 1 then
                reset_all_states(false)
            else
                start_maneuver()
            end
        elseif status == vehicle_control.ABORTED then
            -- Maneuver was aborted
        end
        return update, 10
    end

    -- Select logic based on spring-loaded parameter
    if FLIP_SPRING:get() == 1 then
        update_spring_mode(switch_pos, now)
    else
        update_standard_mode(switch_pos, now)
    end

    last_switch_pos = switch_pos
    return update, 50
end

-- Protected wrapper for error handling
function protected_wrapper()
    local success, err = pcall(update)
    if not success then
        gcs:send_text(MAV_SEVERITY.ERROR, "Flip Error: " .. err)
        reset_all_states(false)
        return protected_wrapper, 1000
    end
    return protected_wrapper, 50
end

gcs:send_text(MAV_SEVERITY.INFO, "Flip on switch script loaded (Advanced)")
return protected_wrapper, 1000
