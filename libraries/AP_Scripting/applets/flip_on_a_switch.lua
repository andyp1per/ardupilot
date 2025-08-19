--[[
  flip_on_switch.lua: An applet to perform continuous flips on a switch.

  This script uses the vehicle_control module to execute a flip maneuver
  repeatedly while an RC switch is held in the high position. The drone
  will maintain altitude during the flips.
]]

local vehicle_control = require('vehicle_control')

-- Header Comment
-- This script allows a user to trigger a continuous flip maneuver on a drone
-- using a 3-position RC switch. The flip axis, rate, and duration are
-- configurable via parameters. The script ensures the drone is in a safe
-- state before starting the maneuver and maintains altitude throughout.

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
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 5), 'flip_on_switch: could not add param table')

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

local FLIP_ENABLE = bind_add_param('ENABLE', 1, 1)
local FLIP_AXIS = bind_add_param('AXIS', 2, vehicle_control.axis.ROLL)
local FLIP_RATE = bind_add_param('RATE', 3, 720)
local FLIP_DURATION = bind_add_param('DURATION', 4, 1.0)
local FLIP_THROTTLE = bind_add_param('THROTTLE', 5, 0.5)


-- RC Function Constant
local SCRIPTING_AUX_FUNC = 300 -- Corresponds to "Scripting1"

-- State variables
local flip_state = nil
local flip_active = false
local original_mode = nil

-- Precondition checks
assert(FWVersion:type() == 2, 'Script requires a Copter frame')

-- Main update function
function update()
    if FLIP_ENABLE:get() == 0 then
        return update, 1000
    end

    local switch_pos = rc:get_aux_cached(SCRIPTING_AUX_FUNC)

    if switch_pos == 2 then -- High position
        if not flip_active then
            -- Pre-flight checks
            if not arming:is_armed() or not vehicle:get_likely_flying() then
                gcs:send_text(MAV_SEVERITY.WARNING, "Flip: Vehicle must be armed and flying")
                return update, 500
            end

            original_mode = vehicle:get_mode()
            if not (original_mode == vehicle_control.mode.LOITER or original_mode == vehicle_control.mode.GUIDED) then
                gcs:send_text(MAV_SEVERITY.WARNING, "Flip: Must be in Loiter or Guided mode to start")
                original_mode = nil
                return update, 500
            end

            -- Set to Guided mode for flip execution
            if original_mode ~= vehicle_control.mode.GUIDED then
                if not vehicle:set_mode(vehicle_control.mode.GUIDED) then
                    gcs:send_text(MAV_SEVERITY.WARNING, "Flip: Failed to set Guided mode")
                    original_mode = nil
                    return update, 500
                end
            end

            -- Start a new flip
            gcs:send_text(MAV_SEVERITY.INFO, "Flip: Starting continuous flip")
            flip_active = true
            flip_state = vehicle_control.maneuver.flip_start(
                FLIP_AXIS:get(),
                FLIP_RATE:get(),
                nil,
                FLIP_DURATION:get()
            )
            if flip_state == nil then
                gcs:send_text(MAV_SEVERITY.WARNING, "Flip: Failed to start flip")
                flip_active = false
                if original_mode then
                    vehicle:set_mode(original_mode)
                    original_mode = nil
                end
            end
        elseif flip_state then
            -- Update existing flip
            local status = vehicle_control.maneuver.flip_update(flip_state)
            if status == vehicle_control.SUCCESS then
                -- Flip finished, start another one immediately
                flip_state = vehicle_control.maneuver.flip_start(
                    FLIP_AXIS:get(),
                    FLIP_RATE:get(),
                    nil,
                    FLIP_DURATION:get()
                )
                if flip_state == nil then
                    gcs:send_text(MAV_SEVERITY.WARNING, "Flip: Failed to restart flip")
                    flip_active = false
                end
            end
            return update, 10
        end
    else -- Low or Middle position
        if flip_active then
            gcs:send_text(MAV_SEVERITY.INFO, "Flip: Stopping continuous flip")
            flip_active = false
            flip_state = nil
            -- Restore attitude by setting target rates to zero and providing a neutral hover throttle
            vehicle:set_target_rate_and_throttle(0, 0, 0, 0.5)
            -- Restore original flight mode
            if original_mode and vehicle:get_mode() ~= original_mode then
                vehicle:set_mode(original_mode)
            end
            original_mode = nil
        end
    end

    return update, 50 -- Run at 20Hz for responsiveness
end

-- Protected wrapper for error handling
function protected_wrapper()
    local success, err = pcall(update)
    if not success then
        gcs:send_text(MAV_SEVERITY.ERROR, "Flip Error: " .. err)
        return protected_wrapper, 1000 -- Reschedule with a longer delay after an error
    end
    return protected_wrapper, 50
end

gcs:send_text(MAV_SEVERITY.INFO, "Flip on switch script loaded")
return protected_wrapper, 1000
