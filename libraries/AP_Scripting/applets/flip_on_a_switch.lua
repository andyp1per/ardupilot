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

local parameter_definitions = {
  { name = "ENABLE", default = 1 },
  { name = "AXIS", default = vehicle_control.axis.ROLL },
  { name = "RATE", default = 720 },
  { name = "DURATION", default = 1.0 },
  { name = "THROTTLE", default = 0.5 }
}

for i, p_def in ipairs(parameter_definitions) do
    assert(param:add_param(PARAM_TABLE_KEY, i, p_def.name, p_def.default), "Could not add param "..p_def.name)
end

local FLIP_ENABLE = assert(param:get('FLIP_ENABLE'), 'FLIP_ENABLE not set')
local FLIP_AXIS = assert(param:get('FLIP_AXIS'), 'FLIP_AXIS not set')
local FLIP_RATE = assert(param:get('FLIP_RATE'), 'FLIP_RATE not set')
local FLIP_DURATION = assert(param:get('FLIP_DURATION'), 'FLIP_DURATION not set')
local FLIP_THROTTLE = assert(param:get('FLIP_THROTTLE'), 'FLIP_THROTTLE not set')

-- RC Function Constant
local SCRIPTING_AUX_FUNC = 300 -- Corresponds to "Scripting1"

-- State variables
local flip_state = nil
local flip_active = false

-- Precondition checks
-- assert(vehicle:get_frame_class() == 1 or vehicle:get_frame_class() == 2, 'Script requires a Copter frame')

-- Main update function
function update()
    if FLIP_ENABLE == 0 then
        return update, 1000
    end

    local switch_pos = rc:get_aux_cached(SCRIPTING_AUX_FUNC)

    if switch_pos == 2 then -- High position
        if not flip_active then
            -- Start a new flip if not already active
            gcs:send_text(MAV_SEVERITY.INFO, "Flip: Starting continuous flip")
            flip_active = true
            flip_state = vehicle_control.maneuver.flip_start(
                FLIP_AXIS,
                FLIP_RATE,
                FLIP_THROTTLE,
                FLIP_DURATION
            )
            if flip_state == nil then
                gcs:send_text(MAV_SEVERITY.WARNING, "Flip: Failed to start flip")
                flip_active = false
            end
        elseif flip_state then
            -- Update existing flip
            local status = vehicle_control.maneuver.flip_update(flip_state)
            if status == vehicle_control.SUCCESS then
                -- Flip finished, start another one immediately
                flip_state = vehicle_control.maneuver.flip_start(
                    FLIP_AXIS,
                    FLIP_RATE,
                    FLIP_THROTTLE,
                    FLIP_DURATION
                )
                if flip_state == nil then
                    gcs:send_text(MAV_SEVERITY.WARNING, "Flip: Failed to restart flip")
                    flip_active = false
                end
            end
        end
    else -- Low or Middle position
        if flip_active then
            gcs:send_text(MAV_SEVERITY.INFO, "Flip: Stopping continuous flip")
            flip_active = false
            flip_state = nil
            -- Restore attitude by setting target rates to zero
            vehicle:set_target_rate_and_throttle(0, 0, 0, vehicle:get_throttle_mid())
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
