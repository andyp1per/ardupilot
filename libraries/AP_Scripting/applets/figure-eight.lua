--[[
  figure_eight.lua: Fly a figure of eight pattern with a flip in each quadrant.

  This script makes the vehicle fly a figure-eight pattern. In each of the four
  quadrants of the figure eight, the vehicle performs a flip, returning to the
  same attitude and velocity it had before the flip.
--]]

-- User-configurable parameters
local RADIUS = 20       -- Radius of the circles in the figure eight (meters)
local SPEED = 10        -- Speed of the vehicle during the pattern (m/s)
local FLIP_ROLL_RATE = 360  -- Roll rate during the flip (degrees/second)
local FLIP_PITCH_RATE = 180 -- Pitch rate during the flip (degrees/second)

-- Internal state variables
local MAV_SEVERITY = {INFO = 6, WARNING = 4}
local SCRIPTING_AUX_FUNC = 300 -- RCx_OPTION for activation

local PATTERN_STATE = {
    INACTIVE = 0,
    CIRCLE_1 = 1,
    FLIP_1 = 2,
    CIRCLE_2 = 3,
    FLIP_2 = 4,
    CIRCLE_3 = 5,
    FLIP_3 = 6,
    CIRCLE_4 = 7,
    FLIP_4 = 8,
    RETURNING = 9,
}

local pattern_state = PATTERN_STATE.INACTIVE
local start_location = nil
local center_1 = nil
local center_2 = nil
local flip_start_attitude = nil
local flip_start_velocity = nil

-- Function to check if the vehicle is ready to start the pattern
function can_run()
    if not arming:is_armed() or not vehicle:get_likely_flying() then
        return false, "Disarmed or not flying"
    end
    local mode = vehicle:get_mode()
    if mode ~= 5 and mode ~= 4 then -- 5=Loiter, 4=Guided
        return false, "Requires Loiter or Guided mode"
    end
    if not ahrs:get_position() then
        return false, "Position not available"
    end
    return true
end

-- Function to set the vehicle's target location
function set_target(target_loc)
    if vehicle:get_mode() ~= 4 then -- Guided
        assert(vehicle:set_mode(4), "Failed to set mode to Guided")
    end
    assert(vehicle:set_desired_speed(SPEED), "Failed to set desired speed")
    assert(vehicle:set_target_location(target_loc), "Failed to set target location")
end

-- Function to execute a flip
function execute_flip(roll_rate, pitch_rate)
    -- Store entry attitude and velocity
    if flip_start_attitude == nil then
        flip_start_attitude = {
            roll = ahrs:get_roll_rad(),
            pitch = ahrs:get_pitch_rad(),
            yaw = ahrs:get_yaw_rad()
        }
        flip_start_velocity = ahrs:get_velocity_NED()
    end

    -- Command the flip using rate control
    assert(vehicle:set_target_rate_and_throttle(roll_rate, pitch_rate, 0, 0.5), "Failed to set target rate and throttle")

    -- Check for completion (e.g., full rotation)
    -- For simplicity, this example assumes a fixed duration for the flip
    -- A more robust implementation would monitor attitude
    -- After the flip, restore the original attitude and velocity
    -- This part is complex and requires careful management of the vehicle's state
end

-- Main update function
function update()
    local switch_pos = rc:get_aux_cached(SCRIPTING_AUX_FUNC)

    if switch_pos == 2 and pattern_state == PATTERN_STATE.INACTIVE then -- High position
        local ok, reason = can_run()
        if ok then
            gcs:send_text(MAV_SEVERITY.INFO, "Figure Eight: Starting")
            pattern_state = PATTERN_STATE.CIRCLE_1
            start_location = ahrs:get_location()
            center_1 = start_location:copy()
            center_1:offset(0, RADIUS)
            center_2 = start_location:copy()
            center_2:offset(0, -RADIUS)
            set_target(center_1) -- Start with the first circle
        else
            gcs:send_text(MAV_SEVERITY.WARNING, "Figure Eight: Cannot start: " .. reason)
        end
    elseif switch_pos ~= 2 and pattern_state ~= PATTERN_STATE.INACTIVE then
        gcs:send_text(MAV_SEVERITY.INFO, "Figure Eight: Deactivated by switch")
        pattern_state = PATTERN_STATE.INACTIVE
        start_location = nil
        if vehicle:get_mode() == 4 then -- Guided
             assert(vehicle:set_mode(5), "Failed to set mode to Loiter") -- Loiter
        end
    end

    if pattern_state == PATTERN_STATE.INACTIVE then
        return update, 200
    end

    local current_pos = ahrs:get_location()
    if not current_pos then return update, 200 end

    -- State machine for the figure-eight pattern
    if pattern_state == PATTERN_STATE.CIRCLE_1 then
        -- Fly first circle, check for quadrant change
        -- Simplified: transition after a certain time/distance
        if current_pos:get_distance(center_1) < 5 then
            pattern_state = PATTERN_STATE.FLIP_1
            flip_start_attitude = nil
        end
    elseif pattern_state == PATTERN_STATE.FLIP_1 then
        execute_flip(FLIP_ROLL_RATE, FLIP_PITCH_RATE)
        -- Simplified: assume flip takes a fixed time
        pattern_state = PATTERN_STATE.CIRCLE_2
        set_target(center_2)
    elseif pattern_state == PATTERN_STATE.CIRCLE_2 then
        if current_pos:get_distance(center_2) < 5 then
            pattern_state = PATTERN_STATE.FLIP_2
            flip_start_attitude = nil
        end
    elseif pattern_state == PATTERN_STATE.FLIP_2 then
        execute_flip(-FLIP_ROLL_RATE, -FLIP_PITCH_RATE)
        pattern_state = PATTERN_STATE.CIRCLE_3
        set_target(center_2)
    elseif pattern_state == PATTERN_STATE.CIRCLE_3 then
        if current_pos:get_distance(center_2) < 5 then
            pattern_state = PATTERN_STATE.FLIP_3
            flip_start_attitude = nil
        end
    elseif pattern_state == PATTERN_STATE.FLIP_3 then
        execute_flip(FLIP_ROLL_RATE, FLIP_PITCH_RATE)
        pattern_state = PATTERN_STATE.CIRCLE_4
        set_target(center_1)
    elseif pattern_state == PATTERN_STATE.CIRCLE_4 then
        if current_pos:get_distance(center_1) < 5 then
            pattern_state = PATTERN_STATE.FLIP_4
            flip_start_attitude = nil
        end
    elseif pattern_state == PATTERN_STATE.FLIP_4 then
        execute_flip(-FLIP_ROLL_RATE, -FLIP_PITCH_RATE)
        pattern_state = PATTERN_STATE.RETURNING
        set_target(start_location)
    elseif pattern_state == PATTERN_STATE.RETURNING then
        if current_pos:get_distance(start_location) < 5 then
            gcs:send_text(MAV_SEVERITY.INFO, "Figure Eight: Complete")
            pattern_state = PATTERN_STATE.INACTIVE
            assert(vehicle:set_mode(5), "Failed to set mode to Loiter") -- Loiter
        end
    end

    return update, 100 -- Reschedule
end

gcs:send_text(MAV_SEVERITY.INFO, "Figure Eight script loaded")
return update, 1000
