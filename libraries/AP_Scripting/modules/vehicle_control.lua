--[[
  vehicle_control.lua: A library for advanced vehicle control in ArduPilot Lua scripting.

  This library provides high-level functions for executing complex flight patterns
  and aerobatic maneuvers. Functions are designed to be re-entrant and are managed
  by a state machine from a parent script's update() loop.
]]

local vehicle_control = {}

-- Define status constants for state machine management
vehicle_control.RUNNING = 0
vehicle_control.SUCCESS = 1

-- Define a constant for the special throttle-cut value to improve readability
vehicle_control.THROTTLE_CUT = -1

-- Enum for flip axis
vehicle_control.axis = {
  ROLL = 1,
  PITCH = 2,
}

-- Enum for vehicle modes
vehicle_control.mode = {
  LOITER = 5,
  GUIDED = 4,
  RTL = 6,
}

-- Enum for MAV_SEVERITY levels, as required by the playbook
vehicle_control.MAV_SEVERITY = {
  EMERGENCY = 0,
  ALERT = 1,
  CRITICAL = 2,
  ERROR = 3,
  WARNING = 4,
  NOTICE = 5,
  INFO = 6,
  DEBUG = 7,
}

--================================================================
-- Pattern Control
--================================================================
vehicle_control.pattern = {}

--[[
  Initializes a flight pattern by performing pre-flight checks and calculating geometry.
  @param radius_m The radius for the pattern's circular elements.
  @return A table containing start_location, center_1, and center_2, or nil and an error message.
]]
function vehicle_control.pattern.start(radius_m)
  -- Precondition checks
  if not arming:is_armed() or not vehicle:get_likely_flying() then
    return nil, "Vehicle must be armed and flying"
  end
  local current_mode = vehicle:get_mode()
  if not (current_mode == vehicle_control.mode.LOITER or current_mode == vehicle_control.mode.GUIDED) then
    return nil, "Vehicle must be in Loiter or Guided mode"
  end
  local current_loc = ahrs:get_location()
  if not current_loc then
    return nil, "Vehicle position not available"
  end

  -- Set vehicle to Guided mode for pattern execution
  if not vehicle:set_mode(vehicle_control.mode.GUIDED) then
    return nil, "Failed to set mode to Guided"
  end

  -- Calculate pattern geometry
  local start_location = current_loc:copy()
  local heading_rad = ahrs:get_yaw_rad()

  local center_1 = start_location:copy()
  center_1:offset_bearing(math.deg(heading_rad) + 90, radius_m)

  local center_2 = start_location:copy()
  center_2:offset_bearing(math.deg(heading_rad) - 90, radius_m)

  return {
    start_location = start_location,
    center_1 = center_1,
    center_2 = center_2,
  }
end

--[[
  Starts flying a circular arc.
  @return A state table for the fly_arc_update function.
]]
function vehicle_control.pattern.fly_arc_start(center_loc, start_bearing_deg, end_bearing_deg, radius_m, speed_ms, direction)
  vehicle:set_desired_speed(speed_ms)
  local total_angle_deg = (end_bearing_deg - start_bearing_deg)
  if direction > 0 and total_angle_deg < 0 then
    total_angle_deg = total_angle_deg + 360
  elseif direction < 0 and total_angle_deg > 0 then
    total_angle_deg = total_angle_deg - 360
  end

  local arc_length = math.abs(math.rad(total_angle_deg)) * radius_m
  local duration_s = arc_length / speed_ms

  return {
    start_time = millis():tofloat(),
    duration_s = duration_s,
    center_loc = center_loc,
    start_bearing_deg = start_bearing_deg,
    total_angle_deg = total_angle_deg,
    radius_m = radius_m,
  }
end

--[[
  Updates the vehicle's position along a circular arc.
  @param state The state table from fly_arc_start.
  @return RUNNING or SUCCESS.
]]
function vehicle_control.pattern.fly_arc_update(state)
  local elapsed_time = (millis():tofloat() - state.start_time) / 1000.0
  if elapsed_time >= state.duration_s then
    return vehicle_control.SUCCESS
  end

  local progress = elapsed_time / state.duration_s
  local current_bearing_deg = state.start_bearing_deg + (state.total_angle_deg * progress)
  local target_loc = state.center_loc:copy()
  target_loc:offset_bearing(current_bearing_deg, state.radius_m)

  vehicle:set_target_location(target_loc)
  return vehicle_control.RUNNING
end


--================================================================
-- Advanced Maneuvers
--================================================================
vehicle_control.maneuver = {}

-- Enum for flip maneuver stages
vehicle_control.maneuver.stage = {
  ACHIEVING_CLIMB = 1,
  WAITING_BALLISTIC_ENTRY = 2,
  FLIPPING = 3,
  RESTORING = 4,
  DONE = 5,
}

--[[
  Starts a flip maneuver.
  @param axis The axis of rotation (vehicle_control.axis.ROLL or vehicle_control.axis.PITCH).
  @param rate_degs The initial rotation rate in degrees/second.
  @param throttle_level The throttle level (0-1), or vehicle_control.THROTTLE_CUT to cut throttle.
  @param flip_duration_s (optional) The desired total duration of the maneuver.
  @param num_flips (optional) The number of flips to perform (default 1).
  @param slew_gain (optional) The proportional gain for rate slewing (default 0.5).
  @param climb_multiplier (optional) A factor to scale the initial climb rate to counteract drag (default 1.5).
  @return A state table for the perform_flip_update function, or nil and an error message.
]]
function vehicle_control.maneuver.flip_start(axis, rate_degs, throttle_level, flip_duration_s, num_flips, slew_gain, climb_multiplier)
  -- 1. Save State & Prepare
  if not vehicle:get_mode() == vehicle_control.mode.GUIDED then
    gcs:send_text(vehicle_control.MAV_SEVERITY.WARNING, "Flip requires Guided mode")
    return nil, "Flip requires Guided mode"
  end

  -- Check for required parameters
  if type(rate_degs) ~= "number" or rate_degs == 0 then
    gcs:send_text(vehicle_control.MAV_SEVERITY.ERROR, "Flip requires a non-zero number for rate_degs")
    return nil, "Invalid rate_degs"
  end

  -- Default throttle_level to zero if not provided
  if throttle_level == nil then
    throttle_level = 0.0
  end

  num_flips = num_flips or 1
  climb_multiplier = climb_multiplier or 1.5
  local total_angle_deg = 360 * num_flips
  local t_flip = flip_duration_s or (total_angle_deg / math.abs(rate_degs))
  if t_flip <= 0 then
    gcs:send_text(vehicle_control.MAV_SEVERITY.WARNING, "Flip duration must be positive")
    return nil, "Flip duration must be positive"
  end

  -- 2. Calculate required climb rate and initiate climb
  local climb_rate_ms = 0.5 * 9.81 * t_flip
  local initial_velocity_ned = ahrs:get_velocity_NED()
  
  local vel_ned = Vector3f()
  vel_ned:x(initial_velocity_ned:x())
  vel_ned:y(initial_velocity_ned:y())
  vel_ned:z(-climb_rate_ms * climb_multiplier)
  
  if not vehicle:set_target_velocity_NED(vel_ned) then
    gcs:send_text(vehicle_control.MAV_SEVERITY.WARNING, "Failed to set target velocity for climb")
    return nil, "Failed to set climb velocity"
  end

  -- 3. Prepare for Flip
  local initial_attitude_euler = Vector3f()
  initial_attitude_euler:x(ahrs:get_roll_rad())
  initial_attitude_euler:y(ahrs:get_pitch_rad())
  initial_attitude_euler:z(ahrs:get_yaw_rad())
  
  local initial_location = ahrs:get_location()
  local initial_state = { attitude = initial_attitude_euler, velocity = initial_velocity_ned, location = initial_location }

  local throttle_cmd = (throttle_level == vehicle_control.THROTTLE_CUT) and 0.0 or throttle_level
  local initial_angle = (axis == vehicle_control.axis.ROLL) and math.deg(initial_state.attitude:x()) or math.deg(initial_state.attitude:y())

  return {
    stage = vehicle_control.maneuver.stage.ACHIEVING_CLIMB,
    commanded_climb_rate = climb_rate_ms * climb_multiplier,
    initial_state = initial_state,
    t_flip = t_flip,
    total_angle_deg = total_angle_deg,
    rate_degs = rate_degs,
    axis = axis,
    throttle_cmd = throttle_cmd,
    last_angle = initial_angle,
    accumulated_angle = 0,
    Kp = slew_gain or 0.5,
  }
end

--[[
  Updates the flip maneuver state machine.
  @param state The state table from flip_start.
  @return RUNNING or SUCCESS.
]]
function vehicle_control.maneuver.flip_update(state)
  if state.stage == vehicle_control.maneuver.stage.ACHIEVING_CLIMB then
    local current_vel_ned = ahrs:get_velocity_NED()
    if not current_vel_ned then return vehicle_control.RUNNING end
    
    -- Check if the actual climb rate is close to the commanded rate
    if math.abs(current_vel_ned:z() - (-state.commanded_climb_rate)) < 0.5 then -- 0.5 m/s tolerance
      state.stage = vehicle_control.maneuver.stage.WAITING_BALLISTIC_ENTRY
    end
    return vehicle_control.RUNNING

  elseif state.stage == vehicle_control.maneuver.stage.WAITING_BALLISTIC_ENTRY then
    local current_vel_ned = ahrs:get_velocity_NED()
    local current_loc = ahrs:get_location()
    if not (current_vel_ned and current_loc) then return vehicle_control.RUNNING end

    local vz = -current_vel_ned:z() -- upward velocity is positive
    if vz <= 0 then return vehicle_control.RUNNING end -- only proceed if we are climbing

    local alt_diff = (current_loc:alt() - state.initial_state.location:alt()) / 100.0
    
    local t_to_apex = vz / 9.81
    local h_gain = vz * t_to_apex - 0.5 * 9.81 * t_to_apex^2
    local h_apex = alt_diff + h_gain
    if h_apex < 0 then h_apex = 0 end -- prevent sqrt of negative number
    local t_fall = math.sqrt(2 * h_apex / 9.81)
    local t_hang = t_to_apex + t_fall

    if t_hang <= state.t_flip then
      state.stage = vehicle_control.maneuver.stage.FLIPPING
      state.start_time = millis():tofloat()
      
      -- Start the flip with the specified throttle
      local roll_rate_dps, pitch_rate_dps = 0, 0
      if state.axis == vehicle_control.axis.ROLL then
        roll_rate_dps = state.rate_degs
      else -- pitch
        pitch_rate_dps = state.rate_degs
      end
      vehicle:set_target_rate_and_throttle(roll_rate_dps, pitch_rate_dps, 0, state.throttle_cmd)
    end
    return vehicle_control.RUNNING

  elseif state.stage == vehicle_control.maneuver.stage.FLIPPING then
    local elapsed_time = (millis():tofloat() - state.start_time) / 1000.0
    if elapsed_time >= state.t_flip then
      state.stage = vehicle_control.maneuver.stage.RESTORING
      return vehicle_control.RUNNING
    end

    -- Unwrap angle to track total rotation
    local current_angle = (state.axis == vehicle_control.axis.ROLL) and math.deg(ahrs:get_roll_rad()) or math.deg(ahrs:get_pitch_rad())
    local delta_angle = current_angle - state.last_angle
    if delta_angle > 180 then
      delta_angle = delta_angle - 360
    elseif delta_angle < -180 then
      delta_angle = delta_angle + 360
    end
    state.accumulated_angle = state.accumulated_angle + delta_angle
    state.last_angle = current_angle

    -- Slew rate to match desired duration
    local expected_angle = (elapsed_time / state.t_flip) * state.total_angle_deg * (state.rate_degs > 0 and 1 or -1)
    local error = expected_angle - state.accumulated_angle
    local new_rate_degs = state.rate_degs + state.Kp * error

    -- Set target rates
    local roll_rate_dps, pitch_rate_dps = 0, 0
    if state.axis == vehicle_control.axis.ROLL then
      roll_rate_dps = new_rate_degs
    else -- pitch
      pitch_rate_dps = new_rate_degs
    end
    vehicle:set_target_rate_and_throttle(roll_rate_dps, pitch_rate_dps, 0, state.throttle_cmd)

    return vehicle_control.RUNNING

  elseif state.stage == vehicle_control.maneuver.stage.RESTORING then
    -- Restore the original FULL 3D velocity vector. The autopilot's controller
    -- will handle the attitude and throttle to achieve this trajectory,
    -- correcting for any accumulated position or altitude errors.
    vehicle:set_target_velocity_NED(state.initial_state.velocity)
    
    gcs:send_text(vehicle_control.MAV_SEVERITY.INFO, "Flip complete, resuming trajectory.")
    state.stage = vehicle_control.maneuver.stage.DONE
    return vehicle_control.SUCCESS
  end

  return vehicle_control.SUCCESS
end


--================================================================
-- Utility Functions
--================================================================
vehicle_control.utils = {}

--[[
  Checks if the vehicle has arrived at a target location.
  @param target_location The destination Location object.
  @param tolerance_m The arrival radius in meters.
  @return true if arrived, false otherwise.
]]
function vehicle_control.utils.has_arrived(target_location, tolerance_m)
  local current_loc = ahrs:get_location()
  if current_loc and current_loc:get_distance(target_location) < tolerance_m then
    return true
  end
  return false
end

return vehicle_control
