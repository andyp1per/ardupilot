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
  CLIMBING = 1,
  WAITING_BALLISTIC_ENTRY = 2,
  FLIPPING = 3,
  RESTORING = 4,
  RESTORING_WAIT = 5,
  DONE = 6,
}

--[[
  Starts a flip maneuver.
  Calculates the third parameter (rate, duration, or number of flips) based on the two provided.
  @param axis The axis of rotation (vehicle_control.axis.ROLL or vehicle_control.axis.PITCH).
  @param rate_degs (optional) The initial rotation rate in degrees/second.
  @param throttle_level The throttle level (0-1), or vehicle_control.THROTTLE_CUT to cut throttle.
  @param flip_duration_s (optional) The desired total duration of the maneuver.
  @param num_flips (optional) The number of flips to perform.
  @param slew_gain (optional) The proportional gain for rate slewing (default 0.5).
  @param climb_multiplier (optional) A factor to scale the initial climb rate to counteract drag (default 1.5).
  @return A state table for the perform_flip_update function, or nil and an error message.
]]
function vehicle_control.maneuver.flip_start(axis, rate_degs, throttle_level, flip_duration_s, num_flips, slew_gain, climb_multiplier)
  -- 1. Pre-flight Checks
  if not (vehicle:get_mode() == vehicle_control.mode.GUIDED) then
    gcs:send_text(vehicle_control.MAV_SEVERITY.WARNING, "Flip requires Guided mode")
    return nil, "Flip requires Guided mode"
  end

  -- 2. Calculate Flip Parameters
  local total_angle_deg

  -- Determine which parameters were provided by the user
  local user_has_flips = (num_flips ~= nil and num_flips > 0)
  local user_has_duration = (flip_duration_s ~= nil and flip_duration_s > 0)
  local user_has_rate = (rate_degs ~= nil and rate_degs ~= 0)

  if not user_has_flips and not user_has_duration and not user_has_rate then
      return nil, "Provide at least one of: rate, duration, or num_flips"
  end

  -- Logic to ensure a whole number of flips
  if user_has_flips then
      -- User specified the number of flips, this is the priority.
      total_angle_deg = 360 * num_flips
      if user_has_duration then
          -- Calculate rate
          rate_degs = total_angle_deg / flip_duration_s
      elseif user_has_rate then
          -- Calculate duration
          flip_duration_s = math.abs(total_angle_deg / rate_degs)
      else
          -- Neither duration nor rate specified, default duration to 1s per flip
          flip_duration_s = num_flips * 1.0
          rate_degs = total_angle_deg / flip_duration_s
      end
  elseif user_has_duration and user_has_rate then
      -- User specified duration and rate, num_flips must be calculated and rounded.
      -- We prioritize the duration and adjust the rate to complete a whole number of flips.
      local theoretical_flips = (math.abs(rate_degs) * flip_duration_s) / 360
      num_flips = math.floor(theoretical_flips + 0.5) -- round to nearest whole number
      if num_flips == 0 then num_flips = 1 end -- ensure at least one flip
      
      total_angle_deg = 360 * num_flips
      -- Recalculate rate to match the whole number of flips within the specified duration
      rate_degs = (total_angle_deg / flip_duration_s) * (rate_degs > 0 and 1 or -1)
  else
      -- This case handles if only duration or only rate is provided.
      -- We default to 1 flip.
      num_flips = 1
      total_angle_deg = 360
      if user_has_duration then
          rate_degs = total_angle_deg / flip_duration_s
      elseif user_has_rate then
          flip_duration_s = math.abs(total_angle_deg / rate_degs)
      end
  end

  -- Validate calculated parameters
  if rate_degs == 0 or flip_duration_s <= 0 then
      return nil, "Invalid flip parameters calculated (rate cannot be zero, duration must be positive)"
  end

  local t_flip = flip_duration_s

  -- 3. Prepare for Climb
  -- The initial velocity required for a total ballistic flight time of t_flip is v = 0.5 * g * t_flip
  local climb_rate_ms = 0.5 * 9.81 * t_flip
  local initial_velocity_ned = ahrs:get_velocity_NED()
  
  local target_climb_vel_ned = Vector3f()
  target_climb_vel_ned:x(initial_velocity_ned:x())
  target_climb_vel_ned:y(initial_velocity_ned:y())
  target_climb_vel_ned:z(-climb_rate_ms * (climb_multiplier or 1.5))
  
  -- 4. Prepare for Flip
  local initial_attitude_euler = Vector3f()
  initial_attitude_euler:x(ahrs:get_roll_rad())
  initial_attitude_euler:y(ahrs:get_pitch_rad())
  initial_attitude_euler:z(ahrs:get_yaw_rad())
  
  local initial_location = ahrs:get_location()
  local initial_pos_ned = ahrs:get_relative_position_NED_origin()
  if not initial_pos_ned then
    gcs:send_text(vehicle_control.MAV_SEVERITY.WARNING, "Could not get EKF origin position")
    return nil, "Could not get EKF origin position"
  end
  local initial_state = { attitude = initial_attitude_euler, velocity = initial_velocity_ned, location = initial_location, pos_ned = initial_pos_ned }

  local throttle_cmd = (throttle_level == vehicle_control.THROTTLE_CUT) and 0.0 or (throttle_level or 0.0)
  local initial_angle = (axis == vehicle_control.axis.ROLL) and math.deg(initial_state.attitude:x()) or math.deg(initial_state.attitude:y())

  return {
    stage = vehicle_control.maneuver.stage.CLIMBING,
    initial_state = initial_state,
    target_climb_vel_ned = target_climb_vel_ned,
    t_flip = t_flip,
    total_angle_deg = total_angle_deg,
    rate_degs = rate_degs,
    axis = axis,
    throttle_cmd = throttle_cmd,
    last_angle = initial_angle,
    accumulated_angle = 0,
    Kp = slew_gain or 0.5,
    num_flips = num_flips,
  }
end

--[[
  Updates the flip maneuver state machine.
  @param state The state table from flip_start.
  @return RUNNING or SUCCESS.
]]
function vehicle_control.maneuver.flip_update(state)
  if state.stage == vehicle_control.maneuver.stage.CLIMBING then
    -- Continuously command the target climb velocity
    vehicle:set_target_velocity_NED(state.target_climb_vel_ned)

    local current_vel_ned = ahrs:get_velocity_NED()
    if not current_vel_ned then return vehicle_control.RUNNING end

    -- Check if we have reached the desired climb rate (e.g., within 95%)
    local target_climb_rate = -state.target_climb_vel_ned:z()
    local current_climb_rate = -current_vel_ned:z()

    if current_climb_rate >= (target_climb_rate * 0.95) then
      -- Capture the state at the moment we enter the ballistic phase
      state.ballistic_entry_state = {
        location = ahrs:get_location(),
      }
      state.stage = vehicle_control.maneuver.stage.WAITING_BALLISTIC_ENTRY
    end
    return vehicle_control.RUNNING

  elseif state.stage == vehicle_control.maneuver.stage.WAITING_BALLISTIC_ENTRY then
    -- In this stage, the vehicle is coasting upwards. We continuously calculate the
    -- total hang time from the current state to determine the precise moment to start the flip.
    local current_vel_ned = ahrs:get_velocity_NED()
    local current_loc = ahrs:get_location()
    if not (current_vel_ned and current_loc) then return vehicle_control.RUNNING end

    local vz = -current_vel_ned:z() -- current upward velocity

    -- If we are no longer climbing, it's too late. Start the flip immediately.
    if vz <= 0.1 then
        state.stage = vehicle_control.maneuver.stage.FLIPPING
    else
        -- Calculate the total hang time from the current position and velocity
        -- h is the current altitude relative to the maneuver's start altitude
        local h = (current_loc:alt() - state.ballistic_entry_state.location:alt()) / 100.0

        -- Time to reach the apex from the current point
        local t_to_apex = vz / 9.81

        -- Additional altitude that will be gained to reach the apex
        local h_gain = (vz^2) / (2 * 9.81)

        -- The total altitude at the apex, relative to the start of the maneuver
        local h_apex = h + h_gain
        if h_apex < 0 then h_apex = 0 end

        -- Time to fall from the apex back down to the starting altitude
        local t_fall = math.sqrt(2 * h_apex / 9.81)

        -- Total hang time is the time to go up plus the time to fall down
        local t_hang = t_to_apex + t_fall

        if t_hang <= state.t_flip then
            state.stage = vehicle_control.maneuver.stage.FLIPPING
        end
    end

    if state.stage == vehicle_control.maneuver.stage.FLIPPING then
        local flip_msg = string.format("Flipping %.2f times over %.2f seconds", state.num_flips, state.t_flip)
        gcs:send_text(vehicle_control.MAV_SEVERITY.INFO, flip_msg)
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

    -- Check if the total rotation has been completed
    if math.abs(state.accumulated_angle) >= state.total_angle_deg then
        state.stage = vehicle_control.maneuver.stage.RESTORING
        return vehicle_control.RUNNING
    end

    -- Slew rate to match desired duration
    local elapsed_time = (millis():tofloat() - state.start_time) / 1000.0
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
    gcs:send_text(vehicle_control.MAV_SEVERITY.INFO, "Flip complete, restoring trajectory.")
    state.restore_start_time = millis():tofloat()
    state.stage = vehicle_control.maneuver.stage.RESTORING_WAIT
    return vehicle_control.RUNNING
    
  elseif state.stage == vehicle_control.maneuver.stage.RESTORING_WAIT then
    local current_pos_ned = ahrs:get_relative_position_NED_origin()
    local current_vel_ned = ahrs:get_velocity_NED()

    if not current_pos_ned or not current_vel_ned then
      return vehicle_control.RUNNING -- Wait for valid data
    end
    
    local elapsed_restore_time_s = (millis():tofloat() - state.restore_start_time) / 1000.0
    
    -- Calculate the ideal absolute target position vector, projecting it forward in time
    local total_elapsed_time_s = state.t_flip + elapsed_restore_time_s
    local displacement = state.initial_state.velocity:copy():scale(total_elapsed_time_s)
    local target_pos_ned_absolute = state.initial_state.pos_ned + displacement
    
    -- Continuously command the vehicle to the moving absolute target position and velocity
    vehicle:set_target_posvel_NED(target_pos_ned_absolute, state.initial_state.velocity)
    
    -- Debugging output at 200ms intervals
    state.last_debug_ms = state.last_debug_ms or 0
    local now_ms = millis():tofloat()
    if (now_ms - state.last_debug_ms) > 200 then
        state.last_debug_ms = now_ms
        -- Convert NED 'Down' to 'Up' for clarity in logging by multiplying by -1
        local target_p_up = -target_pos_ned_absolute:z()
        local curr_p_up = -current_pos_ned:z()
        local target_v_up = -state.initial_state.velocity:z()
        local curr_v_up = -current_vel_ned:z()
        
        local debug_msg = string.format("P Up T:%.1f C:%.1f | V Up T:%.1f C:%.1f", target_p_up, curr_p_up, target_v_up, curr_v_up)
        gcs:send_text(vehicle_control.MAV_SEVERITY.DEBUG, debug_msg)
    end

    -- Define separate tolerances for arrival check
    local pos_tolerance_m = 1.0
    local horizontal_vel_tolerance_ms = 0.5

    -- Check position error
    local pos_error_vec = target_pos_ned_absolute - current_pos_ned
    local pos_ok = pos_error_vec:length() < pos_tolerance_m

    -- Check horizontal velocity error
    local vel_error_x = state.initial_state.velocity:x() - current_vel_ned:x()
    local vel_error_y = state.initial_state.velocity:y() - current_vel_ned:y()
    local horizontal_vel_error_sq = vel_error_x^2 + vel_error_y^2
    local horizontal_vel_ok = horizontal_vel_error_sq < (horizontal_vel_tolerance_ms^2)

    -- Check vertical velocity error. The error is calculated as current_z - target_z.
    -- A positive error means descending faster or climbing slower than the target.
    local vel_error_z = current_vel_ned:z() - state.initial_state.velocity:z()
    -- This check ensures the vehicle is never descending faster than the target (vel_error_z <= 0).
    -- It allows a generous tolerance for "safe" errors (climbing faster/descending slower).
    local vertical_vel_ok = (vel_error_z <= 0) and (vel_error_z > -0.5)

    -- Check if all conditions are met
    if pos_ok and horizontal_vel_ok and vertical_vel_ok then
      gcs:send_text(vehicle_control.MAV_SEVERITY.INFO, "Trajectory restored.")
      state.stage = vehicle_control.maneuver.stage.DONE
      return vehicle_control.SUCCESS
    end

    return vehicle_control.RUNNING
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
