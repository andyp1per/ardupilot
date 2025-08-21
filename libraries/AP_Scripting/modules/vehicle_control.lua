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
  MANUAL_CLIMB = 1,
  FLIPPING = 2,
  LEVEL_VEHICLE = 3,
  MANUAL_BRAKE = 4,
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
  @param true_hover_throttle (optional) The true hover throttle of the vehicle (0-1). Defaults to MOT_THST_HOVER parameter.
  @return A state table for the perform_flip_update function, or nil and an error message.
]]
function vehicle_control.maneuver.flip_start(axis, rate_degs, throttle_level, flip_duration_s, num_flips, slew_gain, true_hover_throttle)
  -- 1. Pre-flight Checks
  if not (vehicle:get_mode() == vehicle_control.mode.GUIDED) then
    gcs:send_text(vehicle_control.MAV_SEVERITY.WARNING, "Flip requires Guided mode")
    return nil, "Flip requires Guided mode"
  end
  
  -- Use provided true hover throttle, or fall back to the parameter
  local hover_throttle
  if true_hover_throttle ~= nil and true_hover_throttle > 0 and true_hover_throttle < 1 then
    hover_throttle = true_hover_throttle
  else
    hover_throttle = param:get('MOT_THST_HOVER')
  end

  if not hover_throttle or hover_throttle <= 0 or hover_throttle >= 1 then
      return nil, "Valid hover throttle must be available"
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
      total_angle_deg = 360 * num_flips
      if user_has_duration then
          rate_degs = total_angle_deg / flip_duration_s
      elseif user_has_rate then
          -- flip_duration_s will be calculated later based on acceleration
      else
          -- Default duration if only num_flips is provided
          flip_duration_s = num_flips * 1.0 
          rate_degs = total_angle_deg / flip_duration_s
      end
  elseif user_has_duration and user_has_rate then
      local theoretical_flips = (math.abs(rate_degs) * flip_duration_s) / 360
      num_flips = math.floor(theoretical_flips + 0.5)
      if num_flips == 0 then num_flips = 1 end
      total_angle_deg = 360 * num_flips
      rate_degs = (total_angle_deg / flip_duration_s) * (rate_degs > 0 and 1 or -1)
  else
      num_flips = 1
      total_angle_deg = 360
      if user_has_duration then
          rate_degs = total_angle_deg / flip_duration_s
      elseif user_has_rate then
          -- flip_duration_s will be calculated later
      end
  end

  if rate_degs == 0 then
      return nil, "Invalid flip rate calculated"
  end

  -- 3. Calculate True Flip Duration (t_flip) including acceleration
  local accel_param_name = (axis == vehicle_control.axis.ROLL) and 'ATC_ACCEL_R_MAX' or 'ATC_ACCEL_P_MAX'
  local accel_max_cdegs2 = param:get(accel_param_name)
  if not accel_max_cdegs2 or accel_max_cdegs2 <= 0 then
    return nil, "Could not get valid ATC_ACCEL_*_MAX parameter"
  end
  local accel_max_degs2 = accel_max_cdegs2 / 100.0

  local time_to_reach_rate = math.abs(rate_degs) / accel_max_degs2
  local angle_during_accel = 0.5 * accel_max_degs2 * time_to_reach_rate^2
  
  local t_flip
  if 2 * angle_during_accel >= total_angle_deg then
    -- Maneuver is purely acceleration and deceleration (bang-bang)
    t_flip = 2 * math.sqrt(total_angle_deg / accel_max_degs2)
  else
    -- Trapezoidal profile (accel, const vel, decel)
    local angle_at_const_vel = total_angle_deg - (2 * angle_during_accel)
    local time_at_const_vel = angle_at_const_vel / math.abs(rate_degs)
    t_flip = (2 * time_to_reach_rate) + time_at_const_vel
  end

  -- 4. Calculate Manual Climb Parameters using the new t_flip
  local effective_gravity = 9.81 * (1 + (throttle_level or 0.0))
  local required_vz = 0.5 * effective_gravity * t_flip
  
  -- Estimate acceleration from throttle. 2*hover_throttle gives ~1g of acceleration.
  local climb_accel = 9.81 
  local climb_throttle = 2 * hover_throttle
  
  -- Time needed to accelerate to the required vertical velocity
  local t_accel = required_vz / climb_accel

  -- 5. Prepare for Flip
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
  local initial_state = { attitude = initial_attitude_euler, velocity = ahrs:get_velocity_NED(), location = initial_location, pos_ned = initial_pos_ned }

  local throttle_cmd = (throttle_level == vehicle_control.THROTTLE_CUT) and 0.0 or (throttle_level or 0.0)
  local initial_angle = (axis == vehicle_control.axis.ROLL) and math.deg(initial_state.attitude:x()) or math.deg(initial_state.attitude:y())

  return {
    stage = vehicle_control.maneuver.stage.MANUAL_CLIMB,
    initial_state = initial_state,
    t_accel = t_accel,
    climb_throttle = climb_throttle,
    t_flip = t_flip,
    total_angle_deg = total_angle_deg,
    rate_degs = rate_degs,
    axis = axis,
    throttle_cmd = throttle_cmd,
    last_angle = initial_angle,
    accumulated_angle = 0,
    Kp = slew_gain or 0.5,
    num_flips = num_flips,
    hover_throttle = hover_throttle,
  }
end

--[[
  Updates the flip maneuver state machine.
  @param state The state table from flip_start.
  @return RUNNING or SUCCESS.
]]
function vehicle_control.maneuver.flip_update(state)
  -- Get initial attitude in degrees for use in multiple stages
  local initial_roll_deg = math.deg(state.initial_state.attitude:x())
  local initial_pitch_deg = math.deg(state.initial_state.attitude:y())
  local initial_yaw_deg = math.deg(state.initial_state.attitude:z())

  if state.stage == vehicle_control.maneuver.stage.MANUAL_CLIMB then
    -- Stage 1: MANUAL_CLIMB
    -- Apply a strong upward thrust for a calculated duration to gain the
    -- required vertical velocity for the ballistic phase (the flip).
    if not state.start_time then
        state.start_time = millis():tofloat()
    end
    
    -- Command initial attitude with high throttle
    vehicle:set_target_angle_and_rate_and_throttle(initial_roll_deg, initial_pitch_deg, initial_yaw_deg, 0, 0, 0, state.climb_throttle)

    -- Debugging output at 200ms intervals
    state.last_debug_ms = state.last_debug_ms or 0
    local now_ms = millis():tofloat()
    if (now_ms - state.last_debug_ms) > 200 then
        state.last_debug_ms = now_ms
        local elapsed_time = (now_ms - state.start_time) / 1000.0
        local vz = -ahrs:get_velocity_NED():z()
        local debug_msg = string.format("Climb T:%.2f/%.2f s | VUp:%.1f m/s", elapsed_time, state.t_accel, vz)
        gcs:send_text(vehicle_control.MAV_SEVERITY.DEBUG, debug_msg)
    end

    local elapsed_time = (millis():tofloat() - state.start_time) / 1000.0
    if elapsed_time >= state.t_accel then
      local flip_msg = string.format("Flipping %.2f times over %.2f seconds", state.num_flips, state.t_flip)
      gcs:send_text(vehicle_control.MAV_SEVERITY.INFO, flip_msg)
      state.stage = vehicle_control.maneuver.stage.FLIPPING
      state.start_time = millis():tofloat()
      state.apex_location = ahrs:get_location() -- Record altitude at the start of the flip
      
      -- Calculate the predicted drop from apex based on total flip time and effective gravity
      local effective_gravity = 9.81 * (1 + state.throttle_cmd)
      state.predicted_drop_m = 0.5 * effective_gravity * state.t_flip^2

      -- Start the flip with the specified throttle and rates
      local roll_rate_dps, pitch_rate_dps = 0, 0
      if state.axis == vehicle_control.axis.ROLL then
          roll_rate_dps = state.rate_degs
      else
          pitch_rate_dps = state.rate_degs
      end
      vehicle:set_target_rate_and_throttle(roll_rate_dps, pitch_rate_dps, 0, state.throttle_cmd)
    end
    return vehicle_control.RUNNING

  elseif state.stage == vehicle_control.maneuver.stage.FLIPPING then
    -- Stage 2: FLIPPING
    -- The vehicle is now in its ballistic phase. This stage commands the
    -- desired rotation rate and monitors the accumulated angle. Once the
    -- rotation is complete, it transitions immediately to the leveling stage.
    
    -- Unwrap angle to track total rotation
    local current_angle = (state.axis == vehicle_control.axis.ROLL) and math.deg(ahrs:get_roll_rad()) or math.deg(ahrs:get_pitch_rad())
    local delta_angle = current_angle - state.last_angle
    if delta_angle > 180 then delta_angle = delta_angle - 360 elseif delta_angle < -180 then delta_angle = delta_angle + 360 end
    state.accumulated_angle = state.accumulated_angle + delta_angle
    state.last_angle = current_angle

    -- Check if the total rotation has been completed
    if math.abs(state.accumulated_angle) >= state.total_angle_deg then
        -- The required rotation has been achieved. Immediately transition to the
        -- LEVEL_VEHICLE stage on the next update loop. This ensures a clean
        -- hand-off and prevents this stage from sending any further rate commands.
        gcs:send_text(vehicle_control.MAV_SEVERITY.INFO, "Flip rotation complete, leveling.")
        state.stage = vehicle_control.maneuver.stage.LEVEL_VEHICLE
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
    else
      pitch_rate_dps = new_rate_degs
    end
    vehicle:set_target_rate_and_throttle(roll_rate_dps, pitch_rate_dps, 0, state.throttle_cmd)
    return vehicle_control.RUNNING

  elseif state.stage == vehicle_control.maneuver.stage.LEVEL_VEHICLE then
    -- Stage 3: LEVEL_VEHICLE
    -- After the flip, the vehicle is commanded to its original attitude. This stage
    -- waits for the vehicle to both stabilize its attitude and fall to the
    -- predicted altitude before applying the braking thrust. This ensures
    -- thrust is not applied in the wrong direction and the maneuver remains symmetrical.
    
    -- Command the vehicle to its original attitude with the maneuver's throttle command
    vehicle:set_target_angle_and_rate_and_throttle(initial_roll_deg, initial_pitch_deg, initial_yaw_deg, 0, 0, 0, state.throttle_cmd)

    -- Check 1: Is the vehicle level (within tolerance of its original attitude)?
    local roll_rad = ahrs:get_roll_rad()
    local pitch_rad = ahrs:get_pitch_rad()
    local is_level = math.abs(roll_rad - state.initial_state.attitude:x()) < math.rad(5) and math.abs(pitch_rad - state.initial_state.attitude:y()) < math.rad(5)

    -- Check 2: Has the vehicle dropped to the predicted altitude?
    local current_loc = ahrs:get_location()
    local actual_drop_m = 0
    if current_loc and state.apex_location then
      actual_drop_m = (state.apex_location:alt() - current_loc:alt()) / 100.0
    end
    local has_dropped = actual_drop_m >= (state.predicted_drop_m * 0.8) -- 80% tolerance

    -- Debugging output at 200ms intervals
    state.last_debug_ms = state.last_debug_ms or 0
    local now_ms = millis():tofloat()
    if (now_ms - state.last_debug_ms) > 200 then
        state.last_debug_ms = now_ms
        local debug_msg = string.format("Leveling... Drop P:%.1f A:%.1f | Level:%s", state.predicted_drop_m, actual_drop_m, tostring(is_level))
        gcs:send_text(vehicle_control.MAV_SEVERITY.DEBUG, debug_msg)
    end

    -- If both conditions are met, proceed to the braking stage
    if is_level and has_dropped then
        gcs:send_text(vehicle_control.MAV_SEVERITY.INFO, "Leveled at altitude, applying brake.")
        state.stage = vehicle_control.maneuver.stage.MANUAL_BRAKE
        state.start_time = millis():tofloat()
    end
    
    return vehicle_control.RUNNING

  elseif state.stage == vehicle_control.maneuver.stage.MANUAL_BRAKE then
    -- Stage 4: MANUAL_BRAKE
    -- This stage applies the symmetrical braking thrust. It commands a high
    -- throttle for the same duration as the initial climb. It exits early
    -- if the vehicle's vertical velocity is already restored, or as a backup,
    -- when the timer expires.
    
    -- Apply the same strong upward thrust for the same duration to symmetrically cancel the initial climb.
    vehicle:set_target_angle_and_rate_and_throttle(initial_roll_deg, initial_pitch_deg, initial_yaw_deg, 0, 0, 0, state.climb_throttle)

    -- Check if we have already recovered our initial vertical velocity
    local current_vel_ned = ahrs:get_velocity_NED()
    local brake_finished = false
    if current_vel_ned then
        if -current_vel_ned:z() >= -state.initial_state.velocity:z() then
            gcs:send_text(vehicle_control.MAV_SEVERITY.INFO, "Brake complete (velocity met), restoring trajectory.")
            brake_finished = true
        end
    end

    -- Debugging output at 200ms intervals
    state.last_debug_ms = state.last_debug_ms or 0
    local now_ms = millis():tofloat()
    if (now_ms - state.last_debug_ms) > 200 then
        state.last_debug_ms = now_ms
        local elapsed_time = (now_ms - state.start_time) / 1000.0
        local vz = -ahrs:get_velocity_NED():z()
        local debug_msg = string.format("Brake T:%.2f/%.2f s | VUp:%.1f m/s", elapsed_time, state.t_accel, vz)
        gcs:send_text(vehicle_control.MAV_SEVERITY.DEBUG, debug_msg)
    end

    -- Check if the timer has expired (backup)
    local elapsed_time = (millis():tofloat() - state.start_time) / 1000.0
    if not brake_finished and elapsed_time >= state.t_accel then
        gcs:send_text(vehicle_control.MAV_SEVERITY.INFO, "Brake complete (timer expired), restoring trajectory.")
        brake_finished = true
    end
    
    if brake_finished then
        state.restore_start_time = millis():tofloat()
        state.stage = vehicle_control.maneuver.stage.RESTORING_WAIT
        
        -- Immediately command zero throttle to arrest the climb before handing off to the position controller.
        vehicle:set_target_rate_and_throttle(0, 0, 0, 0)
    end
    return vehicle_control.RUNNING
    
  elseif state.stage == vehicle_control.maneuver.stage.RESTORING_WAIT then
    -- Stage 5: RESTORING_WAIT
    -- The vehicle is now stable and near its original flight path. This
    -- final stage hands control back to the autopilot's position controller
    -- to make the final small corrections to rejoin the trajectory perfectly.
    
    local current_pos_ned = ahrs:get_relative_position_NED_origin()
    local current_vel_ned = ahrs:get_velocity_NED()

    if not current_pos_ned or not current_vel_ned then
      return vehicle_control.RUNNING
    end
    
    -- Calculate the ideal absolute target position vector, projecting it forward in time
    local elapsed_restore_time_s = (millis():tofloat() - state.restore_start_time) / 1000.0
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
