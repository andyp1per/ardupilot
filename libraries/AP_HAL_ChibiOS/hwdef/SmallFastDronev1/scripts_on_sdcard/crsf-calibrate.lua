-- crsf-calibrate.lua
-- A self-contained script to perform ArduPilot calibrations via a CRSF menu.
-- This version uses the dynamic crsf_helper library for safe, modular operation
-- and includes full, stateful compass calibration functionality.

local crsf_helper = require('crsf_helper')

-- MAVLink constants
local MAV_SEVERITY = {INFO = 6, WARNING = 4, ERROR = 3}
local MAV_CMD_PREFLIGHT_CALIBRATION = 241
local MAV_CMD_DO_START_MAG_CAL = 42424
local MAV_CMD_DO_ACCEPT_MAG_CAL = 42425
local MAV_CMD_DO_CANCEL_MAG_CAL = 42426

-- State variable to track if compass calibration is in progress
local compass_calibration_running = false

-- ####################
-- # CALLBACK FUNCTIONS
-- ####################

-- Called when "Start Compass" is selected
local function on_compass_start(value)
    if not value then return end

    if compass_calibration_running then
        gcs:run_command_int(MAV_CMD_DO_CANCEL_MAG_CAL, { p3 = 1 })
        gcs:send_text(MAV_SEVERITY.WARNING, "Previous compass cal cancelled. Starting new one.")
    end
    
    compass_calibration_running = true
    gcs:run_command_int(MAV_CMD_DO_START_MAG_CAL, { p3 = 1 })
    gcs:send_text(MAV_SEVERITY.INFO, "Compass calibration running. Move vehicle. Accept or Cancel when done.")
end

-- Called when "Accept Compass" is selected
local function on_compass_accept(value)
    if not value then return end
    if not compass_calibration_running then
        gcs:send_text(MAV_SEVERITY.WARNING, "Compass calibration not running.")
        return
    end
    gcs:run_command_int(MAV_CMD_DO_ACCEPT_MAG_CAL, { p3 = 1 })
    gcs:send_text(MAV_SEVERITY.INFO, "Compass calibration accepted.")
    compass_calibration_running = false
end

-- Called when "Cancel Compass" is selected
local function on_compass_cancel(value)
    if not value then return end
    if not compass_calibration_running then
        gcs:send_text(MAV_SEVERITY.WARNING, "Compass calibration not running.")
        return
    end
    gcs:run_command_int(MAV_CMD_DO_CANCEL_MAG_CAL, { p3 = 1 })
    gcs:send_text(MAV_SEVERITY.WARNING, "Compass calibration cancelled by user.")
    compass_calibration_running = false
end

local function on_accel_cal(value)
    if not value then return end
    gcs:run_command_int(MAV_CMD_PREFLIGHT_CALIBRATION, { p5 = 4 })
    gcs:send_text(MAV_SEVERITY.INFO, "Simple Accel calibration started.")
end

local function on_gyro_cal(value)
    if not value then return end
    gcs:run_command_int(MAV_CMD_PREFLIGHT_CALIBRATION, { p1 = 1 })
    gcs:send_text(MAV_SEVERITY.INFO, "Gyro calibration started.")
end

local function on_force_accel_cal(value)
    if not value then return end
    gcs:run_command_int(MAV_CMD_PREFLIGHT_CALIBRATION, { p5 = 76 })
    gcs:send_text(MAV_SEVERITY.INFO, "Forcing Accel calibration.")
end

local function on_force_compass_cal(value)
    if not value then return end
    gcs:run_command_int(MAV_CMD_PREFLIGHT_CALIBRATION, { p2 = 76 })
    gcs:send_text(MAV_SEVERITY.INFO, "Forcing Compass calibration.")
end

local function on_ahrs_trim(value)
    if not value then return end
    gcs:run_command_int(MAV_CMD_PREFLIGHT_CALIBRATION, { p5 = 2 })
    gcs:send_text(MAV_SEVERITY.INFO, "AHRS Trim command sent.")
end


-- ####################
-- # MENU DEFINITION
-- ####################

local menu_definition = {
    name = "Calibrate",
    items = {
        {type = 'INFO',    name = "Compass Cal",     info = "Start then Accept/Cancel"},
        {type = 'COMMAND', name = "Start Compass",   callback = on_compass_start},
        {type = 'COMMAND', name = "Accept Compass",  callback = on_compass_accept},
        {type = 'COMMAND', name = "Cancel Compass",  callback = on_compass_cancel},
        {type = 'INFO',    name = "---------------", info = ""},
        {type = 'COMMAND', name = "Calibrate Accels",  callback = on_accel_cal},
        {type = 'COMMAND', name = "Calibrate Gyros",   callback = on_gyro_cal},
        {type = 'COMMAND', name = "Trim AHRS",         callback = on_ahrs_trim},
        {type = 'INFO',    name = "--- Force ---",     info = "Use with caution"},
        {type = 'COMMAND', name = "Forcecal Accels",   callback = on_force_accel_cal},
        {type = 'COMMAND', name = "Forcecal Compass",  callback = on_force_compass_cal},
    }
}

-- 1. Register this script's menu definition with the helper.
return crsf_helper.register_menu(menu_definition)