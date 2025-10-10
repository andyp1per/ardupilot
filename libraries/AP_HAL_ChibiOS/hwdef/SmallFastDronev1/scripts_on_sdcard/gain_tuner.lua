-- gain_tuner.lua
-- A script to provide a CRSF menu for in-flight PID gain tuning and utility functions.
-- Allows for increasing or decreasing key PID gains for an entire axis at once.
-- Includes Save/Revert, stateful step-based adjustments, and a configurable tuning step.
-- Also includes utilities for logging and autotune setup.

local crsf_helper = require('crsf_helper')

-- MAVLink severity for GCS messages
local MAV_SEVERITY = {INFO = 6, WARNING = 4, ERROR = 3}

-- The percentage change for each step, can be changed via the menu
local tuning_step_percent -- This will be initialized later from the menu definition

-- ####################
-- # PARAMETER SETUP
-- ####################

-- Group parameters by axis into tables for easier handling.
-- Using assert() ensures the script will halt with a clear error if a parameter name is incorrect.
local axis_gains = {
    Roll = {
        {param = assert(Parameter('ATC_RAT_RLL_P'), "Failed to find ATC_RAT_RLL_P"), name = "Roll P"},
        {param = assert(Parameter('ATC_RAT_RLL_I'), "Failed to find ATC_RAT_RLL_I"), name = "Roll I"},
        {param = assert(Parameter('ATC_RAT_RLL_D'), "Failed to find ATC_RAT_RLL_D"), name = "Roll D"},
        {param = assert(Parameter('ATC_ANG_RLL_P'), "Failed to find ATC_ANG_RLL_P"), name = "Roll Ang P"}
    },
    Pitch = {
        {param = assert(Parameter('ATC_RAT_PIT_P'), "Failed to find ATC_RAT_PIT_P"), name = "Pitch P"},
        {param = assert(Parameter('ATC_RAT_PIT_I'), "Failed to find ATC_RAT_PIT_I"), name = "Pitch I"},
        {param = assert(Parameter('ATC_RAT_PIT_D'), "Failed to find ATC_RAT_PIT_D"), name = "Pitch D"},
        {param = assert(Parameter('ATC_ANG_PIT_P'), "Failed to find ATC_ANG_PIT_P"), name = "Pitch Ang P"}
    },
    Yaw = {
        {param = assert(Parameter('ATC_RAT_YAW_P'), "Failed to find ATC_RAT_YAW_P"), name = "Yaw P"},
        {param = assert(Parameter('ATC_RAT_YAW_I'), "Failed to find ATC_RAT_YAW_I"), name = "Yaw I"},
        {param = assert(Parameter('ATC_RAT_YAW_D'), "Failed to find ATC_RAT_YAW_D"), name = "Yaw D"},
        {param = assert(Parameter('ATC_ANG_YAW_P'), "Failed to find ATC_ANG_YAW_P"), name = "Yaw Ang P"}
    }
}

-- Table to store the original parameter values on script startup for the revert function
local original_gains = {}
-- Table to store the number of tuning steps applied to each axis (e.g., +1, -2, etc.)
local gain_steps = { Roll = 0, Pitch = 0, Yaw = 0 }

-- Populate the original_gains table by iterating through all defined parameters
local function populate_original_gains()
    for axis_name, gains in pairs(axis_gains) do
        original_gains[axis_name] = {}
        for _, gain_info in ipairs(gains) do
            table.insert(original_gains[axis_name], {
                param = gain_info.param,
                original_value = gain_info.param:get()
            })
        end
    end
end

-- Initial population of original gains at script start
populate_original_gains()

-- ####################
-- # CORE LOGIC & CALLBACKS
-- ####################

-- This function is called when the "Step %" is changed in the menu.
-- It parses the string (e.g., "10%") and updates the tuning_step_percent variable.
local function on_step_change(new_step_str)
    -- Remove the '%' character and convert the string to a number
    -- The extra parentheses ensure only the first return value of gsub is passed to tonumber
    local percent_num = tonumber((string.gsub(new_step_str, "%%", "")))
    if percent_num then
        tuning_step_percent = percent_num / 100.0
        gcs:send_text(MAV_SEVERITY.INFO, "Tuning step set to: " .. new_step_str)
    end
end

-- This function adjusts all gains for a given axis based on a step counter.
-- @param axis_name (string): The name of the axis to adjust ("Roll", "Pitch", or "Yaw").
-- @param step_change (integer): The change to apply to the step counter (+1 for increase, -1 for decrease).
local function adjust_axis_gains(axis_name, step_change)
    -- Update the step counter for the axis
    gain_steps[axis_name] = gain_steps[axis_name] + step_change

    local original_gain_table = original_gains[axis_name]
    if not original_gain_table then
        gcs:send_text(MAV_SEVERITY.WARNING, "Invalid axis: " .. tostring(axis_name))
        return
    end

    -- Calculate the total multiplier based on the original value and the current step count
    local total_multiplier = 1.0 + (gain_steps[axis_name] * tuning_step_percent)

    -- Iterate through the original parameters for the axis and calculate/set the new value
    for _, gain_info in ipairs(original_gain_table) do
        local new_value = gain_info.original_value * total_multiplier
        gain_info.param:set(new_value)
    end

    -- Send a confirmation message to the GCS for the entire axis
    local total_percent_change = gain_steps[axis_name] * tuning_step_percent * 100
    local gcs_message = string.format("%s Gains %+d%%", axis_name, math.floor(total_percent_change + 0.5))
    gcs:send_text(MAV_SEVERITY.INFO, gcs_message)
    
    -- Also send the new P gain value for quick reference
    local p_gain_val = axis_gains[axis_name][1].param:get()
    local p_gain_msg = string.format("%s P = %.4f", axis_name, p_gain_val)
    gcs:send_text(MAV_SEVERITY.INFO, p_gain_msg)
end

-- Saves all currently set gain values to EEPROM and resets the tuning state.
local function save_all_gains()
    for _, gains in pairs(axis_gains) do
        for _, gain_info in ipairs(gains) do
            local current_value = gain_info.param:get()
            gain_info.param:set_and_save(current_value)
        end
    end
    gcs:send_text(MAV_SEVERITY.INFO, "All tuned gains have been saved.")

    -- After saving, the new values become the baseline for further tuning.
    -- Repopulate the original_gains table and reset the step counters.
    populate_original_gains()
    gain_steps = { Roll = 0, Pitch = 0, Yaw = 0 }
    gcs:send_text(MAV_SEVERITY.INFO, "Tuning state reset to new values.")
end

-- Reverts all gains to the values they had when the script was started.
local function revert_all_gains()
    for _, gains in pairs(original_gains) do
        for _, gain_info in ipairs(gains) do
            gain_info.param:set(gain_info.original_value)
        end
    end
    -- Also reset the step counters back to zero
    gain_steps = { Roll = 0, Pitch = 0, Yaw = 0 }
    gcs:send_text(MAV_SEVERITY.INFO, "Gains reverted to startup values.")
end

-- ####################
-- # UTILITY CALLBACKS
-- ####################

-- Sets parameters for batch logging
local function toggle_batch_logging(enable)
    if enable then
        param:set('INS_LOG_BAT_CNT', 2048)
        param:set('INS_LOG_BAT_LGCT', 32)
        param:set('INS_LOG_BAT_LGIN', 10)
        param:set('INS_LOG_BAT_MASK', 3)
        param:set('INS_LOG_BAT_OPT', 4)
        gcs:send_text(MAV_SEVERITY.INFO, "Batch logging enabled.")
    else
        param:set('INS_LOG_BAT_MASK', 0)
        gcs:send_text(MAV_SEVERITY.INFO, "Batch logging disabled.")
    end
end

-- Enables or disables fast attitude logging by modifying LOG_BITMASK
local function toggle_fast_attitude_logging(enable)
    local log_bitmask = Parameter('LOG_BITMASK')
    local current_val = log_bitmask:get()
    if enable then
        if current_val % 2 == 0 then -- It's even, so add 1 to make it odd
            log_bitmask:set(current_val + 1)
            gcs:send_text(MAV_SEVERITY.INFO, "Fast attitude logging enabled.")
        else
            gcs:send_text(MAV_SEVERITY.INFO, "Fast attitude logging already enabled.")
        end
    else
        if current_val % 2 ~= 0 then -- It's odd, so subtract 1 to make it even
            log_bitmask:set(current_val - 1)
            gcs:send_text(MAV_SEVERITY.INFO, "Fast attitude logging disabled.")
        else
            gcs:send_text(MAV_SEVERITY.INFO, "Fast attitude logging already disabled.")
        end
    end
end

-- Erases all logs from the flight controller
local function erase_logs()
    -- MAV_CMD_PREFLIGHT_STORAGE (245), param2 = 2 for erase logs
    local result = gcs:run_command_int(245, {p2=2})
    if result == 0 then -- MAV_RESULT_ACCEPTED
        gcs:send_text(MAV_SEVERITY.INFO, "Log erase command sent.")
    else
        gcs:send_text(MAV_SEVERITY.ERROR, "Log erase command failed.")
    end
end

-- Sets up parameters for an autotune session
local function setup_autotune(axes_value, name)
    param:set('AUTOTUNE_AGGR', 0.075)
    param:set('AUTOTUNE_AXES', axes_value)
    param:set('AUTOTUNE_MIN_D', 0.0003)
    gcs:send_text(MAV_SEVERITY.INFO, "Autotune configured for: " .. name)
end

-- Sets the AUTOTUNE_GMBK parameter based on user selection
local function on_backoff_change(selection)
    local autotune_gmbk = Parameter('AUTOTUNE_GMBK')
    if selection == "Soft Tune" then
        autotune_gmbk:set(0.25)
        gcs:send_text(MAV_SEVERITY.INFO, "Autotune Backoff set to 0.25 (Soft).")
    elseif selection == "Firm Tune" then
        autotune_gmbk:set(0.1)
        gcs:send_text(MAV_SEVERITY.INFO, "Autotune Backoff set to 0.1 (Firm).")
    end
end

-- Callback for the Fast Attitude Log on/off selection
local function on_fast_att_log_change(selection)
    if selection == "On" then
        toggle_fast_attitude_logging(true)
    elseif selection == "Off" then
        toggle_fast_attitude_logging(false)
    end
end

-- Callback for the Batch Logging on/off selection
local function on_batch_log_change(selection)
    if selection == "On" then
        toggle_batch_logging(true)
    elseif selection == "Off" then
        toggle_batch_logging(false)
    end
end

-- ####################
-- # MENU DEFINITION
-- ####################

-- This table defines the complete menu structure.
local menu_definition = {
    name = "Gain Tuner", -- The root menu name
    items = {
        -- ====== ROLL TUNING SUB-MENU ======
        {
            type = 'MENU',
            name = "Roll Tuning",
            items = {
                {type = 'COMMAND', name = "Increase Gains", callback = function(v) if v then adjust_axis_gains("Roll", 1) end end},
                {type = 'COMMAND', name = "Decrease Gains", callback = function(v) if v then adjust_axis_gains("Roll", -1) end end},
            }
        },
        -- ====== PITCH TUNING SUB-MENU ======
        {
            type = 'MENU',
            name = "Pitch Tuning",
            items = {
                {type = 'COMMAND', name = "Increase Gains", callback = function(v) if v then adjust_axis_gains("Pitch", 1) end end},
                {type = 'COMMAND', name = "Decrease Gains", callback = function(v) if v then adjust_axis_gains("Pitch", -1) end end},
            }
        },
        -- ====== YAW TUNING SUB-MENU ======
        {
            type = 'MENU',
            name = "Yaw Tuning",
            items = {
                {type = 'COMMAND', name = "Increase Gains", callback = function(v) if v then adjust_axis_gains("Yaw", 1) end end},
                {type = 'COMMAND', name = "Decrease Gains", callback = function(v) if v then adjust_axis_gains("Yaw", -1) end end},
            }
        },
        -- ====== AUTOTUNE MENU ======
        {
            type = 'MENU',
            name = "Autotune",
            items = {
                {type = 'COMMAND', name = "Setup Roll/Pitch", callback = function(v) if v then setup_autotune(3, "Roll/Pitch") end end},
                {type = 'COMMAND', name = "Setup Yaw", callback = function(v) if v then setup_autotune(4, "Yaw") end end},
                {type = 'COMMAND', name = "Setup Yaw D Only", callback = function(v) if v then setup_autotune(8, "Yaw D") end end},
                {
                    type = 'SELECTION',
                    name = "Backoff",
                    options = {"Soft Tune", "Firm Tune"},
                    default = 1, -- 1-based index for "Soft Tune"
                    callback = on_backoff_change
                }
            }
        },
        -- ====== LOGGING MENU ======
        {
            type = 'MENU',
            name = "Logging",
            items = {
                {
                    type = 'SELECTION',
                    name = "Batch Logging",
                    options = {"Off", "On"},
                    default = 1, -- 1-based index for "Off"
                    callback = on_batch_log_change
                },
                {
                    type = 'SELECTION',
                    name = "Fast Attitude Log",
                    options = {"Off", "On"},
                    default = 1, -- 1-based index for "Off"
                    callback = on_fast_att_log_change
                },
                {type = 'INFO', name = "Warning", info = "Erase is final"},
                {type = 'COMMAND', name = "Erase All Logs", callback = function(v) if v then erase_logs() end end},
            }
        },
        -- ====== SETTINGS SUB-MENU ======
        {
            type = 'MENU',
            name = "Settings",
            items = {
                {
                    type = 'SELECTION',
                    name = "Step %",
                    options = {"1%", "5%", "10%", "25%", "50%"},
                    default = 3, -- 1-based index for "10%"
                    callback = on_step_change
                },
            }
        },
        -- ====== SAVE & REVERT SUB-MENU ======
        {
            type = 'MENU',
            name = "Save & Revert",
            items = {
                {type = 'INFO', name = "Warning", info = "Save is permanent"},
                {type = 'COMMAND', name = "Save All Gains", callback = function(v) if v then save_all_gains() end end},
                {type = 'COMMAND', name = "Revert All Gains", callback = function(v) if v then revert_all_gains() end end},
            }
        },
    }
}

-- ####################
-- # INITIALIZATION
-- ####################

-- Helper function to find a menu item by its name property
local function find_item_by_name(items_table, name)
    if not items_table then return nil end
    for _, item in ipairs(items_table) do
        if item.name == name then
            return item
        end
    end
    return nil
end

-- Initialize default Step %
local settings_menu = find_item_by_name(menu_definition.items, "Settings")
if settings_menu then
    local step_selection = find_item_by_name(settings_menu.items, "Step %")
    if step_selection then
        on_step_change(step_selection.options[step_selection.default])
    end
end

-- Initialize default logging settings
local logging_menu = find_item_by_name(menu_definition.items, "Logging")
if logging_menu then
    local batch_log_selection = find_item_by_name(logging_menu.items, "Batch Logging")
    if batch_log_selection then
        on_batch_log_change(batch_log_selection.options[batch_log_selection.default])
    end
    local fast_log_selection = find_item_by_name(logging_menu.items, "Fast Attitude Log")
    if fast_log_selection then
        on_fast_att_log_change(fast_log_selection.options[fast_log_selection.default])
    end
end

-- Initialize default Autotune Backoff setting
local autotune_menu = find_item_by_name(menu_definition.items, "Autotune")
if autotune_menu then
    local backoff_selection = find_item_by_name(autotune_menu.items, "Backoff")
    if backoff_selection then
        on_backoff_change(backoff_selection.options[backoff_selection.default])
    end
end

-- 1. Register this script's menu definition with the helper.
return crsf_helper.register_menu(menu_definition)