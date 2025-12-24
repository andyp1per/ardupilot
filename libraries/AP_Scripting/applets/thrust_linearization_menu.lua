--[[
  Thrust Linearization Menu Script
  CRSF menu interface for the thrust linearization test.
  Communicates with core script via TLIN_ parameters.

  Requires crsf_helper.lua module to be available.
--]]

local crsf_helper = require('crsf_helper')

local MAV_SEVERITY = crsf_helper.MAV_SEVERITY
local CRSF_COMMAND_STATUS = crsf_helper.CRSF_COMMAND_STATUS

-- Bind to existing parameter (created by core script)
local function bind_param(name)
    local p = Parameter()
    if not p:init(name) then
        return nil
    end
    return p
end

-- Wait for core script to create parameters (retry on init)
local TLIN_ENABLE = nil
local TLIN_MODE = nil
local TLIN_SPD = nil
local TLIN_DIST = nil
local TLIN_START = nil
local TLIN_STATE = nil
local TLIN_RESULT = nil

local function init_params()
    TLIN_ENABLE = bind_param("TLIN_ENABLE")
    TLIN_MODE = bind_param("TLIN_MODE")
    TLIN_SPD = bind_param("TLIN_SPD")
    TLIN_DIST = bind_param("TLIN_DIST")
    TLIN_START = bind_param("TLIN_START")
    TLIN_STATE = bind_param("TLIN_STATE")
    TLIN_RESULT = bind_param("TLIN_RESULT")

    if not TLIN_ENABLE then
        return false
    end
    return true
end

-- State names for display
local STATE_NAMES = {"Ready", "Running", "Complete", "Error"}

-- Status info item (updated dynamically)
local status_item = nil
local result_item = nil

-- Update status display
local function update_status_info()
    if not status_item or not TLIN_STATE then
        return
    end

    local state = TLIN_STATE:get()
    if state then
        local state_idx = math.floor(state) + 1
        if state_idx >= 1 and state_idx <= #STATE_NAMES then
            status_item.info = STATE_NAMES[state_idx]
        else
            status_item.info = "Unknown"
        end
    else
        status_item.info = "No Core"
    end

    -- Update result display
    if result_item and TLIN_RESULT then
        local result = TLIN_RESULT:get()
        if result and result > 0 then
            result_item.info = string.format("%.2f", result)
        else
            result_item.info = "---"
        end
    end
end

-- Callback: Start test command
local function on_start_test(action)
    update_status_info()

    if action == CRSF_COMMAND_STATUS.START then
        if not TLIN_START then
            return CRSF_COMMAND_STATUS.READY, "No Core"
        end

        local state = TLIN_STATE:get()
        if state == 1 then
            return CRSF_COMMAND_STATUS.READY, "Running..."
        end

        return CRSF_COMMAND_STATUS.CONFIRMATION_NEEDED, "Confirm?"

    elseif action == CRSF_COMMAND_STATUS.CONFIRM then
        if TLIN_START then
            TLIN_START:set(1)
            gcs:send_text(MAV_SEVERITY.INFO, "TLIN: Test triggered from menu")
        end
        return CRSF_COMMAND_STATUS.READY, "Started"

    elseif action == CRSF_COMMAND_STATUS.CANCEL then
        return CRSF_COMMAND_STATUS.READY, "Cancelled"

    elseif action == CRSF_COMMAND_STATUS.POLL then
        update_status_info()
        local state = TLIN_STATE and TLIN_STATE:get() or 0
        if state == 1 then
            return CRSF_COMMAND_STATUS.PROGRESS, "Running..."
        end
        return CRSF_COMMAND_STATUS.READY, "Start"
    end

    return CRSF_COMMAND_STATUS.READY, "Start"
end

-- Callback: Mode selection changed
local function on_mode_change(new_mode)
    if not TLIN_MODE then
        return
    end

    if new_mode == "Hover" then
        TLIN_MODE:set(0)
    elseif new_mode == "Forward" then
        TLIN_MODE:set(1)
    end

    gcs:send_text(MAV_SEVERITY.INFO, "TLIN: Mode set to " .. new_mode)
end

-- Callback: Test speed changed
local function on_speed_change(new_speed)
    if TLIN_SPD then
        TLIN_SPD:set(new_speed)
        gcs:send_text(MAV_SEVERITY.INFO, string.format("TLIN: Speed set to %.0f m/s", new_speed))
    end
end

-- Callback: Max distance changed
local function on_dist_change(new_dist)
    if TLIN_DIST then
        TLIN_DIST:set(new_dist)
        gcs:send_text(MAV_SEVERITY.INFO, string.format("TLIN: Max dist set to %.0f m", new_dist))
    end
end

-- Get current mode selection index
local function get_mode_default()
    if TLIN_MODE then
        local mode = TLIN_MODE:get()
        if mode and mode == 1 then
            return 2  -- Forward
        end
    end
    return 1  -- Hover
end

-- Get current speed value
local function get_speed_default()
    if TLIN_SPD then
        local spd = TLIN_SPD:get()
        if spd then
            return spd
        end
    end
    return 8
end

-- Get current distance value
local function get_dist_default()
    if TLIN_DIST then
        local dist = TLIN_DIST:get()
        if dist then
            return dist
        end
    end
    return 300
end

-- Build menu definition
local function build_menu()
    status_item = {
        type = 'INFO',
        name = "Status",
        info = "Ready"
    }

    result_item = {
        type = 'INFO',
        name = "Result",
        info = "---"
    }

    local menu_definition = {
        name = "Thrust Lin",
        items = {
            status_item,
            result_item,
            {
                type = 'COMMAND',
                name = "Start Test",
                info = "Start",
                callback = on_start_test
            },
            {
                type = 'MENU',
                name = "Config",
                items = {
                    {
                        type = 'SELECTION',
                        name = "Mode",
                        options = {"Hover", "Forward"},
                        default = get_mode_default(),
                        callback = on_mode_change
                    },
                    {
                        type = 'NUMBER',
                        name = "Speed",
                        min = 2,
                        max = 20,
                        default = get_speed_default(),
                        step = 1,
                        dpoint = 0,
                        unit = "m/s",
                        callback = on_speed_change
                    },
                    {
                        type = 'NUMBER',
                        name = "Max Dist",
                        min = 50,
                        max = 1000,
                        default = get_dist_default(),
                        step = 50,
                        dpoint = 0,
                        unit = "m",
                        callback = on_dist_change
                    }
                }
            }
        }
    }

    return menu_definition
end

-- Initialization with retry for parameter binding
local init_attempts = 0
local MAX_INIT_ATTEMPTS = 10

local function init_loop()
    init_attempts = init_attempts + 1

    if init_params() then
        gcs:send_text(MAV_SEVERITY.INFO, "TLIN: Menu params bound")
        local menu_def = build_menu()
        update_status_info()
        return crsf_helper.register_menu(menu_def)
    end

    if init_attempts >= MAX_INIT_ATTEMPTS then
        gcs:send_text(MAV_SEVERITY.WARNING, "TLIN: Menu init failed - no core params")
        return nil
    end

    return init_loop, 1000
end

gcs:send_text(MAV_SEVERITY.INFO, "TLIN: Menu script loaded")

return init_loop, 2000
