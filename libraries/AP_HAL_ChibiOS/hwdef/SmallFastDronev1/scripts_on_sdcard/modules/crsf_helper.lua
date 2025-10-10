-- crsf_helper.lua
-- A reusable helper library to simplify the creation of ArduPilot CRSF menus.
-- This library abstracts away the complexity of binary packing/unpacking and event loop management.
-- Version 6.0: Final definitive version. This version correctly implements the "Peek-and-Yield"
--              architecture for a multi-script, sandboxed environment. Each script runs its
--              own event loop, but they safely coexist by using the peek/pop API to only
--              process events that belong to them.

local helper = {}

-- MAVLink severity levels for GCS messages
local MAV_SEVERITY = {INFO = 6, WARNING = 4, ERROR = 3, DEBUG = 7}

-- CRSF constants
local CRSF_EVENT = {PARAMETER_READ = 1, PARAMETER_WRITE = 2}
local CRSF_PARAM_TYPE = {
    FLOAT = 8,
    TEXT_SELECTION = 9,
    FOLDER = 11,
    INFO = 12,
    COMMAND = 13,
}
local CRSF_COMMAND_STATUS = { READY = 0, START = 1 }

-- These tables are now local, respecting the script sandbox.
-- Each script will have its own instance of this state.
local menu_items = {}
local crsf_objects = {}

-- ####################
-- # PACKING FUNCTIONS
-- ####################

-- These functions create the binary packed strings required by the low-level CRSF API.

-- Creates a CRSF menu text selection item
local function create_selection_entry(name, options_table, current_idx)
    -- The CRSF spec requires options to be separated by a semicolon ';'.
    local options_str = table.concat(options_table, ";")
    local zero_based_idx = current_idx - 1
    local min_val = 0
    local max_val = #options_table - 1
    -- The 4th argument is the current value. The 7th is the default value.
    -- For our purposes, we'll pack the current value into both slots.
    return string.pack(">BzzBBBBz", CRSF_PARAM_TYPE.TEXT_SELECTION, name, options_str, zero_based_idx, min_val, max_val, zero_based_idx, "")
end

-- Creates a CRSF menu number item (as float)
local function create_number_entry(name, value, min, max, default, dpoint, step, unit)
    -- Per CRSF spec, float values are sent as INT32 with a decimal point indicator.
    local scale = 10^(dpoint or 0)
    local packed_value = math.floor(value * scale + 0.5)
    local packed_min = math.floor(min * scale + 0.5)
    local packed_max = math.floor(max * scale + 0.5)
    local packed_default = math.floor(default * scale + 0.5)
    local packed_step = math.floor(step * scale + 0.5)
    return string.pack(">BzllllBlz", CRSF_PARAM_TYPE.FLOAT, name, packed_value, packed_min, packed_max, packed_default, dpoint or 0, packed_step, unit or "")
end

-- Creates a CRSF menu info item
local function create_info_entry(name, info)
    return string.pack(">Bzz", CRSF_PARAM_TYPE.INFO, name, info)
end

-- Creates a CRSF command entry
local function create_command_entry(name)
    return string.pack(">BzBBz", CRSF_PARAM_TYPE.COMMAND, name, CRSF_COMMAND_STATUS.READY, 10, "Execute")
end

-- ####################
-- # MENU PARSING
-- ####################

-- Recursively parses a menu definition table and builds the CRSF menu structure.
local function parse_menu(menu_definition, parent_menu_obj)
    if not menu_definition.items or type(menu_definition.items) ~= "table" then
        return
    end

    for _, item_def in ipairs(menu_definition.items) do
        local param_obj = nil
        local packed_data = nil

        if item_def.type == 'MENU' then
            param_obj = parent_menu_obj:add_menu(item_def.name)
            if param_obj then
                parse_menu(item_def, param_obj) -- Recurse into sub-menu
            else
                gcs:send_text(MAV_SEVERITY.WARNING, "CRSF: Failed to create menu: " .. item_def.name)
            end

        elseif item_def.type == 'SELECTION' then
            item_def.current_idx = item_def.default -- Store the initial 1-based index
            packed_data = create_selection_entry(item_def.name, item_def.options, item_def.current_idx)
            param_obj = parent_menu_obj:add_parameter(packed_data)

        elseif item_def.type == 'NUMBER' then
            packed_data = create_number_entry(item_def.name, item_def.default, item_def.min, item_def.max, item_def.default, item_def.dpoint, item_def.step, item_def.unit)
            param_obj = parent_menu_obj:add_parameter(packed_data)

        elseif item_def.type == 'COMMAND' then
            packed_data = create_command_entry(item_def.name)
            param_obj = parent_menu_obj:add_parameter(packed_data)

        elseif item_def.type == 'INFO' then
            packed_data = create_info_entry(item_def.name, item_def.info)
            param_obj = parent_menu_obj:add_parameter(packed_data)
        end

        if param_obj then
            -- Store a reference to the CRSF object to prevent garbage collection
            table.insert(crsf_objects, param_obj)
            -- Store the CRSF-assigned ID back into our definition table for easy lookup
            menu_items[param_obj:id()] = item_def
        elseif not param_obj and item_def.type ~= 'MENU' then
            gcs:send_text(MAV_SEVERITY.WARNING, "CRSF: Failed to create param: " .. item_def.name)
        end
    end
end

-- ####################
-- # EVENT HANDLING
-- ####################

-- This function runs as an independent loop for each script.
-- It uses the peek/pop API to safely coexist with other menu scripts.
local function event_loop()
    -- ## 1. Peek at the event queue to see if there's anything to do ##
    local count, param_id, payload, events = crsf:peek_menu_event()

    -- If the queue is empty, reschedule with a longer, idle delay to save CPU.
    if count == 0 then
        return event_loop, 200
    end

    -- ## 2. Check if the event belongs to this script's menu ##
    local item_def = menu_items[param_id]
    if not item_def then
        -- This event is not for us. Yield and let another script's loop handle it.
        return event_loop, 20 -- Use a short delay as the UI is active
    end
    
    -- ## 3. Pop the event from the queue ##
    -- This is critical: pop the event before handling it.
    crsf:pop_menu_event()

    -- ## 4. Process the event (it's ours) ##

    -- Handle a READ request from the transmitter first.
    if (events & CRSF_EVENT.PARAMETER_READ) ~= 0 then
        if item_def.type == 'SELECTION' then
            local packed_data = create_selection_entry(item_def.name, item_def.options, item_def.current_idx)
            crsf:send_write_response(packed_data)
        elseif item_def.type == 'COMMAND' then
            local packed_data = create_command_entry(item_def.name)
            crsf:send_write_response(packed_data)
        else
            crsf:send_response()
        end
    end

    -- Handle a WRITE request from the transmitter.
    if (events & CRSF_EVENT.PARAMETER_WRITE) ~= 0 then
        if not item_def.callback then
            -- No callback, but we must still pop the event
            return event_loop, 20 -- Use a short delay as the UI is active
        end

        local new_value = nil

        -- Determine the new value from the payload and call the user's callback
        if item_def.type == 'SELECTION' then
            local selected_index_zero_based = string.unpack(">B", payload)
            item_def.current_idx = selected_index_zero_based + 1
            new_value = item_def.options[item_def.current_idx]
        elseif item_def.type == 'NUMBER' then
            local raw_value = string.unpack(">l", payload)
            local scale = 10^(item_def.dpoint or 0)
            new_value = raw_value / scale
        elseif item_def.type == 'COMMAND' then
            local command_action = string.unpack(">B", payload)
            if command_action == CRSF_COMMAND_STATUS.START then
                new_value = true
            end
        end

        if new_value ~= nil then
            local success, err = pcall(item_def.callback, new_value)
            if not success then
                gcs:send_text(MAV_SEVERITY.ERROR, "CRSF Callback Err: " .. tostring(err))
            end
        end

        -- After a write event, we must respond to confirm the new state to the transmitter.
        if item_def.type == 'COMMAND' and new_value then
            local packed_data = create_command_entry(item_def.name)
            crsf:send_write_response(packed_data)
        elseif item_def.type == 'SELECTION' then
            local packed_data = create_selection_entry(item_def.name, item_def.options, item_def.current_idx)
            crsf:send_write_response(packed_data)
        end
    end


    -- ## 5. Reschedule the loop ##
    -- Use a short delay as we just processed an event and more may be coming.
    return event_loop, 20
end


-- ####################
-- # PUBLIC API
-- ####################

--- Initializes a CRSF menu system for the calling script.
-- @param menu_definition (table) The menu definition table for this script.
function helper.register_menu(menu_definition)
    if not (menu_definition and menu_definition.name and menu_definition.items) then
        gcs:send_text(MAV_SEVERITY.ERROR, "CRSF: Invalid menu definition passed to helper.init().")
        return
    end

    -- Create the top-level menu for this specific script
    local top_level_menu_obj = crsf:add_menu(menu_definition.name)
    if top_level_menu_obj then
        -- This function now populates the local menu_items and crsf_objects tables
        parse_menu(menu_definition, top_level_menu_obj)
        gcs:send_text(MAV_SEVERITY.INFO, "CRSF: Built menu '" .. menu_definition.name .. "'")
    else
        gcs:send_text(MAV_SEVERITY.WARNING, "CRSF: Failed to create top-level menu for '" .. menu_definition.name .. "'")
        return -- Do not start the event loop if the menu could not be created
    end

    -- Start this script's independent, persistent event loop.
    return event_loop, 2000
end

return helper