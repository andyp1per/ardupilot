-- crsf_helper.lua
-- A reusable helper library to simplify the creation of ArduPilot CRSF menus.
-- This library abstracts away the complexity of binary packing/unpacking and event loop management.
-- Version 2.1: Fixed memory corruption bug by keeping references to all CRSF objects.

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

-- Internal storage for menu items, callbacks, and object references
local menu_items = {}
local crsf_objects = {} -- Keep references to all CRSF objects to prevent garbage collection

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

-- Recursively parses the user's declarative menu table and builds the CRSF menu structure.
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

-- This function runs in the background, listens for menu events, and triggers callbacks.
local function event_loop()
    -- Process multiple events per cycle to keep the menu responsive, but with a limit
    -- to prevent starving the main scheduler, which can cause a crash.
    local MAX_EVENTS_PER_CYCLE = 10
    local events_processed = 0

    while events_processed < MAX_EVENTS_PER_CYCLE do
        -- Listen for both read and write events from the transmitter
        local events_to_get = CRSF_EVENT.PARAMETER_READ + CRSF_EVENT.PARAMETER_WRITE
        local param_id, payload, events = crsf:get_menu_event(events_to_get)

        -- An 'events' value of 0 means the event queue is empty.
        if not events or events == 0 then
            break -- Exit the while loop
        end

        events_processed = events_processed + 1

        local item_def = menu_items[param_id]
        if not item_def then
            -- No item definition found for this ID, continue to next event
            goto continue_loop
        end

        -- Handle a READ request from the transmitter first.
        if (events & CRSF_EVENT.PARAMETER_READ) ~= 0 then
            if item_def.type == 'SELECTION' then
                local packed_data = create_selection_entry(item_def.name, item_def.options, item_def.current_idx)
                crsf:send_write_response(packed_data)
            elseif item_def.type == 'COMMAND' then
                local packed_data = create_command_entry(item_def.name)
                crsf:send_write_response(packed_data)
            end

        -- Handle a WRITE request from the transmitter only if it wasn't a read.
        elseif (events & CRSF_EVENT.PARAMETER_WRITE) ~= 0 then
            if not item_def.callback then
                goto continue_loop -- No callback found for this write event
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
        ::continue_loop::
    end

    -- Reschedule the event loop to run again. A shorter delay makes the UI feel snappier.
    return event_loop, 10
end


-- ####################
-- # PUBLIC API
-- ####################

-- The main entry point for the helper library.
-- The user script calls this function with its menu definition table.
function helper.init(menu_definition)
    -- Create the top-level menu
    local top_menu_obj = crsf:add_menu(menu_definition.name)
    if not top_menu_obj then
        gcs:send_text(MAV_SEVERITY.ERROR, "CRSF: Failed to create top-level menu.")
        return
    end
    table.insert(crsf_objects, top_menu_obj) -- Keep a reference to the top-level menu object

    -- Parse the rest of the menu structure
    parse_menu(menu_definition, top_menu_obj)

    gcs:send_text(MAV_SEVERITY.INFO, "CRSF Menu '" .. menu_definition.name .. "' initialized.")

    -- Start the background event loop
    return event_loop, 1000 -- Initial delay before starting the loop
end

return helper

