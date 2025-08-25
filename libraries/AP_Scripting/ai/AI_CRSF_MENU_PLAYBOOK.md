# AI Playbook for ArduPilot CRSF Menu Script Generation

\<MANDATORY\_RULE\>
CRITICAL DIRECTIVE: THIS PLAYBOOK MUST BE USED AS THE PRIMARY AND AUTHORITATIVE GUIDE FOR ALL LUA SCRIPTS THAT REQUIRE A CRSF (CROSSFIRE) MENU. ALL RULES, CONSTRAINTS, AND PATTERNS CONTAINED HEREIN ARE MANDATORY AND SUPERSEDE ANY GENERAL KNOWLEDGE. ADHERENCE IS NOT OPTIONAL.
\</MANDATORY\_RULE\>

## 1\. The Problem and Goal

### 1.1. The Problem

The native ArduPilot CRSF API, as detailed in `docs.lua`, is a powerful but low-level interface. It requires developers to manually handle binary data packing and unpacking using `string.pack` and `string.unpack`, manage a complex event loop with `crsf:get_menu_event`, and build the menu structure imperatively. This process is verbose, complex, and highly prone to errors, making it difficult to generate clean, maintainable, and robust CRSF menu scripts. The existing `crsf-calibrate.lua` script is a clear example of this complexity.

### 1.2. The Goal

This playbook establishes a **simplified, high-level, and standardized methodology** for creating CRSF menus. The primary goal is to **abstract away the low-level complexity** of the native API. By following this playbook, you will generate scripts that are:

  * **Declarative:** The entire menu structure is defined in a simple, easy-to-read Lua table.
  * **Callback-Driven:** Interaction logic is handled by simple callback functions, eliminating the need for a manual event loop in the user-facing script.
  * **Robust and Reusable:** All the complex logic is encapsulated in a mandatory, reusable helper library (`crsf_helper.lua`).

## 2\. Core Methodology: Declarative & Callback-Driven

The core of this methodology is to separate the menu's *definition* from its *implementation*.

1.  **Menu Definition (`user_script.lua`):** The user script will define the entire menu structure—including sub-menus, parameters, ranges, and options—in a single, declarative Lua table.
2.  **Interaction Logic (`user_script.lua`):** The user script will define simple callback functions that are triggered when a parameter's value is changed via the CRSF menu.
3.  **Menu Implementation (`crsf_helper.lua`):** A mandatory helper library will parse the declarative table, build the menu using the low-level CRSF API, manage the event loop, and automatically invoke the correct user-defined callback with the new value.

\<MANDATORY\_RULE\>
CRITICAL DIRECTIVE: EVERY CRSF SCRIPT GENERATED MUST CONSIST OF TWO FILES: THE USER'S SCRIPT (E.G., `my_menu.lua`) AND THE STANDARD `crsf_helper.lua` LIBRARY. THE USER'S SCRIPT MUST `require()` THE HELPER LIBRARY.
\</MANDATORY\_RULE\>

## 3\. The `crsf_helper.lua` Library

This reusable library is the heart of the new methodology. You must include this exact code as a separate `crsf_helper.lua` file alongside any CRSF menu script you generate.

### 3.1. `crsf_helper.lua` Full Code

```lua
-- crsf_helper.lua
-- A reusable helper library to simplify the creation of ArduPilot CRSF menus.
-- This library abstracts away the complexity of binary packing/unpacking and event loop management.

local helper = {}

-- MAVLink severity levels for GCS messages
local MAV_SEVERITY = {INFO = 6, WARNING = 4}

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

-- Internal storage for menu items and their callbacks
local menu_items = {}
local item_id_counter = 1

-- ####################
-- # PACKING FUNCTIONS
-- ####################

-- These functions create the binary packed strings required by the low-level CRSF API.

-- Creates a CRSF menu text selection item
local function create_selection_entry(name, options_str, value_idx)
    -- Note: min, max, default are not used by OpenTX/EdgeTX for selections, but are part of the spec.
    -- Value is 0-indexed.
    return string.pack(">BzzBBBBz", CRSF_PARAM_TYPE.TEXT_SELECTION, name, options_str, value_idx - 1, 0, 0, 0, "")
end

-- Creates a CRSF menu number item (as float)
local function create_number_entry(name, value, min, max, default, dpoint, step, unit)
    return string.pack(">BzllllBlz", CRSF_PARAM_TYPE.FLOAT, name, value, min, max, default, dpoint, step, unit)
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

        -- Assign a unique ID to this item for tracking
        item_def.id = item_id_counter
        menu_items[item_def.id] = item_def
        item_id_counter = item_id_counter + 1

        if item_def.type == 'MENU' then
            param_obj = parent_menu_obj:add_menu(item_def.name)
            if param_obj then
                parse_menu(item_def, param_obj) -- Recurse into sub-menu
            else
                gcs:send_text(MAV_SEVERITY.WARNING, "CRSF: Failed to create menu: " .. item_def.name)
            end

        elseif item_def.type == 'SELECTION' then
            local options_str = table.concat(item_def.options, "\n")
            packed_data = create_selection_entry(item_def.name, options_str, item_def.default)
            param_obj = parent_menu_obj:add_parameter(packed_data)

        elseif item_def.type == 'NUMBER' then
            packed_data = create_number_entry(item_def.name, item_def.default, item_def.min, item_def.max, item_def.default, item_def.dpoint or 0, item_def.step or 1, item_def.unit or "")
            param_obj = parent_menu_obj:add_parameter(packed_data)

        elseif item_def.type == 'COMMAND' then
            packed_data = create_command_entry(item_def.name)
            param_obj = parent_menu_obj:add_parameter(packed_data)

        elseif item_def.type == 'INFO' then
            packed_data = create_info_entry(item_def.name, item_def.info)
            param_obj = parent_menu_obj:add_parameter(packed_data)
        end

        if param_obj and item_def.type ~= 'MENU' then
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
    local param_id, payload, events = crsf:get_menu_event(CRSF_EVENT.PARAMETER_WRITE)

    if (events and (events & CRSF_EVENT.PARAMETER_WRITE) ~= 0) then
        local item_def = menu_items[param_id]
        if not item_def or not item_def.callback then
            return event_loop, 100 -- No item or callback found, continue polling
        end

        local new_value = nil
        if item_def.type == 'SELECTION' then
            -- Unpack the 0-indexed selection from the payload
            local selected_index = string.unpack(">B", payload)
            new_value = item_def.options[selected_index + 1] -- Convert to 1-indexed value for Lua

        elseif item_def.type == 'NUMBER' then
            -- Unpack the float value from the payload
            new_value = string.unpack(">l", payload)

        elseif item_def.type == 'COMMAND' then
            local command_action = string.unpack(">B", payload)
            if command_action ~= CRSF_COMMAND_STATUS.START then
                return event_loop, 100 -- Ignore anything other than the 'start' command
            end
            -- For commands, the value passed to the callback is simply 'true'
            new_value = true
        end

        -- If we have a new value, call the user's callback function
        if new_value ~= nil then
            local success, err = pcall(item_def.callback, new_value)
            if not success then
                gcs:send_text(MAV_SEVERITY.WARNING, "CRSF Callback Err: " .. tostring(err))
            end
        end

        -- For commands, we must send a response to reset the UI element
        if item_def.type == 'COMMAND' then
            local packed_data = create_command_entry(item_def.name)
            crsf:send_write_response(packed_data)
        end
    end

    return event_loop, 100 -- Reschedule the event loop
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
        gcs:send_text(MAV_SEVERITY.WARNING, "CRSF: Failed to create top-level menu.")
        return
    end

    -- Parse the rest of the menu structure
    parse_menu(menu_definition, top_menu_obj)

    gcs:send_text(MAV_SEVERITY.INFO, "CRSF Menu '" .. menu_definition.name .. "' initialized.")

    -- Start the background event loop
    return event_loop, 1000 -- Initial delay before starting the loop
end

return helper
```

## 4\. Declarative Menu Syntax

The menu is defined as a nested Lua table. Each item in the table is a table itself, representing a menu item, sub-menu, or parameter.

### 4.1. Top-Level Menu Table

The root of the definition must be a table with two keys:

  * `name`: (string) The name of the root menu entry that appears on the transmitter.
  * `items`: (table) A list of tables, where each table defines a menu item.

### 4.2. Menu Item Properties

| Property | Type | Description | Used By |
| :--- | :--- | :--- | :--- |
| `type` | `string` | **Required.** The type of menu item. Must be one of: `'MENU'`, `'NUMBER'`, `'SELECTION'`, `'COMMAND'`, `'INFO'`. | All |
| `name` | `string` | **Required.** The text displayed for this menu item. | All |
| `callback` | `function` | **Required.** The function to call when the value is changed or the command is executed. | `NUMBER`, `SELECTION`, `COMMAND` |
| `items` | `table` | A list of item definition tables for a sub-menu. | `MENU` |
| `default` | `number` | The initial value of the parameter. For `SELECTION`, this is the **1-based index** of the `options` table. | `NUMBER`, `SELECTION` |
| `min`, `max` | `number` | The minimum and maximum allowed values. | `NUMBER` |
| `step` | `number` | *Optional.* The increment/decrement step size. Defaults to 1. | `NUMBER` |
| `dpoint` | `number` | *Optional.* The number of decimal points to display. Defaults to 0. | `NUMBER` |
| `unit` | `string` | *Optional.* A short string for the unit (e.g., "m", "s", "%"). | `NUMBER` |
| `options` | `table` | A list of strings for the available choices. | `SELECTION` |
| `info` | `string` | The read-only text to be displayed next to the name. | `INFO` |

## 5\. Main Script Pattern (`user_script.lua`)

This is the standard pattern you must follow for the user-facing script.

1.  **Require the Helper:** The first line must be `local crsf_helper = require('crsf_helper')`.
2.  **Define Callbacks:** Create the functions that will handle value changes. These functions will receive one argument: the new value. For `SELECTION`, this is the selected string. For `NUMBER`, it's the new number. For `COMMAND`, it's simply `true`.
3.  **Define the Menu Table:** Create the `menu_definition` table according to the syntax in section 4.
4.  **Initialize:** The script must return `crsf_helper.init(menu_definition)`.

### 5.1. Complete Example

Here is a complete, working example demonstrating a main script and the required helper.

#### `my_settings_menu.lua` (The user script you generate)

```lua
-- my_settings_menu.lua
-- An example script demonstrating the high-level declarative CRSF menu system.

local crsf_helper = require('crsf_helper')

-- ####################
-- # CALLBACK FUNCTIONS
-- ####################

-- This function is called when the "Flight Mode" is changed in the menu.
-- The 'new_mode' argument will be the string selected, e.g., "Loiter" or "Auto".
local function on_flight_mode_change(new_mode)
    gcs:send_text(6, "Flight Mode changed to: " .. new_mode)
    if new_mode == "Loiter" then
        vehicle:set_mode(5) -- 5 is the mode number for Loiter
    elseif new_mode == "Auto" then
        vehicle:set_mode(3)
    elseif new_mode == "RTL" then
        vehicle:set_mode(6)
    end
end

-- This function is called when the "Target Altitude" is changed.
-- The 'new_altitude' argument will be the number set in the menu.
local function on_altitude_change(new_altitude)
    gcs:send_text(6, "Target altitude set to: " .. new_altitude .. "m")
    -- Here you would typically store this value in a variable for use elsewhere
end

-- This function is called when the "Reboot Vehicle" command is executed.
-- The 'value' argument will simply be 'true'.
local function on_reboot_command(value)
    if value then
        gcs:send_text(4, "Rebooting vehicle via CRSF menu command!")
        vehicle:reboot(false)
    end
end


-- ####################
-- # MENU DEFINITION
-- ####################

-- This single table declaratively defines the entire menu structure.
local menu_definition = {
    name = "My Settings", -- The root menu name
    items = {
        {
            type = 'SELECTION',
            name = "Flight Mode",
            options = {"Loiter", "Auto", "RTL"},
            default = 1, -- 1-based index for "Loiter"
            callback = on_flight_mode_change
        },
        {
            type = 'NUMBER',
            name = "Target Altitude",
            min = 10,
            max = 100,
            step = 5,
            default = 30,
            unit = "m",
            callback = on_altitude_change
        },
        {
            type = 'INFO',
            name = "Firmware",
            info = FWVersion:string()
        },
        {
            type = 'MENU',
            name = "Advanced",
            items = {
                {
                    type = 'COMMAND',
                    name = "Reboot Vehicle",
                    callback = on_reboot_command
                }
            }
        }
    }
}

-- ####################
-- # INITIALIZATION
-- ####################

-- Pass the menu definition to the helper library to build the menu and start the event loop.
return crsf_helper.init(menu_definition)
```

## 6\. Mandatory Rules and Checklist

1.  **\[ \] Use Declarative Table:** The entire menu structure **must** be defined in a single Lua table.
2.  **\[ \] Use `crsf_helper.lua`:** The provided `crsf_helper.lua` library **must** be included and used. Do not attempt to re-implement its logic.
3.  **\[ \] Use `require()`:** The main script **must** load the helper using `require('crsf_helper')`.
4.  **\[ \] Use Callbacks:** All menu interactions **must** be handled via callback functions assigned in the menu definition table. The main script must not contain a `crsf:get_menu_event` loop.
5.  **\[ \] Return `helper.init()`:** The main script **must** conclude by returning the result of the `crsf_helper.init()` function, passing its menu definition table as the argument.
6.  **\[ \] Follow Parameter Syntax:** All parameter types (`NUMBER`, `SELECTION`, etc.) **must** use the exact property names and data types defined in section 4.2.