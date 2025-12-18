--[[
  Driver for SN74HC165 8-bit Parallel-Load Shift Registers
  
  This script bit-bangs the GPIOs to read the state of a 74HC165 shift register.
  It supports cascading multiple chips.
  
  Wiring:
  - Connect SN74HC165 SH/LD (Pin 1) to a GPIO (Configured via HC165_PIN_PL)
  - Connect SN74HC165 CLK (Pin 2) to a GPIO (Configured via HC165_PIN_CP)
  - Connect SN74HC165 QH (Pin 9) to a GPIO (Configured via HC165_PIN_Q7)
  - Connect SN74HC165 CLK INH (Pin 15) to GND
  - Connect SN74HC165 SER (Pin 10) to GND (or QH of previous stage if cascading)
--]]

local SCRIPT_NAME = "SN74HC165"
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 95
local PARAM_TABLE_PREFIX = "HC165_"

-- Parameter Helper Functions
local function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format('Could not find %s parameter', name))
    return p
end

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('Could not add param %s', name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- Add Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 10), 'Could not add param table')

--[[
  // @Param: HC165_ENABLE
  // @DisplayName: Enable SN74HC165 Driver
  // @Description: Enable or disable the shift register driver.
  // @Values: 0:Disabled, 1:Enabled
  // @User: Standard
--]]
local HC165_ENABLE = bind_add_param('ENABLE', 1, 0)

--[[
  // @Param: HC165_PIN_PL
  // @DisplayName: Parallel Load Pin
  // @Description: GPIO pin number connected to the SH/LD (Shift/Load) pin of the 74HC165.
  // @User: Standard
--]]
local HC165_PIN_PL = bind_add_param('PIN_PL', 2, 50) -- Default to AUX1

--[[
  // @Param: HC165_PIN_CP
  // @DisplayName: Clock Pin
  // @Description: GPIO pin number connected to the CLK (Clock) pin of the 74HC165.
  // @User: Standard
--]]
local HC165_PIN_CP = bind_add_param('PIN_CP', 3, 51) -- Default to AUX2

--[[
  // @Param: HC165_PIN_Q7
  // @DisplayName: Data Output Pin
  // @Description: GPIO pin number connected to the QH (Serial Output) pin of the 74HC165.
  // @User: Standard
--]]
local HC165_PIN_Q7 = bind_add_param('PIN_Q7', 4, 52) -- Default to AUX3

--[[
  // @Param: HC165_WIDTH
  // @DisplayName: Register Width (Bytes)
  // @Description: Number of 8-bit shift registers chained together.
  // @Range: 1 4
  // @User: Standard
--]]
local HC165_WIDTH = bind_add_param('WIDTH', 5, 1)

--[[
  // @Param: HC165_INVERT
  // @DisplayName: Invert Input
  // @Description: Invert the read logic logic. 0: Normal, 1: Inverted.
  // @Values: 0:Normal, 1:Inverted
  // @User: Standard
--]]
local HC165_INVERT = bind_add_param('INVERT', 6, 0)

--[[
  // @Param: HC165_DEBUG
  // @DisplayName: Debug Level
  // @Description: Verbosity of GCS messages.
  // @Values: 0:None, 1:State Changes, 2:Continuous
  // @User: Advanced
--]]
local HC165_DEBUG = bind_add_param('DEBUG', 7, 1)

-- State Variables
local last_mask = 0
local pins_initialized = false

-- Init pins
local function init_pins()
    local pin_pl = math.floor(HC165_PIN_PL:get())
    local pin_cp = math.floor(HC165_PIN_CP:get())
    local pin_q7 = math.floor(HC165_PIN_Q7:get())

    if pin_pl <= 0 or pin_cp <= 0 or pin_q7 <= 0 then
        gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME .. ": Pins not configured")
        return false
    end

    -- Configure PL and CP as Outputs
    gpio:pinMode(pin_pl, 1) -- Output
    gpio:pinMode(pin_cp, 1) -- Output
    
    -- Configure Q7 as Input
    gpio:pinMode(pin_q7, 0) -- Input

    -- Set initial states
    -- SH/LD should be High normally
    -- CLK should be Low normally
    gpio:write(pin_pl, 1)
    gpio:write(pin_cp, 0)

    pins_initialized = true
    gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME .. ": Pins initialized")
    return true
end

-- Read Shift Register
local function read_shift_register()
    local pin_pl = math.floor(HC165_PIN_PL:get())
    local pin_cp = math.floor(HC165_PIN_CP:get())
    local pin_q7 = math.floor(HC165_PIN_Q7:get())
    local width = math.floor(HC165_WIDTH:get())
    local invert = math.floor(HC165_INVERT:get()) == 1

    local bit_count = width * 8
    local mask_val = 0

    -- 1. Latch Data: Pulse SH/LD Low then High
    gpio:write(pin_pl, 0)
    -- Short delay might be needed electrically, but Lua execution overhead is usually sufficient
    gpio:write(pin_pl, 1)

    -- 2. Read Bits
    for i = 0, bit_count - 1 do
        -- Read the current bit from Q7
        local bit_val = gpio:read(pin_q7) and 1 or 0
        
        if invert then
            bit_val = bit_val == 1 and 0 or 1
        end

        -- Shift into our mask. 
        -- Note: The 74HC165 shifts MSB (H) first typically, but wiring affects this.
        -- We will assume standard wiring where the first bit read is the "highest" pin (H) 
        -- or "lowest" (A) depending on the cascade. 
        -- We shift the current value left and add the new bit at the LSB.
        -- Or we can fill from MSB. 
        -- Standard 74HC165 operation:
        -- Data is shifted out QH.
        -- Let's construct the integer such that the first bit read is the LSB or MSB.
        -- Usually, bit H (Pin 6) is the first shifted out if SER is used. 
        -- Let's stick to: First bit read becomes Bit 0 (LSB) or Bit N?
        -- Standard: Input A is LSB, H is MSB.
        -- Shift register shifts H -> G -> ... -> A. 
        -- So the first bit read is H (Bit 7).
        
        -- We will construct it so the first bit read is the MSB of the current byte.
        -- Because we read H first, then G...
        
        if bit_val == 1 then
             -- If we assume the stream is MSB first (H, G, F...), we verify the bit position.
             -- Let's map first read bit to highest index for intuitive hardware mapping
             -- (Pin H = bit 7).
             local shift_pos = (bit_count - 1) - i
             mask_val = mask_val | (1 << shift_pos)
        end

        -- Pulse Clock
        gpio:write(pin_cp, 1)
        gpio:write(pin_cp, 0)
    end

    return mask_val
end

-- Main update function
local function update()
    if HC165_ENABLE:get() == 0 then
        return update, 1000
    end

    if not pins_initialized then
        if not init_pins() then
            return update, 1000
        end
    end

    local current_mask = read_shift_register()

    -- Send to GCS as a named float for logging/display
    gcs:send_named_float("HC165_MASK", current_mask)

    -- Debug output
    local debug_lvl = math.floor(HC165_DEBUG:get())
    
    if debug_lvl >= 2 then
        gcs:send_text(MAV_SEVERITY.INFO, string.format("%s: Mask: 0x%X", SCRIPT_NAME, current_mask))
    elseif debug_lvl == 1 then
        if current_mask ~= last_mask then
            gcs:send_text(MAV_SEVERITY.INFO, string.format("%s: Change: 0x%X", SCRIPT_NAME, current_mask))
        end
    end

    last_mask = current_mask

    -- 10Hz Update rate
    return update, 100 
end

-- Wrapper to catch errors
local function protected_wrapper()
  local success, err = pcall(update)
  if not success then
     gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME .. ": Error: " .. err)
     return protected_wrapper, 1000
  end
  return protected_wrapper, 100
end

gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME .. ": Driver Loaded")
return protected_wrapper()
