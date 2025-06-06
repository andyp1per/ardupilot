--[[
   parameter reversion utility. This helps with manual tuning
   in-flight by giving a way to instantly revert parameters to the startup parameters
--]]

---@diagnostic disable: param-type-mismatch


local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 31
local PARAM_TABLE_PREFIX = "PREV_"

local UPDATE_RATE_HZ = 4

-- bind a parameter to a variable, old syntax to support older firmware
function bind_param(name)
   local p = Parameter()
   if not p:init(name) then
      return nil
   end
   return p
end

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   local p = bind_param(PARAM_TABLE_PREFIX .. name)
   assert(p, string.format("count not find parameter %s", name))
   return p
end

-- setup script specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 5), 'could not add param table')

--[[
  // @Param: PREV_ENABLE
  // @DisplayName: parameter reversion enable
  // @Description: Enable parameter reversion system
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local PREV_ENABLE      = bind_add_param('ENABLE',         1, 0)

--[[
  // @Param: PREV_RC_FUNC
  // @DisplayName: param reversion RC function
  // @Description: RCn_OPTION number to used to trigger parameter reversion
  // @User: Standard
--]]
local PREV_RC_FUNC     = bind_add_param('RC_FUNC',       2, 300)

-- params dictionary indexed by name
local params = {}
local param_saved = {}
local param_count = 0

local ATC_prefixes = { "ATC", "Q_A" }
local PSC_prefixes = { "PSC", "Q_P" }
local PID_prefixes = { "_RAT_RLL_", "_RAT_PIT_", "_RAT_YAW_" }
local PID_suffixes = { "FF", "P", "I", "D", "D_FF", "PDMX", "NEF", "NTF", "IMAX", "FLTD", "FLTE", "FLTT", "SMAX" }
local angle_axes = { "RLL", "PIT", "YAW" }
local rate_limit_axes = { "R", "P", "Y"}
local PSC_types = { "ACCZ", "VELZ", "POSZ", "VELXY", "POSXY" }
local OTHER_PARAMS = { "INS_GYRO_FILTER", "INS_ACCEL_FILTER", "PTCH2SRV_TCONST", "RLL2SRV_TCONST" }

-- TECS params
TECS_PARAMS = { "TECS_APPR_SMAX", "TECS_CLMB_MAX", "TECS_FLARE_HGT", "TECS_HDEM_TCONST", "TECS_HGT_OMEGA", "TECS_INTEG_GAIN", "TECS_LAND_ARSPD", "TECS_LAND_DAMP", "TECS_LAND_IGAIN", "TECS_LAND_PDAMP", "TECS_LAND_PMAX", "TECS_LAND_SINK", "TECS_LAND_SPDWGT", "TECS_LAND_SRC", "TECS_LAND_TCONST", "TECS_LAND_TDAMP", "TECS_LAND_THR", "TECS_OPTIONS", "TECS_PITCH_MAX", "TECS_PITCH_MIN", "TECS_PTCH_DAMP", "TECS_PTCH_FF_K", "TECS_PTCH_FF_V0", "TECS_RLL2THR", "TECS_SINK_MAX", "TECS_SINK_MIN", "TECS_SPDWEIGHT", "TECS_SPD_OMEGA", "TECS_SYNAIRSPEED", "TECS_THR_DAMP", "TECS_TIME_CONST", "TECS_TKOFF_IGAIN", "TECS_VERT_ACC" }

local INS_HNTCH_PREFIX = { "INS_HNTCH_", "INS_HNTC2_" }
local INS_NOTCH_PARMS = { "ENABLE", "ATT", "FREQ", "BW", "OPTS", "REF", "FM_RAT", "MODE" }

if PREV_ENABLE:get() == 0 then
   return
end

local function add_param(pname)
   local p = bind_param(pname)
   if p then
      params[pname] = p
      param_saved[pname] = p:get()
      param_count = param_count + 1
      -- gcs:send_text(MAV_SEVERITY.INFO, string.format("Added %s", pname))
   end
end

-- add rate PIDs
for _, atc in ipairs(ATC_prefixes) do
   for _, prefix in ipairs(PID_prefixes) do
      for _, suffix in ipairs(PID_suffixes) do
         add_param(atc .. prefix .. suffix)
      end
   end
end

-- add angle Ps
for _, atc in ipairs(ATC_prefixes) do
   for _, axis in ipairs(angle_axes) do
      add_param(atc .. "_ANG_" .. axis .. "_P" )
   end
end

-- add angular rate limits
for _, atc in ipairs(ATC_prefixes) do
   for _, axis in ipairs(rate_limit_axes) do
      add_param(atc .. "_RATE_" .. axis .. "_MAX")
   end
end

-- add fixed wing tuning
for _, suffix in ipairs(PID_suffixes) do
   add_param("RLL_RATE_" .. suffix)
   add_param("PTCH_RATE_" .. suffix)
   add_param("YAW_RATE_" .. suffix)
end

-- add PSC tuning
for _, psc in ipairs(PSC_prefixes) do
   for _, ptype in ipairs(PSC_types) do
      for _, suffix in ipairs(PID_suffixes) do
         add_param(psc .. "_" .. ptype .. "_" .. suffix)
      end
   end
end

-- add in TECS parameters
for _, p in ipairs(TECS_PARAMS) do
   add_param(p)
end

-- add notch parameters
for _, ins_notch in ipairs(INS_HNTCH_PREFIX) do
   for _, p in ipairs(INS_NOTCH_PARMS) do
      add_param(ins_notch .. p)
   end
end

-- add in other parameters
for _, p in ipairs(OTHER_PARAMS) do
   add_param(p)
end


local function revert_parameters()
   local count = 0
   for pname, p in pairs(params) do
      local v1 = p:get()
      local v2 = param_saved[pname]
      if v1 ~= v2 then
         p:set_and_save(param_saved[pname])
         count = count + 1
      end
   end
   return count
end

gcs:send_text(MAV_SEVERITY.INFO, string.format("Stored %u parameters", param_count))

local done_revert = false

-- main update function
function update()
   local sw_pos = rc:get_aux_cached(PREV_RC_FUNC:get())
   if not sw_pos then
      return
   end
   if sw_pos == 2 and not done_revert then
      done_revert = true
      count = revert_parameters()
      gcs:send_text(MAV_SEVERITY.INFO, string.format("Reverted %u parameters", count))
      return
   end
   if sw_pos == 0 then
      done_revert = false
   end
end

-- wrapper around update(). This calls update() at 10Hz,
-- and if update faults then an error is displayed, but the script is not
-- stopped
function protected_wrapper()
  local success, err = pcall(update)
  if not success then
     gcs:send_text(MAV_SEVERITY.EMERGENCY, "Internal Error: " .. err)
     -- when we fault we run the update function again after 1s, slowing it
     -- down a bit so we don't flood the console with errors
     return protected_wrapper, 1000
  end
  return protected_wrapper, 1000/UPDATE_RATE_HZ
end

-- start running update loop
return protected_wrapper()
