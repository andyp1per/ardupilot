--[[
--]]

SCRIPT_NAME = "CRSF Menu"
SCRIPT_NAME_SHORT = "CRSFMenu"
SCRIPT_VERSION = "0.1"


MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
CRSF_EVENT = {PARAMETER_READ=1, PARAMETER_WRITE=2}

local params = {}
local param = CRSFParameter()
param:id(3)
param:data(string.pack("<BBB", 1, 2, 3)) -- pack a string little endian followed by 1,2,3 as unsigned byte
param:length(string.len(params[0]:data()))
params[0] = param

params[1] = CRSFParameter()
params[1]:id(4)
params[1]:data(string.pack("<BBB", 1, 2, 3))
params[1]:length(string.len(params[1]:data()))

local menu = CRSFMenu()
menu:id(1)
menu:init(2)
menu:name('Example Menu')
menu:params(0, params[0])
menu:params(1, params[1])

crsf:add_menu(menu)

gcs:send_text(MAV_SEVERITY.INFO, string.format("Loaded CRSF menu"))

function update()
    local payload = CRSFPayload()
    local param = CRSFParameter()
    if crsf:get_menu_event(CRSF_EVENT.PARAMETER_WRITE, param, payload) & CRSF_EVENT.PARAMETER_WRITE ~= 0 then
        if param:id() == 3 then
            notify:play_tune("L16GGGL4E-L16FFFL4D") -- Beethoven's 5th intro
        end
    end
    return update, 100
end

return update, 5000