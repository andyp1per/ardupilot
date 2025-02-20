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
-- pack a string big endian followed by 12 (info type), a name and a value
param:data(string.pack(">Bzz", 12, "Menu Item 1", "It goes here"))
gcs:send_text(MAV_SEVERITY.INFO, "wrote string of length " .. string.len(param:data()))
param:length(26)
params[0] = param

params[1] = CRSFParameter()
params[1]:id(4)
params[1]:data(string.pack(">Bzz", 12, "Menu Item 2", "Another one"))
params[1]:length(25)

local menu = CRSFMenu(2)
menu:name('Example Menu')
menu:params(0, params[0])
menu:params(1, params[1])

crsf:add_menu(menu)

gcs:send_text(MAV_SEVERITY.INFO, string.format("Loaded CRSF menu"))

function update()
    local param, payload, events = crsf:get_menu_event(CRSF_EVENT.PARAMETER_WRITE)
    if (events & CRSF_EVENT.PARAMETER_WRITE) ~= 0 then
        gcs:send_text(MAV_SEVERITY.INFO, "Parameter write " .. param:id())
        if param:id() == 3 then
            notify:play_tune("L16GGGL4E-L16FFFL4D") -- Beethoven's 5th intro
        end
    end
    return update, 100
end

return update, 5000