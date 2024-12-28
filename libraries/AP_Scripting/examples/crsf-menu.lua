--[[
--]]

SCRIPT_NAME = "CRSF Menu"
SCRIPT_NAME_SHORT = "CRSFMenu"
SCRIPT_VERSION = "0.1"


MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local params = {}
params[0] = CRSFParameter()
params[0]:id(3)
params[0]:data(string.pack("<BBB", 1, 2, 3))
params[0]:length(string.len(params[0]:data()))
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
    return update, 1000
end

return update, 5000