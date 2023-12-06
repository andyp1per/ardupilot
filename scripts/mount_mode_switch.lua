-- mount_mode_switch.lua: control FPV lock with a radio switch
--

-- constants
local AuxSwitchPos = {LOW=0, MIDDLE=1, HIGH=2}
local servo9_function = Parameter("SERVO9_FUNCTION")
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

-- state
local prev_pos = -1

function update()
    local sw_pos = rc:get_aux_cached(300)
    if sw_pos ~= prev_pos then
        if sw_pos == AuxSwitchPos.LOW then
            servo9_function:set(56)
            gcs:send_text(MAV_SEVERITY.INFO, "FPV Locked Mode")
        else
            servo9_function:set(13)
            gcs:send_text(MAV_SEVERITY.INFO, "FPV Stabilize Mode")
        end
        prev_pos = sw_pos
    end
    return update, 1000
end

return update, 5000
