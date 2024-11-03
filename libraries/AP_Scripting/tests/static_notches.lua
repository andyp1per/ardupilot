local PARAM_TABLE_KEY = 33
local PARAM_TABLE_PREFIX = "SNTCH"

local notches = {}

param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 5)

for idx = 1, 4, 1 do
    param:add_param(PARAM_TABLE_KEY, idx, idx .. "_FREQ", 0)
    notches[idx-1] = Parameter(PARAM_TABLE_PREFIX .. idx .. "_FREQ")
end


function update()
    ---@class HarmonicNotch_ud
    local notch = ins:get_harmonic_notch(0)

    if notch == nil then
        return update, 100
    end
    local nfreqs = 0
    for idx = 0, 4, 1 do
        if notches[idx] ~= nil and notches[idx]:get() ~= nil then
            local f = notches[idx]:get()
            if f ~= 0 then
                if f ~= notch:get_frequency(idx) then
                    print("Notch " .. idx + 1 .. " set to " .. f .. " (was " .. notch:get_frequency(idx) .. ")")
                    notch:set_frequency(idx, f)
                end
                nfreqs = nfreqs + 1
            end
        end
    end
    notch:set_num_frequencies(nfreqs)
    return update, 100
end

return update()
