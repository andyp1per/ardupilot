local PARAM_TABLE_KEY = 35
local PARAM_TABLE_PREFIX = "SNTCH"

local notches = {}

param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 4)

for idx = 1, 4, 1 do
    param:add_param(PARAM_TABLE_KEY, idx, idx .. "_FREQ", 0)
    notches[idx-1] = Parameter(PARAM_TABLE_PREFIX .. idx .. "_FREQ")
end


function update()
    for n = 0, 1, 1 do
        ---@class HarmonicNotch_ud
        local notch = ins:get_harmonic_notch(n)

        if notch == nil then
            goto continue
        end
        local nfreqs = 0
        for idx = 0, 3, 1 do
            if notches[idx] ~= nil and notches[idx]:get() ~= nil then
                local f = notches[idx]:get()
                if f ~= 0 then
                    if f ~= notch:get_frequency(idx) then
                        print("HNTC" .. n+1 .."(" .. idx + 1 .. ") set to " .. f .. " (was " .. notch:get_frequency(idx) .. ")")
                        notch:set_frequency(idx, f)
                    end
                    nfreqs = nfreqs + 1
                end
            end
        end
        notch:set_num_frequencies(nfreqs)
        ::continue::
    end
    return update, 500
end

return update()
