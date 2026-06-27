--[[
Copyright (c) 2025 Zmmfly. All rights reserved.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors
   may be used to endorse or promote products derived from this software
   without specific prior written permission.
]]

includes("FreeRTOS")
includes("rt-thread")
includes("rttnano")
includes("SEGGER_RTT")

target("third-party")
    set_kind("object")
    on_load(function(target)
        local conf = target:data("kconfig") or {}
        local maps = {
            -- Edit this map to add new third-party dependencies or remove existing ones
            ["THIRD_RTOS_FREERTOS"] = "freertos",
            ["THIRD_RTOS_RTTHREAD"] = "rt-thread::deps",
            ["THIRD_RTOS_RTTNANO"]  = "rttnano",
            ["THIRD_SEGGER_RTT"]    = "SEGGER_RTT",
        }

        -- Add dependencies based on configuration
        for key, dep in pairs(maps) do
            if conf[key] then
                target:add("deps", dep, {public = true})
            end
        end
    end)
