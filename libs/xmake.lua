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

includes("cjson")
includes("heatshrink")
includes("micro-ecc")
includes("xxhash")
includes("sha256")
includes("base64")
includes("bitmap")
includes("divsufsort")
includes("llhttp")
includes("mbedtls")

target("libs")
    set_kind("object")
    on_load(function(target)
        local conf = target:data("kconfig") or {}
        local maps = {
            -- Edit this map to add new library dependencies or remove existing ones
            LIBS_ADD_CJSON      = "cjson",
            LIBS_ADD_HEATSHRINK = "heatshrink",
            LIBS_ADD_MICRO_ECC  = "micro-ecc",
            LIBS_ADD_XXHASH     = "xxhash",
            LIBS_ADD_SHA256     = "sha256",
            LIBS_ADD_BASE64     = "base64",
            LIBS_ADD_DIVSUFSORT = "divsufsort",
            LIBS_ADD_BITMAP     = "bitmap",
            LIBS_ADD_LLHTTP     = "llhttp",
            -- LIBS_ADD_MBEDTLS    = "mbedtls", // need explicit declaration
        }

        -- Add dependencies based on configuration
        for key, dep in pairs(maps) do
            if conf[key] then
                target:add("deps", dep)
            end
        end
    end)
