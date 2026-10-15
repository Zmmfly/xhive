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

target("heatshrink")
    set_kind("object")
    set_default(false)
    add_files("src/*.c")
    add_includedirs("inc", {public = true})
    local library_scope = interp_get_scopename()
target_end()

target("heatshrink_cmd")
    set_kind("binary")
    set_default(false)
    add_deps("heatshrink")
    add_files("heatshrink.c")

-- Host-only test support: the same sources are also built in
-- external-buffer mode so the runtime-configurable window/lookahead
-- path (heatshrink_encoder_init / heatshrink_decoder_init) is covered.
local standalone = path.normalize(os.projectdir()) == path.normalize(os.scriptdir())
if is_plat("linux", "windows", "macosx") and
   ((standalone and library_scope == "heatshrink") or
    (library_scope == "xhive_host::heatshrink" and has_config("xhive_tests"))) then
    -- With the search index (default) and without, covering both
    -- HEATSHRINK_USE_INDEX settings of the new init path.
    target("heatshrink_external")
        set_kind("object")
        set_default(false)
        add_files("src/*.c")
        add_includedirs("inc", {public = true})
        add_defines("HEATSHRINK_DYNAMIC_ALLOC=0",
            "HEATSHRINK_EXTERNAL_BUFFERS=1", {public = true})
    target_end()

    target("heatshrink_external_noindex")
        set_kind("object")
        set_default(false)
        add_files("src/*.c")
        add_includedirs("inc", {public = true})
        add_defines("HEATSHRINK_DYNAMIC_ALLOC=0",
            "HEATSHRINK_EXTERNAL_BUFFERS=1", "HEATSHRINK_USE_INDEX=0",
            {public = true})
    target_end()

    add_requires("gtest")
    for _, file in ipairs(os.files("test/**.cc")) do
        local relative = path.relative(file, "test")
        if relative ~= "main.cc" then
            local suffix = relative:gsub("%.cc$", ""):gsub("[/\\]", "_")
            target("heatshrink_test_" .. suffix)
                set_kind("binary")
                set_default(false)
                set_languages("c++17")
                add_files(file, "test/main.cc")
                add_deps("heatshrink_external")
                add_packages("gtest")
                add_tests(path.basename(file), {
                    group = "xhive",
                    realtime_output = false,
                    run_timeout = 120 * 1000
                })
            target_end()
        end
    end

    -- Checked-in C consumer verifies the USE_INDEX=0 build of the new
    -- init path without GoogleTest.
    target("heatshrink_test_noindex_roundtrip")
        set_kind("binary")
        set_default(false)
        set_languages("c99")
        add_files("test/compile/noindex_roundtrip.c")
        add_deps("heatshrink_external_noindex")
        add_tests("noindex_roundtrip", {
            group = "xhive",
            realtime_output = false,
            run_timeout = 120 * 1000
        })
    target_end()
end
