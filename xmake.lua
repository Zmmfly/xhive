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

add_moduledirs("modules")
add_moduledirs("native_modules")

includes("compiler")
includes("rules")
includes("plugins")

-- The SDK is also a native development project. Including it from an
-- application keeps the application's Kconfig and embedded target graph.
local standalone = path.normalize(os.projectdir()) == path.normalize(os.scriptdir())

option("xhive_tests")
    set_default(true)
    set_showmenu(true)
    set_description("Build and register xhive host tests (requires GoogleTest)")
option_end()

if standalone then
    set_project("xhive")
    add_rules("mode.debug", "mode.release")
end

namespace("xhive_host", function()
    if standalone then
        -- Use the libraries' upstream host defaults, not a board .config.
        -- In particular, do not define BUILD_IN_XHIVE or generate a header.
        add_values("xhive.standalone", true)
    else
        add_rules("xhive.conf")
    end
    includes("libs")
    if has_config("xhive_tests") then
        includes("libs/*/tests.lua")
    end
    target("deps")
        set_kind("object")
        add_deps("libs", {public = true})
    target_end()
end)

if standalone then
    includes("tests")
else
    namespace("xhive_embed", function()
        add_rules("xhive.embed")
        includes("libs")
        includes("vendor")
        includes("third-party")

        target("deps")
            set_kind("object")
            add_deps("libs", {public = true})
            add_deps("vendor", {public = true})
            add_deps("third-party", {public = true})
        target_end()
    end)
end

