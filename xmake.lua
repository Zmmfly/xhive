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

includes("compiler")
includes("rules")
includes("plugins")

-- Define xhive_host namespace to include libraries for host builds to provide test isolation
namespace("xhive_host", function()
    includes("libs")
end)

-- Define xhive_embed namespace to include libraries for embedded builds
namespace("xhive_embed", function()
    add_rules("xhive.common")
    includes("libs")
    includes("vendor")
    includes("third-party")

    target("deps")
        set_kind("object")
        add_deps("vendor", {public=true})
    target_end()
end)
