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

includes("dfs")
includes("drivers")
includes("fal")
includes("finsh")
includes("libc")
-- FIXME: lwp need more work to support it
-- includes("lwp")
includes("mm")
includes("mprotect")
includes("net")
includes("utilities")
includes("vbus")

target("components")
    set_kind("object")
    add_deps("dfs", {public=true})
    add_deps("drivers", {public=true})
    add_deps("fal", {public=true})
    add_deps("finsh", {public=true})
    add_deps("libc", {public=true})
    -- FIXME: lwp need more work to support it
    -- add_deps("lwp", {public=true})
    add_deps("mm", {public=true})
    add_deps("mprotect", {public=true})
    add_deps("net", {public=true})
    add_deps("utilities", {public=true})
    add_deps("vbus", {public=true})
