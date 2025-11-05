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

task("xhive_install")
    set_category("action")
    set_menu {
        usage = "xmake xhive_install",
        description = "Install required toolchain for xhive project.",
        options = {
            {}
        }
    }
    on_run(function ()
        import("xhive.base")
        import("xhive.kconf")
        import("xhive.toolset")
        import("core.base.json")
        import("core.cache.memcache")
        
        local tool_infos = toolset.load_tool_infos()
        print(tool_infos)

        if not tool_infos.ready then
            toolset.download_tool(tool_infos)
        else
            print("Toolchain " .. tool_infos.full_name .. " is already installed.")
        end
        
        print("Install toolchain success")
    end)
task_end()

toolchain("xhive_toolchain")
    set_kind("standalone")

    on_load(function(toolchain)
        import("xhive.base")
        import("xhive.kconf")
        import("xhive.toolset")
        import("core.project.config")
        import("core.cache.localcache")

        toolchain:load_cross_toolchain()

        -- load configs
        local conf = kconf.load_configs()
        if conf.COMPILER_CLANG then
            toolchain:set("toolset", "cc", "clang")
            toolchain:set("toolset", "cxx", "clang++")
            toolchain:set("toolset", "ld", "clang++")
            toolchain:set("toolset", "as", "clang")
            toolchain:set("toolset", "ar", "llvm-ar")
            toolchain:set("toolset", "strip", "llvm-strip")
        end

        local flags = toolset.build_toolchain_flags(conf)
        if flags.cxflags then
            toolchain:add("cxflags", flags.cxflags)
        end

        if flags.asflags then
            toolchain:add("asflags", flags.asflags)
        end

        if flags.ldflags then
            toolchain:add("ldflags", flags.ldflags)
        end

        -- force recheck for switching toolchain
        toolchain:config_set("__checked", false)
        localcache.set("config", "recheck", true)
        localcache.save()
    end)

    on_check(function(toolchain)
        import("xhive.base")
        import("xhive.kconf")
        import("xhive.toolset")
        import("core.project.config")
        import("detect.sdks.find_cross_toolchain")

        local infos = toolset.load_tool_infos()
        if not infos.ready then
            raise("Please run 'xmake xhive_install' to install the required toolchain: " .. infos.full_name)
        end

        -- find cross toolchain from external envirnoment
        local cross_toolchain = find_cross_toolchain(infos.tool_dir)
        if not cross_toolchain then
            raise("cannot find cross toolchain from %s", infos.tool_dir or "")
        end

        toolchain:config_set("cross", cross_toolchain.cross)
        toolchain:config_set("bindir", cross_toolchain.bindir)
        toolchain:config_set("sdkdir", cross_toolchain.sdkdir)
        return cross_toolchain
    end)
toolchain_end()
