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

rule("xhive.common")
    before_build(function(target)
        import("xhive.kconf")
        import("xhive.proc")
        import("xhive.base")
        import("core.cache.memcache")

        -- Get dirs
        local dirs = base.load_paths()

        -- Generate xhive_config.h by genconfig command
        local header_generated = memcache.get("xhive", "header_before_build_" .. dirs.prjdir)
        if not header_generated then
            -- print("Generating xhive_config.h for target: " .. target:name() .. " before build")
            kconf.build_header(dirs.prjdir)
            memcache.set("xhive", "header_before_build_" .. path.absolute(dirs.prjdir), true)
        else
            -- print("xhive_config.h already generated for target: " .. target:name() .. " before build")
        end
        target:add("includedirs", dirs.builddir)

        -- Get kconfig
        local conf = target:data("kconfig")

        -- Add link scripts
        if target:kind() == "binary" then
            local ld_template = ""
            if conf.USE_DEFAULT_LD_SCRIPT then
                ld_template = path.join(dirs.sdkdir, "templates", "link.ld")
            else
                ld_template = path.normalize(path.join(dirs.prjdir, conf.CUSTOM_LD_SCRIPT_PATH))
            end

            if not os.isfile(ld_template) then
                raise("Linker script not found: " .. ld_template .. " for target: " .. target:name())
            end

            local ld_output   = path.join(dirs.builddir, "link.ld")
            proc.build_link_script(ld_template, target:tool("cc"), ld_output)
            target:add("ldflags", "-T" .. ld_output, {force = true})
            -- printf("Using link script: %s, for target: %s\n", ld_output, target:name())
        end
    end)

    after_link(function(target) 
        local conf = target:data("kconfig")
        if target:kind() == "binary" then
            import("xhive.toolset")
            local elf_path = target:targetfile()
            local cc_path  = target:tool("cc")
            local bin_path = path.join(path.directory(elf_path), path.basename(elf_path) .. ".bin")

            -- Convert ELF to BIN
            toolset.elf_to_bin(cc_path, elf_path, bin_path)
            print("Generated binary file: " .. bin_path)
            
            -- Display binary size in KB with 2 decimal places
            local bin_size    = os.filesize(bin_path) or 0
            local rom_len     = conf.ROM_LENGTH * 1024
            local rom_used_kb = bin_size / 1024
            local rom_usage   = bin_size / rom_len * 100
            print(string.format("ROM used: %7d / %7d, ~ %5.2f%%, ~ %6.2f KB", bin_size, rom_len, rom_used_kb, rom_usage))

            local ram_used = toolset.elf_ram_usage(cc_path, elf_path)
            local ram_len = conf.RAM_LENGTH * 1024
            local ram_used_kb = ram_used / 1024
            local ram_usage = ram_used / ram_len * 100
            print(string.format("RAM used: %7d / %7d, ~ %5.2f%%, ~ %6.2f KB", ram_used, ram_len, ram_usage, ram_used_kb))
        end
    end)

    on_load(function(target)
        import("xhive.kconf")
        import("xhive.base")
        import("core.cache.memcache")

        --[[
            This section is to ensure that a Kconfig file exists in the build directory. 
            If it doesn't exist or content not same, it generates one that sources the local Kconfig file. 
         ]]

        -- Get buildir
        local dirs      = base.load_paths()
        local scriptdir = os.scriptdir()

        kconf.build_entry()

        --[[
            This section is to parse .config file
         ]]
        local conf = memcache.get("xhive", "kconfig")
        local dot_config_path = path.join(dirs.prjdir, kconf.load_name())
        if os.isfile(dot_config_path) then
            -- use cached kconfig if available
            if conf then
                target:data_add("kconfig", conf)
            else
                -- parse .config, add to target data and cache it
                conf = kconf.parse_cached(dot_config_path)
                target:data_add("kconfig", conf)
                memcache.set("xhive", "kconfig", conf)
            end
        else
            print("Warning: " .. kconf.load_name() .. " file not found at: " .. dot_config_path)
            print("Please run 'xmake menuconfig' to config your project.")
            return
        end

        --[[ 
            This section to build header
         ]]
                -- Generate xhive_config.h by genconfig command
        local header_generated = memcache.get("xhive", "header_on_load_" .. dirs.prjdir)
        if not header_generated then
            -- print("Generating xhive_config.h for target: " .. target:name() .. " on load")
            kconf.build_header(dirs.prjdir)
            memcache.set("xhive", "header_on_load_" .. dirs.prjdir, true)
        else
            -- print("xhive_config.h already generated for target: " .. target:name() .. " on load")
        end

        --[[ 
            This section is to add common include directories and compiler related
         ]]

        -- Add common include directories
        if conf.COMMON_INCLUDES and conf.COMMON_INCLUDES ~= "" then
            local include_dirs = base.split_commas_paths(conf.COMMON_INCLUDES)
            for _, dir in ipairs(include_dirs) do
                target:add("includedirs", dir)
            end
        end

        --[[ 
            This section is to set basic target properties
         ]]
        target:set("plat", "cross")
        target:add("toolchains", "xhive_toolchain")

        -- Set target extension, add deps for binary targets
        if target:kind() == "binary" then
            target:set("extension", ".elf")
        end

        -- Add xhive_embed::deps if target scriptdir not under sdkdir
        local target_scriptdir = path.normalize(target:scriptdir())
        if not target_scriptdir:startswith(dirs.sdkdir) then
            target:add("deps", "xhive_embed::deps")
        end

        if conf.OPTIMIZE_NONE then
            target:set("optimize", "none")
        elseif conf.OPTIMIZE_FAST then 
            target:set("optimize", "fast")
        elseif conf.OPTIMIZE_FASTER then 
            target:set("optimize", "faster")
        elseif conf.OPTIMIZE_FASTEST then 
            target:set("optimize", "fastest")
        elseif conf.OPTIMIZE_SMALLEST then
            target:set("optimize", "smallest")
        elseif conf.OPTIMIZE_AGGRESSIVE then
            target:set("optimize", "aggressive")
        else
            raise("No optimization level selected in kconfig for target: " .. target:name())
        end
        target:set("symbols", "debug")
    end)
rule_end()
