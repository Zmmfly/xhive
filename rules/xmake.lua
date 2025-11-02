rule("xmcu.common")
    before_build(function(target)
        import("xmcu.kconf")
        import("xmcu.proc")
        import("core.cache.memcache")

        -- Get basic dirs
        local scriptdir = os.scriptdir()
        local projdir   = vformat("$(projectdir)")
        local buildir   = vformat(path.join(projdir, "build"))
        local sdkdir    = vformat(path.directory(scriptdir))

        -- Generate xmcu_config.h by genconfig command
        local header_generated = memcache.get("xmcu", "header_before_build_" .. path.absolute(projdir))
        if not header_generated then
            -- print("Generating xmcu_config.h for target: " .. target:name() .. " before build")
            kconf.genconfig(projdir)
            memcache.set("xmcu", "header_before_build_" .. path.absolute(projdir), true)
        else
            -- print("xmcu_config.h already generated for target: " .. target:name() .. " before build")
        end
        target:add("includedirs", buildir)

        -- Add link scripts
        if target:kind() == "binary" then
            local ld_template = path.join(sdkdir, "templates", "link.ld")
            local ld_output   = path.join(buildir, "link.ld")
            proc.build_link_script(ld_template, target:tool("cc"), ld_output)
            target:add("ldflags", "-T" .. ld_output)
            -- printf("Using link script: %s, for target: %s\n", ld_output, target:name())
        end
    end)

    after_link(function(target) 
        local conf = target:data("kconfig")
        if target:kind() == "binary" then
            import("xmcu.toolset")
            local elf_path = target:targetfile()
            local cc_path  = target:tool("cc")
            local bin_path = path.join(path.directory(elf_path), path.basename(elf_path) .. ".bin")

            -- Convert ELF to BIN
            toolset.elf_to_bin(cc_path, elf_path, bin_path)
            print("Generated binary file: " .. bin_path)
            
            -- Display binary size in KB with 2 decimal places
            local bin_size = os.filesize(bin_path) or 0
            print(string.format("Binary size: %d bytes ~= %.2f KB / %d, used %5.2f%%", bin_size, bin_size / 1024, conf.ROM_LENGTH, bin_size / conf.ROM_LENGTH * 100))

            local ram_used = toolset.elf_ram_usage(cc_path, elf_path)
            print(string.format("RAM used: %d bytes / %d, used %5.2f%%", ram_used, conf.RAM_LENGTH, ram_used / conf.RAM_LENGTH * 100))
        end
    end)

    on_load(function(target)
        import("xmcu.kconf")
        import("core.cache.memcache")

        --[[
            This section is to ensure that a Kconfig file exists in the build directory. 
            If it doesn't exist or content not same, it generates one that sources the local Kconfig file. 
         ]]

        -- Get buildir
        local scriptdir = os.scriptdir()
        local projdir   = vformat("$(projectdir)")
        local buildir   = vformat(path.join(projdir, "build"))
        local sdkdir    = path.directory(vformat(scriptdir))

        kconf.build(projdir, sdkdir, buildir)

        --[[
            This section is to parse .config file
         ]]
        local conf = memcache.get("xmcu", "kconfig")
        local dot_config_path = path.join(projdir, kconf.config_name())
        if os.isfile(dot_config_path) then
            -- use cached kconfig if available
            if conf then
                target:data_add("kconfig", conf)
            else
                -- parse .config, add to target data and cache it
                conf = kconf.parse_cached(dot_config_path)
                target:data_add("kconfig", conf)
                memcache.set("xmcu", "kconfig", conf)
            end
        else
            print("Warning: " .. kconf.config_name() .. " file not found at: " .. dot_config_path)
            print("Please run 'xmake menuconfig' to config your project.")
            return
        end

        --[[ 
            This section to build header
         ]]
                -- Generate xmcu_config.h by genconfig command
        local header_generated = memcache.get("xmcu", "header_on_load_" .. path.absolute(projdir))
        if not header_generated then
            -- print("Generating xmcu_config.h for target: " .. target:name() .. " on load")
            kconf.genconfig(projdir)
            memcache.set("xmcu", "header_on_load_" .. path.absolute(projdir), true)
        else
            -- print("xmcu_config.h already generated for target: " .. target:name() .. " on load")
        end

        --[[ 
            This section is to add common include directories and compiler related
         ]]

        -- Add common include directories
        if conf.COMMON_INCLUDES and conf.COMMON_INCLUDES ~= "" then
            local include_dirs = base.get_commas_paths(conf.COMMON_INCLUDES)
            for _, dir in ipairs(include_dirs) do
                target:add("includedirs", dir)
            end
        end

        --[[ 
            This section is to set basic target properties
         ]]
        target:set("plat", "cross")
        target:add("toolchains", "xmcu_toolchain")
        target:add("languages", "c99")

        if target:kind() == "binary" then
            target:set("extension", ".elf")
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
