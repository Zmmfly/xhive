rule("xmcu.common")
    before_build(function(target)
        import("xmcu.kconf")
        import("xmcu.preproc")
        import("core.cache.memcache")

        -- Get basic dirs
        local scriptdir = os.scriptdir()
        local projdir   = vformat("$(projectdir)")
        local buildir   = vformat(path.join(projdir, "build"))
        local sdkdir    = vformat(path.directory(scriptdir))

        -- Generate xmcu_config.h by genconfig command
        local header_generated = memcache.get("xmcu", "header_before_build_" .. path.absolute(projdir))
        if not header_generated then
            -- print("Generating xmcu_config.h for target: " .. target:name())
            kconf.genconfig(projdir)
            memcache.set("xmcu", "header_before_build_" .. path.absolute(projdir), true)
        end
        target:add("includedirs", buildir)

        -- Add link scripts
        if target:kind() == "binary" then
            local ld_template = path.join(sdkdir, "templates", "link.ld")
            local ld_output   = path.join(buildir, "link.ld")
            preproc.build_link_script(ld_template, target:tool("cc"), ld_output)
            target:add("ldflags", "-T" .. ld_output)
            printf("Using link script: %s, for target: %s\n", ld_output, target:name())
        end
    end)

    on_load(function(target)
        import("xmcu.kconf")
        import("core.cache.memcache")

        print("Applying xmcu rule to: " .. target:name())

        target:set("plat", "cross")
        target:add("toolchains", "xmcu_toolchain")
        target:add("languages", "c99")

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
            -- print("Generating xmcu_config.h for target: " .. target:name())
            kconf.genconfig(projdir)
            memcache.set("xmcu", "header_on_load_" .. path.absolute(projdir), true)
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
    end)
rule_end()
