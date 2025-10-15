task("menuconfig")
    set_category("action")
    set_menu {
        usage = "xmake menuconfig",
        description = "Run menuconfig to configure project.",
        options = {
            {}
        }
    }
    on_run(function ()
        import("xmcu.kconf")
        import("xmcu.cross_flags")
        local scriptdir    = os.scriptdir()
        local projdir      = vformat("$(projectdir)")
        local buildir      = vformat(path.join(projdir, "build"))
        local sdkdir       = vformat(path.join(scriptdir, ".."))
        local kconfig_path = path.join(buildir, "Kconfig")
        kconf.build(projdir, sdkdir, buildir)
        
        os.setenv("KCONFIG_CONFIG", path.join(projdir, ".config"))
        os.exec("menuconfig " .. kconfig_path)
    end)
task_end()

rule("xmcu.common")
    before_build(function(target)
        import("xmcu.kconf")
        import("xmcu.cross_flags")
        import("core.cache.memcache")

        local scriptdir = os.scriptdir()
        local projdir   = vformat("$(projectdir)")
        local buildir   = vformat(path.join(projdir, "build"))
        local sdkdir    = vformat(path.join(scriptdir, ".."))

        -- Generate xmcu_config.h by genconfig command
        local header_generated = memcache.get("xmcu", "header_generated_" .. path.absolute(projdir))
        if not header_generated then
            -- print("Generating xmcu_config.h for target: " .. target:name())
            kconf.genconfig(projdir)
            memcache.set("xmcu", "header_generated_" .. path.absolute(projdir), true)
        end
    end)

    on_load(function(target)
        import("xmcu.kconf")
        import("xmcu.cross_flags")
        import("core.cache.memcache")

        print("Applying xmcu.common rule to target: " .. target:name())
        --[[
            This section is to ensure that a Kconfig file exists in the build directory. 
            If it doesn't exist or content not same, it generates one that sources the local Kconfig file. 
         ]]

        -- Get buildir
        local scriptdir = os.scriptdir()
        local projdir   = vformat("$(projectdir)")
        local buildir   = vformat(path.join(projdir, "build"))
        local sdkdir    = vformat(path.join(scriptdir, ".."))

        kconf.build(projdir, sdkdir, buildir)

        --[[
            This section is to parse .config file
         ]]
        local conf = {}
        local dot_config_path = path.join(projdir, ".config")
        if os.isfile(dot_config_path) then
            conf = kconf.parse(dot_config_path)
            target:data_add("kconfig", conf)
        else
            print("Warning: .config file not found at: " .. dot_config_path)
            print("Please run 'xmake menuconfig' to config your project.")
            return
        end

        --[[ 
            This section is to build xmcu_config.h file in build directory
         ]]
        local header_path = path.join(buildir, "xmcu_config.h")

        --[[
            This section is to add build flags
         ]]
        -- get table by

        --[[
            可选：将解析结果保存到 target 的私有数据中
            这样在其他地方可以通过 target:data("kconfig") 获取
            注意：这是 xmake 的内部API，未来可能发生变化

            target:data_set("kconfig", conf)

            然后在项目的 on_config 中可以这样使用：
            local config = target:data("kconfig")
            if config and config.ENABLE_FEATURE_A then
                add_files("src/feature_a/*.c")
            end
         ]]

        --[[
            Optional: Save parsed configuration to target's private data
            This allows accessing it later via target:data("kconfig")
            Note: This is an internal xmake API and may change in future versions

            target:data_set("kconfig", conf)

            Then in project's on_config, you can use it like:
            local config = target:data("kconfig")
            if config and config.ENABLE_FEATURE_A then
                add_files("src/feature_a/*.c")
            end
         ]] 
    end)
rule_end()
