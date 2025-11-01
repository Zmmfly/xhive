task("xmcu_install")
    set_category("action")
    set_menu {
        usage = "xmake xmcu_install",
        description = "Install required toolchain for xmcu project.",
        options = {
            {}
        }
    }
    on_run(function ()
        import("xmcu.base")
        import("xmcu.kconf")
        import("xmcu.toolset")
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

toolchain("xmcu_toolchain")
    set_kind("standalone")

    on_load(function(toolchain)
        import("xmcu.base")
        import("xmcu.kconf")
        import("xmcu.toolset")
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
        import("xmcu.base")
        import("xmcu.kconf")
        import("xmcu.toolset")
        import("core.project.config")
        import("detect.sdks.find_cross_toolchain")

        local infos = toolset.load_tool_infos()
        if not infos.ready then
            raise("Please run 'xmake xmcu_install' to install the required toolchain: " .. infos.full_name)
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
