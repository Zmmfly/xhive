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
