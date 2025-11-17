target("utest")
    set_kind("object")
    add_deps("finsh", {public=true})
    on_load(function(target)
        local conf     = target:data("kconfig")
        local sdir     = os.scriptdir()
        local self_dir = path.join(sdir, "utest")

        if not conf.RT_USING_UTEST then
            return
        end

        target:add("files", path.join(sdir, "*.c"))
        target:add("includedirs", sdir, {public = true})

        if conf.RT_UTEST_SELF_PASS then
            target:add("files", path.join(self_dir, "TC_*.c"))
        end
    end)
