target("mprotect")
    set_kind("object")
    set_default(false)
    on_load(function (target)
        local conf = target:data("kconfig")
        local sdir = os.scriptdir()
        if not conf.RT_USING_MEM_PROTECTION then
            return
        end
        target:add("files", path.join(sdir, "*.c"))
        target:add("includedirs", sdir, {public = true})
    end)
