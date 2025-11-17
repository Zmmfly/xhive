target("var_export")
    set_kind("object")
    on_load(function(target)
        local conf = target:data("kconfig")
        local sdir = os.scriptdir()

        if conf.RT_USING_VAR_EXPORT then
            target:add("files", path.join(sdir, "*.c"))
            target:add("includedirs", sdir, {public = true})
        end
    end)
