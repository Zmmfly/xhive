target("ymodem")
    set_kind("object")
    set_default(false)
    on_load(function(target)
        local conf = target:data("kconfig")
        local sdir = os.scriptdir()

        if conf.RT_USING_RYM then
            target:add("files", path.join(sdir, "ymodem.c"))
            target:add("includedirs", self_dir, {public = true})

            if conf.YMODEM_USING_FILE_TRANSFER then
                target:add("files", path.join(sdir, "ry_sy.c"))
            end
        end
    end)
