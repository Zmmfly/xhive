target("vbus")
    set_kind("object")
    on_load(function(target)
        local conf = target:data("kconfig")
        local sdir = os.scriptdir()

        if conf.RT_USING_VBUS then
            target:add("files", path.join(sdir, "*.c"))
            target:add("includedirs", sdir, {public = true})
            target:add("includedirs", path.join(sdir, "share_hdr"), {public = true})
        end
    end)
