target("resource")
    set_kind("object")
    on_load(function(target)
        local conf = target:data("kconfig")
        local sdir = os.scriptdir()

        if not conf.RT_USING_RESOURCE_ID then
            return
        end

        target:add("files", path.join(sdir, "resource_id.c"))
        if conf.RT_USING_ADT_BITMAP then
            target:add("files", path.join(sdir, "rid_bitmap.c"))
        end
        target:add("includedirs", sdir, {public = true})
    end)
