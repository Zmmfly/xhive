target("libadt")
    set_kind("object")
    on_load(function(target)
        local conf = target:data("kconfig")
        local sdir = os.scriptdir()

        if not conf.RT_USING_ADT then
            return
        end

        -- avl
        if conf.RT_USING_ADT_AVL then
            target:add("files", path.join(sdir, "avl", "*.c"))
            target:add("includedirs", path.join(sdir, "avl"), {public = true})
        end

        -- bitmap
        if conf.RT_USING_ADT_BITMAP then
            target:add("includedirs", path.join(sdir, "bitmap"), {public = true})
        end

        -- hashmap
        if conf.RT_USING_ADT_HASHMAP then
            target:add("includedirs", path.join(sdir, "hashmap"), {public = true})
        end

        -- ref
        if conf.RT_USING_ADT_REF then
            target:add("includedirs", path.join(sdir, "ref"), {public = true})
        end

        -- uthash
        target:add("files", path.join(sdir, "uthash", "*.c"))
        target:add("includedirs", path.join(sdir, "uthash"), {public = true})
    end)
