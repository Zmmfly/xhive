target("rt-link")
    set_kind("object")
    on_load(function(target)
        local conf    = target:data("kconfig")
        local sdir    = os.scriptdir()
        local src_dir = path.join(sdir, "src")
        local inc_dir = path.join(sdir, "inc")
        if not conf.RT_USING_RT_LINK then
            return
        end
        target:add("files", path.join(src_dir, "*.c"))
        target:add("includedirs", inc_dir, {public = true})
    end)
