target("netdev")
    set_kind("object")
    set_default(false)
    on_load(function(target) 
        local conf = target:data("kconfig")
        local sdir = os.scriptdir()
        if not conf.RT_USING_NETDEV then
            return
        end
        target:add("files", path.join(sdir, "src", "*.c"))
        target:add("includedirs", path.join(sdir, "include"), {public = true})

    end)
