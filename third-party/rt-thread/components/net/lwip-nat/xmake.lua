target("lwip-nat")
    set_kind("object")
    set_default(false)
    on_load(function(target) 
        local conf = target:data("kconfig")
        local sdir = os.scriptdir()

        if (not conf.RT_USING_LWIP) or (not conf.LWIP_USING_NAT) then
            return
        end

        target:add("files", path.join(sdir, "lwip_nat.c"))
        target:add("includedirs", sdir, {public = true})
    end)
