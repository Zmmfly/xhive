target("lwip-1.4.1")
    set_kind("object")
    on_load(function(target)
        local conf = target:data("kconfig")
        local sdir = os.scriptdir()

        if (not conf.RT_USING_LWIP) or (not conf.RT_USING_LWIP141) then
            return
        end

        local src_dir = path.join(sdir, "src")

        target:add("includedirs", path.join(src_dir, "include"), {public = true})
        target:add("includedirs", path.join(src_dir, "include", "ipv4"), {public = true})
        target:add("includedirs", path.join(src_dir, "include", "ipv5"), {public = true})

        
        target:add("files", path.join(src_dir, "api", "*.c"))
        target:add("files", path.join(src_dir, "core", "*.c"))
        target:add("files", path.join(src_dir, "core", "ipv4", "*.c"))
        target:add("files", path.join(src_dir, "netif", "*.c"))

        -- SNMP
        if conf.RT_LWIP_SNMP then
            target:add("files", path.join(src_dir, "core", "snmp", "*.c"))
        end

        -- PPP
        if conf.RT_LWIP_PPP then
            target:add("files", path.join(src_dir, "netif", "ppp", "*.c"))
            target:add("includedirs", path.join(src_dir, "netif", "ppp"), {public = true})
        end

        -- PING
        if conf.RT_LWIP_USING_PING then
            target:add("files", path.join(src_dir, "apps", "ping", "ping.c"))
        end
    end)
