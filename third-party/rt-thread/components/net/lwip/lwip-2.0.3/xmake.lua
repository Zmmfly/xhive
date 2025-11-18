target("lwip-2.0.3")
    set_kind("object")
    set_default(false)
    on_load(function(target)
        local conf = target:data("kconfig")
        local sdir = os.scriptdir()

        if (not conf.RT_USING_LWIP) or (not conf.RT_USING_LWIP203) then
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

        -- SAL
        if conf.RT_USING_SAL then
            target:add("includedirs", path.join(src_dir, "include", "posix"), {public = true})
        end

        -- IPv6
        if conf.RT_USING_LWIP_IPV6 then
            target:add("files", path.join(src_dir, "core", "ipv6", "*.c"))
        end

        -- SNMP
        if conf.RT_LWIP_SNMP then
            target:add("files", path.join(src_dir, "apps", "snmp", "*.c"))
        end

        -- PPP
        if conf.RT_LWIP_PPP then
            target:add("files", path.join(src_dir, "netif", "ppp", "*.c"))
            target:add("files", path.join(src_dir, "netif", "ppp", "polarssl", "*.c"))
            target:add("includedirs", path.join(src_dir, "netif", "ppp"), {public = true})
        end

        -- PING
        if conf.RT_LWIP_USING_PING then
            target:add("files", path.join(src_dir, "apps", "ping", "ping.c"))
        end
    end)
