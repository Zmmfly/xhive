target("lwip-dhcpd")
    set_kind("object")
    set_default(false)
    on_load(function(target) 
        local conf = target:data("kconfig")
        local sdir = os.scriptdir()

        if (not conf.RT_USING_LWIP) or (not conf.LWIP_USING_DHCPD) then
            return
        end

        if conf.RT_USING_LWIP141 then
            target:add("files", path.join(sdir, "dhcp_server.c"))
        else
            target:add("files", path.join(sdir, "dhcp_server_raw.c"))
        end
    end)
