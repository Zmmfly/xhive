target("net_utest")
    set_kind("object")
    set_default(false)
    on_load(function(target)
        local conf = target:data("kconfig")
        local sdir = os.scriptdir()

        local test_maps = {
            RT_UTEST_TC_USING_LWIP   = "tc_lwip.c",
            RT_UTEST_TC_USING_NETDEV = "tc_netdev.c",
            RT_UTEST_TC_USING_SAL    = "tc_sal_socket.c",
        }

        for k, v in pairs(test_maps) do
            if conf[k] then
                target:add("files", path.join(sdir, "src", v))
            end
        end
    end)
