target("sal")
    set_kind("object")
    on_load(function(target) 
        local conf = target:data("kconfig")
        local sdir = os.scriptdir()

        if not conf.RT_USING_SAL then
            return
        end
        target:add("files", path.join(sdir, "socket", "net_netdb.c"))
        target:add("includedirs", path.join(sdir, "include"))
        target:add("includedirs", path.join(sdir, "include/socket"))

        if conf.SAL_USING_LWIP or conf.SAL_USING_AT then
            target:add("files", path.join(sdir, "impl"))
        end

        if conf.SAL_USING_LWIP then
            target:add("files", path.join(sdir, "impl", "af_inet_lwip.c"))
        end

        if conf.SAL_USING_AT then
            target:add("files", path.join(sdir, "impl", "af_inet_at.c"))
        end

        if conf.SAL_USING_TLS then
            target:add("files", path.join(sdir, "impl", "proto_mbedtls.c"))
        end

        if conf.SAL_USING_POSIX then
            target:add("includedirs", path.join(sdir, "include", "dfs_net"))
            target:add("files", path.join(sdir, "socket", "net_sockets.c"))
            target:add("files", path.join(sdir, "dfs_net", "*.c"))
        end

        if conf.HAVE_SYS_SOCKET_H then
            target:add("includedirs", path.join(sdir, "include", "socket", "sys_socket"))
        end
    end)
