target("posix")
    set_kind("object")
    on_load(function(target)
        local conf = target:data("kconfig")
        local sdir = os.scriptdir()

        -- posix delay
        if conf.RT_USING_POSIX_DELAY then
            local dir = path.join(sdir, "delay")
            target:add("files", path.join(dir, "*.c"))
            target:add("includedirs", dir, {public = true})
        end

        -- posix io
        local io_dir = path.join(sdir, "io")

        local iodir_maps = {
            RT_USING_POSIX_AIO     = "aio",
            RT_USING_POSIX_EPOLL   = "epoll",
            RT_USING_POSIX_EVENTFD = "eventfd",
            RT_USING_POSIX_MMAN    = "mman",
            RT_USING_POSIX_STDIO   = "stdio",
            RT_USING_POSIX_TERMIOS = "termios",
            RT_USING_POSIX_TIMERFD = "timerfd",
        }
        for k, v in pairs(iodir_maps) do
            if conf[k] then
                local dir = path.join(io_dir, v)
                target:add("files", path.join(dir, "*.c"))
                target:add("includedirs", dir, {public = true})
            end
        end

        -- posix io poll/select
        target:add("includedirs", path.join(io_dir, "poll"), {public = true})
        local poll_dir = path.join(io_dir, "poll")
        if conf.RT_USING_POSIX_POLL then
            target:add("files", path.join(poll_dir, "poll.c"))
        end

        if conf.RT_USING_POSIX_SELECT then
            target:add("files", path.join(poll_dir, "select.c"))
        end

        -- posix io signalfd
        if conf.RT_USING_SMART and conf.RT_USING_POSIX_SIGNALFD then
            local dir = path.join(io_dir, "signalfd")
            target:add("files", path.join(dir, "*.c"))
        end

        -- posix ipc
        local ipc_dir = path.join(sdir, "ipc")
        target:add("includedirs", ipc_dir, {public = true})
        if conf.RT_USING_POSIX_MESSAGE_QUEUE and conf.RT_USING_DFS_MQUEUE then
            target:add("files", path.join(ipc_dir, "mqueue.c"))
        end

        if conf.RT_USING_POSIX_MESSAGE_SEMAPHORE then
            target:add("files", path.join(ipc_dir, "semaphore.c"))
        end

        -- posix libdl
        local libdl_dir = path.join(sdir, "libdl")
        if conf.COMPILER_GCC and conf.RT_USING_MODULE then
            target:add("files", path.join(libdl_dir, "*.c"))
            target:add("files", path.join(libdl_dir, "arch", "*.c"))
            target:add("includedirs", libdl_dir, {public = true})
        end

        -- posix pthreads
        local pthreads_dir = path.join(sdir, "pthreads")
        if conf.RT_USING_PTHREADS then
            target:add("files", path.join(pthreads_dir, "*.c"))
            target:add("includedirs", pthreads_dir, {public = true})
        end

        -- posix signal
        local signal_dir = path.join(sdir, "signal")
        if conf.RT_USING_SIGNALS and conf.RT_USING_PTHREADS then
            target:add("files", path.join(signal_dir, "*.c"))
            target:add("includedirs", signal_dir, {public = true})
        end

        -- posix tls
        local tls_dir = path.join(sdir, "tls")
        if conf.RT_USING_PTHREADS then
            target:add("files", path.join(tls_dir, "*.c"))
            target:add("includedirs", tls_dir, {public = true})
        end
    end)
