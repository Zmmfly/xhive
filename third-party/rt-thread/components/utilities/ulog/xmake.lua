target("ulog")
    set_kind("object")
    set_default(false)
    on_load(function(target)
        local conf        = target:data("kconfig")
        local sdir        = os.scriptdir()
        local backend_dir = path.join(sdir, "backend")
        local syslog_dir  = path.join(sdir, "syslog")

        if not conf.RT_USING_ULOG then
            return
        end

        if conf.ULOG_BACKEND_USING_CONSOLE then
            target:add("files", path.join(backend_dir, "console_be.c"))
        end

        if conf.ULOG_BACKEND_USING_FILE then
            target:add("files", path.join(backend_dir, "file_be.c"))
            target:add("includedirs", syslog_dir, {public = true})
        end

        if conf.ULOG_USING_SYSLOG then
            target:add("files", path.join(syslog_dir, "*.c"))
            target:add("includedirs", syslog_dir, {public = true})
        end
    end)
