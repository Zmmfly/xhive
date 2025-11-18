target("at")
    set_kind("object")
    set_default(false)
    on_load(function(target) 
        local conf = target:data("kconfig")
        local sdir = os.scriptdir()

        if not conf.RT_USING_AT then
            return
        end

        local src_dir = path.join(sdir, "src")
        target:add("files", path.join(src_dir, "at_utils.c"))
        target:add("includedirs", path.join(sdir, "include"), {public = true})

        if conf.AT_USING_CLI then
            target:add("files", path.join(src_dir, "at_cli.c"))
        end

        if conf.AT_USING_SERVER then
            target:add("files", path.join(src_dir, "at_server.c"))
            target:add("files", path.join(src_dir, "at_base_cmd.c"))
        end

        if conf.AT_USING_CLIENT then
            target:add("files", path.join(src_dir, "at_client.c"))
        end

        if conf.AT_USING_SOCKET then
            target:add("files", path.join(src_dir, "at_socket", "*.c"))
            target:add("includedirs", path.join(src_dir, "at_socket"), {public = true})
        end
    end)
