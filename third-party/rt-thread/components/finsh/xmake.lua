target("finsh")
    set_kind("object")
    add_deps("dfs", {public = true})
    on_load(function(target)
        local conf = target:data("kconfig")
        if not conf.RT_USING_FINSH then
            return
        end
        local sdir = os.scriptdir()
        local srcs = {"shell.c", "msh.c", "msh_parse.c"}
        if conf.MSH_USING_BUILT_IN_COMMANDS then
            table.insert(srcs, "cmd.c")
        end
        if conf.DFS_USING_POSIX then
            table.insert(srcs, "msh_file.c")
        end
        -- to abs path
        for i, src in ipairs(srcs) do
            srcs[i] = path.join(sdir, src)
        end
        target:add("files", srcs)
        target:add("includedirs", sdir, {public = true})
        if conf.FINSH_THREAD_STACK_SIZE and conf.COMPILER_GCC then
            local flag = format("-Wstack-usage=%d", conf.FINSH_THREAD_STACK_SIZE)
            target:add("cxflags", flag)
        end
    end)
