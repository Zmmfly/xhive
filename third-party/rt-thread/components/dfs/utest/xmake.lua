target("dfs_utest")
    set_kind("object")
    on_load(function(target)
        local conf = target:data("kconfig")
        if (not conf.RT_USING_UTESTCASES) or (not conf.RT_USING_DFS) then
            return
        end
        local sdir = os.scriptdir()
        target:add("includedirs", sdir)
        if conf.RT_UTEST_TC_USING_DFS_API then
            target:add("files", path.join(sdir, "tc_dfs_api"))
            if conf.RT_UTEST_TC_USING_POSIX_API then
                target:add("files", path.join(sdir, "tc_posix_api.c"))
            end
        end
    end)