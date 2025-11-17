target("dfs_v1")
    set_kind("object")
    on_load(function(target)
        local conf = target:data("kconfig")
        if (not conf.RT_USING_DFS) or (not conf.RT_USING_DFS_V1) then
            return
        end
        local sdir = os.scriptdir()
        local srcs = {}
        local incs = {}

        -- add dfs common source files
        for _, file in ipairs({"dfs_fs.c", "dfs_fs.c", "dfs.c"}) do
            table.insert(srcs, path.join(sdir, "src", file))
        end
        if conf.DFS_USING_POSIX then
            table.insert(srcs, path.join(sdir, "src", "dfs_posix.c"))
        end

        -- add filesystem source files
        local fss_dir = path.join(sdir, "filesystem")
        local fs_maps = {
            RT_USING_DFS_CROMFS  = {dir = "cromfs", srcs = "*.c"},
            RT_USING_DFS_DEVFS   = {dir = "devfs", srcs = "*.c"},
            RT_USING_DFS_ELMFAT  = {dir = "elmfat", srcs = "*.c"},
            RT_USING_DFS_ISO9660 = {dir = "iso9660", srcs = "*.c"},
            RT_USING_DFS_MQUEUE  = {dir = "mqueue", srcs = "*.c"},
            RT_USING_DFS_NFS     = {dir = "nfs", srcs = {"*.c", "rpc/*.c|rpc/auth_none.c"}},
            RT_USING_DFS_RAMFS   = {dir = "ramfs", srcs = "*.c"},
            RT_USING_DFS_ROMFS   = {dir = "romfs", srcs = "*.c"},
            RT_USING_DFS_TMPFS   = {dir = "tmpfs", srcs = "*.c"},
        }
        for k, v in pairs(fs_maps) do
            if conf[k] then
                local fs_dir = path.join(fss_dir, v.dir)
                if type(v.srcs) == "table" then
                    for _, f in ipairs(v.srcs) do 
                        table.insert(srcs, path.join(fs_dir, f))
                    end
                else
                    table.insert(srcs, path.join(fs_dir, v.srcs))
                end
                table.insert(incs, fs_dir)
            end
        end
        target:add("srcs", srcs)
        target:add("includedirs", incs, {public=true})
        target:add("includedirs", path.join(sdir, "include"), {public=true})
    end)