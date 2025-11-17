target("dfs_v2")
    set_kind("object")
    on_load(function(target)
        local conf = target:data("kconfig")
        if (not conf.RT_USING_DFS) or (not conf.RT_USING_DFS_V2) then
            return
        end
        local sdir = os.scriptdir()
        local srcs = {}
        local incs = {}

        -- add dfs common source files
        if conf.RT_USING_SMART then
            table.insert(srcs, path.join(sdir, "src", "*.c"))
            table.insert(srcs, path.join(sdir, "src", "*.cpp"))
        else
            table.insert(srcs, path.join(sdir, "src", "*.c|dfs_file_mmap.c"))
            table.insert(srcs, path.join(sdir, "src", "*.cpp"))
        end

        -- add filesystem source files
        local fss_dir = path.join(sdir, "filesystem")
        local fs_maps = {
            RT_USING_DFS_CROMFS  = {dir = "cromfs", srcs = "*.c"},
            RT_USING_DFS_DEVFS   = {dir = "devfs", srcs = "*.c"},
            RT_USING_DFS_ELMFAT  = {dir = "elmfat", srcs = "*.c"},
            RT_USING_DFS_MQUEUE  = {dir = "mqueue", srcs = "*.c"},
            RT_USING_DFS_PROCFS  = {dir = "procfs", srcs = "*.c"},
            RT_USING_DFS_PTYFS   = {dir = "ptyfs", srcs = "*.c"},
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
        target:add("includedirs", incs)
        target:add("includedirs", "include", {public=true})
    end)