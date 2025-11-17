includes("klibc")
target("src")
    set_kind("object")
    set_default(false)
    add_deps("finsh", {public=true})
    add_deps("klibc", {public=true})
    add_files("*.c")
    add_defines("__RTTHREAD__", "__RT_KERNEL_SOURCE__", {public=true})
    on_load(function(target)
        local conf = target:data("kconfig") or {}
        local sdir = os.scriptdir()

        -- remove unneeded files
        local remove_lst = {}

        -- mapping of config options to source files
        local remove_maps = {
            CONFIG_RT_USING_SMALL_MEM = "mem.c",
            CONFIG_RT_USING_SLAB      = "slab.c",
            CONFIG_RT_USING_MEMPOOL   = "mempool.c",
            CONFIG_RT_USING_MEMHEAP   = "memheap.c",
            CONFIG_RT_USING_SIGNALS   = "signal.c",
            CONFIG_RT_USING_DEVICE    = "device.c",
        }
        for k, v in pairs(remove_maps) do
            if not conf[k] then
                table.insert(remove_lst, path.join(sdir, v))
            end
        end

        -- special handling for SMP/UP files
        if not conf.CONFIG_RT_USING_SMP then
            table.insert(remove_lst, path.join(sdir, "cpu_mp.c"))
            table.insert(remove_lst, path.join(sdir, "scheduler_mp.c"))
        else 
            table.insert(remove_lst, path.join(sdir, "cpu_up.c"))
            table.insert(remove_lst, path.join(sdir, "scheduler_up.c"))
        end

        target:remove("files", remove_lst)
    end)
