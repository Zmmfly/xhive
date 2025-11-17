
target("libcpu")
    set_kind("object")
    add_deps("finsh", {public=true})
    on_load(function(target)
        import("xhive.proc")
        local conf    = target:data("kconfig")

        -- Get CPU info in Kconfig
        local dir     = os.scriptdir()
        local cpuinfo = proc.cpuinfo_by_conf(conf)
        if not cpuinfo.arch or not cpuinfo.core then
            raise("Unsupported CPU architecture or core!")
        end

        -- Add source files
        local cpu_dir = path.join(dir, cpuinfo.arch, cpuinfo.core)
        if not os.isdir(cpu_dir) then
            raise("CPU directory not found: " .. cpu_dir)
        end

        target:add("files", path.join(cpu_dir, "*.c"))
        if conf.COMPILER_GCC or conf.COMPILER_CLANG then
            target:add("files", path.join(cpu_dir, "*_gcc.S"))
        else
            raise("Unsupported compiler!")
        end
        target:add("includedirs", cpu_dir, {public = true})

    end)
