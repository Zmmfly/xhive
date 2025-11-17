includes("vdso")

target("lwp")
    set_kind("object")
    on_load(function(target)
        import("xhive.proc")
        local conf = target:data("kconfig")
        if not conf.RT_USING_SMART then
            return
        end

        local sdir = os.scriptdir()
        local srcs = {}
        local incs = {}
        local arch = nil
        local cpu = nil

        -- Architecture support mapping (from SConscript)
        local support_arch = {
            ["arm"] = {"cortex-m3", "cortex-m4", "cortex-m7", "arm926", "cortex-a"},
            ["aarch64"] = {"cortex-a"},
            ["risc-v"] = {"rv64"},
            ["x86"] = {"i386"}
        }

        -- Platform file mapping (from SConscript)
        local platform_file = {
            ["gcc"] = "gcc.S",
            ["armcc"] = "rvds.S",
            ["iar"] = "iar.S"
        }

        -- Get CPU info
        local cpuinfo = proc.cpuinfo_by_conf(conf)

        -- Determine architecture and CPU based on SConscript logic
        if cpuinfo.arch == "arm" and cpuinfo.cpu == "aarch32" and cpuinfo.core:find("cortex-a") then
            arch = "arm"
            cpu  = "cortex-a"
        elseif cpuinfo.arch == "arm" and cpuinfo.cpu == "aarch64" and cpuinfo.core:find("cortex-a") then
            arch = "aarch64"
            cpu  = "cortex-a"
        elseif cpuinfo.arch == "risc-v" then
            arch = "risc-v"
            -- Fix the cpu for risc-v (from SConscript)
            if conf.ARCH_RISCV_RV64 then
                cpu = "rv64"
            else
                cpu = "rv32"
            end
        elseif cpuinfo.arch == "x86" then
            arch = "x86"
            cpu  = "i386"
        end

        -- Check if architecture and CPU are supported
        if arch and support_arch[arch] and cpu then
            local is_supported = false
            for _, supported_cpu in ipairs(support_arch[arch]) do
                if cpu == supported_cpu then
                    is_supported = true
                    break
                end
            end

            if is_supported then
                local arch_dir        = path.join(sdir, "arch", arch, cpu)
                local arch_common_dir = path.join(sdir, "arch", arch, "common")
                local common_dir      = path.join(sdir, "arch", "common")

                -- Add include paths
                table.insert(incs, sdir)
                if os.isdir(arch_dir) then
                    table.insert(incs, arch_dir)
                end

                -- Determine platform (compiler)
                local platform = "gcc" -- default to gcc
                if conf.COMPILER_GCC or conf.COMPILER_CLANG then
                    platform = "gcc"
                elseif conf.COMPILER_ARMCC then
                    platform = "armcc"
                elseif conf.COMPILER_IAR then
                    platform = "iar"
                end

                -- Add assembly files if platform supported
                if platform_file[platform] and os.isdir(arch_dir) then
                    local asm_pattern = "*_" .. platform_file[platform]
                    for _, file in ipairs(os.files(path.join(arch_dir, asm_pattern))) do
                        table.insert(srcs, file)
                    end
                end

                -- Add architecture-specific C files
                if os.isdir(arch_dir) then
                    for _, file in ipairs(os.files(path.join(arch_dir, "*.c"))) do
                        table.insert(srcs, file)
                    end
                end

                -- Add architecture common files
                if os.isdir(arch_common_dir) then
                    for _, file in ipairs(os.files(path.join(arch_common_dir, "*.c"))) do
                        -- Filter out VDSO files if VDSO is not enabled
                        if conf.RT_USING_VDSO or (not (path.filename(file):match("vdso") or path.filename(file):match("vdso_data"))) then
                            table.insert(srcs, file)
                        end
                    end
                end

                -- Add common architecture files
                if os.isdir(common_dir) then
                    for _, file in ipairs(os.files(path.join(common_dir, "*.c"))) do
                        -- Filter out VDSO files if VDSO is not enabled
                        if conf.RT_USING_VDSO or (not (path.filename(file):match("vdso") or path.filename(file):match("vdso_data"))) then
                            table.insert(srcs, file)
                        end
                    end
                end

                -- Add main LWP source files with MMU consideration
                local excluded_files = {}
                if not conf.ARCH_MM_MMU then
                    excluded_files = {
                        "ioremap.c", "lwp_futex.c", "lwp_mm_area.c",
                        "lwp_pmutex.c", "lwp_shm.c", "lwp_user_mm.c"
                    }
                end

                for _, file in ipairs(os.files(path.join(sdir, "*.c"))) do
                    local filename = path.filename(file)
                    local should_exclude = false
                    for _, excluded in ipairs(excluded_files) do
                        if filename == excluded then
                            should_exclude = true
                            break
                        end
                    end
                    if not should_exclude then
                        table.insert(srcs, file)
                    end
                end

                -- Terminal I/O Subsystem
                local termios_dirs = {"terminal", "terminal/freebsd"}
                for _, dir in ipairs(termios_dirs) do
                    local term_dir = path.join(sdir, dir)
                    if os.isdir(term_dir) then
                        for _, file in ipairs(os.files(path.join(term_dir, "*.c"))) do
                            table.insert(srcs, file)
                        end
                        if dir == "terminal" then
                            table.insert(incs, term_dir)
                        end
                    end
                end

                -- Remove optional sources
                if not conf.LWP_USING_RUNTIME then
                    local filtered_srcs = {}
                    for _, file in ipairs(srcs) do
                        if not path.filename(file):match("lwp_runtime%.c") then
                            table.insert(filtered_srcs, file)
                        end
                    end
                    srcs = filtered_srcs
                end
            end
        end

        -- Set files and include directories
        if #srcs > 0 then
            target:add("files", srcs)
        end
        if #incs > 0 then
            target:add("includedirs", incs, {public = true})
        end
    end)