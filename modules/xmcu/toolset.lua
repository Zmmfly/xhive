function load_tools_dir()
    import("xmcu.base")
    import("xmcu.kconf")
    import("core.cache.memcache")

    -- get all dirs
    local script_dir  = os.scriptdir()
    local project_dir = vformat("$(projectdir)")
    local build_dir   = vformat(path.join(project_dir, "build"))
    local sdk_dir     = path.directory(path.directory(script_dir))

    -- get toolchains dir directory
    local env_path        = os.getenv("XMCU_TOOL_PATH")
    local env_toolchains  = env_path and path.join(env_path, "toolchains") or nil
    local home_dir        = base.get_home_dir()
    local home_toolchains = home_dir and path.join(home_dir, ".xmcu", "toolchains") or nil
    local toolchains_dir  = env_toolchains or home_toolchains

    return toolchains_dir
end

function load_tool_name_by_conf(conf)
    -- This function must return the full name, not based tool name
    import("xmcu.kconf")
    if conf.COMPILER_ARM_GCC then
        return "arm-none-eabi-gcc"
    elseif conf.COMPILER_ATFE then
        return "ATfE"
    else
        raise("No valid compiler selected in " .. kconf.config_name())
    end
end

function load_tool_infos()
    import("xmcu.kconf")
    import("core.base.json")
    local tools_dir       = load_tools_dir()
    local script_dir      = os.scriptdir()
    local sdk_dir         = path.directory(path.directory(script_dir))
    local project_dir     = vformat("$(projectdir)")
    local conf            = kconf.load_configs()
    local tools_info_path = path.join(sdk_dir, "compiler", "toolchains.json")

    -- check toolchains info file exists
    if not os.isfile(tools_info_path) then
        raise("Toolchains info file not found at: " .. tools_info_path)
    end

    -- load toolchains info file
    local tools_info = json.loadfile(tools_info_path)
    if not tools_info then
        raise("Invalid toolchains info file: " .. tools_info_path)
    end

    -- check os and arch related tool info exists
    local os_name = os.host()
    local arch    = os.arch()
    if not tools_info[os_name] or not tools_info[os_name][arch] then
        raise("No toolchain info for os: " .. os_name .. ", arch: " .. arch)
    end
    local tools = tools_info[os_name][arch]

    -- get tool name
    local tool_name      = load_tool_name_by_conf(conf)
    local tool_full_name = vformat("%s-%s-%s", tool_name, os_name, arch)

    -- get tool info
    if not tools[tool_name] then
        raise("No toolchain info for tool: " .. tool_name)
    end
    local tool_info = tools[tool_name]

    -- check tool info include url and sha256
    for _, item in ipairs(tool_info) do
        if not item.url or not item.sha256 then
            raise("Invalid toolchain info for tool: " .. tool_name)
        end
    end

    local ready = false
    local ready_list_json_path = path.join(tools_dir, "ready_list.json")
    if os.isfile(ready_list_json_path) then
        local ready_list = json.loadfile(ready_list_json_path)
        if ready_list and ready_list[tool_full_name] then
            ready = true
        end
    end

    local result = {
        name      = tool_name,
        full_name = tool_full_name,
        tools_dir = tools_dir,
        tools_dl  = path.join(tools_dir, "downloads"),
        tool_dir  = path.join(tools_dir, tool_full_name),
        tool_info = tool_info,
        ready     = ready
    }
    return result
end

function build_arm_flags(conf)
    local cxflags = {}
    local asflags = {}
    local ldflags = {}

    -- Common flags for all ARM toolchains
    table.insert(cxflags, "-mthumb")
    table.insert(asflags, "-mthumb")
    table.insert(ldflags, "-mthumb")

    -- Determine CPU core and set corresponding flags
    local cpu_flags = {}
    if conf.CPU_CORTEX_M0 then
        cpu_flags = {"-mcpu=cortex-m0"}
    elseif conf.CPU_CORTEX_M0PLUS then
        cpu_flags = {"-mcpu=cortex-m0plus"}
    elseif conf.CPU_CORTEX_M1 then
        cpu_flags = {"-mcpu=cortex-m1"}
    elseif conf.CPU_CORTEX_M3 then
        cpu_flags = {"-mcpu=cortex-m3"}
    elseif conf.CPU_CORTEX_M4 then
        cpu_flags = {"-mcpu=cortex-m4"}
    elseif conf.CPU_CORTEX_M7 then
        cpu_flags = {"-mcpu=cortex-m7"}
    elseif conf.CPU_CORTEX_M23 then
        cpu_flags = {"-mcpu=cortex-m23"}
    elseif conf.CPU_CORTEX_M33 then
        cpu_flags = {"-mcpu=cortex-m33"}
    elseif conf.CPU_CORTEX_M35P then
        cpu_flags = {"-mcpu=cortex-m35p"}
    elseif conf.CPU_CORTEX_M55 then
        cpu_flags = {"-mcpu=cortex-m55"}
    elseif conf.CPU_CORTEX_M85 then
        cpu_flags = {"-mcpu=cortex-m85"}
    else
        raise("No valid ARM Cortex-M core selected")
    end

    -- Apply CPU flags to all stages
    for _, flag in ipairs(cpu_flags) do
        table.insert(cxflags, flag)
        table.insert(asflags, flag)
        table.insert(ldflags, flag)
    end

    -- FPU configuration
    local fpu_flags = {}
    if conf.CPU_FPU_SP or conf.CPU_HAS_FPU_SP then
        if conf.CPU_CORTEX_M4 then
            fpu_flags = {"-mfloat-abi=hard", "-mfpu=fpv4-sp-d16"}
        elseif conf.CPU_CORTEX_M7 or conf.CPU_CORTEX_M33 or conf.CPU_CORTEX_M35P then
            fpu_flags = {"-mfloat-abi=hard", "-mfpu=fpv5-sp-d16"}
        elseif conf.CPU_CORTEX_M55 then
            fpu_flags = {"-mfloat-abi=hard", "-mfpu=fpv5-sp-d16"}
        elseif conf.CPU_CORTEX_M85 then
            fpu_flags = {"-mfloat-abi=hard", "-mfpu=fpv5-sp-d16"}
        end
    elseif conf.CPU_FPU_DP or conf.CPU_HAS_FPU_DP then
        if conf.CPU_CORTEX_M7 then
            fpu_flags = {"-mfloat-abi=hard", "-mfpu=fpv5-d16"}
        elseif conf.CPU_CORTEX_M85 then
            fpu_flags = {"-mfloat-abi=hard", "-mfpu=fpv5-d16"}
        end
    else
        -- No FPU or software floating point
        fpu_flags = {"-mfloat-abi=soft"}
    end

    -- Apply FPU flags to all stages
    for _, flag in ipairs(fpu_flags) do
        table.insert(cxflags, flag)
        table.insert(asflags, flag)
        table.insert(ldflags, flag)
    end

    if conf.NO_STD_STARTFILE then
        table.insert(ldflags, "-nostartfiles")
    end

    if conf.COMPILER_ARM_GCC then
        -- GCC specific flags
        -- if conf.CPU_HAS_DSP then
        --     table.insert(cxflags, "-DARM_MATH_CM4")
        --     table.insert(cxflags, "-D__FPU_PRESENT=1")
        -- end

        -- LTO support
        if conf.COMPILER_ENABLE_LTO then
            table.insert(cxflags, "-flto")
            table.insert(ldflags, "-flto")
            table.insert(ldflags, "-fuse-linker-plugin")
        end

        -- GCC specs for embedded systems (only for GCC)
        table.insert(ldflags, "--specs=nano.specs")
        table.insert(ldflags, "--specs=nosys.specs")

    elseif conf.COMPILER_CLANG then
        -- Clang/ATfE common flags
        table.insert(cxflags, "--target=arm-none-eabi")
        table.insert(asflags, "--target=arm-none-eabi")
        table.insert(ldflags, "--target=arm-none-eabi")

        -- if conf.CPU_HAS_DSP then
        --     table.insert(cxflags, "-DARM_MATH_CM4")
        --     table.insert(cxflags, "-D__FPU_PRESENT=1")
        -- end

        -- LTO support
        if conf.COMPILER_ENABLE_LTO then
            if conf.LTO_MODE_THIN then
                table.insert(cxflags, "-flto=thin")
                table.insert(ldflags, "-flto=thin")
            else  -- LTO_MODE_FULL is default
                table.insert(cxflags, "-flto=full")
                table.insert(ldflags, "-flto=full")
            end
        end

        -- Helium MVE support for newer cores
        if conf.CPU_HAS_HELIUM then
            if conf.CPU_HELIUM_VERSION == 1 then
                table.insert(cxflags, "-march=armv8.1-m.main+mve")
                table.insert(asflags, "-march=armv8.1-m.main+mve")
            elseif conf.CPU_HELIUM_VERSION == 2 then
                table.insert(cxflags, "-march=armv8.5-m.main+mve")
                table.insert(asflags, "-march=armv8.5-m.main+mve")
            end
        end

        -- TrustZone support
        if conf.CPU_HAS_TRUSTZONE then
            table.insert(cxflags, "-mthumb-interwork")
            table.insert(asflags, "-mthumb-interwork")
        end

        -- Add printf support with semihosting for ATfE
        if conf.COMPILER_ATFE then
            table.insert(ldflags, "-lcrt0-semihost")
            table.insert(ldflags, "-lsemihost")
        end
    end

    return {cxflags = cxflags, asflags = asflags, ldflags = ldflags}
end

function build_riscv_flags(conf)
    local cxflags = {}
    local asflags = {}
    local ldflags = {}

    if conf.COMPILER_RISCV_GCC then
        -- TODO add riscv gcc common flags by CPU arch configs, change TODO to wait test after complete
    elseif conf.COMPILER_CLANG then
        -- TODO add clang common flags by CPU arch configs, change TODO to wait test after complete
    end
    raise("RISC-V toolchain flags generation not yet implemented")

    return {cxflags = cxflags, asflags = asflags, ldflags = ldflags}
end

function build_toolchain_flags(conf)
    if conf.CPU_ARM then
        return build_arm_flags(conf)
    elseif conf.CPU_RISCV then
        return build_riscv_flags(conf)
    else
        raise("Unsupported CPU architecture in configuration")
    end
end

function mark_tool_as_ready(tool_infos)
    import("core.base.json")
    if not tool_infos.tools_dir then
        raise("Tool directory not specified")
    end

    if not tool_infos.full_name then
        raise("Tool full name not specified")
    end

    local ready_list_json_path = path.join(tool_infos.tools_dir, "ready_list.json")
    local ready_list = {}
    if os.isfile(ready_list_json_path) then
        ready_list = json.loadfile(ready_list_json_path) or {}
    end
    ready_list[tool_infos.full_name] = true
    json.savefile(ready_list_json_path, ready_list)
end

function merge_toolset_to(tool_target_dir, merge_list)
    import("xmcu.base")
    for _, src_dir in ipairs(merge_list) do
        print("Merging tool from " .. src_dir .. " to " .. tool_target_dir)
        -- os.mv(src_dir .. "/*", tool_target_dir)
        base.move_merge(tool_target_dir, src_dir, true)
    end
end

function download_tool(tool_infos)
    import("xmcu.base")
    if not tool_infos.tool_dir then
        raise("Tool directory not specified")
    end

    -- check tool dir empty, skip download if not empty
    if os.isdir(tool_infos.tool_dir) and #os.filedirs(path.join(tool_infos.tool_dir, "**")) > 0 then
        print("Tool directory already exists and not empty: " .. tool_infos.tool_dir)
        return
    end

    if not tool_infos.tool_info then
        raise("Tool info not specified")
    end

    if not tool_infos.tools_dl then
        raise("Tools download directory not specified")
    end

    if not tool_infos.full_name then
        raise("Tool full name not specified")
    end

    -- create download directory
    local tool_dl = path.join(tool_infos.tools_dl, tool_infos.full_name)
    os.mkdir(tool_dl)

    local merge_list = {}
    local extra_list = {}

    -- download and extract each tool archive
    for _, item in ipairs(tool_infos.tool_info) do
        local url    = item.url
        local sha256 = item.sha256
        local filename = path.filename(url)
        local filepath = path.join(tool_dl, filename)

        -- download tool archive
        local verified = false
        if not os.isfile(filepath) then
            print("Downloading tool: " .. url)
            import("net.http").download(url, filepath)
        else
            local file_sha256 = hash.sha256(filepath)
            if file_sha256 ~= sha256 then
                print("SHA256 mismatch for downloaded tool: " .. filename)
                os.rm(filepath)
                print("Removed old file, downloading tool: " .. url)
                import("net.http").download(url, filepath)
            else
                verified = true
                print("SHA256 verified for tool: " .. filename)
            end
        end

        -- verify sha256
        if not verified then
            local file_sha256 = hash.sha256(filepath)
            if file_sha256 ~= sha256 then
                raise("SHA256 mismatch for downloaded tool: " .. filename)
            else
                print("SHA256 verified for tool: " .. filename)
            end
        end

        -- until no dot in base name or reached two levels
        local base_name = filename
        local level = 0
        while true do
            base_name = path.basename(base_name)
            local dot_pos = base_name:find("%.")
            if not dot_pos then
                break
            end
            level = level + 1
            if level >= 2 then
                break
            end
        end

        -- extract tool archive
        local extract_dir = path.join(tool_dl, base_name)
        table.insert(extra_list, extract_dir)

        -- delete if not empty
        if os.isdir(extract_dir) and #os.filedirs(path.join(extract_dir, "**")) > 0 then
            os.rm(extract_dir)
        end

        -- if not exists or empty
        print("Extracting tool: " .. filename .. " to " .. extract_dir)
        os.mkdir(extract_dir)
        import("utils.archive")
        archive.extract(filepath, extract_dir)
        local real_dir = base.find_dir_root(extract_dir)
        print("Real tool dir: " .. real_dir)
        table.insert(merge_list, real_dir)
    end

    -- merge extracted directories into tool_dir
    os.mkdir(tool_infos.tool_dir)
    merge_toolset_to(tool_infos.tool_dir, merge_list)

    -- clean up extra extracted directories
    for _, dir in ipairs(extra_list) do
        os.rm(dir)
    end

    -- mark tool as ready by writing ready_list.json
    mark_tool_as_ready(tool_infos)
end

function elf_to_bin(cc_path, elf_path, bin_output)
    local cc_name = path.filename(cc_path)
    -- If gcc in in cc_name, use objcopy to convert elf to bin
    if cc_name:find("gcc", 1, true) then
        local objcopy, _   = cc_path:gsub("gcc", "objcopy")
        local args    = {"-O", "binary", elf_path, bin_output}
        local result,  err = os.iorunv(objcopy, args)
    -- If clang in cc_name, use llvm-objcopy to convert elf to bin
    elseif cc_name:find("clang", 1, true) then
        local objcopy, _   = cc_path:gsub("clang", "llvm-objcopy")
        local args    = {"-O", "binary", elf_path, bin_output}
        local result,  err = os.iorunv(objcopy, args)
    else
        raise("Unsupported compiler for elf to bin conversion: " .. cc_name)
    end
end

function elf_ram_usage(cc_path, elf_path)
    local function parse_size_output(output)
        for line in output:gmatch("[^\r\n]+") do
            local _, data, bss = line:match("^%s*(%d+)%s+(%d+)%s+(%d+)%s+(%d+)%s+[0-9a-fA-F]+%s+.+$")
            if data and bss then
                local data_val = tonumber(data)
                local bss_val = tonumber(bss)
                if data_val and bss_val then
                    -- size outputs text/data/bss/dec/hex/filename; static RAM equals data + bss
                    return data_val + bss_val
                end
            end
        end
    end

    local function try_size_tools(candidates, err_hint)
        local last_err
        for _, tool in ipairs(candidates) do
            local output, err = os.iorunv(tool, {elf_path})
            if output then
                local usage = parse_size_output(output)
                if usage then
                    return usage
                end
                raise("Failed to parse RAM usage from size output")
            end
            last_err = err
        end
        if last_err then
            raise(last_err)
        end
        raise(err_hint)
    end

    local function build_candidates(tool_names)
        local candidates = {}
        local seen = {}
        local cc_dir = path.directory(cc_path)

        local function push(value)
            if value and value ~= "" and not seen[value] then
                table.insert(candidates, value)
                seen[value] = true
            end
        end

        for _, name in ipairs(tool_names) do
            if cc_dir and cc_dir ~= "" and cc_dir ~= "." then
                push(path.join(cc_dir, name))
            end
            push(name)
        end
        return candidates
    end

    local cc_name = path.filename(cc_path)
    if cc_name:find("gcc", 1, true) then
        local tool_names = {}
        local prefix, suffix = cc_name:match("^(.*)gcc(.*)$")
        if prefix then
            if suffix and #suffix > 0 then
                table.insert(tool_names, prefix .. "size" .. suffix)
            end
            table.insert(tool_names, prefix .. "size")
        end
        local name
        name, _ = cc_name:gsub("gcc$", "size")
        table.insert(tool_names, name)
        name, _ = cc_name:gsub("gcc", "size")
        table.insert(tool_names, name)
        table.insert(tool_names, "size")
        return try_size_tools(build_candidates(tool_names), "Failed to execute size tool")
    elseif cc_name:find("clang", 1, true) then
        local tool_names = {}
        local prefix, suffix = cc_name:match("^(.*)clang(.*)$")
        if prefix then
            if suffix == "++" then
                table.insert(tool_names, prefix .. "llvm-size")
            else
                if suffix and #suffix > 0 then
                    table.insert(tool_names, prefix .. "llvm-size" .. suffix)
                end
                table.insert(tool_names, prefix .. "llvm-size")
            end
        end
        local name
        name, _ = cc_name:gsub("clang$", "llvm-size")
        table.insert(tool_names, name)
        name, _ = cc_name:gsub("clang", "llvm-size")
        table.insert(tool_names, name)
        table.insert(tool_names, "llvm-size")
        return try_size_tools(build_candidates(tool_names), "Failed to execute llvm-size tool")
    else
        raise("Unsupported compiler for elf ram usage: " .. cc_name)
    end
end
