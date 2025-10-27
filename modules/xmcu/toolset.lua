function get_tools_dir()
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

function get_tool_name_by_conf(conf)
    if conf.COMPILER_ARM_GCC then
        return "arm-none-eabi-gcc"
    elseif conf.COMPILER_CLANG then
        return "clang"
    elseif conf.COMPILER_ATFE then
        return "ATfE"
    else
        raise("No valid compiler selected in .config")
    end
end

function load_tool_infos()
    import("xmcu.kconf")
    import("core.base.json")
    local tools_dir       = get_tools_dir()
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
    local tool_name      = get_tool_name_by_conf(conf)
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
