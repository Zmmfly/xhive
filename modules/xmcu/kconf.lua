function parse(config_path)
    -- print("Parsing kconfig file: " .. config_path)
    -- 一次性读取 kconfig .config 文件内容
    local file_content = io.readfile(config_path)

    local kconfig = {}

    -- 按行解析文件内容
    for line in file_content:gmatch("[^\n]+") do
        -- 忽略注释行
        if not line:match("^#") then
            -- 匹配键值对
            local key, value = line:match("^CONFIG_([%w_]+)=(.+)$")
            if key and value then
                -- 处理不同类型的值
                if value == "y" then
                    kconfig[key] = true
                elseif value == "n" then
                    kconfig[key] = false
                elseif value == "m" then
                    kconfig[key] = "module"
                elseif value:match("^\".*\"$") then
                    kconfig[key] = value:sub(2, -2)  -- 去掉引号
                elseif value:match("^0x[0-9a-fA-F]+$") then
                    kconfig[key] = tonumber(value)
                elseif value:match("^[+-]?%d+$") then
                    kconfig[key] = tonumber(value)
                else
                    kconfig[key] = value
                end
            end
        end
    end

    -- 缓存解析结果
    import("core.cache.memcache")
    local cache_key = "kconf_"..path.absolute(config_path)
    memcache.set("xmcu", cache_key, kconfig)
    return kconfig
end

function parse_cached(config_path)
    import("core.cache.memcache")
    local cache_key = "kconf_"..path.absolute(config_path)
    local cached_config = memcache.get("xmcu", cache_key)
    if cached_config then
        -- print("Using cached kconfig for: " .. config_path)
        return cached_config
    else
        return parse(config_path)
    end
end

-- 提供一个更标准的API来获取当前项目的配置
function get_project_config(project_dir)
    if not project_dir then
        project_dir = vformat("$(projectdir)")
    end
    return parse_cached(path.join(project_dir, ".config"))
end

function build(project_dir, sdk_path, entry_dir)
    -- Build kconfig entry in entry_dir
    local proj_kconf_file  = vformat(path.join(project_dir, "Kconfig"))
    local sdk_kconf_file   = vformat(path.join(sdk_path, "Kconfig"))
    local entry_kconf_file = vformat(path.join(entry_dir, "Kconfig"))

    local kconf_content = string.format("source \"%s\"\n", path.absolute(sdk_kconf_file))
    if os.isfile(proj_kconf_file) then
        kconf_content = kconf_content .. string.format("source \"%s\"\n", path.absolute(proj_kconf_file))
    end

    -- Check and update kconfig entry file
    if os.isfile(entry_kconf_file) then
        -- print("Kconfig entry file exists at: " .. entry_kconf_file)
        local existing_content = io.readfile(entry_kconf_file)
        if existing_content ~= kconf_content then
            print("Update kconfig entry file at: " .. entry_kconf_file)
            io.writefile(entry_kconf_file, kconf_content)
        end
    -- If the entry kconfig file does not exist, create it
    else
        -- print("Generating kconfig entry file at: " .. entry_kconf_file)
        io.writefile(entry_kconf_file, kconf_content)
    end
end

function genconfig(project_dir)
    -- Generate xmcu_config.h from .config file
    local buildir     = vformat(path.join(project_dir, "build"))
    local entry       = path.join(buildir, "Kconfig")
    local header_path = path.join(buildir, "xmcu_config.h")

    -- call genconfig command
    os.execv("genconfig", {"--header-path", header_path, entry})

    -- Check if generation succeeded
    if not os.isfile(header_path) then
        print("Error: Failed to generate config header at: " .. header_path)
        return
    end

    -- Read generated content and wrap with guard if needed
    local header_content = io.readfile(header_path)
    if not header_content:match("#ifndef __XMCU_CONFIG_H__") then
        local guard_template = 
            "#ifndef __XMCU_CONFIG_H__\n" ..
            "#define __XMCU_CONFIG_H__\n\n" ..
            "%s\n" ..
            "#endif // __XMCU_CONFIG_H__\n"

        header_content = guard_template:format(header_content)
        io.writefile(header_path, header_content)
    end

    -- print("Generated config header at: " .. header_path)
end

-- get configs for current project, build/Kconfig as entry generated if needed and .config as config file name
function load_configs()
    local script_dir  = os.scriptdir()
    local project_dir = vformat("$(projectdir)")
    local build_dir   = vformat(path.join(project_dir, "build"))
    local sdk_dir     = path.directory(path.directory(script_dir))
    local config_path = path.join(project_dir, ".config")

    build(project_dir, sdk_dir, build_dir)
    if not os.isfile(config_path) then
        print(".config file not found at: " .. config_path)
        raise("Please run 'xmake menuconfig' to config your project.")
        return {}
    end
    return parse_cached(config_path)
end
