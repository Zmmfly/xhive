--[[
Copyright (c) 2025 Zmmfly. All rights reserved.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors
   may be used to endorse or promote products derived from this software
   without specific prior written permission.
]]

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
    return kconfig
end

function parse_cached(config_path)
    import("core.cache.memcache")
    local cache_key = "kconf_"..path.absolute(config_path)
    local cached_config = memcache.get("xhive", cache_key)
    if not cached_config then
        cached_config = parse(config_path)
        memcache.set("xhive", cache_key, cached_config)
    end
    return cached_config
end

function load_name()
    return os.getenv("KCONFIG_CONFIG") or ".config"
end

function build_entry()
    import("xhive.base")
    local dirs = base.load_paths()

    local prj_entry  = path.join(dirs.prjdir, "Kconfig")
    local sdk_entry  = path.join(dirs.sdkdir, "Kconfig")
    local root_entry = path.join(dirs.builddir, "Kconfig")

    -- workaround for windows path issue
    if os.host() == "windows" then
        sdk_entry  = sdk_entry:gsub("\\", "/")
        prj_entry  = prj_entry:gsub("\\", "/")
        root_entry = root_entry:gsub("\\", "/")
    end

    local content = string.format("source \"%s\"\n", sdk_entry)
    if os.isfile(prj_entry) then
        content = content .. string.format("source \"%s\"\n", prj_entry)
    end

    -- Check and update kconfig entry file
    if os.isfile(root_entry) then
        local existing_content = io.readfile(root_entry)
        if existing_content ~= content then
            print("Update kconfig entry file at: " .. root_entry)
            io.writefile(root_entry, content)
        end
    else
        print("Create kconfig entry file at: " .. root_entry)
        io.writefile(root_entry, content)
    end
end

function header_name()
    return os.getenv("XHIVE_HEADER_NAME") or "xhive_config.h"
end

function load_header_path()
    import("xhive.base")
    local dirs = base.load_paths()
    return path.join(dirs.builddir, header_name())
end

function build_header_with_entry(header_path, entry_path)
    import("xhive.base")
    import("lib.detect.find_tool")
    local tool = base.detect_tool({"genconfig", "genconfig.bat"}, {check = "--help"})
    local dirs = base.load_paths()
    if not tool then
        raise("genconfig tool not found! please install by `pip install kconfiglib`")
    end
    os.execv(tool.program, {"--header-path", header_path, entry_path}, {envs = {curdir = dirs.prjdir}})
end

-- Generate xhive_config.h from .config file
function build_header()
    import("core.cache.memcache")
    local dirs        = base.load_paths()
    local tool        = base.detect_tool({"genconfig", "genconfig.bat"}, {check = "--help"})
    local header_path = load_header_path()
    local build_ready = memcache.get("xhive", "header_build_" .. dirs.prjdir)
    if build_ready then
        return
    end

    -- call genconfig command
    os.execv(tool.program, {"--header-path", header_path, dirs.kconf_entry}, {envs = {curdir = dirs.prjdir}})

    -- Check if generation succeeded
    if not os.isfile(header_path) then
        raise("Error: Failed to generate config header at: " .. header_path)
    end

    -- Read generated content and wrap with guard if needed
    local header_content = io.readfile(header_path)
    local guard_template = 
        "#ifndef __XHIVE_CONFIG_H__\n" ..
        "#define __XHIVE_CONFIG_H__\n\n" ..
        "%s\n" ..
        "#endif // __XHIVE_CONFIG_H__\n"

    header_content = guard_template:format(header_content)
    io.writefile(header_path, header_content)
    memcache.set("xhive", "header_build_" .. dirs.prjdir, true)
end

-- Get configs for current project, build/Kconfig as entry generated if needed and .config as config file name
function load_configs()
    import("xhive.base")
    local dirs = base.load_paths()

    local script_dir  = os.scriptdir()
    local config_path = path.join(dirs.prjdir, load_name())

    build_entry()
    if not os.isfile(config_path) then
        print(load_name() .. " file not found at: " .. config_path)
        raise("Please run 'xmake menuconfig' to config your project.")
    end
    return parse_cached(config_path)
end

function set_config(conf_path, key, value)
    -- find CONFIG_key in conf_path and set to value
    -- uncomment if needed
    --[[ 
        true value -> CONFIG_KEY=y
        false value -> # CONFIG_KEY is not set
        string hex value -> CONFIG_KEY=0x1234abcd
        string value -> CONFIG_KEY="value"
        number value -> CONFIG_KEY=123
     ]]
    local file_content = io.readfile(conf_path)
    local pattern = "^(# )?CONFIG_" .. key .. "=.*$"
    local replacement = ""
    if value == true then
        replacement = "CONFIG_" .. key .. "=y"
    elseif value == false then
        replacement = "# CONFIG_" .. key .. " is not set"
    elseif type(value) == "string" and value:startswith("0x") then
        replacement = "CONFIG_" .. key .. "=" .. value
    elseif type(value) == "string" then
        replacement = "CONFIG_" .. key .. "=\"" .. value .. "\""
    elseif type(value) == "number" then
        replacement = "CONFIG_" .. key .. "=" .. value
    end
    if file_content:match(pattern) then
        file_content = file_content:gsub(pattern, replacement)
    else
        -- append to end
        file_content = file_content .. "\n" .. replacement .. "\n"
    end
    io.writefile(conf_path, file_content)
end
