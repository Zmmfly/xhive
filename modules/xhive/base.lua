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

function split_commas_paths(path_list_by_commas)
    local paths = {}
    for dir in path_list_by_commas:gmatch("[^,]+") do
        table.insert(paths, dir)
    end
    return paths
end

function load_paths()
    import("core.cache.memcache")

    local paths = memcache.get("xhive", "common_paths")
    if paths then
        return paths
    end

    local prjdir      = path.normalize( os.projectdir())
    local builddir    = path.normalize( path.join(prjdir, "build") )
    local homedir     = path.normalize( os.getenv("HOME") or os.getenv("USERPROFILE") )
    local sdkdir      = path.normalize( path.directory(path.directory(os.scriptdir())) )
    local sdkhome     = path.normalize( os.getenv("XHIVE_HOME") or path.join(homedir, ".xhive") )
    local kconf_entry = path.normalize( path.join(builddir, "Kconfig") )
    local toolsdir    = path.normalize( os.getenv("XHIVE_TOOLS_PATH") or path.join(sdkhome, "tools") )
    local dldir       = path.normalize( path.join(sdkhome, "downloads") )

    paths = {
        sdkdir      = sdkdir,
        sdkhome     = sdkhome,
        prjdir      = prjdir,
        builddir    = builddir,
        homedir     = homedir,
        dldir       = dldir,
        kconf_entry = kconf_entry,
        toolsdir    = toolsdir,
    }
    memcache.set("xhive", "common_paths", paths)
    return paths
end

-- Find the specific path where multiple folders or any files first appear in a directory
function find_deep_root(dir_path)
    if not os.isdir(dir_path) then
        return nil
    end
    local cur = dir_path
    while true do
        local entries = os.filedirs(path.join(cur, "*"))
        if #entries == 0 then
            return cur
        end

        local dirs, files = {}, {}
        for _, entry in ipairs(entries) do
            if os.isdir(entry) then
                dirs[#dirs + 1] = entry
            else
                files[#files + 1] = entry
            end
        end

        -- If you need "return when multiple files found", change #files > 0 to #files > 1
        if #dirs > 1 or #files > 0 then
            return cur
        elseif #dirs == 1 then
            cur = dirs[1]
        else
            -- No subdirectories and no files (theoretically entries is already 0), fallback return
            return cur
        end
    end
end

-- Check if there are any conflicting files in destination
function check_conflicts(dst, src)
    if not os.isdir(src) then
        return false
    end

    local conflicts = {}
    local src_files = os.filedirs(path.join(src, "**"))

    for _, src_file in ipairs(src_files) do
        local relative_path = path.relative(src_file, src)
        local dst_file = path.join(dst, relative_path)

        if os.exists(dst_file) then
            table.insert(conflicts, relative_path)
        end
    end

    return #conflicts > 0, conflicts
end

-- Copy merge: copy files from src to dst with optional force overwrite
function copy_merge(dst, src, force)
    if not os.isdir(src) then
        print("Error: Source directory does not exist: " .. src)
        return false
    end

    -- Create destination directory if it doesn't exist
    os.mkdir(dst)

    -- Check for conflicts if force is false
    if not force then
        local has_conflicts, conflicts = check_conflicts(dst, src)
        if has_conflicts then
            print("Conflicts found, merging aborted:")
            for _, conflict in ipairs(conflicts) do
                print("  " .. conflict)
            end
            return false
        end
    end

    -- Perform the merge
    local success = true
    local src_files = os.filedirs(path.join(src, "**"))
    local link_list = {}

    for _, src_file in ipairs(src_files) do
        local rel_path = path.relative(src_file, src)
        local dst_file = path.join(dst, rel_path)

        if os.islink(src_file) then
            table.insert(link_list, {src = src_file, dst = dst_file, target = os.readlink(src_file)})

        elseif os.isdir(src_file) then
            if not os.exists(dst_file) then
                -- Create subdirectory
                os.mkdir(dst_file)
                if not os.exists(dst_file) then
                    print("Failed to create directory: " .. dst_file)
                    success = false
                end
            end
        else
            -- Copy file
            local dst_dir = path.directory(dst_file)
            if not os.exists(dst_dir) then
                os.mkdir(dst_dir)
            end

            -- remove if exists
            if os.exists(dst_file) then
                os.tryrm(dst_file)
            end

            if not os.trycp(src_file, dst_dir) then
                print("Failed to copy file: " .. src_file .. " -> " .. dst_file)
                success = false
            end
        end
    end

    return success
end

-- Move merge: move files from src to dst with optional force overwrite
function move_merge(dst, src, force)
    if not os.isdir(src) then
        print("Error: Source directory does not exist: " .. src)
        return false
    end

    -- Create destination directory if it doesn't exist
    os.mkdir(dst)

    -- Check for conflicts if force is false
    if not force then
        local has_conflicts, conflicts = check_conflicts(dst, src)
        if has_conflicts then
            print("Conflicts found, merging aborted:")
            for _, conflict in ipairs(conflicts) do
                print("  " .. conflict)
            end
            return false
        end
    end

    -- Perform the merge
    local success = true
    local src_files = os.filedirs(path.join(src, "**"))
    local link_list = {}

    for _, src_file in ipairs(src_files) do
        local rel_path = path.relative(src_file, src)
        local dst_file = path.join(dst, rel_path)

        if os.islink(src_file) then
            table.insert(link_list, {dst = dst_file, target = os.readlink(src_file)})

        elseif os.isdir(src_file) then
            if not os.exists(dst_file) then
                -- Create subdirectory
                os.mkdir(dst_file)
                if not os.exists(dst_file) then
                    print("Failed to create directory: " .. dst_file)
                    success = false
                end
            end
        elseif os.isfile(src_file) then
            -- Move file
            local dst_dir = path.directory(dst_file)
            if not os.exists(dst_dir) then
                os.mkdir(dst_dir)
            end

            -- remove if exists
            if os.exists(dst_file) then
                os.tryrm(dst_file)
            end

            if not os.trymv(src_file, dst_dir) then
                print("Failed to move file: " .. src_file .. " -> " .. dst_dir)
                if not os.exists(dst_dir) then
                    print("Destination directory does not exist: " .. dst_dir)
                end
                success = false
            end
        end
    end

    -- Remove empty directories from source if move was successful
    if success then
        -- create links
        for _, link_info in ipairs(link_list) do
            local dst_dir  = path.directory(link_info.dst)
            local target   = link_info.target
            local lnk_name = path.filename(link_info.dst)

            os.cd(dst_dir)
            os.ln(target, lnk_name)
            os.cd("-")
        end

        local src_dirs = {}
        for _, src_file in ipairs(src_files) do
            if os.isdir(src_file) then
                table.insert(src_dirs, src_file)
            end
        end

        -- Sort directories in reverse order to remove subdirectories first
        table.sort(src_dirs, function(a, b) return #a > #b end)

        for _, dir in ipairs(src_dirs) do
            local entries = os.filedirs(path.join(dir, "*"))
            if #entries == 0 then
                os.tryrm(dir)
            end
        end

        -- Try to remove the source directory if it's empty
        local entries = os.filedirs(path.join(src, "*"))
        if #entries == 0 then
            os.tryrm(src)
        end
    end

    return success
end
