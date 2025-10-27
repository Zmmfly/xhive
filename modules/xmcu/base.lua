function get_commas_paths(path_list_by_commas)
    local paths = {}
    for dir in path_list_by_commas:gmatch("[^,]+") do
        table.insert(paths, dir)
    end
    return paths
end

function get_home_dir()
    local host = os.host()
    if host == "windows" then
        return os.getenv("USERPROFILE")
    else
        return os.getenv("HOME")
    end
end

-- Find the specific path where multiple folders or any files first appear in a directory
function find_dir_root(dir_path)
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
