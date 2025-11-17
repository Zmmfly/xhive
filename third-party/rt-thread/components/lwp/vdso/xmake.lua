target("vdso")
    set_kind("object")
    on_load(function(target)
        local conf = target:data("kconfig")
        if (not conf.RT_USING_SMART) or (not conf.RT_USING_VDSO) then
            return
        end

        local sdir = os.scriptdir()
        local srcs = {}
        local incs = {sdir}

        -- Add VDSO source files
        local vdso_files = {"vdso_data.c", "vdso.c", "vdso_weak.c"}
        for _, file in ipairs(vdso_files) do
            local filepath = path.join(sdir, file)
            if os.isfile(filepath) then
                table.insert(srcs, filepath)
            end
        end

        -- Add VDSO kernel and user directories if they exist
        local vdso_dirs = {"kernel", "user"}
        for _, dir in ipairs(vdso_dirs) do
            local dirpath = path.join(sdir, dir)
            if os.isdir(dirpath) then
                table.insert(incs, dirpath)
                for _, file in ipairs(os.files(path.join(dirpath, "*.c"))) do
                    table.insert(srcs, file)
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