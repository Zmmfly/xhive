target("compilers")
    set_kind("object")
    set_default(false)
    add_files("common/*.c")
    add_deps("finsh", {public=true})
    add_defines("RT_USING_LIBC", {public = true})
    add_includedirs("common/include", {public=true})

    on_load(function (target)
        local conf = target:data("kconfig")
        local sdir = os.scriptdir()

        -- build libc dir
        local cdir = nil
        if conf.COMPILER_ARMCC or conf.COMPILER_ARMCLANG then
            cdir = path.join(sdir, "armlibc")
            target:add("defines", "RT_USING_ARMLIBC", {public = true})
        elseif conf.COMPILER_GCC or conf.COMPILER_ATFE_USE_NEWLIB then
            cdir = path.join(sdir, "newlib")
            target:add("defines", "RT_USING_NEWLIBC", {public = true})
            target:add("defines", "_POSIX_C_SOURCE=1", {public = true})
        elseif conf.COMPILER_ATFE_USE_PICOLIBC then
            cdir = path.join(sdir, "picolibc")
            target:add("defines", "RT_USING_PICOLIBC", {public = true})
        end

        -- add include dirs
        local dirs = os.dirs(path.join(cdir, "**"))
        for _, dir in ipairs(dirs) do
            target:add("includedirs", dir, {public = true})
        end

        -- add source files
        local files = os.files(path.join(cdir, "*.c"))
    end)