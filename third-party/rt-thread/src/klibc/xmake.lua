includes("utest")
target("klibc")
    set_kind("object")
    set_default(false)
    add_deps("finsh", {public=true})
    add_deps("klibc_utest", {public=true})
    add_files("kerrno.c", "kstdio.c", "kstring.c")
    on_load(function(target) 
        local conf = target:data("kconfig")
        local sdir = os.scriptdir()
        if not conf.RT_KLIBC_USING_LIBC_VSNPRINTF then
            if conf.RT_KLIBC_USING_VSNPRINTF_STANDARD then
                target:add("files", path.join(sdir, "rt_vsnprintf_std.c"))
            else
                target:add("files", path.join(sdir, "rt_vsnprintf_tiny.c"))
            end
        end

        if not conf.RT_KLIBC_USING_LIBC_VSSCANF then
            target:add("files", path.join(sdir, "rt_vsscanf.c"))
        end
    end)
