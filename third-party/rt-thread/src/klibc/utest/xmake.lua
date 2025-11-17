target("klibc_utest")
    set_kind("object")
    on_load(function(target) 
        local conf = target:data("kconfig")
        local sdir = os.scriptdir()
        if conf.RT_UTEST_TC_USING_KLIBC then
            target:add("files", path.join(sdir, "TC_*.c"))
        end
    end)
