target("divsufsort")
    set_kind("object")
    set_default(false)
    set_languages("c99")
    add_files("libdivsufsort/lib/*.c")
    add_includedirs("libdivsufsort/include", {public = true})

    on_load(function(target)
        local conf = target:data("kconfig") or {}
        if conf.DIVSUFSORT_ENABLE_64BIT then
            target:add("defines", "BUILD_DIVSUFSORT64")
        end
    end)
target_end()
