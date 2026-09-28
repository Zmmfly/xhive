if is_plat("linux", "windows", "macosx") then
    add_requires("gtest")

    -- Independent objects let one consumer link both ABI variants together.
    for _, bits in ipairs({32, 64}) do
        target("divsufsort_test" .. bits)
            set_kind("object")
            set_default(false)
            set_languages("c99")
            add_files("libdivsufsort/lib/*.c")
            add_includedirs("libdivsufsort/include", {public = true})
            if bits == 64 then
                add_defines("BUILD_DIVSUFSORT64")
            end
        target_end()
    end

    for _, file in ipairs(os.files("test/*.cc")) do
        if path.basename(file) ~= "main" then
            target("test_" .. path.basename(file))
                set_kind("binary")
                set_default(false)
                set_languages("c++17")
                set_warnings("all", "error")
                add_files(file, "test/main.cc")
                add_deps("divsufsort_test32", "divsufsort_test64")
                add_packages("gtest")
                add_tests("default", {
                    group = "xhive",
                    realtime_output = false,
                    run_timeout = 120 * 1000
                })
            target_end()
        end
    end
end
