target("ezctrl")
    set_kind("object")
    set_default(false)
    set_languages("c99")
    add_files("src/*.c")
    add_includedirs("inc", {public = true})
    local library_scope = interp_get_scopename()
target_end()

-- Declare independent host tests, never copies in the embedded namespace.
local standalone = path.normalize(os.projectdir()) == path.normalize(os.scriptdir())
if is_plat("linux", "windows", "macosx") and
   ((standalone and library_scope == "ezctrl") or
    (library_scope == "xhive_host::ezctrl" and has_config("xhive_tests"))) then
    add_requires("gtest")

    for _, file in ipairs(os.files("test/**.cc")) do
        local relative = path.relative(file, "test")
        if relative ~= "main.cc" then
            local suffix = relative:gsub("%.cc$", ""):gsub("[/\\]", "_")
            target("ezctrl_test_" .. suffix)
                set_kind("binary")
                set_default(false)
                set_languages("c++17")
                set_warnings("all", "error")
                add_files(file, "test/main.cc")
                add_deps("ezctrl")
                add_packages("gtest")
                add_tests(path.basename(file), {
                    group = "xhive",
                    realtime_output = false,
                    run_timeout = 120 * 1000
                })
            target_end()
        end
    end

    -- Checked-in C and C++ consumers verify the public include path, both
    -- header forms, and mixed-language linkage of the same objects.
    target("ezctrl_test_consumer")
        set_kind("binary")
        set_default(false)
        set_languages("c99", "c++17")
        set_warnings("all", "error")
        add_files("test/compile/consumer.c", "test/compile/headers.cpp")
        add_deps("ezctrl")
        add_tests("consumer", {
            group = "xhive",
            realtime_output = false,
            run_timeout = 120 * 1000
        })
    target_end()
end
