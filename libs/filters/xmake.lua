local standalone = path.normalize(os.projectdir()) == path.normalize(os.scriptdir())
if standalone then
    -- A dependency loaded from this entry does not register its own tests.
    includes("../linears")
end

target("filters")
    set_kind("object")
    set_default(false)
    set_languages("c99")
    add_files("src/*.c")
    add_includedirs("include", {public = true})
    add_deps("linears", {public = true})
    local library_scope = interp_get_scopename()
target_end()

if is_plat("linux", "windows", "macosx") and
   ((standalone and library_scope == "filters") or
    (library_scope == "xhive_host::filters" and has_config("xhive_tests"))) then
    add_requires("gtest")
    add_requires("eigen 3.4.0", {system = false})
    for _, file in ipairs(os.files("test/**.cc")) do
        local relative = path.relative(file, "test")
        if relative ~= "main.cc" then
            local suffix = relative:gsub("%.cc$", ""):gsub("[/\\]", "_")
            target("filters_test_" .. suffix)
                set_kind("binary")
                set_default(false)
                set_languages("c++17")
                add_files(file, "test/main.cc")
                add_deps("filters")
                add_packages("gtest", "eigen")
                add_tests(path.basename(file), {
                    group = "xhive",
                    realtime_output = false,
                    run_timeout = 120 * 1000
                })
            target_end()
        end
    end
end
