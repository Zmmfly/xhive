target("bitmap")
    set_kind("headeronly")
    add_includedirs("inc", {public = true})
    local library_scope = interp_get_scopename()
target_end()

-- Declare independent host tests, never copies in the embedded namespace.
local standalone = path.normalize(os.projectdir()) == path.normalize(os.scriptdir())
if is_plat("linux", "windows", "macosx") and
   ((standalone and library_scope == "bitmap") or
    (library_scope == "xhive_host::bitmap" and has_config("xhive_tests"))) then
    add_requires("gtest")
    for _, file in ipairs(os.files("test/**.cc")) do
        local relative = path.relative(file, "test")
        if relative ~= "main.cc" then
            local suffix = relative:gsub("%.cc$", ""):gsub("[/\\]", "_")
            target("bitmap_test_" .. suffix)
                set_kind("binary")
                set_default(false)
                set_languages("c++latest")
                if is_plat("linux") then
                    set_policy("build.sanitizer.address", true)
                    -- SizeMaxBits deliberately tests allocation failure.
                    add_runenvs("ASAN_OPTIONS", "allocator_may_return_null=1")
                end
                add_files(file, "test/main.cc")
                add_deps("bitmap")
                add_packages("gtest")
                add_tests(path.basename(file), {
                    group = "xhive",
                    realtime_output = false,
                    run_timeout = 120 * 1000
                })
            target_end()
        end
    end
end
