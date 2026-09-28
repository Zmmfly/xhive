-- Included by the SDK host namespace or the standalone component project.
if is_plat("linux", "windows", "macosx") then
    add_requires("gtest")
    for _, file in ipairs(os.files("test/*.cc")) do
        target("test_" .. path.basename(file))
            if is_plat("linux") then
                set_policy("build.sanitizer.address", true)
                -- SizeMaxBits deliberately tests allocation failure. Keep
                -- ASan enabled, but let the allocator return NULL here.
                add_runenvs("ASAN_OPTIONS", "allocator_may_return_null=1")
            end
            set_languages("c++latest")
            set_kind("binary")
            set_default(false)
            add_files(file)
            add_deps("bitmap")
            add_tests("default", {
                group = "xhive",
                realtime_output = false,
                run_timeout = 120 * 1000
            })
            add_packages("gtest")
            add_includedirs("test")
        target_end()
    end
end
