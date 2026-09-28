-- Included by the SDK host namespace or the standalone component project.
if is_plat("linux", "windows", "macosx") then
    add_requires("gtest")
    for _, file in ipairs(os.files("test/*.cc")) do
        target("test_" .. path.basename(file))
            set_kind("binary")
            set_default(false)
            set_languages("c++latest")
            add_files(file)
            add_deps("base64")
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
