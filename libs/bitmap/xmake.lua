target("bitmap")
    set_kind("headeronly")
    add_includedirs("inc", {public = true})
target_end()

if is_plat("linux", "windows") then
    add_requires("gtest")
    for _, file in ipairs(os.files("test/*.cc")) do
        target("test_" .. path.basename(file))
            if is_plat("linux") then
                set_policy("build.sanitizer.address", true)
            end
            set_languages("c++latest")
            set_kind("binary")
            set_default(false)
            add_files(file)
            add_deps("bitmap")
            add_tests("default")
            add_packages("gtest")
            add_includedirs("test")
        target_end()
    end
end
