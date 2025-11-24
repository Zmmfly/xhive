target("base64")
    set_kind("object")
    set_default(false)
    add_files("src/*.c")
    add_includedirs("inc", {public = true})
target_end()

if is_plat("linux", "windows") then
    add_requires("gtest")
    set_languages("c++latest")
    for _, file in ipairs(os.files("test/*.cc")) do
        target("test_" .. path.basename(file))
            set_kind("binary")
            set_default(false)
            add_files(file)
            add_deps("base64")
            add_tests("default")
            add_packages("gtest")
            add_includedirs("test")
        target_end()
    end
end
