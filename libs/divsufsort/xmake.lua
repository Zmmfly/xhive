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
    local library_scope = interp_get_scopename()
target_end()

local standalone = path.normalize(os.projectdir()) == path.normalize(os.scriptdir())
local sdk_host = library_scope == "xhive_host::divsufsort"
if is_plat("linux", "windows", "macosx") and
   ((standalone and library_scope == "divsufsort") or
    (sdk_host and has_config("xhive_tests"))) then
    add_requires("gtest")

    -- Separate objects keep both index ABIs linkable in the same test binary.
    for _, bits in ipairs({32, 64}) do
        target("divsufsort_objects" .. bits)
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

    for _, file in ipairs(os.files("test/**.cc")) do
        local relative = path.relative(file, "test")
        if relative ~= "main.cc" then
            local suffix = relative:gsub("%.cc$", ""):gsub("[/\\]", "_")
            target("divsufsort_test_" .. suffix)
                set_kind("binary")
                set_default(false)
                set_languages("c++17")
                set_warnings("all", "error")
                add_files(file, "test/main.cc")
                add_deps("divsufsort_objects32", "divsufsort_objects64")
                add_packages("gtest")
                add_tests(path.basename(file), {
                    group = "xhive",
                    realtime_output = false,
                    run_timeout = 120 * 1000
                })
            target_end()
        end
    end

    -- Checked-in C/C++ consumers verify public headers and mixed linkage.
    for _, bits in ipairs({32, 64}) do
        target("divsufsort_test_consumer" .. bits)
            set_kind("binary")
            set_default(false)
            set_languages("c99", "c++11")
            set_warnings("all", "error")
            add_files("test/compile/consumer.c", "test/compile/headers.cpp")
            add_deps("divsufsort_objects" .. bits)
            if bits == 64 then
                add_defines("DIVSUFSORT_TEST_INDEX64")
            end
            add_tests("consumer", {
                group = "xhive",
                realtime_output = false,
                run_timeout = 120 * 1000
            })
        target_end()
    end
end

-- Opt-in compile-only targets: no RISC-V executable is run on the host.
if sdk_host and has_config("xhive_cross_tests") then
    for _, xlen in ipairs({32, 64}) do
        for _, bits in ipairs({32, 64}) do
            target("divsufsort_test_rv" .. xlen .. "_index" .. bits)
                set_kind("object")
                set_default(false)
                set_plat("cross")
                set_arch("riscv" .. xlen)
                set_toolchains("xhive_test_riscv")
                set_languages("c99")
                set_optimize("smallest")
                set_warnings("all", "extra", "error")
                add_files("libdivsufsort/lib/*.c")
                add_includedirs("libdivsufsort/include")
                add_cflags("-ffreestanding", "-fno-builtin", "-march=rv" .. xlen .. "im",
                           "-mabi=" .. (xlen == 32 and "ilp32" or "lp64"), {force = true})
                if bits == 64 then
                    add_defines("BUILD_DIVSUFSORT64")
                end
                add_tests("compile", {group = "xhive_cross", build_should_pass = true})
            target_end()
        end
    end
end
