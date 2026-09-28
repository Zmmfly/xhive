-- SDK-only regression targets. External applications do not include this file.
option("xhive_cross_tests")
    set_default(false)
    set_showmenu(true)
    set_description("Register optional RISC-V compiler/linker regression tests")
option_end()

option("xhive_riscv_gcc")
    set_default("riscv-none-elf-gcc")
    set_showmenu(true)
    set_description("RISC-V GCC executable for optional compiler/linker tests")
option_end()

option("xhive_clang")
    set_default("clang")
    set_showmenu(true)
    set_description("Clang executable with a RISC-V backend for C2 full checks")
option_end()

-- One native script test per file. Cross-compiler tests are opt-in below.
if has_config("xhive_tests") then
    for _, file in ipairs(os.files("*.lua|xmake.lua|riscv_*.lua")) do
        local name = path.basename(file)
        target("xhive_test_" .. name)
            set_kind("phony")
            set_default(false)
            add_values("xhive.test.script", path.join(os.scriptdir(), file))
            add_tests(name, {group = "xhive", realtime_output = false, run_timeout = 120 * 1000})
            on_test("xhive.testing")
        target_end()
    end

    target("xhive_test_c2")
        set_kind("phony")
        set_default(false)
        add_values("xhive.test.script", path.join(os.scriptdir(), "../vendor/openecos/starrysky/c2/tests/run_tests.py"))
        add_values("xhive.test.python", true)
        add_values("xhive.test.args", "--host-only")
        add_tests("host", {group = "xhive", realtime_output = false, run_timeout = 120 * 1000})
        on_test("xhive.testing")
    target_end()
end

if has_config("xhive_cross_tests") then
    for _, file in ipairs(os.files("riscv_*.lua")) do
        local name = path.basename(file)
        target("xhive_test_" .. name)
            set_kind("phony")
            set_default(false)
            add_values("xhive.test.script", path.join(os.scriptdir(), file))
            add_values("xhive.test.riscv_gcc", true)
            add_tests(name, {group = "xhive_cross", realtime_output = false, run_timeout = 120 * 1000})
            on_test("xhive.testing")
        target_end()
    end

    target("xhive_test_c2_full")
        set_kind("phony")
        set_default(false)
        add_values("xhive.test.script", path.join(os.scriptdir(), "../vendor/openecos/starrysky/c2/tests/run_tests.py"))
        add_values("xhive.test.python", true)
        add_values("xhive.test.clang", true)
        add_tests("full", {group = "xhive_cross", realtime_output = false, run_timeout = 120 * 1000})
        on_test("xhive.testing")
    target_end()
end
