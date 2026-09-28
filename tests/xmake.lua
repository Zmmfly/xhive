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

-- Only build-system regressions live here. Library tests are ordinary
-- targets declared in each library's xmake.lua, outside its production target.
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
end

if has_config("xhive_cross_tests") then
    for _, file in ipairs(os.files("riscv_*.lua")) do
        local name = path.basename(file)
        target("xhive_test_" .. name)
            set_kind("phony")
            set_default(false)
            add_values("xhive.test.script", path.join(os.scriptdir(), file))
            add_values("xhive.test.args", get_config("xhive_riscv_gcc") or "riscv-none-elf-gcc")
            add_tests(name, {group = "xhive_cross", realtime_output = false, run_timeout = 120 * 1000})
            on_test("xhive.testing")
        target_end()
    end

    -- Exact compiler selection for compile-only native Xmake test targets.
    toolchain("xhive_test_riscv")
        set_kind("standalone")
        set_toolset("cc", "gcc@" .. (get_config("xhive_riscv_gcc") or "riscv-none-elf-gcc"))
        on_check(function(toolchain)
            import("core.project.config")
            import("lib.detect.find_program")
            local name = config.get("xhive_riscv_gcc") or "riscv-none-elf-gcc"
            -- Do not let compiler-name canonicalization select host GCC.
            assert(find_program(name),
                   "RISC-V GCC not found: %s; configure --xhive_riscv_gcc=/path/to/gcc", name)
            return true
        end)
    toolchain_end()
end
