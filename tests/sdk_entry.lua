-- Run: xmake l tests/sdk_entry.lua
-- Inspect real SDK/application target graphs in isolated Xmake caches. These
-- checks do not install packages or require a cross compiler / Kconfig tools.

function _probe(workdir, projectdir, body)
    os.mkdir(workdir)
    local script = path.join(workdir, "probe.lua")
    io.writefile(script, string.format([[
function main()
    import("core.project.config")
    import("core.project.project")
    config.load()
    config.set("plat", os.host())
    config.set("arch", os.arch())
    config.set("mode", "debug")
    config.set("builddir", %q)
    %s
end
]], path.join(workdir, "build"), body))
    return os.iorunv(os.programfile(), {"lua", "-P", projectdir, script}, {
        curdir = workdir, timeout = 30 * 1000,
        envs = {XMAKE_CONFIGDIR = workdir, XMAKE_STATS = "false", KCONFIG_CONFIG = ".config"}
    })
end

function main()
    local sdkdir = path.absolute(path.directory(os.scriptdir()))
    import("xhive.testing", {rootdir = path.join(sdkdir, "modules")})
    local work = os.tmpfile() .. "-xhive-sdk-entry"
    os.mkdir(work)
    try {
        function ()
            -- All cleanup fixtures are disposable data under this workspace.
            -- Never link a test fixture to the SDK or any pre-existing directory.
            local disposable = path.join(work, "owned-xhive-cleanup")
            local retained = path.join(work, "retained")
            os.mkdir(disposable)
            os.mkdir(retained)
            local sentinel = path.join(retained, "sentinel.txt")
            io.writefile(sentinel, "must survive nested cleanup")
            io.writefile(path.join(disposable, "owned.txt"), "temporary data")
            if os.host() ~= "windows" then
                os.ln(retained, path.join(disposable, "linked-data"))
            end
            testing.remove_workdir(disposable)
            assert(not os.isdir(disposable))
            assert(io.readfile(sentinel) == "must survive nested cleanup")
            if os.host() ~= "windows" then
                local linked = path.join(work, "linked-xhive-cleanup")
                os.ln(retained, linked)
                local rejected = false
                try {
                    function () testing.remove_workdir(linked) end,
                    catch {function (errors)
                        rejected = tostring(errors):find("Refusing to remove", 1, true) ~= nil
                    end}
                }
                assert(rejected, "A linked workspace root was accepted")
                assert(io.readfile(sentinel) == "must survive nested cleanup")
            end
            print("PASS: cleanup removes owned data without traversing directory links")

            local standalone = [[
    config.set("xhive_tests", true)
    config.set("xhive_cross_tests", false)
    local targets = project.targets()
    assert(targets["xhive_host::deps"])
    assert(targets["xhive_host::base64"])
    assert(targets["xhive_host::heatshrink"])
    assert(targets["xhive_host::micro-ecc"])
    assert(targets["xhive_host::mbedtls"])
    assert(targets["xhive_test_c2"])
    assert(not targets["xhive_test_c2_full"])
    assert(not targets["xhive_test_riscv_toolset"])
    for name, target in pairs(targets) do
        assert(not name:find("xhive_embed::", 1, true), name)
        assert(not target:rule("xhive.conf"), name)
        assert(not target:rule("xhive.embed"), name)
        assert(not target:data("kconfig"), name)
        assert(not table.contains(table.wrap(target:get("defines")), "BUILD_IN_XHIVE"), name)
        if target:get("tests") then
            assert(not target:is_default(), name)
        end
    end
    local deps = table.wrap(targets["xhive_host::libs"]:get("deps"))
    for _, name in ipairs({"base64", "bitmap", "heatshrink", "micro-ecc", "cjson"}) do
        assert(table.contains(deps, name), name)
    end
    if config.get("plat") == "linux" or config.get("plat") == "windows" or config.get("plat") == "macosx" then
        assert(targets["xhive_host::test_base64_decode"])
        assert(targets["xhive_host::test_bitmap"])
    end
]]
            _probe(path.join(work, "standalone"), sdkdir, standalone)
            print("PASS: standalone SDK graph has native defaults and no application configuration")

            _probe(path.join(work, "disabled"), sdkdir, [[
    config.set("xhive_tests", false)
    config.set("xhive_cross_tests", false)
    for name, target in pairs(project.targets()) do
        assert(not target:get("tests"), name)
    end
    assert(#table.wrap(project.get("xhive_host::requires")) == 0)
]])
            print("PASS: disabling tests removes tests and GoogleTest requirement")

            _probe(path.join(work, "cross"), sdkdir, [[
    config.set("xhive_tests", false)
    config.set("xhive_cross_tests", true)
    local targets = project.targets()
    assert(targets["xhive_test_c2_full"])
    assert(targets["xhive_test_riscv_toolset"])
    assert(targets["xhive_test_riscv_startup"])
    assert(not targets["xhive_test_c2"])
    assert(not targets["xhive_embed::deps"])
]])
            print("PASS: cross checks are explicit and independent of firmware targets")

            -- Check the adapter's negative paths without a real cross compiler.
            import("core.project.config")
            local failing = path.join(work, "failure.lua")
            io.writefile(failing, 'function main() raise("expected SDK test failure") end\n')
            local values = {["xhive.test.script"] = failing}
            local mock = {values = function (_, key) return values[key] end}
            local result = {run_timeout = 5000}
            assert(not testing(mock, result))
            assert(result.errors:find("expected SDK test failure", 1, true))
            local old_compiler = config.get("xhive_riscv_gcc")
            config.set("xhive_riscv_gcc", path.join(work, "missing-riscv-gcc"))
            values["xhive.test.riscv_gcc"] = true
            result = {run_timeout = 5000}
            local passed = testing(mock, result)
            config.set("xhive_riscv_gcc", old_compiler)
            assert(not passed and result.errors:find("RISC-V GCC not found", 1, true),
                   "Missing cross compiler fell back to host GCC")
            print("PASS: script failures and unavailable cross compilers fail the test")

            local appdir = path.join(work, "application")
            os.mkdir(appdir)
            io.writefile(path.join(appdir, "xmake.lua"), string.format([[
includes(%q)
target("firmware")
    set_kind("binary")
    add_rules("xhive.embed")
target_end()
]], sdkdir))
            local missing_error
            try {
                function () _probe(path.join(work, "missing"), appdir, "project.targets()") end,
                catch {function (errors) missing_error = tostring(errors) end}
            }
            assert(missing_error and missing_error:find("Application configuration not found", 1, true),
                   missing_error or "Missing application configuration was accepted")
            print("PASS: missing application configuration fails with an actionable diagnostic")

            io.writefile(path.join(appdir, ".config"), [[
CONFIG_CPU_RISCV=y
CONFIG_ARCH_RISCV_RV32=y
CONFIG_RISCV_ISA_I=y
CONFIG_RISCV_ISA_M=y
CONFIG_COMPILER_RISCV_GCC=y
CONFIG_VENDOR_USE_NONE=y
CONFIG_THIRD_RTOS_NONE=y
CONFIG_LIBS_ADD_BASE64=y
]])
            _probe(path.join(work, "application-cache"), appdir, [[
    import("core.cache.memcache")
    -- This is a target-graph test, not a Kconfig generator test. Model an
    -- already generated header so these graph checks do not require genconfig.
    memcache.set("xhive", "header_on_load_" .. os.projectdir(), true)
    memcache.set("xhive", "header_before_build_" .. os.projectdir(), true)
    config.set("xhive_tests", true)
    local targets = project.targets()
    assert(targets["xhive_embed::deps"])
    assert(targets["xhive_host::deps"])
    assert(not targets["xhive_test_c2"])
    assert(not targets["xhive_test_sdk_entry"])
    for name, target in pairs(targets) do
        if name:find("xhive_embed::", 1, true) then
            assert(not target:get("tests"), name)
            assert(target:plat() == "cross", name)
        end
    end
    local deps = table.wrap(targets["xhive_host::libs"]:get("deps"))
    assert(#deps == 1 and deps[1] == "base64")
    assert(targets["xhive_host::base64"]:data("kconfig").LIBS_ADD_BASE64)
    assert(targets.firmware:plat() == "cross")
    assert(table.contains(table.wrap(targets.firmware:get("deps")), "xhive_embed::deps"))
]])
            assert(not os.isfile(path.join(appdir, "build", "xhive_config.h")))
            print("PASS: external includes retain application config and isolate host/embed tests")

            -- Exercise the real Kconfig entry generator without writing in the
            -- SDK or needing a parser. Change only the cached paths in this process.
            import("xhive.base", {rootdir = path.join(sdkdir, "modules")})
            import("xhive.kconf", {rootdir = path.join(sdkdir, "modules")})
            local dirs = base.load_paths()
            local original = {prjdir = dirs.prjdir, builddir = dirs.builddir, kconf_entry = dirs.kconf_entry}
            try {
                function ()
                    dirs.prjdir = dirs.sdkdir
                    dirs.builddir = path.join(work, "kconfig")
                    dirs.kconf_entry = path.join(dirs.builddir, "Kconfig")
                    kconf.build_entry()
                    local content = io.readfile(dirs.kconf_entry)
                    local _, count = content:gsub("source ", "")
                    assert(count == 1, "SDK Kconfig was included more than once")
                    io.writefile(path.join(appdir, "Kconfig"), "config SDK_ENTRY_TEST\n    bool\n")
                    dirs.prjdir = appdir
                    kconf.build_entry()
                    content = io.readfile(dirs.kconf_entry)
                    _, count = content:gsub("source ", "")
                    assert(count == 2, "Application Kconfig was not included")
                end,
                finally {function ()
                    dirs.prjdir, dirs.builddir, dirs.kconf_entry = original.prjdir, original.builddir, original.kconf_entry
                end}
            }
            print("PASS: Kconfig entry includes the SDK once and an optional application once")
        end,
        finally {function () testing.remove_workdir(work) end}
    }
end
