-- Run: xmake l tests/target_selection.lua
-- Exercise actual target hooks without requiring an embedded compiler.

function main()
    local sdkdir = path.absolute(path.directory(os.scriptdir()))
    local work = os.tmpfile() .. "-xhive-target-selection"
    local variants = {
        {name = "bare_riscv", nation = false, nano = false, riscv = true},
        {name = "nation", nation = true, nano = false},
        {name = "nano", nation = false, nano = true},
        {name = "nano_riscv", nation = false, nano = true, riscv = true},
        {name = "nano_riscv_custom", nation = false, nano = true,
         riscv = true, custom_port = true}
    }
    os.mkdir(work)
    try {
        function ()
            for _, variant in ipairs(variants) do
                local projectdir = path.join(work, variant.name)
                os.mkdir(projectdir)
                io.writefile(path.join(projectdir, "xmake.lua"), string.format([[
add_moduledirs(%q)
rule("test.config")
    on_load(function(target)
        target:data_set("kconfig", {
            VENDOR_USE_NATION = %s,
            NATION_USE_N32H47X_48X = %s,
            NATION_N32H474 = %s,
            THIRD_RTOS_RTTNANO = %s,
            CPU_ARM = %s,
            CPU_RISCV = %s,
            CORE_ARM_CORTEX_M4 = true,
            RTTNANO_RISCV_PORT_CUSTOM = %s,
            CORE_RISCV_E906 = %s,
            RT_USING_FINSH = true,
            MSH_USING_BUILT_IN_COMMANDS = true,
            CLOCK_SYSCLK_HSE = true,
            CLOCK_HSE_ENABLE = true,
            CLOCK_HSE_FREQ = 8000000
        })
    end)
rule_end()
add_rules("test.config")
includes(%q)
includes(%q)
target("selection_probe")
    set_kind("phony")
    on_build(function(target)
        import("core.project.project")
        local conf = target:data("kconfig")
        local checks = {
            {name = "nation_n32h47x_48x", enabled = conf.NATION_USE_N32H47X_48X,
             source = "system_n32h47x_48x.c"},
            {name = "rttnano", enabled = conf.THIRD_RTOS_RTTNANO,
             source = conf.RTTNANO_RISCV_PORT_CUSTOM and "scheduler.c" or "context_gcc.S"}
        }
        for _, check in ipairs(checks) do
            local optional = assert(project.target(check.name))
            assert(not optional:is_default(), check.name .. " is a default target")
            local files = optional:sourcefiles()
            if check.enabled then
                local found = false
                for _, file in ipairs(files) do
                    found = found or path.filename(file) == check.source
                end
                assert(found, check.name .. " lost selected sources")
                assert(#table.wrap(optional:get("includedirs")) > 0)
            else
                assert(#files == 0, check.name .. " has unselected sources")
                assert(#table.wrap(optional:get("includedirs")) == 0)
            end
        end
        if conf.THIRD_RTOS_RTTNANO then
            local nano = project.target("rttnano")
            local files = nano:sourcefiles()
            local found = {}
            for _, file in ipairs(files) do
                found[path.filename(file)] = true
                if conf.RTTNANO_RISCV_PORT_CUSTOM then
                    local normalized = file:gsub("\\", "/")
                    assert(not normalized:find("/libcpu/", 1, true),
                           "custom port includes built-in CPU sources")
                    assert(not normalized:find("/rttnano/port/", 1, true),
                           "custom port includes SDK board sources")
                end
            end
            assert(found["scheduler.c"] and found["shell.c"] and found["cmd.c"],
                   "port selection lost kernel or MSH sources")
            if not conf.RTTNANO_RISCV_PORT_CUSTOM then
                assert(found["board.c"] and found["cpuport.c"] and found["context_gcc.S"],
                       "standard port lost board or context implementation")
            end
        end
        if conf.NATION_USE_N32H47X_48X then
            assert(table.contains(project.target("nation_n32h47x_48x"):get("defines"),
                                  "N32H474"))
            assert(table.contains(table.wrap(project.target("vendor_nation"):get("deps")),
                                  "nation_n32h47x_48x"))
        end
    end)
target_end()
]], path.join(sdkdir, "modules"), tostring(variant.nation),
                    tostring(variant.nation), tostring(variant.nation),
                    tostring(variant.nano), tostring(not variant.riscv),
                    tostring(variant.riscv or false),
                    tostring(variant.custom_port or false),
                    tostring(variant.custom_port or false),
                    path.join(sdkdir, "vendor/nation"),
                    path.join(sdkdir, "third-party/rttnano")))
                os.vrunv(os.programfile(), {"-P", projectdir})
                if variant.name == "bare_riscv" then
                    -- Explicitly building disabled targets must also be inert,
                    -- including RTT Nano's rtconfig.h generation hook.
                    os.vrunv(os.programfile(), {"build", "-P", projectdir,
                                               "nation_n32h47x_48x"})
                    os.vrunv(os.programfile(), {"build", "-P", projectdir, "rttnano"})
                    assert(not os.isfile(path.join(projectdir, "build/rtconfig.h")))
                end
                print("PASS: " .. variant.name)
            end
        end,
        catch {function (errors) raise(errors) end},
        finally {function () os.rm(work) end}
    }
end
