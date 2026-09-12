-- Run: xmake l tests/target_selection.lua
-- Exercise actual target hooks without requiring an embedded compiler.

function main()
    local sdkdir = path.absolute(path.directory(os.scriptdir()))
    local work = os.tmpfile() .. "-xhive-target-selection"
    local variants = {
        {name = "bare_riscv", nation = false, nano = false},
        {name = "nation", nation = true, nano = false},
        {name = "nano", nation = false, nano = true}
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
             source = "context_gcc.S"}
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
                    tostring(variant.nano), tostring(variant.name ~= "bare_riscv"),
                    tostring(variant.name == "bare_riscv"),
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
