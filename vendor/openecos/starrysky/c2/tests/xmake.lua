-- Keep the existing C2 software suite in its owning component. These wrappers
-- do not load the firmware graph or perform hardware access.
for _, variant in ipairs({
    {target = "xhive_test_c2", name = "host", option = "xhive_tests", group = "xhive"},
    {target = "xhive_test_c2_full", name = "full", option = "xhive_cross_tests", group = "xhive_cross"}
}) do
    if has_config(variant.option) then
        target(variant.target)
            set_kind("phony")
            set_default(false)
            add_tests(variant.name, {
                group = variant.group,
                realtime_output = false,
                run_timeout = 120 * 1000
            })
            on_test(function(target, opt)
                import("core.project.config")
                import("lib.detect.find_program")
                import("xhive.testing")
                local python = find_program("python3") or find_program("python")
                if not python then
                    opt.errors = "Python 3 is required for the C2 software tests"
                    return false
                end
                local args = {path.join(target:scriptdir(), "run_tests.py")}
                if variant.name == "host" then
                    table.insert(args, "--host-only")
                else
                    table.join2(args, {"--clang", config.get("xhive_clang") or "clang"})
                end
                return testing.run(python, args, opt)
            end)
        target_end()
    end
end
