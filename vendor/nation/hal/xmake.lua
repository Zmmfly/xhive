-- Nation SHRTIM high-level HAL component.
--
-- Registers one object target `nation_hal` on top of the vendor std-periph
-- driver. Files are only added when the Nation N32H47x/48x vendor package is
-- selected in kconfig (same guard pattern as n32h47x_48x/xmake.lua).

target("nation_hal")
    set_kind("object")
    set_default(false)

    on_load(function(target)
        local conf = target:data("kconfig")
        if not conf or not conf.NATION_USE_N32H47X_48X then
            return
        end

        local sdir = os.scriptdir()
        target:add("files", path.join(sdir, "src", "*.c"))
        target:add("includedirs", path.join(sdir, "inc"), {public = true})
        target:add("deps", "nation_n32h47x_48x", {public = true})
    end)
target_end()

-- Compile check for the usage examples: not linked into any firmware image.
-- Registered like `nation_hal` under the same kconfig guard; SDK targets
-- are only reachable through an application's dependency graph (they
-- cannot be built by name from a demo project). A standalone check works
-- with:
--   arm-none-eabi-gcc -std=c99 -Wall -Wextra -Iinc -c examples/*.c
target("nation_hal_examples")
    set_kind("object")
    set_default(false)

    on_load(function(target)
        local conf = target:data("kconfig")
        if not conf or not conf.NATION_USE_N32H47X_48X then
            return
        end

        local sdir = os.scriptdir()
        target:add("files", path.join(sdir, "examples", "*.c"))
        target:add("deps", "nation_hal")
    end)
target_end()
