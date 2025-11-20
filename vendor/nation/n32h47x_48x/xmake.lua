--[[
Copyright (c) 2025 Zmmfly. All rights reserved.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors
   may be used to endorse or promote products derived from this software
   without specific prior written permission.
]]

target("n32h47x_48x")
    set_kind("object")
    -- CMSIS
    add_files("firmware/CMSIS/device/system_n32h47x_48x.c")
    add_includedirs(
        "firmware/CMSIS/core",
        "firmware/CMSIS/device",
        {public = true}
    )
    -- std periph driver
    add_files("firmware/n32h47x_48x_std_periph_driver/src/*.c")
    add_includedirs( 
        "firmware/n32h47x_48x_std_periph_driver/inc",
        {public = true}
    )
    -- algo library
    add_files("firmware/n32h47x_48x_algo_lib/src/*.c")
    add_includedirs(
        "firmware/n32h47x_48x_algo_lib/inc",
        {public = true}
    )

    on_load(function(target)
        -- Import the xhive.base module for utility functions
        import("xhive.base")
        local sdir = os.scriptdir()
        local dirs = base.load_paths()
        local srcs = {}
        local incs = {}
        local defs = {}
        local into = function(tbl, val) 
            table.insert(tbl, val)
        end

        -- Get the kconfig parsed data for the target
        local conf = target:data("kconfig")
        if not conf then
            raise("kconfig not found for target: " .. target:name() .. ". Please make sure to include the xhive.common rule in your project.")
        end

        -- Add HSE_VALUE define if HSE is enabled
        if conf.CLOCK_HSE_ENABLE then
            into(defs, "HSE_VALUE=" .. (conf.CLOCK_HSE_FREQ or 8000000))
        end

        -- Add SYSCLK choose define
        if conf.CLOCK_SYSCLK_HSI then
            into(defs, "HSI_VALUE=" .. (conf.CLOCK_HSI_FREQ or 8000000))
        elseif conf.CLOCK_SYSCLK_HSE then
            into(defs, "SYSCLK_SRC=SYSCLK_USE_HSE")
        elseif conf.CLOCK_SYSCLK_PLL and conf.CLOCK_PLL_SRC_HSI then
            into(defs, "SYSCLK_SRC=SYSCLK_USE_HSI_PLL")
        elseif conf.CLOCK_SYSCLK_PLL and conf.CLOCK_PLL_SRC_HSE then
            into(defs, "SYSCLK_SRC=SYSCLK_USE_HSE_PLL")
        else
            raise("No valid SYSCLK source selected in kconfig for target: " .. target:name())
        end

        -- Add series define
        local defs_map = {
            NATION_N32H473 = "N32H473",
            NATION_N32H474 = "N32H474",
            NATION_N32H475 = "N32H475",
            NATION_N32H481 = "N32H481",
            NATION_N32H482 = "N32H482",
            NATION_N32H487 = "N32H487",
            NATION_N32H488 = "N32H488",
        }
        for k, v in pairs(defs_map) do
            if conf[k] then
                into(defs, v)
                break
            end
        end

        if conf.NATION_N32H47X_48X_USE_USBFS or conf.NATION_N32H47X_48X_USE_USBHS then
            if not conf.NATION_N32H47X_48X_EXTERNAL_INC_DIR or conf.NATION_N32H47X_48X_EXTERNAL_INC_DIR == "" then
                raise("External USB include directory not set in kconfig for target: " .. target:name())
            end
            local ext_inc = path.join(dirs.prjdir, conf.NATION_N32H47X_48X_EXTERNAL_INC_DIR)
            if os.isdir(ext_inc) then
                target:add("includedirs", ext_inc)
            else
                raise("External USB include directory not found: " .. ext_inc)
            end
        end

        -- Enable USBFS driver if selected
        if conf.NATION_N32H47X_48X_USE_USBFS then
            local fs_dir = path.join(sdir, "firmware", "n32h47x_48x_usbfsd_driver")
            -- Add USBFS files
            into(srcs, path.join(fs_dir, "src", "*.c"))
            -- Add USBFS include directories
            into(incs, path.join(fs_dir, "inc"))
        end

        -- Enable USBHS driver if selected
        if conf.NATION_N32H47X_48X_USE_USBHS then
            local hs_dir   = path.join(sdir, "firmware", "n32h47x_48x_usbhs_driver")
            local dev_dir  = path.join(hs_dir, "device")
            local host_dir = path.join(hs_dir, "host")
            -- Add USBHS driver files
            into(srcs, path.join(hs_dir, "driver", "src", "*.c"))
            into(srcs, path.join(dev_dir, "core", "src", "*.c"))
            into(srcs, path.join(dev_dir, "class", "audio", "src", "*.c"))
            into(srcs, path.join(dev_dir, "class", "cdc", "src", "*.c"))
            into(srcs, path.join(dev_dir, "class", "customhid", "src", "*.c"))
            into(srcs, path.join(dev_dir, "class", "hid_cdc_composite", "src", "*.c"))
            into(srcs, path.join(dev_dir, "class", "hid_keyboard", "src", "*.c"))
            into(srcs, path.join(dev_dir, "class", "hid_msc_composite", "src", "*.c"))
            into(srcs, path.join(dev_dir, "class", "mouse", "src", "*.c"))
            into(srcs, path.join(dev_dir, "class", "msc", "src", "*.c"))
            into(srcs, path.join(dev_dir, "class", "msc_cdc_composite", "src", "*.c"))
            into(srcs, path.join(host_dir, "core", "src", "*.c"))
            into(srcs, path.join(host_dir, "class", "CDC", "src", "*.c"))
            into(srcs, path.join(host_dir, "class", "HID", "src", "*.c"))
            into(srcs, path.join(host_dir, "class", "MSC", "src", "*.c"))

            -- Add USBHS include directories
            into(incs, path.join(hs_dir, "driver", "inc"))
            into(incs, path.join(dev_dir, "core", "inc"))
            into(incs, path.join(dev_dir, "class", "audio", "inc"))
            into(incs, path.join(dev_dir, "class", "cdc", "inc"))
            into(incs, path.join(dev_dir, "class", "customhid", "inc"))
            into(incs, path.join(dev_dir, "class", "hid_cdc_composite", "inc"))
            into(incs, path.join(dev_dir, "class", "hid_keyboard", "inc"))
            into(incs, path.join(dev_dir, "class", "hid_msc_composite", "inc"))
            into(incs, path.join(dev_dir, "class", "mouse", "inc"))
            into(incs, path.join(dev_dir, "class", "msc", "inc"))
            into(incs, path.join(dev_dir, "class", "msc_cdc_composite", "inc"))
            into(incs, path.join(host_dir, "core", "inc"))
            into(incs, path.join(host_dir, "class", "CDC", "inc"))
            into(incs, path.join(host_dir, "class", "HID", "inc"))
            into(incs, path.join(host_dir, "class", "MSC", "inc"))
        end

        -- USB device / host mode selection
        if conf.NATION_N32H47X_48X_USB_MODE_DEVICE then
            into(defs, "USB_DEVICE_MODE")
        end
        if conf.NATION_N32H47X_48X_USB_MODE_HOST then
            into(defs, "USB_HOST_MODE")
        end

        -- Enable algo library if selected
        if conf.NATION_N32H47X_48X_ALGO then
            into(defs, "N32H47X_48X_ALGO")
            local algo_dir = path.join(sdir, "firmware", "n32h47x_48x_algo_lib")
            into(incs, path.join(algo_dir, "inc"))
            into(srcs, path.join(algo_dir, "src", "*.c"))
        end

        target:add("defines", defs, {public = true})
        target:add("files", srcs)
        target:add("includedirs", incs, {public = true})
    end)
    on_config(function(target)
        import("core.base.json")
        import("xhive.proc")
        import("xhive.base")
        -- Load the peripheral ISR list based on the selected series
        local prefix = nil
        local sdir   = path.absolute(os.scriptdir())
        local dirs   = base.load_paths()
        local conf   = target:data("kconfig")

        local model_maps = {
            NATION_N32H473 = "n32h473",
            NATION_N32H474 = "n32h474",
            NATION_N32H475 = "n32h475",
            NATION_N32H481 = "n32h481",
            NATION_N32H482 = "n32h482",
            NATION_N32H487 = "n32h487",
            NATION_N32H488 = "n32h488",
        }
        for model, pre in pairs(model_maps) do
            if conf[model] then
                prefix = pre
                break
            end
        end
        if not prefix then
            raise("No valid N32H47X series selected in kconfig for target: " .. target:name())
        end

        local lst_path = path.join(sdir, "periph_isr", prefix .. "_periph_isr.json")
        local isr_lst  = json.loadfile(lst_path)

        -- Build the startup file with the peripheral ISR list

        if conf.USE_DEFAULT_STARTUP then
            local template_path = proc.load_startup_template_path()
            local output_path   = path.join(dirs.builddir, prefix .. "_startup.c")
            proc.build_arm_startup(template_path, isr_lst, output_path)
            target:add("files", output_path)
        end
    end)
target_end()
