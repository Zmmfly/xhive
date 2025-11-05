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
    -- FIXME: the armcc library is not compatible with gcc/clang toolchains
    -- add_files("firmware/n32h47x_48x_algo_lib/lib/*.a")
    -- add_includedirs(
    --     "firmware/n32h47x_48x_algo_lib/inc",
    --     {public = true}
    -- )

    on_load(function(target)
        -- Import the xhive.base module for utility functions
        import("xhive.base")

        -- Get the kconfig parsed data for the target
        local kconfig = target:data("kconfig")
        if not kconfig then
            raise("kconfig not found for target: " .. target:name() .. ". Please make sure to include the xhive.common rule in your project.")
        end

        -- Add HSE_VALUE define if HSE is enabled
        if kconfig.CLOCK_HSE_ENABLE then
            target:add("defines", "HSE_VALUE=" .. (kconfig.CLOCK_HSE_FREQ or 8000000), {public=true})
        end

        -- Add SYSCLK choose define
        if kconfig.CLOCK_SYSCLK_HSI then
            target:add("defines", "SYSCLK_SRC=SYSCLK_USE_HSI")
        elseif kconfig.CLOCK_SYSCLK_HSE then
            target:add("defines", "SYSCLK_SRC=SYSCLK_USE_HSE")
        elseif kconfig.CLOCK_SYSCLK_PLL and kconfig.CLOCK_PLL_SRC_HSI then
            target:add("defines", "SYSCLK_SRC=SYSCLK_USE_HSI_PLL")
        elseif kconfig.CLOCK_SYSCLK_PLL and kconfig.CLOCK_PLL_SRC_HSE then
            target:add("defines", "SYSCLK_SRC=SYSCLK_USE_HSE_PLL")
        else
            raise("No valid SYSCLK source selected in kconfig for target: " .. target:name())
        end

        -- Add series define
        local startup_file_name = ""
        if kconfig.NATION_N32H473 then
            target:add("defines", "N32H473", {public=true})
        elseif kconfig.NATION_N32H474 then
            target:add("defines", "N32H474", {public=true})
        elseif kconfig.NATION_N32H475 then 
            target:add("defines", "N32H475", {public=true})
        elseif kconfig.NATION_N32H481 then
            target:add("defines", "N32H481", {public=true})
        elseif kconfig.NATION_N32H482 then
            target:add("defines", "N32H482", {public=true})
        elseif kconfig.NATION_N32H487 then
            target:add("defines", "N32H487", {public=true})
        elseif kconfig.NATION_N32H488 then
            target:add("defines", "N32H488", {public=true})
        else
            raise("No valid N32H47X series selected in kconfig for target: " .. target:name())
        end

        -- Enable USBFS driver if selected
        if kconfig.NATION_N32H47X_48X_USE_USBFS then
            -- Add USBFS files
            target:add("files", "$(scriptdir)/firmware/n32h47x_48x_usbfsd_driver/src/*.c")
            -- Add USBFS include directories
            target:add("includedirs", "$(scriptdir)/firmware/n32h47x_48x_usbfsd_driver/inc", {public=true})
        end

        -- Enable USBHS driver if selected
        if kconfig.NATION_N32H47X_48X_USE_USBHS then
            -- Add USBHS driver files
            target:add("files", "$(scriptdir)/firmware/n32h47x_48x_usbhs_driver/driver/src/*.c")
            target:add("files", "$(scriptdir)/firmware/n32h47x_48x_usbhs_driver/device/core/src/*.c")
            target:add("files", "$(scriptdir)/firmware/n32h47x_48x_usbhs_driver/device/class/audio/src/*.c")
            target:add("files", "$(scriptdir)/firmware/n32h47x_48x_usbhs_driver/device/class/cdc/src/*.c")
            target:add("files", "$(scriptdir)/firmware/n32h47x_48x_usbhs_driver/device/class/customhid/src/*.c")
            target:add("files", "$(scriptdir)/firmware/n32h47x_48x_usbhs_driver/device/class/hid_cdc_composite/src/*.c")
            target:add("files", "$(scriptdir)/firmware/n32h47x_48x_usbhs_driver/device/class/hid_keyboard/src/*.c")
            target:add("files", "$(scriptdir)/firmware/n32h47x_48x_usbhs_driver/device/class/hid_msc_composite/src/*.c")
            target:add("files", "$(scriptdir)/firmware/n32h47x_48x_usbhs_driver/device/class/mouse/src/*.c")
            target:add("files", "$(scriptdir)/firmware/n32h47x_48x_usbhs_driver/device/class/msc/src/*.c")
            target:add("files", "$(scriptdir)/firmware/n32h47x_48x_usbhs_driver/device/class/msc_cdc_composite/src/*.c")
            target:add("files", "$(scriptdir)/firmware/n32h47x_48x_usbhs_driver/host/core/src/*.c")
            target:add("files", "$(scriptdir)/firmware/n32h47x_48x_usbhs_driver/host/class/CDC/src/*.c")
            target:add("files", "$(scriptdir)/firmware/n32h47x_48x_usbhs_driver/host/class/HID/src/*.c")
            target:add("files", "$(scriptdir)/firmware/n32h47x_48x_usbhs_driver/host/class/MSC/src/*.c")

            -- Add USBHS include directories
            target:add("includedirs", "$(scriptdir)/firmware/n32h47x_48x_usbhs_driver/driver/inc", {public=true})
            target:add("includedirs", "$(scriptdir)/firmware/n32h47x_48x_usbhs_driver/device/core/inc", {public=true})
            target:add("includedirs", "$(scriptdir)/firmware/n32h47x_48x_usbhs_driver/device/class/audio/inc", {public=true})
            target:add("includedirs", "$(scriptdir)/firmware/n32h47x_48x_usbhs_driver/device/class/cdc/inc", {public=true})
            target:add("includedirs", "$(scriptdir)/firmware/n32h47x_48x_usbhs_driver/device/class/customhid/inc", {public=true})
            target:add("includedirs", "$(scriptdir)/firmware/n32h47x_48x_usbhs_driver/device/class/hid_cdc_composite/inc", {public=true})
            target:add("includedirs", "$(scriptdir)/firmware/n32h47x_48x_usbhs_driver/device/class/hid_keyboard/inc", {public=true})
            target:add("includedirs", "$(scriptdir)/firmware/n32h47x_48x_usbhs_driver/device/class/hid_msc_composite/inc", {public=true})
            target:add("includedirs", "$(scriptdir)/firmware/n32h47x_48x_usbhs_driver/device/class/mouse/inc", {public=true})
            target:add("includedirs", "$(scriptdir)/firmware/n32h47x_48x_usbhs_driver/device/class/msc/inc", {public=true})
            target:add("includedirs", "$(scriptdir)/firmware/n32h47x_48x_usbhs_driver/device/class/msc_cdc_composite/inc", {public=true})
            target:add("includedirs", "$(scriptdir)/firmware/n32h47x_48x_usbhs_driver/host/core/inc", {public=true})
            target:add("includedirs", "$(scriptdir)/firmware/n32h47x_48x_usbhs_driver/host/class/CDC/inc", {public=true})
            target:add("includedirs", "$(scriptdir)/firmware/n32h47x_48x_usbhs_driver/host/class/HID/inc", {public=true})
            target:add("includedirs", "$(scriptdir)/firmware/n32h47x_48x_usbhs_driver/host/class/MSC/inc", {public=true})
        end
    end)
    on_config(function(target)
        import("core.base.json")
        import("xhive.proc")
        import("xhive.base")
        -- Load the peripheral ISR list based on the selected series
        local prefix    = ""
        local scriptdir = path.absolute(os.scriptdir())
        local dirs      = base.load_paths()

        local conf = target:data("kconfig")
        if conf.NATION_N32H473 then 
            prefix = "n32h473"
        elseif conf.NATION_N32H474 then
            prefix = "n32h474"
        elseif conf.NATION_N32H475 then
            prefix = "n32h475"
        elseif conf.NATION_N32H481 then
            prefix = "n32h481"
        elseif conf.NATION_N32H482 then
            prefix = "n32h482"
        elseif conf.NATION_N32H487 then
            prefix = "n32h487"
        elseif conf.NATION_N32H488 then
            prefix = "n32h488"
        else
            raise("No valid N32H47X series selected in kconfig for target: " .. target:name())
        end
        local list_path = path.join(scriptdir, "periph_isr", prefix .. "_periph_isr.json")
        local isr_list = json.loadfile(list_path)
        
        -- Build the startup file with the peripheral ISR list

        if conf.USE_DEFAULT_STARTUP then
            local template_path = proc.load_startup_template_path()
            local output_path   = path.join(dirs.builddir, prefix .. "_startup.c")
            proc.build_arm_startup(template_path, isr_list, output_path)
            target:add("files", output_path)
        end
    end)
target_end()
