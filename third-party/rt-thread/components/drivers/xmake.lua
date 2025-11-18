includes("usb")
target("drivers")
    set_kind("object")
    set_default(false)
    add_deps("finsh", {public=true})
    add_deps("cherryusb", {public=true})
    add_includedirs("include", {public=true})
    on_load(function(target)
        local conf = target:data("kconfig")
        local sdir = os.scriptdir()
        local srcs = {}
        local incs = {}
        -- ata
        if conf.RT_USING_ATA then
            local dir = path.join(sdir, "ata")
            table.insert(incs, dir)
            if conf.RT_ATA_AHCI then
                table.insert(srcs, path.join(dir, "ahci.c"))
                if conf.RT_ATA_AHCI_PCI then
                    table.insert(srcs, path.join(dir, "ahci-pci.c"))
                end
            end
        end
        -- audio
        if conf.RT_USING_AUDIO then
            local dir = path.join(sdir, "audio")
            table.insert(incs, dir)
            table.insert(srcs, path.join(dir, "*.c"))
            if conf.RT_USING_UTESTCASES and conf.RT_UTEST_USING_AUDIO_DRIVER then
                table.insert(srcs, path.join(dir, "utest", "tc_*.c"))
            end
        end
        -- block
        if conf.RT_USING_BLK then
            local dir = path.join(sdir, "block")
            table.insert(incs, dir)
            table.insert(srcs, path.join(dir, "*.c"))
            if conf.RT_BLK_PARTITION_DFS then
                table.insert(srcs, path.join(dir, "partitions", "dfs.c"))
            end

            if conf.RT_BLK_PARTITION_EFI then
                table.insert(srcs, path.join(dir, "partitions", "efi.c"))
            end
        end
        -- can
        if conf.RT_USING_CAN then
            local dir = path.join(sdir, "can")
            table.insert(incs, dir)
            table.insert(srcs, path.join(dir, "*.c"))
        end
        -- clk
        if conf.RT_USING_CLK then
            local dir = path.join(sdir, "clk")
            table.insert(incs, dir)
            table.insert(srcs, path.join(dir, "clk.c"))
            if conf.RT_USING_OFW then
                table.insert(srcs, path.join(dir, "clk-fixed-rate.c"))
            end
        end
        -- core
        if conf.RT_USING_DEVICE then
            local dir = path.join(sdir, "core")
            table.insert(incs, dir)
            table.insert(srcs, path.join(dir, "device.c"))
            if conf.RT_USING_DEV_BUS or conf.RT_USING_DM then
                table.insert(srcs, path.join(dir, "bus.c"))
            end
            if conf.RT_USING_DM then
                for _, f in ipairs({"dm.c", "driver.c", "numa.c", "platform.c", "power_domain.c"}) do
                    table.insert(srcs, path.join(dir, f))
                end
                if conf.RT_USING_DFS then
                    table.insert(srcs, path.join(dir, "mnt.c"))
                end
            end
            if conf.RT_USING_OFW then
                table.insert(srcs, path.join(dir, "platform_ofw.c"))
            end
            if conf.RT_USING_UTESTCASES and conf.RT_UTEST_DRIVERS_CORE then
                table.insert(srcs, path.join(dir, "utest", "*.c"))
            end
        end
        -- cputime
        if conf.RT_USING_CPUTIME then
            local dir = path.join(sdir, "cputime")
            table.insert(incs, dir)
            for _, f in ipairs({"cputime.c", "cputimer.c"}) do
                table.insert(srcs, path.join(dir, f))
            end
            if conf.RT_USING_CPUTIME_CORTEXM then
                table.insert(srcs, path.join(dir, "cputime_cortexm.c"))
            end
            if conf.RT_USING_CPUTIME_RISCV then
                table.insert(srcs, path.join(dir, "cputime_riscv.c"))
            end
        end
        -- dma
        if conf.RT_USING_DMA then
            local dir = path.join(sdir, "dma")
            table.insert(incs, dir)
            table.insert(srcs, path.join(dir, "*.c"))
        end
        -- hwcyrpto
        if conf.RT_USING_HWCRYPTO then
            local dir = path.join(sdir, "hwcrypto")
            table.insert(incs, dir)
            table.insert(srcs, path.join(dir, "hwcrypto.c"))
            if conf.RT_HWCRYPTO_USING_AES or conf.RT_HWCRYPTO_USING_DES or conf.RT_HWCRYPTO_USING_3DES or conf.RT_HWCRYPTO_USING_RC4 then
                table.insert(srcs, path.join(dir, "hw_symmetric.c"))
                if conf.RT_HWCRYPTO_USING_GCM then
                    table.insert(srcs, path.join(dir, "hw_gcm.c"))
                end
            end
            if conf.RT_HWCRYPTO_USING_MD5 or conf.RT_HWCRYPTO_USING_SHA1 or conf.RT_HWCRYPTO_USING_SHA2 then
                table.insert(srcs, path.join(dir, "hw_hash.c"))
            end
            if conf.RT_HWCRYPTO_USING_RNG then
                table.insert(srcs, path.join(dir, "hw_rng.c"))
            end
            if conf.RT_HWCRYPTO_USING_CRC then
                table.insert(srcs, path.join(dir, "hw_crc.c"))
            end
            if conf.RT_HWCRYPTO_USING_BIGNUM then
                table.insert(srcs, path.join(dir, "hw_bignum.c"))
            end
        end
        -- hwtimer
        if conf.RT_USING_HWTIMER then
            local dir = path.join(sdir, "hwtimer")
            table.insert(incs, dir)
            table.insert(srcs, path.join(dir, "hwtimer.c"))
            if conf.RT_HWTIMER_ARM_ARCH then
                table.insert(srcs, path.join(dir, "hwtimer-arm_arch.c"))
            end
        end
        -- i2c
        if conf.RT_USING_I2C then
            local dir = path.join(sdir, "i2c")
            table.insert(incs, dir)
            table.insert(srcs, path.join(dir, "dev_i2c_core.c"))
            table.insert(srcs, path.join(dir, "dev_i2c_dev.c"))
            if conf.RT_USING_I2C_BITOPS then
                table.insert(srcs, path.join(dir, "dev_i2c_bit_ops.c"))
            end
            if conf.RT_USING_SOFT_I2C then
                table.insert(srcs, path.join(dir, "dev_soft_i2c.c"))
            end
            if conf.RT_USING_DM then
                table.insert(srcs, path.join(dir, "dev_i2c_bus.c"))
                table.insert(srcs, path.join(dir, "dev_i2c_dm.c"))
            end
        end
        -- iio
        if conf.RT_USING_DM then
            local dir = path.join(sdir, "iio")
            table.insert(incs, dir)
            table.insert(srcs, path.join(dir, "*.c"))
        end
        -- ipc
        if conf.RT_USING_DEVICE_IPC then
            local dir = path.join(sdir, "ipc")
            local ipc_srcs = os.files(path.join(dir, "*.c"))
            table.insert(incs, dir)
            -- remove dataqueue.c and pipe.c when heap is disabled
            for i = #ipc_srcs, 1, -1 do
                local f = ipc_srcs[i]
                if (not conf.RT_USING_HEAP) and (f:endswith("dataqueue.c") or f:endswith("pipe.c")) then
                    table.remove(ipc_srcs, i)
                end

                if not conf.RT_USING_SMP then
                    if f:endswith("completion_mp.c") then
                        table.remove(ipc_srcs, i)
                    end
                else
                    if f:endswith("completion_up.c") then
                        table.remove(ipc_srcs, i)
                    end
                end
            end
            target:add("files", ipc_srcs, {defines="__RT_IPC_SOURCE__"})
            if conf.RT_USING_UTESTCASES then
                if conf.RT_UTEST_COMPLETION then
                    for _, f in ipairs({"completion_tc.c", "completion_timeout_tc.c"}) do
                        table.insert(srcs, path.join(dir, "utest", f))
                    end
                end
                if conf.RT_UTEST_WORKQUEUE then
                    table.insert(srcs, path.join(dir, "utest", "workqueue_tc.c"))
                end
            end
        end
        -- ktime
        if conf.RT_USING_KTIME then
            local dir = path.join(sdir, "ktime")
            table.insert(incs, dir)
            table.insert(srcs, path.join(dir, "src", "*.c"))
            if conf.CPU_ARCH_ARM and conf.ARCH_ARM_AARCH64 then
                table.insert(srcs, path.join(dir, "src", "aarch64", "cputimer.c"))
            elseif conf.CPU_ARCH_RISCV and conf.ARCH_RISCV_RV64 then
                table.insert(srcs, path.join(dir, "src", "risc-v", "virt64", "cputimer.c"))
            end
        end
        -- led
        if conf.RT_USING_DM and conf.RT_USING_LED then
            local dir = path.join(sdir, "led")
            table.insert(incs, dir)
            table.insert(srcs, path.join(dir, "led.c"))
            if conf.RT_LED_GPIO then
                table.insert(srcs, path.join(dir, "led-gpio.c"))
            end
        end
        -- mailbox
        if conf.RT_USING_MBOX then
            local dir = path.join(sdir, "mailbox")
            table.insert(incs, dir)
            table.insert(srcs, path.join(dir, "mailbox.c"))
            if conf.RT_MBOX_PIC then
                table.insert(srcs, path.join(dir, "mailbox-pic.c"))
            end
        end
        -- mfd
        if conf.RT_USING_MFD then
            local dir = path.join(sdir, "mfd")
            table.insert(incs, dir)
            -- table.insert(srcs, path.join(dir, "*.c"))
            if conf.RT_MFD_SYSCON then
                table.insert(srcs, path.join(dir, "mfd-syscon.c"))
            end
        end
        -- misc
        local misc_dir = path.join(sdir, "misc")
        table.insert(incs, misc_dir)
        if conf.RT_USING_ADC then
            table.insert(srcs, path.join(misc_dir, "adc.c"))
        end
        if conf.RT_USING_DAC then
            table.insert(srcs, path.join(misc_dir, "dac.c"))
        end
        if conf.RT_USING_PWM then
            table.insert(srcs, path.join(misc_dir, "pwm.c"))
        end
        if conf.RT_USING_PULSE_ENCODER then
            table.insert(srcs, path.join(misc_dir, "pulse_encoder.c"))
        end
        if conf.RT_USING_INPUT_CAPTURE then
            table.insert(srcs, path.join(misc_dir, "rt_inputcapture.c"))
        end
        if conf.RT_USING_NULL then
            table.insert(srcs, path.join(misc_dir, "rt_null.c"))
        end
        if conf.RT_USING_ZERO then
            table.insert(srcs, path.join(misc_dir, "rt_zero.c"))
        end
        if conf.RT_USING_RANDOM then
            table.insert(srcs, path.join(misc_dir, "rt_random.c"))
        end
        -- mtd
        local mtd_dir = path.join(sdir, "mtd")
        table.insert(incs, mtd_dir)
        if conf.RT_USING_MTD_NOR then
            table.insert(srcs, path.join(mtd_dir, "mtd_nor.c"))
        end
        if conf.RT_USING_MTD_NAND then
            table.insert(srcs, path.join(mtd_dir, "mtd_nand.c"))
        end
        -- nvme
        if conf.RT_USING_NVME then
            local dir = path.join(sdir, "nvme")
            table.insert(incs, dir)
            table.insert(srcs, path.join(dir, "nvme.c"))
            if conf.RT_NVME_PCI then
                table.insert(srcs, path.join(dir, "nvme-pci.c"))
            end
        end
        -- ofw
        if conf.RT_USING_OFW then
            local dir = path.join(sdir, "ofw")
            table.insert(incs, dir)
            local ofw_srcs = os.files(path.join(dir, "*.c"))
            -- remove irq.c if RT_USING_PIC not enabled
            for i = #ofw_srcs, 1, -1 do
                local f = ofw_srcs[i]
                if (not conf.RT_USING_PIC) and f:endswith("irq.c") then
                    table.remove(ofw_srcs, i)
                end
            end
            local fdt_dir = path.join(dir, "libfdt")
            table.insert(incs, fdt_dir)
            table.insert(srcs, path.join(fdt_dir, "*.c"))
        end
        -- pci
        if conf.RT_USING_PCI then
            local dir = path.join(sdir, "pci")
            table.insert(incs, dir)
            for _, f in ipairs({"access.c", "host-bridge.c", "irq.c", "pci.c", "pme.c", "probe.c"}) do
                table.insert(srcs, path.join(dir, f))
            end
            if conf.RT_USING_OFW then
                table.insert(srcs, path.join(dir, "ofw.c"))
            end
            if conf.RT_USING_DFS_PROCFS then
                table.insert(srcs, path.join(dir, "procfs"))
            end
            if conf.RT_PCI_ECAM then
                table.insert(srcs, path.join(dir, "ecam.c"))
            end
            -- endpoint
            if conf.RT_PCI_ENDPOINT then
                table.insert(srcs, path.join(dir, "endpoint", "endpoint.c"))
                table.insert(srcs, path.join(dir, "endpoint", "mem.c"))
            end
            -- host
            if conf.RT_PCI_HOST_COMMON then
                table.insert(srcs, path.join(dir, "host", "pci-host-common.c"))
            end
            if conf.RT_PCI_HOST_GENERIC then
                table.insert(srcs, path.join(dir, "host", "pci-host-generic.c"))
            end
            if conf.RT_PCI_DW then
                table.insert(srcs, path.join(dir, "host", "dw", "pci-dw.c"))
                table.insert(srcs, path.join(dir, "host", "dw", "pci-dw_platfrom.c")) -- FIXME: name wrong?? platform?
                if conf.RT_PCI_DW_HOST then
                    table.insert(srcs, path.join(dir, "host", "dw", "pcie-dw_host.c"))
                end
                if conf.RT_PCI_DW_EP then
                    table.insert(srcs, path.join(dir, "host", "dw", "pcie-dw_ep.c"))
                end
            end
            -- msi
            if conf.RT_PCI_MSI then
                for _, f in ipairs({"device.c", "irq.c", "msi.c"}) do
                    table.insert(srcs, path.join(dir, "msi", f))
                end
            end
        end
        -- phy
        if conf then
            local dir = path.join(sdir, "phy")
            table.insert(incs, dir)
            local phy_srcs = os.files(path.join(dir, "*.c"))
            -- remove ofw.c if RT_USING_OFW not enabled
            -- remove general.c','mdio.c','ofw.c' if RT_USING_PHY_V2 not enabled
            -- remove phy.c if RT_USING_PHY_V2 and RT_USING_PHY not enabled in same time
            for i = #phy_srcs, 1, -1 do
                local f = phy_srcs[i]
                if (not conf.RT_USING_OFW) and f:endswith("ofw.c") then
                    table.remove(phy_srcs, i)
                elseif (not conf.RT_USING_PHY_V2) then
                    if f:endswith("general.c") or f:endswith("mdio.c") or f:endswith("ofw.c") then
                        table.remove(phy_srcs, i)
                    end
                elseif conf.RT_USING_PHY_V2 and (not conf.RT_USING_PHY) and f:endswith("phy.c") then
                    table.remove(phy_srcs, i)
                end
            end
        end
        -- phye
        if conf.RT_USING_PHYE then
            local dir = path.join(sdir, "phye")
            table.insert(incs, dir)
            table.insert(srcs, path.join(dir, "phye.c"))
        end
        -- pic
        if conf.RT_USING_PIC then
            local dir = path.join(sdir, "pic")
            table.insert(incs, dir)
            table.insert(srcs, path.join(dir, "pic.c"))
            table.insert(srcs, path.join(dir, "pic_rthw.c"))
            if conf.RT_PIC_ARM_GIC or conf.RT_PIC_ARM_GIC_V3 then
                table.insert(srcs, path.join(dir, "pic-gic-common.c"))
            end
            if conf.RT_PIC_ARM_GIC then
                table.insert(srcs, path.join(dir, "pic-gicv2.c"))
            end
            if conf.RT_PIC_ARM_GIC_V2M then
                table.insert(srcs, path.join(dir, "pic-gicv2m.c"))
            end
            if conf.RT_PIC_ARM_GIC_V3 then
                table.insert(srcs, path.join(dir, "pic-gicv3.c"))
            end
            if conf.RT_PIC_ARM_GIC_V3_ITS then
                table.insert(srcs, path.join(dir, "pic-gicv3-its.c"))
            end
        end
        -- pin
        if conf.RT_USING_PIN then
            local dir = path.join(sdir, "pin")
            table.insert(incs, dir)
            table.insert(srcs, path.join(dir, "dev_pin.c"))
            if conf.RT_USING_DM then
                table.insert(srcs, path.join(dir, "dev_pin_dm.c"))
            end
            if conf.RT_USING_OFW then
                table.insert(srcs, path.join(dir, "dev_pin_ofw.c"))
            end
        end
        -- pinctrl
        if conf.RT_USING_PINCTRL then
            local dir = path.join(sdir, "pinctrl")
            table.insert(incs, dir)
            table.insert(srcs, path.join(dir, "pinctrl.c"))
        end
        -- pm
        if conf then
            local dir = path.join(sdir, "pm")
            table.insert(incs, dir)
            if conf.RT_USING_PM then
                table.insert(srcs, path.join(dir, "pm.c"))
                table.insert(srcs, path.join(dir, "lptimer.c"))
            end
        end
        -- regulator
        if conf.RT_USING_REGULATOR then
            local dir = path.join(sdir, "regulator")
            table.insert(incs, dir)
            table.insert(srcs, path.join(dir, "regulator.c"))
            table.insert(srcs, path.join(dir, "regulator_dm.c"))
            if conf.RT_REGULATOR_FIXED then
                table.insert(srcs, path.join(dir, "regulator-fixed.c"))
            end
            if conf.RT_REGULATOR_GPIO then
                table.insert(srcs, path.join(dir, "regulator-gpio.c"))
            end
        end
        -- reset
        if conf.RT_USING_RESET then
            local dir = path.join(sdir, "reset")
            table.insert(incs, dir)
            table.insert(srcs, path.join(dir, "reset.c"))
            if conf.RT_RESET_SIMPLE then
                table.insert(srcs, path.join(dir, "reset-simple.c"))
            end
        end
        -- rtc
        if conf.RT_USING_RTC then
            local dir = path.join(sdir, "rtc")
            table.insert(incs, dir)
            if conf.RT_USING_RTC then
                table.insert(srcs, path.join(dir, "dev_rtc.c"))
                if conf.RT_USING_ALARM then
                    table.insert(srcs, path.join(dir, "dev_alarm.c"))
                end
                if conf.RT_USING_SOFT_RTC then
                    table.insert(srcs, path.join(dir, "dev_soft_rtc.c"))
                end
            end
        end
        -- scsi
        if conf.RT_USING_SCSI then
            local dir = path.join(sdir, "scsi")
            table.insert(incs, dir)
            table.insert(srcs, path.join(dir, "scsi.c"))
            if conf.RT_SCSI_SD then
                table.insert(srcs, path.join(dir, "scsi_sd.c"))
            end
            if conf.RT_SCSI_CDROM then
                table.insert(srcs, path.join(dir, "scsi_cdrom.c"))
            end
        end
        -- sdio
        if conf.RT_USING_SDIO then
            local dir = path.join(sdir, "sdio")
            table.insert(incs, dir)
            for _, f in ipairs({"dev_block.c", "dev_mmcsd_core.c", "dev_sd.c", "dev_sdio.c", "dev_mmc.c"}) do
                table.insert(srcs, path.join(dir, f))
            end
            if conf.RT_USING_SDHCI then
                for _, f in ipairs({"sdhci.c", "fit-mmc.c", "sdhci-platform.c"}) do
                    table.insert(srcs, path.join(dir, "sdhci", f))
                end
            end
        end
        -- sensor
        if conf.RT_USING_SENSOR then
            local dir = path.join(sdir, "sensor")
            table.insert(incs, dir)
            local ver = conf.RT_USING_SENSOR_V2 and "v2" or "v1"
            table.insert(srcs, path.join(dir, ver, "sensor.c"))
            if conf.RT_USING_SENSOR_CMD then
                table.insert(srcs, path.join(dir, ver, "sensor_cmd.c"))
            end
        end
        -- serial
        if conf.RT_USING_SERIAL then
            local dir = path.join(sdir, "serial")
            table.insert(incs, dir)
            if conf.RT_USING_SMART then
                table.insert(srcs, "serial_tty.c")
            end
            if conf.RT_USING_SERIAL_V2 then
                table.insert(srcs, path.join(dir, "dev_serial_v2.c"))
            else
                table.insert(srcs, path.join(dir, "dev_serial.c"))
            end
            if conf.RT_USING_SERIAL_BYPASS then
                table.insert(srcs, path.join(dir, "bypass.c"))
            end
            if conf.RT_USING_DM then
                table.insert(srcs, path.join(dir, "serial_dm.c"))
            end
            -- utest
            if conf.RT_USING_UTESTCASES then
                local utest_dir = path.join(dir, "utest")
                if conf.RT_UTEST_SERIAL_BYPASS then
                    table.insert(srcs, path.join(utest_dir, "bypass", "bypass_*.c"))
                end

                if conf.RT_UTEST_SERIAL_V2 then
                    table.insert(srcs, path.join(utest_dir, "v2", "*.c"))
                    if conf.RT_UTEST_SERIAL_POSIX then
                        table.insert(srcs, path.join(utest_dir, "v2", "posix", "*.c"))
                    end
                    if conf.RT_UTEST_SERIAL_QEMU then
                        table.insert(srcs, path.join(utest_dir, "v2", "qemu", "*.c"))
                    end
                end
            end
        end
        -- smp_call
        local smpcall_dir = path.join(sdir, "smp_call")
        table.insert(incs, smpcall_dir)
        if conf.RT_USING_SMP then
            table.insert(srcs, path.join(smpcall_dir, "*.c"))
        end
        if conf.RT_USING_UTESTCASES and conf.RT_UTEST_SMP_CALL_FUNC then
            table.insert(srcs, path.join(smpcall_dir, "utest", "smp*.c"))
        end
        -- spi
        if conf.RT_USING_SPI then
            local dir = path.join(sdir, "spi")
            table.insert(incs, dir)
            table.insert(srcs, path.join(dir, "dev_spi_core.c"))
            table.insert(srcs, path.join(dir, "dev_spi.c"))

            if conf.RT_USING_SOFT_SPI then
                table.insert(srcs, path.join(dir, "dev_spi_bit_ops.c"))
                table.insert(srcs, path.join(dir, "dev_soft_spi.c"))
            end

            if conf.RT_USING_QSPI then
                table.insert(srcs, path.join(dir, "dev_qspi_core.c"))
            end
            
            if conf.RT_USING_SPI_WIFI then
                table.insert(srcs, path.join(dir, "dev_spi_wifi_rw009.c"))
            end

            if conf.RT_USING_ENC28J60 then
                table.insert(srcs, path.join(dir, "enc28j60.c"))
            end

            if conf.RT_USING_SPI_MSD then
                table.insert(srcs, path.join(dir, "dev_spi_msd.c"))
            end

            if conf.RT_USING_SFUD then
                local sfud_dir = path.join(dir, "sfud")
                table.insert(incs, path.join(sfud_dir, "inc"))
                table.insert(srcs, path.join(dir, "dev_spi_flash_sfud.c"))
                table.insert(srcs, path.join(sfud_dir, "src", "sfud.c"))
                if conf.RT_SFUD_USING_SFDP then
                    table.insert(srcs, path.join(sfud_dir, "src", "sfud_sfdp.c"))
                end

                if conf.RT_USING_DM then
                    table.insert(srcs, path.join(dir, "dev_spi_dm.c"))
                    table.insert(srcs, path.join(dir, "dev_spi_bus.c"))
                end
            end
        end
        -- thermal
        if conf.RT_USING_THERMAL then
            local dir = path.join(sdir, "thermal")
            table.insert(incs, dir)
            table.insert(srcs, path.join(dir, "thermal.c"))
            table.insert(srcs, path.join(dir, "thermal_dm.c"))

            if conf.RT_THERMAL_COOL_PWM_FAN then
                table.insert(srcs, path.join(dir, "thermal-cool-pwm-fan.c"))
            end
        end
        -- touch
        if conf.RT_USING_TOUCH and conf.RT_USING_DEVICE then
            local dir = path.join(sdir, "touch")
            table.insert(incs, dir)
            table.insert(srcs, path.join(dir, "dev_touch.c"))
        end
        -- usb
        target:add("deps", "cherryusb", {public=true})
        -- virtio
        if conf.RT_USING_VIRTIO then
            local dir = path.join(sdir, "virtio")
            table.insert(incs, dir)
            table.insert(srcs, path.join(dir, "*.c"))  
        end
        -- watchdog
        if conf.RT_USING_WDT then
            local dir = path.join(sdir, "watchdog")
            table.insert(incs, dir)
            table.insert(srcs, path.join(dir, "dev_watchdog.c"))
            if conf.RT_WDT_DW then
                table.insert(srcs, path.join(dir, "watchdog-dw.c"))
            end
            if conf.RT_WDT_I6300ESB then
                table.insert(srcs, path.join(dir, "watchdog-i6300esb.c"))
            end
        end
        -- wlan
        if conf.RT_USING_WIFI then
            local dir = path.join(sdir, "wlan")
            table.insert(incs, dir)
            table.insert(srcs, path.join(dir, "dev_wlan.c"))
            if conf.RT_WLAN_MANAGE_ENABLE then
                table.insert(srcs, path.join(dir, "dev_wlan_mgnt.c"))
            end
            if conf.RT_WLAN_MSH_CMD_ENABLE then
                table.insert(srcs, path.join(dir, "dev_wlan_cmd.c"))
            end
            if conf.RT_WLAN_PROT_ENABLE then
                table.insert(srcs, path.join(dir, "dev_wlan_prot.c"))
            end
            if conf.RT_WLAN_PROT_LWIP_ENABLE then
                table.insert(srcs, path.join(dir, "dev_wlan_lwip.c"))
            end
            if conf.RT_WLAN_CFG_ENABLE then
                table.insert(srcs, path.join(dir, "dev_wlan_cfg.c"))
            end
            if conf.RT_WLAN_WORK_THREAD_ENABLE then
                table.insert(srcs, path.join(dir, "dev_wlan_workqueue.c"))
            end
        end

        target:add("files", srcs)
        target:add("includedirs", incs, {public=true})
    end)
