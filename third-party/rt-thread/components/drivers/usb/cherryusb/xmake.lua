target("cherryusb")
    set_kind("object")
    on_load(function(target)
        local sdir = os.scriptdir()
        local conf = target:data("kconfig")
        local srcs = {}
        local incs = {}

        if not conf.RT_USING_CHERRYUSB then
            return
        end

        local apply_maps = function(maps)
            for k, v in pairs(maps) do
                if conf[k] then
                    if v.incs then
                        if type(v.incs) == "table" then
                            for _, inc in ipairs(v.incs) do
                                table.insert(incs, path.join(sdir, inc))
                            end
                        else
                            table.insert(incs, path.join(sdir, v.incs))
                        end
                    end
                    if type(v.srcs) == "table" then
                        for _, src in ipairs(v.srcs) do
                            if v.dir then
                                table.insert(srcs, path.join(sdir, v.dir, src))
                            else
                                table.insert(srcs, path.join(sdir, src))
                            end
                        end
                    else
                        if v.dir then
                            table.insert(srcs, path.join(sdir, v.dir, v.srcs))
                        else
                            table.insert(srcs, path.join(sdir, v.srcs))
                        end
                    end
                end
            end
        end

        -- Device
        if conf.RT_CHERRYUSB_DEVICE then
            table.insert(incs, path.join(sdir, "osal"))
            table.insert(srcs, path.join(sdir, "core", "usbd_core.c"))
            table.insert(srcs, path.join(sdir, "osal", "usb_osal_rtthread.c"))
            local device_maps = {
                RT_CHERRYUSB_DEVICE_FSDEV_ST                      = {dir = "port/fsdev", srcs = {"usb_dc_fsdev.c", "usb_glue_st.c"}},
                RT_CHERRYUSB_DEVICE_FSDEV_CUSTOM                  = {dir = "port/fsdev", srcs = "usb_dc_fsdev.c"},
                RT_CHERRYUSB_DEVICE_DWC2_ST                       = {dir = "port/dwc2", srcs = {"usb_dc_dwc2.c", "usb_glue_st.c"}},
                RT_CHERRYUSB_DEVICE_DWC2_ESP                      = {dir = "port/dwc2", srcs = {"usb_dc_dwc2.c", "usb_glue_esp.c"}},
                RT_CHERRYUSB_DEVICE_DWC2_KENDRYTE                 = {dir = "port/dwc2", srcs = {"usb_dc_dwc2.c", "usb_glue_kendryte.c"}},
                RT_CHERRYUSB_DEVICE_DWC2_AT                       = {dir = "port/dwc2", srcs = {"usb_dc_dwc2.c", "usb_glue_at.c"}},
                RT_CHERRYUSB_DEVICE_DWC2_HC                       = {dir = "port/dwc2", srcs = {"usb_dc_dwc2.c", "usb_glue_hc.c"}},
                RT_CHERRYUSB_DEVICE_DWC2_NATION                   = {dir = "port/dwc2", srcs = {"usb_dc_dwc2.c", "usb_glue_nation.c"}},
                RT_CHERRYUSB_DEVICE_DWC2_GD                       = {dir = "port/dwc2", srcs = {"usb_dc_dwc2.c", "usb_glue_gd32.c"}},
                RT_CHERRYUSB_DEVICE_DWC2_CUSTOM                   = {dir = "port/dwc2", srcs = "usb_dc_dwc2.c"},
                RT_CHERRYUSB_DEVICE_MUSB_ES                       = {dir = "port/musb", srcs = {"usb_dc_musb.c", "usb_glue_es.c"}},
                RT_CHERRYUSB_DEVICE_MUSB_SUNXI                    = {dir = "port/musb", srcs = {"usb_dc_musb.c", "usb_glue_sunxi.c"}},
                RT_CHERRYUSB_DEVICE_MUSB_BK                       = {dir = "port/musb", srcs = {"usb_dc_musb.c", "usb_glue_bk.c"}},
                RT_CHERRYUSB_DEVICE_MUSB_SIFLI                    = {dir = "port/musb", srcs = {"usb_dc_musb.c", "usb_glue_sifli.c"}},
                RT_CHERRYUSB_DEVICE_MUSB_CUSTOM                   = {dir = "port/musb", srcs = "usb_dc_musb.c"},
                RT_CHERRYUSB_DEVICE_CHIPIDEA_MCX                  = {dir = "port/chipidea", incs = "port/chipidea", srcs = {"usb_dc_chipidea.c", "usb_glue_mcx.c"}},
                RT_CHERRYUSB_DEVICE_CHIPIDEA_CUSTOM               = {dir = "port/chipidea", incs = "port/chipidea", srcs = "usb_dc_chipidea.c"},
                RT_CHERRYUSB_DEVICE_KINETIS_MCX                   = {dir = "port/kinetis", srcs = {"usb_dc_kinetis.c", "usb_glue_mcx.c"}},
                RT_CHERRYUSB_DEVICE_KINETIS_CUSTOM                = {dir = "port/kinetis", srcs = "usb_dc_kinetis.c"},
                RT_CHERRYUSB_DEVICE_BL                            = {dir = "port/bouffalolab", srcs = "usb_dc_bl.c"},
                RT_CHERRYUSB_DEVICE_HPM                           = {dir = "port/hpmicro", incs = "port/hpmicro", srcs = {"usb_dc_hpm.c", "usb_glue_hpm.c"}},
                RT_CHERRYUSB_DEVICE_AIC                           = {dir = "port/aic", srcs = {"usb_dc_aic.c", "usb_dc_aic_ll.c"}},
                RT_CHERRYUSB_DEVICE_NRF5X                         = {dir = "port/nrf5x", srcs = "usb_dc_nrf5x.c"},
                RT_CHERRYUSB_DEVICE_CDC_ACM                       = {dir = "class/cdc_acm", srcs = "usbd_cdc_acm.c"},
                RT_CHERRYUSB_DEVICE_HID                           = {dir = "class/hid", srcs = "usbd_hid.c"},
                RT_CHERRYUSB_DEVICE_MSC                           = {dir = "class/msc", srcs = "usbd_msc.c"},
                RT_CHERRYUSB_DEVICE_AUDIO                         = {dir = "class/audio", srcs = "usbd_audio.c"},
                RT_CHERRYUSB_DEVICE_VIDEO                         = {dir = "class/video", srcs = "usbd_video.c"},
                RT_CHERRYUSB_DEVICE_CDC_RNDIS                     = {dir = "class/wireless", srcs = "usbd_rndis.c"},
                RT_CHERRYUSB_DEVICE_CDC_ECM                       = {dir = "class/cdc", srcs = "usbd_cdc_ecm.c"},
                RT_CHERRYUSB_DEVICE_CDC_NCM                       = {dir = "class/cdc", srcs = "usbd_cdc_ncm.c"},
                RT_CHERRYUSB_DEVICE_DFU                           = {dir = "class/dfu", srcs = "usbd_dfu.c"},
                RT_CHERRYUSB_DEVICE_CDC_ACM_CHARDEV               = {dir = "platform/rtthread", srcs = "usbd_serial.c"},
                RT_CHERRYUSB_DEVICE_TEMPLATE_CDC_ACM              = {dir = "demo", srcs = "cdc_acm_template.c"},
                RT_CHERRYUSB_DEVICE_TEMPLATE_HID_MOUSE            = {dir = "demo", srcs = "hid_mouse_template.c"},
                RT_CHERRYUSB_DEVICE_TEMPLATE_HID_KEYBOARD         = {dir = "demo", srcs = "hid_keyboard_template.c"},
                RT_CHERRYUSB_DEVICE_TEMPLATE_HID_CUSTOM           = {dir = "demo", srcs = "hid_custom_inout_template.c"},
                RT_CHERRYUSB_DEVICE_TEMPLATE_VIDEO                = {dir = "demo", srcs = "video_static_mjpeg_template.c"},
                RT_CHERRYUSB_DEVICE_TEMPLATE_AUDIO_V1_MIC_SPEAKER = {dir = "demo", srcs = "audio_v1_mic_speaker_multichan_template.c"},
                RT_CHERRYUSB_DEVICE_TEMPLATE_AUDIO_V2_MIC_SPEAKER = {dir = "demo", srcs = "audio_v2_mic_speaker_multichan_template.c"},
                RT_CHERRYUSB_DEVICE_TEMPLATE_CDC_RNDIS            = {dir = "demo", srcs = "cdc_rndis_template.c"},
                RT_CHERRYUSB_DEVICE_TEMPLATE_CDC_ECM              = {dir = "demo", srcs = "cdc_ecm_template.c"},
                RT_CHERRYUSB_DEVICE_TEMPLATE_CDC_NCM              = {dir = "demo", srcs = "cdc_ncm_template.c"},
                RT_CHERRYUSB_DEVICE_TEMPLATE_CDC_ACM_MSC          = {dir = "demo", srcs = "cdc_acm_msc_template.c"},
                RT_CHERRYUSB_DEVICE_TEMPLATE_CDC_ACM_MSC_HID      = {dir = "demo", srcs = "cdc_acm_hid_msc_template.c"},
                RT_CHERRYUSB_DEVICE_TEMPLATE_WINUSBV1             = {dir = "demo", srcs = "winusb1.0_template.c"},
                RT_CHERRYUSB_DEVICE_TEMPLATE_WINUSBV2_CDC         = {dir = "demo", srcs = "winusb2.0_cdc_template.c"},
                RT_CHERRYUSB_DEVICE_TEMPLATE_WINUSBV2_HID         = {dir = "demo", srcs = "winusb2.0_hid_template.c"},
                RT_CHERRYUSB_DEVICE_TEMPLATE_ADB                  = {dir = "demo/adb", srcs = "usbd_adb_template.c"},
                RT_CHERRYUSB_DEVICE_TEMPLATE_CDC_ACM_CHARDEV      = {dir = "demo", srcs = "cdc_acm_rttchardev_template.c"},
            }
            apply_maps(device_maps)

            if conf.RT_CHERRYUSB_DEVICE_ADB then
                table.insert(srcs, path.join(sdir, "class", "adb", "usbd_adb.c"))
                table.insert(srcs, path.join(sdir, "platform", "rtthread", "usbd_adb_shell.c"))
            end

            if conf.RT_CHERRYUSB_DEVICE_SPEED_HS then
                target:add("defines", "CONFIG_USB_HS")
            end

            if conf.RT_CHERRYUSB_DEVICE_CH32 then
                if conf.RT_CHERRYUSB_DEVICE_HS then
                    table.insert(srcs, path.join(sdir, "port", "ch32", "usb_dc_usbhs.c"))
                else
                    table.insert(srcs, path.join(sdir, "port", "ch32", "usb_dc_usbfs.c"))
                end
            end
            if conf.RT_CHERRYUSB_DEVICE_PUSB2 then
                table.insert(incs, path.join(sdir, "port", "pusb2", "rt-thread"))
                table.insert(srcs, path.join(sdir, "port", "pusb2", "rt-thread", "usb_dc_glue_phytium.c"))
                if conf.ARCH_ARMV8 then
                    table.insert(incs, path.join(sdir, "port", "pusb2"))
                    target:add("links", path.join(sdir, "port", "pusb2", "libpusb2_dc_a64.a"))
                end
                if conf.ARCH_ARM_CORTEX_A then
                    table.insert(incs, path.join(sdir, "port", "pusb2"))
                    target:add("links", path.join(sdir, "port", "pusb2", "libpusb2_dc_a32_softfp_neon.a"))
                end
            end
            if conf.RT_CHERRYUSB_DEVICE_TEMPLATE_MSC or conf.RT_CHERRYUSB_DEVICE_TEMPLATE_MSC_BLKDEV then
                table.insert(srcs, path.join(sdir, "demo", "msc_ram_template.c"))
            end
        end

        -- Host
        if conf.RT_CHERRYUSB_HOST then
            table.insert(srcs, path.join(sdir, "core", "usbh_core.c"))
            table.insert(srcs, path.join(sdir, "class", "hub", "usbh_hub.c"))
            table.insert(srcs, path.join(sdir, "osal", "usb_osal_rtthread.c"))

            local host_maps = {
                RT_CHERRYUSB_HOST_EHCI_BL = {dir = "port/ehci", srcs = {"usb_hc_ehci.c", "usb_glue_bouffalo.c"}},
                RT_CHERRYUSB_HOST_EHCI_HPM = {incs = "port/hpmicro", srcs = {
                    "port/ehci/usb_hc_ehci.c", "port/hpmicro/usb_hc_ehci_hpm.c", "port/hpmicro/usb_glue_hpm.c"
                }},
                RT_CHERRYUSB_HOST_EHCI_AIC = {incs = {"port/ehci", "port/ohci"}, srcs = {
                    "port/ehci/usb_hc_ehci.c", "port/aic/usb_glue_aic.c", "port/ohci/usb_hc_ohci.c"
                }},
                RT_CHERRYUSB_HOST_EHCI_MCX = {incs = "port/chipidea", srcs = {
                    "port/ehci/usb_hc_ehci.c", "port/nxp/usb_glue_mcx.c"
                }},
                RT_CHERRYUSB_HOST_EHCI_NUC980    = {dir = "port/ehci", srcs = {"usb_hc_ehci.c", "usb_glue_nuc980.c"}},
                RT_CHERRYUSB_HOST_EHCI_MA35D0    = {dir = "port/ehci", srcs = {"usb_hc_ehci.c", "usb_glue_ma35d0.c"}},
                RT_CHERRYUSB_HOST_EHCI_CUSTOM    = {dir = "port/ehci", srcs = "usb_hc_ehci.c"},
                RT_CHERRYUSB_HOST_DWC2_ST        = {dir = "port/dwc2", srcs = {"usb_hc_dwc2.c", "usb_glue_st.c"}},
                RT_CHERRYUSB_HOST_DWC2_ESP       = {dir = "port/dwc2", srcs = {"usb_hc_dwc2.c", "usb_glue_esp.c"}},
                RT_CHERRYUSB_HOST_DWC2_KENDRYTE  = {dir = "port/dwc2", srcs = {"usb_hc_dwc2.c", "usb_glue_kendryte.c"}},
                RT_CHERRYUSB_HOST_DWC2_HC        = {dir = "port/dwc2", srcs = {"usb_hc_dwc2.c", "usb_glue_hc.c"}},
                RT_CHERRYUSB_HOST_DWC2_NATION    = {dir = "port/dwc2", srcs = {"usb_hc_dwc2.c", "usb_glue_nation.c"}},
                RT_CHERRYUSB_HOST_DWC2_CUSTOM    = {dir = "port/dwc2", srcs = "usb_hc_dwc2.c"},
                RT_CHERRYUSB_HOST_MUSB_STANDARD  = {dir = "port/musb", srcs = "usb_hc_musb.c"},
                RT_CHERRYUSB_HOST_MUSB_ES        = {dir = "port/musb", srcs = {"usb_hc_musb.c", "usb_glue_es.c"}},
                RT_CHERRYUSB_HOST_MUSB_SUNXI     = {dir = "port/musb", srcs = {"usb_hc_musb.c", "usb_glue_sunxi.c"}},
                RT_CHERRYUSB_HOST_MUSB_BK        = {dir = "port/musb", srcs = {"usb_hc_musb.c", "usb_glue_bk.c"}},
                RT_CHERRYUSB_HOST_MUSB_SIFLI     = {dir = "port/musb", srcs = {"usb_hc_musb.c", "usb_glue_sifli.c"}},
                RT_CHERRYUSB_HOST_MUSB_CUSTOM    = {dir = "port/musb", srcs = "usb_hc_musb.c"},
                RT_CHERRYUSB_HOST_KINETIS_MCX    = {dir = "port/kinetis", srcs = {"usb_hc_kinetis.c", "usb_glue_mcx.c"}},
                RT_CHERRYUSB_HOST_KINETIS_CUSTOM = {dir = "port/kinetis", srcs = "usb_hc_kinetis.c"},
                RT_CHERRYUSB_HOST_CDC_ACM        = {dir = "class/cdc", srcs = "usbh_cdc_acm.c"},
                RT_CHERRYUSB_HOST_HID            = {dir = "class/hid", srcs = "usbh_hid.c"},
                RT_CHERRYUSB_HOST_MSC            = {dir = "class/msc", srcs = "usbh_msc.c"},
                RT_CHERRYUSB_HOST_CDC_RNDIS      = {dir = "class/wireless", srcs = "usbh_rndis.c"},
                RT_CHERRYUSB_HOST_CDC_ECM        = {dir = "class/cdc", srcs = "usbh_cdc_ecm.c"},
                RT_CHERRYUSB_HOST_CDC_NCM        = {dir = "class/cdc", srcs = "usbh_cdc_ncm.c"},
                RT_CHERRYUSB_HOST_VIDEO          = {dir = "class/video", srcs = "usbh_video.c"},
                RT_CHERRYUSB_HOST_AUDIO          = {dir = "class/audio", srcs = "usbh_audio.c"},
                RT_CHERRYUSB_HOST_BLUETOOTH      = {dir = "class/wireless", srcs = "usbh_bluetooth.c"},
                RT_CHERRYUSB_HOST_ASIX           = {dir = "class/vendor/net", srcs = "usbh_asix.c"},
                RT_CHERRYUSB_HOST_RTL8152        = {dir = "class/vendor/net", srcs = "usbh_rtl8152.c"},
                RT_CHERRYUSB_HOST_FTDI           = {dir = "class/vendor/serial", srcs = "usbh_ftdi.c"},
                RT_CHERRYUSB_HOST_CH34X          = {dir = "class/vendor/serial", srcs = "usbh_ch34x.c"},
                RT_CHERRYUSB_HOST_CP210X         = {dir = "class/vendor/serial", srcs = "usbh_cp210x.c"},
                RT_CHERRYUSB_HOST_PL2303         = {dir = "class/vendor/serial", srcs = "usbh_pl2303.c"},
                CONFIG_TEST_USBH_HID             = {dir = "demo", srcs = "usbh_hid_test.c"},
            }

            apply_maps(host_maps)

            local usbh_lwip_deps = {
                "RT_CHERRYUSB_HOST_CDC_ECM", "RT_CHERRYUSB_HOST_CDC_RNDIS", "RT_CHERRYUSB_HOST_CDC_NCM",
                "RT_CHERRYUSB_HOST_ASIX", "RT_CHERRYUSB_HOST_RTL8152"
            }
            for _, dep in ipairs(usbh_lwip_deps) do
                if conf[dep] then
                    table.insert(srcs, path.join(sdir, "platform", "rtthread", "usbh_lwip.c"))
                    break
                end
            end

            -- FIXME: and usbh_serial what a dependency ?
            if conf.RT_CHERRYUSB_HOST_MSC then
                table.insert(srcs, path.join(sdir, "platform", "rtthread", "usbh_fs.c"))
            end

            local usbh_serial_deps = {
                "RT_CHERRYUSB_HOST_CDC_ACM", "RT_CHERRYUSB_HOST_FTDI", "RT_CHERRYUSB_HOST_CH34X",
                "RT_CHERRYUSB_HOST_CP210X", "RT_CHERRYUSB_HOST_PL2303"
            }
            for _, dep in ipairs(usbh_serial_deps) do
                if conf[dep] then
                    table.insert(srcs, path.join(sdir, "platform", "rtthread", "usbh_serial.c"))
                    break
                end
            end

            if conf.RT_CHERRYUSB_HOST_PUSB2 then
                table.insert(incs, path.join(sdir, "port", "pusb2", "rt-thread"))
                table.insert(srcs, path.join(sdir, "port", "pusb2", "rt-thread", "usb_hc_glue_phytium.c"))
                if conf.ARCH_ARMV8 then
                    table.insert(incs, path.join(sdir, "port", "pusb2"))
                    target:add("links", path.join(sdir, "port", "pusb2", "libpusb2_hc_a64.a"))
                end
                if conf.ARCH_ARM_CORTEX_A then
                    table.insert(incs, path.join(sdir, "port", "pusb2"))
                    target:add("links", path.join(sdir, "port", "pusb2", "libpusb2_hc_a32_softfp_neon.a"))
                end
            end

            if conf.RT_CHERRYUSB_HOST_XHCI then
                local dir = path.join(sdir, "port", "xhci", "phytium")
                table.insert(incs, path.join(dir, "rt-thread"))
                table.insert(srcs, path.join(dir, "usb_glue_phytium_plat.c"))
                table.insert(srcs, path.join(dir, "usb_glue_phytium.c"))
                if conf.ARCH_ARMV8 then
                    target:add("links", path.join(dir, "libxhci_a64.a"))
                end
                if conf.ARCH_ARM_CORTEX_A then
                    target:add("links", path.join(dir, "libxhci_a32_softfp_neon.a"))
                end
            end
        end

        table.insert(srcs, path.join(sdir, "platform", "rtthread", "usb_msh.c"))
        table.insert(srcs, path.join(sdir, "platform", "rtthread", "usb_check.c"))

        target:add("files", srcs)
        target:add("includedirs", incs, {public = true})
    end)
