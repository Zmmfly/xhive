target("fal")
    set_kind("object")
    set_default(false)
    on_load(function(target)
        local conf = target:data("kconfig")
        local sdir = os.scriptdir()

        if not conf.RT_USING_FAL then
            return
        end
        target:add("files", os.files(path.join(sdir, "src", "*.c")))
        if conf.FAL_USING_SFUD_PORT then
            target:add("files", os.files(path.join(sdir, "samples", "porting", "fal_flash_sfud_port.c")))
        end
        target:add("includedirs", path.join(sdir, "inc"), {public = true})
    end)
