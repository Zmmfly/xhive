target("openecos_starrysky")
    set_kind("object")
    set_default(false)
    on_load(function(target)
        local conf = target:data("kconfig")
        -- Guard for OpenECOS target
        if not conf.VENDOR_USE_OPENECOS then
            return
        end
        if not conf.STARRYSKY_USE_C2 then
            return
        end

        local sdir     = os.scriptdir()
        local srcs     = {}
        local incs_pub = {}
        local mdir     = nil

        if conf.STARRYSKY_USE_C2 then
            mdir = path.join(sdir, "c2")
        end

        if mdir then
            table.insert(srcs, path.join(mdir, "src", "**.c"))
            table.insert(incs_pub, path.join(mdir, "inc"))
        end

        target:add("files", srcs)
        target:add("includedirs", incs_pub, {public = true})
    end)
target_end()
