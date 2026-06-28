includes("starrysky")

target("vendor_openecos")
    set_kind("object")
    set_default(false)
    on_load(function(target)
        local conf = target:data("kconfig")
        -- Guard for OpenECOS target
        if not conf.VENDOR_USE_OPENECOS then
            return
        end

        target:add("deps", "openecos_starrysky", {public=true})
    end)
target_end()