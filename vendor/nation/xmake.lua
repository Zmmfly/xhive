includes("n32h47x_48x")
target("vendor_nation")
    set_kind("object")
    set_default(false)
    on_load(function(target)
        local conf = target:data("kconfig")
        if conf.NATION_USE_N32H47X_48X then
            target:add("deps", "n32h47x_48x", {public=true})
        end
    end)
