includes("nation")
includes("stmicro")

target("vendor")
    set_kind("object")
    on_load(function(target)
        local conf = target:data("kconfig")
        if conf.VENDOR_USE_NATION then
            target:add("deps", "vendor_nation", {public=true})
        elseif conf.VENDOR_USE_STMICRO then
            target:add("deps", "vendor_stmicro", {public=true})
        end
    end)
