includes("lwip-1.4.1")
includes("lwip-2.0.3")
includes("lwip-2.1.2")
includes("port")

target("lwip")
    add_deps("lwip-1.4.1", "lwip-2.0.3", "lwip-2.1.2", "lwip-port", {public=true})

    set_kind("object")
    on_load(function(target) 
        local conf = target:data("kconfig")
        local sdir = os.scriptdir()
    end)
