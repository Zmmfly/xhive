includes("at")
includes("lwip")
includes("lwip-dhcpd")
includes("lwip-nat")
includes("netdev")
includes("sal")
includes("utest")

target("net")
    set_kind("object")
    add_deps(
        "at", "lwip", "lwip-dhcpd", "lwip-nat", "netdev", "sal", "net_utest",
        {public = true}
    )

    on_load(function(target)
        local conf = target:data("kconfig")
        local sdir = os.scriptdir()
    end)
