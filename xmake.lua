add_moduledirs("modules")

includes("compiler")
includes("rules")
includes("plugins")

-- Define xmcu_host namespace to include libraries for host builds to provide test isolation
namespace("xmcu_host", function()
    includes("libs")
end)

-- Define xmcu_embed namespace to include libraries for embedded builds
namespace("xmcu_embed", function()
    add_rules("xmcu.common")
    includes("libs")
    includes("vendor")
    includes("third-party")
    target("deps")
        set_kind("object")
        add_deps("vendor", {public=true})
    target_end()
end)
