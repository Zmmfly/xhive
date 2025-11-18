includes("libadt")
includes("resource")
includes("rt-link")
includes("ulog")
includes("utest")
includes("var_export")
includes("ymodem")

target("utilities")
    set_kind("object")
    set_default(false)
    add_deps(
        "libadt", "resource", "rt-link", "ulog", "utest", "var_export", "ymodem", 
        {public=true}
    )
    on_load(function(target)
        local conf = target:data("kconfig")
        local sdir = os.scriptdir()
    end)
