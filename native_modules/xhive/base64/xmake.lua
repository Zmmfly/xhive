includes("../../../libs/base64")

target("base64")
    add_rules("module.shared")
    add_deps("base64")
    add_files("base64.c")