includes("../../../libs/micro-ecc")
includes("../../../libs/sha256")
includes("../../../libs/base64")

target("mecc")
    add_rules("module.shared")
    add_deps("micro-ecc", "sha256", "base64")
    add_files("mecc.c")
