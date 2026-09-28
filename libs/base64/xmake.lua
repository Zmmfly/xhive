target("base64")
    set_kind("object")
    set_default(false)
    add_files("src/*.c")
    add_includedirs("inc", {public = true})
target_end()

-- The SDK registers these tests only in xhive_host. Preserve direct
-- `xmake test -P libs/base64` usage without registering embedded copies.
if path.normalize(os.projectdir()) == path.normalize(os.scriptdir()) then
    includes("tests.lua")
end
