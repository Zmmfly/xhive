target("bitmap")
    set_kind("headeronly")
    add_includedirs("inc", {public = true})
target_end()

-- The SDK registers these tests only in xhive_host. Preserve direct
-- `xmake test -P libs/bitmap` usage without registering embedded copies.
if path.normalize(os.projectdir()) == path.normalize(os.scriptdir()) then
    includes("tests.lua")
end
