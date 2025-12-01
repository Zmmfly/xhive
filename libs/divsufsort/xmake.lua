-- local build_divsufsort_header = function(target, is_64)

-- end

includes("@builtin/check")

target("divsufsort")
    set_kind("object")
    set_configdir("$(builddir)/divsufsort")
    add_configfiles("config.h.in")
    configvar_check_cincludes("HAVE_INTTYPES_H", "inttypes.h")
    configvar_check_cincludes("HAVE_MEMORY_H", "memory.h")
    configvar_check_cincludes("HAVE_STDDEF_H", "stddef.h")
    configvar_check_cincludes("HAVE_STDINT_H", "stdint.h")
    configvar_check_cincludes("HAVE_STDLIB_H", "stdlib.h")
    configvar_check_cincludes("HAVE_STRING_H", "string.h")
    configvar_check_cincludes("HAVE_STRINGS_H", "strings.h")
    configvar_check_cincludes("HAVE_SYS_TYPES_H", "sys/types.h")

    add_files("libdivsufsort/lib/*.c")
    add_includedirs("$(builddir)/divsufsort")
    add_includedirs("libdivsufsort/include")
    add_defines("HAVE_CONFIG_H", [[PROJECT_VERSION_FULL="2.0.1"]])

    on_load(function(target)
        -- import("xhive.base")
        local conf = target:data("kconfig")
        local is_64    = (conf and conf.DIVSUFSORT_ENABLE_64BIT) or false
        local suffix   = is_64 and "64" or "32"
        local sdir     = os.scriptdir()
        local builddir = path.join(os.projectdir(), "build")
        local mk_dir   = path.join(builddir, "divsufsort")
        local inc_dir  = path.join(mk_dir, "inc")
        local orig_dir = path.join(sdir, "libdivsufsort")
        local orig_inc = path.join(orig_dir, "include")
        local incs     = {}
        local defs     = {}

        -- mkdir inc dir
        if not os.isdir(inc_dir) then
            os.mkdir(inc_dir)
        end

        -- read header file
        local orig_header = io.readfile(path.join(orig_inc, "divsufsort.h.cmake"))
        if not orig_header then
            raise("divsufsort: read header file failed!")
        end

        -- export/import
        if target:kind() == "shared" and target:is_plat("windows") then
            -- replace @DIVSUFSORT_EXPORT@ in orig_header
            orig_header = orig_header:gsub("@DIVSUFSORT_EXPORT@", "__declspec(dllexport)")
        else
            orig_header = orig_header:gsub("@DIVSUFSORT_EXPORT@", "")
            orig_header = orig_header:gsub("@DIVSUFSORT_IMPORT@", "")
        end

        -- replace @INCFILE@ to #include <stdint.h>
        orig_header = orig_header:gsub("@INCFILE@", "#include <stdint.h>\n" .. "#include <inttypes.h>")

        -- replace @SAUCHAR_TYPE@ to uint8_t
        orig_header = orig_header:gsub("@SAUCHAR_TYPE@", "uint8_t")

        -- replace @SAINT32_TYPE@ to int32_t
        orig_header = orig_header:gsub("@SAINT32_TYPE@", "int32_t")

        local header_path = ""

        -- replace types
        if is_64 then
            header_path = path.join(inc_dir, "divsufsort64.h")
            -- replace @W64BIT@ to 64
            orig_header = orig_header:gsub("@W64BIT@", "64")
            -- replace @SAINDEX_TYPE@ to int64_t
            orig_header = orig_header:gsub("@SAINDEX_TYPE@", "int64_t")
            -- replace @SAINDEX_PRId@ to PRId64
            orig_header = orig_header:gsub("@SAINDEX_PRId@", "PRId64")
            -- replace @SAINT_PRId@ to PRId64
            orig_header = orig_header:gsub("@SAINT_PRId@", "PRId64")
            -- add BUILD_DIVSUFSORT64
            target:add("defines", "BUILD_DIVSUFSORT64")
        else
            header_path = path.join(inc_dir, "divsufsort.h")
            -- replace @W64BIT@ to empty
            orig_header = orig_header:gsub("@W64BIT@", "")
            -- replace @SAINDEX_TYPE@ to int32_t
            orig_header = orig_header:gsub("@SAINDEX_TYPE@", "int32_t")
            -- replace @SAINDEX_PRId@ to PRId32
            orig_header = orig_header:gsub("@SAINDEX_PRId@", "PRId32")
            -- replace @SAINT_PRId@ to PRId32
            orig_header = orig_header:gsub("@SAINT_PRId@", "PRId32")
        end
        
        -- write header file
        io.writefile(header_path, orig_header)
        target:add("includedirs", inc_dir, {public = true})
    end)
target_end()
