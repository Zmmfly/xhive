target("cplusplus")
    set_kind("object")
    set_default(false)
    on_load(function(target)
        local conf = target:data("kconfig")
        local sdir = os.scriptdir()
        if conf.RT_USING_CPLUSPLUS and conf.RT_USING_CPLUSPLUS11 then
            local cxxdir = nil
            if conf.COMPILER_GCC then
                cxxdir = path.join(sdir, "cpp11", "gcc")
            elseif conf.COMPILER_ARMCLANG then
                cxxdir = path.join(sdir, "cpp11", "armclang")
            end
            if not cxxdir then 
                raise("Not supported c++11 compiler!")
            end

            -- add include dirs
            target:add("includedirs", cxxdir, {public = true})
            target:add("files", path.join(sdir, "*.c"))
            target:add("files", path.join(sdir, "*.cpp"))
            target:add("files", path.join(cxxdir, "*.cpp"))
        end

        if conf.RT_USING_CPP_WRAPPER then 
            local wrap_dir = path.join(sdir, "os")
            target:add("includedirs", wrap_dir, {public = true})
            target:add("files", path.join(wrap_dir, "*.c"))
            target:add("files", path.join(wrap_dir, "*.cpp"))
        end
    end)