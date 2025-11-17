target("mm")
    set_kind("object")
    on_load(function(target)
        local conf = target:data("kconfig")
        local sdir = os.scriptdir()

        local is_any_enabled = function(cfg, lst)
            for _, v in ipairs(lst) do
                if cfg[v] then
                    return true
                end
            end
            return false
        end
        if not is_any_enabled(conf, {"ARCH_ARM_CORTEX_A", "ARCH_ARMV8", "ARCH_RISCV64"}) then
            return
        end
        if not conf.ARCH_MM_MMU then
            return
        end

        local cfiles = os.files(path.join(sdir, "*.c"))
        local asmfiles = os.files(path.join(sdir, "*_gcc.S"))
        -- remove mm_memblock.c if RT_USING_MEMBLOCK not enabled, from tail to head with -1 index to void index shift
        if not conf.RT_USING_MEMBLOCK then
            for i = #cfiles, 1, -1 do
                if path.filename(cfiles[i]) == "mm_memblock.c" then
                    table.remove(cfiles, i)
                    break
                end
            end
        end
        target:add("files", cfiles)
        if #asmfiles > 0 then
            target:add("files", asmfiles)
        end
        target:add("includedirs", sdir, {public = true})
    end)
