function build_link_script(template_path, cc_path, output_path)
    import("xmcu.kconf")
    local conf             = kconf.load_configs()
    local xmcu_config_path = kconf.load_header_path()

    local args = {"-E", "-x", "c", "-P", "-include", xmcu_config_path, template_path}
    if output_path then
        table.insert(args, "-o")
        table.insert(args, output_path)
        os.execv(cc_path, args)
        return output_path
    else
        local result, err = os.iorunv(cc_path, args)
        assert(result, err or "failed to process template")
        return result
    end
end

function load_startup_template_path()
    import("xmcu.kconf")
    local conf = kconf.load_configs()
    local startup_dir = path.join(path.directory(path.directory(os.scriptdir())), "templates")
    if conf.CPU_ARM then
        return path.join(startup_dir, "startup_arm.c")
    else
        return path.join(startup_dir, "startup_riscv.c")
    end
end

function inject_lines(code, begin_marker, end_marker, content)
    local begin_pos = code:find(begin_marker, 1, true)
    assert(begin_pos, string.format("%s not found in startup template", begin_marker))
    local end_pos = code:find(end_marker, begin_pos, true)
    assert(end_pos, string.format("%s not found in startup template", end_marker))

    local prefix = code:sub(1, begin_pos + #begin_marker)
    local suffix = code:sub(end_pos)

    local body = content or ""
    if (#body > 0) and (body:sub(-1) ~= "\n") then
        body = body .. "\n"
    end

    return prefix .. body .. suffix
end

function build_arm_startup(template_path, periph_isr_list, output_path)
    --[[
        ISR list like {"TIM1_Handler", 0, "USART1_Handler", 0, ...}, the zero means reserved
        The every item need build to C function declare line and vector line
        The C function declare line like: "WEAK_ALIAS void TIM1_Handler(void);\n"
        The vector line like: "    TIM1_Handler,\n"
        The \n means new line
        Notice: reserved item need to build to "    0,\n" in vector line

        The C declare lines insert after "/* Peripheral Interrupt Handlers begin */" comment line in template, like:
            /* Peripheral Interrupt Handlers begin */"
            WEAK_ALIAS void TIM1_Handler(void);
            WEAK_ALIAS void USART1_Handler(void);
            /* Peripheral Interrupt Handlers end */
        The vector lines insert after "/* Peripheral Interrupts begin */" comment line in template, like:
            /* Peripheral Interrupts begin */
                TIM1_Handler,
                USART1_Handler,
            /* Peripheral Interrupts end */
     ]]

    import("xmcu.kconf")
    local conf      = kconf.load_configs()
    local dec_lines = ""
    local vec_lines = ""

    local startup = io.readfile(template_path)

    

    for _, isr in ipairs(periph_isr_list) do
        if (isr == 0) or (isr == "0") or (isr == "") or (isr == nil) then
            vec_lines = vec_lines .. "    0,\n"
        else
            dec_lines = dec_lines .. string.format("WEAK_ALIAS void %s(void);\n", isr)
            vec_lines = vec_lines .. string.format("    %s,\n", isr)
        end
    end

    startup = inject_lines(startup, "/* Peripheral Interrupt Handlers begin */", "/* Peripheral Interrupt Handlers end */", dec_lines)
    startup = inject_lines(startup, "/* Peripheral Interrupts begin */", "/* Peripheral Interrupts end */", vec_lines)

    if (output_path ~= nil) and (output_path ~= "") then
        if os.isdir(output_path) then
            raise("output_path cannot be a directory")
        end
        io.writefile(output_path, startup)
    end
    return startup
end
