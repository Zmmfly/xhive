--[[
Copyright (c) 2025 Zmmfly. All rights reserved.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors
   may be used to endorse or promote products derived from this software
   without specific prior written permission.
]]

function build_link_script(template_path, cc_path, output_path)
    import("xhive.kconf")
    local conf        = kconf.load_configs()
    local config_path = kconf.load_header_path()

    local args = {"-E", "-x", "c", "-P", "-include", config_path, template_path}
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
    import("xhive.kconf")
    local conf = kconf.load_configs()
    local startup_dir = path.join(path.directory(path.directory(os.scriptdir())), "templates")
    if conf.CPU_ARM then
        return path.join(startup_dir, "startup_arm.c")
    elseif conf.CPU_RISCV then
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

    import("xhive.kconf")
    local conf      = kconf.load_configs()
    local dec_lines = ""
    local vec_lines = ""

    local startup = io.readfile(template_path)

    -- Build declare lines and vector lines
    for _, isr in ipairs(periph_isr_list) do
        if (isr == 0) or (isr == "0") or (isr == "") or (isr == nil) then
            vec_lines = vec_lines .. "    0,\n"
        else
            dec_lines = dec_lines .. string.format("WEAK_ALIAS void %s(void);\n", isr)
            vec_lines = vec_lines .. string.format("    %s,\n", isr)
        end
    end

    -- Inject lines into template
    startup = inject_lines(startup, "/* Peripheral Interrupt Handlers begin */", "/* Peripheral Interrupt Handlers end */", dec_lines)
    startup = inject_lines(startup, "/* Peripheral Interrupts begin */", "/* Peripheral Interrupts end */", vec_lines)

    -- Write to output file if specified
    if (output_path ~= nil) and (output_path ~= "") then
        if os.isdir(output_path) then
            raise("output_path cannot be a directory")
        end

        local old_content = os.exists(output_path) and io.readfile(output_path) or ""
        if old_content ~= startup then
            print("Generating ARM startup file: " .. output_path)
            io.writefile(output_path, startup)
        end
    end
    return startup
end

local floor, min = math.floor, math.min

-- 单位表，顺序必须是 B→KB→MB→GB→TB→PB
local UNITS = {"B", "KB", "MB", "GB", "TB", "PB"}
local BASE  = 1024

--[[--
  将字节数转换成人类可读字符串
  @param size  整数（number 或 string）
  @param prec  保留小数位，默认 2
  @return string  如 "1.50 KB"
--]]
 function len2hum(size, prec)
    prec = prec or 2
    size = tonumber(size)
    if not size or size < 0 then
        return nil
    end

    local u = 1
    while u < #UNITS and size >= BASE do
        size = size / BASE
        u = u + 1
    end

    -- 按 prec 四舍五入
    local mul = 10^prec
    size = floor(size * mul + 0.5) / mul

    -- 去掉无意义的 .00
    local fmt = (floor(size) == size) and "%.0f %s" or ("%."..prec.."f %s")
    return fmt:format(size, UNITS[u])
end

--[[--
  将人类可读字符串转回字节数
  @param str  如 "123.45kb"、"12 KB"、"3TB"
  @return number|nil  字节数，格式非法返回 nil
--]]
function hum2len(str)
    if type(str) ~= "string" then return nil end
    -- 提取数字部分与单位部分，忽略大小写
    local num, unit = str:match("^%s*(%d+%.?%d*)%s*([a-zA-Z]+)%s*$")
    if not num then return nil end
    unit = unit:upper()

    local uidx
    for i, v in ipairs(UNITS) do
        if v == unit then
            uidx = i
            break
        end
    end
    if not uidx then return nil end  -- 单位不在表内

    local bytes = tonumber(num)
    if not bytes then return nil end

    return floor(bytes * (BASE ^ (uidx - 1)) + 0.5)
end

function cpuinfo_by_conf(conf)
    local arch = nil
    local core = nil

    local arch_maps = {
        CPU_ARM   = "arm",
        CPU_RISCV = "risc-v",
    }

    local core_maps = {
        CPU_ARM = {
            CORE_ARM_CORTEX_M0   = "cortex-m0",
            CORE_ARM_CORTEX_M0P  = "cortex-m0plus",
            CORE_ARM_CORTEX_M3   = "cortex-m3",
            CORE_ARM_CORTEX_M4   = "cortex-m4",
            CORE_ARM_CORTEX_M7   = "cortex-m7",
            CORE_ARM_CORTEX_M23  = "cortex-m23",
            CORE_ARM_CORTEX_M33  = "cortex-m33",
            CORE_ARM_CORTEX_M35P = "cortex-m35plus",
            CORE_ARM_CORTEX_M55  = "cortex-m55",
            CORE_ARM_CORTEX_M85  = "cortex-m85",
            CORE_ARM_CORTEX_R4   = "cortex-r4",
            CORE_ARM_CORTEX_R5   = "cortex-r5",
            CORE_ARM_CORTEX_R52  = "cortex-r52",
            CORE_ARM_CORTEX_A5   = "cortex-a5",
            CORE_ARM_CORTEX_A7   = "cortex-a7",
            CORE_ARM_CORTEX_A8   = "cortex-a8",
            CORE_ARM_CORTEX_A9   = "cortex-a9",
            CORE_ARM_CORTEX_A53  = "cortex-a53",
            CORE_ARM_CORTEX_A55  = "cortex-a55",
        },
        CPU_RISCV = {
            CORE_RISCV_E902       = "e902",
            CORE_RISCV_E906       = "e906",
            CORE_RISCV_E907       = "e907",
            CORE_RISCV_C906       = "c906",
            CORE_RISCV_C910       = "c910",
            CORE_RISCV_BUMBLEBEE  = "bumblebee",
            CORE_RISCV_QINGKE_V2A = "qingke_v2a",
            CORE_RISCV_QINGKE_V2C = "qingke_v2c",
            CORE_RISCV_QINGKE_V3A = "qingke_v3a",
            CORE_RISCV_QINGKE_V3B = "qingke_v3b",
            CORE_RISCV_QINGKE_V3C = "qingke_v3c",
            CORE_RISCV_QINGKE_V3F = "qingke_v3f",
            CORE_RISCV_QINGKE_V4A = "qingke_v4a",
            CORE_RISCV_QINGKE_V4B = "qingke_v4b",
            CORE_RISCV_QINGKE_V4C = "qingke_v4c",
            CORE_RISCV_QINGKE_V4F = "qingke_v4f",
            CORE_RISCV_QINGKE_V4J = "qingke_v4j",
            CORE_RISCV_QINGKE_V5F = "qingke_v5f",
            CORE_RISCV_ANDES_D45  = "andes_d45",
            CORE_RISCV_ANDES_D25F = "andes_d25f",
        },
    }

    local cpu_maps = {
        ARCH_ARM_AARCH32 = "aarch32",
        ARCH_ARM_AARCH64 = "aarch64",
        ARCH_RISCV_RV32  = "rv32",
        ARCH_RISCV_RV64  = "rv64",
    }

    for k, v in pairs(arch_maps) do
        if conf[k] then
            arch = v
            local maps = core_maps[k]
            for ck, cv in pairs(maps) do
                if conf[ck] then
                    core = cv
                    break
                end
            end
            break
        end
    end

    local result = {
        arch = arch,
        core = core
    }

    for k, v in pairs(cpu_maps) do
        if conf[k] then
            result.cpu = v
            break
        end
    end

    return result
end
