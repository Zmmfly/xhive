-- Run: xmake l tests/riscv_toolset.lua /path/to/riscv-none-elf-gcc
-- Cross-compile and link real objects; no hardware or simulator is required.

function main(cc)
    assert(cc, "Pass the RISC-V GCC executable path")
    local sdkdir = path.directory(os.scriptdir())
    import("xhive.toolset", {rootdir = path.join(sdkdir, "modules")})

    local variants = {
        {name = "rv32imc", bits = 32, isa = {"I", "M", "C"}},
        {name = "rv32imc_lto", bits = 32, isa = {"I", "M", "C"}, lto = true},
        {name = "rv64gc", bits = 64, isa = {"G", "C"}, double = true},
        {name = "rv32im_zicsr", bits = 32, isa = {"I", "M", "ZICSR"}}
    }
    local work = os.tmpfile() .. "-xhive-riscv"
    os.mkdir(work)
    local source = path.join(work, "entry.c")
    io.writefile(source, [[
#include <stdint.h>
_Static_assert(sizeof(uintptr_t) * 8 == EXPECTED_XLEN, "Incorrect ABI width");
volatile uint32_t initialized = 0x12345678u;
volatile uint32_t zeroed;
void _start(void) { zeroed = initialized; for (;;) {} }
]])

    for _, variant in ipairs(variants) do
        local conf = {
            COMPILER_RISCV_GCC = true,
            ARCH_RISCV_RV32 = variant.bits == 32,
            ARCH_RISCV_RV64 = variant.bits == 64,
            ARCH_RISCV_FPU_DOUBLE = variant.double,
            COMPILER_ENABLE_LTO = variant.lto,
            NO_STD_STARTFILE = true
        }
        for _, isa in ipairs(variant.isa) do
            conf["RISCV_ISA_" .. isa] = true
        end
        local flags = toolset.build_riscv_flags(conf)
        local object = path.join(work, variant.name .. ".o")
        local elf = path.join(work, variant.name .. ".elf")
        local binary = path.join(work, variant.name .. ".bin")
        os.vrunv(cc, table.join(flags.cxflags,
                 {"-DEXPECTED_XLEN=" .. variant.bits, "-c", source, "-o", object}))
        os.vrunv(cc, table.join(flags.ldflags, {object, "-o", elf}))
        -- Also exercise paths whose parent directory contains "gcc".
        toolset.elf_to_bin(cc, elf, binary)
        assert(os.filesize(binary) > 0, "Empty binary: " .. variant.name)
        print("PASS: " .. variant.name)
    end

    for _, width in ipairs({{}, {ARCH_RISCV_RV32 = false},
                           {ARCH_RISCV_RV32 = true, ARCH_RISCV_RV64 = true}}) do
        local conf = table.join(width, {COMPILER_RISCV_GCC = true, RISCV_ISA_I = true})
        local rejected = false
        try {
            function () toolset.build_riscv_flags(conf) end,
            catch {function () rejected = true end}
        }
        assert(rejected, "Accepted invalid XLEN")
    end
    print("PASS: reject missing/ambiguous XLEN")
    os.rm(work)
end
