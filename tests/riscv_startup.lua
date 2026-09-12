-- Run: xmake l tests/riscv_startup.lua /path/to/riscv-none-elf-gcc
-- Link the real startup/linker templates, including assembly-only references.

function main(cc)
    assert(cc, "Pass the RISC-V GCC executable path")
    local sdkdir = path.absolute(path.directory(os.scriptdir()))
    local work = os.tmpfile() .. "-xhive-startup"
    os.mkdir(work)
    try {
        function ()
            local source = path.join(work, "main.c")
            local config = path.join(work, "xhive_config.h")
            local linker = path.join(work, "link.ld")
            io.writefile(source, [[
#include <stdint.h>
volatile uint32_t initialized = 0x12345678u;
volatile uint32_t zeroed;
__attribute__((section(".ramfunc"), noinline)) int main(void)
{
    zeroed = initialized;
    for (;;) {}
}
]])
            for _, bits in ipairs({32, 64}) do
                io.writefile(config, string.format([[
#define CONFIG_CPU_RISCV 1
#define CONFIG_CPU_64BITS %d
#define CONFIG_FLASH_START 0
#define CONFIG_RAM_START 0x30000000
#define CONFIG_STACK_LENGTH 0x400
#define CONFIG_ENABLE_EXEC_IN_RAM 1
#define CONFIG_ENABLE_FAST_STARTUP 1
]], bits == 64 and 1 or 0))
                os.vrunv(cc, {"-E", "-P", "-x", "c", "-include", config,
                             path.join(sdkdir, "templates", "link.ld"), "-o", linker})
                for _, lto in ipairs({false, true}) do
                    local name = "rv" .. bits .. (lto and "_lto" or "_no_lto")
                    local elf = path.join(work, name .. ".elf")
                    local flags = {"-march=rv" .. bits .. "imc",
                                   "-mabi=" .. (bits == 64 and "lp64" or "ilp32"),
                                   "-Os", "-ffreestanding", "-ffunction-sections",
                                   "-fdata-sections", "-nostdlib", "-Wl,--gc-sections",
                                   "-I", work, "-T", linker}
                    if lto then
                        table.insert(flags, "-flto")
                    end
                    os.vrunv(cc, table.join(flags, {
                        path.join(sdkdir, "templates", "startup_riscv.c"),
                        source, "-o", elf}))
                    local nm_name = path.filename(cc):gsub("gcc", "nm")
                    local nm = path.join(path.directory(cc), nm_name)
                    local symbols = os.iorunv(nm, {"--defined-only", elf})
                    for _, symbol in ipairs({"Reset_Entry", "Reset_Handler"}) do
                        assert(symbols:find(" T " .. symbol .. "\n", 1, true),
                               name .. " lost " .. symbol)
                    end
                    -- LTO may internalize main and create an IPA clone.
                    local address = symbols:match("(%x+) [Tt] main[%w%._]*\n")
                    assert(address and tonumber(address, 16) == 0x30000000,
                           name .. " did not place main in internal RAM")
                    print("PASS: startup " .. name)
                end
            end
        end,
        catch {function (errors) raise(errors) end},
        finally {function () os.rm(work) end}
    }
end
