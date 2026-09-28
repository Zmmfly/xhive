-- Run: xmake l tests/riscv_divsufsort.lua /path/to/riscv-none-elf-gcc
-- Compile both index ABIs for RV32/RV64 with the SDK's freestanding model.

function main(cc)
    assert(cc, "Pass the RISC-V GCC executable path")
    local sdkdir = path.absolute(path.directory(os.scriptdir()))
    import("xhive.testing", {rootdir = path.join(sdkdir, "modules")})
    local library = path.join(sdkdir, "libs/divsufsort/libdivsufsort")
    local work = os.tmpfile() .. "-xhive-riscv-divsufsort"
    os.mkdir(work)
    try {
        function ()
            for _, xlen in ipairs({32, 64}) do
                for _, indices in ipairs({32, 64}) do
                    for _, source in ipairs(os.files(path.join(library, "lib/*.c"))) do
                        local name = path.basename(source) .. "-rv" .. xlen .. "-idx" .. indices .. ".o"
                        local args = {"-std=c99", "-ffreestanding", "-fno-builtin", "-Os",
                                      "-Wall", "-Wextra", "-Werror", "-march=rv" .. xlen .. "im",
                                      "-mabi=" .. (xlen == 32 and "ilp32" or "lp64"),
                                      "-I", path.join(library, "include")}
                        if indices == 64 then
                            table.insert(args, "-DBUILD_DIVSUFSORT64")
                        end
                        table.join2(args, {"-c", source, "-o", path.join(work, name)})
                        os.vrunv(cc, args, {timeout = 30000})
                    end
                    print("PASS: freestanding RV" .. xlen .. ", " .. indices .. "-bit suffix indices")
                end
            end
        end,
        finally {function () testing.remove_workdir(work) end}
    }
end
