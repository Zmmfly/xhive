-- Run: xmake l tests/divsufsort.lua
-- Build real 32/64-bit Xmake consumers in fresh directories, without Kconfig
-- tools, generated headers, or dependencies on existing SDK build artifacts.

function main()
    local sdkdir = path.absolute(path.directory(os.scriptdir()))
    import("xhive.testing", {rootdir = path.join(sdkdir, "modules")})
    local work = os.tmpfile() .. "-xhive-divsufsort"
    os.mkdir(work)
    try {
        function ()
            for _, bits in ipairs({32, 64}) do
                local projectdir = path.join(work, tostring(bits))
                local suffix = bits == 64 and "64" or ""
                os.mkdir(projectdir)
                io.writefile(path.join(projectdir, "xmake.lua"), string.format([[
rule("test.config")
    on_load(function(target)
        target:data_set("kconfig", {DIVSUFSORT_ENABLE_64BIT = %s})
    end)
rule_end()
add_rules("test.config")
includes(%q)
target("consumer")
    set_kind("binary")
    set_languages("c99", "c++11")
    set_warnings("all", "error")
    add_files("consumer.c", "reverse_headers.cc")
    add_deps("divsufsort")
target_end()
]], tostring(bits == 64), path.join(sdkdir, "libs/divsufsort")))
                io.writefile(path.join(projectdir, "consumer.c"), string.format([[
#include <divsufsort.h>
#include <divsufsort64.h>
#include <divsufsort.h>
#include <divsufsort64.h>
#include <stdio.h>
#include <string.h>

typedef char byte_width[(sizeof(sauchar_t) == 1) ? 1 : -1];
typedef char status_width[(sizeof(saint_t) == 4) ? 1 : -1];
typedef char index32_width[(sizeof(saidx_t) == 4) ? 1 : -1];
typedef char index64_width[(sizeof(saidx64_t) == 8) ? 1 : -1];
int reverse_header_check(void);
int main(void) {
    const sauchar_t text[] = "banana";
    saidx%s_t sa[6];
    const saidx%s_t expected[6] = {5, 3, 1, 0, 4, 2};
    char formatted[64];
    int i;
    if (reverse_header_check() != 0) return 1;
    if (divsufsort%s(text, sa, 6) != 0) return 2;
    for (i = 0; i < 6; ++i) if (sa[i] != expected[i]) return 3;
    if (sufcheck%s(text, sa, 6, 0) != 0) return 4;
    snprintf(formatted, sizeof(formatted), "%%" PRIdSAINT_T, (saint_t)-17);
    if (strcmp(formatted, "-17") != 0) return 5;
    return 0;
}
]], suffix, suffix, suffix, suffix))
                io.writefile(path.join(projectdir, "reverse_headers.cc"), [[
#include <divsufsort64.h>
#include <divsufsort.h>
#include <divsufsort64.h>
#include <divsufsort.h>
#include <type_traits>
static_assert(std::is_same<decltype(divsufsort64(nullptr, nullptr, 0)), saint_t>::value, "status ABI");
static_assert(std::is_same<decltype(divbwt64(nullptr, nullptr, nullptr, 0)), saidx64_t>::value, "index ABI");
extern "C" int reverse_header_check(void) {
    return sizeof(saint_t) != 4 || sizeof(saidx_t) != 4 || sizeof(saidx64_t) != 8;
}
]])
                local runopt = {
                    curdir = projectdir, timeout = 30000,
                    envs = {XMAKE_CONFIGDIR = projectdir, XMAKE_STATS = "false"}
                }
                os.vrunv(os.programfile(), {"f", "-P", projectdir, "-y", "-m", "debug"}, runopt)
                os.vrunv(os.programfile(), {"build", "-P", projectdir, "consumer"}, runopt)
                os.vrunv(os.programfile(), {"run", "-P", projectdir, "consumer"}, runopt)
                assert(#os.files(path.join(projectdir, "build", "**.h")) == 0,
                       "A header was generated in the consumer build directory")
                print("PASS: real " .. bits .. "-bit Xmake consumer; C/C++ headers; no generated headers")
            end
        end,
        finally {function () testing.remove_workdir(work) end}
    }
end
