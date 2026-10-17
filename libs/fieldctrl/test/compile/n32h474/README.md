# N32H474 compile/link fixture

This is a **compile/link-only** Cortex-M4, single-precision-FPU application for
`linears`, `filters`, and `fieldctrl`. It is not a numerical runtime test, a motor
application, or permission to flash/run hardware. It contains only ordinary C
sources, so the library's `test/**.cc` discovery does not register another host
test entry.

The fixture makes real calls in all four formats and uses 292
`--require-defined` linker roots to retain every public algorithm variant while
allowing unused vendor code to be garbage-collected. It uses caller-owned,
fixed-size storage, the SDK startup/linker templates, no RTOS, 256 KiB ROM and
96 KiB RAM. The empty `_init`/`_fini` hooks satisfy Newlib's `-nostartfiles`
contract; they are not substitutes for library implementations.

## Reproduce outside the SDK

Run the following from the SDK root. Choose a **new, absent** application path;
do not build inside this source fixture or reuse another application's cache.
The existing xHive ARM GCC toolchain and application Kconfig tools must already
be installed. These commands do not install a cross-toolchain or access a board.

```sh
sdk="$PWD"
app="$HOME/prjs/xhive_testspace/fieldctrl-reproduce"
test ! -e "$app" || exit 1
mkdir -p "$app"
cp -R "$sdk/libs/fieldctrl/test/compile/n32h474/." "$app/"
cd "$app"

XHIVE_SDK_PATH="$sdk" XMAKE_CONFIGDIR="$app" \
    xmake f -P . -y -m release --xhive_tests=n
XHIVE_SDK_PATH="$sdk" XMAKE_CONFIGDIR="$app" \
    xmake b -P . app::fieldctrl_compile
```

Outputs are `dist/fieldctrl_compile.elf`, `.bin`, and `.map`. For the tested
Arm GNU 14.3.1 toolchain, the first retained-all-variants image used 260,180 of
262,144 ROM bytes; **only 1,964 bytes remained**. A fresh-directory reproduction
from these six payload files and the final library headers used 260,140 bytes.
Both links retained all 292 roots; binary byte identity is not claimed.
Compiler/configuration changes can alter the size. Do not enlarge the linker
region to turn an overflow into a passing result.

`-fstack-usage`, VLA/alloca errors and a 4096-byte stack-warning threshold are
configured. Compiler `.su` files are beside objects under `build/.objs`.
Per-function stack sizes are not a whole-call-chain, interrupt-nesting, libc/libm
stack bound, or a measurement of runtime RAM. The reported data+bss size likewise
does not establish peak RAM use.

Use the matching toolchain's `nm`, `size`, and `readelf` to inspect the produced
ELF. The validated image retained 292/292 requested algorithm symbols, had no
unresolved or matched allocator symbols, and identified itself as ELF32 ARM,
EABI5 hard-float, VFPv4-D16. The exact checks, failures encountered before the
final flags, and validation limits are recorded in
[`docs/fieldctrl-worklog.md`](../../../../../docs/fieldctrl-worklog.md).
There is deliberately no flash, debugger, serial-port, or hardware-run command.

## Validated payload identity

These six files were copied byte-for-byte from the successful external fixture
at `/home/zmmfly/prjs/xhive_testspace/fieldctrl`. The SHA-256 values identify the
source/configuration payload, not a promise of identical ELF debug paths after
copying to another directory. No build/cache/log/map/binary output is included.

| Relative path | SHA-256 |
| --- | --- |
| `.config` | `579693c9b4c5363a1317573898d8299ec0d52e93bef93cec4b7e436219812ff2` |
| `xmake.lua` | `bcb0becca3141c8249f7285165587d000fef3396cc0a677b5ff408fd3478d8ed` |
| `src/main.c` | `0f8d8a9f177978c29007f1e354d6e6ce49e440d9561801de7ac6878f6704a9f9` |
| `src/linears_calls.c` | `0bba4f377b5ff2c71c424f75993ba08d32b71120900a75d742d0233d589b872b` |
| `src/filters_calls.c` | `c04563aab901c91b6909416d11c12da7ab50ab217396405e81840391f276311c` |
| `src/control_calls.c` | `6fcf163ff9105ee66d3d1a5f442966b8efbee31eba1ca8df07507604ffa85ced` |
