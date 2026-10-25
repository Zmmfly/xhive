# N32H474 fixed-Q compile/link fixture

The `src/` application is a **compile/link-only** Cortex-M4, single-FPU fixture
for `linears`, `filters`, and `fieldctrl`. It is not a motor application or
permission to flash/run hardware. The separate top-level `host_benchmark.c` is
a Linux host comparison tool; it is not included by the firmware's `src/*.c`
rule. None of these `.c` files adds a `test/**.cc` GoogleTest entry.

The firmware makes real calls in all four formats and uses 296
`--require-defined` roots to retain every public algorithm variant while
collecting unused vendor code. It uses caller-owned fixed-size storage, SDK
startup/linker templates, no RTOS, 256 KiB ROM and 96 KiB RAM. Empty `_init`/
`_fini` hooks satisfy the C-only Newlib `-nostartfiles` contract; no algorithm
implementation is stubbed.

## Fixed-Q inputs and storage

Matrices have three fields; Q signals are raw Q1.15/Q1.31 values. PID/scalar
Kalman coefficients use the documented wide F31 format. Q matrix Kalman uses
actual `linears_fixed_t[]` intermediates and the `FILTERS_KALMAN_Q_*` capacity
macros, not a cast of the former raw-signal workspace.

The Q decomposition example uses `diag(1/4,3/8)` and a `(1/8)*I` right-hand
side. Inverse uses a separate `(3/4)*[[1,1],[1,-1]]` input whose inverse entries
are representable +/-2/3, even though intermediate LU values exceed Q1. Signals
in the filter/control examples have similar headroom. C99, C++17 and sanitized
host consumers executed all probe calls successfully; that is not an ARM or
motor runtime result.

## Reproduce the ARM link outside the SDK

From the SDK root, select an existing external scratch directory. Keep cwd at
the copied application and isolate its cache. The xHive ARM toolchain and
application Kconfig tools must already be installed; no cross-toolchain install
or device access is performed by these commands.

```sh
set -eu
sdk="$PWD"
scratch="$HOME/prjs/xhive_testspace/fieldctrl"
test -d "$scratch" || exit 1
app=$(mktemp -d "$scratch/fieldctrl-xhive-arm-XXXXXX")
cp -R "$sdk/libs/fieldctrl/test/compile/n32h474/." "$app/"
cd "$app"

XHIVE_SDK_PATH="$sdk" XMAKE_CONFIGDIR="$app" \
    xmake f -P . -y -m release --xhive_tests=n
XHIVE_SDK_PATH="$sdk" XMAKE_CONFIGDIR="$app" \
    xmake b -P . app::fieldctrl_compile
```

At source revision `3e84626`, Arm GNU 14.3.1 produced
`dist/fieldctrl_compile.elf`, `.bin`, and `.map`. The image used
**238,708 / 262,144 ROM bytes**, with data+bss of 1,128 bytes. All 292 requested
function symbols survived collection; no unresolved or matched allocator
symbols were present. ELF identity was ELF32 ARM, EABI5 hard-float, VFPv4-D16.
Compiler/layout changes can alter size; do not enlarge the memory region to
turn an overflow into a pass.

The PI-to-PID change replaced the `pi_init`/`pi_step` roots with
`pid_init`/`pid_reset`/`pid_step` (292 to 296 roots). Re-linked with the same
xHive Arm GNU 14.3.1 toolchain, the image used **246,300 ROM bytes** against
238,732 for the preceding source in the same environment; data+bss stayed
1,128 bytes and all 12 PID roots survived collection. The largest fieldctrl
frame became `fieldctrl_f64_speed_foc_step` at 480 bytes (previously 320), as
the speed cascade now embeds the larger PID context.

`-fstack-usage`, VLA/alloca errors and a 4096-byte frame-warning threshold are
enabled. The measured 392 library `.su` entries were static; largest individual
frames were 656 bytes (linears), 488 (filters), and 384 (fieldctrl). These are
not whole-call-chain/interrupt/libc bounds or peak runtime RAM measurements.
Fixed-Q does not universally reduce storage: on the tested ARM ABI, q15 Kalman
update buffers for n=2,m=1 grew from 120 to 408 bytes. Old scalar ABI sizes
8→2 / 8→4 bytes refer to exponent-bearing q15/q31 **scalars**; old matrix
elements were already int16/int32, and remain so. Details and additional memory
measurements are in the [worklog](../../../../../docs/fieldctrl-worklog.md).

No ARM image was flashed or executed. In particular, successful linking and
numerical tests do not restore the documented fixed-Q15 SMO scenarios' former
0.25 Hz peak / 0.03 Hz final-sample accuracy; use Q31/f32 for that requirement.

## Minimal historical/new host comparison

`host_benchmark.c` is the exact small program used for the paired measurements.
Its `BENCH_BFP=1` branch adapts calls **only to the archived `ab11032` BFP
source**. `BENCH_BFP=0` uses raw fixed-Q at `3e84626`. This switch never changes
production code or offers a current-library BFP compatibility mode. Long-double
decoding is an independent postcondition check outside the timer, not Q
computation inside the library.

The program measures q15/q31 matrix4, matrix16, dot256, FIR16, PI and VVVF calls.
Every call's status is checked, with exact binary-fraction result checks outside
`CLOCK_MONOTONIC` timing. Initialization, warmup and printing are excluded. VVVF
resets after warmup and ends after 20,480 steps at angle zero with exact duties
(7/8,1/8,1/8). This covers shared representable workloads, not all accuracy or
range behavior.

The following Bash commands use ordinary Linux host `cc`, not the ARM compiler.
They rebuild both recorded source versions with the same
O2/strict-aliasing/no-LTO flags:

```sh
set -euo pipefail
# Start in the SDK root; use an existing external scratch directory.
sdk="$PWD"
scratch="$HOME/prjs/xhive_testspace/fieldctrl"
work=$(mktemp -d "$scratch/fieldctrl-xhive-hostbench-XXXXXX")
cp "$sdk/libs/fieldctrl/test/compile/n32h474/host_benchmark.c" "$work/benchmark.c"
for mode in before after; do
    if [ "$mode" = before ]; then rev=ab11032; legacy=1; else rev=3e84626; legacy=0; fi
    src="$work/$mode-source"
    mkdir "$src" "$work/$mode-objects"
    git -C "$sdk" archive "$rev" LICENSE libs/linears libs/filters libs/fieldctrl | tar -x -C "$src"
    for lib in linears filters fieldctrl; do
        for file in "$src/libs/$lib/src/"*.c; do
            cc -std=c99 -pedantic-errors -O2 -fstrict-aliasing -fno-lto \
                -Wall -Wextra -Werror -Wvla -Walloca -ffunction-sections -fdata-sections \
                -I"$src/libs/linears/include" -I"$src/libs/filters/include" -I"$src/libs/fieldctrl/include" \
                -c "$file" -o "$work/$mode-objects/$lib-$(basename "$file" .c).o"
        done
    done
    cc -std=c99 -pedantic-errors -O2 -fstrict-aliasing -fno-lto \
        -Wall -Wextra -Werror -Wvla -Walloca -ffunction-sections -fdata-sections \
        -I"$src/libs/linears/include" -I"$src/libs/filters/include" -I"$src/libs/fieldctrl/include" \
        -DBENCH_BFP="$legacy" "$work/benchmark.c" "$work/$mode-objects/"*.o \
        -Wl,--gc-sections -lm -o "$work/$mode-benchmark"
done
```

For the recorded experiment, GCC 15.2.0 on an AMD Ryzen 9 7945HX ran seven
alternating before/after batches pinned to allowed logical CPU 0. Frequency
policy was not changed; min/median/max spread is retained. To repeat that
method without changing global affinity or frequency settings:

```sh
python3 - "$work" <<'PY'
import os, pathlib, subprocess, sys
root = pathlib.Path(sys.argv[1])
os.sched_setaffinity(0, {min(os.sched_getaffinity(0))})
for batch in range(7):
    order = ('before', 'after') if batch % 2 == 0 else ('after', 'before')
    for mode in order:
        with (root / f'{batch}-{mode}.csv').open('x') as out:
            subprocess.run([str(root / f'{mode}-benchmark')], stdout=out, check=True)
PY
```

Report these as **host implementation timings**, not MCU cycles. The new VVVF
implementation includes the separately verified CORDIC divide-to-shift change;
its improvement is not solely caused by removing exponents. Keep every workload,
including PI, and do not replace medians/peaks with whichever result looks best.
Recorded values, limits and source identities are in the worklog; no general
performance framework or new build/test target is introduced here.

## Validated source identities

The six firmware inputs match the successful external run at
`fixed-q-xhive-arm-9b2DGw` byte-for-byte. The host program matches the measured
`fixed-q-xhive-comparison-ut5dAw/benchmark.c`. These hashes identify source inputs,
not identical ELF debug/link layout across directories. No generated artifacts
are checked in. `xmake.lua` has since changed only in its PID root list; its
current SHA-256 is
`494ce203c5191a646e0f1bdb2020c7aebefa4e0b5413e3591d0b2e773ea1289c`.

| Relative path | SHA-256 |
| --- | --- |
| `.config` | `579693c9b4c5363a1317573898d8299ec0d52e93bef93cec4b7e436219812ff2` |
| `xmake.lua` | `bcb0becca3141c8249f7285165587d000fef3396cc0a677b5ff408fd3478d8ed` |
| `src/main.c` | `0f8d8a9f177978c29007f1e354d6e6ce49e440d9561801de7ac6878f6704a9f9` |
| `src/linears_calls.c` | `a476e4ff10c5881b47decd234d04406fbc686774e29ae7575aed851dd034842f` |
| `src/filters_calls.c` | `b724e29a86769661a5eb09faca81a5e05eeb1b3c835fe98c3b4c51975f5dcacf` |
| `src/control_calls.c` | `8b2a79af828a57ad6f24de3a2df6c39d0fd865fed540103a8bf0b735f59e45fc` |
| `host_benchmark.c` | `35c2e4b182443b514a39283d0b7aba2048022739ddb5a124fec6ef9930fae66c` |
