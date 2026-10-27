# N32H474RC benchmark platform

This is a bare-metal benchmark build, not an SDK application or a hardware
control tool. `build.py` never opens J-Link/PXView, flashes a device, downloads a
toolchain, or touches Xmake configuration. Hardware operation is a separate,
explicitly authorized step owned by the benchmark operator.

## Build

The installed toolchain must provide `/usr/bin/arm-none-eabi-gcc`, `g++`,
`objdump`, `readelf`, `size`, `nm` and `objcopy`, including the matching newlib,
libm and libgcc multilibs. GCC 14.2.1 (20241119) has been checked locally for
both the hard and soft ABI. The script checks the tools/libraries and a newlib
header compile before building; missing dependencies are errors, not downloads.

From the SDK root (an absolute script path works from another directory):

```sh
# Immutable production baseline; current platform/workload driver.
python3 libs/filters/bench/n32h474/build.py --format f32 --revision 3ee417a --label baseline
python3 libs/filters/bench/n32h474/build.py --format f64 --revision 3ee417a --label baseline
python3 libs/filters/bench/n32h474/build.py --format q15 --revision 3ee417a --label baseline
python3 libs/filters/bench/n32h474/build.py --format q31 --revision 3ee417a --label baseline

# Current production working-tree snapshot.
python3 libs/filters/bench/n32h474/build.py --format q15 --label current

# Floating-point software-only image; pass --fpu off to run.py/capture.py too.
python3 libs/filters/bench/n32h474/build.py --format f32 --fpu off --label fpu-off

# Explicit platform-only boot/command smoke image. No filter cases or timings.
python3 libs/filters/bench/n32h474/build.py --format q31 --platform-only --label smoke
```

One image contains one workload format, selected with `-DBENCH_FORMAT=0` (f32),
`1` (f64), `2` (q15), or `3` (q31). Unreferenced production entry points are
removed by section garbage collection; no production sources are rewritten.

All translation units use `-mcpu=cortex-m4 -mthumb -O3 -fno-fast-math
-ffp-contract=off -fno-lto`, with function/data sections. There is no LTO.

| Workload / mode | Whole-program ABI / library selection | Runtime CPACR |
| --- | --- | --- |
| f32 / f64, `--fpu on` (default) | `-mfpu=fpv4-sp-d16 -mfloat-abi=hard` | `0x00f00000` |
| f32 / f64, `--fpu off` | `-mfloat-abi=soft` (no FPU option) | `0` |
| q15 / q31, `--fpu off` (default, on rejected) | `-mfloat-abi=soft` (no FPU option) | `0` |

`BENCH_FPU_ENABLED=0/1` selects CPACR independently of `BENCH_FORMAT`, in both
startup assembly and platform initialization. The build metadata records the
resolved `fpu_mode` and `float_abi`. `run.py` and `capture.py` require the same
explicit `--fpu off` for software-only f32/f64; they reject a mismatched CPACR
before starting, rather than inferring permission from the register readback.

The M4 FPU is single precision: an f64/FPU-on image uses the hard ABI, not
hardware double-precision arithmetic. Q kernels can still call their documented
local floating-point operations, but those operations execute through software
float helpers. The builder rejects **any FPU-off ELF** containing a VFP
instruction, including instructions linked from newlib/libgcc. At runtime NOCP
is caught as UsageFault or HardFault and reported in the mailbox. `platform.c`
additionally uses `-mgeneral-regs-only` so fault reporting itself cannot require
the FPU.

## Provenance and artifacts

Each invocation creates and retains a unique
`/tmp/filters-xhive-n32h474-<label>-<format>-*` directory. Compiler temporary
files also use that directory. No output directory override or shared build
cache exists.

- `source/`: all production headers/sources, platform/workload files and vendor
  CMSIS inputs actually used. `--revision` exports only production
  `libs/{filters,linears}/{include,src}` via `git archive`; the platform,
  workload and vendor CMSIS files are always copied from the current checkout.
  Without `--revision`, production inputs are also copied from the working tree.
- `commands.json`, `build.log`: exact commands and stdout/stderr, retained even
  after a failed build. Commands are non-interactive and contain no download,
  debugger or flashing step.
- `metadata.json`: revision/source mode, compiler and binutils versions, complete
  source SHA-256 set, explicit vendor system-file hash, selected library paths
  and hashes, flags, mailbox address/size, linked memory bounds, ELF hash, VFP
  instruction count and build success/failure. `portable_commands` uses the
  existing host benchmark's `${SOURCE}`/`${REPO}`/`${BUILD}` convention.
- `benchmark.elf`, `benchmark.bin`, `benchmark.map`: loadable image and link map.
- `benchmark.dis`, `benchmark.instructions.txt`: source-interleaved and plain
  disassembly of the final linked image.
- `benchmark.attributes.txt`, `benchmark.size.txt`, `benchmark.symbols.txt`:
  ELF attributes/sections/segments, section sizes and symbol addresses/sizes.
- `objects/`: objects, dependency files and GCC `.su` stack-usage reports.

The builder validates the frozen mailbox size (1728 bytes). ELF/map size and
static instruction checks are build evidence, not evidence of MCU execution.

## Board and startup contract

The linker uses Flash `0x08000000..0x0803ffff` (256 KiB) and RAM
`0x20000000..0x20017fff` (96 KiB). These are the existing N32H474RC demo's explicit
`CONFIG_FLASH_LENGTH=256` and `CONFIG_RAM_LENGTH=96` boundaries, not the vendor
family's larger default SRAM setting. The original application is read-only to
this build; its `.config` is not loaded or modified.

Eight KiB at the top of RAM is reserved for the descending stack. Linker
assertions reject static data crossing that boundary or Flash overflow. This
is a link-time reservation, not a measured stack high-water mark or runtime
stack guard. The `.stack` NOLOAD section is placed at the RAM top, so GNU ld's
region occupancy prints 100%; `metadata.json`'s `linked_memory` distinguishes
static data, the free gap and the explicit stack reserve. `size` includes the
8 KiB stack section in its BSS total. No heap or syscalls are supplied.

`startup.S` sets CPACR with DSB/ISB **before C/newlib constructors**, initializes
`.data` and `.bss`, and calls `main`. Its vector table covers all 118 external
N32H474 interrupt slots. Every non-reset exception reaches the integer-only
fault recorder; unused peripheral IRQs stay disabled.

`bench_platform_init()` calls the unmodified vendor
`system_n32h47x_48x.c` with `SYSCLK_SRC=2` and `SYSCLK_FREQ=240000000`. The reviewed
vendor path uses:

- 8 MHz HSI, PLL multiplier 30, HCLK divide by 1, APB1/APB2 divide by 2.
- Flash latency **5** before selecting PLL; instruction cache on and prefetch
  off. The vendor rule is one wait state per 40 MHz step up to 240 MHz.
- PWR clock enabled, NRST digital-filter setting `0x90`, PVD level 1.78 V,
  polling until the PVD status clears, then a 1 ms DWT delay. The PVD threshold
  is not a software-configured core supply voltage.

The clock routine polls oscillator/PLL/PVD readiness without a timeout; if
hardware never becomes ready, `phase=1` is never published. This matches the
vendor startup path. `SystemCoreClockUpdate()` reads back the clock registers
before publishing `core_hz`; its HSI reference is still the nominal 8 MHz, not
an external frequency measurement. No independent supply-range or oscillator
accuracy measurement is performed by this software.

The platform disables SysTick, clears its pending interrupt and all peripheral
NVIC enables/pending bits, and enables synchronous fault handling. It enables
no UART, RTOS, SHRTIM, PWM, DMA or peripheral interrupts. A device reset is
required before running this image; jumping into it from a live application is
not supported.

## Markers, timing and mailbox

Only these twelve logic-analyzer-connected pins become ordinary 2 mA
push-pull GPIO outputs, initially low, with no pull resistor and fast slew:

| LA | Pin | LA | Pin | LA | Pin |
| --- | --- | --- | --- | --- | --- |
| 0 | PA8 | 4 | PB12 | 8 | PC8 |
| 1 | PA9 | 5 | PB13 | 9 | PC9 |
| 2 | PA10 | 6 | PB14 | 10 | PC6 |
| 3 | PA11 | 7 | PB15 | 11 | PC7 |

This is the mapping in the existing demo's `src/shrtim_test/main.c` and
`tools/verify_shrtim.py`; no SHRTIM alternate function is used. The authorized
load is the logic analyzer only, not a motor/power stage. PA13/PA14 debug pins
are unchanged. `bench_marker()` uses the atomic set/clear registers and a DSB;
the workload owns the meaning of each marker.

The platform enables DWT CYCCNT at initialization. The measurement operator
also rearms it before starting because this rig's debugger sequence has been
observed to leave it stopped; see [the timing workflow](README.md#timing-and-analyzer-mapping).
`bench_cycles()` is a non-inlined function; the workload measures its call/read
overhead and does not subtract it. Unsigned subtraction handles one 32-bit wrap; an individual
measurement must span fewer than 2^32 cycles (about 17.90 s at nominal 240 MHz).
Debugger halts and SWD traffic inside timed regions are outside this builder's
control and must be excluded by the operator.

After initialization, `main` publishes magic/version/format, the read-back
`core_hz`, CPACR and `phase=1`. It polls `g_bench_results.command`; a value of 1
is cleared before `phase=2` and `bench_run()`. The workload publishes its own
terminal phase. Later commands can rerun the workload without a reset. The
platform-only image instead reports `phase=3, case_count=0` and is never a
performance result.

On a CPU fault the recorder preserves CFSR, HFSR and CPACR, writes `phase=4`,
and spins without returning. The active case and partial records remain
available; reset is required after such a fault. It does not clear the hardware
fault status or attempt to continue a failed instruction.
