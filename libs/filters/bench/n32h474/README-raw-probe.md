# Build the raw-conversion correctness probe

`raw_probe.c` includes the production private PF helpers and compares them with
native `scaled +/- 0.5` expressions. It checks 196,614 q15 values and 300,027 q31
values, including half-LSB neighbors and both full-scale endpoints. This is a
correctness image, not a timing benchmark.

Build from the SDK root with GNU Arm Embedded GCC. The following commands use
C99, the existing N32H474 startup/linker layout, and the true software-floating
ABI. The source rejects hardware/softfp builds and unrecognized IEEE layouts.
No SDK configuration or Xmake invocation is needed.

```sh
set -eu
out=$(mktemp -d /tmp/pf-raw-xhive-build-XXXXXX)

arm-none-eabi-gcc -std=c99 -O3 -fno-fast-math -Wall -Wextra -Werror \
    -mcpu=cortex-m4 -mthumb -mfloat-abi=soft \
    -ffunction-sections -fdata-sections \
    -Ilibs/filters/include -Ilibs/linears/include \
    -c libs/filters/bench/n32h474/raw_probe.c -o "$out/raw_probe.o"

arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=soft \
    -DBENCH_FORMAT=2 -c libs/filters/bench/n32h474/startup.S \
    -o "$out/startup.o"

arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=soft \
    -nostartfiles --specs=nosys.specs -Wl,--gc-sections \
    -Wl,-Map="$out/raw_probe.map" -Tlibs/filters/bench/n32h474/link.ld \
    "$out/startup.o" "$out/raw_probe.o" \
    -Wl,--start-group -lm -lc -lgcc -Wl,--end-group \
    -o "$out/raw_probe.elf"

arm-none-eabi-size "$out/raw_probe.elf"
arm-none-eabi-nm -S "$out/raw_probe.elf" | grep -E 'g_pf_raw_probe|pf_raw_probe_done'
sha256sum "$out/raw_probe.elf"
printf 'Build artifacts: %s\n' "$out"
```

`platform.c` is intentionally not linked: this probe does not configure PLL,
GPIO markers, UART, or measurement timers. The startup selects CPACR=0 through
`BENCH_FORMAT=2`; one image exercises both q15 and q31. The result structure and
completion symbol remain in the ELF for inspection by the hardware owner.

Some newlib distributions emit warnings about unused nosys syscall stubs before
linker garbage collection. The final image does not need those syscalls; inspect
its symbol table rather than treating an intermediate archive warning as a
runtime dependency. Rebuild and record a new ELF hash whenever the production
helper or probe source changes. These instructions only build the image; they do
not flash or run hardware.
