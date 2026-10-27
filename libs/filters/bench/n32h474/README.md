# N32H474 real-board UKF / PF / UPF benchmark

This fixture measures the actual filter transactions on an N32H474RC Cortex-M4.
It does not run motor-control firmware. The twelve SHRTIM-capable pins are
configured as ordinary GPIO timing markers, **not complementary PWM**. Disconnect
all power stages, actuators and other output drivers before using it; only the
logic analyzer may be attached to these pins.

The four images use identical 240 MHz HSI/PLL clock setup, GCC 14.2.1,
`-O3 -fno-fast-math -ffp-contract=off`, no LTO, and the same mathematical inputs:

| Image | Compiler ABI | Runtime CPACR CP10/11 | Arithmetic limitation |
| --- | --- | --- | --- |
| f32 | `hard`, `fpv4-sp-d16` | Enabled | Hardware single precision |
| f64 | `hard`, `fpv4-sp-d16` | Enabled | Double arithmetic is still software on this M4 |
| q15 | `soft` | Disabled | Integer Q work with required software-float boundaries |
| q31 | `soft` | Disabled | Integer Q work with required software-double boundaries |

`build.py` checks that the *entire linked* q15/q31 image contains no VFP
instructions, including the selected newlib/libgcc libraries. Runtime mailbox
and live SWD CPACR reads must agree. These Q implementations are mixed arithmetic,
not claims of completely integer callbacks, decompositions or exponentials.

## Workload and acceptance

Each image runs 20 cases: linear and quadratic callbacks, `n=m=2/4`, UKF,
PF with `N=32/128`, and UPF with `N=32/128`. The measured transactions are:

- UKF: predict + update.
- PF: predict + update + estimate.
- UPF: step (no extra resampling).

Each case restores the same initial state and draw position for two warmups and
nine measured transactions. Callback execution and copying precomputed Gaussian
draws are included. Allocation (none), initialization, restoration, output checks,
and GPIO writes are outside the DWT interval. Counter-read overhead is recorded,
not subtracted. See [the workload details](README-workload.md) for exact inputs,
RAM accounting and the driver-equivalence test.

A successful measurement requires every API result to be `OK`, finite outputs,
repeatable complete-output hashes, the exact 20-case matrix, the expected clock
and FPU mode, and no processor fault. A failed run is not a speed result. Hashes
and aggregate checksums are repeatability aids, not independent mathematical
proofs; retain the host numerical regressions and full element comparisons too.

## Build, flash, capture

Prerequisites: the installed ARM GCC/newlib tools, J-Link Commander, PXView with
its Python automation package, the powered N32H474RC, and confirmed safe wiring.
No tool downloads, privilege changes or Xmake configuration changes are performed.

```sh
# Builds only; the printed exclusive /tmp/*-xhive-* directory contains the ELF,
# immutable source snapshot, commands, hashes, map, ABI and disassembly evidence.
python3 libs/filters/bench/n32h474/build.py --format q15 --revision 3ee417a --label baseline

# Supply the actual printed ELF path and installed JLinkExe path.
# --flash explicitly authorizes overwriting the board application Flash.
python3 libs/filters/bench/n32h474/run.py /path/to/benchmark.elf \
  --jlink /path/to/JLinkExe --flash --prepare-only --output /path/to/prepare-artifacts

# Uses the currently active real PXView logic device; keep the exact same ELF.
# Arms all twelve channels, starts the board, and retains full packed waveforms.
python3 libs/filters/bench/n32h474/capture.py /path/to/benchmark.elf \
  --jlink /path/to/JLinkExe --seconds 28 --output /path/to/capture-artifacts
```

Repeat for `f32`, `f64`, `q15` and `q31`. Omit `--revision` only when intentionally
measuring the working-tree production snapshot. The current driver/platform are
snapshotted in either mode; compare their recorded hashes as well as production
hashes. Preserve the program/verify log alongside the matching ELF hash; merely
passing an ELF to the read-only runner does not identify the firmware on silicon.

Do not reuse an output directory for a different run. A raw Flash backup can be
retained before replacing an existing application, but restoration must be a
separate deliberate operation, not an implicit action of these scripts.

## Timing and analyzer mapping

Channels are zero-based and follow the existing board wiring:

| LA | Pin | Marker |
| --- | --- | --- |
| 0 / 1 / 2 / 3 | PA8 / PA9 / PA10 / PA11 | UKF f32 / f64 / q15 / q31 |
| 4 / 5 / 6 / 7 | PB12 / PB13 / PB14 / PB15 | PF f32 / f64 / q15 / q31 |
| 8 / 9 / 10 / 11 | PC8 / PC9 / PC6 / PC7 | UPF f32 / f64 / q15 / q31 |

A complete image generates 44 UKF, 88 PF and 88 UPF pulses, including warmups.
The capture reader validates every measured pulse against the corresponding DWT
sample, allowing 1 microsecond marker/sample uncertainty plus 2% for the separate
HSI/analyzer clocks. It also rejects any pulse at least one complete DWT wrap
period long (`2^32/240 MHz`, approximately 17.896 seconds).

Three measurement pitfalls were observed and must not silently produce results:

1. With this debugger connection sequence, DWT once remained stopped despite
   its enable bit reading as set. The direct triggering mechanism is not known.
   The runner therefore halts **before** measurement, rearms DEMCR/CYCCNT,
   writes the start command, resumes, and keeps the same J-Link connection open
   with no memory polling throughout a bounded quiet interval. A too-short
   interval can invalidate later samples when the connection closes; zero counts
   or incomplete execution are rejected. The default capture window is 28 s,
   with a 24 s debugger quiet interval. If the workload cannot finish within that
   bounded interval, do not accept the output or mask the problem by polling.
2. PXView's binary export returned only the first snapshot block in the observed
   session. The reader uses `get_samples` metadata and continues at
   `first_sample + sample_count` until every requested sample has been read. It
   validates the packed LSB-first encoding and records the complete waveform
   hash, not the hash of an unnoticed truncated first block.
3. An immediate debugger reconnect once read an all-zero mailbox before
   `main` had initialized it. After flash/reset/run, the runner now keeps the
   same connection for a bounded 250 ms startup interval. The first header
   read must still validate the ABI and ready phase; there is no retry loop
   that could hide an unfinished startup or a running measurement.

Every J-Link operation has a 30-second process timeout. No interactive debugger
or background debug server is left behind. The firmware waits for another
explicit mailbox command after completing; all marker pins are then low.

## Latest round: proposal cores and diagonal density paths

[`baseline-c334fdc.csv`](baseline-c334fdc.csv) and
[`pipeline.csv`](pipeline.csv) record the next 80-case before/after sweep.
Their [baseline](baseline-c334fdc.json) and [optimized](pipeline.json) JSON
companions retain source/ELF/compiler/library hashes, all nine cycle samples,
clock/FPU readbacks, memory counts and independently checked waveform data.
`pipeline.json` was measured from a working-tree snapshot. Its base
`revision` remains `c334fdc`; `source_commit` now identifies `48c1d53`, whose
87 recorded compilation-input hashes were checked against Git. The older
`current.*` records below are left unchanged.

All 80 scenarios became faster, with identical output hashes, binary64
checksums and public per-instance byte counts. Representative quadratic-model
results, `n=m=4`, `N=128` for PF/UPF, in **milliseconds**:

| Transaction | f32, FPU on | f64, FPU on | q15, FPU off | q31, FPU off |
| --- | ---: | ---: | ---: | ---: |
| UKF predict + update | 0.542 → 0.210 | 2.243 → 1.103 | 0.858 → 0.627 | 1.267 → 1.013 |
| PF predict + update + estimate | 3.535 → 1.383 | 21.263 → 10.055 | 4.870 → 4.752 | 9.087 → 8.969 |
| UPF step | 80.185 → 20.594 | 343.230 → 145.932 | 125.720 → 82.607 | 189.095 → 140.682 |

Across the 20 cases of each format, baseline/optimized time ratios are
2.327–6.867x (f32), 1.663–3.135x (f64), 1.020–1.578x (q15), and
1.010–1.380x (q31). At `n=m=4, N=128`, PF/UPF ratios are respectively
2.555/3.894x for f32 and 2.115/2.352x for f64. **The Q PF improvement is small**
in this round: 2.4% less time for q15 and 1.3% for q31; the larger
Q gains are in the proposal-heavy UPF.

Both sides still use the original workload's **diagonal Q and R**. The
floating density shortcut tests this property once per noise factor and
retains the original quotient and squared-norm arithmetic. Non-diagonal
factors still use the original pivoted LU; these timing ratios must not be
presented as measurements of correlated-noise workloads. The public
`gaussian_exponent` continues to accept general nonsingular dense factors.

The larger changes are shared by all covariance shapes: validated private
UKF cores avoid repeated descriptor/alias/workspace checks, small matrix
operations use direct loops, and each UPF step factors R once rather than
once per proposal. Callback order, callback-result validation, each distinct
covariance's positive-definiteness check and atomic state commits remain.
Q metadata computes exact quantization errors and finite magnitude comparisons
with guarded IEEE integer operations; its error budgets and integer MACs are
unchanged. Default Q PF moments keep each wide accumulator local across the
particle loop. Public storage macros have not been reduced, including the Q
UKF decomposition capacity that is still validated but no longer written.

The direct internal covariance staging also removes an existing failure for
`m > 2n+1`: the former nested public weighted-covariance call rejected the
UKF update's own basic scratch size. Exact-minimum guarded workspace tests
now cover that region for UKF and UPF without increasing the public macros.

### Numerical decision and validation

An unrestricted replacement of pivoted LU by triangular forward substitution
was **not accepted**. A 200,000-trial-per-format broad-exponent probe found
13 f32 cases where the old result met the original `2e-5` tolerance and the
new result did not, despite both returning `OK`. Exact rational arithmetic
confirmed all six exported examples. A `RANGE`-only fallback therefore did
not satisfy the accuracy requirement. The final shortcut is diagonal-only;
all other factors retain the old solve. The permanent cancellation regression
in `pf_pipeline.cc` locks this distinction.

Validation of the frozen final source:

- `xmake b` passed; the six new focused test entries passed;
  `xmake test -g xhive -v` passed **69 registered host entries**.
- Independent GCC `-O2`, Clang `-O2`, and Clang
  `-O1 -g -fsanitize=address,undefined,float-cast-overflow
  -fno-sanitize-recover=all -fno-omit-frame-pointer` builds each passed
  **661 GoogleTest cases**, with eight existing applicability skips.
  Production C was compiled as strict C99 with
  `-pedantic-errors -Wall -Wextra -Werror -Wvla`.
- `host_oracle.py` passed 80 cases, 46,704 full output elements and 42
  failure-publication checks. Every raw element also matched the `c334fdc`
  host baseline. This is driver/implementation equivalence, not a replacement
  for the long-double and Kalman-reference tests.
- A test-only wrapper called the actual final private diagonal path or the
  unchanged public solve on the same broad-exponent corpus: 103,791 valid
  f32 factors and 89,777 f64 factors, **zero status or raw-output differences**.
  This preserves legacy behavior; it is not a forward-error guarantee for
  ill-conditioned inputs.
- The exact Q quantization-error host probe made 5,996,845 comparisons with
  zero bit differences. The permanent `q_quantization.cc` tests also cover
  cell boundaries, adjacent values, all unit-range exponents, signed zeros,
  subnormals and finite magnitude comparisons.
- All 80 new baseline and 80 final board cases passed DWT/GPIO checks. The
  largest nonlinear UPF state was read in full after each final image:
  **2688 elements per format**, byte-identical to the previously measured
  baseline state. Host-versus-board floating libm differences remain recorded
  separately in JSON rather than being called exact matches.
- `python3 -B libs/filters/bench/n32h474/test_scripts.py -v` passed **32**
  offline tests, including bounded startup settling and rejection without
  polling when the mailbox is still uninitialized or running.

Whole-image static RAM and public per-instance RAM are unchanged. Flash
usage increases by 1128 / 1104 / 2016 / 2560 bytes for f32 / f64 / q15 / q31
respectively. The 96 KiB RAM / 256 KiB Flash layout and 8 KiB reserved stack
are unchanged; stack high-water usage was not measured.

**Q does not uniformly beat floating point.** With the much faster floating
paths, q15/FPU-off wins **0/20** pairs against f32/FPU-on; f32-time/q15-time is
0.249–0.472. q31/FPU-off wins **14/20** pairs against f64, with
f64-time/q31-time 0.867–1.133. Persistent Q arrays remain int16/int32 and the
Q statistical loops remain integer, but those facts alone do not establish
a speed advantage over the optimized floating implementation.

Complete artifacts are retained under `/tmp/pf-upf-xhive-round3-g0ml7l/` and
its indexed build directories. `final-builds.json`, `final-host/`, and
`final-run-commands.json` locate the immutable images, host checks and exact
measurement commands. The board was left in the final q31 image's ready
phase; an idle SWD read confirmed format q31, phase 1, command 0 and CPACR=0.
No PWM runs.

## Archived board results: first optimization round

[`baseline-3ee417a.csv`](baseline-3ee417a.csv) and
[`current.csv`](current.csv) contain all 80 matched scenarios before and after
these optimizations. Their [baseline](baseline-3ee417a.json) and
[current](current.json) JSON companions retain all nine cycle observations,
compiler/library/ELF/source hashes, clock/FPU readbacks, explicit RAM counts,
and waveform hashes with independent pulse measurements. `current.json` is a
frozen measurement, not a promise about later working trees: its `revision`
is the base commit and `source_mode=working-tree-copy`; the source hashes
identify the measured implementation. The same platform/workload sources and
compiler flags were used for both sides.

Every scenario became faster; no persistent array or workspace was widened:

| Format and FPU mode | Baseline median cycles / optimized median cycles, all 20 cases |
| --- | ---: |
| f32, FPU enabled | 1.277–1.993x |
| f64, FPU enabled (software double) | 1.395–1.776x |
| q15, FPU disabled | 1.445–3.462x |
| q31, FPU disabled | 1.395–2.401x |

Representative quadratic-model results, `n=m=4`, `N=128` for PF/UPF. Values
are baseline → optimized **milliseconds**, converted from DWT at nominal
240 MHz; the physical GPIO pulse measurements independently passed:

| Transaction | f32, FPU on | f64, FPU on | q15, FPU off | q31, FPU off |
| --- | ---: | ---: | ---: | ---: |
| UKF predict + update | 0.703 → 0.542 | 3.278 → 2.243 | 1.311 → 0.858 | 1.842 → 1.267 |
| PF predict + update + estimate | 5.969 → 3.535 | 33.027 → 21.263 | 15.522 → 4.870 | 20.599 → 9.087 |
| UPF step | 103.063 → 80.185 | 484.162 → 343.230 | 196.118 → 125.720 | 274.910 → 189.095 |

**Q15 without FPU does not yet consistently beat hardware f32.** It is faster
in 6/20 paired cases, not all 20; the floating-time/Q-time ratio is
0.631–1.156. Q31 without FPU is faster than the optimized f64 image in all
20 cases, by 1.741–2.488x. The historical desktop-host ratios must not replace
these board observations.

Explicit per-instance RAM for the same representative dimensions is unchanged
by this optimization round:

| Family | f32 bytes | f64 bytes | q15 bytes | q31 bytes |
| --- | ---: | ---: | ---: | ---: |
| UKF | 2264 | 4176 | 1104 | 2144 |
| PF | 13296 | 22432 | 3496 | 6672 |
| UPF | 24524 | 48680 | 12600 | 25120 |

Whole-image static RAM is also unchanged. The q31 image uses 968 additional
Flash bytes (75,868 → 76,836) for its expanded integer paths; the other three
images become smaller. These image figures include the benchmark, startup and
draw table, and are separate from the per-instance RAM table.

The optimizations remove redundant generic-matrix calls in floating mean,
covariance, perturbation and Gaussian dot work without changing reduction
order. Q paths use exact representation-guarded finite/range/conversion
helpers, omit known-zero integer products, and avoid per-offset mean-error
arithmetic only when a sufficient bound proves the original decision. The PF
raw conversion's integer shortcut is deliberately restricted to verified GNU
Arm EABI **no-FP** semantics; hardware-FP/softfp/other platforms retain the old
expression and floating-environment behavior. No tolerance, public capacity,
callback workload, extended intermediate domain or transaction commit rule
was relaxed.

### Validation tied to these records

- All 80 baseline and 80 optimized cases completed; every measured pulse was
  checked against its corresponding DWT interval. Every matched output hash,
  binary64 checksum and explicit byte count stayed identical.
- The largest nonlinear UPF state was also read in full from the board for
  each format: 2688 elements per format, 10,752 elements total, with identical
  raw bytes before/after. Float libm last-bit differences against the host are
  retained separately rather than described as exact cross-platform matches.
- The final dedicated [raw-conversion probe](README-raw-probe.md) ran on the
  board with CPACR=0: 196,614 q15 and 300,027 q31 comparisons, **zero
  mismatches** against the native GNU Arm software expression. The
  [result record](raw-probe-result.json) identifies the exact final ELF and
  probe source. This probe checks correctness, not latency.
- `xmake b` passed; the seven focused estimator test entries passed;
  `xmake test -g xhive` passed all 63 registered host entries.
- Independent GCC `-O2`, Clang `-O2`, and fully instrumented Clang
  `-O1 -g -fsanitize=address,undefined,float-cast-overflow
  -fno-omit-frame-pointer` builds each passed 142 GoogleTest cases with six
  existing format-applicability skips and no sanitizer diagnostics. All
  production C files were compiled as strict C99 with
  `-pedantic-errors -Wall -Wextra -Werror -Wvla`.
- `python3 -B libs/filters/bench/n32h474/test_scripts.py -v` passed all 30
  offline capture/parser tests; they cannot access hardware.

Complete ELF snapshots, raw packed waveforms, J-Link logs, host logs and the
original Flash backup from this run are retained under
`/tmp/filters-xhive-n32-session-urh53rwt/` and the build directories indexed
there. They are not embedded as large binary files in the repository. The
board was finally left in the optimized q31 benchmark's ready state, FPU
disabled, awaiting an explicit command; no PWM is running.

## Evidence boundaries

- DWT cycles and analyzer pulse widths here are **real-board observations**.
- Host GTest/oracle execution and ARM disassembly are separate evidence, not
  substitutes for those observations.
- Reported filter memory is persistent arrays + state descriptor + reusable
  explicit workspace + PF estimate outputs. It excludes stack, inputs, fixed
  draw tables, restoration copies and the measurement harness. The linker also
  reports the whole test-image static RAM and reserves 8 KiB for stack, but this
  is not a measured stack high-water mark or system peak RAM.
- No conclusion about all CPUs, arbitrary callbacks, larger dimensions or other
  clock/cache settings follows from this finite workload matrix.

Platform/startup and memory-layout details are in [README-platform.md](README-platform.md).
