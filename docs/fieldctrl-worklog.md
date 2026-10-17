# linears / filters / fieldctrl implementation worklog

## Scope and evidence rules

This log records the design decisions, milestone boundaries, actual checks, and
remaining limitations for the `dev/fieldctrl` worktree. The initial revision
reported by the coordinating agent is `b62d2ae`. The coordinating agent alone
reviews, stages, and commits each milestone; implementation agents do not stage
or commit. All task agents use GPT-6 Astra with maximum reasoning.

The authorized validation boundary is host software tests and ARM
cross-compilation/linking. Flashing a board, running a motor, opening a hardware
control connection, and claiming physical motor validation are out of scope.
A passing software plant simulation is not a board or motor result.

Evidence labels used below:

- **Confirmed:** observed source, command output, or an explicitly agreed
  requirement. A delegated check identifies its reporting owner.
- **Inferred:** a conclusion derived from identified evidence, not a new test.
- **Unverified:** planned work or behavior not yet exercised.

## Accepted requirements

- Add independent `libs/linears`, `libs/filters`, and `libs/fieldctrl` libraries.
- Expose all requested algorithms in `f32`, `f64`, `q15`, and `q31` forms.
- Production code is C99; host tests are C++17 and use GoogleTest. Eigen is a
  host-test numerical reference only, never a production or embedded dependency.
- No dynamic allocation, variable-length arrays, `alloca`, recursive
  dimension-dependent stack storage, or mutable hidden singleton state.
  Dimension-dependent storage belongs to the caller and has documented capacity
  requirements and workspace-size macros. Fixed-size local scalars/vectors are
  allowed.
- Fixed-point implementations use integer arithmetic, not floating-point
  wrappers. Scaling, rounding, overflow, invalid input, aliasing, partial output,
  and state lifetime must be explicit in the public contracts.
- All public headers carry adjacent English Doxygen contracts. Algorithms must
  cite their published basis and document actual applicability limits.
- Tests are split by algorithm family and cover numerical references, invalid
  input, insufficient workspace, sentinels, aliasing, repeatability/reentrancy,
  and the relevant edge cases. Tolerances must follow quantization and numerical
  conditioning rather than conceal implementation defects.

### Algorithm coverage

| Library | Required algorithms |
| --- | --- |
| `linears` | Matrix add/subtract/multiply, matrix-vector multiply, transpose, weighted sums/scaling/copy/zero/identity, dot/norm/trace/elementwise operations; pivoted LU, solve, inverse, determinant, Cholesky, rectangular Householder QR, symmetric-indefinite LDLT, rectangular/rank-deficient SVD |
| `filters` | FIR, cascaded second-order sections, first-order low/high pass, moving average, EMA, median, complementary filter, scalar Kalman, linear matrix Kalman |
| `fieldctrl` | Clarke/Park and inverse transforms, limited anti-windup PI, voltage-vector limiting and SVPWM; VVVF; FOC with SMO; DTC flux/torque estimation, hysteresis and switching table |

The user's “determinant decomposition” requirement is interpreted as pivoted LU
plus determinant evaluation, not a separate unnamed factorization. This was
explicitly accepted before implementation.

## Design decisions

### Shared numerical contracts

- One matrix descriptor contains `data`, `rows`, `cols`, and `exponent`.
  Floating-point operations ignore `exponent`; fixed-point matrices use one
  binary block exponent.
- A q15/q31 scalar carries its own exponent and represents
  `mantissa * 2^(exponent - fractional_bits)`, with 15 or 31 fractional bits.
- `linears` owns the shared integer scalar arithmetic. Decompositions, filters,
  and control algorithms reuse that implementation instead of introducing
  separate arithmetic conventions. The numerical-contract owner froze exponent
  range `[-16384, 16384]` and a normalized q31 working scalar with nonzero
  magnitude in `[2^30, 2^31)`; zero is `{0, 0}`. Each primitive rounds to nearest,
  ties away from zero, does not saturate, and leaves its output unchanged on
  failure. Multiplication/division/restoring square root use `int64_t`/`uint64_t`
  without floating point or `__int128`.
- Normalized primitive rounding error is at most half a mantissa ULP (relative
  error at most `2^-31`). Block quantization shares the largest exponent, so
  sufficiently small entries may round to zero. Long reductions round each
  arithmetic step and are not exact dot products.
- Each stateful filter/controller is caller-owned and independently reusable.
  Matrix Kalman uses a solve rather than an explicit inverse and a Joseph-form
  covariance update; implementation and numerical tests remain unverified.
- SMO does not establish zero-speed observability or standstill self-start.
  Hardware protections, motor parameters, sample timing, and physical tuning
  remain application responsibilities and are not validated by this task.

### Build and selection

- Each new library uses `include/` for public headers and `src/` for production
  code. Production targets remain ordinary SDK library targets.
- `filters` publicly depends on `linears`; `fieldctrl` depends on both.
- Kconfig `LIBS_ADD_FILTERS` selects `LIBS_ADD_LINEARS`;
  `LIBS_ADD_FIELDCTRL` selects both dependencies. All are opt-in for applications.
- The standalone SDK default host-library build includes all three new libraries.
- Each library's own `xmake.lua` auto-discovers `test/**.cc`, excluding only
  `test/main.cc`. Each file is a separate non-default test binary, linked with
  the shared test entry and registered in group `xhive` with a 120-second
  timeout and realtime output disabled.
- SDK tests are registered only in `xhive_host` when `xhive_tests` is enabled;
  no embedded test copies or Eigen/GoogleTest package dependencies are allowed.
  Each library also supports a standalone host-test entry.
- Pin Eigen to a supported 3.4.x package release. Download and checksum
  verification must use Xmake's normal package manager. No system Eigen include
  directory or preinstalled package is assumed.
- The build/worklog agent serializes all SDK-root Xmake commands. Algorithm
  agents may use isolated temporary compiler checks, but do not compete for the
  SDK project configuration or build lock.

### ARM validation application

The authorized application directory is
`/home/zmmfly/prjs/xhive_testspace/fieldctrl`. The existing
`/home/zmmfly/prjs/xhive_testspace/n32h474_demo` is a read-only reference.
The agreed target is N32H474 / Cortex-M4, single-precision FPU, ARM GCC, no RTOS,
SDK startup/linker defaults, 256 KiB flash and 96 KiB RAM. The existing toolchain
is used without installing another cross-compiler.

Every application Xmake command runs from the new application directory with
`XHIVE_SDK_PATH` set to this worktree and `XMAKE_CONFIGDIR` set to the application
directory. The SDK host cache must not be redirected to the application.
The link fixture will exercise real APIs for all four variants and preserve
symbols against section garbage collection. Heap-symbol inspection and compiler
stack-usage diagnostics are required; measured per-function stack bytes do not
by themselves prove a complete runtime call-chain stack bound. Final host
validation also includes ASan/UBSan and `-O2` strict-aliasing checks in isolated
build/configuration directories, without replacing the SDK's normal host cache.

## Milestones and review gates

| ID | Commit-sized scope | Required review/validation | State |
| --- | --- | --- | --- |
| M0 | Requirements, design, validation plan, and this worklog | Review scope and accepted decisions; whitespace check | Committed as `63e7132` |
| M1a | Shared numerical types/core and linears build entry | Contract/quantization tests, SDK graph regression, full host group, strict C99 | Committed as `37d5961` |
| M1b | Foundation matrix/vector operations | Four-variant Eigen references, contract/audit regressions, native-f32 ARM evidence | Committed as `b5ffaf2` |
| M2 | LU/solve/inverse/determinant, Cholesky, QR, LDLT, SVD | Eight family entries: 258 cases passed, 2 floating cases inapplicable; host build | Committed as `c9a14da`; independent audit `70e9d2c` |
| M3 | Filters including scalar and matrix Kalman | 10 entries / 201 cases, Eigen Kalman, strict GCC/Clang C99, full 42-entry SDK group | Committed as `70be7fa` |
| M4 | Control foundations, PI/PWM, VVVF | 3 entries / 82 cases; default build and SDK graph isolation | Reviewed implementation and integration ready |
| M5 | FOC and SMO | 2 entries / 74 cases, including software observer/handover scenarios | Tests passed; separate commit pending |
| M6 | DTC | 1 entry / 60 cases, including subnormal sector regression | Tests passed; separate commit pending |
| M7 | Complete SDK integration and final regression | Full host group, default build, standalone entries, isolated ARM compile/link, heap/stack audit | Planned |

The coordinating agent may split a milestone into smaller coherent commits.
No milestone is considered complete merely because files exist or a test selector
prints `nothing to test`. The worklog is updated before each commit with the
actual commands and results available at that point.

## Validation ledger

### 2026-10-17 — initial plan

**Confirmed:** inspected `xmake.lua`, `libs/xmake.lua`, `libs/Kconfig`, and
`libs/base64/xmake.lua`. The SDK has separate `xhive_host` and `xhive_embed`
namespaces; its standalone default `libs` target selects mapped portable
libraries. Existing base64 tests demonstrate per-file discovery, namespace
filtering, group `xhive`, and the 120-second timeout.

**Confirmed:** loaded the Xmake target, package, test, cross-compilation,
introspection, command, and style guidance before modifying build configuration.
The repository-specific test registration and CLI rules take precedence where
more restrictive (notably options before selectors with Xmake 3.0.8).

**Confirmed (numerical-contract owner report):** public numerical headers were
created at `libs/linears/include/linears.h` and `linears_numeric.h`; the integer
working-scalar conventions above are frozen. Foundation operations stage results in caller-provided typed workspace,
permitting data overlap while preserving output on failure. The frozen workspace
parameters are `linears_workspace_unit_t *workspace` plus a byte capacity;
implementations access each union array element's member rather than casting a
union array to a flat scalar array. The numerical owner reports a strict C99,
`-O2 -fstrict-aliasing`, UBSan check for the numerical core; foundation operators
have not yet been validated.

**Confirmed:** the authorized ARM fixture directory was inspected and is empty.
The reference demo was listed read-only; no reference files were changed.

**Unverified:** baseline host configuration/build/tests, Eigen installation,
new-library registration, algorithm correctness, standalone library tests,
ARM compilation/linking, retained API symbols, and heap/stack checks have not yet
run. No board or motor operation has been performed.

## Reproducible command plan (not yet results)

SDK root:

```sh
xmake f -P . -y -m debug --xhive_tests=y --xhive_cross_tests=n
xmake b
xmake test -g xhive -v
```

During iteration, use precise selectors with `-g`/`-v` before the selector,
for example `xmake test -g xhive -v 'linears_test_*/*'`. Exact registered entry
counts and GoogleTest case counts are recorded separately when known.

The ARM command sequence and application source/configuration are recorded once
the public APIs are stable and the real fixture has been constructed.

### M0 review checks

- `git status --short`: inspected; only this worklog and the three new library
  trees were untracked at this checkpoint. Library-tree changes belong to the
  implementation agents and are not part of M0.
- `git diff --check`: passed for tracked changes at this checkpoint.
- `git diff --no-index --check /dev/null docs/fieldctrl-worklog.md`: passed;
  this explicitly covers the new untracked document.
- Build/runtime checks: not run for M0 (documentation-only milestone).
- Coordinating agent reviewed and committed M0 as `63e7132`
  (`docs(fieldctrl): record library plan and validation milestones`).

### 2026-10-17 — host baseline before build integration

**Confirmed:** Xmake `3.0.8+dev.fe5d480`, GCC/G++ `15.2.0`, Python `3.14.4` on
Linux x86-64. The following commands completed successfully before new-library
registration:

```sh
xmake f -P . -y -m debug --xhive_tests=y --xhive_cross_tests=n
xmake b
xmake test -g xhive -v
```

The full group passed **17 Xmake test entries**, not 17 individual GoogleTest
cases. Existing test output included an AddressSanitizer huge-allocation warning
(`failed to allocate 0x1000000000000000 bytes`); the existing tests and process
returned success. This is a baseline observation, not a new-library sanitizer
qualification.

### 2026-10-17 — M1a numerical core and linears build integration

**Confirmed:** only `linears` was added to the SDK library map and Kconfig for
this milestone. The new `filters/xmake.lua` and `fieldctrl/xmake.lua` were drafted
but are not included by the SDK root yet and are not part of the M1a stage list.
This avoids compiling incomplete downstream algorithms during core acceptance.

Eigen **3.4.0** downloaded from the upstream archive and installed successfully
through Xmake's normal package manager. Its include paths appear on the test
compile commands, not on the production `linears` target. SDK graph regression
confirms a production target with no packages, no embedded tests/requires, no
test packages when tests are disabled, standalone per-file tests, and C++17/Eigen
on the new tests.

Executed successfully:

```sh
xmake f -P . -y -m debug --xhive_tests=y --xhive_cross_tests=n
xmake b
xmake test -g xhive -v 'linears_test_numeric_contract/numeric_contract' 'xhive_test_sdk_entry/sdk_entry'
cc -std=c99 -pedantic-errors -Wall -Wextra -Werror -Ilibs/linears/include -fsyntax-only libs/linears/src/numeric.c
xmake test -g xhive -v
```

- Precise run: **2 Xmake test entries**, including **7 numerical-contract
  GoogleTest cases**, passed.
- Full group: **18 Xmake test entries**, zero failed; retained command log:
  `/tmp/xhive-fieldctrl-M1a-host-gmVvj2.log`.
- The coordinator's first strict-C99 review rejected four `_Alignof` expressions
  (C11-only). The numerical owner replaced them with named `offsetof` alignment
  probes; the strict-C99 command above passed after that fix.
- **Confirmed by numerical-owner report:** isolated strict C99,
  `-O2 -fstrict-aliasing`, `-Wall -Wextra -Werror`, UBSan compilation and execution
  passed the same 7 tests in `/tmp/xhive-linears-numeric-VDXiOr`. This is a
  delegated check; the build/worklog agent independently ran the commands above.
- **Unverified:** foundation operators, decompositions, downstream filters and
  control, ARM linking, full three-library sanitizer/stack/heap coverage.

M1a integration stage paths are `libs/linears/xmake.lua`, `libs/xmake.lua`,
`libs/Kconfig`, `tests/sdk_entry.lua`, and this worklog. The numerical owner's
five frozen implementation/header/test paths are reviewed separately by the
coordinator. No broad `git add` operation is authorized for the other in-progress
library trees. `git diff --check` passed; the untracked
`libs/linears/xmake.lua` was also checked with
`git diff --no-index --check /dev/null libs/linears/xmake.lua` and produced no
whitespace diagnostics (exit 1 denotes the new-file difference).
The coordinator reviewed and committed M1a as `37d5961`
(`feat(linears): add static fixed-point numeric foundation`).

### 2026-10-17 — downstream decisions before implementation acceptance

The following are **confirmed design reports from the named implementation
owners**, not claims that their pending algorithms already pass tests.

#### Foundation operations (numerical owner)

- Raw copy preserves input bits and exponent with `memmove`; unlike computed
  fixed-point results, it does not requantize or renormalize an existing block.
  Computed results normalize their block exponent; zero uses exponent 0 and
  identity uses exponent 1.
- f32 uses native float arithmetic. A double workspace member is only an exact
  storage container for a float result, not a hidden f64 computation.
- Norm uses scaled sum-of-squares to avoid premature overflow. Fixed-point norm
  scales by the maximum magnitude. An initial exponent-gap omission branch was
  later identified as unreachable for a single q15/q31 block and removed during
  review; see the foundation follow-up below. It is not a current algorithmic
  approximation option.

#### Decompositions (decomposition owner)

- Typed decomposition workspaces use `float`, `double`, or `linears_fixed_t`
  arrays and element capacities. This deliberately differs from the foundation
  API's union-unit pointer plus byte capacity.
- No arbitrary 64-dimension or 128-sweep compile-time limit is retained.
  Dimensions are bounded by checked size arithmetic and caller capacity;
  Jacobi SVD is bounded by the caller's finite `max_sweeps` budget. The public
  dimension-parameterized workspace macros give the required maximum storage.
- LU separates L and U so a q block does not mix differently scaled factors.
  Solve supports multiple right-hand sides without constructing an inverse;
  determinant is a 1x1 matrix so it carries a fixed-point exponent.
- Householder QR returns full `m*m` Q and `m*n` R for tall, square, or wide
  matrices, including rank deficiency. Cholesky requires symmetry and positive
  computed pivots. Output matrices are failure-atomic; permutation/block
  metadata may be partially updated on a numerical failure as documented.
- LDLT follows symmetric complete pivoting with 1x1/2x2 blocks (Bunch–Parlett /
  Higham references in the library documentation). A shared q D exponent may
  lose tiny pivots; success is not a condition-number guarantee.
- SVD uses scaled cyclic one-sided Jacobi, transposes wide problems, returns
  thin U/S/V, and completes numerical-zero columns by two-pass MGS. The owner
  documents its zero threshold and does not claim LAPACK-grade relative
  accuracy for the smallest singular values.
- **Delegated check:** shared four-variant adapter compiled using
  `cc -std=c99 -Wall -Wextra -Werror -pedantic` in
  `/tmp/xhive-decomp-compile-qtpbJy`; this does not yet validate factorization
  results.

#### Filters (filter owner and Kalman design documents)

- Low-pass/EMA uses caller-supplied dimensionless alpha in `[0,1]`; high-pass
  uses `alpha*(previous_output + input - previous_input)`. No hidden sample-time
  conversion or coefficient design is performed.
- FIR coefficient order is increasing delay; normalized SOS uses transposed
  direct form II and caller-owned tentative state. Moving average recomputes
  its zero-padded finite window rather than retaining a drifting sum. Median
  uses caller insertion-sort scratch and averages the two central elements for
  an even window. Complementary filtering does not wrap angles.
- A failed filter step preserves persistent state and output; scratch is
  unspecified. Context/buffer ownership and serialization are caller contracts.
- Kalman separates reset/predict/update, uses Joseph covariance in both scalar
  and matrix forms, solves innovation equations without an inverse, and
  symmetrizes stored covariance. Symmetrization is not PSD projection.
  State/process PSD is a caller precondition; symmetry/diagonal signs and
  measurement/innovation SPD are checked as documented. No automatic jitter or
  regularization is added.

#### Control (control owner)

- Angles are electrical turns in `[0,1)`; electrical frequency is signed Hz.
  Mechanical Hz converts through pole-pair count. Clarke is amplitude-invariant;
  Park uses `d=alpha*cos+beta*sin`, `q=-alpha*sin+beta*cos`.
- Fixed-point trigonometry uses a bounded 32-step CORDIC on the shared integer
  scalar core. q15 and q31 internal integration/phase/prediction/flux state
  retains normalized q31 working precision; only public outputs quantize to
  q15. This prevents small per-sample increments disappearing in q15 state.
- PI uses conditional integration; vector limiting is radial; SVPWM subtracts
  max/min common mode. VVVF accepts explicit signed frequency and amplitude.
- FOC rejects a joint integration increment when its dot product with the
  saturation residual is positive and recomputes output. SMO uses the existing
  low-pass filter (q31 internally even for q15), declared direction and discrete
  low-pass phase compensation. Lock requires observable back-EMF/speed/current
  conditions; standstill self-start is not claimed.
- DTC uses actually applied voltage, flux/torque hysteresis, six-sector active
  vectors, and chooses 000/111 zero state by the fewest transitions from the
  previous switching state. Initial flux bias and low-speed integrator drift
  are documented limitations.
- Published references include Park (1929), Volder (1959), Åström/Hägglund
  (1995), and van der Broeck (1988); exact citations are maintained in each
  library's algorithm documentation.

#### Build fixture preparation

Created `.config` and `xmake.lua` only in the authorized external application
folder. The fixture selects the three libraries, disables LTO, chooses the
existing Cortex-M4 single-FPU configuration, enables `-fstack-usage`, rejects VLA
and `alloca`, and requests a linker map. Its real API-use source is pending stable
headers. **No cross configure/build or hardware action has run yet.**

### 2026-10-17 — independent fixed-point audit test node

The independent test owner added `libs/linears/test/audit_fixed_numeric.cc`
without changing production code. The following SDK command passed:

```sh
xmake test -g xhive -v 'linears_test_audit_fixed_numeric/audit_fixed_numeric'
```

**Confirmed:** one Xmake entry, **36 GoogleTest cases in 11 suites**, all passed
(44 ms GoogleTest runtime). Full output is retained at
`/tmp/xhive-fieldctrl-audit-fixed-moKIhe.log`. The existing linears sources,
including newly completed decomposition wrappers, compiled/linked for this run;
the selected test exercises the numeric API, not decomposition correctness.

Coverage includes all 65,536 q15 mantissas at three exponents (196,608 exact
round trips), tie-away rounding, exponent gaps 31/32/33, cancellation,
`INT64_MIN`, 4,000 multiply/divide references, 5,015 square-root references,
500 mixed-exponent block exports, output atomicity, alias/alignment rejection,
size overflow, and static workspace macros.

**Delegated independent-owner check:** the same 36 cases passed an isolated
C99/C++17 `-O2 -fstrict-aliasing -fsanitize=address,undefined
-fno-sanitize-recover=all` build in 37 ms; XML is retained at
`/tmp/xhive-audit-fixed-aVvjAd/results.xml`. No production numeric defect was
reported by that audit.

**Platform limitation:** this test oracle explicitly requires binary
`long double` with at least 64 significand bits. Linux x86-64 meets that
requirement. A host where `long double` equals 53-bit double (including the
usual MSVC configuration) fails the oracle's compile-time assertion rather than
silently skipping or weakening the audit. Other host platforms are unverified.

Additional **delegated independent-owner evidence** on GCC/G++ 15.2.0:

```sh
cc -std=c99 -pedantic-errors -O2 -g -Wall -Wextra -Werror -Werror=vla -Werror=alloca -Wframe-larger-than=1024 -fstack-usage -fstrict-aliasing -Ilibs/linears/include -c libs/linears/src/numeric.c -o /tmp/xhive-audit-fixed-aVvjAd/numeric_plain.o
```

A plain C++17/O2/strict-aliasing audit binary also passed 36/36 cases (15 ms).
Both XML reports contain 36 tests, zero failures/errors/disabled tests.
Non-instrumented numerical-core `.su` entries are all static; the largest
reported single-function frame is `export_block`, **112 bytes** on this host.
This is not an ARM or whole-call-chain bound. `nm -u` reports only
`__stack_chk_fail`, no heap/float helper; object `.data` and `.bss` are zero.
The owner used cached GoogleTest after `pkg-config gtest` failed; no package was
downloaded by that node. Numeric source SHA-256 for that measurement:
`a0fadd4e69fdca0e0ef927c6de8a29d709acdb0a1b51df7f8b2d5704a2aae270`.
The coordinator committed this independent test node as `207eeca`
(`test(linears): audit fixed-point rounding and boundary contracts`).

### 2026-10-17 — foundation test baseline and early ARM object audit

Before the f32 optimization described below, these commands passed:

```sh
xmake b
xmake test -g xhive -v 'linears_test_basic_elementwise/basic_elementwise' 'linears_test_basic_products/basic_products' 'linears_test_basic_reductions/basic_reductions' 'linears_test_basic_contract/basic_contract'
```

**Confirmed:** 4 Xmake entries, **61 GoogleTest cases** (20 elementwise,
12 products, 10 reductions, 19 contracts). Full log:
`/tmp/xhive-fieldctrl-basic-rzX6xG.log`. The numerical owner separately reported
the same 61 cases passing strict C99, O2, strict-aliasing and UBSan in an isolated
build against Eigen and the actual quantized inputs. Foundation acceptance is
held for the ARM f32 implementation improvement and rerun, not represented as
final by this initial pass.

Using the already installed **Arm GNU Toolchain 14.3.Rel1 / GCC 14.3.1**, the
build agent compiled all seven then-present `linears/src/*.c` files to isolated
ARM objects. Each compile used:

```sh
arm-none-eabi-gcc -std=c99 -pedantic-errors -O2 -fstrict-aliasing -Wall -Wextra -Werror -Wvla -Werror=vla -Walloca -Werror=alloca -fstack-usage -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -I"$sdk/libs/linears/include" -c "$source" -o "$object"
```

**Confirmed:** 7 objects, 136 stack-usage entries, all classified `static`.
Largest measured single-function frame: `linears_svd_f64`, **672 bytes**.
Undefined-symbol inspection found no `malloc`, `calloc`, `realloc`, `free`,
`alloca`, or `sbrk`/`_sbrk`. Objects, `.su`, source SHA-256 manifest, undefined
symbols and disassembly are in
`/home/zmmfly/prjs/xhive_testspace/fieldctrl/arm-linears-peSzBF`.
This was object compilation, not an application link or hardware run; a
single-frame maximum is not a call-chain stack bound.

#### Confirmed ARM f32 performance issue and requested fix

The coordinator requested instruction-level evidence for the original shared
double-container foundation kernel. ARM `objdump -dr basic.o` confirmed that
f32 results were numerically rounded in float, but each element still incurred
software double conversion/comparison helpers on the single-FPU Cortex-M4:

- `input_values` f32 branch: `0x444 __aeabi_f2d`, `0x454 __aeabi_dcmpun`,
  `0x462 __aeabi_dcmpgt` inside the element loop.
- `commit_result` f32 loop: `0x3a8 __aeabi_d2f` for every stored element.
- f32 reduction loop: `0x8fe __aeabi_d2f`, native `vadd.f32` at `0x906`, then
  `0x90e __aeabi_f2d` and double finite comparisons.

This concrete overhead was sent to the numerical owner for a focused native-f32
kernel/workspace-member change. The q input-validation branch (`kind>1`) instead
calls `linears_fixed_from_i64`; source branches in `matrix_compute` and
`reduction` route q computation only through integer helpers, not through the
floating evaluators. Presence of f64 helpers somewhere in a mixed-format object
is not evidence that q operations execute them. Final post-fix ARM and host
results remain pending.

### 2026-10-17 — initial filters and standalone control checks

After the filter owner confirmed that all filter/Kalman source files were
complete enough to compile, added only filters to the SDK map and Kconfig.
The filters standalone entry imports linears production code but does not
register linears tests. SDK graph assertions check that exact isolation,
public dependency visibility, C++17/Eigen tests and production-package absence.

```sh
xmake f -P . -y -m debug --xhive_tests=y --xhive_cross_tests=n
xmake b
xmake test -g xhive -v 'filters_test_lowpass/lowpass' 'filters_test_fir/fir' 'filters_test_biquad/biquad' 'filters_test_highpass/highpass' 'xhive_test_sdk_entry/sdk_entry'
```

**Confirmed:** all 5 Xmake entries passed, with **72 filter GoogleTest cases**
(20 low-pass, 20 FIR, 16 biquad, 16 high-pass). Log:
`/tmp/xhive-fieldctrl-filters-initial-WIQfyE.log`. Remaining filter classes and
Kalman tests have not been accepted by this partial run.

Before registering control in the SDK root, validated its real standalone
library entry with linears/filters production dependencies and an isolated
external cache/build directory. No source stub or fake dependency was used.
Commands ran from the new external work directory:

```sh
XMAKE_CONFIGDIR="$work" xmake f -P "$sdk/libs/fieldctrl" -y -m debug -o "$work/build"
XMAKE_CONFIGDIR="$work" xmake test -P "$sdk/libs/fieldctrl" -g xhive -v 'fieldctrl_test_transforms/transforms' 'fieldctrl_test_pi_pwm/pi_pwm' 'fieldctrl_test_vvvf/vvvf'
```

**Confirmed:** 3 Xmake entries, **64 GoogleTest cases** (28 transforms,
20 PI/PWM, 16 VVVF), all passed. Complete output is in
`/home/zmmfly/prjs/xhive_testspace/fieldctrl/host-fieldctrl-base-wmKQDl/build-test.log`.
This validates a host standalone entry and software formulas, not an ARM
application or a motor. FOC/SMO/DTC runtime checks are not covered by this run.

The control owner also reported a temporary C99/C++17 O2 ASan/UBSan build passing
all 64 cases. Two changes were made before that acceptance: f32 trigonometry
reduces to the nearest quarter turn before multiplying its small residual by
2*pi, removing an observed roughly 3e-6 VVVF voltage error without widening
tolerance; the f64 independent reference was changed from double to long double
to remove approximately 5e-15 reference-side rounding. An initial temporary
UBSan-only link failed because cached GoogleTest required ASan symbols; linking
with both ASan and UBSan fixed the validation setup.

### 2026-10-17 — M1b post-fix native-f32 acceptance

The numerical owner replaced the shared double-container float path with a
small twice-instantiated native float/double evaluator/reducer and added a
`float` workspace member. Shape, alias, integer and failure-atomicity logic
remain shared. This supersedes the earlier double-container design note.
The overflow-size macro's zero divisor in an unevaluated expression was also
made explicitly nonzero to avoid strict compiler diagnostics for `(0,0)`;
constant-expression zero/overflow tests were retained.

Executed successfully after those changes:

```sh
xmake b
xmake test -g xhive -v 'linears_test_basic_elementwise/basic_elementwise' 'linears_test_basic_products/basic_products' 'linears_test_basic_reductions/basic_reductions' 'linears_test_basic_contract/basic_contract' 'linears_test_numeric_contract/numeric_contract' 'linears_test_audit_fixed_numeric/audit_fixed_numeric'
```

**Confirmed:** 6 Xmake entries, **104 GoogleTest cases** (61 foundation + 7
numeric contracts + 36 independent audit), all passed. Log:
`/tmp/xhive-fieldctrl-basic-native-f32-K77183.log`.

Recompiled `basic.c` using the exact strict C99/O2/Cortex-M4 flags recorded in
the early ARM audit. Results are in
`/home/zmmfly/prjs/xhive_testspace/fieldctrl/arm-basic-native-f32-HfRecB`:

- The entire resulting object has **no `__aeabi_f2d` or `__aeabi_d2f` undefined
  symbols**. `commit_result` has no floating helper call.
- `input_values` kind 0 now loops over `vldmia`, `vabs.f32`, `vcmp.f32` at
  `0x3ca` onward. The double-comparison helpers are confined to the separate
  kind-1 branch starting at `0x43c`; q values take the integer branch.
- Source inspection confirms the native f32 kernel/reducer uses float operands,
  intermediates, finiteness tests and workspace storage, not a double bridge.
- All **64** reported stack entries are static; the largest single function is
  `matrix_compute`, **264 bytes**. Object/disassembly and source hashes are
  retained. This is compile/static-path evidence, not a measured MCU speedup.

M1b worklog is ready for review together with the numerical owner's frozen
foundation implementation/test files. Downstream integration files remain a
separate milestone and must not be staged merely because their temporary tests
have already run.

### 2026-10-17 — independent decomposition regression

The audit owner found a real q15 LDLT export bug: the legal symmetric input
`[32767,128;128,-32767]` at exponent 16384 produces a negative D pivot that
rounds to representable -32768. Previewing only its positive absolute maximum
incorrectly returned RANGE; at exponent 0 the same mistake unnecessarily
increased the block exponent and lost one precision bit. The decomposition
owner changed shared output scaling to preview signed extrema.

**Delegated before/after evidence:** isolated C99/C++17 O2 ASan/UBSan audit
reports contain 26 tests with 2 failures before the fix
(`/tmp/xhive-audit-decomp-cOgVUD/regression_before.xml`) and 26 with no failures
after (`/tmp/xhive-audit-decomp-fixed-bDX3uE/regression_after.xml`). The audit
covers four variants, extreme 1x1 SVD, wide rank-1 convergence budgets and null
basis, singular LDLT failure atomicity, empty/overflow dimensions, metadata
aliasing and capacity. No second confirmed defect was reported.

The build agent subsequently executed:

```sh
xmake b
xmake test -g xhive -v 'linears_test_audit_decomp_boundaries/audit_decomp_boundaries'
```

**Confirmed:** 1 Xmake entry, **26 GoogleTest cases in 6 suites**, all passed.
Log: `/tmp/xhive-fieldctrl-audit-decomp-Mhq9PM.log`. This does not replace the
full per-factorization numerical suite recorded next.

### 2026-10-17 — full decomposition and basic-filter suites

Ran the default `xmake b`, then one `xmake test -g xhive -v` invocation with these
17 exact entries:

```text
linears_test_decomp_lu/decomp_lu
linears_test_decomp_solve/decomp_solve
linears_test_decomp_determinant/decomp_determinant
linears_test_decomp_cholesky/decomp_cholesky
linears_test_decomp_qr/decomp_qr
linears_test_decomp_contract/decomp_contract
linears_test_decomp_ldlt/decomp_ldlt
linears_test_decomp_svd/decomp_svd
linears_test_audit_decomp_boundaries/audit_decomp_boundaries
filters_test_lowpass/lowpass
filters_test_fir/fir
filters_test_biquad/biquad
filters_test_highpass/highpass
filters_test_moving_average/moving_average
filters_test_median/median
filters_test_ema/ema
filters_test_complementary/complementary
```

**Confirmed:** all 17 Xmake entries passed. GoogleTest totals were **424 passed,
2 skipped, zero failed**: six core decomposition files 100 passed; LDLT 90
passed; SVD 68 passed/2 skipped; independent decomposition audit 26 passed; eight
basic-filter files 140 passed. Full log:
`/tmp/xhive-fieldctrl-decomp-filters-Amb3X6.log`.

SVD skips are precisely
`Svd/0.FixedExtremeExponentsPreserveNormalizedFactors` (f32) and the same test
under `Svd/1` (f64). This checks explicit fixed-point block exponents near
+/-15000; floating formats ignore that exponent and the test is inapplicable.
Both q variants execute it. These are not skipped NaN/Inf or convergence tests.

The decomposition owner separately reported strict GCC/Clang C99 and independent
C99/C++17 public-header checks, plus ASan/UBSan core, LDLT and SVD runs. An initial
Cholesky test fixture used Eigen `B*B^T` whose two triangles differed by floating
rounding; it was changed to mirror one triangle explicitly, preserving the
API's exact-symmetry contract rather than relaxing production validation.

**Delegated audit frame detail:** the audit owner's non-instrumented host LDLT
objects reported a maximum 272-byte frame, all static. SVD reported a maximum
336-byte frame; some q entries were classified `dynamic,bounded` (320 or 304
bytes), with no unbounded entry. This differs from the separate ARM result and
must not be reported as “all host frames static.” The initial audit utility's
all-static assertion was corrected to admit the compiler's bounded dynamic
classification. q-only SVD/LDLT instantiations had only integer-core/stack-guard
undefined symbols; no libm or allocator dependency.

The basic-filter owner also reported 140/140 O2/strict-aliasing cases and the
same families under ASan/UBSan. An O2 `maybe-uninitialized` warning on biquad's
result was resolved by expressing the already validated nonzero section count
as a `do/while` loop, not by adding an unreachable check or dummy initialization.
GCC strict C99 accepted all six filter source files; a separate Clang check then
found an anonymous `offsetof` type in matrix Kalman to be a C23 extension. That
portability fix and final Eigen-based Kalman acceptance remain pending.

### 2026-10-17 — foundation final-review follow-up

A real strict-compiler diagnostic for `LINEARS_SIZE_MUL(0, runtime_count)` led
the numerical owner to refine the size-macro guard while retaining constant
expression behavior and checked arithmetic. The added
`RuntimeAndLiteralZeroMacrosStayChecked` case exercises literal/runtime zero
and an 8x8 boundary-value matrix.

The build agent reran `xmake b` and the same six foundation/numeric/audit
selectors used above: **105 GoogleTest cases in six Xmake entries passed**
(62 foundation + 7 contracts + 36 audit), log
`/tmp/xhive-fieldctrl-basic-final-J9hPaC.log`. This is a real intermediate result,
not the final post-review count: the coordinator then requested removal of an
unreachable fixed-norm branch and an additional maximum-raw-span regression.
For one block exponent, nonzero q15/q31 raw magnitudes can span at most 15/31
binary exponents, so the old `gap>32` skip could never execute. Its removal is
review cleanup of dead logic, not a change in attainable numerical behavior.
After removal, added two maximum-raw-span norm tests with
`{INT16_MIN,1}` / `{INT32_MIN,1}` at shared exponents -100, 0 and 100.
The build agent reran `xmake b` and all six foundation/numeric/audit selectors:
**107 GoogleTest cases in six Xmake entries passed** (64 foundation + 7 numeric
contracts + 36 independent numeric audit), log
`/tmp/xhive-fieldctrl-basic-reviewed-c5PXn5.log`. This is the final reviewed M1b
host result. The owner separately reran all 64 foundation cases under strict
C99/O2/strict-aliasing/ASan/UBSan. f32 code was unchanged by this last cleanup;
the prior ARM native-f32 disassembly observation still describes its code path.
The coordinator committed M1b as `b5ffaf2`
(`feat(linears): add allocation-free basic matrix operations`).

### Control review defects pending final verification

The independent control reviewer reproduced three finite-input defects under
ASan/UBSan without sanitizer diagnostics. Passing ordinary-scale baseline tests
did not expose these numerical branches:

- DTC floating subnormal flux `(2*denorm_min, denorm_min)` was assigned sector 2,
  although its angle is 26.565 degrees and belongs to sector 1. Absolute
  alpha/sqrt(3) rounding created a false boundary; the owner is normalizing the
  comparison before sector selection.
- FOC radial anti-windup used a dot product whose sign test underflowed at
  ordinary finite tiny radii `2^-80` (f32) / `2^-600` (f64). Integral state could
  continue growing under saturation. The speed-loop product sign test had the
  same issue. The owner is replacing these with scale-safe sign decisions.
- SVPWM failed common-scale invariance for alpha=bus=denorm_min, beta=0,
  returning duties `(1,0.5,0.5)` instead of the normalized limited-vector result.
  The coordinator authorized a narrow base implementation/test fix.

The independent pre-fix control audit reported 16 cases: 8 passed and 8 failed
in the floating variants; q variants passed these examples. Artifacts are in
`/tmp/fieldctrl-xhive-audit-gzIbFd`. The earlier 64-case control baseline remains
historical evidence, not acceptance of the pending repaired versions.

### 2026-10-17 — isolated sanitizer checkpoint and output-path correction

A separately configured Xmake ASan/UBSan/O2/strict-aliasing linears run completed
successfully: **15 Xmake entries, 389 GoogleTest cases passed and the same two
fixed-exponent-only floating SVD cases skipped**. This checkpoint preceded the
two final maximum-raw-span norm tests. Compiler commands in the log confirm
`-O2 -fstrict-aliasing -fsanitize=address,undefined
-fno-sanitize-recover=all -fno-omit-frame-pointer` on production and test objects,
plus C99/C++17. The isolated cache and full log are at
`/home/zmmfly/prjs/xhive_testspace/fieldctrl/host-sanitized-QuOKXV`.

**Confirmed environment error:** invoking Xmake 3.0.8 from that temporary
working directory with `-P` pointing at the SDK and `-o` set to an absolute
external build directory resulted in output paths being rebased twice. Actual
objects/binaries landed at
`/home/zmmfly/prjs/xhive_testspace/xhive_testspace/fieldctrl/host-sanitized-QuOKXV/build`,
outside the specifically authorized application directory. The expected cache
and log stayed in the correct directory. The build agent detected the mismatch
in compiler commands and verified the unexpected directory, stopped using that
configuration, and reported it to the coordinator before any cleanup.

Only the newly generated, identified output tree is eligible for authorized
relocation/cleanup; existing neighboring data must not be touched. Future
isolated runs will keep cwd at the real source project, isolate only
`XMAKE_CONFIGDIR`/build output, and assert resolved target paths before compiling.
The earlier standalone-control output location is also being checked. This
incident does not turn the executed test results into a hardware result or
justify claiming the final suite complete; final post-fix regression is pending.

The coordinator inspected and authorized two exact relocations. After checking
all source/parent nodes with `lstat` (no symlink), exact realpaths, current-user
ownership, absent destinations, terminated Xmake jobs and no current-user
`/proc` cwd/fd references to the source trees, the build agent used `os.rename`
to move only:

1. The sanitizer `build` directory from the doubly nested path above into its
   correct `host-sanitized-QuOKXV/build` directory.
2. The early standalone-control `build` directory from
   `/home/zmmfly/xhive_testspace/fieldctrl/host-fieldctrl-base-wmKQDl/build` into
   `/home/zmmfly/prjs/xhive_testspace/fieldctrl/host-fieldctrl-base-wmKQDl/build`.

Both moves completed without overwriting a destination. No recursive deletion
occurred; all old empty parent directories were retained. New SDK sanitizer
configuration at `host-verify-DXUdWt` now runs with SDK-root cwd and separately
isolated cache. Before any build, explicit Xmake target/object introspection
with `config.load()` verified all 323 returned output-path entries lie inside
its authorized build root. A standalone preflight that omitted `config.load()`
correctly stopped before compilation when it observed default paths; it must be
rerun with loaded configuration, not bypassed.

### M2 commit handoff

The decomposition owner froze 26 owned files after source/whitespace review.
The accepted eight family entries contain 260 registered GoogleTest cases:
258 passed and exactly two floating instantiations of the fixed-exponent-only
SVD test are inapplicable/skipped. The independent 26-case audit also passed and
is reserved for a following test-only commit. The production signed-extrema
export correction is included in the decomposition change. This handoff does
not include the still-in-progress filters/control source trees or the draft
root integration/documentation updates for them.
The coordinator committed M2 as `c9a14da`
(`feat(linears): add four-format static matrix decompositions`) and the separate
independent audit as `70e9d2c`
(`test(linears): cover decomposition scale and failure boundaries`).

The build agent also independently checked every current linears production
source with both GCC 15.2.0 and Clang 21.1.8:

```sh
for compiler in cc clang; do
    "$compiler" -std=c99 -pedantic-errors -Wall -Wextra -Werror -Wvla -Walloca \
        -O2 -fstrict-aliasing -Ilibs/linears/include -fsyntax-only libs/linears/src/*.c
done
```

Both compilers passed. This check includes the final size macros and native-f32
template, and is independent of the owners' earlier compiler reports.

### 2026-10-17 — corrected-isolation final linears sanitizer run

From the actual SDK root, using only an external cache/output directory whose
resolved target and object paths were checked beforehand:

```sh
work=/home/zmmfly/prjs/xhive_testspace/fieldctrl/host-verify-DXUdWt
XMAKE_CONFIGDIR="$work" xmake test -P . -j2 -g xhive -v 'linears_test_*/*'
```

The configuration uses O2, strict aliasing, ASan/UBSan,
`-fno-sanitize-recover=all`, frame pointers and VLA/alloca compile errors as
recorded above. **Confirmed:** 15 Xmake entries passed, **391 GoogleTest cases
passed and the same 2 inapplicable floating SVD cases skipped**. This includes
both final maximum-raw-span norm regressions. Compiler commands show the intended
flags and correct output paths; the build tree was confirmed in the authorized
location. No ASan/UBSan/runtime-error diagnostic was found in the full
`host-verify-DXUdWt/linears-final.log`.

### 2026-10-17 — M3 complete filter acceptance

All ten filter categories were frozen by their owners before this run. The
matrix Kalman test now uses Eigen 3.4.0 `Matrix<long double>` with an independent
Eigen LDLT innovation solve; the build agent inspected those source references.
The temporary handwritten Gauss–Jordan oracle was replaced, not accepted as the
required Eigen comparison. Tolerances were not relaxed to make that change pass.

The final matrix workspace is three separate caller-owned typed arrays: raw
numeric intermediates, basic-kernel union units, and decomposition working
scalars. This avoids casting a union array into unrelated flat element arrays.
Measurement/innovation SPD is checked before the existing pivoted linear solve;
no explicit inverse is formed. The earlier Clang C23-extension diagnostic on an
anonymous `offsetof` type was fixed with named alignment carriers.

Executed from the SDK root:

```sh
xmake b
xmake test -g xhive -v 'filters_test_*/*'
for compiler in cc clang; do
    "$compiler" -std=c99 -pedantic-errors -Wall -Wextra -Werror -Wvla -Walloca \
        -O2 -fstrict-aliasing -Ilibs/linears/include -Ilibs/filters/include \
        -fsyntax-only libs/filters/src/*.c
done
xmake test -g xhive -v
```

**Confirmed results:**

- Filter selection: **10 Xmake entries, 201 GoogleTest cases, all passed**.
  Eight basic-filter categories contribute 140; scalar Kalman 24; matrix Kalman
  37. Log: `/tmp/xhive-fieldctrl-filters-final-2bBK2n.log`.
- Every filter production source passed strict C99 with GCC 15.2.0 and Clang
  21.1.8, including the named-alignment-carrier portability correction.
- Default SDK build passed. The full SDK host group passed **42 Xmake entries**,
  zero failed; the two already documented SVD type-specific skips remain.
  Log: `/tmp/xhive-fieldctrl-M3-host-iqxREx.log`.
- SDK-entry regression confirms filters is in the default host map, its linears
  dependency is public, standalone filters registers only its own tests, no
  production Eigen package is attached, and disabled/embedded tests do not leak
  test-package requirements.
- **Delegated owner evidence:** all 201 cases also passed isolated release and
  sanitizer runs; Kalman's final 61 cases passed Clang ASan/UBSan with Eigen.
  Those logs are under `/tmp/xhive-kalman.2A60yo`; basic-filter logs are recorded
  earlier. Unified final sanitizer regression remains a separate final gate.

M3 integration staging paths: `libs/filters/xmake.lua`, `libs/xmake.lua`,
`libs/Kconfig`, `tests/sdk_entry.lua`, and this worklog, plus the filter owner's
frozen library files. `libs/fieldctrl/xmake.lua` is still an unregistered draft;
root README and library-catalog edits covering all three libraries remain for
the final documentation node. No control source belongs in the M3 commit.
The coordinator committed M3 as `70be7fa`
(`feat(filters): add static four-format filters and Kalman estimators`).

### 2026-10-17 — control defect closure and frozen host acceptance

The confirmed control defects above were repaired before final acceptance:

- Floating DTC sector comparisons normalize their scale before testing the
  sector boundary, including finite subnormal inputs.
- Current-loop anti-windup scales vectors before taking a sign-sensitive dot
  product; speed-loop sign tests compare signs rather than multiplying tiny
  operands. Neither normal-scale underflow nor giant intermediate products
  silently disable anti-windup.
- SVPWM and VVVF calculate modulation in unit-bus coordinates and convert the
  physical voltage only at the output boundary. Common-scale tests include
  floating denorm/min/ordinary/large values and q exponents +/-1000.
- A shared rescale helper uses native `frexp`/`ldexp` for floating values and
  integer mantissa/exponent arithmetic for q. This fixes both multiply/divide
  orderings that previously lost representable radial-limit components.
- A final independent q-hypot counterexample used separately scaled components
  at exponent MAX and MIN, with limit equal to the larger component. The old
  unused `lo/hi` precision contribution returned RANGE. For canonical gap>32,
  the relative norm correction is below `2^-65`, so returning the larger value
  is correctly rounded at 31-bit working precision. This reachable scalar case
  is distinct from the removed, unreachable single-block matrix-norm branch.
  Two q-only owner cases and two independent cases confirm exact mantissa/
  exponent preservation.

Final arithmetic-header SHA-256 is
`d2aafc6a7ebe466ae4b6afb6b4dd14a57a29ad162b8a3554dc44e9150364c08a`.
The independent reviewer reran all **34** audit cases under ASan/UBSan, with
source hashes checked before/after compilation and after testing. The earlier
32-case count predates the final q-hypot regression. No hardware was accessed.

A first SDK control run succeeded while the last arithmetic/header and PI/PWM
regression were being updated. The build agent's before/after hash comparison
caught the change; `/tmp/xhive-fieldctrl-root-aERIZG.log` was explicitly not used
as frozen acceptance. After all owners froze their final files, ran:

```sh
xmake b
xmake test -g xhive -v 'fieldctrl_test_*/*' 'xhive_test_sdk_entry/sdk_entry'
xmake test -g xhive -v
```

**Confirmed:** all control source/header/test hashes stayed unchanged throughout
the precise run. **11 Xmake entries** passed (10 control entries plus SDK graph
regression). Control internal counts are:

| Node | Xmake entries | GoogleTest cases passed |
| --- | ---: | ---: |
| M4 transforms / PI-PWM / VVVF | 3 | 20 + 42 + 20 = 82 |
| M5 FOC / SMO | 2 | 38 + 36 = 74 |
| M6 DTC | 1 | 60 |
| Independent DTC / FOC / numeric / PWM audit | 4 | 4 + 12 + 8 + 10 = 34 |
| Total control | 10 | 250 |

The full SDK group then passed **52 Xmake entries**, zero failed, retaining only
the two documented inapplicable SVD floating cases. Logs:
`/tmp/xhive-fieldctrl-frozen-1cKxOb.log` (with before/after hash sidecars) and
`/tmp/xhive-fieldctrl-all-host-pKYoiO.log`.

The SDK graph now checks fieldctrl's two public production dependencies, all
three libraries' C99 production setting/no test packages, each standalone
library's own tests only, and an application selecting fieldctrl with host tests
disabled. That application includes linears and filters transitively without
Eigen/GoogleTest requirements.

The build agent also ran the six authored control entries in a separately
configured real library entry (cwd at `libs/fieldctrl`), first verifying all
228 returned target/object paths were inside its authorized output directory.
That earlier post-rescale/pre-q-hypot checkpoint passed 214 cases and both GCC /
Clang strict C99 checks. It is superseded by the frozen 250-case SDK run above.
The owners additionally linked/executed q-only base/FOC/SMO/DTC call closures on
the host without libm and reported no reachable floating-arithmetic helper or
allocator symbols; this is delegated host evidence, not MCU timing evidence.

M4 integration staging paths are `libs/fieldctrl/xmake.lua`, `libs/xmake.lua`,
`libs/Kconfig`, `tests/sdk_entry.lua`, and this worklog, together with the control
owner's frozen foundation files. FOC/SMO, DTC and independent audits retain their
separate commit boundaries despite being tested together in the workspace.

### 2026-10-17 — real Cortex-M4 application link and resource checks

The external fixture now contains a real C application with separate
`linears_calls.c`, `filters_calls.c`, `control_calls.c`, and `main.c`. It calls
all four variants of the decompositions, low-pass, scalar/matrix Kalman, VVVF,
current/speed FOC, SMO and DTC. Linker `--require-defined` roots additionally
retain **292 distinct public algorithm symbols** across all variants, even if
a representative call does not exercise every entry. This prevents section
collection from turning missing/unlinked implementations into a false pass.

All application commands run in
`/home/zmmfly/prjs/xhive_testspace/fieldctrl`, with the SDK worktree and isolated
cache passed per command. No embedded toolchain was downloaded:

```sh
sdk=/home/zmmfly/prjs/xhive_worktrees/fieldctrl
XHIVE_SDK_PATH="$sdk" XMAKE_CONFIGDIR="$PWD" xmake f -P . -y -m release --xhive_tests=n
XHIVE_SDK_PATH="$sdk" XMAKE_CONFIGDIR="$PWD" xmake b -P . app::fieldctrl_compile
```

Before building, Xmake introspection verified 162 returned target/object paths
inside the application directory and no package attached to any firmware
dependency. The toolchain is the existing Arm GNU 14.3.1, with Cortex-M4,
Thumb, `fpv4-sp-d16`, hard-float ABI, O2 and no LTO.

The **first real link failed**: without section-level GC the entire vendor
object set was retained, exceeding the fixed 256 KiB ROM by 77,796 bytes; the
`-nostartfiles` application also lacked Newlib `_init`/`_fini` hooks. The read-only
reference demo provides those hooks at its `src/main.c` lines 33/34. Only the new
fixture was changed: add its C-only runtime hooks, compile function/data sections
and link `--gc-sections`, while retaining every one of the 292 algorithm roots.
No library implementation was stubbed and no memory-region size was increased.

**Confirmed corrected link:**

- `dist/fieldctrl_compile.elf` and `.bin` were produced; library source hashes
  were unchanged throughout the build.
- ELF identity is **ELF32 ARM, EABI5 hard-float, CPU 7E-M, VFPv4-D16, VFP
  argument registers**. The Xmake cache's host-architecture directory name is
  not used as an architecture claim.
- ROM image: **260,180 / 262,144 bytes (99.25%)**, leaving 1,964 bytes.
  ELF `size`: text 260,080, data 100, bss 1,028 bytes. The 1,128-byte data+bss
  sum is not a peak runtime RAM/stack measurement. This deliberately retains
  every variant/API and is not a representative selectively linked product.
- Defined-symbol enumeration confirmed all **292 / 292** required algorithm
  symbols. No unresolved symbols and no matched malloc/calloc/realloc/free,
  sbrk/alloca or C++ allocation entry were present in the linked image.
- Actual application-build `.su` files contain **390 library entries**, all
  `static`: linears 142 entries (largest single frame 648 bytes,
  `linears_svd_f64`); filters 146 (448 bytes,
  `filters_kalman_matrix_f64_update`); fieldctrl 102 (584 bytes,
  `fieldctrl_q15_smo_step`). VLA/alloca compile errors were enabled.
- These are per-function compiler bounds, **not** a whole call-chain, interrupt
  nesting, libc/libm stack bound, WCET, or measured hardware RAM result.
- ELF SHA-256:
  `f66aae04621c22844498c2293bf69f4c30bc24fddd4a216124dcec4dbdc3b869`.

Artifacts include `arm-link.log` (original failure), `arm-link-gc.log` (successful
corrected build), source-hash sidecars, linker map, ELF identity, required/defined
symbols and `dist/validation-report.json`. All remain in the authorized external
application. **The ELF was not flashed or executed on a board or motor.**
