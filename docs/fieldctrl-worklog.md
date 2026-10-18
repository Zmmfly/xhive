# linears / filters / fieldctrl implementation worklog

## Current request — fixed-Q migration (2026-10-18)

The user has replaced the previous block-floating-point (BFP) requirement:
**remove exponents from every Q-format interface and implementation, and migrate
all affected tests.** Public q15/q31 values are raw fixed-fraction integers;
matrices contain only `data`, `rows`, and `cols`. Internal Q arithmetic must also
use a constant fractional width, not hidden per-value exponents/renormalization
or a floating-point wrapper. Native f32/f64 numerical behavior is retained.

The BFP requirements, decisions, commits, test counts and ARM images below are
**historical evidence only**, superseded for Q behavior by this request. In
particular, the old 842 passed / 2 skipped result is **not a fixed-Q migration
result**. Current requirements, pending decisions and new validation results
are tracked in [the migration section](#2026-10-18-fixed-q-migration).

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

## Historical BFP verification snapshot (superseded)

The following results verify the **old BFP source revision `6f23ff1`**, after
the M0–M6 implementation and independent-audit commits. That phase's fixture
was committed as `c06120d`, with final documentation at baseline `ab11032`.
They are preserved for provenance and before/after comparison, not reused as
proof that the new exponent-free Q implementation passes.

| New library | Xmake entries | GoogleTest passed | GoogleTest skipped |
| --- | ---: | ---: | ---: |
| linears | 15 | 391 | 2 |
| filters | 10 | 201 | 0 |
| fieldctrl | 10 | 250 | 0 |
| New-library total | 35 | 842 | 2 |

- Default SDK host build and the complete **52-entry** host group passed.
  The new-library subset also passed with **O2 + strict aliasing + ASan/UBSan**.
  The two skips are floating instantiations of a fixed-exponent-only SVD case,
  not failed or untested fixed-point variants.
- All three real standalone library entries passed their complete test sets.
  An isolated `xhive_tests=n, xhive_cross_tests=n` configuration had zero test
  targets/packages and built the default libraries successfully.
- GCC 15.2.0 and Clang 21.1.8 accepted all **18 production C files** under strict
  C99/O2/strict-aliasing/VLA/alloca diagnostics.
- The checked-in-source ARM fixture reproduced a real Cortex-M4 hard-float
  link retaining **292/292 public algorithm symbols**, with no unresolved or
  matched heap-allocation symbols. Final reproduction ROM is **260,140 bytes**
  of 262,144; data+bss is 1,128 bytes. All 390 library stack-usage entries are
  static; largest individual frames are 648 / 448 / 584 bytes by library.
- Native/sanitized and final reproduced ARM checks refer to identical SHA-256
  content for all **51 production/header files**. Per-function stack evidence
  is not a complete call-chain/WCET/interrupt bound.
- **No board, inverter, or motor was flashed or run.** Physical control behavior,
  tuning, protection and timing remain outside this task's evidence.

## Historical BFP requirements

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

## Historical BFP design decisions

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
| M4 | Control foundations, PI/PWM, VVVF | 3 entries / 82 cases; default build and SDK graph isolation | Committed as `8ab6396` |
| M5 | FOC and SMO | 2 entries / 74 cases, including software observer/handover scenarios | Committed as `6184b7e` |
| M6 | DTC | 1 entry / 60 cases, including subnormal sector regression | Committed as `5fa2c05`; independent control audit `6f23ff1` |
| M7 | Complete SDK integration and final regression | Full host group, optimized sanitizers, standalone/disabled-test builds, reproduced ARM link and resource audit | Verification complete; fixture `c06120d`; final results recorded in this documentation commit |

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
The coordinator committed M4 as `8ab6396`.

M5/M6 implementation files are frozen and ready for their separate commits.
The existing final SDK run above covers M5's 74 and M6's 60 cases against the
same algorithm hashes. Only the umbrella header and README links are being
connected at each commit boundary; algorithm implementations are unchanged.
The coordinator committed M5 as `6184b7e`, M6 as `5fa2c05`, and the four
independent control-audit files as `6f23ff1`. The final umbrella includes the
completed FOC/SMO/DTC headers. No algorithm change was needed for those final
header/README navigation commits.

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

### 2026-10-17 — M7 final optimized, standalone and disabled-test gates

After the final DTC umbrella integration, reran the default SDK build and the
complete native host group. In a separate cache/output directory, reran the
three new libraries with the following configuration from the **actual SDK
root**, never from a different `-P` invoking directory:

```sh
work=/home/zmmfly/prjs/xhive_testspace/fieldctrl/host-verify-DXUdWt
XMAKE_CONFIGDIR="$work" xmake f -P . -y -m debug -o "$work/build" \
    --xhive_tests=y --xhive_cross_tests=n \
    '--cxflags=-O2 -fstrict-aliasing -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer' \
    '--cflags=-Werror=vla -Werror=alloca' \
    '--ldflags=-fsanitize=address,undefined'
XMAKE_CONFIGDIR="$work" xmake test -P . -j2 -g xhive -v \
    'linears_test_*/*' 'filters_test_*/*' 'fieldctrl_test_*/*'
```

Resolved output paths were checked before building. Actual compiler commands
contain the requested instrumentation/O2/strict-aliasing flags. Final normal
and instrumented logs were parsed per test entry, not estimated from file counts:

| Gate | Xmake entries | New-library GoogleTest passed / skipped |
| --- | ---: | ---: |
| Full SDK native group | 52 | 842 / 2 |
| New-library optimized ASan/UBSan group | 35 | 842 / 2 |

The per-library totals are the final snapshot table at the start of this log.
No ASan/UBSan/runtime-error diagnostic was found in the final instrumented log.
All 51 production/header file hashes were unchanged through those runs and
subsequently matched current committed revision `6f23ff1`.
Artifacts in `host-verify-DXUdWt` include `final-sdk-build.log`,
`final-sdk-host.log`, `final-sanitized.log`, `final-summary.json`, and final
before/after source manifests.

Actual independent library entries were also configured and run completely.
For each library, cwd was that library's real source directory, with an external
cache/build directory under `standalone-final-y28JLe/<library>`:

```sh
XMAKE_CONFIGDIR="$work" xmake f -P . -y -m debug -o "$work/build"
XMAKE_CONFIGDIR="$work" xmake test -P . -j2 -g xhive -v
```

Preflight loaded the saved Xmake configuration, asserted every target/object
path was inside the assigned output root, and rejected dependency-library tests.
Results: linears 15 entries / 391 passed / 2 skipped; filters 10 / 201 / 0;
fieldctrl 10 / 250 / 0. Fieldctrl was rerun once more after the final umbrella
change (`fieldctrl/final-umbrella.log`). These are real independent executions,
not just successful target-graph parsing.

An additional isolated configuration at `host-no-tests-iIFQmV` used
`--xhive_tests=n --xhive_cross_tests=n`. Inspection verified zero registered test
targets, zero global/host package requirements and zero target packages, while
all three libraries remained selected by the default SDK aggregate. All 112
returned object paths stayed inside its assigned output root. Its `xmake b -P .`
completed successfully. No `nothing to test` result was counted as a passing
runtime test; this gate verifies the intentionally test-free library build.

The final strict-C99 check at revision `6f23ff1` compiled all 18 production C
files with both GCC 15.2.0 and Clang 21.1.8:

```sh
for compiler in cc clang; do
    "$compiler" -std=c99 -pedantic-errors -Wall -Wextra -Werror -Wvla -Walloca \
        -O2 -fstrict-aliasing -Ilibs/linears/include -Ilibs/filters/include \
        -Ilibs/fieldctrl/include -fsyntax-only libs/{linears,filters,fieldctrl}/src/*.c
done
```

Both passed. Kconfig selections were evaluated in memory: enabling filters
selected linears, and enabling fieldctrl selected both. The system `python3`
initially lacked the `kconfiglib` module; instead of installing anything, the
check used the existing `genconfig` launcher's recorded pipx interpreter
(`/home/zmmfly/.local/share/pipx/venvs/kconfiglib/bin/python`). The selection check
then passed without writing any configuration file.

### Reproducible fixture preserved in the repository

The coordinator granted ownership of
`libs/fieldctrl/test/compile/n32h474/` for the final integration node. It contains
only the tested `.config`, `xmake.lua`, four `.c` files, and a short README.
The six build-input files were copied byte-for-byte from the successful external
application. Their source/external SHA-256 correspondence is recorded in the
fixture README; no build, cache, map, log or binary output was added to Git.
There is no test generator and no new `.cc` file/test registration.
The coordinator committed the seven-file fixture as `c06120d`
(`test(fieldctrl): preserve reproducible N32H474 link fixture`).

To validate the committed-source reproduction process itself, copied this
fixture into a fresh authorized external directory,
`/home/zmmfly/prjs/xhive_testspace/fieldctrl/reproduce-EgR2r0`, then configured and
built it with the same documented per-command SDK/cache environment. Before
build, all 162 firmware target/object paths were confirmed inside that directory
and firmware packages were absent. The six payload files matched both the repo
copy and original external application exactly.

**Final reproduced ARM result:**

- Compile/link and the subsequent quick rebuild at source revision `6f23ff1`
  passed without changing library source/header hashes.
- All **292** required algorithm symbols are defined; no unresolved or matched
  allocator symbol is present.
- Final ROM image is **260,140 / 262,144 bytes (99.24%)**, leaving 2,004 bytes;
  text/data/bss are 260,040 / 100 / 1,028 bytes. All 390 library `.su` entries
  remain static, with maximum single frames 648 / 448 / 584 bytes.
- The final native/sanitized source manifest and reproduced ARM manifest match
  exactly across **51 production/header files**.
- ELF SHA-256:
  `2249c1e50720ec2ede453b03b4b0c102dee329e26d5397bc913e6fa80cba7811`.
- BIN SHA-256:
  `c9f831084776c8f5bf1aa76eafe80c4a6322d0c33df07f218868f9788aa7f7f2`.
- The original 260,180-byte image and this final reproduced 260,140-byte image
  are not byte-identical. Both retained all algorithm roots and stayed within
  the same memory limit; no bit-reproducible binary claim is made. The preserved
  fixture guarantees the tested source/configuration inputs, not identical
  compiler/debug/link layout across work directories or header snapshots.

Final ARM details are in `reproduce-EgR2r0/dist/final-validation.json`, its ELF
identity/symbol listings, source manifests and `final-recheck.log`. Neither
image was flashed or executed on hardware. **M7 verification is complete**;
the fixture is preserved in `c06120d`, and the final results are recorded in
this documentation commit. No further source-code change is part of this work.

Final staging review at `6f23ff1` confirmed only the three entry README files,
this worklog, and the seven-file fixture remained changed/untracked. Tracked
`git diff --check`, individual new-file whitespace checks, six-file payload
byte-identity checks, and local Markdown-link resolution all passed. No agent
other than the coordinator staged or committed these final files.

## 2026-10-18 fixed-Q migration

### Request, scope, and compatibility boundary

**Confirmed new requirement:** remove all Q-format exponents and update related
production code, tests and consumers. This supersedes the earlier explicit-BFP
approval; retaining that representation for compatibility would violate the new
request. Native f32/f64 algorithms and numerical behavior must remain intact.
The migration begins from committed baseline `ab11032`.

- Public q15 is an `int16_t` raw value with 15 fractional bits; public q31 is an
  `int32_t` raw value with 31 fractional bits. A matrix descriptor has only
  `data`, `rows`, and `cols`; callers and test initializers must be migrated.
- The working Q representation has one constant fractional width. There must
  be no hidden mantissa/exponent pair, per-value pack/normalize path, or Q
  computation routed through float/double. Wider integer working values and
  explicitly fixed-width coefficient/state types must document their format.
- Allocation-free, caller-owned state/workspace, bounded stack, C99 production,
  C++17 tests, host-only Eigen and independent contexts remain requirements.
- This is an intentional source/ABI-breaking change. The coordinator will
  commit the shared production/API/test migration as one logically complete
  change across all three libraries, not a sequence of broken intermediate
  library heads. No temporary BFP compatibility layer is added merely to split
  commits. Independent audit, validation fixture and documentation can follow
  in separate reviewed commits.
- The build/worklog agent remains the only SDK-root Xmake operator. No active
  half-migrated source tree will be compiled before owners declare a coherent
  ready boundary. Agents do not stage/commit; the coordinator owns those actions.
- Software tests and ARM compilation/linking only: no board or motor execution,
  flashing, reference-demo edits, or new cross-toolchain installation.

### Frozen shared contract (coordinator confirmation)

The coordinator approved the shared contract after the numeric proposal. These
are **confirmed design requirements**, not yet claims that implementation or
validation has completed:

- Public matrix storage is exactly `data`, `rows`, `cols`; public q15/q31 are
  `int16_t` F15 / `int32_t` F31 raw values.
- `linears_fixed_t` is `int64_t` with a constant 31 fractional bits, representing
  values from `-2^32` through `2^32 - 2^-31`. Working ONE is raw 2,147,483,648.
  There is no exponent field, normalized-mantissa representation, pack path, or
  floating seed in Q computation.
- Public Q output conversion accepts a working value only in `[-1,+1]`.
  Exact +1 and the entire in-domain positive endpoint rounding cell map to
  the maximum raw positive value, with at most one endpoint LSB error; this
  rule is not restricted to an exact working value of one.
  A working value genuinely larger than one returns RANGE without modifying
  outputs. Q MAX itself still means `(2^F-1)/2^F`, not mathematical one; inverse of 0.5 cannot
  be represented publicly and must fail rather than silently scale or clip.
- MAC uses two `uint64_t` limbs as a signed 128-bit accumulator at constant F62,
  accumulating exact products before the defined final rounding. It does not
  depend on ARM `__int128`. Basic raw-product loops must not import, normalize,
  or invoke checked scalar conversion for every multiply/add.
- Each matrix output uses one local 16-byte accumulator and then quantizes into
  the existing 8-byte workspace unit. The whole caller workspace is not doubled
  merely to hold accumulators for every output.
- Parameters that genuinely need values beyond Q1, such as matrix scale/AXPBY
  weights and SOS/PI coefficients, use the same explicit `int64_t` F31 type,
  including q15 gains; there is no separate int32-F15 coefficient format.
  Ordinary signal storage remains raw q15/q31. Float behavior is retained,
  apart from removing its formerly ignored matrix exponent field.
- Shared `mean_q15`/`mean_q31` operations consume the F62 sum and a `uint64_t`
  count for the actual averaging requirement; rounding follows the same
  constant-width integer contract.
- One `lowpass_work_step` is shared by Q low-pass and SMO wide fixed-state
  updates. This does not create a fifth family of public filter contexts.
- Eight shared working-matrix entries in `linears_work.h` were approved for the
  concrete Kalman intermediate requirement: mul/add/sub/transpose/AXPBY/identity
  plus solve/Cholesky. They reuse linears kernels, not a second matrix engine
  inside filters or a complete fifth public numeric variant. Working identity
  is exactly one; storing it as raw Q MAX would otherwise shrink covariance
  even for H=0, and S/K/J can legitimately exceed the public Q1 range.
  Kalman Q temporary data is therefore a typed `linears_fixed_t[]` array.
  This may cost more bytes than the old raw-q15 temporary storage; exact macro
  capacities and measured memory must be reported, not a blanket RAM reduction.
- Unclipped requested-voltage and unlocked-frequency diagnostics use explicit
  wide F31 storage. A legitimate command above one must not prevent the normal
  saturation path merely because its diagnostic could not fit raw Q1; limited
  signal outputs remain raw q15/q31.
- Compound Kalman/decomposition output checks apply to the computed, rounded
  F31 working value, not an ideal-real whole-expression interval proof. The
  approved n=5 Kalman example has x=1/4 and F row `{MAX,MAX,MAX,MAX,raw5}`:
  exact F62 is `2^62 + 2^29` (ideal value `1 + 2^-33`), `mul_fixed` rounds to
  working ONE, and final Kalman export legally maps to MAX. The working error
  `2^-33` and endpoint error `2^-31` are documented separately. Direct raw-Q
  matrix multiply / `accum_to_q` instead checks the exact F62 domain and returns
  RANGE atomically for that same direct inner product. Neither contract is
  weakened by adding a mixed-output API or silently changing tolerances.

Consequences to preserve in tests and documentation:

- +1 is not exactly representable in signed Q1.15/Q1.31. The approved endpoint
  export rule does not make MAX equal to one for input arithmetic or turn
  coefficients/alpha into implicit identity operations.
- LU/LDLT/QR/SVD factors, inverse results and gains can exceed the public Q1
  range even when the inputs fit. Tests and ARM calls must select real
  representable successful cases, and separately verify explicit overflow
  failures. The reference must not secretly rescale/normalize its output.
- Mathematical scaling inside an algorithm, such as a norm's ratio to its
  maximum value or voltage-vector normalization, is distinct from changing the
  working value's fractional width. The former may be required by the
  algorithm; the latter/per-value exponent metadata is forbidden in Q paths.
  Ordinary mathematical vector normalization and integer limb shifts are
  allowed; dtype exponents and per-MAC representation normalization are not.
- The filter owner chose one fixed 16-byte exact F62 running sum in each Q moving
  average context: candidate subtraction of the departing sample, addition of
  the new sample, one integer mean/rounding, then atomic commit. This makes Q
  updates O(1) without an accumulating rounding drift; native floating windows
  retain their old O(N) recomputation. The additional context bytes must be
  included in memory reporting.
- Native floating `frexp`/`ldexp` use is not Q exponent metadata. Source checks
  must distinguish the preserved floating implementation and historical text
  from active Q storage/conversion paths.

### New validation gates (not yet results)

| Gate | Required evidence | State |
| --- | --- | --- |
| FQ0 requirements and plan | Historical BFP notice, coherent breaking-commit boundary, old snapshot identity | Committed as `23e22e2` |
| FQ1 shared migration | Frozen raw-Q ABI/format; all three production APIs and related tests migrate together | Coherent native/default build and tests passed; review/commit pending |
| FQ2 independent correctness review | Raw integer/rational oracles, Eigen on quantized inputs, float regressions, failure atomicity | Isolated owner/audit results reported; SDK acceptance pending |
| FQ3 SDK and compiler regression | Default build/full host group, O2 strict-aliasing ASan/UBSan, GCC/Clang strict C99, standalone entries, tests-disabled build | New native/sanitizer/compiler gates passed; standalone/disabled-test gates pending |
| FQ4 ARM and fixture | Three-field matrices/raw Q constants, representable calls, 292 retained APIs, heap/stack/architecture/source-hash checks | Not run for new Q |
| FQ5 representative performance | Same-valued old/new matrix multiply, dot, FIR and control workloads; host timings and ARM hot-loop evidence | Old-only unified preflight and independent math comparison measured; full comparison pending |
| FQ6 final documentation | Updated contracts/entry points and an exact new-results ledger; no old pass count presented as new | Planned |

ABI checks will establish raw scalar types and removal of public Q exponent
fields, while implementation review checks the constant-width integer working
representation. Numerical checks use actual quantized inputs and independent
raw-integer/rational or Eigen references, not production conversions as the
oracle. Obsolete exponent cases must map to applicable raw MIN/MAX/LSB,
headroom, rounding/underflow, overflow, alias/workspace and atomicity boundaries.
Floating extreme-scale regressions remain. New entry/case counts will be counted
from actual execution; preserving or reducing an old count is not a success
criterion.

The independent control reviewer has proposed mapping prior Q-exponent cases
to raw endpoints/LSBs, angle wrap/CORDIC boundaries, valid unclipped commands,
finite-bus anti-windup/recovery, long-running representable fixed-F31 integration,
and consistent per-unit DTC quantities. At this checkpoint that is a **coverage
plan only**: no migration audit code or new-Q validation has completed.

For every new isolated Xmake run, cwd stays at the real source project, only
`XMAKE_CONFIGDIR`/output roots move externally, and `config.load()` precedes
resolved target/object path assertions. Every new temporary directory is unique,
inside the authorized test space, and contains `-xhive-` in its name. Prior
artifacts are not overwritten; no broad cleanup or reference-demo change is
permitted.

### Preserved before-migration baseline

**Confirmed:** archived the committed baseline, not the potentially changing
working tree, into the exclusive authorized directory:

`/home/zmmfly/prjs/xhive_testspace/fieldctrl/fixed-q-xhive-baseline-WhgnPz`

- Full revision: `ab11032235c9f94ecd7e7708cb1a72a0e5422966`.
- `git archive` captured `LICENSE` and the complete three library trees,
  including headers, implementation fragments, tests and the old fixture:
  **118 regular files**. Extraction rejected links, absolute paths and traversal.
- Archive SHA-256:
  `5070e7cb709327dc328e9b63ceb0d8a445b1c9701da7ff0acd8b23ca01977006`.
- `source-sha256.json` identifies every extracted file. All **51** old production
  and header hashes match the previously verified ARM reproduction manifest.
- `arm-before/basic.o`, `numeric.o` and their `objdump -dr` listings were copied
  from that prior verified ARM build. `arm-artifacts.json` records original
  paths and object digests. No prior artifact was modified and no target binary
  was executed.

The preserved source and old ARM objects are comparison inputs, not evidence
for the new implementation. New benchmark wrappers/output will live outside
this source snapshot. Representative workloads will keep dimensions, input
values, iteration count, compiler/optimization and observable results comparable;
all chosen numerical results must fit the new public Q range. Host measurements
will identify the host and measurement method and cannot be reported as Cortex-M4
cycles or an unmeasured MCU speedup. ARM disassembly will examine actual hot Q
loop calls/instructions for old pack/bit-length/per-scalar normalize work versus
the new constant-width arithmetic, not merely compare an entire mixed-format
object's symbol inventory. No general-purpose performance framework is planned.

### FQ0 execution ledger

- Read the existing worklog and marked its completed BFP verification/requirements
  as historical; the old evidence remains intact.
- `git status --short` before snapshot creation produced no changes at the
  reported baseline checkpoint.
- Created and verified the committed-source/archive/object snapshot described
  above. This was a read-only source extraction plus file preservation, not a
  build or performance measurement.
- The coordinator subsequently approved the shared fixed-F31/two-limb-MAC
  contract recorded above and released the owners to migrate their files.
- **New-Q build, runtime tests, sanitizer checks, ARM link and performance
  comparison: not run.** They await coherent migrated sources. No old 842/2
  result is counted toward these gates.
- The coordinator committed FQ0 as `23e22e2`
  (`docs(numerics): define exponent-free Q migration contract`).

### Old-only representative benchmark preparation

Created a small standalone C measurement program in the exclusive directory
`/home/zmmfly/prjs/xhive_testspace/fieldctrl/fixed-q-xhive-bench-LLfReO`.
It is not a new SDK performance framework or test registration. Only the
preserved `ab11032` source snapshot has been compiled/executed so far; the
active partly migrated tree has not been built.

The ten workloads are q15/q31 versions of 4x4 and 16x16 matrix multiplication,
a 256-element dot product, a 16-tap FIR step, and an alternating-error PI control
step. Inputs and exact expected results are common, deliberately representable
binary fractions, not dynamically normalized reference values:

- Matrix entries 1/16 produce 1/64 (order 4) and 1/16 (order 16).
- Dot entries 1/64 produce 1/16 over 256 products.
- FIR coefficients 1/32 and signal 1/4 produce steady output 1/8 over 16 taps.
- PI gains 1/2 and 1/4, errors +/-1/8, dt=1/256 and limits +/-1/2 exercise
  integration without clipping. First positive and negative samples are checked
  separately, so a later ABI adapter cannot silently give dt the wrong units.

Initialization, warmup, exact-result validation and printing occur outside the
`CLOCK_MONOTONIC` timed loops. The old build used GCC 15.2.0, strict C99, O2,
strict aliasing, no LTO and no sanitizers; commands are preserved in
`build-before.sh` / `build-before.log`. The first old-only run pinned its own
process to allowed logical CPU 0 on an AMD Ryzen 9 7945HX. It completed ten
exact-result checks; `before-smoke.csv` and `before-smoke.json` preserve raw
measurements, compiler/host/affinity, load averages and source/binary digests.

These are **old-BFP preflight measurements only**, not a fixed-Q pass or a
speedup result. After coherent new sources are ready, the same workload program
will be compiled against the new API with explicitly checked representation
adapters, then repeated paired old/new batches will report host timing spread.
No old normalized arithmetic is substituted into the new implementation, and
host nanoseconds will not be presented as MCU cycles.

### Isolated migration checks reported before SDK acceptance

The following are **delegated owner/auditor results on their own coherent
snapshots**, not a completed SDK build of the still-migrating three-library
working tree:

- Numeric/basic/working owner: six categories, **86/86** cases under strict
  C99/C++17 O2/strict-aliasing and ASan/UBSan. Breakdown: numeric 10, elementwise
  25, products 13, reductions 12, contracts 20, fixed working matrices 6.
- Independent numeric/basic/working audit: **42/42** in both plain and
  ASan/UBSan runs, 18 suites, no skips/errors/disabled cases. Decomposition audit:
  **41/41**, 5 suites, likewise both plain and sanitizer runs. XML counts were
  independently parsed by the auditor. Artifacts are in
  `/tmp/linears-xhive-audit-full-41esVC` and
  `/tmp/linears-xhive-audit-decomp-GTkzNu`.
- The new audit uses host `__int128` integer reference arithmetic, guarded by
  `__SIZEOF_INT128__`. This replaces the old long-double-mantissa requirement;
  it does not add a production/ARM `__int128` dependency. Entry README drafts
  were updated accordingly. Coverage includes all q15 values and midpoint
  neighbors, full-width arithmetic, signed-128 carry/borrow/overflow,
  cancellation, mean count boundaries, raw output-domain checks, working
  headroom, and decomposition overflow/atomicity. No new numerical production
  defect was reported; a stale AXPBY separate-rounding comment was corrected.
- Decomposition owner: nine categories, **293 cases: 291 passed / 2 skipped**,
  with plain and sanitizer runs. Core plus work solve/Cholesky: 113; LDLT: 99;
  SVD: 81 registered, 79 passed. The skips are precisely the f32/f64
  instantiations of `SmallestStoredQuantumRetainsOrthogonalDirections`; raw
  fixed-point quantum semantics do not apply to those two floating types.
  Both Q variants execute it. The independent 41-case audit is separate.
  Compiler/header checks and logs are in
  `/tmp/xhive-decomp-fixed-san-FmZyMe` and
  `/tmp/xhive-decomp-fixed-clang-Q1RJv1`.
- Filters owner reported complete release compilation and per-category results:
  six basic families 118, FIR/SOS 40+44, scalar/matrix Kalman 28+41, totaling
  **271**. Final sanitizer/freeze work is still pending; this unlocks component
  compilation, not an assertion that the full SDK gate passed.
- Independent control audit migrated four source files to 40 planned cases,
  preserving 16 floating regressions and adding 24 raw-Q cases. At this
  checkpoint only selected syntax checks have run; those 40 cases are **not
  yet reported as executed or passing**.

The numeric owner's separate small math benchmark provides cross-check evidence
on GCC 15.2.0/x86-64, O2 and strict aliasing, without LTO/fastmath/sanitizers.
It used deterministic signed values
`a=((i*17)%15-7)/256`, `b=((i*11)%13-6)/256`, warmup 100 and five timed batches.
Per-batch iterations were 20,000 (4x4), 500 (16x16), and 10,000 (dot256).
Reported median host ns/call were:

| Format / workload | Old BFP | New fixed-Q |
| --- | ---: | ---: |
| q15 matrix4 | 1403.66 | 101.37 |
| q15 matrix16 | 79642.61 | 2446.23 |
| q15 dot256 | 5849.54 | 113.68 |
| q31 matrix4 | 1339.96 | 113.24 |
| q31 matrix16 | 79073.14 | 3541.91 |
| q31 dot256 | 5693.88 | 161.83 |

Source, raw min/max results and digests are preserved in
`/tmp/linears-xhive-fixed-xDyzEf`. CPU affinity/frequency were not fixed in that
independent check. Old/new output quantization differs: its aggregate checksum
changed by one q15 half-LSB, while every result met the independent raw-integer
reference within one signal LSB. These are observed timings for that workload,
not bit-equivalent outputs, universal speed ratios, or MCU cycle counts. The
build agent's exact-binary-fraction paired workload remains the primary planned
four-family comparison.

### First ready-core ARM compiler check: failure retained

Copied a stable 23-file linears production/header snapshot to
`/home/zmmfly/prjs/xhive_testspace/fieldctrl/fixed-q-xhive-armcore-rSWTyu`.
Only the ready numeric/basic component was compiled; no SDK-root Xmake command
or partially migrated filter/control build was launched. Existing ARM GCC
14.3.1 used strict C99/O2/strict-aliasing/no-LTO, Cortex-M4 Thumb hard-float,
function/data sections and VLA/alloca errors.

`numeric.c` compiled. `basic.c` failed at `basic_q_impl.h:13`:

```text
error: comparison is always true due to limited range of data type [-Werror=type-limits]
if ((uint64_t)count <= (uint64_t)INT64_MAX / (UINT64_C(1) << 30)) {
```

On 32-bit `size_t`, that q15 signed-64 accumulation bound always holds. The
numeric owner was asked to use a compile-time size-width branch, retaining the
real large-count fallback on wider hosts, rather than suppressing the warning
or preserving an impossible runtime failure check. The original failed snapshot,
source hashes and `failure.txt` remain intact; successful corrected ARM basic
compilation and hot-loop comparison are still pending at this checkpoint.
Additional old ARM FIR/control objects and their disassembly were preserved
alongside the original basic/numeric objects, with a separate digest manifest;
no prior artifact was overwritten.

### Corrected ARM raw-MAC core and CORDIC hot-loop evidence

The q15 guard portability correction was applied only in `basic_q_impl.h`:
32-bit `size_t` builds use the proven signed-64 sum directly and do not compile
its impossible runtime guard or wider fallback; wider hosts retain the actual
large-count guard and 128-bit fallback. Q31 keeps its 128-bit accumulator.
No warning was disabled and the arithmetic/API contract did not change.

A new immutable snapshot at
`fixed-q-xhive-armcore-pass-hZN3Rd` compiled numeric/basic successfully with the
same strict ARM GCC 14.3.1/Cortex-M4/O2 flags. Numeric reports 23 static stack
entries (largest single function 72 bytes, `linears_fixed_muldiv`); basic reports
69 static entries (224 bytes, `matrix_compute`). These are component object
checks, not the final firmware link or complete call-stack bounds.

Observed offsets within the new ARM object procedures:

| Path | Inner loop | Observed arithmetic/calls |
| --- | --- | --- |
| q15 matrix multiply | `matrix_compute+0x608..0x616` | Halfword loads and `SMLALBB`; no helper call |
| q15 dot | `reduction+0x5dc..0x5ea` | Halfword loads and `SMLALBB`; no helper call |
| q31 matrix multiply | `matrix_compute+0x456..0x48e` | `SMULL` plus `ADDS/ADCS/ADC` limb accumulation; no helper call |
| q31 dot | `reduction+0x142..0x17c` | `SMULL` plus carry/sign-extension accumulation; no helper call |

Quantization calls/tail calls occur after a whole dot or completed output,
not each product. The preserved `ab11032` matrix loop instead has two
`linears_fixed_from_i64` imports followed by `linears_fixed_mul` and
`linears_fixed_add` within each multiply/accumulate iteration. This establishes
removal of that per-scalar representation work in the observed hot loops; it
does not measure Cortex-M4 cycles or imply an entire mixed-format object has no
floating functions.

The coordinator then requested a specific CORDIC audit before any optimization.
Strict ARM compilation of source SHA
`01b55a68b285a86b7a6046a73a7e89403e4a3d531daa2c8b7d384bcfa31c5485`
confirmed two generic signed-64 divide helper calls **inside each iteration**:
`fcq_sincos+0x124/+0x13a` and `fcq_atan2+0x16c/+0x17e`, with observed backedges
and the 32-iteration bound. This was a real code-generation issue, not an
inference from whole-object undefined symbols. Before evidence is retained in
`fixed-q-xhive-cordic-before-UAhPSZ`.

Under the coordinator's narrow authorization, only coordinate division by
2^i was replaced with unsigned-magnitude shifting and sign restoration.
The fixed coordinate bound excludes INT64_MIN, truncation stays toward zero,
and the iteration budget/end conversions are unchanged. The owner compared
before/after real shared libraries under O2/UBSan: full 65,536 q15-angle grids,
65,536 seeded atan2 pairs, 4,096 full working-range pairs and 121 corner/axis
statuses matched bit-for-bit. Independent numeric audit reran 10/10 cases on
the new hash. The owner also reran all 277 control cases, ordinary and sanitizer,
after the change, rather than reusing pre-change totals.

After source SHA
`b226b18678b1a0e3b45ac3f9cc7a4863da40f05ac11665b2f018f6c433f5013e`,
strict ARM snapshot `fixed-q-xhive-cordic-after-vAddqO` contains no generic
64-bit divide-helper call in either CORDIC procedure. Only the intended
loop-external absolute/division setup and terminal `linears_fixed_muldiv`
conversions remain. Native shift instructions implement the iteration update;
`call-audit.json` and `cordic.dis` preserve the observations.

The unified small host benchmark was extended with VVVF, retaining PI and all
other items. Frequency 1/8, amplitude 1/4, bus 1/2, and dt 1/256 produce exact
1/2048-turn increments. Reset after warmup and 20,480 timed steps end at angle
zero with voltage (1/4,0), duties (7/8,1/8,1/8), no saturation. The old-only
extended smoke passed all 12 exact-result checks. Later VVVF timing changes
include this CORDIC optimization as well as the fixed-Q migration: the overall
comparison is **old implementation versus new implementation**, not an isolated
measurement of exponent-field overhead.

### Important fixed-Q15 SMO accuracy limitation

Removing BFP changes fixed-Q15 input/configuration resolution. The control owner
and coordinator explicitly retained two failures of the old dynamic SMO
software-model physical-unit targets. No further basis/gain search, tolerance
increase, mean-for-peak substitution, or claim of restored performance is made.
The targets remain peak error <0.25 Hz and final-sample error <0.03 Hz.

| Q15 unit basis (V / A / seconds) | Peak speed error | Final-sample error | Implementation vs actual quantized-reference error |
| --- | ---: | ---: | ---: |
| 32 / 4 / 0.005 | 0.533496223390 Hz | 0.034133337438 Hz | about 2.39304e-5 Hz |
| 16 / 2 / 0.0064 | 0.317164357426 Hz | 0.104364096653 Hz | about 1.05771e-5 Hz |

The one engineering candidate keeps a true 100 us plant period and has peak
current approximately 1.788916 A, below its 2 A base. Both cases preserve
`meets_legacy_peak=false` and `meets_legacy_last_sample=false`; the corresponding
unquantized-sensor reference peak is approximately 0.0917 Hz. Agreement with the
quantized numerical oracle does **not** make either old physical accuracy gate
pass. The coordinator approved these as explicit quantization-limit regressions;
use Q31/f32 when that legacy peak/final-sample accuracy is required. Native/Q31
legacy standards are retained. Q15 DTC is separate: its valid exact-time unit
basis did recover its old physical-unit accuracy target, and its original
quantization-bias example remains documented rather than hidden.

### First coherent fixed-Q SDK and optimized sanitizer acceptance

All three owners froze coherent source/test sets before the build agent started
SDK-root Xmake. Fresh directories were used; earlier BFP build/cache artifacts
were not overwritten. Cwd remained the real SDK root, `XMAKE_CONFIGDIR` and
build output were external, and saved configuration was loaded before asserting
resolved paths stayed under the new output root.

Native gate directory:
`/home/zmmfly/prjs/xhive_testspace/fieldctrl/fixed-q-xhive-native-YrTGBq`.
Executed configure (`-P . -y -m debug --xhive_tests=y --xhive_cross_tests=n`
with that directory's cache/build), default `xmake b`, the three library
selectors plus SDK entry, and the full `xmake test -g xhive -v` group.
**Confirmed:** 38 selected entries and the complete **54-entry** SDK group
passed. All 112 captured code/test/build-script file hashes stayed unchanged.

Optimized gate directory:
`/home/zmmfly/prjs/xhive_testspace/fieldctrl/fixed-q-xhive-sanitized-vuvnrS`.
Actual compiler commands used O2, strict aliasing, ASan/UBSan,
`-fno-sanitize-recover=all`, frame pointers and VLA/alloca errors. The selected
37-entry group passed with an unchanged 112-file snapshot and no sanitizer or
runtime-error diagnostics. Counts were parsed from each real test report:

| New fixed-Q library | Registered entries | GoogleTest passed | Skipped |
| --- | ---: | ---: | ---: |
| linears | 17 | 460 | 2 |
| filters | 10 | 278 | 0 |
| fieldctrl | 10 | 277 | 0 |
| Total | 37 | 1015 | 2 |

The two skips are the floating SVD `SmallestStoredQuantumRetainsOrthogonalDirections`
instantiations identified earlier, not missing Q coverage. These are **new
fixed-Q counts**, not the historical BFP 842/2. Passing the numerical/contract
suite includes documenting the Q15 SMO limitation above and must not be described
as restoration of every old Q15 physical-unit target.

The build agent separately executed the two Q15 SMO cases from the freshly built
host binary with GoogleTest XML output. `q15-smo-limits.xml` confirms both
positive and negative direction metrics and both false legacy-gate properties;
this is now direct build-agent evidence, not just an owner report.

Both GCC 15.2.0 and Clang 21.1.8 also accepted all **20 production C translation
units** under `-std=c99 -pedantic-errors -Wall -Wextra -Werror -Wvla -Walloca
-O2 -fstrict-aliasing`. Logs, exact output-path checks, count summaries and
before/after source manifests remain in the two new gate directories. Final
standalone/disabled-test, complete ARM fixture and paired performance gates
are still pending at this checkpoint.
