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
| M1b | Foundation matrix/vector operations | Four-variant Eigen references and failure tests, default host build | Implementation in progress |
| M2 | LU/solve/inverse/determinant, Cholesky, QR, LDLT, SVD | Per-family four-variant numerical and failure tests; host build | Implementation in progress |
| M3 | Filters including scalar and matrix Kalman | Per-family reference/long-sequence/state tests; host build | Implementation in progress |
| M4 | Control foundations, PI/PWM, VVVF | Transform/formula tests, limit/direction/state tests; host build | Implementation in progress |
| M5 | FOC and SMO | Four-variant software control/observer tests and stated observability limits | Implementation in progress |
| M6 | DTC | Flux/torque/sector/hysteresis/table tests and software plant scenarios | Planned |
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
  scales by the maximum magnitude and may omit contributions below its stated
  rounding threshold.

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
