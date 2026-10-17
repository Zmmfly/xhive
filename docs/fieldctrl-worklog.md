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
| M0 | Requirements, design, validation plan, and this worklog | Review scope and accepted decisions; whitespace check | Ready for coordinating-agent review |
| M1 | Shared numerical types/core and foundation operations | Contract/quantization tests, Eigen operator references, default host build | Implementation in progress |
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
working-scalar conventions above are frozen. Foundation operations are planned
to stage results in caller-provided aligned workspace, permitting data overlap
while preserving output on failure. The workspace representation's C effective
type/strict-aliasing contract is under review; it is not yet treated as verified.

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
