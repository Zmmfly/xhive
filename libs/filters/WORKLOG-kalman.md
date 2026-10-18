# Kalman implementation work log

## Fixed-fraction migration scope

- Baseline `ab11032` was clean; all ten owned files matched the previous frozen hashes before this breaking migration.
- Migrated scalar and dense linear Kalman reset/predict/update to raw fixed-F15/F31 signals, retaining f32/f64 behavior.
- The parent approved the composite F31-working matrix contract. The sub-working-LSB boundary below is now an explicit exact regression, not an unresolved behavior.
- Owned paths: `include/filters_kalman.h`, `src/kalman_scalar.c`, `src/kalman_scalar_impl.h`, `src/kalman_matrix.c`, `src/kalman_matrix_impl.h`, `test/kalman_scalar.cc`, `test/kalman_matrix.cc`, `test/kalman_support.h`, `KALMAN.md`, and this log, all relative to `libs/filters/`.
- No root/library build configuration, `filters.h`, shared test main/helper or other filter source was changed. No staging or commit was performed.
- Production is C99. Q arithmetic delegates exclusively to shared integer F31/F62 helpers and the selected `linears_work.h` matrix/decomposition operations. Floating conversion appears only in host test references.

## Design decisions

- Separate reset, prediction and update; do not add control-input abstractions. Q scalar state/signals/noise use raw int16_t/int32_t, while transition/observation use public wide F31 `linears_fixed_t` so exact unity and gain two remain expressible. Matrix F/H remain raw Q and are imported exactly for computation.
- Use Joseph covariance updates in all formats, including the scalar path. Zero residual remains a successful measurement update; noise variance must be strictly positive.
- Caller owns state, matrix data and all scratch. Stage every fallible calculation and commit estimate/covariance only after success. A separately successful predict is not rolled back by a later failed update.
- Matrix state stores explicit element capacities; sources remain ordinary `linears_matrix_t` views whose actual allocations the caller guarantees.
- Require symmetric PSD state/process covariance. Check exact stored symmetry and nonnegative diagonal, but do not claim full PSD validation. Require symmetric SPD measurement noise and verify R and S through shared Cholesky.
- Use shared Cholesky plus partial-pivot LU solve for `S*K^T=H*P`, never inverse. Q uses the approved working fixed-F31 entry points, including exact working identity; there is no filters-local multiplication/factorization kernel.
- Symmetrize S and posterior/predicted P by `(A+transpose(A))/2` using weighted sums. This is not PSD projection.
- Three separate typed arrays remain: floating/native or Q/int64-F31 intermediates, workspace-unit basic scratch, and typed decomposition scratch. Q final x/P values are staged through individual declared `unit.q15/q31` members, never by casting an entire union array. No hidden fourth buffer.
- Floating storage/macros remain unchanged. New Q working counts are `2*n+7*n*n` for prediction and `5*n*n+5*n*m+3*m*m+3*n+3*m` for update, including imported signal matrices. Q basic units are `n+n*n` for prediction and `max(n*n,n*m,m*m,n+n*n)` for update. Decomposition remains `m*m+m*n` elements; prediction ignores it.
- Scalar Q final expressions retain F62 products through direct signal export, including exact out-of-range results below one F31 LSB. Gain uses shared `fixed_muldiv` rather than prematurely rounding its numerator.
- Narrowing follows shared nearest-away rounding and closed-input-interval policy: exact +1 maps to MAX (still 1-LSB); values outside [-1,+1] at the checked conversion point fail. Zero raw R is rejected, including a positive real variance rounded to raw zero.
- No heap allocation, VLA, alloca, mutable static state, recursion or arbitrary dimension cap. Concurrency with shared state/scratch requires caller serialization.
- Existing named alignment carriers remain strictly C99; both GCC and Clang checks pass.

## Sources checked

- Read the new `linears.h`, `linears_numeric.h`, `linears_decomp.h` and approved eight-operation `linears_work.h` contracts before implementing call sites.
- Retrieved `https://www.kalmanfilter.net/covUpdate.html`: the page displays and derives the Joseph covariance equation under uncorrelated prior-error/measurement-noise assumptions.
- Retrieved `https://www.kalmanfilter.net/multiSummary.html`: the equation/dimension table confirms prediction, gain, state update and Joseph covariance. The implementation selects the zero-control-input model and replaces the algebraic inverse by a solve.

## Observed host results

- Scalar: **28/28 GoogleTest cases**, seven typed cases across four formats.
- Matrix: **42/42 GoogleTest cases**, ten typed cases across four formats, one workspace-macro case and one exact q31 boundary case.
- Total: **70/70**, passed after the boundary/documentation changes in freshly rebuilt GCC release and Clang AddressSanitizer/UndefinedBehaviorSanitizer builds without sanitizer diagnostics.
- Each format runs 1,000 fixed-seed randomized scalar trials and a 6,000-step sequence; matrix tests run 96 rectangular models with n,m in 1..4 and a 1,200-step sequence.
- Eigen 3.4.0 long-double products/LDLT and independent `P-KHP` remain the matrix oracle. It uses actual decoded raw inputs. Scalar/matrix comparisons convert the same raw F/H to wide scalar coefficients; they do not replace MAX with unity. Existing tolerance values and long-sequence multipliers were not widened.
- Q models were replaced only where the old values were not representable. Test encoding rejects values outside [-1,+1] rather than silently clipping them; wide coefficients have a separate test conversion.
- Tests cover exact unity, observation gain two, K/S above one with valid final states, 400 H=0 updates with no covariance decay, zero/rank-deficient PSD state, raw endpoints/MAX semantics, minimum positive/rounded-zero noise, noise SPD, NaN/Inf, capacity sentinels, alias, reset/reentrancy and atomic late output failures.
- Scalar regression additionally rejects exact products `+/- (1+2^-33)` obtained from x=1/4 and wide coefficient `+/- (4+2^-31)`. The n=5 matrix regression independently proves its exact F62 dot, nearest F31 ONE, final MAX success and both direct F62/Q-matmul RANGE results with integer equalities, never tolerance.
- Both production files pass GCC and Clang C99 `-pedantic-errors -Wall -Wextra -Werror -Wvla` syntax checks.
- Scoped `git diff --check` passed. Both C99 code examples extracted from `KALMAN.md` compiled and ran successfully with GCC and Clang.

## Commands actually run

All commands ran from `/home/zmmfly/prjs/xhive_worktrees/fieldctrl`; build artifacts are outside the SDK.

### GCC release and Eigen oracle

```sh
build=/tmp/xhive-kalman-fixed.bmBYY1/release
mkdir "$build"
gtest=/home/zmmfly/.xmake/packages/g/gtest/v1.17.0/fe669f042f444a6cb3a06c810a609640
eigen=/home/zmmfly/.xmake/packages/e/eigen/3.4.0/0680b2f04c6b44f0a59425757a228017/include/eigen3
for src in libs/linears/src/numeric.c libs/linears/src/basic.c \
           libs/linears/src/decomp_lu.c libs/linears/src/decomp_cholesky.c \
           libs/linears/src/decomp_work.c libs/filters/src/kalman_scalar.c \
           libs/filters/src/kalman_matrix.c; do
  cc -std=c99 -pedantic-errors -Wall -Wextra -Werror -Wvla -O2 -fstrict-aliasing \
     -Ilibs/filters/include -Ilibs/linears/include -c "$src" \
     -o "$build/$(basename "${src%.c}").o"
done
for test in kalman_scalar kalman_matrix; do
  c++ -std=c++17 -Wall -Wextra -Werror -O2 \
      -Ilibs/filters/include -Ilibs/linears/include -I"$gtest/include" -isystem "$eigen" \
      "libs/filters/test/$test.cc" libs/filters/test/main.cc "$build"/*.o \
      -L"$gtest/lib" -lgtest -lpthread -lm -o "$build/$test"
  "$build/$test" --gtest_color=no >"$build/$test.log" 2>&1
done
```

Result: compile/link and both runs succeeded. Logs:

- `/tmp/xhive-kalman-fixed.bmBYY1/release/kalman_scalar.log` — 28 passed.
- `/tmp/xhive-kalman-fixed.bmBYY1/release/kalman_matrix.log` — 42 passed.

### Clang ASan/UBSan

```sh
build=/tmp/xhive-kalman-fixed.bmBYY1/sanitized
mkdir "$build"
# gtest and eigen are the same paths as above.
for src in libs/linears/src/numeric.c libs/linears/src/basic.c \
           libs/linears/src/decomp_lu.c libs/linears/src/decomp_cholesky.c \
           libs/linears/src/decomp_work.c libs/filters/src/kalman_scalar.c \
           libs/filters/src/kalman_matrix.c; do
  clang -std=c99 -pedantic-errors -Wall -Wextra -Werror -Wvla -O1 -g \
        -fno-omit-frame-pointer -fsanitize=address,undefined \
        -Ilibs/filters/include -Ilibs/linears/include -c "$src" \
        -o "$build/$(basename "${src%.c}").o"
done
for test in kalman_scalar kalman_matrix; do
  clang++ -std=c++17 -Wall -Wextra -Werror -O1 -g \
          -fno-omit-frame-pointer -fsanitize=address,undefined \
          -Ilibs/filters/include -Ilibs/linears/include -I"$gtest/include" -isystem "$eigen" \
          "libs/filters/test/$test.cc" libs/filters/test/main.cc "$build"/*.o \
          -L"$gtest/lib" -lgtest -lpthread -lm -o "$build/$test"
  ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1 \
      "$build/$test" --gtest_color=no >"$build/$test.log" 2>&1
done
```

Result: both runs succeeded without sanitizer diagnostics. After adding the
exact n=5 regression, all seven production/dependency objects and both test
binaries were rebuilt again in the existing release/sanitized directories
(without repeating mkdir), with the same flags. The logs below contain the
final 28/42 counts, not the earlier 28/41 runs.

Logs:

- `/tmp/xhive-kalman-fixed.bmBYY1/sanitized/kalman_scalar.log` — 28 passed.
- `/tmp/xhive-kalman-fixed.bmBYY1/sanitized/kalman_matrix.log` — 42 passed.

### Strict C99/no-VLA syntax

```sh
for compiler in cc clang; do
  for source in libs/filters/src/kalman_scalar.c libs/filters/src/kalman_matrix.c; do
    "$compiler" -std=c99 -pedantic-errors -Wall -Wextra -Werror -Wvla \
        -Ilibs/linears/include -Ilibs/filters/include -fsyntax-only "$source"
  done
done
```

Result: all four checks succeeded.

### Documentation examples

The two C blocks from `KALMAN.md` were combined under `main()` in
`/tmp/xhive-kalman-fixed.bmBYY1/doc-examples.c`, with the actual scalar status
and workspace capacity checked by the return value. Commands actually run:

```sh
for compiler in cc clang; do
  "$compiler" -std=c99 -pedantic-errors -Wall -Wextra -Werror -Wvla \
    -Ilibs/linears/include -Ilibs/filters/include \
    /tmp/xhive-kalman-fixed.bmBYY1/doc-examples.c \
    /tmp/xhive-kalman-fixed.bmBYY1/release/*.o -lm \
    -o "/tmp/xhive-kalman-fixed.bmBYY1/doc-examples-$compiler"
  "/tmp/xhive-kalman-fixed.bmBYY1/doc-examples-$compiler"
done
```

Both executables returned zero. No target or hardware execution was involved.

## Resolved matrix endpoint contract

A confirmed q31 probe exposes the distinction between checked final F31
working values and the ideal unrounded dot product. For n=5, x is all 1/4,
F's first row is `[MAX,MAX,MAX,MAX,raw5]` and remaining rows are zero, with
P=Q=0. The exact first dot is `1+2^-33`; `mul_fixed` rounds it to working ONE,
so the current Kalman path narrows to MAX. Direct raw `linears_mul_q31`, whose
last export sees F62, instead returns RANGE. The executable printed:

```text
reset=0 predict=0 estimate0=2147483647 direct_mul=-6 direct0=123
```

The probe source/binary is `/tmp/xhive-kalman-fixed.bmBYY1/boundary_probe.c`
and `boundary_probe`; it was compiled with strict C99 and linked to the same
release objects. The parent explicitly chose to retain the composite F31
working contract and not add a mixed-output/interval API. That decision is
implemented as `KalmanMatrixQ31.WorkingRoundingBoundaryDiffersFromDirectSignalContract`
and documented in every Q matrix operation's public contract and `KALMAN.md`.
Matrix RANGE refers to the computed/rounded F31 state, not the ideal unrounded
full expression. Working-rounding loss and the separate positive-endpoint
one-signal-LSB mapping error are explicitly distinguished. Scalar direct
final-F62 checks were not weakened. No local matrix kernel or tolerance change
was introduced; the 1.5 and negative out-of-range state tests still reject
atomically.

## Unverified and integration boundaries

- No root Xmake configuration/build/test was run here; the parent owns SDK integration.
- No MCU cross-build, hardware execution, flash, peripheral access, interrupt timing or physical tracking experiment was performed.
- PSD of arbitrary user-supplied P/Q is not validated; neither successful tests nor Joseph's real-arithmetic proof imply universal finite-precision PSD preservation or forward-error bounds.
- Arrays and descriptor allocations must actually cover declared shapes/capacities. Runtime checks cannot discover allocation extents.
- Tests are Linux host checks. Other compilers/ABIs and target floating-point environments remain unverified.
