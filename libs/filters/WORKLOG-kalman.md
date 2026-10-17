# Kalman implementation work log

## Scope and final state

- Implemented scalar and dense linear Kalman reset/predict/update for f32, f64, q15 and q31.
- Owned paths: `include/filters_kalman.h`, `src/kalman_scalar.c`, `src/kalman_scalar_impl.h`, `src/kalman_matrix.c`, `src/kalman_matrix_impl.h`, `test/kalman_scalar.cc`, `test/kalman_matrix.cc`, `test/kalman_support.h`, `KALMAN.md`, and this log, all relative to `libs/filters/`.
- No root/library build configuration, `filters.h`, shared test main/helper or other filter source was changed. No staging or commit was performed.
- Production is C99. q arithmetic delegates exclusively to the shared integer-only `linears_fixed_*` and q matrix/decomposition functions; test-only reference conversion uses floating point.

## Design decisions

- Separate reset, prediction and measurement update. Use scalar transition/observation coefficients and dense linear models with no unrequested control-input abstraction.
- Use Joseph covariance updates in all formats, including the scalar path. Zero residual remains a successful measurement update; noise variance must be strictly positive.
- Caller owns state, matrix data and all scratch. Stage every fallible calculation and commit estimate/covariance only after success. A separately successful predict is not rolled back by a later failed update.
- Matrix state stores explicit element capacities; sources remain ordinary `linears_matrix_t` views whose actual allocations the caller guarantees.
- Require symmetric PSD state/process covariance. Check exact stored symmetry and nonnegative diagonal, but do not claim full PSD validation. Require symmetric SPD measurement noise and verify R and S through shared Cholesky.
- Use existing partial-pivot LU solve for `S*K^T=H*P`, not an inverse and not a second numerical kernel. The first draft's proposed dedicated SPD-solve operation was replaced by the actual available Cholesky-plus-LU API.
- Symmetrize S and posterior/predicted P by `(A+transpose(A))/2` using weighted sums. This is not PSD projection.
- Workspace has three separate, correctly typed arrays: raw intermediates, `linears_workspace_unit_t` basic scratch, and typed decomposition scratch. Do not cast an entire union array to another numeric array. This supersedes the initial single-arena draft after the decomposition contract was received.
- Raw prediction storage: `n+4*n*n` elements. Update: `4*n*n+4*n*m+2*m*m+2*n+2*m`. Basic bytes and decomposition element counts have separate overflow-checked macros. Prediction ignores decomposition storage. Only required prefixes are used.
- No heap allocation, VLA, alloca, mutable static state, recursion or arbitrary dimension cap. Concurrency with shared state/scratch requires caller serialization.
- Named alignment-carrier typedefs replace the initial anonymous struct inside `offsetof`: GCC accepted the latter but parent review exposed Clang's C23-extension diagnostic. Both strict C99 compilers now pass.

## Sources checked

- Read the actual `linears.h`, `linears_numeric.h` and `linears_decomp.h` contracts before implementing their call sites.
- Retrieved `https://www.kalmanfilter.net/covUpdate.html`: the page displays and derives the Joseph covariance equation under uncorrelated prior-error/measurement-noise assumptions.
- Retrieved `https://www.kalmanfilter.net/multiSummary.html`: the equation/dimension table confirms prediction, gain, state update and Joseph covariance. The implementation selects the zero-control-input model and replaces the algebraic inverse by a solve.

## Final host results

- Scalar: **24/24 GoogleTest cases**, six typed cases across four formats.
- Matrix: **37/37 GoogleTest cases**, nine typed cases across four formats plus one workspace-macro case.
- Total: **61/61**, independently passed in GCC release and Clang AddressSanitizer/UndefinedBehaviorSanitizer builds.
- Each format runs 1,000 fixed-seed randomized scalar differential trials and a 6,000-step scalar sequence; matrix tests run 96 fixed-seed rectangular models with n,m in 1..4 and a 1,200-step sequence.
- The final matrix oracle is Eigen 3.4.0 long-double products and LDLT solve with `P-KHP`, not a second copy of Joseph code. An earlier 49-case run used a standalone Gauss-Jordan oracle; that helper was removed when the explicit Eigen requirement was clarified. Tolerances were not widened.
- Additional checks cover tiny noise below gain resolution, q block rounding/exponent updates, zero/rank-deficient PSD state covariance, invalid noise/SPD rejection, NaN/Inf, q exponent boundaries, capacities, sentinels, alias, reset, reentrancy, dimension/size errors, and atomic state after a late prediction failure.
- GCC and Clang C99 `-pedantic-errors -Wall -Wextra -Werror -Wvla` syntax checks pass for both production translation units.
- All ten owned files pass `git diff --no-index --check /dev/null <file>` with empty diagnostic output. No-index status 1 denotes the new-file difference and was accepted by the checker; an initial `set -e` wrapper stopped on that expected difference status, not on a whitespace diagnostic.

## Commands actually run

All commands ran from `/home/zmmfly/prjs/xhive_worktrees/fieldctrl`; build artifacts are outside the SDK.

### GCC release and Eigen oracle

```sh
build=/tmp/xhive-kalman.2A60yo
gtest=/home/zmmfly/.xmake/packages/g/gtest/v1.17.0/fe669f042f444a6cb3a06c810a609640
eigen=/home/zmmfly/.xmake/packages/e/eigen/3.4.0/0680b2f04c6b44f0a59425757a228017/include/eigen3
for src in libs/linears/src/numeric.c libs/linears/src/basic.c \
           libs/linears/src/decomp_lu.c libs/linears/src/decomp_cholesky.c \
           libs/filters/src/kalman_scalar.c libs/filters/src/kalman_matrix.c; do
  cc -std=c99 -pedantic-errors -Wall -Wextra -Werror -O2 -fstrict-aliasing \
     -Ilibs/filters/include -Ilibs/linears/include -c "$src" \
     -o "$build/$(basename "${src%.c}").o"
done
for test in kalman_scalar kalman_matrix; do
  c++ -std=c++17 -Wall -Wextra -Werror -O2 \
      -Ilibs/filters/include -Ilibs/linears/include -I"$gtest/include" -isystem "$eigen" \
      "libs/filters/test/$test.cc" libs/filters/test/main.cc "$build"/*.o \
      -L"$gtest/lib" -lgtest -lpthread -lm -o "$build/$test"
  "$build/$test" --gtest_color=no >"$build/release-$test.log" 2>&1
done
```

Result: compile/link and both runs succeeded. Logs:

- `/tmp/xhive-kalman.2A60yo/release-kalman_scalar.log` — 24 passed.
- `/tmp/xhive-kalman.2A60yo/release-kalman_matrix.log` — 37 passed.

### Clang ASan/UBSan

```sh
build=/tmp/xhive-kalman.2A60yo/sanitized
mkdir "$build"
# gtest and eigen are the same paths as above.
for src in libs/linears/src/numeric.c libs/linears/src/basic.c \
           libs/linears/src/decomp_lu.c libs/linears/src/decomp_cholesky.c \
           libs/filters/src/kalman_scalar.c libs/filters/src/kalman_matrix.c; do
  clang -std=c99 -pedantic-errors -Wall -Wextra -Werror -O1 -g \
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

Result: both runs succeeded without sanitizer diagnostics. Logs:

- `/tmp/xhive-kalman.2A60yo/sanitized/kalman_scalar.log` — 24 passed.
- `/tmp/xhive-kalman.2A60yo/sanitized/kalman_matrix.log` — 37 passed.

### Strict C99/no-VLA syntax

```sh
for compiler in cc clang; do
  for source in libs/filters/src/kalman_scalar.c libs/filters/src/kalman_matrix.c; do
    "$compiler" -std=c99 -pedantic-errors -Wall -Wextra -Werror -Wvla \
        -Ilibs/linears/include -Ilibs/filters/include -fsyntax-only "$source"
  done
done
```

Result: all four checks succeeded. Parent's exact Clang matrix reproduction command, including `-ferror-limit=3`, also succeeded after the alignment-carrier fix.

## Unverified and integration boundaries

- No root Xmake configuration/build/test was run here; the parent owns SDK integration.
- No MCU cross-build, hardware execution, flash, peripheral access, interrupt timing or physical tracking experiment was performed.
- PSD of arbitrary user-supplied P/Q is not validated; neither successful tests nor Joseph's real-arithmetic proof imply universal finite-precision PSD preservation or forward-error bounds.
- Arrays and descriptor allocations must actually cover declared shapes/capacities. Runtime checks cannot discover allocation extents.
- Tests are Linux host checks. Other compilers/ABIs and target floating-point environments remain unverified.
