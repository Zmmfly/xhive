# Unscented Kalman filter

## Model and algorithm

The state is `x` (n-by-1) with covariance `P` (n-by-n); the model is
`x_next = f(x) + w`, `z = h(x) + v` with zero-mean independent noise,
`cov(w)=Q` (n-by-n) and `cov(v)=R` (m-by-m). `f` and `h` are caller callbacks
that map one point to one point; `Q` and `R` are stored in the filter at init
and re-read on every call. Hyperparameters follow the scaled unscented
transform of Wan and van der Merwe:

```text
lambda  = alpha^2 * (n + kappa) - n          scale = n + lambda > 0
W_m[0]  = lambda / scale                     W_c[0] = W_m[0] + 1 - alpha^2 + beta
W_m[i]  = W_c[i] = 1 / (2 * scale)           i = 1..2n
```

Both predict and update regenerate 2n+1 sigma points from the current `x`
and `P`: `L*L^T = scale*P` (shared Cholesky), `chi_0 = x`,
`chi_i = x + L[:,i]`, `chi_{n+i} = x - L[:,i]`. Nothing is carried between
the two transactions except `x` and `P`, so either may be called any number
of times in any order.

Predict:

```text
Y_i   = f(chi_i)
x-    = mean(Y, W_m)
D_i   = Y_i (-) x-
P-    = sum_i W_c[i] * D_i * D_i^T + Q           then symmetrized
```

Update:

```text
Z_i   = h(chi_i)
z^    = mean(Z, W_m)
Dz_i  = Z_i (-) z^            Dx_i = chi_i - x  (exactly +-L[:,i])
S     = sum_i W_c[i] * Dz_i * Dz_i^T + R         symmetrized, Cholesky-checked
C     = sum_i W_c[i] * Dx_i * Dz_i^T
S K^T = C^T                                      pivoted LU solve, no inverse
x+    = x + K * (z (-) z^)
P+    = P - K * C^T                              (= P - K S K^T), symmetrized
```

`(-)` is the residual: plain subtraction when the caller passes `NULL`, or the
caller's residual callback for spaces with angle-like components. A residual
also changes the mean: `mean(Y, W) = Y_0 + sum_i W[i] * (Y_i (-) Y_0)`. For
linear components with weights summing to one this is algebraically the
weighted sum; for a wrapped angle it averages the short-way differences
relative to the propagated center point and leaves the result near `Y_0`.
`predict` takes one residual for the state space (mean and spread of the
propagated points); `update` takes one residual for the measurement space
(predicted-measurement mean, its deviations, and the innovation). Sigma points
themselves are formed by plain addition, so callbacks must accept angle
components slightly outside their normalized interval; normalizing the state
inside `f` keeps it bounded across steps.

### Source and reasoning

- E. A. Wan and R. van der Merwe, *The Unscented Kalman Filter for Nonlinear
  Estimation*, IEEE AS-SPCC 2000: scaled sigma points, the three weight values,
  and the additive-noise predict/update equations above.
- The cross-covariance uses the regenerated sigma points, whose deviations
  from `x` are exactly the columns of `+-L`. The gain solve
  `S*K^T = C^T` replaces the written inverse; `K*S*K^T = K*C^T` follows from it
  and avoids a second product.
- For a linear `f(x)=F*x`, `h(x)=H*x` the unscented transform is exact:
  `sum_i W_c[i] (F L_i)(F L_i)^T = F*P*F^T` for any `W_c[0]` because the center
  deviation is zero. The test suite uses this to cross-check the UKF against
  the existing dense linear filter.

## Ownership and contracts

All storage is caller-owned: the state object, its four arrays (`x`, `P`,
`Q`, `R`) with explicit element capacities, and three scratch arrays. `init`
writes values and weights but never changes shapes, addresses or capacities;
each source may exactly alias its destination or be the state's own
descriptor. There is no allocation, I/O, recursion, variable-length array or
mutable global state. Serialize calls sharing a state or scratch, including
task/interrupt callers; independent instances are reentrant.

A failed `init`, `predict` or `update` preserves `x`, `P` and the descriptors
(scratch may change). Every callback runs before any state write, so a
callback error is returned verbatim with the state intact. A nonfinite
floating callback output fails with `RANGE`.

`P` must be exactly symmetric with nonnegative diagonal (`DOMAIN` otherwise)
and positive definite at working precision: a singular positive-semidefinite
`P` has no Cholesky factor and fails with `NOT_POSITIVE_DEFINITE` in predict
and update. `Q` must be symmetric PSD (symmetry and diagonal signs checked);
`R` must be symmetric and is Cholesky-checked before any callback runs,
because `S` can be definite while a supplied `R` is not. `Q` and `R` contents
are validated on every call, so a caller may rewrite them between calls for
time-varying noise; their descriptors and capacities stay fixed.

`W_c[0]` is negative for small `alpha`, which can make a computed `P` indefinite
even for a valid model; the next Cholesky then fails. Choose `alpha`, `beta`,
`kappa` so that `scale > 0` (`alpha > 0`, `n + kappa > 0`); `alpha = 1`,
`beta = 2`, `kappa = 0` or `3 - n` are conventional starting points.

## Working-format kernels

The steps are public kernels so particle and unscented particle filters can
reuse them with their own point sets and weights. They operate on working
matrices (float for the `f32` kernels, double for `f64`; the q15 filters run
on the `f32` kernels and q31 on `f64`), one point per row:

| Kernel | Computes |
| --- | --- |
| `weights(n, alpha, beta, kappa)` | `scale`, `W_m[0]`, `W_c[0]`, `W_i` |
| `sigma_points(x, P, scale, points)` | (2n+1)-by-n rows via Cholesky of `scale*P` |
| `propagate(points, fn, context, out)` | row-wise `out_i = fn(points_i)` |
| `mean(points, w, residual, context, out)` | weighted mean, residual-aware |
| `deviations(points, mean, residual, context, out)` | row-wise `points_i (-) mean` |
| `weighted_covariance(a, b, w, out)` | `a^T * diag(w) * b` |
| `predict_work(x, P, Q, weights, f, residual, out_x, out_P)` | the complete predict step on working matrices |
| `update_work(x, P, R, z, weights, h, residual, out_x, out_P)` | the complete update step on working matrices |

`weighted_covariance` with `a == b` is a covariance; with two deviation sets
it is a cross covariance; noise is added by the caller. Weights are an
s-by-1 working vector of any sign; the residual form of `mean` assumes they
sum to one. `predict_work` and `update_work` compose the smaller kernels plus
the shared `linears` add/transpose/axpby/Cholesky/solve operations; the
signal-format `predict` and `update` only validate the state storage, import
Q signals, call the kernel and commit. Their storage macros are therefore the
kernel amount (`FILTERS_UKF_PREDICT_WORK_STORAGE`,
`FILTERS_UKF_UPDATE_WORK_STORAGE`) plus `n + n*n` output staging and, for Q,
the imported inputs.

## Fixed-fraction Q contract

- q15/q31 state and noise arrays hold raw `int16_t/2^15` and `int32_t/2^31`
  signals; init copies them unchanged. Every persistent array is stored in
  the signal type; nothing is widened for storage.
- Intermediates need range beyond the signal interval, so the q15 filters
  compute in float and the q31 filters in double: the significand (24 and
  53 bits) covers the signal (15 and 31 bits), and the floating range
  absorbs sigma points and weights that exceed `[-1,+1]`. There is no
  separate fixed working type; q15 uses the `f32` kernels, callbacks,
  weights, workspace and `float alpha/beta/kappa`, q31 the `f64` ones.
- Signals widen exactly once on entry (`x`, `P`, `Q`, `R`, `z` into the
  working scratch). Only the final `x` and `P` are range-checked against
  `[-1,+1]` on their computed working values and narrowed once, with the
  positive-endpoint policy of `linears_numeric.h` (exact +1 and the rounding
  cell reaching it map to MAX = 1-LSB, ties away from zero). The check
  applies to the computed working result, not to an ideal unrounded
  expression, as for the dense linear filter.
- Both outputs are checked before either is written, so a `RANGE` leaves the
  state unchanged.
- Positive noise must remain positive after quantization. A zero `R` fails the
  Cholesky check with `NOT_POSITIVE_DEFINITE`.

## Workspace and RAM cost

Each call borrows three real arrays of the declared element types. Size them
with the macros below (constant expressions; `SIZE_MAX` means overflow and is
rejected, never allocated). Only required prefixes are accessed. `s = 2n+1`.

| Field | Element type | Predict | Update |
| --- | --- | --- | --- |
| float `data` | float/double | `FILTERS_UKF_PREDICT_STORAGE(n)` = `8n²+8n+2` | `FILTERS_UKF_UPDATE_STORAGE(n,m)` = `7n²+2m²+10nm+7n+5m+2+s·max(n,m)` |
| Q `data` | float for q15, double for q31 | `FILTERS_UKF_Q_PREDICT_STORAGE(n)` = `10n²+9n+2` | `FILTERS_UKF_Q_UPDATE_STORAGE(n,m)` = float count `+ n²+m²+n+m` |
| `basic` | `linears_workspace_unit_t` | `FILTERS_UKF_PREDICT_BASIC_BYTES(n)` (`n·s` units) | `FILTERS_UKF_UPDATE_BASIC_BYTES(n,m)` (`max(n,m)·s` units) |
| `decomposition` | float for f32/q15, double for f64/q31 | `FILTERS_UKF_PREDICT_DECOMP_STORAGE(n)` = `n²` | `FILTERS_UKF_UPDATE_DECOMP_STORAGE(n,m)` = `max(n², m²+mn)` |

The kernels have their own macros (`FILTERS_UKF_SIGMA_POINTS_*`,
`FILTERS_UKF_MEAN_*`, `FILTERS_UKF_DEVIATIONS_BASIC_BYTES`,
`FILTERS_UKF_WEIGHTED_COVARIANCE_*`). Inside `predict`/`update` every kernel
borrows dead blocks adjacent to its inputs, so the filter totals above already
include the kernels' needs. For `n=2, m=1` a q15 update needs 89 float
working elements, 10 basic units and 4 float decomposition elements; the
state adds 2+4+4+1 raw q15 elements (22 bytes).

A C99 f32 pendulum filter with an angle residual:

```c
#include "filters_ukf.h"
#include <math.h>

static linears_status_t pendulum(const linears_matrix_t *in, linears_matrix_t *out, void *context)
{
    const float *x = (const float *)in->data, dt = *(const float *)context;
    float *y = (float *)out->data;
    y[0] = x[0] + dt * x[1];
    y[1] = x[1] - dt * 9.81f * sinf(x[0]);
    return LINEARS_OK;
}

static linears_status_t angle_only(const linears_matrix_t *in, linears_matrix_t *out, void *context)
{
    (void)context;
    ((float *)out->data)[0] = ((const float *)in->data)[0];
    return LINEARS_OK;
}

static linears_status_t wrap_angle(const linears_matrix_t *a, const linears_matrix_t *b,
                                   linears_matrix_t *out, void *context)
{
    float d = ((const float *)a->data)[0] - ((const float *)b->data)[0];
    (void)context;
    while (d >= 3.14159265f) d -= 6.2831853f;
    while (d < -3.14159265f) d += 6.2831853f;
    ((float *)out->data)[0] = d;
    return LINEARS_OK;
}

int main(void)
{
    float x[2] = {3.0f, 0.0f}, p[4] = {0.1f, 0, 0, 0.1f}, q[4] = {1e-4f, 0, 0, 1e-3f}, r[1] = {0.05f};
    float z_value = -3.1f, dt = 0.01f;
    float data[FILTERS_UKF_UPDATE_STORAGE(2, 1)];
    linears_workspace_unit_t basic[LINEARS_WORKSPACE_UNITS(FILTERS_UKF_UPDATE_BASIC_BYTES(2, 1))];
    float decomposition[FILTERS_UKF_UPDATE_DECOMP_STORAGE(2, 1)];
    filters_ukf_f32_workspace_t workspace = {
        data, FILTERS_UKF_UPDATE_STORAGE(2, 1), basic, sizeof basic,
        {decomposition, FILTERS_UKF_UPDATE_DECOMP_STORAGE(2, 1)}
    };
    filters_ukf_f32_t filter = {{x, 2, 1}, {p, 2, 2}, {q, 2, 2}, {r, 1, 1}, 2, 4, 4, 1, {0, 0, 0, 0}};
    linears_matrix_t z = {&z_value, 1, 1};
    linears_status_t status = filters_ukf_f32_init(&filter, 1.0f, 2.0f, 0.0f,
        &filter.estimate, &filter.covariance, &filter.process_noise, &filter.measurement_noise);
    if (status == LINEARS_OK)
        status = filters_ukf_f32_predict(&filter, pendulum, &dt, NULL, NULL, workspace);
    if (status == LINEARS_OK)
        status = filters_ukf_f32_update(&filter, &z, angle_only, NULL, wrap_angle, NULL, workspace);
    return status == LINEARS_OK ? 0 : 1;
}
```

The predict workspace requirement does not exceed the update requirement for
these dimensions, so one update-sized workspace serves both calls here; in
general size each array by the larger of the two macros.

## Precision and limitations

`f32`/`f64` keep native precision and reject nonfinite values. Full positive
definiteness is established by the Cholesky factor at working precision, not
an eigendecomposition. Symmetrization is `(A + A^T)/2`, not projection onto
the PSD cone; finite-precision results are not guaranteed PSD. There is no
square-root/augmented UKF, no noise-free or additive-control variant, no
automatic regularization, and no iteration of the manifold mean beyond the
single center-relative pass described above.

## Validation

Host tests live in `test/ukf.cc` and run for all four formats
(`xmake test -g xhive -v 'filters_test_ukf/ukf'`, 33 GoogleTest cases, 31
pass and the 2 floating instances of the Q-only range case are skipped by
design). Oracles are independent of the production code:

- Weights against the closed form, including `sum W_m = 1`, and rejection of
  `alpha <= 0`, `n + kappa <= 0`, zero `n`, NULL and nonfinite parameters.
- 36 random linear models (`n,m = 1..3`) against `filters_kalman_matrix_*`
  predict and update, with `alpha = 1`, `beta = 2`, `kappa = 3 - n`.
- A 60-step pendulum (`f` nonlinear, `h = [sin(theta), omega]`) against a
  long-double unscented reference using Eigen 3.4.0 LLT/LDLT, reseeded from the
  committed state each transaction; the posterior stays within four sigma of
  the simulated truth.
- A wrapped angle crossing +-pi (radians) or +-1 (`theta/pi` for Q): with the
  residual the predicted mean and variance match the closed form and the
  innovation takes the short way; without it the floating mean lands near zero
  and the Q covariance (about 2.9) leaves the signal interval and fails
  atomically with `RANGE`.
- Manual composition of the kernels reproduces `predict`; `predict_work` and
  `update_work` reproduce the filter transactions from the same prior; the
  same kernels compute the mean and cross covariance of seven uniformly
  weighted points.
- Init validation, capacities, exact/own-descriptor alias acceptance, partial
  and cross alias rejection, workspace bounds and misalignment, callback and
  residual error propagation, nonfinite callback output, semidefinite `P`,
  indefinite `R` before any callback, edited-noise re-validation, zeroed
  weights, kernel alias rules, Q sigma points above +1 with in-range results,
  and an out-of-range Q result rejected atomically. Every failure path checks
  that the state object and all four arrays are byte-identical afterwards.
- Size macros as constant expressions with `SIZE_MAX` overflow sentinels.

All comparisons use the unchanged per-format envelopes of `KALMAN.md`
(`tolerance * (1 + |reference|)`, f32 `2e-5`, f64 `2e-12`, q15 `4e-3`,
q31 `2e-7`) without multipliers; the q31 envelope holds with double working
arithmetic. Additional runs: GCC `-O2` release build of the production
objects and test, and a Clang `-fsanitize=address,undefined` build with
`-fno-sanitize-recover=undefined`, both 31 passed (2 float-only skips) with
no sanitizer diagnostics; `ukf.c` passes GCC and Clang
`-std=c99 -pedantic-errors -Wall -Wextra -Werror -Wvla -fsyntax-only`. The
example above compiles and returns zero with both compilers. Host simulation
is not MCU, hardware or interrupt-timing validation.
