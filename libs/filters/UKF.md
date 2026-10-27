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
S K^T = C^T                                      solve, no inverse
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
`Q`, `R`) with explicit element capacities, and typed scratch arrays. Floating
workspaces have `data`, `basic` and `decomposition`; Q workspaces additionally
have raw `signal` staging. `init` writes values and weights but never changes
shapes, addresses or capacities;
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

The floating steps are public kernels so particle and unscented particle
filters can reuse them with their own point sets and weights. The `f32` and
`f64` kernels operate on float/double working matrices, one point per row:

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
sum to one. The floating `predict_work` and `update_work` share private
arithmetic cores with those kernels. Public entries validate shapes, aliasing
and capacities before calling the cores; small matrix operations use direct
loops rather than repeatedly registering the same scratch views. Cholesky
uses a private factor routine with the same multiply/subtract order as
`linears`; the floating gain still uses its pivoted `solve`.

The Q `predict_work` and `update_work` entries are distinct mixed-format
kernels: their small input/output matrices are float for q15 or double for
q31, but their statistics use Q integer multiply-accumulate paths. They do
not delegate the entire step to `filters_ukf_f32_*`/`filters_ukf_f64_*`.
This interface lets the Q UPF pass a temporary prior outside `[-1,+1]` to
its update without first quantizing that prior into persistent signal
storage. Only results committed through the signal-format filter API must
fit the raw signal range. Working-kernel outputs may be partially written
on failure; the signal-format state wrapper supplies the atomic commit.

## Fixed-fraction Q contract

- q15/q31 state and noise arrays hold raw `int16_t/2^15` and `int32_t/2^31`
  signals; init copies them unchanged. Every persistent array is stored in
  the signal type; nothing is widened for storage.
- Q is a computation format, not only compressed storage. The dedicated
  Q kernels use integer statistics; float (q15) or double (q31) is reserved
  for callbacks, decompositions and local values needing wider range.
  Floating significands of 24/53 bits cover the 15/31-bit signal precision.
  Model/residual callbacks, sigma weights and `alpha/beta/kappa` keep their
  float/double interfaces. Sigma points, callback outputs and residuals may
  exceed `[-1,+1]`; they are not silently clipped into a raw Q vector.
- Final `x` and `P` are checked against `[-1,+1]` on the computed candidate
  values before commit, with the positive-endpoint policy of
  `linears_numeric.h` (exact +1 and the rounding cell reaching it map to
  MAX = 1-LSB, ties away from zero). This is a check of the implemented
  finite-precision result, not an ideal unrounded expression. Intermediate
  Q quantization and final signal conversion are separate error sources.
- Both outputs are checked before either is written, so a `RANGE` leaves the
  state unchanged.
- Positive noise must remain positive after quantization. A zero `R` fails the
  Cholesky check with `NOT_POSITIVE_DEFINITE`.

### Integer and floating boundaries

[`ukf.c`](src/ukf.c) instantiates
[`ukf_q_work_impl.h`](src/ukf_q_work_impl.h) separately for q15/q31. Mean
sums quantize offsets from the original first point, not that point's
absolute value; a sub-LSB cloud therefore keeps its unrounded center.
Covariance statistics encode bounded deviation/factor columns at native Q
precision. q15 products accumulate in `int64_t`; q31 sums use the portable
two-word accumulator. These are transient accumulators, not wider
persistent signal arrays or a new fixed-point state format.

The integer mean path requires a weighted conversion-error bound of at
most half a signal LSB; covariance/cross-covariance entries require a bound
of at most one LSB. An offset/deviation/factor column outside `[-1,+1]`,
one exceeding that error bound, or one with a nonzero value that would
quantize to zero keeps a local floating sum/product. A covariance or cross
entry also stays floating when both participating columns are nonzero and
the product of their maximum magnitudes is below one signal LSB, even if
each deviation quantizes to a nonzero raw value. An absolute one-LSB error
budget alone cannot resolve the relative spread of such a small cloud.
These local choices preserve small working covariances, including clouds
around a nonzero center; they do not restore information already lost when
a persistent covariance was encoded as raw zero. No sample is clipped to
enable the integer path. Residual-aware means accumulate the caller's
float/double residuals locally. None of these cases calls the complete
f32/f64 UKF kernel.

Cholesky, gain-row triangular solves and final corrections use float/double.
The Q path retains real integer MAC work without claiming that sigma
generation, manifold callbacks or all matrix operations are integer-only.

### Computation changes

These changes are visible in the dedicated Q source, independently of any
host timing result:

- A d-dimensional covariance computes `d(d+1)/2` entries and mirrors them,
  instead of accumulating the full d-by-d matrix.
- Integer cross-covariance MACs use n paired differences of the regenerated
  `+L`/`-L` sigma points, instead of a `2n+1`-term general point-cloud dot.
- The Q path avoids expanded weight vectors and matrix transposes used by
  the general floating composition.
- The update reuses the Cholesky factor of `S` for gain-row triangular
  solves, rather than factoring `S` again with LU or storing a full
  transposed gain matrix.
- On verified IEEE layouts, finite magnitude comparisons and the exact
  error between a working value and its quantized Q value use integer bit
  operations. The weighted error sum and the decision budget are unchanged;
  other layouts retain the native floating expressions.

These reduce repeated arithmetic and memory passes; they do not establish
that an integer operation is faster than an FPU operation on a given MCU.
Use the [measurement tools](README.md#measurements) for measured comparisons.

## Migrating Q callers

Q filter calls take `filters_ukf_q15_workspace_t` or
`filters_ukf_q31_workspace_t`, not a `filters_ukf_f32_workspace_t` or
`filters_ukf_f64_workspace_t`. Do not cast the old workspace to the new type.
The Q workspace fields are:

- `data` and `capacity`: local float/double work and its element count.
- `basic` and `basic_bytes`: `linears_workspace_unit_t` storage and its byte
  count.
- `decomposition`: a float/double `linears` decomposition workspace.
- `signal` and `signal_capacity`: raw `int16_t`/`int32_t` staging and its
  element count.

The current Q UKF kernels factor into `data` and do not write the
`decomposition` array. Its public capacity and aliasing requirements are
still validated: callers must continue supplying the macro-sized buffer.
Other users of the same workspace type, including PF initialization, still
use it for decomposition.

Existing `*_Q_*_STORAGE` macros size `data`; the separate
`*_Q_*_SIGNAL_STORAGE` macros size `signal`. Recompute both capacities for
all calls sharing a workspace. Persistent arrays, callback signatures and
hyperparameter types do not change. The f32/f64 workspace and call pattern
below are unchanged.

## Workspace and RAM cost

Size each array with the macros in [`filters_ukf.h`](include/filters_ukf.h).
They are constant expressions; `SIZE_MAX` signals overflow and is not an
allocation request. Only required prefixes are accessed, and a zero required
prefix accepts `NULL`. Below, `s = 2n+1` and `k = max(n,m)`.

For f32/f64, `data` and `decomposition` contain float/double:

| Field | Predict | Update |
| --- | --- | --- |
| `data` elements | `FILTERS_UKF_PREDICT_STORAGE(n)` = `8n²+8n+2` | `FILTERS_UKF_UPDATE_STORAGE(n,m)` = `7n²+2m²+10nm+7n+5m+2+s·k` |
| `basic` bytes | `FILTERS_UKF_PREDICT_BASIC_BYTES(n)` (`n·s` units) | `FILTERS_UKF_UPDATE_BASIC_BYTES(n,m)` (`k·s` units) |
| `decomposition` elements | `FILTERS_UKF_PREDICT_DECOMP_STORAGE(n)` = `n²` | `FILTERS_UKF_UPDATE_DECOMP_STORAGE(n,m)` = `max(n²,m²+mn)` |

For q15/q31, `data` and `decomposition` contain float/double; `signal`
contains `int16_t`/`int32_t`. Use the Q-specific macros for **every** field,
not the floating basic/decomposition requirements:

| Field | Predict | Update |
| --- | --- | --- |
| `data` elements | `FILTERS_UKF_Q_PREDICT_STORAGE(n)` = `7n²+8n` | `FILTERS_UKF_Q_UPDATE_STORAGE(n,m)` = `2k²+2n²+m²+3nm+6n+9m` |
| `signal` elements | `FILTERS_UKF_Q_PREDICT_SIGNAL_STORAGE(n)` = `s·n` | `FILTERS_UKF_Q_UPDATE_SIGNAL_STORAGE(n,m)` = `n²+s·m` |
| `basic` bytes | `FILTERS_UKF_Q_PREDICT_BASIC_BYTES(n)` = `0` | `FILTERS_UKF_Q_UPDATE_BASIC_BYTES(n,m)` = `0` |
| `decomposition` elements | `FILTERS_UKF_Q_PREDICT_DECOMP_STORAGE(n)` = `n²` | `FILTERS_UKF_Q_UPDATE_DECOMP_STORAGE(n,m)` = `max(n²,m²)` |

The mixed-format work entries use `FILTERS_UKF_Q_PREDICT_WORK_STORAGE(n)`
(`4n²+6n`) or `FILTERS_UKF_Q_UPDATE_WORK_STORAGE(n,m)`
(`2k²+3nm+4n+8m`) for `data`; their other requirements are the same as the Q
state APIs. Their float/double input/output matrices are separate from that
scratch. Floating kernels likewise have their own `*_WORK_STORAGE` and
per-kernel macros; do not substitute them for state-transaction totals.

For `n=2, m=1`, a q15 update needs 44 float `data` elements, 9 raw q15
`signal` elements and 4 float decomposition elements, with no basic scratch:
210 bytes of workspace arrays. Sharing scratch with predict requires 10 raw
elements instead of 9, or 212 bytes. Persistent `x`, `P`, `Q`, `R` add
2+4+4+1 raw q15 elements (22 bytes); these counts exclude state/workspace
objects and call-stack storage.

### f32 example

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

### q15 typed-workspace example

This C99 scalar identity model allocates raw persistent state and separate
float/raw scratch. Its callbacks still receive float, not `int16_t` views:

```c
#include "filters_ukf.h"
#include <stdint.h>

static linears_status_t identity(const linears_matrix_t *in,
                                 linears_matrix_t *out, void *context)
{
    (void)context;
    ((float *)out->data)[0] = ((const float *)in->data)[0];
    return LINEARS_OK;
}

int main(void)
{
    int16_t x[] = {INT16_C(8192)}, p[] = {INT16_C(3277)};
    int16_t q[] = {INT16_C(328)}, r[] = {INT16_C(1638)};
    int16_t z_value = INT16_C(9830);
    float data[FILTERS_KALMAN_MAX_SIZE(FILTERS_UKF_Q_PREDICT_STORAGE(1),
                                      FILTERS_UKF_Q_UPDATE_STORAGE(1, 1))];
    int16_t signal[FILTERS_KALMAN_MAX_SIZE(FILTERS_UKF_Q_PREDICT_SIGNAL_STORAGE(1),
                                          FILTERS_UKF_Q_UPDATE_SIGNAL_STORAGE(1, 1))];
    float decomposition[FILTERS_KALMAN_MAX_SIZE(FILTERS_UKF_Q_PREDICT_DECOMP_STORAGE(1),
                                               FILTERS_UKF_Q_UPDATE_DECOMP_STORAGE(1, 1))];
    filters_ukf_q15_workspace_t workspace = {
        .data = data,
        .capacity = sizeof data / sizeof data[0],
        .basic = NULL, .basic_bytes = 0, /* Both Q UKF basic-byte macros are zero. */
        .decomposition = {decomposition, sizeof decomposition / sizeof decomposition[0]},
        .signal = signal,
        .signal_capacity = sizeof signal / sizeof signal[0]
    };
    filters_ukf_q15_t filter = {
        {x, 1, 1}, {p, 1, 1}, {q, 1, 1}, {r, 1, 1},
        1, 1, 1, 1, {0, 0, 0, 0}
    };
    linears_matrix_t z = {&z_value, 1, 1};
    linears_status_t status = filters_ukf_q15_init(&filter, 1.0f, 2.0f, 0.0f,
        &filter.estimate, &filter.covariance, &filter.process_noise, &filter.measurement_noise);
    if (status == LINEARS_OK)
        status = filters_ukf_q15_predict(&filter, identity, NULL, NULL, NULL, workspace);
    if (status == LINEARS_OK)
        status = filters_ukf_q15_update(&filter, &z, identity, NULL, NULL, NULL, workspace);
    return status == LINEARS_OK ? 0 : 1;
}
```

For q31, use `int32_t` signal arrays, double callback/work arrays and
`filters_ukf_q31_workspace_t`; encode the numeric inputs as Q1.31 rather
than reusing the q15 integer literals. Capacity macros count elements, not
bytes, except the explicitly named `*_BASIC_BYTES` macros.

## Precision and limitations

`f32`/`f64` keep native precision and reject nonfinite values. Full positive
definiteness is established by the Cholesky factor at working precision, not
an eigendecomposition. Symmetrization is `(A + A^T)/2`, not projection onto
the PSD cone; finite-precision results are not guaranteed PSD. There is no
square-root/augmented UKF, no noise-free or additive-control variant, no
automatic regularization, and no iteration of the manifold mean beyond the
single center-relative pass described above.

## Validation

Host tests live in [`test/ukf.cc`](test/ukf.cc), with independent references
in `test/ukf_support.h` and dedicated Q regressions in
[`test/q_estimators.cc`](test/q_estimators.cc). From the configured SDK root:

```sh
xmake test -g xhive -v 'filters_test_ukf/ukf' 'filters_test_q_estimators/q_estimators'
```

The suite covers:

- Closed-form sigma weights and invalid hyperparameters.
- Linear-model agreement with the dense Kalman filter and nonlinear
  pendulum transactions against a long-double Eigen reference.
- Wrapped-angle residuals, manual floating-kernel composition, and
  agreement between working and signal-format transaction interfaces.
- Exact capacities, aliasing, misalignment, callback failures, nonfinite
  values, Cholesky failures and persistent-state preservation.
- Q sigma points and mixed-work priors outside the signal interval with
  valid committed results, signed sigma weights, tiny working covariances
  at zero and nonzero centers, endpoint conversion, rejected out-of-range
  results and size-macro overflow sentinels.

The f32 and q15 C examples above were independently compiled and linked
with GCC and Clang using `-std=c99 -O2 -pedantic-errors -Wall -Wextra
-Werror -Wvla`, then both returned zero. That checks examples, not the full
estimator suites.

Current integration results are recorded under [Host tests](README.md#host-tests).
The earlier floating-wrapper test totals and sanitizer results are not
carried forward as validation of these Q kernels. Match the source revision
when reproducing a result. See [measurement tools and scope](README.md#measurements)
for timing, allocation and ARM object-code comparisons. Host simulation is
not MCU, hardware or interrupt-timing validation.
