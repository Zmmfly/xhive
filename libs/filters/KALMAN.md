# Scalar and linear Kalman filters

## Model and algorithm

The scalar model is `x_next = a*x + w`, `z = h*x + v`, with independent,
zero-mean process/measurement noise, `var(w)=q` and `var(v)=r`. Prediction
computes `x-=a*x` and `p-=a*p*a+q`. Measurement update computes

```text
s = h*p*h + r
k = p*h / s
e = z - h*x
x+ = x + k*e
j = 1 - k*h
p+ = j*p*j + k*r*k
```

The dense model has `n>0` states, `m>0` measurements, `F:n*n`, `Q:n*n`,
`H:m*n`, `R:m*m`, `x:n*1`, `P:n*n`, and `z:m*1`. Prediction computes
`x-=F*x`, `P-=F*P*F^T+Q`. Update solves `S*K^T = H*P`, where
`S=H*P*H^T+R`, without explicitly forming an inverse. It then computes
`x+=x+K*(z-H*x)` and the Joseph covariance

```text
J  = I - K*H
P+ = J*P*J^T + K*R*K^T
```

The transpose solve is valid because `P` and `S` are symmetric. Both `R` and
`S` are checked by the shared Cholesky routine, then the existing partial-pivot
LU solve computes the gain; there is no separate SPD-solve implementation in
filters. `S` and newly computed state covariance are symmetrized as `(A+A^T)/2`
through the shared weighted-sum kernel. Symmetrization is not projection onto
the PSD cone.

### Source and reasoning

- Alex Becker, [Kalman Filter Equations — Summary](https://www.kalmanfilter.net/multiSummary.html).
  The retrieved table gives state/covariance prediction, measurement update,
  gain and Joseph covariance equations with dimensions. This implementation
  takes its zero-control-input model and replaces the written inverse in the
  gain formula by the algebraically equivalent transposed linear solve.
- Alex Becker, [Covariance Update Equation and derivation](https://www.kalmanfilter.net/covUpdate.html).
  The retrieved page explicitly states the Joseph equation above and derives
  it by expanding the posterior error and eliminating cross terms under the
  uncorrelated prior-error/measurement-noise assumption.
- For real arithmetic, if `P` is positive semidefinite (PSD) and `R` is
  positive definite (SPD), both Joseph terms are PSD: for any vector `u`,
  `u^T*(J*P*J^T)*u=(J^T*u)^T*P*(J^T*u)>=0`, and likewise for `K*R*K^T`.
  This is the reason for retaining the Joseph form rather than using the
  subtractive shortcut `(I-KH)*P`. It is not a claim of exact PSD preservation
  under finite-precision rounding.

## Ownership and contracts

All state, matrix data and scratch storage belong to the caller. Scalar
reset initializes both state fields; prediction and update need a prior
successful reset. Matrix state is a caller-initialized storage descriptor
with explicit estimate/covariance capacities measured in elements of the
selected numeric format. Reset fills those buffers; it does not allocate or
change their addresses or dimensions. The same context must not be used
concurrently, including interrupt/task concurrency; independent contexts and
disjoint workspaces are reentrant. There is no allocation, locking or I/O.

Scalar covariance and process variance must be nonnegative. Measurement
variance must be strictly positive. A zero innovation residual is valid and
still updates covariance; a nonpositive or unrepresentable innovation
variance is an error. Matrix state/process covariances must be symmetric
PSD and measurement covariance symmetric SPD. Physical units must be
consistent: covariance entries have products of state units, `Q` has those
same units, `R` has products of measurement units, and `H` maps state units
to measurement units. `F` is the caller's transition over one prediction
interval; there is no implicit sample time or control-input term.

Every failed reset/predict/update preserves estimate data, covariance data,
and state descriptors. Scratch may change. Callers can retry after adjusting
inputs or replacing the workspace. A separate successful prediction is not
rolled back if a later, separate update fails.

For predict/update, all input matrices are read-only and may alias each
other, but must not overlap active state data or the state object. Workspace
must not overlap any matrix data, descriptors, or state. Reset permits exact
alias of an initial matrix with its corresponding destination; partial or
cross-state alias is prohibited. Actual source allocation sizes remain the
caller's responsibility because `linears_matrix_t` has no capacity field.

## Precision and limits

`f32`/`f64` use their respective native formats. All external floating values
must be finite; arithmetic overflow/nonfinite intermediates fail. Fixed-point
scalar values use separate exponents, while each fixed-point matrix has one
block exponent. `q15` represents `mantissa * 2^(exponent-15)` and `q31`
represents `mantissa * 2^(exponent-31)`. Valid exponents and rounding follow
`linears.h` and `linears_numeric.h`; successful operations may change output
exponents. Fixed-point arithmetic uses the shared integer-only normalized
working representation and never converts through floating point.

Matrix covariance symmetry and diagonal signs are checked. Full PSD of
state covariance and process covariance is a caller precondition, not a
runtime eigendecomposition. Measurement and innovation SPD are checked with
the shared Cholesky kernel before the LU solve. Roundoff, block quantization,
ill-conditioning, exponent overflow or a rounded-away positive pivot can
cause a failure even when the real-valued model is valid. There is no jitter,
noise-floor injection, automatic regularization, eigenvalue clipping,
nonlinear/extended Kalman logic or square-root filtering.

## Workspace

Each format has a borrowed workspace view containing **three separate real
arrays of the declared types**. Do not cast a workspace-unit array to a raw
float/double/int16/int32 array: that violates the effective-type contract and
can also introduce the wrong element stride. Prediction ignores decomposition
storage; update uses all three arrays. Only the required prefixes are touched.

| Field | Array type | Prediction requirement | Update requirement |
| --- | --- | --- | --- |
| `data`, `capacity` | float/double/int16_t/int32_t for the selected API | `FILTERS_KALMAN_PREDICT_STORAGE(n)` elements | `FILTERS_KALMAN_UPDATE_STORAGE(n,m)` elements |
| `basic`, `basic_bytes` | `linears_workspace_unit_t` | `FILTERS_KALMAN_PREDICT_BASIC_BYTES(n)` bytes | `FILTERS_KALMAN_UPDATE_BASIC_BYTES(n,m)` bytes |
| `decomposition` | float/double for floating APIs, `linears_fixed_t` for q APIs | Ignored; `{NULL,0}` is valid | `FILTERS_KALMAN_UPDATE_DECOMP_STORAGE(n,m)` elements |

Use `LINEARS_WORKSPACE_UNITS(bytes)` to declare the `basic` array. Raw storage
counts are `n+4*n*n` for prediction and
`4*n*n+4*n*m+2*m*m+2*n+2*m` for update. Decomposition storage is the shared
`LINEARS_SOLVE_WORKSPACE(m,n)` bound, which also covers Cholesky. Size macros
are constant expressions, propagate arithmetic overflow as `SIZE_MAX`, and
require side-effect-free nonnegative arguments. `SIZE_MAX` is invalid, not an
allocation request. Operations also check count-to-byte/address overflow.
There is no fixed dimension cap; available caller storage and checked sizes
limit dimensions. Loops are dimension-bounded; no recursive calls or variable
stack allocations occur.

A C99 example for a two-state, one-measurement f32 update workspace is:

```c
float intermediate[FILTERS_KALMAN_UPDATE_STORAGE(2, 1)];
linears_workspace_unit_t basic[LINEARS_WORKSPACE_UNITS(
    FILTERS_KALMAN_UPDATE_BASIC_BYTES(2, 1))];
float decomposition[FILTERS_KALMAN_UPDATE_DECOMP_STORAGE(2, 1)];
filters_kalman_matrix_f32_workspace_t workspace = {
    intermediate, FILTERS_KALMAN_UPDATE_STORAGE(2, 1),
    basic, sizeof basic,
    {decomposition, FILTERS_KALMAN_UPDATE_DECOMP_STORAGE(2, 1)}
};
```

## Validation

Both test files instantiate all four formats. Scalar tests use an independent
long-double analytic reference; matrix tests use Eigen 3.4.0 long-double matrix
products and LDLT solves, with the independent exact-arithmetic covariance
formula `P-KHP` rather than copying the production Joseph implementation.
Random seeds are fixed. Tests cover 1,000 random scalar models and 6,000-step
scalar sequences per format; 96 random rectangular matrix models (`n,m=1..4`)
and 1,200-step tracking sequences per format; zero residual, zero/rank-deficient
PSD state covariance, very small measurement noise, explicit q block rounding,
reset/reentrancy, invalid noise, nonfinite/exponent limits, alias rejection,
capacity boundaries, sentinels, and state preservation after late failure.

The base absolute/relative test envelope is `tolerance * (1 + abs(reference))`:
f32 `2e-5`, f64 `2e-12`, q15 `4e-3`, q31 `2e-7`. Long scalar/matrix sequences
use factors two/three respectively. These are test acceptance bounds for the
specified models, not universal error guarantees. The tiny-noise regression
also requires a positive posterior variance and relative error below `1e-5`;
it catches loss of the Joseph noise term even when ordinary absolute bounds
would hide that loss.

Real commands, host results and remaining unverified boundaries are recorded
in `WORKLOG-kalman.md`. Host simulation is not target or hardware validation.
