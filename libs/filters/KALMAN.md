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

## Fixed-fraction Q contract

- `q15` signals and state are `int16_t`, with value `raw/2^15` and range
  `[-1, 1-2^-15]`.
- `q31` signals and state are `int32_t`, with value `raw/2^31` and range
  `[-1, 1-2^-31]`.
- `linears_matrix_t` contains only `data`, `rows`, and `cols`. The binary point
  never changes in a descriptor, state, workspace, coefficient or result.
- Scalar q15/q31 **transition and observation coefficients** are
  `linears_fixed_t`: int64_t raw/2^31. This allows exact unity
  (`LINEARS_FIXED_ONE`) and gains such as two. Other scalar arguments remain
  raw Q signals/variances. Matrix F/H entries remain raw Q, like other matrix
  inputs; scalar/matrix comparisons must convert those actual raw F/H entries
  to wide coefficients, not replace MAX by exact unity.
- All Q production arithmetic is integer-only. Scalar computations use shared
  F31 operations and signed 128-bit F62 MAC/MSU helpers; final scalar output
  expressions are checked and rounded directly from F62. Matrix operations
  use the selected shared fixed-F31 working helpers in `linears_work.h`, not
  an independent filters matrix kernel. Products accumulate in F62 and round
  once per F31 output element; the working unit is always `2^-31`.

Narrow Q export checks its **supplied conversion input** against `[-1,+1]`
before signal rounding. A conversion input above +1 or below -1 fails with
RANGE. Scalar final expressions supply an F62 accumulator; matrix filters
supply their **already computed/rounded F31 estimate and covariance values**,
not the ideal unrounded full expression. Exact +1 at that conversion point,
and in-range values whose nearest rounding reaches the unavailable positive
endpoint, map to MAX; **MAX remains `1-LSB`, never exact one**.

Round-to-nearest ties go away from zero. Signal narrowing has at most half a
signal LSB error except in the positive endpoint cell, where the error can be
one LSB. Matrix F31 working-rounding error is **additional and separate** from
that endpoint mapping error. Neither bound is a universal error bound for the
full composite algorithm; this is not exact rational arithmetic.

### Matrix range-check boundary

The distinction is observable even in prediction. For q31 with five states,
let x contain five copies of 1/4; let the first F row be
`[MAX, MAX, MAX, MAX, raw 5]`, all remaining F rows be zero, and P=Q=0.
The independently calculated exact F62 dot has raw integer
`2^62 + 2^29`, meaning **`1 + 2^-33`**. Nearest rounding to the working F31
lattice yields exact working ONE. Kalman therefore succeeds and narrows that
working value to MAX. Relative to the ideal dot, the difference is the sum of
`2^-33` working-rounding loss and one q31 signal LSB (`2^-31`) at the endpoint.

In contrast, direct `linears_mul_q31` and direct F62-to-q31 export inspect the
unrounded F62 dot before narrowing and return RANGE for the same exact dot.
Those direct operations have a stronger terminal-domain contract than the
composite F31-working Kalman path. Scalar Kalman retains its direct final-F62
checks; in particular, its exact final products `+/- (1+2^-33)` are rejected.
The n=5 matrix regression verifies the integer dot, working ONE, Kalman MAX
success, and both direct RANGE results exactly, without tolerance. Clearly
out-of-range computed F31 states, such as +1.5 or -1.1, still fail atomically.

Matrix working values may exceed one. For example, `P=1/2, H=1/4, R=1/64`
gives `K=8/3`, but with `x=0,z=1/8`, the outputs are `x+=1/3,P+=1/6`.
Similarly, `P=H=R=3/4` gives `S=75/64` while the posterior remains in range.
These valid updates must not narrow S or K into a Q signal. The Joseph identity
uses exact wide unity; replacing it by raw Q MAX would incorrectly decay P
when H is zero. Only final state is narrowed, after every output has passed
its range check and been staged in scratch.

Choose physical units and input models so raw signals, variances and matrix
coefficients are representable. The API never automatically rescales a model.
A matrix requiring exact positive-unity coefficients cannot express those
entries exactly in raw Q; use the floating interface when that exact model is
required. Q measurement noise must remain positive **after quantization**.
A physical noise value of `2^-80` rounds to zero in both Q formats and is
rejected; it is not a positive-noise input. Minimum positive scalar noise is
one raw LSB. Covariance entries and process noise can round to zero.

A C99 scalar q15 example uses explicit wide coefficients:

```c
filters_kalman_scalar_q15_t state;
linears_status_t status = filters_kalman_scalar_q15_reset(
    &state, INT16_C(8192), INT16_C(8192)); /* x=1/4, p=1/4. */
if (status == LINEARS_OK)
    status = filters_kalman_scalar_q15_predict(
        &state, LINEARS_FIXED_ONE, INT16_C(256));
if (status == LINEARS_OK)
    status = filters_kalman_scalar_q15_update(
        &state, INT16_C(4096), 2 * LINEARS_FIXED_ONE, INT16_C(4096));
```

## Precision and limitations

`f32`/`f64` retain their native precision and range, including finite values
above one. External floating values must be finite; nonfinite results fail.
Matrix covariance symmetry and diagonal signs are checked. Full PSD of state
covariance and process covariance is a caller precondition, not a runtime
eigendecomposition. Measurement and innovation SPD are checked with shared
Cholesky before LU solve. Quantization, ill-conditioning, working integer
range overflow or a rounded-away positive pivot can fail even for a valid
real-valued model. There is no jitter, noise-floor injection, eigenvalue
clipping, automatic regularization, nonlinear/extended Kalman logic or
square-root filtering. A successfully stored covariance is symmetric, but
finite-precision Joseph plus symmetrization is not a universal PSD guarantee.

## Workspace and RAM cost

Each format borrows **three separate real arrays of their declared types**.
Do not cast an entire workspace-unit array to another numeric array: that
violates the effective-type contract and may give the wrong stride. Prediction
ignores decomposition storage; update uses all three arrays. Only required
prefixes are accessed, and no hidden allocation occurs.

| Format/field | Actual array element type | Prediction requirement | Update requirement |
| --- | --- | --- | --- |
| float `data`, `capacity` | float or double | `FILTERS_KALMAN_PREDICT_STORAGE(n)` elements | `FILTERS_KALMAN_UPDATE_STORAGE(n,m)` elements |
| Q `data`, `capacity` | **linears_fixed_t (int64_t), not raw Q** | `FILTERS_KALMAN_Q_PREDICT_STORAGE(n)` elements | `FILTERS_KALMAN_Q_UPDATE_STORAGE(n,m)` elements |
| float `basic`, `basic_bytes` | linears_workspace_unit_t | `FILTERS_KALMAN_PREDICT_BASIC_BYTES(n)` bytes | `FILTERS_KALMAN_UPDATE_BASIC_BYTES(n,m)` bytes |
| Q `basic`, `basic_bytes` | linears_workspace_unit_t | `FILTERS_KALMAN_Q_PREDICT_BASIC_BYTES(n)` bytes | `FILTERS_KALMAN_Q_UPDATE_BASIC_BYTES(n,m)` bytes |
| `decomposition` | float/double for float APIs, linears_fixed_t for Q | Ignored; `{NULL,0}` is valid | `FILTERS_KALMAN_UPDATE_DECOMP_STORAGE(n,m)` elements |

Floating intermediate counts remain `n+4*n*n` for prediction and
`4*n*n+4*n*m+2*m*m+2*n+2*m` for update. Q working storage additionally imports
raw inputs without changing their values: **`2*n+7*n*n`** F31 elements for
prediction and **`5*n*n+5*n*m+3*m*m+3*n+3*m`** for update. Decomposition uses
`m*m+m*n` working elements, covering both LU solve and Cholesky.

Q basic scratch must also cover `n+n*n` units to stage the final raw estimate
and covariance through each unit's declared `q15`/`q31` member. This storage is
reused after numerical operations; it is not a fourth array or a raw-pointer
cast. The Q basic count is `n+n*n` for prediction and
`max(n*n,n*m,m*m,n+n*n)` for update. For `n=2,m=1`, Q prediction therefore
needs 32 F31 elements plus six basic units; update needs 42 F31 elements,
six basic units and three decomposition F31 elements. State storage is
additional: two raw-Q estimate elements and four raw-Q covariance elements.

Size macros are constant expressions, return `SIZE_MAX` for arithmetic
overflow, and require side-effect-free positive dimensions. `SIZE_MAX` is
invalid, not an allocation request. Calls also check byte/address arithmetic.
There is no arbitrary dimension cap; available storage and checked integer
sizes bound dimensions. Loops are dimension-bounded, with no recursion/VLA.

A C99 two-state, one-measurement q15 update workspace is:

```c
linears_fixed_t intermediate[FILTERS_KALMAN_Q_UPDATE_STORAGE(2, 1)];
linears_workspace_unit_t basic[LINEARS_WORKSPACE_UNITS(
    FILTERS_KALMAN_Q_UPDATE_BASIC_BYTES(2, 1))];
linears_fixed_t decomposition[FILTERS_KALMAN_UPDATE_DECOMP_STORAGE(2, 1)];
filters_kalman_matrix_q15_workspace_t workspace = {
    intermediate, FILTERS_KALMAN_Q_UPDATE_STORAGE(2, 1),
    basic, sizeof basic,
    {decomposition, FILTERS_KALMAN_UPDATE_DECOMP_STORAGE(2, 1)}
};
```

For f32/f64, retain the original macros without `_Q_` and use float/double
intermediate/decomposition arrays. `LINEARS_WORKSPACE_UNITS(bytes)` sizes the
basic array in every format.

## Validation and represented models

Both test files instantiate all four formats. Scalar tests use an independent
long-double analytic reference; matrix tests use Eigen 3.4.0 long-double
products and LDLT solve with independent covariance `P-KHP`, not copied Joseph
code. References use the **actual decoded raw model**, including Q MAX as
`1-LSB`; they are neither clipped nor adjusted to hide range failures.

Fixed seeds cover 1,000 randomized scalar trials and 6,000-step scalar
sequences per format; 96 rectangular matrix models (`n,m=1..4`) and 1,200-step
matrix sequences per format. The Q random models deliberately fit the fixed
signal interval: scalar initial magnitudes at most 0.4, P at most 0.5, Q at
most 0.01, measurement magnitude at most 0.3, and R in [0.1,0.3]; matrix random
models use state/measurement magnitudes at most 0.1, F around 0.7*I, H entries
at most 0.2, and well-conditioned P/R around 0.125*I. Q tracking uses stable F,
nonzero representable Q/R, and bounded measurements. Floating tests retain
the wider original models and `2^-80` noise regression.

Other regressions cover wide gain/innovation, exact working unity and scalar
observation gain two, raw-Q endpoints/MAX semantics, minimum positive and
rounded-to-zero noise, zero/rank-deficient PSD state, reset/reentrancy, invalid
noise, NaN/Inf, alias, all workspace capacities/sentinels, and atomic failures
for computed F31 matrix states outside the signal interval. The n=5 boundary
and direct F62 scalar regressions separately enforce their different terminal
range contracts without a tolerance comparison.

The unchanged envelope is `tolerance * (1 + abs(reference))`: f32 `2e-5`,
f64 `2e-12`, q15 `4e-3`, q31 `2e-7`; long scalar/matrix sequences use factors
two/three respectively. These are measured-test acceptance bounds for these
models, not universal error guarantees. The noise-term regressions additionally
require positive posterior variance and relative error below `1e-5` against
the supplied tiny floating or minimum-positive Q variance.

Final counts and real commands are in `WORKLOG-kalman.md`. Host simulation is
not MCU, hardware or interrupt-timing validation.
