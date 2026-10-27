# Particle and unscented particle filters

## Model and algorithm

The model is the one of [UKF.md](UKF.md): `x_next = f(x) + w`, `z = h(x) + v`
with `cov(w) = Q` (n-by-n), `cov(v) = R` (m-by-m), caller callbacks `f`, `h`
and optional residual callbacks for angle-like components. Instead of one
Gaussian, a filter keeps N particles `x_i` (one n-vector per row) with
normalized weights `w_i`; the posterior is the weighted cloud. The equations
below use normalized masses; raw Q weights represent the same distribution
up to a common scale. The library has no random generator: callers own one
and hand over standard-normal draws through a sampler callback and a uniform
draw to `resample`.

### Bootstrap particle filter (`filters_pf.h`)

```text
init      x_i = mean + L0*xi_i        L0*L0^T = P0, xi_i ~ N(0,I)      w_i = 1/N
predict   x_i = f(x_i) + LQ*xi_i      LQ*LQ^T = Q
update    e_i = (1/2)*d_i^T R^-1 d_i  d_i = h(x_i) (-) z
          w_i = w_i*exp(-(e_i - min e)) / sum                          (shift-normalized)
ESS       (sum w)^2 / sum w^2
resample  systematic: targets (u+i)/N*sum w, u ~ U[0,1); x_i = x_{idx[i]}; w_i = 1/N
estimate  xhat = mean(X, w, residual)  P = sum_i w_i*(x_i (-) xhat)*(x_i (-) xhat)^T
```

`(-)` is the residual: plain subtraction or the caller's callback. Floating
`estimate` reuses the UKF kernels `mean`, `deviations` and
`weighted_covariance`; Q uses dedicated integer moment accumulation. Both
use the same center-relative residual-aware mean as the UKF
(`xhat = x_0 + sum_i w_i*(x_i (-) x_0)`). Resampling is left to the caller so
the threshold is explicit: call `effective_sample_size` after `update` and
`resample` when it falls below the chosen fraction of N. `resample` also
returns the source index of every new particle so caller-owned per-particle
data can be resampled alongside.

### Unscented particle filter (`filters_upf.h`)

Every particle carries its own covariance `P_i`. One `step` with measurement
`z` runs, per particle, the UKF kernels `predict_work` and `update_work`
(`filters_ukf.h`) to obtain the Gaussian proposal `N(M_i, C_i)`, draws the new
particle from it and corrects the weight by likelihood times transition prior
over proposal:

```text
(xbar_i, Pbar_i) = ukf_predict(x_i, P_i, Q, f)
(M_i, C_i)       = ukf_update(xbar_i, Pbar_i, R, z, h)
L_i*L_i^T = C_i      xhat_i = M_i + L_i*xi_i         xi_i ~ N(0,I)
a_i = (1/2)*(h(xhat_i) (-) z)^T R^-1 (h(xhat_i) (-) z)        likelihood
b_i = (1/2)*(xhat_i (-) f(x_i))^T Q^-1 (xhat_i (-) f(x_i))    transition prior
w_i = w_i * prod_j L_i[j][j] * exp(-(a_i + b_i - (1/2)*xi_i^T xi_i))   then normalized
x_i = xhat_i      P_i = C_i
```

Derivation: `q(xhat) = N(xhat; M, C) = exp(-xi^T xi / 2) / ((2 pi)^(n/2) prod_j L[j][j])`
because `xhat - M = L*xi` and `(L*xi)^T C^-1 (L*xi) = xi^T xi`. The Gaussian
normalizers of `R` and `Q` are constant across particles and drop out of the
normalized weights; `sqrt(det C_i) = prod_j L_i[j][j]` does not and is kept.
The floating implementation applies the determinant factor as the ratio
`prod_j (L_i[j][j] / L_0[j][j])` to particle 0, avoiding tiny absolute
products before normalization. The Q implementation instead subtracts
`sum_j log(L_i[j][j])` from each particle's combined exponent and passes it
to the Q reweighting path. Both retain the proposal-determinant correction;
it must not be dropped as though every proposal had the same covariance.
`resample` gathers particles and covariances together; `estimate` and
`effective_sample_size` are the PF functions applied to the embedded `base`.

### Source and reasoning

- N. J. Gordon, D. J. Salmond and A. F. M. Smith, *Novel approach to
  nonlinear/non-Gaussian Bayesian state estimation*, IEE Proceedings F 140(2),
  1993: the bootstrap filter (prior proposal, likelihood weights, resampling).
- J. Carpenter, P. Clifford and P. Fearnhead, *Improved particle filter for
  nonlinear problems*, IEE Proceedings RSN 146(1), 1999, and G. Kitagawa,
  *Monte Carlo filter and smoother*, J. Comput. Graph. Stat. 5(1), 1996:
  systematic (stratified, single-draw) resampling and the effective sample
  size criterion.
- R. van der Merwe, A. Doucet, N. de Freitas and E. Wan, *The Unscented
  Particle Filter*, NIPS 13, 2000: a UKF per particle as the proposal, the
  weight `likelihood * prior / proposal`, and per-particle covariances that
  are resampled with their particles.
- The shift by `min e` before `exp` stabilizes relative likelihoods without
  changing the normalized distribution. The Q path selects that minimum
  among particles with nonzero incoming weight, so a zero-mass particle
  cannot determine the shift or cause an unnecessary exponential overflow.

## Ownership and contracts

All storage is caller-owned: the state object, its arrays (particles N-by-n,
weights N-by-1, Q, R, and for the UPF the N-by-(n*n) covariances) with
explicit element capacities, and typed scratch arrays. Q workspaces have
raw `signal` staging in addition to `data`, `basic` and `decomposition`.
`init` fills the arrays but never changes shapes, addresses or capacities;
Q and R sources may exactly alias their destination or be the state's own
descriptor. There is no
allocation, I/O, recursion, variable-length array, mutable global state or
random generator. Serialize calls sharing a state or scratch; independent
instances are reentrant.

Every persistent array is stored in the signal type: `float`/`double` for
f32/f64, raw `int16_t/2^15` or `int32_t/2^31` for q15/q31. Particles,
weights, (UPF) covariances, Q, R, the init mean/covariance, measurements and
the estimate outputs all follow that rule, so a Q particle must stay inside
`[-1,+1]`; see the Q contract below. `weights` and `particles` are public
fields so the kernels can be applied to them directly; `covariances` of the
UPF likewise.

Every function validates its arguments, registers all descriptor, data and
scratch ranges and rejects any overlap except read-only inputs with each
other and identical descriptors. Q is checked symmetric with nonnegative
diagonal (`DOMAIN`), R symmetric, and both must have a Cholesky factor
(`NOT_POSITIVE_DEFINITE`): `predict` factors Q, `update` factors R, and a UPF
`step` factors both before the first callback. A singular PSD Q or R is
therefore rejected, unlike in the UKF where only `R` is factored. New
particles, weights and covariances are staged in scratch and committed by a
copy after the last fallible operation, so a failing callback, sampler,
residual or Cholesky leaves the state byte-identical; a nonfinite floating
callback or sampler output fails with `RANGE`. The public floating
`reweight` kernel may leave its weight vector partially modified on failure;
filter transactions keep weight work in private scratch until commit.

`resample` writes `indices[0..N)` and the state only on success; the kernel
`resample_indices` likewise writes nothing on failure. Indices are
nondecreasing and every particle is copied `floor(N*w_i)` or `ceil(N*w_i)`
times. Because the indices are nondecreasing, the rows are gathered in
place (a forward pass for sources above the destination, a backward pass for
sources below), so `resample` needs no staging scratch in any format. A zero
weight sum, a negative weight, or a uniform draw outside `[0,1)` fails with
`DOMAIN`.

## Kernels

The public floating working-format kernels (tags `f32` and `f64`) operate
on one point per row, so other particle schemes can be composed from them
together with the floating UKF kernels:

| Kernel | Computes |
| --- | --- |
| `perturb(points, L, sample, ctx, out)` | `out_i = points_i + L*xi_i`; `out` may be exactly `points` |
| `gaussian_exponent(D, L, out)` | `out_i = (1/2)*d_i^T (L L^T)^-1 d_i` through one pivoted solve per row |
| `reweight(w, e)` | `w_i = w_i*exp(-(e_i - min e))`, normalized to sum one |
| `effective_sample_size(w, out)` | `(sum w)^2 / sum w^2` |
| `resample_indices(w, u, idx, cap)` | systematic resampling indices from one uniform draw |
| `gather(points, idx, count, out)` | `out_i = points[idx[i]]` |

The public `gaussian_exponent` continues to accept general nonsingular dense
factors. Floating PF/UPF state calls use a private shortcut only when the
validated Cholesky factor is exactly diagonal: divide each residual by its
diagonal, then retain the original squared-norm reduction order. Other
factors keep the pivoted solve. An unrestricted triangular replacement was
rejected because broad-exponent cancellation cases lost accepted f32
accuracy even when they returned `OK`; a `RANGE`-only retry did not catch
those cases. The diagonal test is performed once per shared noise factor,
not once per particle.

`filters_ukf_f32/f64_*` propagation and statistics supply the remaining
floating steps. Q PF/UPF use dedicated integer weight, resampling, ESS and
moment paths rather than widening the complete cloud into those kernels.
The Q UPF uses the private cores of `filters_ukf_q15/q31_predict_work` and
`update_work`: their small float/double matrices preserve a temporary prior
that may exceed `[-1,+1]`, while the statistics inside are Q integer
operations. A prior outside the persistent signal range is not itself a
commit failure; the new particle and posterior covariance must fit when the
step commits.

## Fixed-fraction Q contract

- q15/q31 states hold raw `int16_t/2^15` or `int32_t/2^31` particles,
  weights, (UPF) covariances, Q and R; the init mean/covariance,
  measurements and estimate outputs are raw too. No persistent array is
  widened to hold intermediates.
- Dedicated Q paths perform integer weight processing, systematic
  resampling, ESS and weighted moments. Necessary local range expansion
  uses float for q15 and double for q31 (24/53 significand bits cover the
  15/31-bit signals); this is not a call to the complete floating filter.
  Sampler, model and residual callbacks still see float/double vectors;
  `alpha`, `beta`, `kappa` and uniform draws retain float/double types.
  A standard-normal draw, residual or temporary UKF prior may exceed the
  signal interval without being clipped into raw Q.
- A typed Q workspace separates local floating `data` from raw `signal`
  staging. Persistent particle clouds and per-particle covariances are not
  duplicated as whole floating working arrays just to reuse an f32/f64
  filter. See the per-call capacity macros below.
- Committed outputs pass a `[-1,+1]` range check on the computed candidate
  values, with the positive-endpoint policy of
  `linears_numeric.h` (exact +1 and the rounding cell reaching it map to
  MAX, ties away from zero). `init` and `predict` check every new particle,
  the UPF `step` every new particle and covariance, `estimate` both outputs,
  all before anything is written: a particle that leaves the interval fails
  its transaction with `RANGE` and the state stays byte-identical. Scale the
  problem so particles keep a margin inside the interval.
- Raw Q weights are scaled so that the largest equals `INT16_MAX` or
  `INT32_MAX` instead of summing to one: a sum-normalized vector would leave
  most of the fixed range unused once `1/N` is a few LSBs. The weights define
  the distribution up to that common factor, which `update`, `resample`,
  `estimate` and `filters_pf_q15/q31_effective_sample_size` account for
  directly; no sum-normalized floating weight array is required. A zero
  weight sum fails with `DOMAIN`. `init` and `resample` reset every weight
  to the raw maximum. Final integer rounding discards sufficiently small
  relative mass: approximately below half a raw LSB, or `2^-16` (q15) and
  `2^-32` (q31) relative to the largest weight. Likelihood quantization also
  contributes error; Q weights do not have f32/f64 exponential dynamic range.

### Integer and floating boundaries

The dedicated implementation is in
[`pf_q_filter_impl.h`](src/pf_q_filter_impl.h), with integer helpers in
[`pf_q_internal.h`](src/pf_q_internal.h). It is not an instantiation of the
complete floating PF kernel:

- Weight products, maximum selection and normalized raw output use integer
  arithmetic. `expf`/`exp` and one shared normalization reciprocal remain
  floating; transient unsigned likelihoods use 31 fractional bits for q15
  and 63 for q31. Those scalars are not persistent particle/weight storage.
- ESS accumulates the raw weight sum and sum of squares in integers, then
  forms the float/double result. Systematic resampling uses integer
  cumulative masses and an exact representation of the supplied binary
  uniform draw; it does not allocate a floating weight vector.
- Mean/covariance loops accumulate raw weighted moments. Without a residual
  callback, each two-word sum stays local across its particle loop and is
  written to `basic` once, not loaded/stored for every contribution. The
  accumulator width and particle order are unchanged; no widened particle
  cloud is retained. A caller residual outside the signal interval contributes
  through a local float/double term rather than being clipped.
- Gaussian operations keep Cholesky and scalar divisions in float/double.
  Their mixed dot products use Q integer MACs for bounded operands and
  local floating products for operands needing more range, including
  standard-normal draws. Direct triangular solves reuse the factor instead
  of running a pivoted LU for each particle.

[`upf_q_filter_impl.h`](src/upf_q_filter_impl.h) combines those helpers with
the dedicated Q UKF cores. It retains only one particle's extended
prior/posterior in floating scratch and stages the complete new particle
and covariance sets raw.

Both floating and Q UPF steps validate their shared noise matrices, sigma
weights and workspace at the step boundary, then use private checked UKF
cores for each proposal. Each persistent particle covariance is still
validated before its callbacks; callback outputs and intermediate arithmetic
retain their range checks. `R` is factored once per step, not again inside
every proposal update. Each distinct particle, innovation and posterior
covariance still requires its own positive-definiteness check.

### Computation changes

The floating [`gaussian_exponent`](src/pf_work_impl.h) used by the previous
Q wrappers calls the general pivoted-LU `solve` on the same Cholesky factor
for every particle: its repeated factorization costs `O(N·m³)` for
m-dimensional likelihood residuals. The dedicated Q update factors `R`
once, then evaluates each residual by direct triangular forward
substitution: `O(m³ + N·m²)` for that density calculation. This removes
repeated decomposition work, not merely floating arithmetic. The UPF also
uses direct triangular density solves and the
[dedicated UKF reductions](UKF.md#computation-changes); each distinct proposal
covariance still needs its own factor.

Streaming one particle at a time removes N-by-n/N-by-m floating staging;
integer weight, ESS and moment accumulation removes corresponding floating
array passes. These source-backed changes explain reduced work and
workspace, but do not imply that integer instructions always beat an FPU.
Actual host timing and counted RAM must come from the same-revision
[benchmark](README.md#measurements).

## Migrating Q callers

Use `filters_ukf_q15_workspace_t` for q15 PF/UPF calls and
`filters_ukf_q31_workspace_t` for q31, not the old floating workspace type.
`data`/`capacity` describe float/double local work; `signal`/`signal_capacity`
describe separate raw `int16_t`/`int32_t` staging. `basic`/`basic_bytes` and
the floating `decomposition` workspace keep their explicit units. Reallocate
from the current per-call macros: `*_Q_*_STORAGE` counts floating elements,
and `*_Q_*_SIGNAL_STORAGE` counts raw signal elements. Do not cast a floating
workspace or reinterpret one array as both types. The callback/sampler types,
model records and persistent-array types remain unchanged; f32/f64 callers
keep their existing workspace type. The Q allocation pattern is the same as
the [typed UKF example](UKF.md#q15-typed-workspace-example), using the PF/UPF
per-call macros instead.

### Q callback order changes

The Q PF processes one particle completely before moving to the next,
rather than retaining an N-by-n or N-by-m floating cloud:

- `predict`: for particle `i`, call `transition`, then `sample`, form the
  candidate, check its final signal range and stage it raw; only then start
  particle `i+1`.
- `update`: for particle `i`, call `observation`, then the residual callback
  when supplied, compute its Gaussian exponent and store that one exponent;
  only then start particle `i+1`.

An error stops immediately. For example, if sampling particle `i` fails,
transitions for particles after `i` have not run. A successful predict still
calls transition and sampler N times each; a successful update calls
observation N times and a supplied residual N times. Callback input/output
views are disjoint, and intermediate callback results or draws are not
restricted to `[-1,+1]` merely to fit staging. Persistent filter arrays are
committed only after the entire transaction succeeds. Caller-owned callback
or RNG state is **not** rolled back after a failure.

This is an intentional Q API behavior change as well as a workspace-type
migration. Code that consumes a shared RNG or records callback order must
account for the interleaving and early exit. f32/f64 callback order is
unchanged. No compatibility switch retains the old whole-cloud floating
staging.

## Workspace and RAM cost

The macros in [`filters_pf.h`](include/filters_pf.h) and
[`filters_upf.h`](include/filters_upf.h) are constant expressions;
`SIZE_MAX` signals overflow and is not an allocation request. Only required
prefixes are accessed. Let `s = 2n+1`, `k = max(n,m)`,
`W = FILTERS_UKF_UPDATE_WORK_STORAGE(n,m)` and
`Wq = max(FILTERS_UKF_Q_PREDICT_WORK_STORAGE(n),
FILTERS_UKF_Q_UPDATE_WORK_STORAGE(n,m))`.

For f32/f64, `data` and `decomposition` contain float/double:

| Call | `data` elements | `basic` bytes | `decomposition` elements |
| --- | --- | --- | --- |
| PF `init` / `predict` | `FILTERS_PF_PREDICT_STORAGE(N,n)` = `N·n+n²+2n` | `FILTERS_PF_PREDICT_BASIC_BYTES(n)` (`n` units) | `FILTERS_PF_PREDICT_DECOMP_STORAGE(n)` = `n²` |
| PF `update` | `FILTERS_PF_UPDATE_STORAGE(N,m)` = `2N·m+2N+m²+m` | `FILTERS_PF_UPDATE_BASIC_BYTES(m)` (`m` units) | `FILTERS_PF_UPDATE_DECOMP_STORAGE(m)` = `m²+m` |
| PF `resample` | `0` | `0` | `0` |
| PF `estimate` | `FILTERS_PF_ESTIMATE_STORAGE(N,n)` = `3N·n+2n²+n` | `FILTERS_PF_ESTIMATE_BASIC_BYTES(N,n)` (`n·max(N,n)` units) | `0` |
| UPF `init` | as PF `init` | as PF `init` | as PF `init` |
| UPF `step` | `FILTERS_UPF_STEP_STORAGE(N,n,m)` = `N·(n²+n+2)+3n²+6n+m²+2m+1+W` | `FILTERS_UPF_STEP_BASIC_BYTES(n,m)` (`k·s` units) | `FILTERS_UPF_STEP_DECOMP_STORAGE(n,m)` = `max(n²+n,m²+mn)` |
| UPF `resample` | `0` | `0` | `0` |

For q15/q31, `data` and `decomposition` contain float/double and `signal`
contains raw `int16_t`/`int32_t`. Floating requirements are **not** the Q
requirements plus a widening buffer:

| Call | `data` elements | `signal` elements | `basic` bytes | `decomposition` elements |
| --- | --- | --- | --- | --- |
| PF `init` | `FILTERS_PF_Q_INIT_STORAGE(N,n)` = `2n²+3n` | `FILTERS_PF_Q_INIT_SIGNAL_STORAGE(N,n)` = `N·n` | `FILTERS_PF_Q_PREDICT_BASIC_BYTES(n)` = `0` | `FILTERS_PF_PREDICT_DECOMP_STORAGE(n)` = `n²` |
| PF `predict` | `FILTERS_PF_Q_PREDICT_STORAGE(N,n)` = `2n²+3n` | `FILTERS_PF_Q_PREDICT_SIGNAL_STORAGE(N,n)` = `N·n` | `FILTERS_PF_Q_PREDICT_BASIC_BYTES(n)` = `0` | `FILTERS_PF_PREDICT_DECOMP_STORAGE(n)` = `n²` |
| PF `update` | `FILTERS_PF_Q_UPDATE_STORAGE(N,n,m)` = `N+2m²+n+3m` | `FILTERS_PF_Q_UPDATE_SIGNAL_STORAGE(N,n,m)` = `N` | `FILTERS_PF_Q_UPDATE_BASIC_BYTES(m)` = `0` | `FILTERS_PF_Q_UPDATE_DECOMP_STORAGE(m)` = `m²` |
| PF `resample` | `FILTERS_PF_Q_RESAMPLE_STORAGE(N)` = `0` | `FILTERS_PF_Q_RESAMPLE_SIGNAL_STORAGE(N)` = `0` | `0` | `0` |
| PF `estimate` | `FILTERS_PF_Q_ESTIMATE_STORAGE(N,n)` = `n²+3n` | `FILTERS_PF_Q_ESTIMATE_SIGNAL_STORAGE(N,n)` = `n²+2n` | `FILTERS_PF_Q_ESTIMATE_BASIC_BYTES(N,n)` (`2n²` units) | `0` |
| UPF `init` | as PF `init` | as PF `init` | as PF `init` | as PF `init` |
| UPF `step` | `FILTERS_UPF_Q_STEP_STORAGE(N,n,m)` = `N+4n²+2m²+4n+3m+Wq` | `FILTERS_UPF_Q_STEP_SIGNAL_STORAGE(N,n,m)` = `N·(n²+n+1)+max(s·n,n²+s·m)` | `FILTERS_UPF_Q_STEP_BASIC_BYTES(n,m)` = `0` | `FILTERS_UPF_Q_STEP_DECOMP_STORAGE(n,m)` = `max(n²,m²)` |
| UPF `resample` | as PF `resample` | as PF `resample` | `0` | `0` |

A basic unit is `linears_workspace_unit_t`; allocate
`LINEARS_WORKSPACE_UNITS(required_bytes)` units, not that many bytes or raw
signals. Q estimate scratch is independent of N; Q update keeps N scalar
exponents, not N measurement vectors. Raw staging still grows with the
particle count to preserve atomic commit. PF init/predict intentionally
share the non-Q-named decomposition macro; Q update and UPF step require
their Q-specific decomposition macros. All resample calls need a separate
caller-owned `size_t indices[N]`, despite needing no workspace arrays.

For `N=64, n=2, m=1`, the Q PF maximum requirements across all calls are
71 floating `data` elements, 128 raw `signal` elements, 8 basic units and
4 floating decomposition elements. The Q UPF step requires 123 floating
`data` elements, 458 raw `signal` elements, no basic scratch and 4 floating
decomposition elements. These are array capacities, not total-RAM or
speed measurements. Native f32 PF needs at most 394 `data` elements,
128 basic units and 4 decomposition elements; its UPF step needs 615,
10 and 6 respectively.

Persistent arrays are `N·n+N+n²+m²` signal elements (PF) plus `N·n²`
(UPF): for these dimensions the UPF has 453 elements, 906 bytes in q15 and
1812 bytes in q31 or f32. State/workspace objects, indices, estimate outputs,
stack and callback/RNG state are additional allocations.

### f32 example

A C99 f32 particle filter for the pendulum of `UKF.md` with a caller-owned
generator and an angle residual; one estimate-sized workspace serves every
call for these dimensions:

```c
#include "filters_pf.h"
#include <math.h>
#include <stdint.h>

#define PARTICLES 64

/* Caller-owned generator: xorshift64* with a Box-Muller transform. */
typedef struct { uint64_t state; float spare; int has_spare; } rng_t;

static float rng_uniform(rng_t *rng)
{
    rng->state ^= rng->state >> 12;
    rng->state ^= rng->state << 25;
    rng->state ^= rng->state >> 27;
    return (float)((rng->state * UINT64_C(2685821657736338717)) >> 40) / 16777216.0f;
}

static linears_status_t rng_normal(linears_matrix_t *out, void *context)
{
    rng_t *rng = (rng_t *)context;
    float *draw = (float *)out->data;
    size_t i;
    for (i = 0; i < out->rows; ++i) {
        float u, v, s;
        if (rng->has_spare) { draw[i] = rng->spare; rng->has_spare = 0; continue; }
        do {
            u = 2.0f * rng_uniform(rng) - 1.0f;
            v = 2.0f * rng_uniform(rng) - 1.0f;
            s = u * u + v * v;
        } while (s >= 1.0f || s == 0.0f);
        s = sqrtf(-2.0f * logf(s) / s);
        draw[i] = u * s;
        rng->spare = v * s;
        rng->has_spare = 1;
    }
    return LINEARS_OK;
}

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
    size_t i;
    (void)context;
    while (d >= 3.14159265f) d -= 6.2831853f;
    while (d < -3.14159265f) d += 6.2831853f;
    ((float *)out->data)[0] = d;
    /* estimate also applies this residual to the two-component state. */
    for (i = 1; i < out->rows; ++i)
        ((float *)out->data)[i] = ((const float *)a->data)[i] - ((const float *)b->data)[i];
    return LINEARS_OK;
}

int main(void)
{
    static float particles[PARTICLES * 2], weights[PARTICLES];
    static float data[FILTERS_PF_ESTIMATE_STORAGE(PARTICLES, 2)];
    static linears_workspace_unit_t basic[LINEARS_WORKSPACE_UNITS(FILTERS_PF_ESTIMATE_BASIC_BYTES(PARTICLES, 2))];
    static float decomposition[FILTERS_KALMAN_MAX_SIZE(FILTERS_PF_PREDICT_DECOMP_STORAGE(2),
                                                       FILTERS_PF_UPDATE_DECOMP_STORAGE(1))];
    static size_t indices[PARTICLES];
    float x[2] = {3.0f, 0.0f}, p[4] = {0.1f, 0, 0, 0.1f}, q[4] = {1e-4f, 0, 0, 1e-3f}, r[1] = {0.05f};
    float z_value = -3.1f, dt = 0.01f, mean[2], covariance[4], ess;
    filters_ukf_f32_workspace_t workspace = {
        data, FILTERS_PF_ESTIMATE_STORAGE(PARTICLES, 2), basic, sizeof basic,
        {decomposition, sizeof decomposition / sizeof decomposition[0]}
    };
    filters_pf_f32_t filter = {{particles, PARTICLES, 2}, {weights, PARTICLES, 1}, {q, 2, 2}, {r, 1, 1},
                               PARTICLES * 2, PARTICLES, 4, 1};
    linears_matrix_t x0 = {x, 2, 1}, p0 = {p, 2, 2}, z = {&z_value, 1, 1};
    linears_matrix_t mean_out = {mean, 2, 1}, covariance_out = {covariance, 2, 2};
    rng_t rng = {UINT64_C(0x9E3779B97F4A7C15), 0, 0};
    linears_status_t status = filters_pf_f32_init(&filter, &x0, &p0, &filter.process_noise,
                                                  &filter.measurement_noise, rng_normal, &rng, workspace);
    if (status == LINEARS_OK)
        status = filters_pf_f32_predict(&filter, pendulum, &dt, rng_normal, &rng, workspace);
    if (status == LINEARS_OK)
        status = filters_pf_f32_update(&filter, &z, angle_only, NULL, wrap_angle, NULL, workspace);
    if (status == LINEARS_OK)
        status = filters_pf_f32_effective_sample_size(&filter.weights, &ess);
    if (status == LINEARS_OK && ess < PARTICLES / 2)
        status = filters_pf_f32_resample(&filter, rng_uniform(&rng), indices, PARTICLES, workspace);
    if (status == LINEARS_OK)
        status = filters_pf_f32_estimate(&filter, wrap_angle, NULL, &mean_out, &covariance_out, workspace);
    return status == LINEARS_OK ? 0 : 1;
}
```

In general size each array by the largest macro among the calls you make;
the decomposition array above takes the larger of the predict (Cholesky of Q)
and update (solve with R) requirements.

## Precision and limitations

- Process and measurement noise are additive Gaussian: `predict` and `step`
  draw `L*xi`, `update` and `step` weight with Gaussian exponents of `R` and
  `Q`. Non-Gaussian likelihoods need a caller-side loop over
  `gaussian_exponent`/`reweight` replacements; the kernels and the public
  `weights` field make that possible but nothing is provided.
- Resampling is systematic only and never automatic; the caller chooses the
  ESS threshold. There is no roughening, regularization, MCMC move step,
  auxiliary variable, or adaptive particle count, so sample impoverishment
  after repeated resampling is the caller's concern.
- The UPF transition must be deterministic (noise enters only through `Q`),
  because the prior exponent evaluates `f(x_i)` again for each particle.
  Each particle runs a UKF predict/update plus proposal sampling and density
  evaluation, and the state grows with `N·n²`. Proposals are per-particle
  UKF posteriors whose spread includes the carried `P_i`; their weights are
  not near-uniform even for linear Gaussian models. Q proposals retain
  local floating decompositions and triangular solves; Q storage alone
  does not make the entire UPF integer-only.
- `init` draws the particles from `N(mean, P0)` with a Cholesky factor, so
  `P0` must be positive definite; the UKF accepts a semidefinite `P0` at init,
  the PF does not.
- All formats reject nonfinite floating callback outputs and draws. Native
  floating weights and raw Q relative weights have different precision
  limits; a particle assigned zero weight contributes no mass until an
  operation such as resampling replaces that weight. See the Q weight
  contract above rather than applying f32/f64 underflow bounds to Q.
- Q particles, covariances and estimates must lie in `[-1,+1]`; a transaction
  that would leave the interval fails with `RANGE` and commits nothing.
- Estimates are Monte Carlo: the statistical error scales with
  `1/sqrt(ESS)`, independent of the numeric format.

## Validation

Host tests live in [`test/pf.cc`](test/pf.cc) and
[`test/upf.cc`](test/upf.cc), sharing `test/pf_support.h` and
`test/ukf_support.h`; [`test/q_estimators.cc`](test/q_estimators.cc) adds
dedicated Q regressions. From the configured SDK root:

```sh
xmake test -g xhive -v 'filters_test_pf/pf' 'filters_test_upf/upf' 'filters_test_q_estimators/q_estimators'
```

The suites cover:

- Reweighting against long-double exponentials; ESS against its closed
  form; systematic resampling against an independent index generator.
- Scripted perturbations and gathers, Gaussian exponents against Eigen,
  and raw Q weights interpreted as a scale-free distribution.
- Nonlinear pendulum PF/UPF transactions against independent long-double
  references, plus linear-Gaussian and tracking checks with Monte Carlo
  tolerances rather than claims of exact Kalman equivalence.
- Wrapped-angle residuals, Q particle-range rejection, and per-particle
  covariance gathering during UPF resampling.
- Exact capacities, aliasing, misalignment, callback/sampler failures,
  nonfinite values, invalid Q/R and singular proposal covariances, with
  persistent-state and output preservation on rejected transactions.
- Q callback interleaving and early exit, large raw-weight pipelines,
  expanded likelihood residuals and temporary UPF priors, and exact-minimum
  workspaces for both state-dominant and measurement-dominant dimensions.
- Constant-expression storage macros and overflow sentinels.

The complete f32 C example above was independently compiled and linked
with GCC and Clang using `-std=c99 -O2 -pedantic-errors -Wall -Wextra
-Werror -Wvla`, and returned zero with each compiler. That is an example
check, not a pass report for the estimator suites.

Current integration results are recorded under [Host tests](README.md#host-tests).
The earlier floating-wrapper test totals and sanitizer results are not
carried forward as validation of these Q kernels. Match the source revision
when reproducing a result. See [measurement tools and scope](README.md#measurements)
for timing, allocation and ARM object-code comparisons. Host simulation is
not MCU, hardware or interrupt-timing validation.
