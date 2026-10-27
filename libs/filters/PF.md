# Particle and unscented particle filters

## Model and algorithm

The model is the one of [UKF.md](UKF.md): `x_next = f(x) + w`, `z = h(x) + v`
with `cov(w) = Q` (n-by-n), `cov(v) = R` (m-by-m), caller callbacks `f`, `h`
and optional residual callbacks for angle-like components. Instead of one
Gaussian, a filter keeps N particles `x_i` (one n-vector per row) with
normalized weights `w_i`; the posterior is the weighted cloud. The library has
no random generator: callers own one and hand over standard-normal draws
through a sampler callback and a uniform draw to `resample`.

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

`(-)` is the residual: plain subtraction or the caller's callback. `estimate`
reuses the UKF kernels `mean`, `deviations` and `weighted_covariance`, so the
residual-aware mean is the same center-relative pass as the UKF's
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
Inside the implementation the determinant factor is applied as the ratio
`prod_j L_i[j][j] / L_0[j][j]` to particle 0 so that the weight products stay
near one instead of being scaled by tiny absolute determinants before
normalization; the normalized result is the same. `resample` gathers
particles and covariances together; `estimate` and `effective_sample_size`
are the PF functions applied to the embedded `base`.

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
- The shift by `min e` before `exp` is the usual log-sum-exp stabilization;
  the best particle always receives `exp(0) = 1` before normalization, so no
  weight set underflows to zero in floating point unless every weight was
  zero already.

## Ownership and contracts

All storage is caller-owned: the state object, its arrays (particles N-by-n,
weights N-by-1, Q, R, and for the UPF the N-by-(n*n) covariances) with
explicit element capacities, and three scratch arrays. `init` fills the
arrays but never changes shapes, addresses or capacities; Q and R sources may
exactly alias their destination or be the state's own descriptor. There is no
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
callback or sampler output fails with `RANGE`. The exceptions are the
weight-vector kernels themselves: `reweight` may leave a partially modified
vector on failure, which the filters avoid by reweighting a staged copy.

`resample` writes `indices[0..N)` and the state only on success; the kernel
`resample_indices` likewise writes nothing on failure. Indices are
nondecreasing and every particle is copied `floor(N*w_i)` or `ceil(N*w_i)`
times. Because the indices are nondecreasing, the rows are gathered in
place (a forward pass for sources above the destination, a backward pass for
sources below), so `resample` needs no staging scratch in f32/f64. A zero
weight sum, a negative weight, or a uniform draw outside `[0,1)` fails with
`DOMAIN`.

## Kernels

The steps are public working-format kernels (tags `f32` and `f64`; the q15
filters run on `f32`, q31 on `f64`), one point per row, so other particle
schemes can be composed from them together with the UKF kernels:

| Kernel | Computes |
| --- | --- |
| `perturb(points, L, sample, ctx, out)` | `out_i = points_i + L*xi_i`; `out` may be exactly `points` |
| `gaussian_exponent(D, L, out)` | `out_i = (1/2)*d_i^T (L L^T)^-1 d_i` through one pivoted solve per row |
| `reweight(w, e)` | `w_i = w_i*exp(-(e_i - min e))`, normalized to sum one |
| `effective_sample_size(w, out)` | `(sum w)^2 / sum w^2` |
| `resample_indices(w, u, idx, cap)` | systematic resampling indices from one uniform draw |
| `gather(points, idx, count, out)` | `out_i = points[idx[i]]` |

`filters_ukf_*_propagate`, `mean`, `deviations` and `weighted_covariance`
supply the remaining steps; `filters_ukf_*_predict_work` and `update_work`
are the per-particle UKF of the UPF.

## Fixed-fraction Q contract

- q15/q31 states hold raw `int16_t/2^15` or `int32_t/2^31` particles,
  weights, (UPF) covariances, Q and R; the init mean/covariance,
  measurements and estimate outputs are raw too. Nothing is stored wider
  than the signal type.
- Intermediates need range beyond the signal interval, so the q15 filters
  compute in float and the q31 filters in double (the significand covers
  the signal). Sampler and model callbacks see float/double vectors;
  `alpha`, `beta`, `kappa` and uniform draws are float/double. The kernels,
  workspace and model types are the `f32`/`f64` ones.
- Signals widen once on entry. Q and R, the init mean/covariance and the
  measurement are widened whole; particles are widened one row at a time
  into an `n`-element scratch slot as they are handed to the transition or
  observation, so the per-call scratch grows with `n`, not `N·n` (UPF: one
  particle and one `n·n` covariance slot). `estimate` widens the whole set.
- Results narrow once on commit after a `[-1,+1]` range check on the
  computed working values, with the positive-endpoint policy of
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
  `estimate` and the raw `filters_pf_q15/q31_effective_sample_size` divide
  out (`estimate` normalizes the widened weights to sum one and fails with
  `DOMAIN` when they sum to zero). `init` and `resample` reset every weight
  to the raw maximum. A weight below `2^-16` (q15) or `2^-32` (q31) of the
  largest rounds to zero, so a particle more than about 11 or 22 nats behind
  the best one is lost at the commit; floating weights keep `exp(-87)`
  (f32) or `exp(-708)` (f64).
- `exp(-x)` is `expf`/`exp`; there is no fixed-point exponential.

## Workspace and RAM cost

Each call borrows the three arrays of `filters_ukf_*_workspace_t` sized by
the macros below (constant expressions; `SIZE_MAX` means overflow and is
rejected). Only required prefixes are accessed. `W` is the UKF kernel amount
`FILTERS_UKF_UPDATE_WORK_STORAGE(n,m)`.

| Call | `data` elements (float / Q adds) | `basic` bytes | `decomposition` elements |
| --- | --- | --- | --- |
| PF `init` | `FILTERS_PF_PREDICT_STORAGE(N,n)` = `N·n + n² + 2n` / `FILTERS_PF_Q_INIT_STORAGE` `+ n² + n` | `FILTERS_PF_PREDICT_BASIC_BYTES(n)` = `n` units | `FILTERS_PF_PREDICT_DECOMP_STORAGE(n)` = `n²` |
| PF `predict` | `FILTERS_PF_PREDICT_STORAGE(N,n)` / `FILTERS_PF_Q_PREDICT_STORAGE` `+ n² + n` | `n` units | `n²` |
| PF `update` | `FILTERS_PF_UPDATE_STORAGE(N,m)` = `2N·m + 2N + m² + m` / `FILTERS_PF_Q_UPDATE_STORAGE(N,n,m)` `+ n + m² + m` | `FILTERS_PF_UPDATE_BASIC_BYTES(m)` = `m` units | `FILTERS_PF_UPDATE_DECOMP_STORAGE(m)` = `m² + m` |
| PF `resample` | 0 / `FILTERS_PF_Q_RESAMPLE_STORAGE(N)` = `N` | 0 | 0 |
| PF `estimate` | `FILTERS_PF_ESTIMATE_STORAGE(N,n)` = `3N·n + 2n² + n` / `FILTERS_PF_Q_ESTIMATE_STORAGE` `+ N·n + N` | `FILTERS_PF_ESTIMATE_BASIC_BYTES(N,n)` = `n·max(N,n)` units | 0 |
| UPF `init` | as PF `init` | as PF `init` | as PF `init` |
| UPF `step` | `FILTERS_UPF_STEP_STORAGE(N,n,m)` = `N·(n² + n + 2) + 3n² + 6n + m² + 2m + 1 + W` / `FILTERS_UPF_Q_STEP_STORAGE` `+ n + 2n² + m² + m` | `FILTERS_UPF_STEP_BASIC_BYTES(n,m)` = `max(n,m)·(2n+1)` units | `FILTERS_UPF_STEP_DECOMP_STORAGE(n,m)` = `max(n² + n, m² + mn)` |
| UPF `resample` | as PF `resample` | 0 | 0 |

The kernels have their own macros (`FILTERS_PF_PERTURB_*`,
`FILTERS_PF_GAUSSIAN_EXPONENT_*`). Q `data` is float for q15 and double for
q31. The UPF step evaluates the likelihood and prior exponents per particle,
so its scratch grows with `N·n²` (the staged covariances) rather than with
`N·m`; the state itself adds `N·n²` signal elements for the covariances. For
`N = 64, n = 2, m = 1` an f32 PF needs at most 394 working floats
(estimate), 128 basic units and 4 decomposition floats across all calls; a
UPF step needs 615 working floats, 10 basic units and 6 decomposition floats.
The state arrays are `N·n + N + n² + m²` signal elements (PF) plus `N·n²`
(UPF): for the same dimensions the UPF state is 453 elements, 906 bytes in
q15 and 1812 bytes in q31 or f32; the q15 PF's 64 particles and weights take
384 bytes.

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
    (void)context;
    while (d >= 3.14159265f) d -= 6.2831853f;
    while (d < -3.14159265f) d += 6.2831853f;
    ((float *)out->data)[0] = d;
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
  because the prior exponent evaluates `f(x_i)` once more per particle. Each
  particle runs a full UKF predict and update: `2n+2` transition and `2n+2`
  observation evaluations plus three Cholesky factorizations and one solve
  per particle per step, and the state grows with `N·n²`. Proposals are the
  per-particle UKF posteriors, whose spread includes the carried `P_i`, so
  weights are not near-uniform even for linear Gaussian models.
- `init` draws the particles from `N(mean, P0)` with a Cholesky factor, so
  `P0` must be positive definite; the UKF accepts a semidefinite `P0` at init,
  the PF does not.
- Floating formats reject nonfinite inputs, draws and callback outputs;
  weights of very unequal particles underflow to zero below `exp(-87)` (f32,
  also q15 working arithmetic) or `exp(-708)` (f64, q31), and the raw Q
  weights additionally drop particles below `2^-16` (q15) or `2^-32` (q31) of
  the best one when committed.
- Q particles, covariances and estimates must lie in `[-1,+1]`; a transaction
  that would leave the interval fails with `RANGE` and commits nothing.
- Estimates are Monte Carlo: the statistical error scales with
  `1/sqrt(ESS)`, independent of the numeric format.

## Validation

Host tests live in `test/pf.cc` and `test/upf.cc`, sharing `test/pf_support.h`
and `test/ukf_support.h` with the UKF suite. The kernel cases run for the
`f32` and `f64` kernels, the filter cases for all four formats:

- `xmake test -g xhive -v 'filters_test_pf/pf'`: 45 GoogleTest cases, 41 pass
  and the 4 floating instances of the two Q-only cases are skipped by design.
- `xmake test -g xhive -v 'filters_test_upf/upf'`: 17 cases, all pass.

Oracles are independent of the production code:

- `reweight` against `expl` over exponents 0..22 in half-nat steps (both
  uniform and random weights, shifted and unshifted) and
  `DOMAIN`/`ALIAS`/`DIMENSION`/`RANGE` rejection.
- `effective_sample_size` against the closed form for uniform, one-hot and
  random weights, and the raw q15/q31 version on maximum-scaled raw weights
  with its `DOMAIN`/`ARGUMENT`/`DIMENSION` rejection; `resample_indices`
  against an independent systematic
  implementation on the quantized weights for 12 random weight vectors
  (including zero entries and unnormalized totals), with nondecreasing
  indices and `floor`/`ceil` copy counts; `gather` and `perturb` against
  element-wise reconstruction with scripted draws, in place and to a copy;
  `gaussian_exponent` against Eigen `(1/2) d^T (L L^T)^-1 d`, plus `SINGULAR`
  for a zero pivot.
- A 48-particle, 25-step pendulum PF (`f` nonlinear, `h = [sin(theta),
  omega]`) against a long-double bootstrap filter replaying the logged
  (quantized) draws, reseeded from the committed state before every
  transaction: particles after `predict`, weights after `update`, the ESS,
  indices and gathered particles after `resample`, and mean/covariance after
  `estimate` all match within the per-format envelopes without multipliers;
  Q weights are compared as the distribution they define, and the committed
  raw vector is checked to carry the raw maximum.
- A 24-particle, 20-step pendulum UPF against a long-double UPF built from
  the Eigen UKF reference of the UKF suite plus the weight formula above,
  alternating NULL and explicit subtraction residuals; particles and
  covariances match within the envelopes, weights within four times the
  envelope (the `|d|/Q` amplification of working rounding described above;
  q31 misses the plain envelope by up to about 15%).
- Linear Gaussian models against `filters_kalman_matrix_*` posteriors: PF
  with 4000 particles and UPF with 600, means within `0.2 sigma` and
  covariances within 30% of the geometric mean of the diagonals (Monte Carlo
  agreement, not exactness).
- 60-step pendulum tracking with resampling below `N/2` (PF 400, UPF 60
  particles): the final estimate lies within four posterior sigma of the
  simulated truth.
- A three-particle angle cloud around the cut: the residual-aware estimate
  and covariance match the reference and stay on the short side, the naive
  estimate lands across the cut, and a measurement beyond the cut weights the
  particles by short-way distances.
- A Q init whose scripted draw lands a particle at 1.2 fails atomically with
  `RANGE`, as does a predict that shifts a particle past +1; in-range draws
  and an identity transition commit and estimate normally.
- Init validation, capacities, exact/own-descriptor alias acceptance, partial
  and cross alias rejection, workspace bounds and misalignment, NULL
  callbacks, pre-callback Q/R checks (`DOMAIN`, `NOT_POSITIVE_DEFINITE` with
  zero callback counts), callback/sampler/residual failures in the middle of
  the particle loop, nonfinite callback and sampler outputs, zeroed weights,
  zeroed UPF sigma weights, a singular per-particle covariance, and
  resampling that moves covariances with their particles. Every failure path
  checks that the state object and all arrays are byte-identical afterwards
  and that `indices` were not written.
- Size macros as constant expressions with `SIZE_MAX` overflow sentinels.

Comparisons use the per-format envelopes of `KALMAN.md` (`tolerance * (1 +
|reference|)`, f32 `2e-5`, f64 `2e-12`, q15 `4e-3`, q31 `2e-7`), with the one
multiplier noted above; the q31 envelope holds with double working
arithmetic. Additional runs: GCC `-O2` release build of the production
objects and both tests, and a Clang `-fsanitize=address,undefined` build with
`-fno-sanitize-recover=undefined`, both with no sanitizer diagnostics; `pf.c`
passes GCC and Clang `-std=c99 -pedantic-errors -Wall -Wextra -Werror -Wvla
-fsyntax-only`. The example above compiles and returns zero with both
compilers. Host simulation is not MCU, hardware or interrupt-timing
validation.
