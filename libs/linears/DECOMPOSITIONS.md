# Matrix decompositions

`linears_decomp.h` exposes f32, f64, q15 and q31 versions of the same eight
operations. The production implementation is C99 and has no Eigen dependency.
Eigen is used only by the host tests.

## Representation and storage

All matrices are contiguous row-major `linears_matrix_t` values:

```c
{data, rows, cols}
```

A q15 element represents `int16_t raw / 2^15`; q31 represents
`int32_t raw / 2^31`. There is no matrix, scalar, temporary or metadata exponent.
Floating routines use native float or double. Q outputs never select a new
binary point or rescale the represented problem to make a factor fit.

The fixed-point algorithms call the shared integer-only `linears_fixed_*`
arithmetic. Every working value is an `int64_t` divided by exactly `2^31`,
with range `[-2^32, 2^32-2^-31]`. Multiplication, division and square roots round
to nearest with halfway cases away from zero. Their absolute working rounding
error is at most `2^-32`; relative precision deteriorates near zero. Selected
inner products and subtraction of products accumulate in a signed two-limb
128-bit integer with exactly 62 fractional bits, then round once. This uses
portable `uint64_t` limbs, not the `__int128` extension or a floating wrapper.

Before committing a Q result, every computed working value must be in the
closed interval `[-1,+1]`. An out-of-domain value returns RANGE, with no general
saturation. Inside that domain, quantization uses the nearest available signal
code: positive values whose rounded code reaches `2^F`, including logical +1,
map to `INT_MAX`. Their error is at most one signal LSB; ordinary q15 rounding
has a half-LSB bound. q31 export is exact except for the positive endpoint,
because work and q31 have the same fractional-bit count. Negative one is exact.

Consequently, the stored diagonal of a logically unit L factor is
`INT_MAX/2^F`, not exact +1. Small work pivots can round to zero when exported
to q15; working arithmetic itself can round still smaller values to zero.
A successful factorization does not promise that final stored factors retain
the input rank. Domain and pivot checks concern rounded working values, not
an exact-real or interval proof about an arbitrary mathematical factorization.

### Caller-owned workspace

Each function takes one workspace **by value**:

- `linears_f32_workspace_t`: `float *data`, element `capacity`.
- `linears_f64_workspace_t`: `double *data`, element `capacity`.
- `linears_q15_workspace_t` and `linears_q31_workspace_t`:
  `linears_fixed_t *data` (int64 fixed-F31 working values), plus element `capacity`.
  This is not raw int16/int32 signal storage.

This capacity is an element count, not the byte count used by the basic matrix
operator workspace. Callers can declare constant-size arrays directly:

```c
#include "linears_decomp.h"

/* All arrays can instead be members of a caller-owned long-lived context. */
float work[LINEARS_SOLVE_WORKSPACE(3, 2)];
linears_f32_workspace_t workspace = {work, LINEARS_SOLVE_WORKSPACE(3, 2)};
/* linears_solve_f32(&a, &b, &x, workspace); */
```

For square order `n`, right-hand-side count `k`, and rectangular `m` by `n`
input, the required upper bounds are:

| Operation | Typed workspace elements |
| --- | ---: |
| LU, determinant, Cholesky, LDLT | `n*n` |
| Solve | `n*n + n*k` |
| Inverse | `2*n*n` |
| Full QR | `m*n + m*m + m` |
| Thin SVD, `r=min(m,n)` | `m*n + r*r + r` |

The `LINEARS_*_WORKSPACE(...)` macros are constant expressions with overflow
checks; `SIZE_MAX` is an invalid size, not an allocation request. Arguments must
have no side effects. There is no arbitrary dimension ceiling or configurable
maximum-sized internal array. Array extents are validated before traversal.
No routine uses malloc/free, VLA, alloca, recursion, or dimension-sized local
arrays. Scalars, fixed-size descriptor pointer lists, and call frames are the
only automatic storage. The caller, not the algorithm, chooses where scratch
arrays live.

`linears_work.h` additionally exposes only the two factorization operations
needed for wide intermediate matrices: `linears_solve_fixed` and
`linears_cholesky_fixed`. Their matrix data is `linears_fixed_t[]`; their
`linears_fixed_workspace_t` uses the same element-count formulas. Results keep
the full int64 fixed-F31 range and exact `LINEARS_FIXED_ONE`, without narrowing
to a Q signal format. Kalman covariance, gain and Joseph intermediates use these
entries rather than duplicating algorithms or replacing exact working identity
with the Q signal code `INT_MAX`. They instantiate the same solve/Cholesky
kernels; no unused wide LU/QR/SVD API is added.

### Alignment, aliases and failure

Data and workspace require natural type alignment; descriptors and metadata
are alignment-checked before dereference. There is no stronger SIMD alignment
requirement. All output data, output descriptors, used workspace, input
storage/descriptors and metadata must be disjoint. Two read-only inputs can
share data. Extra workspace capacity is not touched. Descriptors themselves
must not be embedded in any matrix data range used by the call.

Matrix data and descriptors are unchanged on every failure. LU and LDLT
permutation/block metadata can be partially modified after a numerical failure;
it is meaningful only after `LINEARS_OK`. Argument failures do not modify that
metadata. Scratch contents are unspecified after a call. Independent calls are
reentrant; sharing mutable storage requires caller serialization.

NaN/Inf input or a nonfinite floating intermediate yields `LINEARS_ERROR_RANGE`.
Fixed working overflow and out-of-domain Q outputs return RANGE instead of
wrapping or rescaling. Native IEEE underflow can produce zero. Fixed working
underflow can likewise round to zero at its invariant `2^-31` quantum.
An intermediate range error can occur even when a mathematically equivalent
rearrangement would avoid it. The documented norm, reflector, determinant and
LDLT calculations address specific cancellation/overflow/underflow paths, not
unlimited precision or universal scale invariance.

## Algorithms and boundaries

### Partial-pivot LU, solve, inverse and determinant

`linears_lu_*` produces `P*A=L*U`, where row `i` of `P*A` is original row
`permutation[i]`. L is unit lower triangular and U is upper triangular. Parity
is +1 for an even number of row swaps and -1 for an odd number. Entire rows,
including previously computed L entries, follow each pivot swap.

`linears_solve_*` solves all columns of `A*X=B` without forming an inverse.
Row swaps and elimination are applied to the right-hand sides as LU proceeds,
then upper-triangular back substitution completes the solve. `linears_inverse_*`
uses the same operation with an exact working identity right-hand side. It
inverts the actual stored input: a Q signal diagonal `INT_MAX` equals `1-2^-F`,
so its scalar inverse is larger than one and returns RANGE, not a clamped one.
The inverse of stored -1 is exactly -1. Nontrivial representable inverses exist,
for example the inverse of `[[.75,.75],[.75,-.75]]` has entries of magnitude 2/3.

Partial pivoting keeps logical L multipliers within one, but U can grow beyond
the Q signal range. Such a LU output returns RANGE. Solve and determinant use
the wide working factor internally and can still succeed when their own final
results fit. Fixed Schur and RHS updates use one F62 subtraction-of-product
rounding; back substitution subtracts a complete F62 dot product before the
final working division.

A zero working pivot gives `LINEARS_ERROR_SINGULAR` for LU/solve/inverse. No
arbitrary small-pivot threshold is used, and success is not a condition-number
estimate. LU and solve do not perform iterative refinement. A zero-column
right-hand side is supported, but its coefficient matrix is still factored.

`linears_determinant_*` returns a 1-by-1 matrix. It returns successful zero for a
singular matrix and logical one for a 0-by-0 matrix (the Q positive endpoint
code). Native float/double retain their frexp-based diagonal product to avoid
multiplication-order overflow. Fixed products never use exponent state: when
the running absolute product is at most one, the largest remaining pivot
magnitude is selected; otherwise the smallest is selected. Only the private
LU diagonal order is changed. This balances direct fixed-F31 multiplications,
avoiding a premature zero followed by amplifying large factors. Final working
or signal overflow still fails; a genuinely tiny product can round to zero.
LU/solve/inverse also accept order zero.

Published algorithm references, not vendored source:

- LAPACK [DGETF2](https://www.netlib.org/lapack/double/dgetf2.f), unblocked
  partial-pivot LU; [DGETRF](https://www.netlib.org/lapack/double/dgetrf.f)
  documents the `P/L/U` factorization.
- LAPACK [DGETRS](https://www.netlib.org/lapack/double/dgetrs.f), solves from LU.
  This implementation applies row swaps/elimination to B during factorization
  rather than exposing a separate reusable packed-LU solve API.
- Golub and Van Loan, *Matrix Computations*, 4th edition (2013), Gaussian
  elimination with partial pivoting and triangular solves.

### Cholesky

`linears_cholesky_*` produces the lower-triangular L in `A=L*L^T`. It requires
**exact symmetry of the stored input**, checks both triangles, and returns
DIMENSION on nonsymmetry. It does not silently discard one triangle or
symmetrize input. Floating matrix products can differ by a rounding unit between
the two triangles; callers constructing covariance matrices must explicitly
choose their symmetric representation before calling.

A nonpositive computed diagonal pivot returns
`LINEARS_ERROR_NOT_POSITIVE_DEFINITE`. Positive semidefinite/singular inputs are
not accepted as SPD. The upper output triangle is zero; order zero succeeds.
Fixed Cholesky subtracts each complete dot product in F62 before one F31
rounding; a nonpositive rounded pivot still fails, even if the exact-real input
was positive definite. The algorithm is the unblocked lower-triangular
Cholesky recurrence published in LAPACK [DPOTF2](https://www.netlib.org/lapack/double/dpotf2.f).

### Full Householder QR

`linears_qr_*` produces Q of size `m*m` and R of size `m*n`, with `A=Q*R`.
It supports tall, square and wide inputs. R is upper trapezoidal; Q has a full
orthonormal basis even when A is rank-deficient. The implementation has no
column pivoting and does not estimate numerical rank.

The reflector vector is formed from a scaled column as
`x/||x|| + sign(x[0])*e0` and normalized, avoiding the cancellation-prone sign
and an unnecessary `x[0]-alpha` overflow. Norms divide by the largest absolute
entry before squaring. Applying a reflector to R similarly works on a scaled
column, then restores its actual magnitude. In fixed arithmetic the direction
is formed before restoring the tiny actual norm: the raw column `[1,1]` must
not be divided by a norm already rounded to one working LSB. All values and
ratios keep the same F31 binary point; this is ordinary mathematical division,
not a representation-layer exponent. Q accumulates the reflectors on its right.
For `n=0`, Q is the `m*m` identity (positive endpoint codes in Q output);
`m=0` gives empty outputs. Fixed R can exceed one even when all input elements
fit: the column `[.75,.75]` has norm greater than one and returns RANGE without
publishing Q or R.

References: Householder, *Unitary Triangularization of a Nonsymmetric Matrix*,
JACM 5(4), 1958; LAPACK [DGEQR2](https://www.netlib.org/lapack/double/dgeqr2.f)
and [DLARFG](https://www.netlib.org/lapack/double/dlarfg.f). This is an explicit,
unblocked reflector implementation, not LAPACK's blocked packed-reflector API.

### Symmetric-indefinite LDLT

`linears_ldlt_*` uses Bunch–Parlett complete symmetric pivoting with 1-by-1 and
2-by-2 diagonal blocks, so zero-diagonal exchange matrices with a nonzero
representable off-diagonal are supported, including raw ±1. It is
not an SPD-only LDLT presented as an indefinite solver. Both triangles must be
exactly symmetric. Outputs satisfy `P*A*P^T=L*D*L^T`; `blocks[i]` is 1, 2, or 0
for a scalar pivot, the start of a 2-by-2 block, or its continuation. The method
can generate L or D entries outside the Q output range; these are explicit
RANGE failures, not truncated factors. For instance `[[.5,.75],[.75,.5]]`
requires an L multiplier of 1.5. Logical unit L diagonals use the documented
positive endpoint code on export.

See [the LDLT algorithm note](docs/decomp_ldlt.md) for the pivot rule, scaled
2-by-2 arithmetic, singular behavior, references and quantization boundaries.

### Thin SVD

`linears_svd_*` returns U(m,r), s(r,1), V(n,r), where `r=min(m,n)` and
`A ≈ U*diag(s)*V^T`. It uses bounded cyclic one-sided Jacobi sweeps, transposes
wide problems internally, sorts nonnegative singular values in descending
order, and completes numerical-zero columns with two-pass modified Gram–Schmidt.
V is returned, not its transpose. The input is divided by its largest absolute
value during the numerical iteration, then singular values are multiplied by
that actual magnitude before domain validation. The scale is an ordinary F31
value with no binary-point metadata, and the decoded input reference is never
changed. This retains accurate directions even for raw-one input columns.
Computed singular values outside [-1,+1] cannot be exported to Q signals.

The caller supplies `max_sweeps`; zero checks the initial state without doing a
rotation sweep. `LINEARS_ERROR_NO_CONVERGENCE` reports exhausted budget or
insufficient working precision for the required basis. A required fixed-point
rotation that rounds to zero or leaves both columns unchanged is reported as
NO_CONVERGENCE rather than consuming a huge budget without progress. This implementation
uses an explicit dimension/working-epsilon numerical-zero threshold. It is not
a guarantee of high relative accuracy for arbitrarily tiny singular values.
The threshold, truncation residual bound, references and examples are in the
[SVD algorithm note](docs/decomp_svd.md).

## Host comparison tests

Tests are split into LU, solve/inverse, determinant, Cholesky, QR, LDLT, SVD and
shared API contracts, plus the two necessary wide working operations. Every
signal-format numerical category instantiates all four formats.
The Eigen reference is always built from the **decoded quantized input**, not
the unquantized random sample. Seeds are fixed.

For factors, tests separate absolute fixed-F31 working roundoff from final
signal quantization. A q15 ordinary output has at most half a `2^-15` quantum
of error; q31 ordinary export from F31 is exact. Positive endpoint codes are
charged up to one full signal LSB. The Frobenius budget combines these bounds
per element, and product tests propagate them through actual factor norms.
Native arithmetic retains its relative error model. Solve tests use
condition-aware bounds for ill-conditioned represented inputs. SVD tests add
its explicit numerical-zero truncation and final amplitude multiplication
roundoff. Fixed successful fixtures use explicit small actual inputs whose
factors fit; separate tests require RANGE for genuinely unrepresentable results.
No implicit input normalization or widened fixed tolerance substitutes for this.

Coverage includes pivoting, multiple right-hand sides, positive/negative data,
0/1 dimensions, tall/wide matrices, singular/indefinite/rank-deficient matrices,
repeated singular values, native magnitude spans, fixed raw minima/maxima,
working and final-output underflow, nonfinite inputs, iteration budgets,
aliases, workspace/descriptor alignment, insufficient capacities, output and
workspace sentinels, failure atomicity, and independent concurrent calls.
Host and sanitizer results are software evidence only; they do not constitute
ARM board validation or a WCET measurement.
