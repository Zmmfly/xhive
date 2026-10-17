# Matrix decompositions

`linears_decomp.h` exposes f32, f64, q15 and q31 versions of the same eight
operations. The production implementation is C99 and has no Eigen dependency.
Eigen is used only by the host tests.

## Representation and storage

All matrices are contiguous row-major `linears_matrix_t` values:

```c
{data, rows, cols, exponent}
```

A q15 element represents `raw * 2^(exponent - 15)`. A q31 element represents
`raw * 2^(exponent - 31)`. Floating routines ignore input exponents and write
zero output exponents. Every fixed-point output matrix has its own automatically
selected block exponent. In particular, L and U, or L and D, do **not** share an
exponent: their magnitudes and physical units differ.

The fixed-point algorithms call the shared integer-only `linears_fixed_*`
arithmetic. Its normalized working scalar has 31 significant magnitude bits,
a separate binary exponent, and nearest rounding with ties away from zero.
Products, division and square roots use at most 64-bit integers. There is no
floating conversion, `__int128` dependency, or exact wide dot-product
accumulator in the q15/q31 implementations. q15 intermediates are not rounded
to 15 bits after each operation; final factors are quantized to q15 blocks.

Output exponent selection checks the actual most-negative and most-positive
values. It preserves the extra negative integer endpoint (`INT16_MIN` or
`INT32_MIN`) when rounding there is valid, rather than testing an unsigned
magnitude as if it were positive. Small elements can round to zero at the
shared output exponent. A successful fixed-point factorization therefore does
not guarantee that quantized factors retain tiny pivots or the input rank.

### Caller-owned workspace

Each function takes one workspace **by value**:

- `linears_f32_workspace_t`: `float *data`, element `capacity`.
- `linears_f64_workspace_t`: `double *data`, element `capacity`.
- `linears_q15_workspace_t` and `linears_q31_workspace_t`: canonical q31 working
  scalar storage (`linears_q31_t`, also named `linears_fixed_t` in
  `linears_numeric.h`), plus element `capacity`.

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

### Alignment, aliases and failure

Data and workspace require natural type alignment; descriptors and metadata
are alignment-checked before dereference. There is no stronger SIMD alignment
requirement. All output data, output descriptors, used workspace, input
storage/descriptors and metadata must be disjoint. Two read-only inputs can
share data. Extra workspace capacity is not touched. Descriptors themselves
must not be embedded in any matrix data range used by the call.

Matrix data and output exponents are unchanged on every failure. LU and LDLT
permutation/block metadata can be partially modified after a numerical failure;
it is meaningful only after `LINEARS_OK`. Argument failures do not modify that
metadata. Scratch contents are unspecified after a call. Independent calls are
reentrant; sharing mutable storage requires caller serialization.

NaN/Inf input or a nonfinite floating intermediate yields `LINEARS_ERROR_RANGE`.
Fixed exponent overflow and unrepresentable block exports return RANGE instead
of saturation or wrapping. Native IEEE underflow can produce zero; these are
native float/double algorithms, not an extended-exponent floating format.
An intermediate range error can occur even when a mathematically equivalent
rearrangement would avoid it. The documented scaled-norm, reflector, determinant
and LDLT pivot calculations eliminate specific avoidable overflow paths; they
do not promise universal scaling invariance at every exponent boundary.

## Algorithms and boundaries

### Partial-pivot LU, solve, inverse and determinant

`linears_lu_*` produces `P*A=L*U`, where row `i` of `P*A` is original row
`permutation[i]`. L is unit lower triangular and U is upper triangular. Parity
is +1 for an even number of row swaps and -1 for an odd number. Entire rows,
including previously computed L entries, follow each pivot swap.

`linears_solve_*` solves all columns of `A*X=B` without forming an inverse.
Row swaps and elimination are applied to the right-hand sides as LU proceeds,
then upper-triangular back substitution completes the solve. A and B may have
independent fixed-point exponents. `linears_inverse_*` uses the same operation
with an identity right-hand side.

A zero working pivot gives `LINEARS_ERROR_SINGULAR` for LU/solve/inverse. No
arbitrary small-pivot threshold is used, and success is not a condition-number
estimate. LU and solve do not perform iterative refinement. A zero-column
right-hand side is supported, but its coefficient matrix is still factored.

`linears_determinant_*` returns a 1-by-1 matrix. It returns successful zero for a
singular matrix and one for a 0-by-0 matrix. It multiplies normalized diagonal
mantissas while accumulating their binary exponents separately, avoiding an
intermediate overflow solely from multiplying huge factors before tiny ones.
Final exponent/range limits still apply. LU/solve/inverse also accept order zero.

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
The algorithm is the unblocked lower-triangular dot-product Cholesky recurrence
published in LAPACK [DPOTF2](https://www.netlib.org/lapack/double/dpotf2.f).

### Full Householder QR

`linears_qr_*` produces Q of size `m*m` and R of size `m*n`, with `A=Q*R`.
It supports tall, square and wide inputs. R is upper trapezoidal; Q has a full
orthonormal basis even when A is rank-deficient. The implementation has no
column pivoting and does not estimate numerical rank.

The reflector vector is formed from a scaled column as
`x/||x|| + sign(x[0])*e0` and normalized, avoiding the cancellation-prone sign
and an unnecessary `x[0]-alpha` overflow. Norms divide by the largest absolute
entry before squaring. Applying a reflector to R similarly works on a scaled
column, then restores its scale. Q accumulates the same reflectors on its right.
For `n=0`, Q is the `m*m` identity; `m=0` gives empty outputs.

References: Householder, *Unitary Triangularization of a Nonsymmetric Matrix*,
JACM 5(4), 1958; LAPACK [DGEQR2](https://www.netlib.org/lapack/double/dgeqr2.f)
and [DLARFG](https://www.netlib.org/lapack/double/dlarfg.f). This is an explicit,
unblocked reflector implementation, not LAPACK's blocked packed-reflector API.

### Symmetric-indefinite LDLT

`linears_ldlt_*` uses Bunch–Parlett complete symmetric pivoting with 1-by-1 and
2-by-2 diagonal blocks, so matrices such as `[[0,1],[1,0]]` are supported. It is
not an SPD-only LDLT presented as an indefinite solver. Both triangles must be
exactly symmetric. Outputs satisfy `P*A*P^T=L*D*L^T`; `blocks[i]` is 1, 2, or 0
for a scalar pivot, the start of a 2-by-2 block, or its continuation.

See [the LDLT algorithm note](docs/decomp_ldlt.md) for the pivot rule, scaled
2-by-2 arithmetic, singular behavior, references and quantization boundaries.

### Thin SVD

`linears_svd_*` returns U(m,r), s(r,1), V(n,r), where `r=min(m,n)` and
`A ≈ U*diag(s)*V^T`. It uses bounded cyclic one-sided Jacobi sweeps, transposes
wide problems internally, sorts nonnegative singular values in descending
order, and completes numerical-zero columns with two-pass modified Gram–Schmidt.
V is returned, not its transpose.

The caller supplies `max_sweeps`; zero checks the initial state without doing a
rotation sweep. `LINEARS_ERROR_NO_CONVERGENCE` reports exhausted budget or
insufficient working precision for the required basis. This implementation
uses an explicit dimension/working-epsilon numerical-zero threshold. It is not
a guarantee of high relative accuracy for arbitrarily tiny singular values.
The threshold, truncation residual bound, references and examples are in the
[SVD algorithm note](docs/decomp_svd.md).

## Host comparison tests

Tests are split into LU, solve/inverse, determinant, Cholesky, QR, LDLT, SVD and
shared API contracts. Every numerical category instantiates all four formats.
The Eigen reference is always built from the **decoded quantized input**, not
the unquantized random sample. Seeds are fixed.

For factors, tests separate working arithmetic error from final block rounding.
A block containing `N` values at exponent `e` and `b` fractional bits has
Frobenius rounding bound `sqrt(N)*2^(e-b-1)`. Product tests propagate these bounds
through the actual factor norms. Solve tests use condition-aware forward/error
bounds for ill-conditioned represented inputs. SVD tests separately account for
its documented numerical-zero truncation. No large fixed tolerance is used as
a replacement for accounting for these effects.

Coverage includes pivoting, multiple right-hand sides, positive/negative data,
0/1 dimensions, tall/wide matrices, singular/indefinite/rank-deficient matrices,
repeated singular values, exponent spans, nonfinite inputs, iteration budgets,
aliases, workspace/descriptor alignment, insufficient capacities, output and
workspace sentinels, failure atomicity, and independent concurrent calls.
Host and sanitizer results are software evidence only; they do not constitute
ARM board validation or a WCET measurement.
