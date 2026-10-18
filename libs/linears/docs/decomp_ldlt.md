# Symmetric-indefinite LDLT

`linears_ldlt_f32`, `linears_ldlt_f64`, `linears_ldlt_q15`, and
`linears_ldlt_q31` compute a **block** LDLT factorization. They are not an
SPD-only LDLT implementation. In particular, `[[0, 1], [1, 0]]` is a valid
floating-point input and produces a 2-by-2 pivot. For fixed point, the same
holds for `[[0, b], [b, 0]]` with every nonzero representable q15/q31 value b,
including raw `1` and the negative endpoint `-1`.

## Algorithm and sources

The algorithm is unblocked Bunch–Parlett complete symmetric pivoting:

- J. R. Bunch and B. N. Parlett, “Direct Methods for Solving Symmetric
  Indefinite Systems of Linear Equations,” *SIAM Journal on Numerical
  Analysis* 8 (1971), pp. 639–655.
- N. J. Higham, “[Stability of the Diagonal Pivoting Method with Partial
  Pivoting](https://nhigham.com/wp-content/uploads/2022/11/high97d.pdf),”
  *SIAM Journal on Matrix Analysis and Applications* 18 (1997), pp. 52–65.
  Section 1 describes block LDLT and distinguishes complete from partial
  pivoting. Section 4.2 discusses stable solution of the 2-by-2 pivot systems.
- “[Performance Optimization of Symmetric Factorization
  Algorithms](https://cs.nyu.edu/overton/mstheses/tarca/perf_optm_sym_factor.pdf),”
  Section 5, describes unblocked Schur updates and Bunch–Parlett complete
  pivoting alongside partial and rook pivoting.

At each stage, search the entire active symmetric Schur complement. Let
`eta` be the largest absolute diagonal entry and `xi` the largest absolute
off-diagonal entry. Use `alpha = (1 + sqrt(17)) / 8`:

1. If both maxima are zero, report `LINEARS_ERROR_SINGULAR`.
2. If `eta >= alpha * xi`, move a diagonal attaining `eta` to the current
   pivot position and take a 1-by-1 block.
3. Otherwise, move the pair attaining `xi` to the next two pivot positions
   and take a 2-by-2 block.

Equal maxima are resolved by the first row-major index encountered. When
`eta < xi`, the comparison uses `eta/xi >= alpha` rather than multiplying
`alpha*xi`; this avoids rounding a tiny fixed-point threshold to zero merely
to select a pivot. For `eta >= xi`, a 1-by-1 pivot is
selected without arithmetic. Symmetric interchanges also swap the already
computed rows of L. Pivot search and arithmetic each require O(n³) work; this
implementation deliberately chooses complete pivoting rather than the cheaper
partial-pivot search.

For a 2-by-2 block `B = [[a, b], [b, c]]`, complete pivoting gives
`|a/b|, |c/b| < alpha`. Consequently
`|det(B)| >= (1 - alpha²) * b²` in exact arithmetic. The implementation solves
for the multipliers using the scaled denominator `(a/b)*(c/b) - 1`, not
`a*c - b*b`. It never squares `b`, avoiding determinant overflow or underflow
caused solely by the absolute scale of B. This is a scalar reformulation of
the 2-by-2 inverse, not a floating-point conversion of fixed-point data.

For a coupling row `[x, y]`, the multipliers are

```text
l0 = ((c/b)*(x/b) - y/b) / ((a/b)*(c/b) - 1)
l1 = ((a/b)*(y/b) - x/b) / ((a/b)*(c/b) - 1)
```

The Schur complement is updated by `S <- S - L21*C^T`. For q15/q31, the
denominator, multiplier numerators and one-/two-column Schur updates accumulate
exact products in the common two-limb F62 accumulator, then round once to F31.
In particular, `round(s - x*y)` is not replaced by `s - round(x*y)`; these
differ at halfway cases with nearest-away rounding. Each updated
lower-triangle value is copied to the corresponding upper-triangle position
so later searches use exactly the same computed value in both triangles.

## Representation and workspace

All three matrices A, L and D have shape n-by-n and row-major storage.

```text
P*A*P^T = L*D*L^T
(P*A*P^T)[i,j] = A[permutation[i], permutation[j]]
```

The working factor L is unit lower triangular. Its fixed-point stored diagonal
is `INT16_MAX` or `INT32_MAX`, the closest available value below +1; the
reconstruction identity is therefore approximate after fixed-point export.
D is symmetric block diagonal. `blocks[k] == 1`
marks a 1-by-1 block; `blocks[k] == 2` marks the start of a 2-by-2 block, and
`blocks[k+1] == 0` its continuation. The within-block entry `L[k+1,k]` is zero.
D is explicitly zero outside its declared blocks; L is zero above its diagonal.

The workspace requirement is exactly `LINEARS_LDLT_WORKSPACE(n) == n*n`
**typed elements**, not bytes. Its type is `float`, `double`, or
`linears_fixed_t` (`int64_t` with a constant 31-bit fraction) as selected by the
API. The caller supplies naturally
aligned storage and its element capacity. The implementation packs the active
Schur complement, L multipliers and D pivots into this one matrix. Original
couplings remain in the upper triangle during each update; no additional
columns, heap allocations, variable-length arrays, or dimension-dependent
stack storage are used.

`LINEARS_LDLT_WORKSPACE(n)` gives the required maximum scratch count for the
caller-selected matrix order; it returns `SIZE_MAX` on overflow. There is no
arbitrary dimension cap. A zero-order input is valid and needs no matrix data,
metadata elements or scratch elements.

Matrix data, matrix descriptors, permutation and block metadata, and the used
workspace must be mutually disjoint. Both metadata capacities must be at least
n. Unused capacity is not accessed. Independent calls have no shared mutable
state; callers must serialize calls that share storage.

## Numerical and failure boundaries

Both input triangles are checked for exact equality. A nonsymmetric input is
rejected with `LINEARS_ERROR_DIMENSION`; no triangle is silently discarded.
Float NaN/Inf input, nonfinite intermediate results and unrepresentable output
ranges return `LINEARS_ERROR_RANGE`.

Singularity is tested at **working precision**, not by a heuristic tolerance
or a condition estimate. A zero active Schur complement returns
`LINEARS_ERROR_SINGULAR`. A nonzero active complement always admits one of the
two pivot choices above; the selected 2-by-2 block is bounded away from
singularity by the pivot criterion. No zero pivot is replaced, regularized or
silently skipped. Rounding may turn a mathematically singular matrix into a
small nonzero residual, or a sufficiently small residual into zero. Success
does not certify rank or forward accuracy for an ill-conditioned input.

The q15/q31 kernels use only the shared integer arithmetic from
`linears_numeric.h`. Signals are raw `int16_t / 2^15` or `int32_t / 2^31`;
working values are raw `int64_t / 2^31`; wide products have the constant fraction
`2^62`. There is no exponent, normalization, dynamic scaling state,
floating-point operation or `__int128` dependency in a fixed-point call.

Working arithmetic rounds to nearest with halfway cases away from zero. Its
absolute quantum is `2^-31`, not a constant relative precision. Work values
can exceed one, up to the signed int64 F31 range. Arithmetic outside that
range returns `LINEARS_ERROR_RANGE`. For example,
`A=[[1/2,2^-31],[2^-31,0]]` is mathematically nonsingular, but its Schur pivot
`-2^-61` rounds to zero at working precision, so q31 reports SINGULAR. With
coupling `2^-15`, the Schur pivot `-2^-29` survives in F31 work but rounds to
zero in the q15 output. That q15 call succeeds with a singular **stored** D.
The API does not add a second rank test after export.

Fixed output conversion first checks each working value against `[-1,+1]`.
A value outside that interval returns RANGE; it is not saturated or rescaled.
Within the interval, +1 and positive rounding carry map to `INT16_MAX` or
`INT32_MAX`, and -1 is exact. Ordinary q15 output cells have at most half-LSB
error, but its positive endpoint cell has at most one-LSB error. q31 export is
exact for other F31 values and has one-LSB error at +1.

Complete pivoting bounds multipliers, but does not bound them by one. For
example, `[[.5,.75],[.75,.5]]` has a multiplier 1.5 and must return RANGE in a
fixed-point call even though both D pivots fit the signal format. Conversely,
`[[0,.75,.75],[.75,0,-.75],[.75,-.75,0]]` produces a D pivot 1.5. These are
format limits, not evidence that the input is SPD or mathematically singular.

L and D, including their descriptor fields, are unchanged on every failure.
Both factor ranges are checked before either output is written. Permutation/block
metadata is unchanged on invalid-argument failures, but can be partially
written on a numerical failure and is valid only after `LINEARS_OK`. Workspace
contents are unspecified after a call. The input is never modified.

## Test oracle and error model

Tests use Eigen on the **actual stored input values**, including q15/q31 input
quantization, rather than comparing against an unquantized source matrix.
Reconstruction checks apply the documented permutation explicitly and verify
the declared block pattern. Floating roundoff uses a relative operation-count
budget. Fixed working roundoff uses the absolute F31 quantum; final q15
rounding and the positive endpoint error are accounted for separately through
the factor product, including the stored unit diagonal. Representable random
indefinite congruences are tested separately from explicit factor-RANGE cases.
Eigen's diagonal-pivot LDLT is not used as the sole indefinite oracle: it does
not provide the required 2-by-2 block pivot behavior. Eigen dense products and
self-adjoint eigensolvers can check reconstruction and inertia independently
of the chosen pivots.
