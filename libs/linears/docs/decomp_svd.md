# Thin singular value decomposition

## Algorithm and sources

The four `linears_svd_*` entry points use the same allocation-free, column-cyclic,
one-sided Jacobi template. For an `m` by `n` input, `r = min(m,n)` and the
factorization has the form

```text
A = U diag(s) V^T
U: m by r       s: r by 1       V: n by r
```

For wide inputs the template works on `A^T` and exchanges the left and right
factors when exporting. It never explicitly constructs `A^T A`.

The algorithm is based on the following public references, not a new SVD method:

- Robert A. van de Geijn and Margaret Myers, *Advanced Linear Algebra: Foundations
  to Frontiers*, [11.3.3, Jacobi's method for computing the Singular Value
  Decomposition](https://www.cs.utexas.edu/~flame/laff/alaff/chapter11-one-sided-Jacobi-method.html).
  This describes column-pair Gram matrices, right-applied Jacobi rotations,
  accumulation into an identity matrix, normalization, and final reordering.
- The same text, [11.3.1, Jacobi
  rotation](https://www.cs.utexas.edu/~flame/laff/alaff/chapter11-Jacobi-rotation.html),
  derives the symmetric two-by-two problem and calls for the stable quadratic
  root rather than trigonometric evaluation of the angle.
- LAPACK [`DGESVJ`](https://www.netlib.org/lapack/double/dgesvj.f) provides a public
  implementation of one-sided Jacobi SVD. Its normalized column inner-product
  stopping criterion and small-root Jacobi parameterization are useful
  references. This implementation is not a port of DGESVJ: it does not implement
  its preconditioning, block scheduling, fast scaled rotations, or high-relative-
  accuracy guarantees.
- Ake Bjorck, [*Gram-Schmidt Orthogonalization: 100 Years and
  More*](https://www.cis.upenn.edu/~cis6100/Gram-Schmidt-Bjorck.pdf), IWMS 2010,
  sections on column-wise modified Gram-Schmidt and reorthogonalization. These
  describe immediate sequential projections and a second orthogonalization
  pass before normalization. Null-column completion uses that MGS2 operation on
  canonical coordinate vectors, not on a numerically dependent input column.

For a pair of columns with squared norms `alpha`, `beta` and inner product
`gamma`, use the rotation

```text
new_p = c * old_p - s * old_q
new_q = s * old_p + c * old_q
```

The small root solves `gamma*t*t + (beta-alpha)*t - gamma = 0`, with `t=s/c`.
It is evaluated using a scaled square root and a non-cancelling denominator.
The same rotation is applied to the accumulating right singular-vector matrix.

Zero singular values need a basis, not zero columns. After the nonzero columns
are normalized, the template projects canonical coordinate vectors twice
against previously accepted columns (modified Gram-Schmidt reorthogonalization)
and normalizes a nonzero residual. The arbitrary basis in a null or repeated-
singular-value subspace is not compared entry-by-entry against another SVD.

## Storage and arithmetic

The caller provides one typed workspace with element capacity

```text
m*n + r*r + r
```

The regions hold the working tall matrix, the accumulated right rotations, and
column norms respectively. Orthogonal completion reuses the working column. No
production path allocates memory, declares a dimension-dependent local array,
or recurses. Calls with disjoint caller-owned storage are reentrant.

`f32` and `f64` use their native scalar precision. Both fixed-point variants use
the shared `linears_fixed_t` arithmetic: an `int64_t` raw integer divided by
`2^31`, with 31 fractional bits on every operation and in every workspace slot.
The public matrices contain only `data`, `rows`, and `cols`; q15 data is
`int16_t / 2^15` and q31 data is `int32_t / 2^31`. There is no exponent or
variable binary-point metadata, including in temporary values. q15/q31 SVD is
not a floating-point wrapper and adds no separate fixed-point arithmetic.

Fixed correlations, rotation linear combinations, and Gram-Schmidt projections
use the common two-`uint64_t`-limb F62 accumulator and round once to F31. Norms
use the shared accumulator-backed helper. Products in an accumulator are exact
for its supplied operands; earlier normalization/division still rounds to F31.
No `__int128` extension is needed. Native floating-point expression order is
unchanged by these integer-only paths.

The working input is divided by its largest absolute element, and the singular
values are multiplied by that value before export. This is ordinary scalar
arithmetic: the divisor itself is an F31 value, and no fractional-bit count or
binary exponent changes. It preserves the directions of an all-small input,
including a column whose raw values are `[1,1]`; dividing that column by an
already rounded one-quantum norm would incorrectly produce `[1,1]` as a unit
vector. Column norms additionally use the shared maximum-magnitude-scaled norm
helper. Pair correlations use normalized columns; the two-by-two rotation is
also scaled before taking its square root. Working integer overflow returns
`LINEARS_ERROR_RANGE`. This is not a high-relative-accuracy SVD for arbitrarily
small singular values.

Q output is intrinsically range-limited. Even when every input entry is a valid
Q value, a singular value can exceed one: the column `[0.75,0.75]` has norm
`sqrt(1.125)`. Any computed F31 output outside `[-1,+1]` fails with `RANGE` before
any output is written. The bound is checked on the computed, rounded working
value, not by an exact symbolic evaluation of the mathematical SVD. An exact
working `+1`, and q15 positive rounding carry from within the domain, encode as
`INT16_MAX` or `INT32_MAX`. This documented endpoint encoding is not permission
to saturate a working value outside the domain. `-1` is exactly representable.

## Iteration and comparison contract

`max_sweeps` limits completed cyclic rotation sweeps. A zero budget permits a
convergence check and succeeds for empty inputs, one-column internal problems,
and already orthogonal columns. A non-converged problem returns
`LINEARS_ERROR_NO_CONVERGENCE`; it does not report an unfinished factorization as
successful. Singular values are nonnegative and sorted in nonincreasing order.

Let `M=max(m,n)` and `epsilon` be `FLT_EPSILON`, `DBL_EPSILON`, or the constant
F31 quantum `2^-31` for both integer working types. The template sets `tau=8*M*epsilon`. A column pair
is converged when the absolute normalized inner product is at most `tau`.
A nonempty problem with `tau >= 1` returns `LINEARS_ERROR_NO_CONVERGENCE`, because
that working precision cannot distinguish the rank cutoff from the largest
column. This is a numerical precision condition, not a configured dimension cap.
At each sweep/check boundary, columns with norm at most `tau` times the largest
current column norm are treated as numerical zero. This is intentional rank
truncation, not a claim that the mathematical singular value was exactly zero.
A conservative exact-arithmetic bound for all such discarded contributions is
`r*tau*||A||_F`: each column is discarded at most once, its norm is at most
`tau*||A||_F`, and the triangle inequality bounds the sum. Rounding of rotations
and output quantization contribute additional error. A small retained singular
value has no separate relative-error guarantee. In the integer variants, a
required rotation whose sine rounds to zero, or whose working columns remain
unchanged after rounding, returns `NO_CONVERGENCE` immediately. A large sweep
budget does not turn a quantized zero step into progress.

Input, outputs, and workspace are separate storage. Workspace and matrix
storage must satisfy the corresponding scalar alignment. Validation, arithmetic,
export-range, and convergence failures leave output data and descriptors
unchanged; workspace contents are unspecified after a call.

Fixed-point tests compare with the *decoded quantized input*, not an unquantized
fixture or a rescaled reference. Successful random Q fixtures deliberately
change their actual values so that their Frobenius norm is at most one half;
this bounds their largest singular value while leaving room for rounding.
Separate cases retain out-of-range singular values and require atomic failure.

One output quantum is always `2^-15` or `2^-31`. Ordinary nearest rounding has
at most half-quantum error; the unavailable positive endpoint has at most
one-quantum error. The test budget distinguishes these cells, F31 arithmetic
roundoff, and intentional rank truncation. Raw minimum quantum, signed extrema,
small singular values rounding to zero, and finite-precision stagnation are
covered explicitly. Checks compare singular values, reconstruction, and
orthogonality, not signs or a particular basis for repeated singular values.
