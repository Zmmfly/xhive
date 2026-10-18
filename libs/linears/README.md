# linears

Allocation-free dense linear algebra in C99, with C++-compatible public headers.
The four signal API suffixes are `f32`, `f64`, `q15`, and `q31`. Production code
has no Eigen dependency, heap allocation, `alloca`, variable-length arrays,
dimension-dependent stack arrays, recursion, or mutable global state. The caller
owns every matrix buffer and workspace.

## Fixed binary points, not block floating point

`linears_matrix_t` contains exactly `data`, `rows`, and `cols`. Storage is dense,
contiguous and row-major. There is no stride, capacity field, exponent, or
per-value normalization metadata. The suffix selects the element type:

| Suffix | Element / scalar type | Real value and range |
| --- | --- | --- |
| `f32` | `float` | Native floating value |
| `f64` | `double` | Native floating value |
| `q15` | `linears_q15_t` = `int16_t` | `raw / 2^15`, `[-1, 1 - 2^-15]` |
| `q31` | `linears_q31_t` = `int32_t` | `raw / 2^31`, `[-1, 1 - 2^-31]` |

Every raw q15/q31 bit pattern is valid. These typedefs occupy two/four bytes;
they are not structures. The former block-exponent interface is removed,
including its exponent limits, validity/normalization routines and variable
fraction/exponent importer. Old code must remove the fourth matrix initializer
field and replace `.mantissa`/`.exponent` scalar objects with raw integers.

### Wide working values and coefficients

`linears_fixed_t` is `int64_t`, always interpreted as **raw / 2^31**, with exact
range `[-2^32, 2^32 - 2^-31]`. It is not a normalized mantissa and does not have a
variable binary point. The wider integer part supports factorization
intermediates, filter coefficients near -2, and control gains above one.

Only q15/q31 `scale` and `axpby` use wide coefficients in the basic signal API.
Their input and output matrices still contain raw int16/int32 signal values.

```c
/* Even in a q15 API, alpha is a 64-bit F31 coefficient, NOT a q15 raw code. */
linears_fixed_t unity = LINEARS_FIXED_ONE;       /* 2147483648, exactly 1 */
linears_fixed_t half = LINEARS_FIXED_ONE / 2;    /* exactly 0.5 */
linears_fixed_t gain = 2 * LINEARS_FIXED_ONE;    /* exactly 2 */
```

Passing q15 raw `16384` as an `alpha` argument does **not** mean one half; it
means `16384 / 2^31`. Likewise, the q31 signal code `INT32_MAX` is not mathematical
one. Use `LINEARS_FIXED_ONE` for an exact unit coefficient or working identity.
Other libraries use this same wide F31 type only where coefficients or internal
state need its range; ordinary signals remain narrow q15/q31.

## Rounding and representable outputs

Fixed-point execution is entirely integer-only, including division and square
root. It never converts q values to float/double or requires production
`__int128` support.

- Multiplication/division/square root round to nearest, with halfway cases away
  from zero. A successful wide F31 primitive has absolute rounding error at most
  `2^-32`. This is an **absolute** bound, not constant relative precision near
  zero. Add/sub/neg/abs are exact if their result fits.
- Working int64 overflow returns `LINEARS_ERROR_RANGE` without changing output.
  In particular `abs(INT64_MIN)` and `neg(INT64_MIN)` fail. Division by zero and
  negative square root return `LINEARS_ERROR_DOMAIN`.
- Q export first checks the supplied mathematical value against **[-1,+1]**,
  before quantization. Values genuinely outside that interval return RANGE.
- Within that interval, a positive rounded code of `2^15`/`2^31` maps to
  `INT16_MAX`/`INT32_MAX`. Consequently exact +1 has one output-LSB error; -1 is
  represented exactly. The upper q15 rounding cell also maps to INT16_MAX, so
  accepting +1 does not create a gap immediately below it. This cell has at most
  one-LSB error; other cells have at most half-LSB error.
- A Q identity therefore has diagonal `1 - 2^-15` or `1 - 2^-31`, **not exact
  real one**. `inverse([0.5])` would produce 2 and must return RANGE, not silently
  saturate. A q31 norm of raw `[INT32_MIN, 1]` is strictly above one and fails even
  when a floating reference rounds that tiny excess away.

The exact raw accumulator in basic products is range-checked before its output
rounding: even a result one F62 accumulator bit above +1 is rejected. Complex
factorizations operate at F31 precision; their output range check applies to
that computed approximation, not to an uncomputed exact real solution.
Conditioning, cancellation and iteration count affect final accuracy.

## Shared arithmetic and exact multiply-accumulate

`linears_numeric.h` provides the reusable integer core:

- `linears_fixed_from_q15(raw)` is an inline multiplication by 65536;
  `linears_fixed_from_q31(raw)` is inline sign extension. Neither needs a validity
  check or failure result. `linears_fixed_from_int(integer, &out)` converts an
  integer within `[-2^32,2^32-1]` to F31.
- `linears_fixed_add/sub/mul/div/sqrt/neg/abs/compare` operate on wide F31 values.
  `linears_fixed_muldiv(a,b,divisor,&out)` computes the exact raw product divided
  by the raw divisor with one final rounding; it does not overflow a rounded
  intermediate F31 product before dividing it down.
- `linears_accum_t {low, high}` is an explicit two-uint64-limb **signed
  two's-complement integer divided by 2^62**. Initialize it to `{0,0}`.
  `linears_fixed_mac/msu` add/subtract exact F31 raw products. A signed-128-bit
  accumulation overflow returns RANGE and leaves the accumulator unchanged.
- `linears_accum_to_fixed/q15/q31` perform one final rounding. The narrow
  conversions check the exact F62 value directly instead of first rounding it
  into F31, which could conceal an out-of-range result.
- `linears_accum_mean_q15/q31(sum,count,&out)` divide an exact F62 sum by a
  positive uint64 count and round only the output. The count is never converted
  to F31. The exact range check compares `abs(sum)` to `count * 2^62` in limbs;
  even `UINT64_MAX` is a supported divisor. Zero count is a domain error.
- `linears_fixed_to_q15/q31` and `linears_fixed_export_q15/q31` convert wide
  values to signals, with no exponent parameter. Array export is failure-atomic
  and requires input/output storage to be disjoint.

The common small-operand multiplication path uses one 32-by-32-to-64 product.
The wider path uses four 32-bit partial products and carries. Division uses an
ordinary uint64 fast path when its numerator fits, otherwise bounded restoring
long division with an explicit remainder carry bit. Square root uses integer
restoring digits, including for raw working value 1; there is no floating seed.

## Basic matrix operations and hot paths

All operations below have the four signal suffixes. Matrix output dimensions
are supplied by the caller and are never changed.

| API stem | Operation | Scratch |
| --- | --- | --- |
| `linears_add`, `linears_sub` | Elementwise addition/subtraction | One output block |
| `linears_hadamard` | Elementwise product | One output block |
| `linears_mul`, `linears_matvec` | Matrix product / matrix-column-vector product | One output block |
| `linears_transpose` | Rectangular transpose | One output block |
| `linears_axpby`, `linears_scale` | Weighted sum / scalar multiplication | One output block |
| `linears_copy` | Exact copy with `memmove` semantics | None |
| `linears_zero`, `linears_identity` | Zero or square identity fill | None |
| `linears_dot`, `linears_norm`, `linears_trace` | Dot/Frobenius inner product, norm, trace | None |

The Q product/dot inner loops load raw integers, compute exact raw products,
and accumulate them without scalar imports, normalization, or per-product
rounding. q15 uses int64 when the term count is at most `INT64_MAX / 2^30`, and
limbs otherwise. q31 uses exact limbs throughout. On the supported 32/64-bit
`size_t` targets, at most `SIZE_MAX` terms of magnitude at most `2^62` sum to a
magnitude below `2^126`, so the raw q31 accumulator cannot overflow signed 128
bits. Each output is range-checked and quantized once into narrow typed scratch;
all staged outputs are then committed together.

Q norms accumulate exact squares and compare the sum with the exact unit-square
bound before an integer square root. Floating norms retain scaled sums of
squares, so representable norms such as `norm([1e300,1e300])` avoid intermediate
square overflow. Floating input validation, arithmetic and workspace accesses
remain native float/double respectively. `-ffast-math` is unsupported because it
invalidates finite-input checks. Floating contraction follows compiler settings.

### Internal working matrices

`linears_work.h` supplies only the eight operations actually needed to share
library-internal F31 intermediates: `add/sub/mul/transpose/axpby/identity_fixed`
and `solve/cholesky_fixed`. This is not a fifth signal format. Buffers are
int64 F31 arrays, and output values above one are retained if they fit the wide
working range. `identity_fixed` stores exact `LINEARS_FIXED_ONE`; Kalman's
Joseph identity must not substitute a Q signal identity and attenuate covariance.

The six basic working operations reuse basic matrix validation and typed byte
scratch. Solve and Cholesky reuse the factorization implementation and
`linears_fixed_workspace_t`, whose capacity counts F31 elements. Full int64
working products do not have the raw-Q headroom bound, so their signed-128-bit
accumulator is checked on each addition. If an intermediate sum overflows,
RANGE is returned even if later terms could cancel it. There is no arbitrary-
precision accumulator or hidden exponent fallback.

## Static workspace, aliasing and failure

The checked size macros accept nonnegative, side-effect-free arguments.
`SIZE_MAX` is an invalid-size sentinel, not an allocation request. Literal-zero
and runtime-zero dimension combinations remain valid constant/ordinary
expressions without division by zero.

```c
#include "linears.h"

#define MAX_ROWS 8
#define MAX_COLS 8
#define SCRATCH_BYTES LINEARS_BASIC_WORKSPACE_BYTES(MAX_ROWS, MAX_COLS)

struct example_storage {
    int32_t a[MAX_ROWS * MAX_COLS];
    int32_t b[MAX_ROWS * MAX_COLS];
    int32_t result[MAX_ROWS * MAX_COLS];
    linears_workspace_unit_t scratch[LINEARS_WORKSPACE_UNITS(SCRATCH_BYTES)];
};

linears_status_t example_add(struct example_storage *storage) {
    linears_matrix_t a = {storage->a, 8, 8};
    linears_matrix_t b = {storage->b, 8, 8};
    linears_matrix_t out = {storage->result, 8, 8};
    return linears_add_q31(&a, &b, &out, storage->scratch,
                           sizeof(storage->scratch));
}
```

Basic scratch remains one eight-byte typed unit per output on the supported
host/ARM targets. A local fixed-size 16-byte accumulator does not double the
entire workspace. Declare an actual `linears_workspace_unit_t[]`; an aligned
`char[]` cast to that type does not satisfy C effective-type rules. Each unit is
accessed through its active float/double/fixed/q15/q31 member.

Input/output data can overlap partially or completely, including in-place
products and rectangular transpose through distinct descriptors. Workspace may
not overlap any data or descriptor. Descriptors can be identical but may not
partially overlap one another or occupy data storage. On every basic error,
output data, scalar results and descriptors are unchanged; scratch contents are
unspecified. Functions are synchronous and do not retain storage pointers.
Independent calls are reentrant; callers serialize overlapping calls.

The caller guarantees actual storage capacity and lifetime. The library checks
size/address arithmetic and specified overlaps, not the allocation size behind
an arbitrary pointer. A zero row or column permits NULL data; shape rules still
apply. An `m x 0` times `0 x n` product produces an `m x n` zero matrix. Empty
reductions produce zero. Identity and trace require square matrices, including
valid `0 x 0` matrices. Factorization alias/workspace rules are stricter: see
[linears decompositions](DECOMPOSITIONS.md).

## Algorithm references and validation

Implementations are independent integer/C recurrences, not copied Eigen code.

- Netlib BLAS [DGEMM](https://www.netlib.org/blas/dgemm.f),
  [DGEMV](https://www.netlib.org/blas/dgemv.f), and
  [DDOT](https://www.netlib.org/blas/ddot.f): dense product definitions.
- Netlib LAPACK [DLASSQ](https://www.netlib.org/lapack/double/dlassq.f): scaled
  sum-of-squares recurrence retained by floating norms.
- Donald E. Knuth, *The Art of Computer Programming*, volume 2, third edition,
  section 4.3.1: multiple-precision arithmetic, radix-limb multiplication and
  division. The implementation uses bounded radix-2 restoring division, not a
  floating estimate or a compiler 128-bit integer extension.
- Henry S. Warren, Jr., *Hacker's Delight*, second edition, section 11-1,
  “Integer Square Root”: integer restoring square root. Nearest rounding uses
  the exact integer remainder comparison with `floor_root + 1`.

GoogleTest and Eigen 3.4.0 are host-test dependencies only. Tests compare decoded
**already quantized** Q inputs with Eigen, and use independent integer/rational
oracles for raw products, ties, endpoint range checks, carries and overflow.
Test-only `__int128` is used on GCC/Clang hosts; production code never uses it.
Success fixtures reserve explicit Q output headroom. Separate full-range tests
check required RANGE failures rather than silently dropping coverage or relaxing
tolerances. Floating fixture values/references are retained from the prior API.

From the configured SDK root:

```sh
xmake test -g xhive -v 'linears_test_basic_*/*'
xmake test -g xhive -v 'linears_test_numeric_contract/numeric_contract'
xmake test -g xhive -v 'linears_test_audit_fixed_numeric/audit_fixed_numeric'
```

See the central worklog for commands and measurements actually run. Host timing,
ARM object/disassembly checks, and physical board execution are different
observations; host nanoseconds are not claimed as MCU cycle counts.
