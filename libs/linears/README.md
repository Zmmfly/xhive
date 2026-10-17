# linears

Allocation-free dense linear algebra in C99, with C++-compatible public headers.
The four API suffixes are `f32`, `f64`, `q15`, and `q31`. Production code does not
use Eigen, a heap allocator, `alloca`, variable-length arrays, dimension-dependent
stack arrays, recursive algorithms, or mutable global state. The caller owns all
matrix storage and any workspace.

## Data and numeric contracts

`linears_matrix_t` has exactly four fields: `data`, `rows`, `cols`, and `exponent`.
Elements are contiguous, row-major, with no padding or stride between rows. The
function suffix selects the actual element type:

| Suffix | Stored element | Represented value |
| --- | --- | --- |
| `f32` | `float` | Native floating value; `exponent` ignored |
| `f64` | `double` | Native floating value; `exponent` ignored |
| `q15` | `int16_t` | `mantissa * 2^(exponent - 15)` |
| `q31` | `int32_t` | `mantissa * 2^(exponent - 31)` |

The q formats use one exponent for the entire matrix. Scalar coefficients and
scalar results use `linears_q15_t` or `linears_q31_t`, with `mantissa` and
`exponent` fields. Exponents must be in `[-16384, 16384]`. An input whose
*normalized* exponent falls outside this interval is also rejected: for example,
a q31 mantissa of `1` at exponent `-16384` is below the working format's range.

Fixed-point execution is integer-only, including division and square root. It
never converts q data to floating point and does not require `__int128`.
`linears_numeric.h` exposes the shared working arithmetic used by decompositions,
filters and field control:

- `linears_fixed_t` is a normalized q31 scalar. A nonzero mantissa has magnitude
  in `[2^30, 2^31)`; zero is exactly `{0, 0}`. Import raw storage using
  `linears_fixed_from_q15`, `linears_fixed_from_q31`, or `linears_fixed_from_i64`.
- There are **31 significant magnitude bits**, not a 64-bit accumulator's
  precision. Multiplication uses an exact signed 64-bit product; division uses
  unsigned 64-bit numerator/remainder arithmetic. Square root uses a bounded
  integer restoring recurrence and an integer nearest-rounding comparison.
- Scalar results round to nearest, with exact half cases rounded away from zero.
  The rounding error of one successful primitive is at most half an output
  mantissa unit; for a nonzero normalized result its relative rounding bound is
  `2^-31`. Addition aligns exactly when alignment can affect rounding; it does
  not first truncate both operands to a common 31-bit mantissa.
- Arithmetic never wraps or saturates. A nonrepresentable normalized result
  returns `LINEARS_ERROR_RANGE`. Division by zero and negative square root return
  `LINEARS_ERROR_DOMAIN`. Scalar destinations are unchanged on failure.
- Working inputs to arithmetic must be canonical. Raw q31 storage is not
  automatically a canonical working value, even though both typedefs have the
  same C layout. `linears_fixed_compare` assumes canonical inputs.

Computed q matrix outputs are automatically block-normalized. Positive q15
rounding that would overflow the mantissa increases the entire block's exponent
by one. The quantization error per element at the chosen output exponent `e`
is at most `2^(e - bits - 1)`, in addition to preceding working-arithmetic errors.
Very small elements can become zero when they share an exponent with large
ones. `linears_copy_*` is deliberately different: it preserves the original raw
mantissas and exponent exactly, avoiding precision changes during a copy.

Products, sums, and reductions round **each** working operation. They are not
exact dot products and do not promise a dimension-independent final relative
error. Cancellation can make final relative errors large; conditioning and
iteration count also affect decompositions. Tests compare against Eigen after
quantizing and decoding the actual q inputs, not against unquantized inputs.

## Basic operations

All rows in this table have all four suffixes. Matrix output dimensions are
provided by the caller and are never resized.

| API stem | Operation | Scratch |
| --- | --- | --- |
| `linears_add`, `linears_sub` | Elementwise addition/subtraction | One output block |
| `linears_hadamard` | Elementwise multiplication | One output block |
| `linears_mul` | Matrix multiplication | One output block |
| `linears_matvec` | Matrix times a `k x 1` column vector | One output block |
| `linears_transpose` | Rectangular transpose | One output block |
| `linears_axpby` | `alpha*A + beta*B` | One output block |
| `linears_scale` | `alpha*A` | One output block |
| `linears_copy` | Exact representation copy with `memmove` semantics | None |
| `linears_zero`, `linears_identity` | Fill zeros or a square identity | None |
| `linears_dot` | Vector dot product / matrix Frobenius inner product | None |
| `linears_norm` | Euclidean / Frobenius norm | None |
| `linears_trace` | Sum of a square matrix's diagonal | None |

Floating kernels use native `float` or `double` arithmetic throughout,
including input checks and workspace loads/stores. In particular f32 does not
convert each element through a software double container on Cortex-M4. Inputs
and coefficients must be finite. Floating exponent fields
are not changed, even by a copy or fill. Do not compile with options such as
`-ffast-math` that invalidate the finite-value checks.

Norms use scaled sums of squares rather than squaring the largest original
input. This lets, for example, `norm([1e300, 1e300])` succeed in f64. The fixed
norm first selects the largest absolute value, then accumulates squared ratios.
Overflow of the final norm is still an error.

## Static workspace and aliasing

Use the overflow-checked constant-expression macros with nonnegative arguments
that have no side effects. `SIZE_MAX` is an invalid-size sentinel, not an
allocation request. A zero row or column gives zero required bytes.

```c
#include "linears.h"

#define MAX_ROWS 8
#define MAX_COLS 8
#define SCRATCH_BYTES LINEARS_BASIC_WORKSPACE_BYTES(MAX_ROWS, MAX_COLS)

/* Caller-owned fixed-size objects; no dimension-dependent automatic arrays. */
struct example_storage {
    int32_t a[MAX_ROWS * MAX_COLS];
    int32_t b[MAX_ROWS * MAX_COLS];
    int32_t result[MAX_ROWS * MAX_COLS];
    int result_exponent;
    linears_workspace_unit_t scratch[LINEARS_WORKSPACE_UNITS(SCRATCH_BYTES)];
};

linears_status_t example_add(struct example_storage *storage) {
    linears_matrix_t a = {storage->a, 8, 8, 0};
    linears_matrix_t b = {storage->b, 8, 8, 0};
    linears_matrix_t out = {storage->result, 8, 8, 0};
    linears_status_t status = linears_add_q31(&a, &b, &out, storage->scratch,
                                              sizeof(storage->scratch));
    if (status == LINEARS_OK) storage->result_exponent = out.exponent;
    return status;
}
```

Workspace must be an **actual array of `linears_workspace_unit_t` objects**.
A suitably aligned `char[]` cast to that pointer type is not the same effective
type contract. Implementation accesses `workspace[i].f32`,
`workspace[i].floating`, or `workspace[i].fixed` directly; it never reinterprets a union array as a flat
`float[]` or `linears_fixed_t[]`. Decomposition APIs have their own typed
workspace structures and capacity macros; consult `linears_decomp.h`.

Basic functions stage computed outputs before committing them. Input/output
element ranges can overlap partially or completely, including in-place matrix
multiplication. Distinct rectangular-transpose descriptors can refer to the same
storage. Workspace cannot overlap any matrix data or descriptor. Descriptors may
be identical, but may not partially overlap one another or lie in element data.

On any basic-operation error, output elements, scalar results, and descriptors
remain unchanged; scratch contents are unspecified. All functions are
synchronous. Independent instances are reentrant. A caller must serialize calls
that access overlapping data or scratch. No storage pointer is retained.

The caller guarantees real object lifetimes and buffer capacities; the library
can check dimension/byte arithmetic and specified overlaps, not the size of an
arbitrary external allocation. Matrices with either dimension zero permit NULL
data. Shape rules still apply. An `m x 0` times `0 x n` product is an `m x n`
zero matrix, and empty dot/norm/trace reductions produce zero. Identity and trace
require square matrices, including the valid `0 x 0` case.

## Algorithms and references

These are independent implementations of published recurrences, not copied
Eigen code. Basic dense products use the row/column/inner-index definition of
GEMM, without hidden packing allocation or recursive blocking.

- Netlib reference BLAS: [DGEMM](https://www.netlib.org/blas/dgemm.f),
  [DGEMV](https://www.netlib.org/blas/dgemv.f), and
  [DDOT](https://www.netlib.org/blas/ddot.f).
- Netlib LAPACK [DLASSQ](https://www.netlib.org/lapack/double/dlassq.f): scaled
  sum-of-squares recurrence, the basis of the overflow-resistant floating norm.
- Henry S. Warren, Jr., *Hacker's Delight*, second edition (2013), ISBN
  978-0-321-84268-8, section 11-1, “Integer Square Root”: integer binary restoring
  square-root recurrence. `numeric.c` adds nearest rounding by comparing the
  integer remainder with `floor_root + 1`.
- [Decompositions](DECOMPOSITIONS.md) documents decomposition-specific algorithms,
  limits and references; `linears_decomp.h` is their public interface.

## Tests

Production dependencies do not include Eigen or GoogleTest. Host tests use
GoogleTest and Eigen 3.4.0. The basic test categories are separate files:
`numeric_contract.cc`, `basic_elementwise.cc`, `basic_products.cc`,
`basic_reductions.cc`, and `basic_contract.cc`; `audit_fixed_numeric.cc` is an
independently authored numeric audit. Shared `main.cc` contains only the normal
GoogleTest entry point.

From the SDK root, after its host-test configuration:

```sh
xmake test -g xhive -v 'linears_test_basic_*/*'
xmake test -g xhive -v 'linears_test_numeric_contract/numeric_contract'
xmake test -g xhive -v 'linears_test_audit_fixed_numeric/audit_fixed_numeric'
```

Host numerical comparisons, sanitizer runs, target compilation and physical
board execution are different evidence. Consult the repository worklog for the
commands actually run; a host test does not establish board-level validation.
