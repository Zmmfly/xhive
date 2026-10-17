/**
 * @file linears_decomp.h
 * @brief Bounded, allocation-free dense matrix factorizations.
 *
 * All matrices are contiguous row-major blocks. Floating variants perform
 * arithmetic in their named type and ignore input exponents; successful
 * floating outputs have exponent zero. Fixed variants use the integer-only
 * linears_fixed_t working arithmetic and round each output matrix to its own
 * automatically selected block exponent. This is 31-significant-bit working
 * arithmetic, not an exact wide dot-product accumulator.
 *
 * Every data pointer must have the natural alignment of its element type.
 * Workspaces hold elements of their declared type, not bytes, and need that
 * type's natural alignment. Matrix data, descriptors, used workspace, and
 * permutation/block metadata must be mutually disjoint. Read-only input
 * matrices may share data with each other. A NULL data pointer is permitted
 * only for a zero-element matrix. A NULL workspace pointer is permitted only
 * when the required element count is zero. Capacities describe actual caller
 * allocations. Unused workspace capacity is not accessed.
 *
 * These routines neither allocate nor retain state. Calls with disjoint
 * storage are reentrant; callers serialize shared storage. They are blocking,
 * CPU-only routines with dimension-bounded loops and no hardware effects.
 * Matrix outputs, including exponents, remain unchanged on every failure.
 * Workspace contents are unspecified after any call. Permutation/block
 * metadata may be partially written on numerical failure and is valid only
 * after LINEARS_OK. Invalid-argument failures do not write metadata.
 *
 * Common failures are ARGUMENT (NULL/misaligned data or metadata), DIMENSION
 * (wrong shapes), SIZE (size/address overflow),
 * WORKSPACE (insufficient/misaligned workspace or metadata capacity), ALIAS
 * (prohibited overlap), and RANGE (nonfinite input/intermediate, exponent or
 * output range). No conditioning estimate or forward-error guarantee is
 * implied by success. Zero pivots are tested at working precision.
 */
#ifndef LINEARS_DECOMP_H
#define LINEARS_DECOMP_H

#include "linears.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Caller-owned f32 factorization scratch; capacity counts float elements.
 */
typedef struct {
    float *data;     /**< Naturally aligned scratch, NULL only for a zero requirement. */
    size_t capacity; /**< Number of available float elements, not bytes. */
} linears_f32_workspace_t;

/**
 * @brief Caller-owned f64 factorization scratch; capacity counts double elements.
 */
typedef struct {
    double *data;    /**< Naturally aligned scratch, NULL only for a zero requirement. */
    size_t capacity; /**< Number of available double elements, not bytes. */
} linears_f64_workspace_t;

/**
 * @brief Caller-owned q15 scratch containing normalized integer working scalars.
 */
typedef struct {
    linears_q31_t *data; /**< Canonical working scalars (linears_fixed_t), not raw int16_t. */
    size_t capacity;      /**< Number of available linears_fixed_t elements. */
} linears_q15_workspace_t;

/**
 * @brief Caller-owned q31 scratch containing normalized integer working scalars.
 */
typedef struct {
    linears_q31_t *data; /**< Canonical working scalars (linears_fixed_t), not raw int32_t. */
    size_t capacity;      /**< Number of available linears_fixed_t elements. */
} linears_q31_workspace_t;

/**
 * @brief Smaller dimension; arguments must be nonnegative and side-effect-free.
 */
#define LINEARS_DECOMP_MIN(m, n) ((size_t)(m) < (size_t)(n) ? (size_t)(m) : (size_t)(n))
/**
 * @brief LU scratch elements for a square order n; SIZE_MAX signals overflow.
 */
#define LINEARS_LU_WORKSPACE(n) LINEARS_SIZE_MUL((n), (n))
/**
 * @brief Solve scratch elements for n equations and rhs columns; SIZE_MAX on overflow.
 */
#define LINEARS_SOLVE_WORKSPACE(n, rhs) \
    LINEARS_SIZE_ADD(LINEARS_SIZE_MUL((n), (n)), LINEARS_SIZE_MUL((n), (rhs)))
/**
 * @brief Inverse scratch elements for square order n; SIZE_MAX on overflow.
 */
#define LINEARS_INVERSE_WORKSPACE(n) LINEARS_SOLVE_WORKSPACE((n), (n))
/**
 * @brief Determinant scratch elements for square order n; SIZE_MAX on overflow.
 */
#define LINEARS_DETERMINANT_WORKSPACE(n) LINEARS_LU_WORKSPACE(n)
/**
 * @brief Cholesky scratch elements for square order n; SIZE_MAX on overflow.
 */
#define LINEARS_CHOLESKY_WORKSPACE(n) LINEARS_LU_WORKSPACE(n)
/**
 * @brief Full Householder QR scratch elements: m*n + m*m + m; SIZE_MAX on overflow.
 */
#define LINEARS_QR_WORKSPACE(m, n) \
    LINEARS_SIZE_ADD(LINEARS_SIZE_ADD(LINEARS_SIZE_MUL((m), (n)), \
                                     LINEARS_SIZE_MUL((m), (m))), (m))
/**
 * @brief Symmetric pivoted LDLT scratch elements for order n; SIZE_MAX on overflow.
 */
#define LINEARS_LDLT_WORKSPACE(n) LINEARS_LU_WORKSPACE(n)
/**
 * @brief Thin SVD scratch elements: m*n + min(m,n)^2 + min(m,n); SIZE_MAX on overflow.
 */
#define LINEARS_SVD_WORKSPACE(m, n) \
    LINEARS_SIZE_ADD(LINEARS_SIZE_ADD(LINEARS_SIZE_MUL((m), (n)), \
        LINEARS_SIZE_MUL(LINEARS_DECOMP_MIN((m), (n)), LINEARS_DECOMP_MIN((m), (n)))), \
        LINEARS_DECOMP_MIN((m), (n)))
/**
 * @brief Factor a square matrix with partial row pivoting: P*A = L*U.
 *
 * @param[in] a Non-NULL n-by-n input descriptor.
 * @param[out] l Non-NULL n-by-n unit-lower-triangular output descriptor.
 * @param[out] u Non-NULL n-by-n upper-triangular output descriptor.
 * @param[out] permutation Array of n size_t values; NULL only for n=0.
 *                        Row i of P*A is original row permutation[i].
 * @param[in] permutation_capacity Available permutation elements, at least n.
 * @param[out] parity Non-NULL integer destination, +1 for even and -1 for odd
 *                    row-swap count; written only on success.
 * @param[in,out] workspace Typed scratch of at least LINEARS_LU_WORKSPACE(n) elements.
 * @return linears_status_t Common status or SINGULAR for a zero pivot.
 * @note n=0 succeeds with parity +1. L and U have independent block exponents;
 *       fixed block quantization can discard tiny nonzero U pivots. Failure,
 *       alignment, alias and concurrency rules are in the file contract.
 */
linears_status_t linears_lu_f32(const linears_matrix_t *a, linears_matrix_t *l,
    linears_matrix_t *u, size_t *permutation, size_t permutation_capacity,
    int *parity, linears_f32_workspace_t workspace);
/**
 * @copydoc linears_lu_f32
 */
linears_status_t linears_lu_f64(const linears_matrix_t *a, linears_matrix_t *l,
    linears_matrix_t *u, size_t *permutation, size_t permutation_capacity,
    int *parity, linears_f64_workspace_t workspace);
/**
 * @copydoc linears_lu_f32
 */
linears_status_t linears_lu_q15(const linears_matrix_t *a, linears_matrix_t *l,
    linears_matrix_t *u, size_t *permutation, size_t permutation_capacity,
    int *parity, linears_q15_workspace_t workspace);
/**
 * @copydoc linears_lu_f32
 */
linears_status_t linears_lu_q31(const linears_matrix_t *a, linears_matrix_t *l,
    linears_matrix_t *u, size_t *permutation, size_t permutation_capacity,
    int *parity, linears_q31_workspace_t workspace);

/**
 * @brief Solve A*X=B by partial-pivot LU without forming an inverse.
 *
 * @param[in] a Non-NULL n-by-n coefficient matrix.
 * @param[in] b Non-NULL n-by-rhs right-hand-side matrix.
 * @param[out] x Non-NULL n-by-rhs solution matrix, unchanged on failure.
 * @param[in,out] workspace At least LINEARS_SOLVE_WORKSPACE(n,rhs) typed elements.
 * @return linears_status_t Common status or SINGULAR for a zero pivot.
 * @note Empty n or zero rhs is supported; A is still factored when n is nonzero.
 *       A and B may have different fixed-point block exponents. File-level
 *       alignment, nonaliasing, ownership and concurrency rules apply.
 */
linears_status_t linears_solve_f32(const linears_matrix_t *a, const linears_matrix_t *b,
    linears_matrix_t *x, linears_f32_workspace_t workspace);
/**
 * @copydoc linears_solve_f32
 */
linears_status_t linears_solve_f64(const linears_matrix_t *a, const linears_matrix_t *b,
    linears_matrix_t *x, linears_f64_workspace_t workspace);
/**
 * @copydoc linears_solve_f32
 */
linears_status_t linears_solve_q15(const linears_matrix_t *a, const linears_matrix_t *b,
    linears_matrix_t *x, linears_q15_workspace_t workspace);
/**
 * @copydoc linears_solve_f32
 */
linears_status_t linears_solve_q31(const linears_matrix_t *a, const linears_matrix_t *b,
    linears_matrix_t *x, linears_q31_workspace_t workspace);

/**
 * @brief Invert a square matrix through partial-pivot LU solves.
 *
 * @param[in] a Non-NULL n-by-n input matrix.
 * @param[out] inverse Non-NULL n-by-n output matrix, unchanged on failure.
 * @param[in,out] workspace At least LINEARS_INVERSE_WORKSPACE(n) typed elements.
 * @return linears_status_t Common status or SINGULAR for a zero pivot.
 * @note n=0 succeeds. File-level alignment, alias and concurrency rules apply.
 */
linears_status_t linears_inverse_f32(const linears_matrix_t *a, linears_matrix_t *inverse,
    linears_f32_workspace_t workspace);
/**
 * @copydoc linears_inverse_f32
 */
linears_status_t linears_inverse_f64(const linears_matrix_t *a, linears_matrix_t *inverse,
    linears_f64_workspace_t workspace);
/**
 * @copydoc linears_inverse_f32
 */
linears_status_t linears_inverse_q15(const linears_matrix_t *a, linears_matrix_t *inverse,
    linears_q15_workspace_t workspace);
/**
 * @copydoc linears_inverse_f32
 */
linears_status_t linears_inverse_q31(const linears_matrix_t *a, linears_matrix_t *inverse,
    linears_q31_workspace_t workspace);

/**
 * @brief Compute the determinant from partial-pivot LU and swap parity.
 *
 * @param[in] a Non-NULL n-by-n input matrix.
 * @param[out] determinant Non-NULL 1-by-1 output matrix, unchanged on failure.
 * @param[in,out] workspace At least LINEARS_DETERMINANT_WORKSPACE(n) typed elements.
 * @return linears_status_t Common status; singular input succeeds with zero.
 * @note The determinant of the 0-by-0 matrix is one. Floating product overflow
 *       returns RANGE. Fixed output has its own exponent, subject to the shared
 *       exponent limits. File-level alias/alignment/concurrency rules apply.
 */
linears_status_t linears_determinant_f32(const linears_matrix_t *a,
    linears_matrix_t *determinant, linears_f32_workspace_t workspace);
/**
 * @copydoc linears_determinant_f32
 */
linears_status_t linears_determinant_f64(const linears_matrix_t *a,
    linears_matrix_t *determinant, linears_f64_workspace_t workspace);
/**
 * @copydoc linears_determinant_f32
 */
linears_status_t linears_determinant_q15(const linears_matrix_t *a,
    linears_matrix_t *determinant, linears_q15_workspace_t workspace);
/**
 * @copydoc linears_determinant_f32
 */
linears_status_t linears_determinant_q31(const linears_matrix_t *a,
    linears_matrix_t *determinant, linears_q31_workspace_t workspace);

/**
 * @brief Factor an exactly symmetric positive-definite matrix as A=L*L^T.
 *
 * @param[in] a Non-NULL n-by-n exactly symmetric input matrix.
 * @param[out] l Non-NULL n-by-n lower-triangular output, unchanged on failure.
 * @param[in,out] workspace At least LINEARS_CHOLESKY_WORKSPACE(n) typed elements.
 * @return linears_status_t Common status, DIMENSION for nonsymmetry, or
 *         NOT_POSITIVE_DEFINITE for a nonpositive computed diagonal pivot.
 * @note n=0 succeeds. Both input triangles are checked. Upper output entries
 *       are zero. File-level alignment, alias and concurrency rules apply.
 */
linears_status_t linears_cholesky_f32(const linears_matrix_t *a, linears_matrix_t *l,
    linears_f32_workspace_t workspace);
/**
 * @copydoc linears_cholesky_f32
 */
linears_status_t linears_cholesky_f64(const linears_matrix_t *a, linears_matrix_t *l,
    linears_f64_workspace_t workspace);
/**
 * @copydoc linears_cholesky_f32
 */
linears_status_t linears_cholesky_q15(const linears_matrix_t *a, linears_matrix_t *l,
    linears_q15_workspace_t workspace);
/**
 * @copydoc linears_cholesky_f32
 */
linears_status_t linears_cholesky_q31(const linears_matrix_t *a, linears_matrix_t *l,
    linears_q31_workspace_t workspace);

/**
 * @brief Compute full Householder QR of a tall, square or wide matrix: A=Q*R.
 *
 * @param[in] a Non-NULL m-by-n input matrix.
 * @param[out] q Non-NULL m-by-m orthogonal output, unchanged on failure.
 * @param[out] r Non-NULL m-by-n upper-trapezoidal output, unchanged on failure.
 * @param[in,out] workspace At least LINEARS_QR_WORKSPACE(m,n) typed elements.
 * @return linears_status_t Common status. Rank deficiency is not an error.
 * @note Zero columns need no reflector. For n=0, Q is the m-by-m identity;
 *       for m=0 both outputs are empty. QR does not pivot columns or estimate
 *       rank. File-level alignment, alias and concurrency rules apply.
 */
linears_status_t linears_qr_f32(const linears_matrix_t *a, linears_matrix_t *q,
    linears_matrix_t *r, linears_f32_workspace_t workspace);
/**
 * @copydoc linears_qr_f32
 */
linears_status_t linears_qr_f64(const linears_matrix_t *a, linears_matrix_t *q,
    linears_matrix_t *r, linears_f64_workspace_t workspace);
/**
 * @copydoc linears_qr_f32
 */
linears_status_t linears_qr_q15(const linears_matrix_t *a, linears_matrix_t *q,
    linears_matrix_t *r, linears_q15_workspace_t workspace);
/**
 * @copydoc linears_qr_f32
 */
linears_status_t linears_qr_q31(const linears_matrix_t *a, linears_matrix_t *q,
    linears_matrix_t *r, linears_q31_workspace_t workspace);

/**
 * @brief Factor a symmetric possibly-indefinite matrix: P*A*P^T=L*D*L^T.
 *
 * @param[in] a Non-NULL exactly symmetric n-by-n matrix; both triangles checked.
 * @param[out] l Non-NULL n-by-n unit-lower-triangular matrix.
 * @param[out] d Non-NULL n-by-n block-diagonal matrix with 1-by-1 or 2-by-2 blocks.
 * @param[out] permutation Array of n original row indices; NULL only if n=0.
 * @param[in] permutation_capacity Available size_t permutation entries, at least n.
 * @param[out] blocks Array of n uint8_t entries, NULL only if n=0: 1 starts a
 *                    1-by-1 block, 2 starts a 2-by-2 block, 0 continues it.
 * @param[in] blocks_capacity Available block entries, at least n.
 * @param[in,out] workspace At least LINEARS_LDLT_WORKSPACE(n) typed elements.
 * @return linears_status_t Common status, DIMENSION for nonsymmetry, or SINGULAR
 *         for a zero working pivot.
 * @note Complete symmetric pivoting supports indefinite zero-diagonal inputs.
 *       Fixed output block quantization can discard tiny nonzero D pivots;
 *       success does not assert that quantized factors retain the input rank.
 *       n=0 succeeds. L/D remain unchanged on failure; metadata can be partial
 *       after a numerical failure. File-level alignment/alias rules apply.
 */
linears_status_t linears_ldlt_f32(const linears_matrix_t *a, linears_matrix_t *l,
    linears_matrix_t *d, size_t *permutation, size_t permutation_capacity,
    uint8_t *blocks, size_t blocks_capacity, linears_f32_workspace_t workspace);
/**
 * @copydoc linears_ldlt_f32
 */
linears_status_t linears_ldlt_f64(const linears_matrix_t *a, linears_matrix_t *l,
    linears_matrix_t *d, size_t *permutation, size_t permutation_capacity,
    uint8_t *blocks, size_t blocks_capacity, linears_f64_workspace_t workspace);
/**
 * @copydoc linears_ldlt_f32
 */
linears_status_t linears_ldlt_q15(const linears_matrix_t *a, linears_matrix_t *l,
    linears_matrix_t *d, size_t *permutation, size_t permutation_capacity,
    uint8_t *blocks, size_t blocks_capacity, linears_q15_workspace_t workspace);
/**
 * @copydoc linears_ldlt_f32
 */
linears_status_t linears_ldlt_q31(const linears_matrix_t *a, linears_matrix_t *l,
    linears_matrix_t *d, size_t *permutation, size_t permutation_capacity,
    uint8_t *blocks, size_t blocks_capacity, linears_q31_workspace_t workspace);

/**
 * @brief Compute thin SVD A=U*diag(S)*V^T by bounded one-sided Jacobi sweeps.
 *
 * @param[in] a Non-NULL m-by-n matrix.
 * @param[out] u Non-NULL m-by-r matrix, r=min(m,n), with orthonormal columns.
 * @param[out] singular_values Non-NULL r-by-1 nonnegative nonincreasing values.
 * @param[out] v Non-NULL n-by-r matrix with orthonormal columns, not V transposed.
 * @param[in] max_sweeps Maximum number of cyclic sweeps. Zero only checks
 *                      whether the initial columns are converged. The caller
 *                      chooses the finite run-time budget; 64 is a typical
 *                      starting choice, not a convergence guarantee.
 * @param[in,out] workspace At least LINEARS_SVD_WORKSPACE(m,n) typed elements.
 * @return linears_status_t Common status or NO_CONVERGENCE when the budget or
 *         working precision cannot produce the required orthogonal factors.
 * @note Wide inputs use the transposed tall problem. Numerically zero singular
 *       columns are completed to an orthonormal basis; output block quantization
 *       can also round small singular values to zero. Empty r succeeds. The
 *       algorithmic rank/convergence thresholds are documented in decomp_svd.md.
 *       File-level alignment, alias and failure-atomic output rules apply.
 */
linears_status_t linears_svd_f32(const linears_matrix_t *a, linears_matrix_t *u,
    linears_matrix_t *singular_values, linears_matrix_t *v, size_t max_sweeps,
    linears_f32_workspace_t workspace);
/**
 * @copydoc linears_svd_f32
 */
linears_status_t linears_svd_f64(const linears_matrix_t *a, linears_matrix_t *u,
    linears_matrix_t *singular_values, linears_matrix_t *v, size_t max_sweeps,
    linears_f64_workspace_t workspace);
/**
 * @copydoc linears_svd_f32
 */
linears_status_t linears_svd_q15(const linears_matrix_t *a, linears_matrix_t *u,
    linears_matrix_t *singular_values, linears_matrix_t *v, size_t max_sweeps,
    linears_q15_workspace_t workspace);
/**
 * @copydoc linears_svd_f32
 */
linears_status_t linears_svd_q31(const linears_matrix_t *a, linears_matrix_t *u,
    linears_matrix_t *singular_values, linears_matrix_t *v, size_t max_sweeps,
    linears_q31_workspace_t workspace);

#ifdef __cplusplus
}
#endif
#endif
