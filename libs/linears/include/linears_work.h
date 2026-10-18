/**
 * @file linears_work.h
 * @brief Shared library-internal fixed-F31 matrix intermediates.
 *
 * This is not another Q signal format. Every matrix element is linears_fixed_t
 * (int64_t raw / 2^31), with its full [-2^32,2^32-2^-31] range. It has no
 * exponent and is never normalized. These selected operations let filters and
 * control code retain intermediates and exact unit factors without narrowing
 * them to q15/q31 signal arrays between matrix operations.
 *
 * Basic operations use the same descriptor, alias, empty-shape and atomic-error
 * contracts as linears.h, and the same byte-counted typed basic scratch array.
 * Products retain a signed 128-bit F62 sum until one F31 rounding per output.
 * An overflowing intermediate 128-bit sum returns RANGE even if later terms
 * could cancel it. Identity uses exact LINEARS_FIXED_ONE, unlike Q signal
 * identity. These functions never apply the narrow Q positive-endpoint policy.
 * Solve/Cholesky instead use the element-counted factorization scratch below
 * and the stricter disjoint-storage contracts documented in linears_decomp.h.
 */
#ifndef LINEARS_WORK_H
#define LINEARS_WORK_H

#include "linears.h"
#include "linears_decomp.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Caller-owned fixed-F31 factorization scratch, with capacity in elements.
 */
typedef struct {
    linears_fixed_t *data; /**< Naturally aligned int64_t F31 array; NULL only for zero requirement. */
    size_t capacity;       /**< Available fixed-F31 elements, not bytes. */
} linears_fixed_workspace_t;

/**
 * @brief Add fixed-F31 working matrices without narrowing to a Q signal format.
 *
 * @param[in] a First working matrix, not NULL.
 * @param[in] b Second working matrix, not NULL, same shape as a.
 * @param[in,out] out Working destination, not NULL, same shape as a.
 * @param[in,out] workspace Typed basic scratch array; NULL only for empty output.
 * @param[in] workspace_bytes Available bytes; LINEARS_BASIC_WORKSPACE_BYTES bounds the requirement.
 * @return linears_status_t OK or a shared basic error; RANGE includes signed int64_t overflow.
 */
linears_status_t linears_add_fixed(const linears_matrix_t *a, const linears_matrix_t *b,
                                 linears_matrix_t *out, linears_workspace_unit_t *workspace,
                                 size_t workspace_bytes);

/**
 * @brief Subtract fixed-F31 working matrices exactly when the result fits int64_t.
 *
 * @param[in] a Working minuend, not NULL.
 * @param[in] b Working subtrahend, not NULL, same shape as a.
 * @param[in,out] out Working destination, not NULL, same shape as a.
 * @param[in,out] workspace Typed basic scratch array; NULL only for empty output.
 * @param[in] workspace_bytes Available bytes; use LINEARS_BASIC_WORKSPACE_BYTES.
 * @return linears_status_t OK or an applicable shared basic error.
 */
linears_status_t linears_sub_fixed(const linears_matrix_t *a, const linears_matrix_t *b,
                                 linears_matrix_t *out, linears_workspace_unit_t *workspace,
                                 size_t workspace_bytes);

/**
 * @brief Multiply fixed-F31 working matrices with checked exact wide accumulation.
 *
 * @param[in] a Working left matrix, not NULL, m by k.
 * @param[in] b Working right matrix, not NULL, k by n.
 * @param[in,out] out Working destination, not NULL, m by n.
 * @param[in,out] workspace Typed basic scratch array; NULL only for empty output.
 * @param[in] workspace_bytes Available bytes; use LINEARS_BASIC_WORKSPACE_BYTES(m,n).
 * @return linears_status_t OK or a shared basic error; RANGE includes accumulator
 *         overflow or an F31 rounded output outside int64_t range.
 */
linears_status_t linears_mul_fixed(const linears_matrix_t *a, const linears_matrix_t *b,
                                 linears_matrix_t *out, linears_workspace_unit_t *workspace,
                                 size_t workspace_bytes);

/**
 * @brief Transpose fixed-F31 working storage without changing raw values.
 *
 * @param[in] a Working source, not NULL, m by n.
 * @param[in,out] out Working destination, not NULL, n by m.
 * @param[in,out] workspace Typed basic scratch array; NULL only for empty output.
 * @param[in] workspace_bytes Available bytes; use LINEARS_BASIC_WORKSPACE_BYTES.
 * @return linears_status_t OK or an applicable shared basic error.
 */
linears_status_t linears_transpose_fixed(const linears_matrix_t *a, linears_matrix_t *out,
                                       linears_workspace_unit_t *workspace, size_t workspace_bytes);

/**
 * @brief Compute alpha*a + beta*b for fixed-F31 working matrices.
 *
 * @param[in] alpha F31 coefficient, e.g. LINEARS_FIXED_ONE for exact one.
 * @param[in] a First working input, not NULL.
 * @param[in] beta F31 coefficient.
 * @param[in] b Second working input, not NULL, same shape as a.
 * @param[in,out] out Working destination, not NULL, same shape as a.
 * @param[in,out] workspace Typed basic scratch array; NULL only for empty output.
 * @param[in] workspace_bytes Available bytes; use LINEARS_BASIC_WORKSPACE_BYTES.
 * @return linears_status_t OK or a shared basic error; products are accumulated
 *         exactly in signed 128 bits before one F31 rounding per output.
 */
linears_status_t linears_axpby_fixed(linears_fixed_t alpha, const linears_matrix_t *a,
                                   linears_fixed_t beta, const linears_matrix_t *b,
                                   linears_matrix_t *out, linears_workspace_unit_t *workspace,
                                   size_t workspace_bytes);

/**
 * @brief Fill a square working identity with exact LINEARS_FIXED_ONE on its diagonal.
 *
 * @param[in,out] out Square working destination, not NULL; zero order is permitted.
 * @return linears_status_t OK or ARGUMENT, DIMENSION, SIZE, ALIAS; no scratch needed.
 */
linears_status_t linears_identity_fixed(linears_matrix_t *out);

/**
 * @brief Solve A*X=B using the shared pivoted-LU algorithm in fixed F31.
 *
 * @param[in] a Working coefficient matrix, not NULL, n by n.
 * @param[in] b Working right-hand side, not NULL, n by nrhs.
 * @param[in,out] x Working solution destination, not NULL, n by nrhs.
 * @param[in,out] workspace Disjoint working scratch; capacity is at least
 *                          LINEARS_SOLVE_WORKSPACE(n,nrhs) F31 elements.
 * @return linears_status_t Shared factorization status, including SINGULAR or RANGE.
 * @note A is factored even when nrhs is zero. Output storage remains unchanged
 *       on failure; workspaces are unspecified. Factorization alias rules apply.
 */
linears_status_t linears_solve_fixed(const linears_matrix_t *a, const linears_matrix_t *b,
                                   linears_matrix_t *x, linears_fixed_workspace_t workspace);

/**
 * @brief Factor a strictly stored-symmetric positive-definite working matrix A=L*L^T.
 *
 * @param[in] a Working input matrix, not NULL, n by n.
 * @param[in,out] l Lower-triangular working destination, not NULL, n by n.
 * @param[in,out] workspace Disjoint scratch with at least
 *                          LINEARS_CHOLESKY_WORKSPACE(n) F31 elements.
 * @return linears_status_t Shared factorization status, including
 *         NOT_POSITIVE_DEFINITE for a nonpositive working pivot, or RANGE.
 * @note Outputs are not narrowed to Q signals; failure leaves l unchanged.
 */
linears_status_t linears_cholesky_fixed(const linears_matrix_t *a, linears_matrix_t *l,
                                      linears_fixed_workspace_t workspace);

#ifdef __cplusplus
}
#endif
#endif
