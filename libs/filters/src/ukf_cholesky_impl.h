/* Private unblocked Cholesky on validated working arrays. Include from
 * ukf_internal.h with U_CHOL_SUFFIX, U_CHOL_TYPE, U_CHOL_SQRT and
 * U_CHOL_ISFINITE defined; this file undefines them.
 *
 * The arithmetic is that of linears_cholesky_<suffix> (decomp_cholesky.inc)
 * in the same order: one product and one subtraction at a time, the
 * diagonal through sqrt, the rest divided by the finished diagonal, and the
 * upper triangle written as exact zeros. A factor is therefore bit-identical
 * to the public decomposition of the same input; only the descriptor,
 * workspace, alias and input-finite validation of the public entry point is
 * absent, because every caller has already established those facts. */
#define U_CHOL_JOIN_INNER(a, b) a##b
#define U_CHOL_JOIN(a, b) U_CHOL_JOIN_INNER(a, b)

/**
 * @brief Factor a validated symmetric positive-definite n-by-n working matrix.
 *
 * @param[in] a Row-major n-by-n matrix; every element finite and the stored
 *              triangles exactly equal. Only the lower triangle is read.
 * @param[in] n Order of the matrix, at least one.
 * @param[out] lower Row-major n-by-n destination for L with L*L^T = a; the
 *                   upper triangle is set to zero. Must not overlap a. After
 *                   a failure it holds a partial factor.
 * @return linears_status_t Factorization status.
 * @retval LINEARS_OK lower holds the complete factor.
 * @retval LINEARS_ERROR_NOT_POSITIVE_DEFINITE A pivot was not positive.
 * @retval LINEARS_ERROR_RANGE A product, difference or quotient overflowed.
 */
static inline linears_status_t U_CHOL_JOIN(ukf_cholesky_, U_CHOL_SUFFIX)(const U_CHOL_TYPE *a, size_t n,
                                                                        U_CHOL_TYPE *lower)
{
    size_t i, j, k;
    for (i = 0; i < n; ++i) {
        for (j = 0; j <= i; ++j) {
            U_CHOL_TYPE value = a[i * n + j];
            for (k = 0; k < j; ++k) {
                U_CHOL_TYPE product = lower[i * n + k] * lower[j * n + k];
                if (!U_CHOL_ISFINITE(product)) return LINEARS_ERROR_RANGE;
                value = value - product;
                if (!U_CHOL_ISFINITE(value)) return LINEARS_ERROR_RANGE;
            }
            if (i == j) {
                if (!(value > (U_CHOL_TYPE)0)) return LINEARS_ERROR_NOT_POSITIVE_DEFINITE;
                /* The root of a finite positive value is finite. */
                lower[i * n + j] = U_CHOL_SQRT(value);
            } else {
                value = value / lower[j * n + j];
                if (!U_CHOL_ISFINITE(value)) return LINEARS_ERROR_RANGE;
                lower[i * n + j] = value;
            }
        }
        for (j = i + 1; j < n; ++j) lower[i * n + j] = (U_CHOL_TYPE)0;
    }
    return LINEARS_OK;
}

#undef U_CHOL_JOIN
#undef U_CHOL_JOIN_INNER
#undef U_CHOL_SUFFIX
#undef U_CHOL_TYPE
#undef U_CHOL_SQRT
#undef U_CHOL_ISFINITE
