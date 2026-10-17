/**
 * @file decomp_qr.inc
 * @brief Full QR using explicitly normalized Householder reflectors.
 */
linears_status_t D_NAME(qr)(const linears_matrix_t *a, linears_matrix_t *q,
                           linears_matrix_t *r, D_WORKSPACE workspace)
{
    const linears_matrix_t *matrices[3] = {a, q, r};
    linears_status_t status = D_LOCAL(descriptors)(matrices, 3);
    size_t m, n, r_count, q_count, required, i, j, k;
    D_SCALAR *r_work, *q_work, *v;
    int q_exponent, r_exponent;
    if (status != LINEARS_OK) return status;
    m = a->rows;
    n = a->cols;
    if (q->rows != m || q->cols != m || r->rows != m || r->cols != n)
        return LINEARS_ERROR_DIMENSION;
    required = LINEARS_QR_WORKSPACE(m, n);
    status = D_LOCAL(validate)(matrices, 3, 1, workspace, required);
    if (status != LINEARS_OK) return status;
    if (m == 0) { q->exponent = 0; r->exponent = 0; return LINEARS_OK; }
    r_count = m * n;
    q_count = m * m;
    r_work = workspace.data;
    q_work = r_work + r_count;
    v = q_work + q_count;
    for (i = 0; i < r_count; ++i) r_work[i] = D_LOCAL(load)(a, i, &status);
    for (i = 0; i < m; ++i)
        for (j = 0; j < m; ++j) q_work[i * m + j] = i == j ? D_ONE : D_ZERO;
    if (status != LINEARS_OK) return status;
    for (k = 0; k < m && k < n; ++k) {
        D_SCALAR norm = D_LOCAL(norm)(r_work + k * n + k, m - k, n, &status);
        D_SCALAR v_norm, diagonal, two;
        if (status != LINEARS_OK) return status;
        if (D_CMP(norm, D_ZERO) == 0) continue;
        /* Normalize x before forming x + sign(x0)*||x||e0. This avoids
         * cancellation and the otherwise avoidable x0-alpha overflow. */
        for (i = k; i < m; ++i) v[i] = D_DIV(r_work[i * n + k], norm);
        if (D_CMP(v[k], D_ZERO) < 0) {
            v[k] = D_SUB(v[k], D_ONE);
            diagonal = norm;
        } else {
            v[k] = D_ADD(v[k], D_ONE);
            diagonal = D_NEG(norm);
        }
        v_norm = D_LOCAL(norm)(v + k, m - k, 1, &status);
        for (i = k; i < m; ++i) v[i] = D_DIV(v[i], v_norm);
        two = D_CONST(2);
        if (status != LINEARS_OK) return status;
        for (j = k + 1; j < n; ++j) {
            D_SCALAR maximum = D_ZERO, dot = D_ZERO;
            for (i = k; i < m; ++i) {
                D_SCALAR magnitude = D_ABS(r_work[i * n + j]);
                if (D_CMP(magnitude, maximum) > 0) maximum = magnitude;
            }
            if (D_CMP(maximum, D_ZERO) == 0) continue;
            /* Apply H to a scaled column, then restore its scale. */
            for (i = k; i < m; ++i) {
                D_SCALAR ratio = D_DIV(r_work[i * n + j], maximum);
                D_SCALAR product = D_MUL(v[i], ratio);
                dot = D_ADD(dot, product);
            }
            dot = D_MUL(two, dot);
            for (i = k; i < m; ++i) {
                D_SCALAR ratio = D_DIV(r_work[i * n + j], maximum);
                D_SCALAR product = D_MUL(v[i], dot);
                ratio = D_SUB(ratio, product);
                r_work[i * n + j] = D_MUL(ratio, maximum);
            }
        }
        r_work[k * n + k] = diagonal;
        for (i = k + 1; i < m; ++i) r_work[i * n + k] = D_ZERO;
        /* Q <- Q*H; reflectors act on columns k..m-1. */
        for (i = 0; i < m; ++i) {
            D_SCALAR dot = D_ZERO;
            for (j = k; j < m; ++j) {
                D_SCALAR product = D_MUL(q_work[i * m + j], v[j]);
                dot = D_ADD(dot, product);
            }
            dot = D_MUL(two, dot);
            for (j = k; j < m; ++j) {
                D_SCALAR product = D_MUL(dot, v[j]);
                q_work[i * m + j] = D_SUB(q_work[i * m + j], product);
            }
        }
        if (status != LINEARS_OK) return status;
    }
    status = D_LOCAL(block_exponent)(q_work, q_count, &q_exponent);
    if (status != LINEARS_OK) return status;
    status = D_LOCAL(block_exponent)(r_work, r_count, &r_exponent);
    if (status != LINEARS_OK) return status;
    D_LOCAL(store_block)(q, q_work, q_count, q_exponent);
    D_LOCAL(store_block)(r, r_work, r_count, r_exponent);
    return LINEARS_OK;
}
