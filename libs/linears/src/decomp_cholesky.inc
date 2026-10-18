/**
 * @file decomp_cholesky.inc
 * @brief Unblocked Cholesky for exactly symmetric positive-definite inputs.
 */
linears_status_t D_NAME(cholesky)(const linears_matrix_t *a, linears_matrix_t *l,
                                 D_WORKSPACE workspace)
{
    const linears_matrix_t *matrices[2] = {a, l};
    linears_status_t status = D_LOCAL(descriptors)(matrices, 2);
    size_t n, count, i, j;
#if !D_FIXED
    size_t k;
#endif
    if (status != LINEARS_OK) return status;
    n = a->rows;
    if (a->cols != n || l->rows != n || l->cols != n) return LINEARS_ERROR_DIMENSION;
    count = LINEARS_CHOLESKY_WORKSPACE(n);
    status = D_LOCAL(validate)(matrices, 2, 1, workspace, count);
    if (status != LINEARS_OK) return status;
    for (i = 0; i < count; ++i) workspace.data[i] = D_LOCAL(load)(a, i, &status);
    if (status != LINEARS_OK) return status;
    status = D_LOCAL(check_symmetric)(workspace.data, n);
    if (status != LINEARS_OK) return status;
    for (i = 0; i < n; ++i) {
        for (j = 0; j <= i; ++j) {
            D_SCALAR value = workspace.data[i * n + j];
#if D_FIXED
            value = D_LOCAL(dot_sub)(value, workspace.data + i * n, 1,
                                    workspace.data + j * n, 1, j, &status);
#else
            for (k = 0; k < j; ++k) {
                D_SCALAR product = D_MUL(workspace.data[i * n + k], workspace.data[j * n + k]);
                value = D_SUB(value, product);
            }
#endif
            if (status != LINEARS_OK) return status;
            if (i == j) {
                if (D_CMP(value, D_ZERO) <= 0) return LINEARS_ERROR_NOT_POSITIVE_DEFINITE;
                workspace.data[i * n + j] = D_SQRT(value);
            } else workspace.data[i * n + j] = D_DIV(value, workspace.data[j * n + j]);
            if (status != LINEARS_OK) return status;
        }
        for (j = i + 1; j < n; ++j) workspace.data[i * n + j] = D_ZERO;
    }
    status = D_LOCAL(check_block)(workspace.data, count);
    if (status != LINEARS_OK) return status;
    D_LOCAL(store_block)(l, workspace.data, count);
    return LINEARS_OK;
}
