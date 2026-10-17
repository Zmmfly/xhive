/**
 * @file decomp_lu.inc
 * @brief Unblocked partial-pivot Gaussian elimination shared by the four formats.
 */

/* B, when present, follows row swaps and elimination; only back substitution
 * remains. Entire A rows are swapped, including previously stored L columns. */
static linears_status_t D_LOCAL(lu_factor)(D_SCALAR *a, size_t n, D_SCALAR *b,
    size_t rhs, size_t *permutation, int *parity)
{
    size_t k, i, j;
    linears_status_t status = LINEARS_OK;
    *parity = 1;
    if (permutation != NULL)
        for (i = 0; i < n; ++i) permutation[i] = i;
    for (k = 0; k < n; ++k) {
        size_t pivot = k;
        D_SCALAR maximum = D_ABS(a[k * n + k]);
        for (i = k + 1; i < n; ++i) {
            D_SCALAR magnitude = D_ABS(a[i * n + k]);
            if (D_CMP(magnitude, maximum) > 0) { maximum = magnitude; pivot = i; }
        }
        if (D_CMP(maximum, D_ZERO) == 0) return LINEARS_ERROR_SINGULAR;
        if (pivot != k) {
            for (j = 0; j < n; ++j) {
                D_SCALAR temporary = a[k * n + j];
                a[k * n + j] = a[pivot * n + j];
                a[pivot * n + j] = temporary;
            }
            for (j = 0; j < rhs; ++j) {
                D_SCALAR temporary = b[k * rhs + j];
                b[k * rhs + j] = b[pivot * rhs + j];
                b[pivot * rhs + j] = temporary;
            }
            if (permutation != NULL) {
                size_t temporary = permutation[k];
                permutation[k] = permutation[pivot];
                permutation[pivot] = temporary;
            }
            *parity = -*parity;
        }
        for (i = k + 1; i < n; ++i) {
            D_SCALAR factor = D_DIV(a[i * n + k], a[k * n + k]);
            a[i * n + k] = factor;
            for (j = k + 1; j < n; ++j) {
                D_SCALAR product = D_MUL(factor, a[k * n + j]);
                a[i * n + j] = D_SUB(a[i * n + j], product);
            }
            for (j = 0; j < rhs; ++j) {
                D_SCALAR product = D_MUL(factor, b[k * rhs + j]);
                b[i * rhs + j] = D_SUB(b[i * rhs + j], product);
            }
            if (status != LINEARS_OK) return status;
        }
    }
    return LINEARS_OK;
}

static linears_status_t D_LOCAL(back_substitute)(const D_SCALAR *lu, size_t n,
                                                D_SCALAR *b, size_t rhs)
{
    size_t row, column, k;
    linears_status_t status = LINEARS_OK;
    for (row = n; row != 0;) {
        --row;
        for (column = 0; column < rhs; ++column) {
            D_SCALAR value = b[row * rhs + column];
            for (k = row + 1; k < n; ++k) {
                D_SCALAR product = D_MUL(lu[row * n + k], b[k * rhs + column]);
                value = D_SUB(value, product);
            }
            b[row * rhs + column] = D_DIV(value, lu[row * n + row]);
            if (status != LINEARS_OK) return status;
        }
    }
    return LINEARS_OK;
}

linears_status_t D_NAME(lu)(const linears_matrix_t *a, linears_matrix_t *l,
    linears_matrix_t *u, size_t *permutation, size_t permutation_capacity,
    int *parity, D_WORKSPACE workspace)
{
    const linears_matrix_t *matrices[3] = {a, l, u};
    linears_status_t status = D_LOCAL(descriptors)(matrices, 3);
    size_t n, count, i, j;
    int sign, l_exponent, u_exponent;
    D_SCALAR l_minimum = D_ZERO, l_maximum = D_ZERO;
    D_SCALAR u_minimum = D_ZERO, u_maximum = D_ZERO;
    if (status != LINEARS_OK) return status;
    n = a->rows;
    if (a->cols != n || l->rows != n || l->cols != n || u->rows != n || u->cols != n)
        return LINEARS_ERROR_DIMENSION;
    count = LINEARS_LU_WORKSPACE(n);
    status = D_LOCAL(validate)(matrices, 3, 1, workspace, count);
    if (status != LINEARS_OK) return status;
    if (permutation_capacity < n) return LINEARS_ERROR_WORKSPACE;
    status = D_LOCAL(metadata)(permutation, n * sizeof(*permutation), D_ALIGNOF(size_t),
                               matrices, 3, workspace, count);
    if (status != LINEARS_OK) return status;
    status = D_LOCAL(metadata)(parity, sizeof(*parity), D_ALIGNOF(int), matrices, 3, workspace, count);
    if (status != LINEARS_OK) return status;
    if (linears_decomp_overlap(permutation, n * sizeof(*permutation), parity, sizeof(*parity)))
        return LINEARS_ERROR_ALIAS;
    for (i = 0; i < count; ++i) workspace.data[i] = D_LOCAL(load)(a, i, &status);
    if (status != LINEARS_OK) return status;
    status = D_LOCAL(lu_factor)(workspace.data, n, NULL, 0, permutation, &sign);
    if (status != LINEARS_OK) return status;
    if (n != 0) l_maximum = D_ONE;
    for (i = 0; i < n; ++i) {
        for (j = 0; j < n; ++j) {
            D_SCALAR value = workspace.data[i * n + j];
            if (i > j) {
                if (D_CMP(value, l_minimum) < 0) l_minimum = value;
                if (D_CMP(value, l_maximum) > 0) l_maximum = value;
            } else {
                if (D_CMP(value, u_minimum) < 0) u_minimum = value;
                if (D_CMP(value, u_maximum) > 0) u_maximum = value;
            }
        }
    }
    status = D_LOCAL(output_exponent_bounds)(l_minimum, l_maximum, &l_exponent);
    if (status != LINEARS_OK) return status;
    status = D_LOCAL(output_exponent_bounds)(u_minimum, u_maximum, &u_exponent);
    if (status != LINEARS_OK) return status;
    for (i = 0; i < n; ++i) {
        for (j = 0; j < n; ++j) {
            D_SCALAR value = workspace.data[i * n + j];
            D_LOCAL(store)(l, i * n + j, i == j ? D_ONE : (i > j ? value : D_ZERO), l_exponent);
            D_LOCAL(store)(u, i * n + j, i <= j ? value : D_ZERO, u_exponent);
        }
    }
    l->exponent = l_exponent;
    u->exponent = u_exponent;
    *parity = sign;
    return LINEARS_OK;
}

linears_status_t D_NAME(solve)(const linears_matrix_t *a, const linears_matrix_t *b,
                              linears_matrix_t *x, D_WORKSPACE workspace)
{
    const linears_matrix_t *matrices[3] = {a, b, x};
    linears_status_t status = D_LOCAL(descriptors)(matrices, 3);
    size_t n, rhs, matrix_count, result_count, required, i;
    int sign, exponent;
    D_SCALAR *solution;
    if (status != LINEARS_OK) return status;
    n = a->rows;
    rhs = b->cols;
    if (a->cols != n || b->rows != n || x->rows != n || x->cols != rhs)
        return LINEARS_ERROR_DIMENSION;
    required = LINEARS_SOLVE_WORKSPACE(n, rhs);
    status = D_LOCAL(validate)(matrices, 3, 2, workspace, required);
    if (status != LINEARS_OK) return status;
    matrix_count = n * n;
    result_count = n * rhs;
    solution = n == 0 ? workspace.data : workspace.data + matrix_count;
    for (i = 0; i < matrix_count; ++i) workspace.data[i] = D_LOCAL(load)(a, i, &status);
    for (i = 0; i < result_count; ++i) solution[i] = D_LOCAL(load)(b, i, &status);
    if (status != LINEARS_OK) return status;
    status = D_LOCAL(lu_factor)(workspace.data, n, solution, rhs, NULL, &sign);
    if (status != LINEARS_OK) return status;
    status = D_LOCAL(back_substitute)(workspace.data, n, solution, rhs);
    if (status != LINEARS_OK) return status;
    status = D_LOCAL(block_exponent)(solution, result_count, &exponent);
    if (status != LINEARS_OK) return status;
    D_LOCAL(store_block)(x, solution, result_count, exponent);
    return LINEARS_OK;
}

linears_status_t D_NAME(inverse)(const linears_matrix_t *a, linears_matrix_t *inverse,
                                D_WORKSPACE workspace)
{
    const linears_matrix_t *matrices[2] = {a, inverse};
    linears_status_t status = D_LOCAL(descriptors)(matrices, 2);
    size_t n, count, required, i, j;
    int sign, exponent;
    D_SCALAR *solution;
    if (status != LINEARS_OK) return status;
    n = a->rows;
    if (a->cols != n || inverse->rows != n || inverse->cols != n)
        return LINEARS_ERROR_DIMENSION;
    required = LINEARS_INVERSE_WORKSPACE(n);
    status = D_LOCAL(validate)(matrices, 2, 1, workspace, required);
    if (status != LINEARS_OK) return status;
    count = n * n;
    solution = n == 0 ? workspace.data : workspace.data + count;
    for (i = 0; i < count; ++i) workspace.data[i] = D_LOCAL(load)(a, i, &status);
    for (i = 0; i < n; ++i)
        for (j = 0; j < n; ++j) solution[i * n + j] = i == j ? D_ONE : D_ZERO;
    if (status != LINEARS_OK) return status;
    status = D_LOCAL(lu_factor)(workspace.data, n, solution, n, NULL, &sign);
    if (status != LINEARS_OK) return status;
    status = D_LOCAL(back_substitute)(workspace.data, n, solution, n);
    if (status != LINEARS_OK) return status;
    status = D_LOCAL(block_exponent)(solution, count, &exponent);
    if (status != LINEARS_OK) return status;
    D_LOCAL(store_block)(inverse, solution, count, exponent);
    return LINEARS_OK;
}

/* Accumulate determinant exponents separately, so alternating huge and tiny
 * diagonal factors do not overflow merely because of their multiplication order. */
static D_SCALAR D_LOCAL(diagonal_product)(const D_SCALAR *lu, size_t n, int sign,
                                          linears_status_t *result_status)
{
    size_t i;
    int64_t exponent = 0;
    linears_status_t status = LINEARS_OK;
    D_SCALAR product = D_ONE;
    for (i = 0; i < n; ++i) {
#if D_FIXED
        D_SCALAR factor = lu[i * n + i];
        exponent += factor.exponent;
        factor.exponent = 0;
        product = D_MUL(product, factor);
        exponent += product.exponent;
        product.exponent = 0;
#else
        int factor_exponent, product_exponent;
#if D_FORMAT == 1
        D_SCALAR factor = frexpf(lu[i * n + i], &factor_exponent);
        product = frexpf(product * factor, &product_exponent);
#else
        D_SCALAR factor = frexp(lu[i * n + i], &factor_exponent);
        product = frexp(product * factor, &product_exponent);
#endif
        exponent += (int64_t)factor_exponent + product_exponent;
#endif
    }
    if (sign < 0) product = D_NEG(product);
#if D_FIXED
    if (n != 0) {
        if (exponent < LINEARS_EXPONENT_MIN || exponent > LINEARS_EXPONENT_MAX)
            status = LINEARS_ERROR_RANGE;
        else product.exponent = (int)exponent;
    }
#else
    if (exponent > INT_MAX) status = LINEARS_ERROR_RANGE;
    else if (exponent < INT_MIN) product = D_ZERO;
    else product = D_LOCAL(finite)(D_MATH_SCALE(product, (int)exponent), &status);
#endif
    *result_status = status;
    return product;
}

linears_status_t D_NAME(determinant)(const linears_matrix_t *a,
    linears_matrix_t *determinant, D_WORKSPACE workspace)
{
    const linears_matrix_t *matrices[2] = {a, determinant};
    linears_status_t status = D_LOCAL(descriptors)(matrices, 2);
    size_t n, count, i;
    int sign, exponent;
    D_SCALAR result;
    if (status != LINEARS_OK) return status;
    n = a->rows;
    if (a->cols != n || determinant->rows != 1 || determinant->cols != 1)
        return LINEARS_ERROR_DIMENSION;
    count = LINEARS_DETERMINANT_WORKSPACE(n);
    status = D_LOCAL(validate)(matrices, 2, 1, workspace, count);
    if (status != LINEARS_OK) return status;
    for (i = 0; i < count; ++i) workspace.data[i] = D_LOCAL(load)(a, i, &status);
    if (status != LINEARS_OK) return status;
    status = D_LOCAL(lu_factor)(workspace.data, n, NULL, 0, NULL, &sign);
    if (status == LINEARS_ERROR_SINGULAR) { result = D_ZERO; status = LINEARS_OK; }
    else if (status == LINEARS_OK) result = D_LOCAL(diagonal_product)(workspace.data, n, sign, &status);
    else return status;
    if (status != LINEARS_OK) return status;
    status = D_LOCAL(block_exponent)(&result, 1, &exponent);
    if (status != LINEARS_OK) return status;
    D_LOCAL(store)(determinant, 0, result, exponent);
    determinant->exponent = exponent;
    return LINEARS_OK;
}
