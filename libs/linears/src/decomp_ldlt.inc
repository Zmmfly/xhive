/**
 * @file decomp_ldlt.inc
 * @brief Shared Bunch--Parlett symmetric-indefinite factorization kernel.
 *
 * Instantiate through decomp_internal.h. Working arithmetic is selected by
 * D_* macros; fixed-point instantiations use only linears_fixed_t operations.
 */

/**
 * @brief Apply a symmetric active-submatrix interchange and permute prior L.
 *
 * The leading columns below their pivots contain L, not a symmetric matrix.
 * Only active rows participate in the column interchange, preserving old D.
 */
static void D_LOCAL(ldlt_swap)(D_SCALAR *work, size_t n, size_t first,
                               size_t p, size_t q, size_t *perm)
{
    size_t i;
    size_t index;
    D_SCALAR value;
    if (p == q) {
        return;
    }
    for (i = 0; i < n; ++i) {
        value = work[p * n + i];
        work[p * n + i] = work[q * n + i];
        work[q * n + i] = value;
    }
    for (i = first; i < n; ++i) {
        value = work[i * n + p];
        work[i * n + p] = work[i * n + q];
        work[i * n + q] = value;
    }
    index = perm[p];
    perm[p] = perm[q];
    perm[q] = index;
}

/**
 * @brief Factor a validated symmetric n-by-n matrix in its workspace.
 *
 * Complete symmetric pivoting uses alpha = (1 + sqrt(17)) / 8. The largest
 * diagonal is a 1-by-1 pivot when its magnitude is at least alpha times the
 * largest off-diagonal; otherwise the largest off-diagonal defines a 2-by-2
 * pivot. Strict comparisons select the first row-major maximum on ties.
 *
 * The lower triangle holds L multipliers except within a 2-by-2 D block.
 * Diagonal entries and the first off-diagonal of each 2-by-2 block hold D.
 * Upper entries in the current pivot rows retain the original coupling C
 * until the Schur update completes, so no extra column buffers are needed.
 */
static linears_status_t D_LOCAL(ldlt_factor)(D_SCALAR *work, size_t n,
                                             size_t *perm, uint8_t *blocks)
{
    linears_status_t status = LINEARS_OK;
    D_SCALAR alpha;
    D_SCALAR eight;
    size_t i;
    size_t j;
    size_t k;

    alpha = D_CONST(17);
    alpha = D_SQRT(alpha);
    alpha = D_ADD(D_ONE, alpha);
    eight = D_CONST(8);
    alpha = D_DIV(alpha, eight);
    if (status != LINEARS_OK) {
        return status;
    }
    for (i = 0; i < n; ++i) {
        perm[i] = i;
        blocks[i] = 0;
    }

    k = 0;
    while (k < n) {
        D_SCALAR diagonal_max = D_ZERO;
        D_SCALAR off_diagonal_max = D_ZERO;
        int single_pivot;
        size_t diagonal_index = k;
        size_t p = k;
        size_t q = k;

        for (i = k; i < n; ++i) {
            D_SCALAR magnitude = D_ABS(work[i * n + i]);
            if (D_CMP(magnitude, diagonal_max) > 0) {
                diagonal_max = magnitude;
                diagonal_index = i;
            }
            for (j = i + 1; j < n; ++j) {
                magnitude = D_ABS(work[i * n + j]);
                if (D_CMP(magnitude, off_diagonal_max) > 0) {
                    off_diagonal_max = magnitude;
                    p = i;
                    q = j;
                }
            }
        }
        if (D_CMP(diagonal_max, D_ZERO) == 0 &&
            D_CMP(off_diagonal_max, D_ZERO) == 0) {
            return LINEARS_ERROR_SINGULAR;
        }
        single_pivot = D_CMP(diagonal_max, off_diagonal_max) >= 0;
        if (!single_pivot) {
            /* The ratio is below one. Avoid scaling the off-diagonal maximum
             * down past the fixed working exponent limit just to compare it. */
            D_SCALAR ratio = D_DIV(diagonal_max, off_diagonal_max);
            if (status != LINEARS_OK) {
                return status;
            }
            single_pivot = D_CMP(ratio, alpha) >= 0;
        }

        if (single_pivot) {
            D_SCALAR pivot;
            D_LOCAL(ldlt_swap)(work, n, k, k, diagonal_index, perm);
            pivot = work[k * n + k];
            blocks[k] = 1;
            for (i = k + 1; i < n; ++i) {
                work[i * n + k] = D_DIV(work[i * n + k], pivot);
            }
            for (i = k + 1; i < n; ++i) {
                for (j = i; j < n; ++j) {
                    D_SCALAR product = D_MUL(work[j * n + k], work[k * n + i]);
                    D_SCALAR updated = D_SUB(work[j * n + i], product);
                    work[j * n + i] = updated;
                    work[i * n + j] = updated;
                }
            }
            ++k;
        } else {
            D_SCALAR a;
            D_SCALAR b;
            D_SCALAR c;
            D_SCALAR delta;

            /* p < q, so swapping k with p cannot move q. */
            D_LOCAL(ldlt_swap)(work, n, k, k, p, perm);
            D_LOCAL(ldlt_swap)(work, n, k, k + 1, q, perm);
            blocks[k] = 2;
            blocks[k + 1] = 0;
            b = work[k * n + k + 1];
            a = D_DIV(work[k * n + k], b);
            c = D_DIV(work[(k + 1) * n + k + 1], b);
            delta = D_MUL(a, c);
            delta = D_SUB(delta, D_ONE);

            /* Divide by b before forming the determinant. Since |a|, |c| <
             * alpha, delta stays away from zero; b*b is never evaluated. */
            for (i = k + 2; i < n; ++i) {
                D_SCALAR x = D_DIV(work[i * n + k], b);
                D_SCALAR y = D_DIV(work[i * n + k + 1], b);
                D_SCALAR first = D_MUL(c, x);
                D_SCALAR second = D_MUL(a, y);
                first = D_SUB(first, y);
                second = D_SUB(second, x);
                work[i * n + k] = D_DIV(first, delta);
                work[i * n + k + 1] = D_DIV(second, delta);
            }
            for (i = k + 2; i < n; ++i) {
                for (j = i; j < n; ++j) {
                    D_SCALAR first = D_MUL(work[j * n + k], work[k * n + i]);
                    D_SCALAR second = D_MUL(work[j * n + k + 1],
                                             work[(k + 1) * n + i]);
                    D_SCALAR updated = D_SUB(work[j * n + i], first);
                    updated = D_SUB(updated, second);
                    work[j * n + i] = updated;
                    work[i * n + j] = updated;
                }
            }
            k += 2;
        }
        if (status != LINEARS_OK) {
            return status;
        }
    }
    return LINEARS_OK;
}

linears_status_t D_NAME(ldlt)(const linears_matrix_t *a, linears_matrix_t *l,
    linears_matrix_t *d, size_t *permutation, size_t permutation_capacity,
    uint8_t *blocks, size_t blocks_capacity, D_WORKSPACE workspace)
{
    const linears_matrix_t *matrices[3] = {a, l, d};
    linears_status_t status;
    D_SCALAR minimum_l = D_ZERO;
    D_SCALAR maximum_l;
    D_SCALAR minimum_d = D_ZERO;
    D_SCALAR maximum_d = D_ZERO;
    size_t n;
    size_t required;
    size_t permutation_bytes;
    size_t i;
    size_t j;
    int exponent_l;
    int exponent_d;

    status = D_LOCAL(descriptors)(matrices, 3);
    if (status != LINEARS_OK) {
        return status;
    }
    n = a->rows;
    if (a->cols != n || l->rows != n || l->cols != n ||
        d->rows != n || d->cols != n) {
        return LINEARS_ERROR_DIMENSION;
    }
    required = LINEARS_LDLT_WORKSPACE(n);
    status = D_LOCAL(validate)(matrices, 3, 1, workspace, required);
    if (status != LINEARS_OK) {
        return status;
    }
    if (permutation_capacity < n || blocks_capacity < n) {
        return LINEARS_ERROR_WORKSPACE;
    }
    permutation_bytes = LINEARS_SIZE_MUL(n, sizeof(*permutation));
    status = D_LOCAL(metadata)(permutation, permutation_bytes,
        D_ALIGNOF(size_t), matrices, 3, workspace, required);
    if (status != LINEARS_OK) {
        return status;
    }
    status = D_LOCAL(metadata)(blocks, n, D_ALIGNOF(uint8_t),
        matrices, 3, workspace, required);
    if (status != LINEARS_OK) {
        return status;
    }
    if (linears_decomp_overlap(permutation, permutation_bytes, blocks, n)) {
        return LINEARS_ERROR_ALIAS;
    }

    for (i = 0; i < required; ++i) {
        workspace.data[i] = D_LOCAL(load)(a, i, &status);
    }
    if (status != LINEARS_OK) {
        return status;
    }
    status = D_LOCAL(check_symmetric)(workspace.data, n);
    if (status != LINEARS_OK) {
        return status;
    }
    status = D_LOCAL(ldlt_factor)(workspace.data, n, permutation, blocks);
    if (status != LINEARS_OK) {
        return status;
    }

    maximum_l = n != 0 ? D_ONE : D_ZERO;
    for (i = 0; i < n; ++i) {
        D_SCALAR value = workspace.data[i * n + i];
        if (D_CMP(value, minimum_d) < 0) {
            minimum_d = value;
        }
        if (D_CMP(value, maximum_d) > 0) {
            maximum_d = value;
        }
        for (j = 0; j < i; ++j) {
            value = workspace.data[i * n + j];
            if (blocks[j] == 2 && i == j + 1) {
                if (D_CMP(value, minimum_d) < 0) {
                    minimum_d = value;
                }
                if (D_CMP(value, maximum_d) > 0) {
                    maximum_d = value;
                }
            } else {
                if (D_CMP(value, minimum_l) < 0) {
                    minimum_l = value;
                }
                if (D_CMP(value, maximum_l) > 0) {
                    maximum_l = value;
                }
            }
        }
    }
    status = D_LOCAL(output_exponent_bounds)(minimum_l, maximum_l, &exponent_l);
    if (status != LINEARS_OK) {
        return status;
    }
    status = D_LOCAL(output_exponent_bounds)(minimum_d, maximum_d, &exponent_d);
    if (status != LINEARS_OK) {
        return status;
    }

    /* Both output exponents are preflighted before either matrix is written. */
    for (i = 0; i < n; ++i) {
        for (j = 0; j < n; ++j) {
            D_SCALAR value_l = D_ZERO;
            D_SCALAR value_d = D_ZERO;
            if (i == j) {
                value_l = D_ONE;
                value_d = workspace.data[i * n + i];
            } else if (i > j) {
                if (blocks[j] == 2 && i == j + 1) {
                    value_d = workspace.data[i * n + j];
                } else {
                    value_l = workspace.data[i * n + j];
                }
            } else if (blocks[i] == 2 && j == i + 1) {
                value_d = workspace.data[i * n + j];
            }
            D_LOCAL(store)(l, i * n + j, value_l, exponent_l);
            D_LOCAL(store)(d, i * n + j, value_d, exponent_d);
        }
    }
    l->exponent = exponent_l;
    d->exponent = exponent_d;
    return LINEARS_OK;
}
