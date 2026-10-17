/*
 * Column-cyclic one-sided Jacobi SVD, instantiated by decomp_svd.c.
 * Algorithm references and precision boundaries: ../docs/decomp_svd.md.
 * D_* arithmetic uses the shared numeric adapter; status is sticky.
 */

static linears_status_t D_LOCAL(svd_correlation)(const D_SCALAR *b,
                                                size_t rows, size_t columns,
                                                size_t p, size_t q,
                                                D_SCALAR norm_p,
                                                D_SCALAR norm_q,
                                                D_SCALAR *correlation)
{
    linears_status_t status = LINEARS_OK;
    D_SCALAR sum = D_ZERO;
    size_t row;
    for (row = 0; row < rows; ++row) {
        D_SCALAR left = D_DIV(b[row * columns + p], norm_p);
        D_SCALAR right = D_DIV(b[row * columns + q], norm_q);
        D_SCALAR product = D_MUL(left, right);
        sum = D_ADD(sum, product);
    }
    if (status == LINEARS_OK) {
        *correlation = sum;
    }
    return status;
}

static linears_status_t D_LOCAL(svd_rotation)(D_SCALAR norm_p,
                                             D_SCALAR norm_q,
                                             D_SCALAR correlation,
                                             D_SCALAR *cosine,
                                             D_SCALAR *sine)
{
    linears_status_t status = LINEARS_OK;
    D_SCALAR scale = D_CMP(norm_p, norm_q) > 0 ? norm_p : norm_q;
    D_SCALAR p = D_DIV(norm_p, scale);
    D_SCALAR q = D_DIV(norm_q, scale);
    D_SCALAR alpha = D_MUL(p, p);
    D_SCALAR beta = D_MUL(q, q);
    D_SCALAR delta = D_SUB(beta, alpha);
    D_SCALAR gamma = D_MUL(p, q);
    D_SCALAR two = D_CONST(2);
    D_SCALAR abs_delta;
    D_SCALAR abs_gamma;
    D_SCALAR h;
    D_SCALAR t;
    D_SCALAR c;
    D_SCALAR s;
    gamma = D_MUL(gamma, correlation);
    gamma = D_MUL(two, gamma);
    abs_delta = D_ABS(delta);
    abs_gamma = D_ABS(gamma);
    h = D_CMP(abs_delta, abs_gamma) > 0 ? abs_delta : abs_gamma;
    if (status != LINEARS_OK) {
        return status;
    }
    /* A nonzero normalized correlation and retained norms imply h > 0. */
    p = D_DIV(delta, h);
    q = D_DIV(gamma, h);
    alpha = D_MUL(p, p);
    beta = D_MUL(q, q);
    t = D_ADD(alpha, beta);
    t = D_SQRT(t);
    t = D_MUL(h, t);
    t = D_ADD(abs_delta, t);
    t = D_DIV(abs_gamma, t);
    if ((D_CMP(delta, D_ZERO) < 0) !=
        (D_CMP(correlation, D_ZERO) < 0)) {
        t = D_NEG(t);
    }
    c = D_MUL(t, t);
    c = D_ADD(D_ONE, c);
    c = D_SQRT(c);
    c = D_DIV(D_ONE, c);
    s = D_MUL(t, c);
    if (status == LINEARS_OK) {
        *cosine = c;
        *sine = s;
    }
    return status;
}

static linears_status_t D_LOCAL(svd_rotate_columns)(D_SCALAR *b,
                                                   size_t rows,
                                                   size_t columns,
                                                   size_t p, size_t q,
                                                   D_SCALAR cosine,
                                                   D_SCALAR sine)
{
    linears_status_t status = LINEARS_OK;
    size_t row;
    for (row = 0; row < rows; ++row) {
        D_SCALAR old_p = b[row * columns + p];
        D_SCALAR old_q = b[row * columns + q];
        D_SCALAR cp = D_MUL(cosine, old_p);
        D_SCALAR sq = D_MUL(sine, old_q);
        D_SCALAR sp = D_MUL(sine, old_p);
        D_SCALAR cq = D_MUL(cosine, old_q);
        b[row * columns + p] = D_SUB(cp, sq);
        b[row * columns + q] = D_ADD(sp, cq);
    }
    return status;
}

static linears_status_t D_LOCAL(svd_complete_column)(D_SCALAR *b,
                                                    size_t rows,
                                                    size_t columns,
                                                    size_t column)
{
    linears_status_t status = LINEARS_OK;
    D_SCALAR dimension = D_CONST((int64_t)rows);
    D_SCALAR two = D_CONST(2);
    D_SCALAR minimum_norm;
    size_t coordinate;
    dimension = D_SQRT(dimension);
    dimension = D_MUL(two, dimension);
    minimum_norm = D_DIV(D_ONE, dimension);
    for (coordinate = 0; coordinate < rows; ++coordinate) {
        D_SCALAR norm;
        size_t row;
        size_t previous;
        unsigned int pass;
        for (row = 0; row < rows; ++row) {
            b[row * columns + column] = row == coordinate ? D_ONE : D_ZERO;
        }
        /* Two-pass modified Gram-Schmidt against accepted unit columns. */
        for (pass = 0; pass < 2; ++pass) {
            for (previous = 0; previous < column; ++previous) {
                D_SCALAR projection = D_ZERO;
                for (row = 0; row < rows; ++row) {
                    D_SCALAR product = D_MUL(b[row * columns + previous],
                                             b[row * columns + column]);
                    projection = D_ADD(projection, product);
                }
                for (row = 0; row < rows; ++row) {
                    D_SCALAR product = D_MUL(projection,
                                             b[row * columns + previous]);
                    b[row * columns + column] =
                        D_SUB(b[row * columns + column], product);
                }
            }
        }
        if (status != LINEARS_OK) {
            return status;
        }
        norm = D_LOCAL(norm)(b + column, rows, columns, &status);
        if (status != LINEARS_OK) {
            return status;
        }
        /* At least one coordinate has residual norm >= 1/sqrt(rows). */
        if (D_CMP(norm, minimum_norm) >= 0) {
            for (row = 0; row < rows; ++row) {
                b[row * columns + column] =
                    D_DIV(b[row * columns + column], norm);
            }
            return status;
        }
    }
    return LINEARS_ERROR_NO_CONVERGENCE;
}

static linears_status_t D_LOCAL(svd_factor)(D_SCALAR *b, D_SCALAR *w,
                                           D_SCALAR *norms, size_t rows,
                                           size_t columns, size_t max_sweeps,
                                           D_SCALAR tolerance)
{
    linears_status_t status = LINEARS_OK;
    size_t sweep = 0;
    size_t p;
    size_t q;
    size_t row;
    for (;;) {
        D_SCALAR largest = D_ZERO;
        D_SCALAR cutoff;
        int rotated = 0;
        for (p = 0; p < columns; ++p) {
            norms[p] = D_LOCAL(norm)(b + p, rows, columns, &status);
            if (status != LINEARS_OK) {
                return status;
            }
            if (D_CMP(norms[p], largest) > 0) {
                largest = norms[p];
            }
        }
        cutoff = D_MUL(largest, tolerance);
        if (status != LINEARS_OK) {
            return status;
        }
        for (p = 0; p < columns; ++p) {
            if (D_CMP(norms[p], cutoff) <= 0) {
                norms[p] = D_ZERO;
                for (row = 0; row < rows; ++row) {
                    b[row * columns + p] = D_ZERO;
                }
            }
        }
        for (p = 0; p < columns; ++p) {
            for (q = p + 1; q < columns; ++q) {
                D_SCALAR correlation;
                D_SCALAR magnitude;
                D_SCALAR cosine;
                D_SCALAR sine;
                if (D_CMP(norms[p], D_ZERO) == 0 ||
                    D_CMP(norms[q], D_ZERO) == 0) {
                    continue;
                }
                status = D_LOCAL(svd_correlation)(b, rows, columns, p, q,
                                                   norms[p], norms[q],
                                                   &correlation);
                if (status != LINEARS_OK) {
                    return status;
                }
                magnitude = D_ABS(correlation);
                if (D_CMP(magnitude, tolerance) <= 0) {
                    continue;
                }
                if (sweep == max_sweeps) {
                    return LINEARS_ERROR_NO_CONVERGENCE;
                }
                status = D_LOCAL(svd_rotation)(norms[p], norms[q], correlation,
                                               &cosine, &sine);
                if (status != LINEARS_OK) {
                    return status;
                }
                status = D_LOCAL(svd_rotate_columns)(b, rows, columns, p, q,
                                                     cosine, sine);
                if (status != LINEARS_OK) {
                    return status;
                }
                status = D_LOCAL(svd_rotate_columns)(w, columns, columns, p, q,
                                                     cosine, sine);
                if (status != LINEARS_OK) {
                    return status;
                }
                norms[p] = D_LOCAL(norm)(b + p, rows, columns, &status);
                norms[q] = D_LOCAL(norm)(b + q, rows, columns, &status);
                if (status != LINEARS_OK) {
                    return status;
                }
                rotated = 1;
            }
        }
        if (!rotated) {
            break;
        }
        ++sweep;
    }
    /* Selection sort keeps both singular-vector factors paired with s. */
    for (p = 0; p < columns; ++p) {
        size_t largest = p;
        for (q = p + 1; q < columns; ++q) {
            if (D_CMP(norms[q], norms[largest]) > 0) {
                largest = q;
            }
        }
        if (largest != p) {
            D_SCALAR temporary = norms[p];
            norms[p] = norms[largest];
            norms[largest] = temporary;
            for (row = 0; row < rows; ++row) {
                temporary = b[row * columns + p];
                b[row * columns + p] = b[row * columns + largest];
                b[row * columns + largest] = temporary;
            }
            for (row = 0; row < columns; ++row) {
                temporary = w[row * columns + p];
                w[row * columns + p] = w[row * columns + largest];
                w[row * columns + largest] = temporary;
            }
        }
    }
    for (p = 0; p < columns; ++p) {
        if (D_CMP(norms[p], D_ZERO) == 0) {
            status = D_LOCAL(svd_complete_column)(b, rows, columns, p);
        } else {
            for (row = 0; row < rows; ++row) {
                b[row * columns + p] = D_DIV(b[row * columns + p], norms[p]);
            }
        }
        if (status != LINEARS_OK) {
            return status;
        }
    }
    return LINEARS_OK;
}

linears_status_t D_NAME(svd)(const linears_matrix_t *a,
                              linears_matrix_t *u,
                              linears_matrix_t *singular_values,
                              linears_matrix_t *v, size_t max_sweeps,
                              D_WORKSPACE workspace)
{
    const linears_matrix_t *matrices[4];
    linears_status_t status = LINEARS_OK;
    D_SCALAR *b;
    D_SCALAR *w;
    D_SCALAR *norms;
    D_SCALAR *left;
    D_SCALAR *right;
    D_SCALAR scale = D_ZERO;
    D_SCALAR tolerance;
    D_SCALAR epsilon;
    D_SCALAR eight;
    size_t m;
    size_t n;
    size_t rows;
    size_t columns;
    size_t required;
    size_t row;
    size_t column;
    size_t index;
    int u_exponent;
    int s_exponent;
    int v_exponent;
    matrices[0] = a;
    matrices[1] = u;
    matrices[2] = singular_values;
    matrices[3] = v;
    status = D_LOCAL(descriptors)(matrices, 4);
    if (status != LINEARS_OK) {
        return status;
    }
    m = a->rows;
    n = a->cols;
    columns = m < n ? m : n;
    rows = m < n ? n : m;
    if (u->rows != m || u->cols != columns ||
        singular_values->rows != columns || singular_values->cols != 1 ||
        v->rows != n || v->cols != columns) {
        return LINEARS_ERROR_DIMENSION;
    }
    required = LINEARS_SVD_WORKSPACE(m, n);
    status = D_LOCAL(validate)(matrices, 4, 1, workspace, required);
    if (status != LINEARS_OK) {
        return status;
    }
    if (columns == 0) {
        u->exponent = 0;
        singular_values->exponent = 0;
        v->exponent = 0;
        return LINEARS_OK;
    }
    b = workspace.data;
    w = b + m * n;
    norms = w + columns * columns;
    for (row = 0; row < rows; ++row) {
        for (column = 0; column < columns; ++column) {
            size_t input_index = m < n ? column * n + row : row * n + column;
            D_SCALAR value = D_LOCAL(load)(a, input_index, &status);
            D_SCALAR magnitude = D_ABS(value);
            b[row * columns + column] = value;
            if (D_CMP(magnitude, scale) > 0) {
                scale = magnitude;
            }
        }
    }
    if (status != LINEARS_OK) {
        return status;
    }
    /* Keep every Jacobi intermediate near unit scale, including q exponents. */
    if (D_CMP(scale, D_ZERO) != 0) {
        for (index = 0; index < m * n; ++index) {
            b[index] = D_DIV(b[index], scale);
        }
    }
    for (row = 0; row < columns; ++row) {
        for (column = 0; column < columns; ++column) {
            w[row * columns + column] = row == column ? D_ONE : D_ZERO;
        }
    }
    tolerance = D_CONST((int64_t)rows);
    epsilon = D_EPSILON;
    tolerance = D_MUL(tolerance, epsilon);
    eight = D_CONST(8);
    tolerance = D_MUL(eight, tolerance);
    if (status != LINEARS_OK) {
        return status;
    }
    if (D_CMP(tolerance, D_ONE) >= 0) {
        return LINEARS_ERROR_NO_CONVERGENCE;
    }
    status = D_LOCAL(svd_factor)(b, w, norms, rows, columns, max_sweeps, tolerance);
    if (status != LINEARS_OK) {
        return status;
    }
    for (column = 0; column < columns; ++column) {
        norms[column] = D_MUL(norms[column], scale);
    }
    if (status != LINEARS_OK) {
        return status;
    }
    left = m < n ? w : b;
    right = m < n ? b : w;
    /* Preflight every independent output block before writing any output. */
    status = D_LOCAL(block_exponent)(left, m * columns, &u_exponent);
    if (status != LINEARS_OK) {
        return status;
    }
    status = D_LOCAL(block_exponent)(norms, columns, &s_exponent);
    if (status != LINEARS_OK) {
        return status;
    }
    status = D_LOCAL(block_exponent)(right, n * columns, &v_exponent);
    if (status != LINEARS_OK) {
        return status;
    }
    for (index = 0; index < m * columns; ++index) {
        D_LOCAL(store)(u, index, left[index], u_exponent);
    }
    for (index = 0; index < columns; ++index) {
        D_LOCAL(store)(singular_values, index, norms[index], s_exponent);
    }
    for (index = 0; index < n * columns; ++index) {
        D_LOCAL(store)(v, index, right[index], v_exponent);
    }
    u->exponent = u_exponent;
    singular_values->exponent = s_exponent;
    v->exponent = v_exponent;
    return LINEARS_OK;
}
