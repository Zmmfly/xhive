/* Private native-type instantiation, included for float and double. Keeping the
 * full load/compute/store path in FLOAT_TYPE avoids software double conversions
 * on Cortex-M4 f32 calls. Matrix validation and output commit remain in basic.c. */
#define FLOAT_JOIN_INNER(a, b) a##b
#define FLOAT_JOIN(a, b) FLOAT_JOIN_INNER(a, b)
#define FLOAT_NAME(name) FLOAT_JOIN(name, FLOAT_SUFFIX)

static linears_status_t FLOAT_NAME(evaluate_)(matrix_op operation,
                                            const linears_matrix_t *a, const linears_matrix_t *b,
                                            linears_matrix_t *out, FLOAT_TYPE alpha, FLOAT_TYPE beta,
                                            linears_workspace_unit_t *workspace) {
    const FLOAT_TYPE *adata = (const FLOAT_TYPE *)a->data;
    const FLOAT_TYPE *bdata = b == NULL ? NULL : (const FLOAT_TYPE *)b->data;
    for (size_t row = 0; row < out->rows; ++row) {
        for (size_t col = 0; col < out->cols; ++col) {
            size_t i = row * out->cols + col;
            FLOAT_TYPE result;
            if (operation == MUL || operation == MATVEC) {
                result = 0;
                for (size_t k = 0; k < a->cols; ++k) {
                    FLOAT_TYPE product = adata[row * a->cols + k] * bdata[k * b->cols + col];
                    result += product;
                    if (!isfinite(result)) return LINEARS_ERROR_RANGE;
                }
            } else if (operation == TRANSPOSE) {
                result = adata[col * a->cols + row];
            } else {
                FLOAT_TYPE x = adata[i];
                FLOAT_TYPE y = bdata == NULL ? 0 : bdata[i];
                switch (operation) {
                    case ADD: result = x + y; break;
                    case SUB: result = x - y; break;
                    case HADAMARD: result = x * y; break;
                    case SCALE: result = alpha * x; break;
                    default: {
                        FLOAT_TYPE left = alpha * x, right = beta * y;
                        result = left + right;
                        break;
                    }
                }
            }
            if (!isfinite(result)) return LINEARS_ERROR_RANGE;
            workspace[i].FLOAT_MEMBER = result;
        }
    }
    return LINEARS_OK;
}

static linears_status_t FLOAT_NAME(reduce_)(reduction_op operation,
                                          const linears_matrix_t *a, const linears_matrix_t *b,
                                          size_t count, FLOAT_TYPE *out) {
    const FLOAT_TYPE *adata = (const FLOAT_TYPE *)a->data;
    const FLOAT_TYPE *bdata = b == NULL ? NULL : (const FLOAT_TYPE *)b->data;
    FLOAT_TYPE sum = 0, scale = 0;
    for (size_t i = 0; i < count; ++i) {
        size_t index = operation == TRACE ? i * a->cols + i : i;
        FLOAT_TYPE x = adata[index];
        if (operation == NORM) {
            /* Scaled sum of squares: BLAS xNRM2 / LAPACK xLASSQ recurrence. */
            x = FLOAT_ABS(x);
            if (x == 0) continue;
            if (scale < x) {
                FLOAT_TYPE ratio = scale / x;
                FLOAT_TYPE squared = ratio * ratio;
                sum = 1 + sum * squared;
                scale = x;
            } else {
                FLOAT_TYPE ratio = x / scale;
                sum += ratio * ratio;
            }
        } else {
            FLOAT_TYPE term = operation == DOT ? x * bdata[i] : x;
            sum += term;
        }
        if (!isfinite(sum)) return LINEARS_ERROR_RANGE;
    }
    FLOAT_TYPE result = operation == NORM ? scale * FLOAT_SQRT(sum) : sum;
    if (!isfinite(result)) return LINEARS_ERROR_RANGE;
    *out = result;
    return LINEARS_OK;
}

#undef FLOAT_NAME
#undef FLOAT_JOIN
#undef FLOAT_JOIN_INNER
#undef FLOAT_TYPE
#undef FLOAT_SUFFIX
#undef FLOAT_MEMBER
#undef FLOAT_ABS
#undef FLOAT_SQRT
