/* Private raw-Q kernel instantiation. Q_BITS is a compile-time format (15/31),
 * never dynamic scaling metadata. The innermost product loops contain only
 * raw loads, exact integer products, and bounded integer accumulation. */
#define Q_JOIN_INNER(a, b) a##b
#define Q_JOIN(a, b) Q_JOIN_INNER(a, b)
#define Q_NAME(name) Q_JOIN(name, Q_SUFFIX)

static linears_accum_t Q_NAME(dot_raw_)(const Q_TYPE *a, size_t astride,
                                       const Q_TYPE *b, size_t bstride, size_t count) {
#if Q_BITS == 15
    /* Every product magnitude is <=2^30. All 32-bit size_t counts fit the
     * signed-64-bit bound, so neither guard nor fallback exists on that target.
     * Larger 64-bit-host counts retain every product bit in limbs. */
#if SIZE_MAX > UINT32_MAX
    if ((uint64_t)count <= (uint64_t)INT64_MAX / (UINT64_C(1) << 30)) {
#endif
        int64_t sum = 0;
        for (size_t i = 0; i < count; ++i)
            sum += (int64_t)a[i * astride] * b[i * bstride];
        return ln_shift_i64(sum, 32);
#if SIZE_MAX > UINT32_MAX
    }
#endif
#endif
#if Q_BITS == 31 || SIZE_MAX > UINT32_MAX
    linears_accum_t sum = {0, 0};
    for (size_t i = 0; i < count; ++i) {
        int64_t product = (int64_t)a[i * astride] * b[i * bstride];
        ln_add_product64(&sum, product);
    }
#if Q_BITS == 15
    sum = ln_shift_accum(sum, 32);
#endif
    return sum;
#endif
}

static linears_status_t Q_NAME(evaluate_)(matrix_op operation,
                                         const linears_matrix_t *a, const linears_matrix_t *b,
                                         linears_matrix_t *out, linears_fixed_t alpha,
                                         linears_fixed_t beta, linears_workspace_unit_t *workspace) {
    const Q_TYPE *adata = (const Q_TYPE *)a->data;
    const Q_TYPE *bdata = b == NULL ? NULL : (const Q_TYPE *)b->data;
    for (size_t row = 0; row < out->rows; ++row) {
        for (size_t col = 0; col < out->cols; ++col) {
            size_t i = row * out->cols + col;
            if (operation == TRANSPOSE) {
                workspace[i].Q_SUFFIX = adata[col * a->cols + row];
                continue;
            }
            linears_accum_t sum = {0, 0};
            if (operation == MUL || operation == MATVEC) {
                /* Do not form even a zero-offset pointer from NULL empty data. */
                if (a->cols != 0)
                    sum = Q_NAME(dot_raw_)(adata + row * a->cols, 1, bdata + col, b->cols, a->cols);
            } else {
                int64_t x = adata[i], y = bdata == NULL ? 0 : bdata[i];
                if (operation == ADD || operation == SUB) {
                    sum = ln_shift_i64(operation == ADD ? x + y : x - y, 62 - Q_BITS);
                } else if (operation == HADAMARD) {
                    int64_t product = x * y;
#if Q_BITS == 15
                    sum = ln_shift_i64(product, 32);
#else
                    sum.low = (uint64_t)product;
                    sum.high = product < 0 ? UINT64_MAX : 0;
#endif
                } else {
#if Q_BITS == 15
                    x *= INT64_C(65536);
                    y *= INT64_C(65536);
#endif
                    sum = ln_product(alpha, x);
                    if (operation == AXPBY)
                        sum = ln_add_unchecked(sum, ln_product(beta, y));
                    /* Two coefficient*signal products each have magnitude
                     * <=2^94; their exact sum cannot overflow signed 128 bits. */
                }
            }
            linears_status_t status = Q_NAME(linears_accum_to_)(sum, &workspace[i].Q_SUFFIX);
            if (status != LINEARS_OK) return status;
        }
    }
    return LINEARS_OK;
}

static linears_status_t Q_NAME(reduce_)(reduction_op operation,
                                       const linears_matrix_t *a, const linears_matrix_t *b,
                                       size_t count, Q_TYPE *out) {
    const Q_TYPE *adata = (const Q_TYPE *)a->data;
    linears_accum_t sum = {0, 0};
    if (operation == TRACE) {
        for (size_t i = 0; i < count; ++i)
            ln_add_product64(&sum, adata[i * a->cols + i]);
        sum = ln_shift_accum(sum, 62 - Q_BITS);
    } else {
        const Q_TYPE *bdata = operation == NORM ? adata : (const Q_TYPE *)b->data;
        sum = Q_NAME(dot_raw_)(adata, 1, bdata, 1, count);
    }
    if (operation == NORM) {
        /* A norm is representable only if its exact squared magnitude <=1.
         * This also proves the integer radicand fits in one uint64_t. */
        if (sum.high != 0 || sum.low > (UINT64_C(1) << 62)) return LINEARS_ERROR_RANGE;
#if Q_BITS == 15
        uint64_t raw = ln_sqrt_nearest64(sum.low >> 32);
#else
        uint64_t raw = ln_sqrt_nearest64(sum.low);
#endif
        if (raw == (UINT64_C(1) << Q_BITS)) --raw;
        *out = (Q_TYPE)raw;
        return LINEARS_OK;
    }
    return Q_NAME(linears_accum_to_)(sum, out);
}

#undef Q_NAME
#undef Q_JOIN
#undef Q_JOIN_INNER
#undef Q_TYPE
#undef Q_BITS
#undef Q_SUFFIX
