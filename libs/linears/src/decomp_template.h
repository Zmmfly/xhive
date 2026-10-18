/**
 * @file decomp_template.h
 * @brief Private scalar adapter, included once per D_FORMAT by each algorithm.
 *
 * D_FORMAT is 1=f32, 2=f64, 3=q15, 4=q31, 5=wide fixed working matrices.
 * Include decomp_cleanup.h afterwards.
 * The arithmetic macros use a local linears_status_t status as a sticky error.
 * Keep calls with status side effects in separate expressions. Helper inputs
 * are already validated unless the helper itself is the documented boundary.
 */
#define D_JOIN_INNER(a, b) a##b
#define D_JOIN(a, b) D_JOIN_INNER(a, b)
#define D_NAME_INNER(name, suffix) linears_##name##_##suffix
#define D_NAME_EXPAND(name, suffix) D_NAME_INNER(name, suffix)
#define D_NAME(name) D_NAME_EXPAND(name, D_SUFFIX)
#define D_LOCAL_INNER(name, suffix) linears_decomp_##name##_##suffix
#define D_LOCAL_EXPAND(name, suffix) D_LOCAL_INNER(name, suffix)
#define D_LOCAL(name) D_LOCAL_EXPAND(name, D_SUFFIX)
#define D_WORKING (D_FORMAT == 5)

#if D_FORMAT == 1
#define D_SUFFIX f32
#define D_SCALAR float
#define D_RAW float
#define D_WORKSPACE linears_f32_workspace_t
#define D_FIXED 0
#define D_ZERO 0.0f
#define D_ONE 1.0f
#define D_EPSILON FLT_EPSILON
#define D_MATH_SQRT sqrtf
#define D_MATH_ABS fabsf
#define D_MATH_SCALE scalbnf
#elif D_FORMAT == 2
#define D_SUFFIX f64
#define D_SCALAR double
#define D_RAW double
#define D_WORKSPACE linears_f64_workspace_t
#define D_FIXED 0
#define D_ZERO 0.0
#define D_ONE 1.0
#define D_EPSILON DBL_EPSILON
#define D_MATH_SQRT sqrt
#define D_MATH_ABS fabs
#define D_MATH_SCALE scalbn
#elif D_FORMAT == 3 || D_FORMAT == 4 || D_FORMAT == 5
#if D_FORMAT == 3
#define D_SUFFIX q15
#define D_RAW int16_t
#define D_WORKSPACE linears_q15_workspace_t
#define D_IMPORT linears_fixed_from_q15
#define D_QUANTIZE linears_fixed_to_q15
#elif D_FORMAT == 4
#define D_SUFFIX q31
#define D_RAW int32_t
#define D_WORKSPACE linears_q31_workspace_t
#define D_IMPORT linears_fixed_from_q31
#define D_QUANTIZE linears_fixed_to_q31
#else
#define D_SUFFIX fixed
#define D_RAW linears_fixed_t
#define D_WORKSPACE linears_fixed_workspace_t
#define D_IMPORT(value) (value)
#endif
#define D_SCALAR linears_fixed_t
#define D_FIXED 1
#define D_ZERO INT64_C(0)
#define D_ONE LINEARS_FIXED_ONE
#define D_EPSILON INT64_C(1)
#else
#error Unsupported D_FORMAT
#endif

#if D_FIXED
/** @brief Adapt a checked fixed binary operation to the template's sticky status. */
#define D_BINARY_ADAPTER(operation) \
    static inline D_SCALAR D_LOCAL(operation)(D_SCALAR a, D_SCALAR b, linears_status_t *status) \
    { \
        D_SCALAR result = D_ZERO; \
        linears_status_t current; \
        if (*status != LINEARS_OK) return result; \
        current = linears_fixed_##operation(a, b, &result); \
        if (current != LINEARS_OK) *status = current; \
        return result; \
    }
D_BINARY_ADAPTER(add)
D_BINARY_ADAPTER(sub)
D_BINARY_ADAPTER(mul)
D_BINARY_ADAPTER(div)
#undef D_BINARY_ADAPTER
/** @brief Adapt a checked fixed unary operation to the template's sticky status. */
#define D_UNARY_ADAPTER(operation) \
    static inline D_SCALAR D_LOCAL(operation)(D_SCALAR value, linears_status_t *status) \
    { \
        D_SCALAR result = D_ZERO; \
        linears_status_t current; \
        if (*status != LINEARS_OK) return result; \
        current = linears_fixed_##operation(value, &result); \
        if (current != LINEARS_OK) *status = current; \
        return result; \
    }
D_UNARY_ADAPTER(sqrt)
D_UNARY_ADAPTER(neg)
D_UNARY_ADAPTER(abs)
#undef D_UNARY_ADAPTER
/** @brief Construct an integer constant; status is sticky and caller-owned. */
static inline D_SCALAR D_LOCAL(constant)(int64_t value, linears_status_t *status)
{
    D_SCALAR result = D_ZERO;
    if (*status == LINEARS_OK) *status = linears_fixed_from_int(value, &result);
    return result;
}
#define D_CMP(a, b) linears_fixed_compare((a), (b))
#else
/** @brief Reject nonfinite native arithmetic and retain the first failure. */
static inline D_SCALAR D_LOCAL(finite)(D_SCALAR value, linears_status_t *status)
{
    if (!isfinite(value) && *status == LINEARS_OK) *status = LINEARS_ERROR_RANGE;
    return value;
}
/** @brief Native binary operation; no widening of the selected floating type. */
#define D_BINARY_ADAPTER(operation, expression) \
    static inline D_SCALAR D_LOCAL(operation)(D_SCALAR a, D_SCALAR b, linears_status_t *status) \
    { \
        if (*status != LINEARS_OK) return D_ZERO; \
        return D_LOCAL(finite)((expression), status); \
    }
D_BINARY_ADAPTER(add, a + b)
D_BINARY_ADAPTER(sub, a - b)
D_BINARY_ADAPTER(mul, a * b)
D_BINARY_ADAPTER(div, a / b)
#undef D_BINARY_ADAPTER
/** @brief Native square root with finite-result validation. */
static inline D_SCALAR D_LOCAL(sqrt)(D_SCALAR value, linears_status_t *status)
{
    return *status == LINEARS_OK ? D_LOCAL(finite)(D_MATH_SQRT(value), status) : D_ZERO;
}
/** @brief Negate a finite native operand without widening. */
static inline D_SCALAR D_LOCAL(neg)(D_SCALAR value, linears_status_t *status)
{
    (void)status;
    return -value;
}
/** @brief Absolute value of a finite native operand. */
static inline D_SCALAR D_LOCAL(abs)(D_SCALAR value, linears_status_t *status)
{
    (void)status;
    return D_MATH_ABS(value);
}
/** @brief Convert the bounded integral algorithm constant to the native type. */
static inline D_SCALAR D_LOCAL(constant)(int64_t value, linears_status_t *status)
{
    (void)status;
    return (D_SCALAR)value;
}
#define D_CMP(a, b) (((a) > (b)) - ((a) < (b)))
#endif

#define D_ADD(a, b) D_LOCAL(add)((a), (b), &status)
#define D_SUB(a, b) D_LOCAL(sub)((a), (b), &status)
#define D_MUL(a, b) D_LOCAL(mul)((a), (b), &status)
#define D_DIV(a, b) D_LOCAL(div)((a), (b), &status)
#define D_SQRT(a) D_LOCAL(sqrt)((a), &status)
#define D_NEG(a) D_LOCAL(neg)((a), &status)
#define D_ABS(a) D_LOCAL(abs)((a), &status)
#define D_CONST(a) D_LOCAL(constant)((a), &status)

/** @brief Load an already range-checked matrix element, importing fixed storage. */
static inline D_SCALAR D_LOCAL(load)(const linears_matrix_t *matrix, size_t index,
                                    linears_status_t *status)
{
#if D_FIXED
    (void)status;
    return D_IMPORT(((const D_RAW *)matrix->data)[index]);
#else
    return D_LOCAL(finite)(((const D_RAW *)matrix->data)[index], status);
#endif
}

/**
 * @brief Validate descriptor addresses before any caller reads their fields.
 * @param[in] matrices Descriptor pointers to validate; array itself is internal.
 * @param[in] count Number of descriptor pointers, a constant per operation.
 * @return linears_status_t OK, ARGUMENT for NULL/alignment, SIZE for address overflow.
 */
static inline linears_status_t D_LOCAL(descriptors)(const linears_matrix_t *const *matrices,
                                                   size_t count)
{
    size_t i;
    for (i = 0; i < count; ++i) {
        linears_status_t status = linears_decomp_range(matrices[i], sizeof(*matrices[i]),
                                                      D_ALIGNOF(linears_matrix_t));
        if (status != LINEARS_OK) return status;
    }
    return LINEARS_OK;
}

/**
 * @brief Validate input values, sizes, used workspace and aliasing.
 * @pre descriptors() has already validated every descriptor before field access.
 */
static inline linears_status_t D_LOCAL(validate)(const linears_matrix_t *const *matrices,
    size_t count, size_t input_count, D_WORKSPACE workspace, size_t required)
{
    size_t i, j, workspace_bytes;
    linears_status_t status;
    if (required == SIZE_MAX) return LINEARS_ERROR_SIZE;
    workspace_bytes = LINEARS_SIZE_MUL(required, sizeof(D_SCALAR));
    if (workspace_bytes == SIZE_MAX) return LINEARS_ERROR_SIZE;
    if (workspace.capacity < required) return LINEARS_ERROR_WORKSPACE;
    status = linears_decomp_range(workspace.data, workspace_bytes, D_ALIGNOF(D_SCALAR));
    if (status != LINEARS_OK) return status == LINEARS_ERROR_ARGUMENT ? LINEARS_ERROR_WORKSPACE : status;
    for (i = 0; i < count; ++i) {
        const linears_matrix_t *matrix = matrices[i];
        size_t elements, bytes;
        elements = LINEARS_SIZE_MUL(matrix->rows, matrix->cols);
        bytes = LINEARS_SIZE_MUL(elements, sizeof(D_RAW));
        if (elements == SIZE_MAX || bytes == SIZE_MAX) return LINEARS_ERROR_SIZE;
        status = linears_decomp_range(matrix->data, bytes, D_ALIGNOF(D_RAW));
        if (status != LINEARS_OK) return status;
        if (linears_decomp_overlap(workspace.data, workspace_bytes, matrix, sizeof(*matrix)) ||
            linears_decomp_overlap(workspace.data, workspace_bytes, matrix->data, bytes))
            return LINEARS_ERROR_ALIAS;
    }
    for (i = 0; i < count; ++i) {
        size_t bytes = matrices[i]->rows * matrices[i]->cols * sizeof(D_RAW);
        for (j = 0; j < count; ++j) {
            size_t other_bytes = matrices[j]->rows * matrices[j]->cols * sizeof(D_RAW);
            if (linears_decomp_overlap(matrices[i]->data, bytes, matrices[j], sizeof(*matrices[j])))
                return LINEARS_ERROR_ALIAS;
            if (j <= i) continue;
            if (i < input_count && j < input_count) continue;
            if (linears_decomp_overlap(matrices[i], sizeof(*matrices[i]), matrices[j], sizeof(*matrices[j])) ||
                linears_decomp_overlap(matrices[i]->data, bytes, matrices[j]->data, other_bytes))
                return LINEARS_ERROR_ALIAS;
        }
    }
#if !D_FIXED
    status = LINEARS_OK;
    for (i = 0; i < input_count; ++i)
        for (j = 0; j < matrices[i]->rows * matrices[i]->cols; ++j) {
            (void)D_LOCAL(load)(matrices[i], j, &status);
            if (status != LINEARS_OK) return status;
        }
#endif
    return LINEARS_OK;
}

/** @brief Validate caller metadata against previously validated matrix/workspace ranges. */
static inline linears_status_t D_LOCAL(metadata)(const void *pointer, size_t bytes,
    size_t alignment, const linears_matrix_t *const *matrices, size_t count,
    D_WORKSPACE workspace, size_t required)
{
    size_t i;
    linears_status_t status = linears_decomp_range(pointer, bytes, alignment);
    if (status != LINEARS_OK) return status;
    if (linears_decomp_overlap(pointer, bytes, workspace.data, required * sizeof(D_SCALAR)))
        return LINEARS_ERROR_ALIAS;
    for (i = 0; i < count; ++i) {
        if (linears_decomp_overlap(pointer, bytes, matrices[i], sizeof(*matrices[i])) ||
            linears_decomp_overlap(pointer, bytes, matrices[i]->data,
                matrices[i]->rows * matrices[i]->cols * sizeof(D_RAW)))
            return LINEARS_ERROR_ALIAS;
    }
    return LINEARS_OK;
}

/** @brief Check exact symmetry of a validated n-by-n working array. */
static inline linears_status_t D_LOCAL(check_symmetric)(const D_SCALAR *data, size_t n)
{
    size_t i, j;
    for (i = 0; i < n; ++i)
        for (j = i + 1; j < n; ++j)
            if (D_CMP(data[i * n + j], data[j * n + i]) != 0)
                return LINEARS_ERROR_DIMENSION;
    return LINEARS_OK;
}

/**
 * @brief Preflight the actual signed extrema of a result without modifying output.
 * @param[in] minimum Smallest working value, including zero in the bound.
 * @param[in] maximum Largest working value, including zero in the bound.
 * @return linears_status_t OK, or RANGE outside the output representation.
 */
static inline linears_status_t D_LOCAL(output_bounds)(D_SCALAR minimum, D_SCALAR maximum)
{
#if D_WORKING
    (void)minimum;
    (void)maximum;
#elif D_FIXED
    if (minimum < -LINEARS_FIXED_ONE || maximum > LINEARS_FIXED_ONE)
        return LINEARS_ERROR_RANGE;
#else
    if (!isfinite(minimum) || !isfinite(maximum)) return LINEARS_ERROR_RANGE;
#endif
    return LINEARS_OK;
}

/** @brief Preflight a contiguous result block without changing its output matrix. */
static inline linears_status_t D_LOCAL(check_block)(const D_SCALAR *values, size_t count)
{
#if D_WORKING
    /* Every int64_t is representable; checked working arithmetic already ran. */
    (void)values;
    (void)count;
    return LINEARS_OK;
#else
    size_t i;
    D_SCALAR minimum = D_ZERO, maximum = D_ZERO;
    for (i = 0; i < count; ++i) {
        if (D_CMP(values[i], minimum) < 0) minimum = values[i];
        if (D_CMP(values[i], maximum) > 0) maximum = values[i];
    }
    return D_LOCAL(output_bounds)(minimum, maximum);
#endif
}

/** @brief Commit one value after signed-extrema output preflight has succeeded. */
static inline void D_LOCAL(store)(linears_matrix_t *matrix, size_t index, D_SCALAR value)
{
#if D_FIXED && !D_WORKING
    D_RAW raw = 0;
    (void)D_QUANTIZE(value, &raw);
    ((D_RAW *)matrix->data)[index] = raw;
#else
    ((D_RAW *)matrix->data)[index] = value;
#endif
}

/** @brief Commit a preflighted contiguous result in the unchanged fixed format. */
static inline void D_LOCAL(store_block)(linears_matrix_t *matrix,
                                       const D_SCALAR *values, size_t count)
{
    size_t i;
    for (i = 0; i < count; ++i) D_LOCAL(store)(matrix, i, values[i]);
}

#if D_FIXED
/**
 * @brief Compute value-a*b with one final F31 rounding, avoiding cancellation loss.
 * @pre Inputs are fixed-F31 values; status is caller-owned and sticky.
 */
static inline D_SCALAR D_LOCAL(sub_product)(D_SCALAR value, D_SCALAR a, D_SCALAR b,
                                           linears_status_t *status)
{
    linears_accum_t accumulator = {0, 0};
    D_SCALAR result = D_ZERO;
    if (*status == LINEARS_OK) *status = linears_fixed_mac(&accumulator, value, D_ONE);
    if (*status == LINEARS_OK) *status = linears_fixed_msu(&accumulator, a, b);
    if (*status == LINEARS_OK) *status = linears_accum_to_fixed(accumulator, &result);
    return result;
}

/**
 * @brief Sum fixed-F31 products at fixed F62 precision and round once.
 * @pre Valid bounded strided ranges; status is caller-owned and sticky.
 */
static inline D_SCALAR D_LOCAL(dot)(const D_SCALAR *left, size_t left_stride,
    const D_SCALAR *right, size_t right_stride, size_t count, linears_status_t *status)
{
    linears_accum_t accumulator = {0, 0};
    D_SCALAR result = D_ZERO;
    size_t i;
    for (i = 0; i < count && *status == LINEARS_OK; ++i)
        *status = linears_fixed_mac(&accumulator, left[i * left_stride], right[i * right_stride]);
    if (*status == LINEARS_OK) *status = linears_accum_to_fixed(accumulator, &result);
    return result;
}

/**
 * @brief Subtract a fixed-F31 dot product from value with one final rounding.
 * @pre Valid bounded strided ranges; status is caller-owned and sticky.
 */
static inline D_SCALAR D_LOCAL(dot_sub)(D_SCALAR value, const D_SCALAR *left,
    size_t left_stride, const D_SCALAR *right, size_t right_stride, size_t count,
    linears_status_t *status)
{
    linears_accum_t accumulator = {0, 0};
    D_SCALAR result = D_ZERO;
    size_t i;
    if (*status == LINEARS_OK) *status = linears_fixed_mac(&accumulator, value, D_ONE);
    for (i = 0; i < count && *status == LINEARS_OK; ++i)
        *status = linears_fixed_msu(&accumulator, left[i * left_stride], right[i * right_stride]);
    if (*status == LINEARS_OK) *status = linears_accum_to_fixed(accumulator, &result);
    return result;
}
#endif

/** @brief Euclidean norm with maximum-magnitude scaling; count/stride are bounded. */
static inline D_SCALAR D_LOCAL(norm)(const D_SCALAR *values, size_t count,
                                     size_t stride, linears_status_t *result_status)
{
    size_t i;
    D_SCALAR maximum = D_ZERO, sum = D_ZERO, root;
    linears_status_t status = *result_status;
#if D_FIXED
    linears_accum_t accumulator = {0, 0};
#endif
    for (i = 0; i < count; ++i) {
        D_SCALAR magnitude = D_ABS(values[i * stride]);
        if (D_CMP(magnitude, maximum) > 0) maximum = magnitude;
    }
    if (status != LINEARS_OK) { *result_status = status; return D_ZERO; }
    if (D_CMP(maximum, D_ZERO) == 0) return D_ZERO;
    for (i = 0; i < count; ++i) {
        D_SCALAR ratio = D_DIV(values[i * stride], maximum);
#if D_FIXED
        if (status == LINEARS_OK) status = linears_fixed_mac(&accumulator, ratio, ratio);
#else
        D_SCALAR square = D_MUL(ratio, ratio);
        sum = D_ADD(sum, square);
#endif
    }
#if D_FIXED
    if (status == LINEARS_OK) status = linears_accum_to_fixed(accumulator, &sum);
#endif
    root = D_SQRT(sum);
    root = D_MUL(maximum, root);
    *result_status = status;
    return root;
}
