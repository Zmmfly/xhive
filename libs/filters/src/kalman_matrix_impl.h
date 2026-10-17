/* Repeated typed implementation; included only by kalman_matrix.c. */
#define K_JOIN_INNER(a, b) a##b
#define K_JOIN(a, b) K_JOIN_INNER(a, b)
#define K_NAMED(base, name) K_JOIN(K_JOIN(base, K_TAG), name)
#define K_API(name) K_NAMED(filters_kalman_matrix_, K_JOIN(_, name))
#define K_LOCAL(name) K_NAMED(kalman_matrix_, K_JOIN(_, name))
#define K_LINEAR(name) K_JOIN(K_JOIN(linears_, name), K_JOIN(_, K_TAG))
#define K_TRY(expression) do { \
    status = (expression); \
    if (status != LINEARS_OK) return status; \
} while (0)
#define K_MUL(a, b, out) K_LINEAR(mul)((a), (b), (out), workspace.basic, workspace.basic_bytes)
#define K_TRANSPOSE(a, out) K_LINEAR(transpose)((a), (out), workspace.basic, workspace.basic_bytes)
#define K_SUM(a, b, out) K_LINEAR(axpby)(K_ONE, (a), K_ONE, (b), (out), workspace.basic, workspace.basic_bytes)
#define K_SUB(a, b, out) K_LINEAR(axpby)(K_ONE, (a), K_MINUS_ONE, (b), (out), workspace.basic, workspace.basic_bytes)
#define K_AVERAGE(a, b, out) K_LINEAR(axpby)(K_HALF, (a), K_HALF, (b), (out), workspace.basic, workspace.basic_bytes)

static linears_status_t K_LOCAL(shape)(const linears_matrix_t *matrix,
                                      size_t rows, size_t cols,
                                      kalman_range_t *data, kalman_range_t *descriptor)
{
    linears_status_t status;
    size_t count, bytes;
    K_TRY(kalman_range(matrix, sizeof(*matrix), K_ALIGNOF(linears_matrix_t), descriptor));
    if (matrix->rows == 0 || matrix->cols == 0 ||
        (rows != 0 && matrix->rows != rows) || matrix->cols != cols)
        return LINEARS_ERROR_DIMENSION;
    count = LINEARS_SIZE_MUL(matrix->rows, matrix->cols);
    bytes = LINEARS_SIZE_MUL(count, sizeof(K_RAW));
    if (count == SIZE_MAX || bytes == SIZE_MAX) return LINEARS_ERROR_SIZE;
    return kalman_range(matrix->data, bytes, K_ALIGNOF(K_RAW), data);
}

static linears_status_t K_LOCAL(values)(const linears_matrix_t *matrix)
{
    linears_status_t status;
    size_t i, count = matrix->rows * matrix->cols;
    const K_RAW *values = (const K_RAW *)matrix->data;
    for (i = 0; i < count; ++i) K_TRY(K_VALUE_CHECK(values[i], matrix->exponent));
    return LINEARS_OK;
}

static linears_status_t K_LOCAL(covariance)(const linears_matrix_t *matrix,
                                          int check_diagonal)
{
    size_t i, j, n = matrix->rows;
    const K_RAW *values = (const K_RAW *)matrix->data;
    for (i = 0; i < n; ++i) {
        if (check_diagonal && values[i * n + i] < 0) return LINEARS_ERROR_DOMAIN;
        for (j = 0; j < i; ++j)
            if (values[i * n + j] != values[j * n + i]) return LINEARS_ERROR_DOMAIN;
    }
    return LINEARS_OK;
}

static linears_status_t K_LOCAL(state)(const filters_kalman_matrix_t *state,
                                      kalman_boundary_t *boundary)
{
    linears_status_t status;
    size_t n, nn;
    kalman_range_t descriptor;
    K_TRY(kalman_range(state, sizeof(*state), K_ALIGNOF(filters_kalman_matrix_t),
                       &boundary->state[0]));
    n = state->estimate.rows;
    nn = LINEARS_SIZE_MUL(n, n);
    if (nn == SIZE_MAX) return LINEARS_ERROR_SIZE;
    if (n == 0 || state->estimate.cols != 1 || state->covariance.rows != n ||
        state->covariance.cols != n) return LINEARS_ERROR_DIMENSION;
    if (state->estimate_capacity < n || state->covariance_capacity < nn)
        return LINEARS_ERROR_WORKSPACE;
    K_TRY(K_LOCAL(shape)(&state->estimate, n, 1, &boundary->state[1], &descriptor));
    K_TRY(K_LOCAL(shape)(&state->covariance, n, n, &boundary->state[2], &descriptor));
    if (kalman_overlap(boundary->state[0], boundary->state[1]) ||
        kalman_overlap(boundary->state[0], boundary->state[2]) ||
        kalman_overlap(boundary->state[1], boundary->state[2]))
        return LINEARS_ERROR_ALIAS;
    return LINEARS_OK;
}

static linears_status_t K_LOCAL(workspace)(K_WORKSPACE workspace,
                                          size_t raw_count, size_t basic_bytes,
                                          size_t decomposition_count,
                                          const kalman_boundary_t *boundary)
{
    size_t raw_bytes = LINEARS_SIZE_MUL(raw_count, sizeof(K_RAW));
    size_t decomposition_bytes = LINEARS_SIZE_MUL(decomposition_count, sizeof(K_DECOMP_RAW));
    if (raw_count == SIZE_MAX || raw_bytes == SIZE_MAX || basic_bytes == SIZE_MAX ||
        decomposition_count == SIZE_MAX || decomposition_bytes == SIZE_MAX)
        return LINEARS_ERROR_SIZE;
    if (workspace.capacity < raw_count || workspace.basic_bytes < basic_bytes ||
        (decomposition_count != 0 && workspace.decomposition.capacity < decomposition_count))
        return LINEARS_ERROR_WORKSPACE;
    return kalman_workspace_alias(boundary, workspace.data, raw_bytes, K_ALIGNOF(K_RAW),
        workspace.basic, basic_bytes,
        decomposition_count != 0 ? workspace.decomposition.data : NULL,
        decomposition_bytes, K_ALIGNOF(K_DECOMP_RAW));
}

static linears_matrix_t K_LOCAL(take)(K_RAW **cursor, size_t rows, size_t cols)
{
    linears_matrix_t result = {*cursor, rows, cols, 0};
    *cursor += rows * cols;
    return result;
}

static void K_LOCAL(commit)(filters_kalman_matrix_t *state,
                           const linears_matrix_t *estimate,
                           const linears_matrix_t *covariance)
{
    size_t n = state->estimate.rows;
    if (state->estimate.data != estimate->data)
        memcpy(state->estimate.data, estimate->data, n * sizeof(K_RAW));
    if (state->covariance.data != covariance->data)
        memcpy(state->covariance.data, covariance->data, n * n * sizeof(K_RAW));
    state->estimate.exponent = K_OUTPUT_EXPONENT(estimate->exponent);
    state->covariance.exponent = K_OUTPUT_EXPONENT(covariance->exponent);
}

linears_status_t K_API(reset)(filters_kalman_matrix_t *state,
                              const linears_matrix_t *estimate,
                              const linears_matrix_t *covariance)
{
    linears_status_t status;
    kalman_boundary_t boundary;
    size_t n;
    K_TRY(K_LOCAL(state)(state, &boundary));
    n = state->estimate.rows;
    boundary.count = 2;
    K_TRY(K_LOCAL(shape)(estimate, n, 1, &boundary.input_data[0], &boundary.input_desc[0]));
    K_TRY(K_LOCAL(shape)(covariance, n, n, &boundary.input_data[1], &boundary.input_desc[1]));
    K_TRY(kalman_input_alias(state, &boundary, 1));
    K_TRY(K_LOCAL(values)(estimate));
    K_TRY(K_LOCAL(values)(covariance));
    K_TRY(K_LOCAL(covariance)(covariance, 1));
    K_LOCAL(commit)(state, estimate, covariance);
    return LINEARS_OK;
}

linears_status_t K_API(predict)(filters_kalman_matrix_t *state,
                                const linears_matrix_t *transition,
                                const linears_matrix_t *process_noise,
                                K_WORKSPACE workspace)
{
    linears_status_t status;
    kalman_boundary_t boundary;
    linears_matrix_t nx, a, b, c, d;
    K_RAW *cursor;
    size_t n, raw_count, basic_bytes;
    K_TRY(K_LOCAL(state)(state, &boundary));
    n = state->estimate.rows;
    boundary.count = 2;
    K_TRY(K_LOCAL(shape)(transition, n, n, &boundary.input_data[0], &boundary.input_desc[0]));
    K_TRY(K_LOCAL(shape)(process_noise, n, n, &boundary.input_data[1], &boundary.input_desc[1]));
    K_TRY(kalman_input_alias(state, &boundary, 0));
    raw_count = FILTERS_KALMAN_PREDICT_STORAGE(n);
    basic_bytes = FILTERS_KALMAN_PREDICT_BASIC_BYTES(n);
    K_TRY(K_LOCAL(workspace)(workspace, raw_count, basic_bytes, 0, &boundary));
    K_TRY(K_LOCAL(values)(&state->estimate));
    K_TRY(K_LOCAL(values)(&state->covariance));
    K_TRY(K_LOCAL(values)(transition));
    K_TRY(K_LOCAL(values)(process_noise));
    K_TRY(K_LOCAL(covariance)(&state->covariance, 1));
    K_TRY(K_LOCAL(covariance)(process_noise, 1));
    /* Pass only the used basic prefix; unused supplied capacity stays untouched. */
    workspace.basic_bytes = basic_bytes;
    cursor = workspace.data;
    nx = K_LOCAL(take)(&cursor, n, 1);
    a = K_LOCAL(take)(&cursor, n, n);
    b = K_LOCAL(take)(&cursor, n, n);
    c = K_LOCAL(take)(&cursor, n, n);
    d = K_LOCAL(take)(&cursor, n, n);
    K_TRY(K_MUL(transition, &state->estimate, &nx));
    K_TRY(K_TRANSPOSE(transition, &a));
    K_TRY(K_MUL(transition, &state->covariance, &b));
    K_TRY(K_MUL(&b, &a, &c));
    K_TRY(K_SUM(&c, process_noise, &d));
    K_TRY(K_TRANSPOSE(&d, &a));
    K_TRY(K_AVERAGE(&d, &a, &b));
    K_LOCAL(commit)(state, &nx, &b);
    return LINEARS_OK;
}

linears_status_t K_API(update)(filters_kalman_matrix_t *state,
                               const linears_matrix_t *measurement,
                               const linears_matrix_t *observation,
                               const linears_matrix_t *measurement_noise,
                               K_WORKSPACE workspace)
{
    linears_status_t status;
    kalman_boundary_t boundary;
    linears_matrix_t a, b, c, d, ht, hp, kt, k, s0, s1, nx, dx, residual, hx;
    K_RAW *cursor;
    size_t n, m, i, j, raw_count, basic_bytes, decomposition_count;
    K_TRY(K_LOCAL(state)(state, &boundary));
    n = state->estimate.rows;
    boundary.count = 3;
    K_TRY(K_LOCAL(shape)(measurement, 0, 1, &boundary.input_data[0], &boundary.input_desc[0]));
    m = measurement->rows;
    K_TRY(K_LOCAL(shape)(observation, m, n, &boundary.input_data[1], &boundary.input_desc[1]));
    K_TRY(K_LOCAL(shape)(measurement_noise, m, m, &boundary.input_data[2], &boundary.input_desc[2]));
    K_TRY(kalman_input_alias(state, &boundary, 0));
    raw_count = FILTERS_KALMAN_UPDATE_STORAGE(n, m);
    basic_bytes = FILTERS_KALMAN_UPDATE_BASIC_BYTES(n, m);
    decomposition_count = FILTERS_KALMAN_UPDATE_DECOMP_STORAGE(n, m);
    K_TRY(K_LOCAL(workspace)(workspace, raw_count, basic_bytes, decomposition_count, &boundary));
    K_TRY(K_LOCAL(values)(&state->estimate));
    K_TRY(K_LOCAL(values)(&state->covariance));
    K_TRY(K_LOCAL(values)(measurement));
    K_TRY(K_LOCAL(values)(observation));
    K_TRY(K_LOCAL(values)(measurement_noise));
    K_TRY(K_LOCAL(covariance)(&state->covariance, 1));
    K_TRY(K_LOCAL(covariance)(measurement_noise, 0));
    workspace.basic_bytes = basic_bytes;
    workspace.decomposition.capacity = decomposition_count;
    cursor = workspace.data;
    a = K_LOCAL(take)(&cursor, n, n);
    b = K_LOCAL(take)(&cursor, n, n);
    c = K_LOCAL(take)(&cursor, n, n);
    d = K_LOCAL(take)(&cursor, n, n);
    ht = K_LOCAL(take)(&cursor, n, m);
    hp = K_LOCAL(take)(&cursor, m, n);
    kt = K_LOCAL(take)(&cursor, m, n);
    k = K_LOCAL(take)(&cursor, n, m);
    s0 = K_LOCAL(take)(&cursor, m, m);
    s1 = K_LOCAL(take)(&cursor, m, m);
    nx = K_LOCAL(take)(&cursor, n, 1);
    dx = K_LOCAL(take)(&cursor, n, 1);
    residual = K_LOCAL(take)(&cursor, m, 1);
    hx = K_LOCAL(take)(&cursor, m, 1);

    /* Validate R itself: S can be SPD even when a supplied R is not. */
    K_TRY(K_LINEAR(cholesky)(measurement_noise, &s0, workspace.decomposition));
    K_TRY(K_TRANSPOSE(observation, &ht));
    K_TRY(K_MUL(observation, &state->covariance, &hp));
    K_TRY(K_MUL(&hp, &ht, &s0));
    K_TRY(K_SUM(&s0, measurement_noise, &s1));
    K_TRY(K_TRANSPOSE(&s1, &s0));
    K_TRY(K_AVERAGE(&s1, &s0, &s1));
    K_TRY(K_LINEAR(cholesky)(&s1, &s0, workspace.decomposition));
    /* S*K^T=H*P. The existing pivoted solve avoids an explicit inverse. */
    K_TRY(K_LINEAR(solve)(&s1, &hp, &kt, workspace.decomposition));
    K_TRY(K_TRANSPOSE(&kt, &k));
    K_TRY(K_MUL(observation, &state->estimate, &hx));
    K_TRY(K_SUB(measurement, &hx, &residual));
    K_TRY(K_MUL(&k, &residual, &dx));
    K_TRY(K_SUM(&state->estimate, &dx, &nx));

    K_TRY(K_MUL(&k, observation, &a));
    for (i = 0; i < n; ++i)
        for (j = 0; j < n; ++j)
            ((K_RAW *)b.data)[i * n + j] = i == j ? K_RAW_ONE : 0;
    b.exponent = K_IDENTITY_EXPONENT;
    K_TRY(K_SUB(&b, &a, &c)); /* c = I-KH. */
    K_TRY(K_TRANSPOSE(&c, &a));
    K_TRY(K_MUL(&c, &state->covariance, &b));
    K_TRY(K_MUL(&b, &a, &d));
    K_TRY(K_MUL(&k, measurement_noise, &ht));
    K_TRY(K_MUL(&ht, &kt, &a));
    K_TRY(K_SUM(&d, &a, &b));
    K_TRY(K_TRANSPOSE(&b, &c));
    K_TRY(K_AVERAGE(&b, &c, &a));
    K_LOCAL(commit)(state, &nx, &a);
    return LINEARS_OK;
}

#undef K_AVERAGE
#undef K_SUB
#undef K_SUM
#undef K_TRANSPOSE
#undef K_MUL
#undef K_TRY
#undef K_LINEAR
#undef K_LOCAL
#undef K_API
#undef K_NAMED
#undef K_JOIN
#undef K_JOIN_INNER
#undef K_TAG
#undef K_RAW
#undef K_DECOMP_RAW
#undef K_WORKSPACE
#undef K_VALUE_CHECK
#undef K_OUTPUT_EXPONENT
#undef K_ONE
#undef K_MINUS_ONE
#undef K_HALF
#undef K_RAW_ONE
#undef K_IDENTITY_EXPONENT
