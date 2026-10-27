/* Q scratch helpers. Include with U_TAG, U_WORK, U_SIGNAL and U_WORKSPACE.
 * The two typed numeric arrays are separate objects; no type-punned arena. */
#define U_Q_JOIN_INNER(a, b) a##b
#define U_Q_JOIN(a, b) U_Q_JOIN_INNER(a, b)
#define U_Q_HELPER(name) U_Q_JOIN(U_Q_JOIN(ukf_work_, U_TAG), U_Q_JOIN(_, name))

static inline linears_matrix_t U_Q_HELPER(take)(U_WORK **cursor, size_t rows, size_t cols)
{
    linears_matrix_t result = {*cursor, rows, cols};
    *cursor += rows * cols;
    return result;
}

static inline linears_matrix_t U_Q_HELPER(take_signal)(U_SIGNAL **cursor, size_t rows, size_t cols)
{
    linears_matrix_t result = {*cursor, rows, cols};
    *cursor += rows * cols;
    return result;
}

static inline linears_matrix_t U_Q_HELPER(row)(const linears_matrix_t *matrix, size_t index)
{
    linears_matrix_t result = {(U_WORK *)matrix->data + index * matrix->cols, matrix->cols, 1};
    return result;
}

static inline linears_status_t U_Q_HELPER(workspace)(ukf_spans_t *spans, U_WORKSPACE workspace,
                                              size_t data_count, size_t basic_bytes,
                                              size_t decomposition_count, size_t signal_count)
{
    linears_status_t status;
    size_t data_bytes = LINEARS_SIZE_MUL(data_count, sizeof(U_WORK));
    size_t decomposition_bytes = LINEARS_SIZE_MUL(decomposition_count, sizeof(U_WORK));
    size_t signal_bytes = LINEARS_SIZE_MUL(signal_count, sizeof(U_SIGNAL));
    if (data_count == SIZE_MAX || data_bytes == SIZE_MAX || basic_bytes == SIZE_MAX ||
        decomposition_count == SIZE_MAX || decomposition_bytes == SIZE_MAX ||
        signal_count == SIZE_MAX || signal_bytes == SIZE_MAX)
        return LINEARS_ERROR_SIZE;
    if (workspace.capacity < data_count || workspace.basic_bytes < basic_bytes ||
        workspace.decomposition.capacity < decomposition_count || workspace.signal_capacity < signal_count)
        return LINEARS_ERROR_WORKSPACE;
    status = ukf_scratch(spans, workspace.data, data_bytes, U_ALIGNOF(U_WORK));
    if (status != LINEARS_OK) return status;
    status = ukf_scratch(spans, workspace.basic, basic_bytes, U_ALIGNOF(linears_workspace_unit_t));
    if (status != LINEARS_OK) return status;
    status = ukf_scratch(spans, workspace.decomposition.data, decomposition_bytes, U_ALIGNOF(U_WORK));
    if (status != LINEARS_OK) return status;
    status = ukf_scratch(spans, workspace.signal, signal_bytes, U_ALIGNOF(U_SIGNAL));
    if (status != LINEARS_OK) return status;
    return ukf_disjoint(spans);
}

/* Each cursor is the end of the used prefix; NULL is allowed for an empty
 * corresponding array. Equality avoids subtracting two null pointers. */
static inline U_WORKSPACE U_Q_HELPER(remaining)(U_WORKSPACE workspace, const U_WORK *cursor,
                                         const U_SIGNAL *signal_cursor)
{
    if (cursor != workspace.data) workspace.capacity -= (size_t)(cursor - workspace.data);
    if (signal_cursor != workspace.signal)
        workspace.signal_capacity -= (size_t)(signal_cursor - workspace.signal);
    workspace.data = (U_WORK *)cursor;
    workspace.signal = (U_SIGNAL *)signal_cursor;
    return workspace;
}

#undef U_Q_HELPER
#undef U_Q_JOIN
#undef U_Q_JOIN_INNER
