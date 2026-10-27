/* Per-tag scratch helpers shared by the UKF, PF and UPF templates. Include
 * with U_TAG, U_WORK and U_WORKSPACE defined; they stay defined afterwards. */
#define U_SCRATCH_JOIN_INNER(a, b) a##b
#define U_SCRATCH_JOIN(a, b) U_SCRATCH_JOIN_INNER(a, b)
#define U_SCRATCH(name) U_SCRATCH_JOIN(U_SCRATCH_JOIN(ukf_work_, U_TAG), U_SCRATCH_JOIN(_, name))

static linears_matrix_t U_SCRATCH(take)(U_WORK **cursor, size_t rows, size_t cols)
{
    linears_matrix_t result = {*cursor, rows, cols};
    *cursor += rows * cols;
    return result;
}

/* Row i of a row-major s-by-k matrix viewed as a contiguous k-by-1 vector. */
static linears_matrix_t U_SCRATCH(row)(const linears_matrix_t *matrix, size_t index)
{
    linears_matrix_t result;
    result.data = (U_WORK *)matrix->data + index * matrix->cols;
    result.rows = matrix->cols;
    result.cols = 1;
    return result;
}

static linears_status_t U_SCRATCH(workspace)(ukf_spans_t *spans, U_WORKSPACE workspace,
                                            size_t data_count, size_t basic_bytes,
                                            size_t decomposition_count)
{
    linears_status_t status;
    size_t data_bytes = LINEARS_SIZE_MUL(data_count, sizeof(U_WORK));
    size_t decomposition_bytes = LINEARS_SIZE_MUL(decomposition_count, sizeof(U_WORK));
    if (data_count == SIZE_MAX || data_bytes == SIZE_MAX || basic_bytes == SIZE_MAX ||
        decomposition_count == SIZE_MAX || decomposition_bytes == SIZE_MAX)
        return LINEARS_ERROR_SIZE;
    if (workspace.capacity < data_count || workspace.basic_bytes < basic_bytes ||
        workspace.decomposition.capacity < decomposition_count)
        return LINEARS_ERROR_WORKSPACE;
    status = ukf_scratch(spans, workspace.data, data_bytes, U_ALIGNOF(U_WORK));
    if (status != LINEARS_OK) return status;
    status = ukf_scratch(spans, workspace.basic, basic_bytes, U_ALIGNOF(linears_workspace_unit_t));
    if (status != LINEARS_OK) return status;
    status = ukf_scratch(spans, workspace.decomposition.data, decomposition_bytes, U_ALIGNOF(U_WORK));
    if (status != LINEARS_OK) return status;
    return ukf_disjoint(spans);
}

/* The caller keeps everything before the cursor; the callee gets the rest. */
static U_WORKSPACE U_SCRATCH(remaining)(U_WORKSPACE workspace, const U_WORK *cursor)
{
    workspace.capacity -= (size_t)(cursor - (const U_WORK *)workspace.data);
    workspace.data = (U_WORK *)cursor;
    return workspace;
}

#undef U_SCRATCH
#undef U_SCRATCH_JOIN
#undef U_SCRATCH_JOIN_INNER
