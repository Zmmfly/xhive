/* Validation helpers shared by the UKF, PF and UPF translation units. */
#ifndef FILTERS_UKF_INTERNAL_H
#define FILTERS_UKF_INTERNAL_H

#include "filters_ukf.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* Named carrier types keep offsetof strictly C99, including Clang. Each
 * translation unit adds carriers for its own state object types. */
#define U_ALIGNMENT_TYPE(type) typedef struct { char byte; type object; } ukf_align_##type
U_ALIGNMENT_TYPE(float);
U_ALIGNMENT_TYPE(double);
U_ALIGNMENT_TYPE(int16_t);
U_ALIGNMENT_TYPE(int32_t);
U_ALIGNMENT_TYPE(size_t);
U_ALIGNMENT_TYPE(linears_workspace_unit_t);
U_ALIGNMENT_TYPE(linears_matrix_t);
#define U_ALIGN_NAME_INNER(type) ukf_align_##type
#define U_ALIGN_NAME(type) U_ALIGN_NAME_INNER(type)
#define U_ALIGNOF(type) offsetof(U_ALIGN_NAME(type), object)

/* Read-only inputs may overlap each other; descriptors may only be identical;
 * writable storage (state, outputs, scratch) must be disjoint from everything. */
typedef enum { UKF_INPUT, UKF_DESCRIPTOR, UKF_WRITABLE } ukf_span_kind_t;

typedef struct {
    uintptr_t begin;
    uintptr_t end;
    ukf_span_kind_t kind;
} ukf_span_t;

/* UPF init registers the most ranges: the state object, five owned arrays,
 * four source descriptor/data pairs and three scratch arrays. */
typedef struct {
    ukf_span_t items[24];
    size_t count;
} ukf_spans_t;

static inline linears_status_t ukf_span(ukf_spans_t *spans, const void *pointer, size_t bytes,
                                        size_t alignment, ukf_span_kind_t kind)
{
    uintptr_t address = (uintptr_t)pointer;
    ukf_span_t *span;
    if (bytes == SIZE_MAX) return LINEARS_ERROR_SIZE;
    if (bytes != 0 && (pointer == NULL || address % alignment != 0))
        return LINEARS_ERROR_ARGUMENT;
    if (bytes > UINTPTR_MAX - address) return LINEARS_ERROR_SIZE;
    if (bytes == 0) return LINEARS_OK;
    span = &spans->items[spans->count++];
    span->begin = address;
    span->end = address + bytes;
    span->kind = kind;
    return LINEARS_OK;
}

static inline linears_status_t ukf_scratch(ukf_spans_t *spans, const void *pointer, size_t bytes,
                                           size_t alignment)
{
    linears_status_t status = ukf_span(spans, pointer, bytes, alignment, UKF_WRITABLE);
    return status == LINEARS_ERROR_ARGUMENT ? LINEARS_ERROR_WORKSPACE : status;
}

static inline linears_status_t ukf_disjoint(const ukf_spans_t *spans)
{
    size_t i, j;
    for (i = 1; i < spans->count; ++i) {
        const ukf_span_t *a = &spans->items[i];
        for (j = 0; j < i; ++j) {
            const ukf_span_t *b = &spans->items[j];
            if (!(a->begin < b->end && b->begin < a->end)) continue;
            if (a->kind == UKF_INPUT && b->kind == UKF_INPUT) continue;
            if (a->kind == UKF_DESCRIPTOR && b->kind == UKF_DESCRIPTOR &&
                a->begin == b->begin && a->end == b->end) continue;
            return LINEARS_ERROR_ALIAS;
        }
    }
    return LINEARS_OK;
}

/* A zero expected extent accepts any positive extent. */
static inline linears_status_t ukf_shape(const linears_matrix_t *matrix, size_t rows, size_t cols,
                                         size_t element_size, size_t *bytes)
{
    size_t count;
    if (matrix->rows == 0 || matrix->cols == 0 ||
        (rows != 0 && matrix->rows != rows) || (cols != 0 && matrix->cols != cols))
        return LINEARS_ERROR_DIMENSION;
    count = LINEARS_SIZE_MUL(matrix->rows, matrix->cols);
    *bytes = LINEARS_SIZE_MUL(count, element_size);
    return count == SIZE_MAX || *bytes == SIZE_MAX ? LINEARS_ERROR_SIZE : LINEARS_OK;
}

/* A caller-supplied descriptor: validate its address before reading fields. */
static inline linears_status_t ukf_matrix(ukf_spans_t *spans, const linears_matrix_t *matrix,
                                          size_t rows, size_t cols, size_t element_size,
                                          size_t element_alignment, ukf_span_kind_t data_kind)
{
    linears_status_t status;
    size_t bytes;
    status = ukf_span(spans, matrix, sizeof(*matrix), U_ALIGNOF(linears_matrix_t), UKF_DESCRIPTOR);
    if (status != LINEARS_OK) return status;
    status = ukf_shape(matrix, rows, cols, element_size, &bytes);
    if (status != LINEARS_OK) return status;
    return ukf_span(spans, matrix->data, bytes, element_alignment, data_kind);
}

/* An array described inside an already validated state object. */
static inline linears_status_t ukf_owned(ukf_spans_t *spans, const linears_matrix_t *matrix,
                                         size_t rows, size_t cols, size_t capacity,
                                         size_t element_size, size_t element_alignment)
{
    linears_status_t status;
    size_t bytes;
    status = ukf_shape(matrix, rows, cols, element_size, &bytes);
    if (status != LINEARS_OK) return status;
    if (capacity < matrix->rows * matrix->cols) return LINEARS_ERROR_WORKSPACE;
    return ukf_span(spans, matrix->data, bytes, element_alignment, UKF_WRITABLE);
}

/* Register an init source: its own descriptor/storage is not a second range. */
static inline linears_status_t ukf_source(ukf_spans_t *spans, const linears_matrix_t *source,
                                          const linears_matrix_t *target, size_t element_size,
                                          size_t element_alignment)
{
    linears_status_t status;
    size_t bytes;
    if (source != target) {
        status = ukf_span(spans, source, sizeof(*source), U_ALIGNOF(linears_matrix_t), UKF_DESCRIPTOR);
        if (status != LINEARS_OK) return status;
    }
    status = ukf_shape(source, target->rows, target->cols, element_size, &bytes);
    if (status != LINEARS_OK) return status;
    if (source->data == target->data) return LINEARS_OK;
    return ukf_span(spans, source->data, bytes, element_alignment, UKF_INPUT);
}

static inline linears_status_t ukf_finite_f32(const linears_matrix_t *matrix)
{
    size_t i, count = matrix->rows * matrix->cols;
    const float *values = (const float *)matrix->data;
    for (i = 0; i < count; ++i)
        if (!isfinite(values[i])) return LINEARS_ERROR_RANGE;
    return LINEARS_OK;
}

static inline linears_status_t ukf_finite_f64(const linears_matrix_t *matrix)
{
    size_t i, count = matrix->rows * matrix->cols;
    const double *values = (const double *)matrix->data;
    for (i = 0; i < count; ++i)
        if (!isfinite(values[i])) return LINEARS_ERROR_RANGE;
    return LINEARS_OK;
}

/* Exact stored symmetry, optionally with a nonnegative diagonal, per element type. */
#define U_SYMMETRIC_TYPE(suffix, type) \
static inline linears_status_t ukf_symmetric_##suffix(const linears_matrix_t *matrix, int check_diagonal) \
{ \
    size_t i, j, n = matrix->rows; \
    const type *values = (const type *)matrix->data; \
    for (i = 0; i < n; ++i) { \
        if (check_diagonal && values[i * n + i] < 0) return LINEARS_ERROR_DOMAIN; \
        for (j = 0; j < i; ++j) \
            if (values[i * n + j] != values[j * n + i]) return LINEARS_ERROR_DOMAIN; \
    } \
    return LINEARS_OK; \
}
U_SYMMETRIC_TYPE(f32, float)
U_SYMMETRIC_TYPE(f64, double)
U_SYMMETRIC_TYPE(q15, int16_t)
U_SYMMETRIC_TYPE(q31, int32_t)
#undef U_SYMMETRIC_TYPE

/* Q signals widen exactly to the floating working type whose significand
 * covers them: q15 to float, q31 to double. widen converts a block; import
 * also carves the destination from a scratch cursor. */
#define U_WIDEN_TYPE(suffix, type, work, scale) \
static inline void ukf_widen_##suffix(const type *input, size_t count, work *output) \
{ \
    size_t i; \
    for (i = 0; i < count; ++i) output[i] = (work)input[i] * (scale); \
} \
static inline linears_matrix_t ukf_import_##suffix(const linears_matrix_t *source, work **cursor) \
{ \
    size_t count = source->rows * source->cols; \
    linears_matrix_t result = {*cursor, source->rows, source->cols}; \
    ukf_widen_##suffix((const type *)source->data, count, *cursor); \
    *cursor += count; \
    return result; \
}
U_WIDEN_TYPE(q15, int16_t, float, 1.0f / 32768.0f)
U_WIDEN_TYPE(q31, int32_t, double, 1.0 / 2147483648.0)
#undef U_WIDEN_TYPE

/* Narrowing is split into a range check that writes nothing and a store that
 * cannot fail, so a transaction checks every output before committing any.
 * Rounding is to nearest with ties away from zero; exact +1 and the rounding
 * cell that reaches it map to the positive maximum (the linears policy). */
#define U_NARROW_TYPE(suffix, type, work, scale, maximum, round) \
static inline linears_status_t ukf_fits_##suffix(const work *values, size_t count) \
{ \
    size_t i; \
    for (i = 0; i < count; ++i) \
        if (!(values[i] >= (work)-1 && values[i] <= (work)1)) return LINEARS_ERROR_RANGE; \
    return LINEARS_OK; \
} \
static inline void ukf_store_##suffix(const work *values, size_t count, type *output) \
{ \
    size_t i; \
    for (i = 0; i < count; ++i) { \
        work scaled = values[i] * (scale); \
        /* Below the cell that rounds to the maximum, |scaled| fits a 32-bit long. */ \
        output[i] = scaled >= (work)(maximum) + (work)0.5 ? (maximum) : (type)round(scaled); \
    } \
}
U_NARROW_TYPE(q15, int16_t, float, 32768.0f, INT16_MAX, lroundf)
U_NARROW_TYPE(q31, int32_t, double, 2147483648.0, INT32_MAX, lround)
#undef U_NARROW_TYPE

/* Gather rows in place for nondecreasing indices (systematic resampling).
 * Forward moves copy from later rows, which no earlier forward move has
 * overwritten; backward moves copy from earlier rows p = indices[i] < i,
 * which no forward move touched because indices[p] <= indices[i] = p. */
static inline void ukf_gather_rows(void *rows, size_t row_bytes, const size_t *indices, size_t count)
{
    unsigned char *bytes = (unsigned char *)rows;
    size_t i;
    for (i = 0; i < count; ++i)
        if (indices[i] > i) memcpy(bytes + i * row_bytes, bytes + indices[i] * row_bytes, row_bytes);
    for (i = count; i-- > 0;)
        if (indices[i] < i) memcpy(bytes + i * row_bytes, bytes + indices[i] * row_bytes, row_bytes);
}

#endif
