#include "filters_ukf.h"

#include <math.h>
#include <string.h>

/* Named carrier types keep offsetof strictly C99, including Clang. */
#define U_ALIGNMENT_TYPE(type) typedef struct { char byte; type object; } ukf_align_##type
U_ALIGNMENT_TYPE(float);
U_ALIGNMENT_TYPE(double);
U_ALIGNMENT_TYPE(int16_t);
U_ALIGNMENT_TYPE(int32_t);
U_ALIGNMENT_TYPE(linears_fixed_t);
U_ALIGNMENT_TYPE(linears_workspace_unit_t);
U_ALIGNMENT_TYPE(linears_matrix_t);
U_ALIGNMENT_TYPE(filters_ukf_f32_t);
U_ALIGNMENT_TYPE(filters_ukf_f64_t);
U_ALIGNMENT_TYPE(filters_ukf_q15_t);
U_ALIGNMENT_TYPE(filters_ukf_q31_t);
#undef U_ALIGNMENT_TYPE
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

/* Init registers the most ranges: the state object, four owned arrays and
 * four source descriptor/data pairs. */
typedef struct {
    ukf_span_t items[16];
    size_t count;
} ukf_spans_t;

static linears_status_t ukf_span(ukf_spans_t *spans, const void *pointer, size_t bytes,
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

static linears_status_t ukf_scratch(ukf_spans_t *spans, const void *pointer, size_t bytes,
                                    size_t alignment)
{
    linears_status_t status = ukf_span(spans, pointer, bytes, alignment, UKF_WRITABLE);
    return status == LINEARS_ERROR_ARGUMENT ? LINEARS_ERROR_WORKSPACE : status;
}

static linears_status_t ukf_disjoint(const ukf_spans_t *spans)
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
static linears_status_t ukf_shape(const linears_matrix_t *matrix, size_t rows, size_t cols,
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
static linears_status_t ukf_matrix(ukf_spans_t *spans, const linears_matrix_t *matrix,
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
static linears_status_t ukf_owned(ukf_spans_t *spans, const linears_matrix_t *matrix,
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

static linears_status_t ukf_finite_f32(const linears_matrix_t *matrix)
{
    size_t i, count = matrix->rows * matrix->cols;
    const float *values = (const float *)matrix->data;
    for (i = 0; i < count; ++i)
        if (!isfinite(values[i])) return LINEARS_ERROR_RANGE;
    return LINEARS_OK;
}

static linears_status_t ukf_finite_f64(const linears_matrix_t *matrix)
{
    size_t i, count = matrix->rows * matrix->cols;
    const double *values = (const double *)matrix->data;
    for (i = 0; i < count; ++i)
        if (!isfinite(values[i])) return LINEARS_ERROR_RANGE;
    return LINEARS_OK;
}

#define U_TAG f32
#define U_WORK float
#define U_FIXED 0
#define U_WEIGHTS filters_ukf_f32_weights_t
#define U_WORKSPACE filters_ukf_f32_workspace_t
#define U_FN filters_ukf_f32_fn
#define U_RESIDUAL_FN filters_ukf_f32_residual_fn
#define U_ZERO 0.0f
#define U_HALF 0.5f
#define U_FINITE ukf_finite_f32
#include "ukf_work_impl.h"

#define U_TAG f64
#define U_WORK double
#define U_FIXED 0
#define U_WEIGHTS filters_ukf_f64_weights_t
#define U_WORKSPACE filters_ukf_f64_workspace_t
#define U_FN filters_ukf_f64_fn
#define U_RESIDUAL_FN filters_ukf_f64_residual_fn
#define U_ZERO 0.0
#define U_HALF 0.5
#define U_FINITE ukf_finite_f64
#include "ukf_work_impl.h"

#define U_TAG fixed
#define U_WORK linears_fixed_t
#define U_FIXED 1
#define U_WEIGHTS filters_ukf_fixed_weights_t
#define U_WORKSPACE filters_ukf_fixed_workspace_t
#define U_FN filters_ukf_fixed_fn
#define U_RESIDUAL_FN filters_ukf_fixed_residual_fn
#define U_ZERO INT64_C(0)
#define U_HALF (LINEARS_FIXED_ONE / 2)
#include "ukf_work_impl.h"

#define U_TAG f32
#define U_KERNEL f32
#define U_SIGNAL float
#define U_WORK float
#define U_FIXED 0
#define U_STATE filters_ukf_f32_t
#define U_WEIGHTS filters_ukf_f32_weights_t
#define U_WORKSPACE filters_ukf_f32_workspace_t
#define U_FN filters_ukf_f32_fn
#define U_RESIDUAL_FN filters_ukf_f32_residual_fn
#define U_FINITE ukf_finite_f32
#define U_PREDICT_STORAGE FILTERS_UKF_PREDICT_STORAGE
#define U_PREDICT_BASIC FILTERS_UKF_PREDICT_BASIC_BYTES
#define U_UPDATE_STORAGE FILTERS_UKF_UPDATE_STORAGE
#define U_UPDATE_BASIC FILTERS_UKF_UPDATE_BASIC_BYTES
#include "ukf_filter_impl.h"

#define U_TAG f64
#define U_KERNEL f64
#define U_SIGNAL double
#define U_WORK double
#define U_FIXED 0
#define U_STATE filters_ukf_f64_t
#define U_WEIGHTS filters_ukf_f64_weights_t
#define U_WORKSPACE filters_ukf_f64_workspace_t
#define U_FN filters_ukf_f64_fn
#define U_RESIDUAL_FN filters_ukf_f64_residual_fn
#define U_FINITE ukf_finite_f64
#define U_PREDICT_STORAGE FILTERS_UKF_PREDICT_STORAGE
#define U_PREDICT_BASIC FILTERS_UKF_PREDICT_BASIC_BYTES
#define U_UPDATE_STORAGE FILTERS_UKF_UPDATE_STORAGE
#define U_UPDATE_BASIC FILTERS_UKF_UPDATE_BASIC_BYTES
#include "ukf_filter_impl.h"

#define U_TAG q15
#define U_KERNEL fixed
#define U_SIGNAL int16_t
#define U_WORK linears_fixed_t
#define U_FIXED 1
#define U_STATE filters_ukf_q15_t
#define U_WEIGHTS filters_ukf_fixed_weights_t
#define U_WORKSPACE filters_ukf_fixed_workspace_t
#define U_FN filters_ukf_fixed_fn
#define U_RESIDUAL_FN filters_ukf_fixed_residual_fn
#define U_IMPORT linears_fixed_from_q15
#define U_EXPORT linears_fixed_to_q15
#define U_PACK_FIELD q15
#define U_PREDICT_STORAGE FILTERS_UKF_Q_PREDICT_STORAGE
#define U_PREDICT_BASIC FILTERS_UKF_Q_PREDICT_BASIC_BYTES
#define U_UPDATE_STORAGE FILTERS_UKF_Q_UPDATE_STORAGE
#define U_UPDATE_BASIC FILTERS_UKF_Q_UPDATE_BASIC_BYTES
#include "ukf_filter_impl.h"

#define U_TAG q31
#define U_KERNEL fixed
#define U_SIGNAL int32_t
#define U_WORK linears_fixed_t
#define U_FIXED 1
#define U_STATE filters_ukf_q31_t
#define U_WEIGHTS filters_ukf_fixed_weights_t
#define U_WORKSPACE filters_ukf_fixed_workspace_t
#define U_FN filters_ukf_fixed_fn
#define U_RESIDUAL_FN filters_ukf_fixed_residual_fn
#define U_IMPORT linears_fixed_from_q31
#define U_EXPORT linears_fixed_to_q31
#define U_PACK_FIELD q31
#define U_PREDICT_STORAGE FILTERS_UKF_Q_PREDICT_STORAGE
#define U_PREDICT_BASIC FILTERS_UKF_Q_PREDICT_BASIC_BYTES
#define U_UPDATE_STORAGE FILTERS_UKF_Q_UPDATE_STORAGE
#define U_UPDATE_BASIC FILTERS_UKF_Q_UPDATE_BASIC_BYTES
#include "ukf_filter_impl.h"

#undef U_ALIGNOF
#undef U_ALIGN_NAME
#undef U_ALIGN_NAME_INNER
