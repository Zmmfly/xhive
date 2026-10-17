#include "filters_kalman.h"

#include <math.h>
#include <string.h>

/* Named carrier types keep offsetof strictly C99, including Clang. */
#define K_ALIGNMENT_TYPE(type) typedef struct { char byte; type object; } kalman_align_##type
K_ALIGNMENT_TYPE(float);
K_ALIGNMENT_TYPE(double);
K_ALIGNMENT_TYPE(int16_t);
K_ALIGNMENT_TYPE(int32_t);
K_ALIGNMENT_TYPE(linears_fixed_t);
K_ALIGNMENT_TYPE(linears_workspace_unit_t);
K_ALIGNMENT_TYPE(linears_matrix_t);
K_ALIGNMENT_TYPE(filters_kalman_matrix_t);
#undef K_ALIGNMENT_TYPE
#define K_ALIGN_NAME_INNER(type) kalman_align_##type
#define K_ALIGN_NAME(type) K_ALIGN_NAME_INNER(type)
#define K_ALIGNOF(type) offsetof(K_ALIGN_NAME(type), object)

typedef struct {
    uintptr_t begin;
    uintptr_t end;
} kalman_range_t;

typedef struct {
    kalman_range_t state[3]; /* Context, estimate data, covariance data. */
    kalman_range_t input_data[3];
    kalman_range_t input_desc[3];
    size_t count;
} kalman_boundary_t;

static linears_status_t kalman_range(const void *pointer, size_t bytes,
                                     size_t alignment, kalman_range_t *out)
{
    uintptr_t address = (uintptr_t)pointer;
    if (bytes == SIZE_MAX) return LINEARS_ERROR_SIZE;
    if (bytes != 0 && (pointer == NULL || address % alignment != 0))
        return LINEARS_ERROR_ARGUMENT;
    if (bytes > UINTPTR_MAX - address) return LINEARS_ERROR_SIZE;
    out->begin = address;
    out->end = address + bytes;
    return LINEARS_OK;
}

static int kalman_overlap(kalman_range_t a, kalman_range_t b)
{
    return a.begin < b.end && b.begin < a.end;
}

static int kalman_same_range(kalman_range_t a, kalman_range_t b)
{
    return a.begin == b.begin && a.end == b.end;
}

static linears_status_t kalman_input_alias(const filters_kalman_matrix_t *state,
                                          const kalman_boundary_t *b, int reset)
{
    size_t i, j;
    for (i = 0; i < b->count; ++i) {
        int own_descriptor = reset &&
            ((i == 0 && b->input_desc[i].begin == (uintptr_t)&state->estimate) ||
             (i == 1 && b->input_desc[i].begin == (uintptr_t)&state->covariance));
        if (kalman_overlap(b->input_data[i], b->state[0]) ||
            (!own_descriptor && kalman_overlap(b->input_desc[i], b->state[0])))
            return LINEARS_ERROR_ALIAS;
        for (j = 1; j < 3; ++j) {
            if (kalman_overlap(b->input_desc[i], b->state[j]))
                return LINEARS_ERROR_ALIAS;
            if (kalman_overlap(b->input_data[i], b->state[j]) &&
                !(reset && i + 1 == j &&
                  kalman_same_range(b->input_data[i], b->state[j])))
                return LINEARS_ERROR_ALIAS;
        }
        for (j = 0; j < b->count; ++j) {
            if (kalman_overlap(b->input_data[i], b->input_desc[j]))
                return LINEARS_ERROR_ALIAS;
        }
    }
    return LINEARS_OK;
}

static linears_status_t kalman_workspace_alias(const kalman_boundary_t *b,
                                              const void *data, size_t data_bytes,
                                              size_t data_alignment,
                                              linears_workspace_unit_t *basic,
                                              size_t basic_bytes,
                                              const void *decomposition,
                                              size_t decomposition_bytes,
                                              size_t decomposition_alignment)
{
    kalman_range_t scratch[3];
    linears_status_t status;
    size_t i, j;
    status = kalman_range(data, data_bytes, data_alignment, &scratch[0]);
    if (status == LINEARS_ERROR_ARGUMENT) return LINEARS_ERROR_WORKSPACE;
    if (status != LINEARS_OK) return status;
    status = kalman_range(basic, basic_bytes, K_ALIGNOF(linears_workspace_unit_t), &scratch[1]);
    if (status == LINEARS_ERROR_ARGUMENT) return LINEARS_ERROR_WORKSPACE;
    if (status != LINEARS_OK) return status;
    status = kalman_range(decomposition, decomposition_bytes,
                          decomposition_alignment, &scratch[2]);
    if (status == LINEARS_ERROR_ARGUMENT) return LINEARS_ERROR_WORKSPACE;
    if (status != LINEARS_OK) return status;
    for (i = 0; i < 3; ++i) {
        for (j = 0; j < i; ++j)
            if (kalman_overlap(scratch[i], scratch[j])) return LINEARS_ERROR_ALIAS;
        for (j = 0; j < 3; ++j)
            if (kalman_overlap(scratch[i], b->state[j])) return LINEARS_ERROR_ALIAS;
        for (j = 0; j < b->count; ++j)
            if (kalman_overlap(scratch[i], b->input_data[j]) ||
                kalman_overlap(scratch[i], b->input_desc[j])) return LINEARS_ERROR_ALIAS;
    }
    return LINEARS_OK;
}

static linears_status_t kalman_value_f32(float value, int exponent)
{
    (void)exponent;
    return isfinite(value) ? LINEARS_OK : LINEARS_ERROR_RANGE;
}

static linears_status_t kalman_value_f64(double value, int exponent)
{
    (void)exponent;
    return isfinite(value) ? LINEARS_OK : LINEARS_ERROR_RANGE;
}

static linears_status_t kalman_value_q15(int16_t value, int exponent)
{
    linears_q15_t raw = {value, exponent};
    linears_fixed_t normalized;
    return linears_fixed_from_q15(raw, &normalized);
}

static linears_status_t kalman_value_q31(int32_t value, int exponent)
{
    linears_q31_t raw = {value, exponent};
    linears_fixed_t normalized;
    return linears_fixed_from_q31(raw, &normalized);
}

#define K_TAG f32
#define K_RAW float
#define K_DECOMP_RAW float
#define K_WORKSPACE filters_kalman_matrix_f32_workspace_t
#define K_VALUE_CHECK kalman_value_f32
#define K_OUTPUT_EXPONENT(value) 0
#define K_ONE 1.0f
#define K_MINUS_ONE (-1.0f)
#define K_HALF 0.5f
#define K_RAW_ONE 1.0f
#define K_IDENTITY_EXPONENT 0
#include "kalman_matrix_impl.h"

#define K_TAG f64
#define K_RAW double
#define K_DECOMP_RAW double
#define K_WORKSPACE filters_kalman_matrix_f64_workspace_t
#define K_VALUE_CHECK kalman_value_f64
#define K_OUTPUT_EXPONENT(value) 0
#define K_ONE 1.0
#define K_MINUS_ONE (-1.0)
#define K_HALF 0.5
#define K_RAW_ONE 1.0
#define K_IDENTITY_EXPONENT 0
#include "kalman_matrix_impl.h"

#define K_TAG q15
#define K_RAW int16_t
#define K_DECOMP_RAW linears_fixed_t
#define K_WORKSPACE filters_kalman_matrix_q15_workspace_t
#define K_VALUE_CHECK kalman_value_q15
#define K_OUTPUT_EXPONENT(value) (value)
#define K_ONE ((linears_q15_t){INT16_C(16384), 1})
#define K_MINUS_ONE ((linears_q15_t){-INT16_C(16384), 1})
#define K_HALF ((linears_q15_t){INT16_C(16384), 0})
#define K_RAW_ONE INT16_C(16384)
#define K_IDENTITY_EXPONENT 1
#include "kalman_matrix_impl.h"

#define K_TAG q31
#define K_RAW int32_t
#define K_DECOMP_RAW linears_fixed_t
#define K_WORKSPACE filters_kalman_matrix_q31_workspace_t
#define K_VALUE_CHECK kalman_value_q31
#define K_OUTPUT_EXPONENT(value) (value)
#define K_ONE ((linears_q31_t){INT32_C(1073741824), 1})
#define K_MINUS_ONE ((linears_q31_t){-INT32_C(1073741824), 1})
#define K_HALF ((linears_q31_t){INT32_C(1073741824), 0})
#define K_RAW_ONE INT32_C(1073741824)
#define K_IDENTITY_EXPONENT 1
#include "kalman_matrix_impl.h"

#undef K_ALIGNOF
