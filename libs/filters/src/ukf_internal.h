/* Validation helpers shared by the UKF, PF and UPF translation units. */
#ifndef FILTERS_UKF_INTERNAL_H
#define FILTERS_UKF_INTERNAL_H

#include "filters_ukf.h"

#include <float.h>
#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

/* Integer bit paths require the advertised IEEE encoding and matching integer
 * and floating word order. Unknown representations keep native conversions.
 * CLZ below also requires the compiler's unsigned int to have 32 bits. These
 * private detection constants remain available to the estimator templates. */
#if defined(__GNUC__) && defined(__BYTE_ORDER__) && defined(__FLOAT_WORD_ORDER__) && \
    __BYTE_ORDER__ == __FLOAT_WORD_ORDER__ && UINT_MAX == UINT32_MAX && \
    defined(__SIZEOF_FLOAT__) && __SIZEOF_FLOAT__ == 4 && FLT_RADIX == 2 && \
    FLT_MANT_DIG == 24 && FLT_MIN_EXP == -125 && FLT_MAX_EXP == 128 && \
    ((defined(__FLT_IS_IEC_60559__) && __FLT_IS_IEC_60559__ > 0) || \
     (defined(__STDC_IEC_559__) && __STDC_IEC_559__))
#define UKF_IEEE_BINARY32 1
#else
#define UKF_IEEE_BINARY32 0
#endif
#if defined(__GNUC__) && defined(__BYTE_ORDER__) && defined(__FLOAT_WORD_ORDER__) && \
    __BYTE_ORDER__ == __FLOAT_WORD_ORDER__ && UINT_MAX == UINT32_MAX && \
    defined(__SIZEOF_DOUBLE__) && __SIZEOF_DOUBLE__ == 8 && FLT_RADIX == 2 && \
    DBL_MANT_DIG == 53 && DBL_MIN_EXP == -1021 && DBL_MAX_EXP == 1024 && \
    ((defined(__DBL_IS_IEC_60559__) && __DBL_IS_IEC_60559__ > 0) || \
     (defined(__STDC_IEC_559__) && __STDC_IEC_559__))
#define UKF_IEEE_BINARY64 1
#else
#define UKF_IEEE_BINARY64 0
#endif

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

/**
 * @brief Test finiteness without soft floating arithmetic on known IEEE layouts.
 *
 * @param[in] value Any single-precision value, including NaNs and infinities.
 * @return Nonzero for finite values, including subnormals and both signed zeros.
 */
static inline int ukf_isfinite_f32(float value)
{
#if UKF_IEEE_BINARY32
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return (bits & UINT32_C(0x7f800000)) != UINT32_C(0x7f800000);
#else
    return isfinite(value);
#endif
}

/**
 * @brief Test double-precision finiteness with the same contract as f32.
 *
 * @param[in] value Any double-precision value, including NaNs and infinities.
 * @return Nonzero for finite values, including subnormals and both signed zeros.
 */
static inline int ukf_isfinite_f64(double value)
{
#if UKF_IEEE_BINARY64
    uint64_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return (bits & UINT64_C(0x7ff0000000000000)) != UINT64_C(0x7ff0000000000000);
#else
    return isfinite(value);
#endif
}

/**
 * @brief Test membership in the inclusive Q signal interval without narrowing.
 *
 * @param[in] value Any single-precision value.
 * @return Nonzero for finite values in [-1,+1], including both signed zeros.
 */
static inline int ukf_in_unit_f32(float value)
{
#if UKF_IEEE_BINARY32
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return (bits & UINT32_C(0x7fffffff)) <= UINT32_C(0x3f800000);
#else
    return value >= -1.0f && value <= 1.0f;
#endif
}

/**
 * @brief Test double-precision membership in the inclusive Q signal interval.
 *
 * @param[in] value Any double-precision value.
 * @return Nonzero for finite values in [-1,+1], including both signed zeros.
 */
static inline int ukf_in_unit_f64(double value)
{
#if UKF_IEEE_BINARY64
    uint64_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return (bits & UINT64_C(0x7fffffffffffffff)) <= UINT64_C(0x3ff0000000000000);
#else
    return value >= -1.0 && value <= 1.0;
#endif
}

static inline linears_status_t ukf_finite_f32(const linears_matrix_t *matrix)
{
    size_t i, count = matrix->rows * matrix->cols;
    const float *values = (const float *)matrix->data;
    for (i = 0; i < count; ++i)
        if (!ukf_isfinite_f32(values[i])) return LINEARS_ERROR_RANGE;
    return LINEARS_OK;
}

static inline linears_status_t ukf_finite_f64(const linears_matrix_t *matrix)
{
    size_t i, count = matrix->rows * matrix->cols;
    const double *values = (const double *)matrix->data;
    for (i = 0; i < count; ++i)
        if (!ukf_isfinite_f64(values[i])) return LINEARS_ERROR_RANGE;
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

/**
 * @brief Widen one raw q15 signal exactly to its float working value.
 *
 * @param[in] value Raw int16_t signal representing value/2^15.
 * @return The exact signal value, without rounding or range loss.
 */
static inline float ukf_widen_value_q15(int16_t value)
{
#if UKF_IEEE_BINARY32
    uint32_t sign = value < 0 ? UINT32_C(0x80000000) : 0;
    uint32_t magnitude = value < 0 ? (uint32_t)-(int32_t)value : (uint32_t)value;
    uint32_t bits;
    unsigned leading;
    float result;
    if (magnitude == 0) return 0.0f;
    leading = (unsigned)__builtin_clz(magnitude);
    /* Highest magnitude bit is 31-leading; subtracting 15 gives the
     * unbiased exponent. All 16 magnitude bits fit the 24-bit significand. */
    bits = sign | ((uint32_t)(143 - leading) << 23) |
           ((magnitude << (leading - 8)) & UINT32_C(0x7fffff));
    memcpy(&result, &bits, sizeof(result));
    return result;
#else
    return (float)value * (1.0f / 32768.0f);
#endif
}

/**
 * @brief Widen one raw q31 signal exactly to its double working value.
 *
 * @param[in] value Raw int32_t signal representing value/2^31.
 * @return The exact signal value, without rounding or range loss.
 */
static inline double ukf_widen_value_q31(int32_t value)
{
#if UKF_IEEE_BINARY64
    uint32_t sign = (uint32_t)value >> 31;
    uint32_t magnitude = sign ? UINT32_C(0) - (uint32_t)value : (uint32_t)value;
    uint64_t bits;
    unsigned leading;
    double result;
    if (magnitude == 0) return 0.0;
    leading = (unsigned)__builtin_clz(magnitude);
    bits = ((uint64_t)sign << 63) | ((uint64_t)(1023 - leading) << 52) |
           (((uint64_t)magnitude << (21 + leading)) & UINT64_C(0xfffffffffffff));
    memcpy(&result, &bits, sizeof(result));
    return result;
#else
    return (double)value * (1.0 / 2147483648.0);
#endif
}

/* Q signals widen exactly to the floating working type whose significand
 * covers them: q15 to float, q31 to double. widen converts a block; import
 * also carves the destination from a scratch cursor. */
#define U_WIDEN_TYPE(suffix, type, work) \
static inline void ukf_widen_##suffix(const type *input, size_t count, work *output) \
{ \
    size_t i; \
    for (i = 0; i < count; ++i) output[i] = ukf_widen_value_##suffix(input[i]); \
} \
static inline linears_matrix_t ukf_import_##suffix(const linears_matrix_t *source, work **cursor) \
{ \
    size_t count = source->rows * source->cols; \
    linears_matrix_t result = {*cursor, source->rows, source->cols}; \
    ukf_widen_##suffix((const type *)source->data, count, *cursor); \
    *cursor += count; \
    return result; \
}
U_WIDEN_TYPE(q15, int16_t, float)
U_WIDEN_TYPE(q31, int32_t, double)
#undef U_WIDEN_TYPE

/**
 * @brief Narrow one checked float signal to q15, with half ties away from zero.
 *
 * @param[in] value Finite signal in [-1,+1], checked before any state commit.
 * @return Raw q15; +1 and its rounding cell map to INT16_MAX, -1 to INT16_MIN.
 * @pre The caller has validated value; this helper performs no range check.
 */
static inline int16_t ukf_narrow_value_q15(float value)
{
#if UKF_IEEE_BINARY32
    uint32_t bits, exponent, significand, raw;
    memcpy(&bits, &value, sizeof(bits));
    exponent = (bits & UINT32_C(0x7fffffff)) >> 23;
    /* |value|*2^15 = significand*2^(exponent-135). Below exponent 111
     * even the largest significand is less than half a raw unit. */
    if (exponent < 111) return 0;
    significand = (bits & UINT32_C(0x7fffff)) | UINT32_C(0x800000);
    raw = ((significand >> (134 - exponent)) + 1) >> 1;
    if ((bits >> 31) != 0) return (int16_t)-(int32_t)raw;
    return raw == 32768 ? INT16_MAX : (int16_t)raw;
#else
    float scaled = value * 32768.0f;
    return scaled >= (float)INT16_MAX + 0.5f ? INT16_MAX : (int16_t)lroundf(scaled);
#endif
}

/**
 * @brief Narrow one checked double signal to q31, with half ties away from zero.
 *
 * @param[in] value Finite signal in [-1,+1], checked before any state commit.
 * @return Raw q31; +1 and its rounding cell map to INT32_MAX, -1 to INT32_MIN.
 * @pre The caller has validated value; this helper performs no range check.
 */
static inline int32_t ukf_narrow_value_q31(double value)
{
#if UKF_IEEE_BINARY64
    uint64_t bits, significand;
    uint32_t exponent, raw;
    memcpy(&bits, &value, sizeof(bits));
    exponent = (uint32_t)((bits & UINT64_C(0x7fffffffffffffff)) >> 52);
    /* |value|*2^31 = significand*2^(exponent-1044). Round once from
     * the original significand; adding floating 0.5 could double-round. */
    if (exponent < 991) return 0;
    significand = (bits & UINT64_C(0xfffffffffffff)) | UINT64_C(0x10000000000000);
    raw = (uint32_t)(((significand >> (1043 - exponent)) + 1) >> 1);
    if ((bits >> 63) != 0) return (int32_t)-(int64_t)raw;
    return raw == UINT32_C(0x80000000) ? INT32_MAX : (int32_t)raw;
#else
    double scaled = value * 2147483648.0;
    return scaled >= (double)INT32_MAX + 0.5 ? INT32_MAX : (int32_t)lround(scaled);
#endif
}

/* Narrowing is split into a range check that writes nothing and a store that
 * cannot fail, so a transaction checks every output before committing any.
 * Rounding is to nearest with ties away from zero; exact +1 and the rounding
 * cell that reaches it map to the positive maximum (the linears policy). */
#define U_NARROW_TYPE(suffix, type, work, in_unit) \
static inline linears_status_t ukf_fits_##suffix(const work *values, size_t count) \
{ \
    size_t i; \
    for (i = 0; i < count; ++i) \
        if (!in_unit(values[i])) return LINEARS_ERROR_RANGE; \
    return LINEARS_OK; \
} \
static inline void ukf_store_##suffix(const work *values, size_t count, type *output) \
{ \
    size_t i; \
    for (i = 0; i < count; ++i) output[i] = ukf_narrow_value_##suffix(values[i]); \
}
U_NARROW_TYPE(q15, int16_t, float, ukf_in_unit_f32)
U_NARROW_TYPE(q31, int32_t, double, ukf_in_unit_f64)
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
