#include "linears.h"
#include "linears_numeric.h"

#include <float.h>
#include <math.h>
#include <string.h>

/* C99 alignment probes. Scratch is an array of actual union objects: each
 * element is accessed through its member, never as a reinterpreted scalar array. */
struct basic_align_f32 { char padding; float value; };
struct basic_align_f64 { char padding; double value; };
struct basic_align_q15 { char padding; int16_t value; };
struct basic_align_q31 { char padding; int32_t value; };
struct basic_align_matrix { char padding; linears_matrix_t value; };
struct basic_align_workspace { char padding; linears_workspace_unit_t value; };
struct basic_align_scalar_q15 { char padding; linears_q15_t value; };
struct basic_align_scalar_q31 { char padding; linears_q31_t value; };

typedef enum { F32, F64, Q15, Q31 } numeric_kind;
typedef enum { ADD, SUB, HADAMARD, MUL, MATVEC, TRANSPOSE, AXPBY, SCALE } matrix_op;
typedef enum { DOT, NORM, TRACE } reduction_op;

static size_t element_size(numeric_kind kind) {
    switch (kind) {
        case F32: return sizeof(float);
        case F64: return sizeof(double);
        case Q15: return sizeof(int16_t);
        default: return sizeof(int32_t);
    }
}

static size_t element_alignment(numeric_kind kind) {
    switch (kind) {
        case F32: return offsetof(struct basic_align_f32, value);
        case F64: return offsetof(struct basic_align_f64, value);
        case Q15: return offsetof(struct basic_align_q15, value);
        default: return offsetof(struct basic_align_q31, value);
    }
}

static int overlap(const void *a, size_t asize, const void *b, size_t bsize) {
    if (asize == 0 || bsize == 0) return 0;
    uintptr_t x = (uintptr_t)a, y = (uintptr_t)b;
    return x <= y ? y - x < asize : x - y < bsize;
}

static linears_status_t matrix_storage(const linears_matrix_t *matrix,
                                      numeric_kind kind, size_t *count) {
    if (matrix == NULL || (uintptr_t)matrix % offsetof(struct basic_align_matrix, value) != 0)
        return LINEARS_ERROR_ARGUMENT;
    if (matrix->cols != 0 && matrix->rows > SIZE_MAX / matrix->cols)
        return LINEARS_ERROR_SIZE;
    *count = matrix->rows * matrix->cols;
    size_t width = element_size(kind);
    if (*count > SIZE_MAX / width) return LINEARS_ERROR_SIZE;
    if (*count != 0 && (matrix->data == NULL ||
        (uintptr_t)matrix->data % element_alignment(kind) != 0)) return LINEARS_ERROR_ARGUMENT;
    if (*count * width > UINTPTR_MAX - (uintptr_t)matrix->data) return LINEARS_ERROR_SIZE;
    return LINEARS_OK;
}

static linears_status_t load_fixed(const linears_matrix_t *a, size_t i,
                                  numeric_kind kind, linears_fixed_t *out) {
    int64_t raw = kind == Q15 ? ((const int16_t *)a->data)[i] : ((const int32_t *)a->data)[i];
    return linears_fixed_from_i64(raw, kind == Q15 ? 15 : 31, a->exponent, out);
}

static linears_status_t input_values(const linears_matrix_t *a, size_t count, numeric_kind kind) {
    if (kind == F32) {
        for (size_t i = 0; i < count; ++i)
            if (!isfinite(((const float *)a->data)[i])) return LINEARS_ERROR_RANGE;
    } else if (kind == F64) {
        for (size_t i = 0; i < count; ++i)
            if (!isfinite(((const double *)a->data)[i])) return LINEARS_ERROR_RANGE;
    } else {
        if (a->exponent < LINEARS_EXPONENT_MIN || a->exponent > LINEARS_EXPONENT_MAX)
            return LINEARS_ERROR_RANGE;
        for (size_t i = 0; i < count; ++i) {
            linears_fixed_t ignored;
            linears_status_t status = load_fixed(a, i, kind, &ignored);
            if (status != LINEARS_OK) return status;
        }
    }
    return LINEARS_OK;
}

/* Matrix data may overlap other data. Descriptors may be identical but may
 * neither partially overlap one another nor occupy an element buffer. */
static linears_status_t matrix_aliases(const linears_matrix_t *a, size_t acount,
                                      const linears_matrix_t *b, size_t bcount,
                                      const linears_matrix_t *out, size_t outcount,
                                      numeric_kind kind) {
    const linears_matrix_t *matrices[3] = {a, b, out};
    const size_t counts[3] = {acount, bcount, outcount};
    for (unsigned i = 0; i < 3; ++i) if (matrices[i] != NULL) {
        for (unsigned j = 0; j < 3; ++j) if (matrices[j] != NULL) {
            if (overlap(matrices[i], sizeof(*a), matrices[j]->data,
                        counts[j] * element_size(kind))) return LINEARS_ERROR_ALIAS;
            if (matrices[i] != matrices[j] &&
                overlap(matrices[i], sizeof(*a), matrices[j], sizeof(*a)))
                return LINEARS_ERROR_ALIAS;
        }
    }
    return LINEARS_OK;
}

static linears_status_t scratch_storage(linears_workspace_unit_t *workspace, size_t bytes,
                                       size_t count, const linears_matrix_t *a, size_t acount,
                                       const linears_matrix_t *b, size_t bcount,
                                       linears_matrix_t *out, numeric_kind kind) {
    if (count > SIZE_MAX / sizeof(*workspace)) return LINEARS_ERROR_SIZE;
    size_t needed = count * sizeof(*workspace);
    if (needed == 0) return LINEARS_OK;
    if (workspace == NULL || bytes < needed ||
        (uintptr_t)workspace % offsetof(struct basic_align_workspace, value) != 0)
        return LINEARS_ERROR_WORKSPACE;
    if (bytes > UINTPTR_MAX - (uintptr_t)workspace) return LINEARS_ERROR_SIZE;
    const linears_matrix_t *matrices[3] = {a, b, out};
    const size_t counts[3] = {acount, bcount, count};
    for (unsigned i = 0; i < 3; ++i) if (matrices[i] != NULL) {
        if (overlap(workspace, bytes, matrices[i], sizeof(*out)) ||
            overlap(workspace, bytes, matrices[i]->data, counts[i] * element_size(kind)))
            return LINEARS_ERROR_ALIAS;
    }
    return LINEARS_OK;
}

#define FLOAT_TYPE float
#define FLOAT_SUFFIX f32
#define FLOAT_MEMBER f32
#define FLOAT_ABS fabsf
#define FLOAT_SQRT sqrtf
#include "basic_float_impl.h"

#define FLOAT_TYPE double
#define FLOAT_SUFFIX f64
#define FLOAT_MEMBER floating
#define FLOAT_ABS fabs
#define FLOAT_SQRT sqrt
#include "basic_float_impl.h"

/* Uses the public scalar quantizers, but accesses each typed union member
 * directly instead of pretending that workspace is a linears_fixed_t array. */
static linears_status_t commit_result(linears_matrix_t *out, size_t count,
                                     linears_workspace_unit_t *workspace, numeric_kind kind) {
    if (kind == F32 || kind == F64) {
        for (size_t i = 0; i < count; ++i) {
            if (kind == F32) ((float *)out->data)[i] = workspace[i].f32;
            else ((double *)out->data)[i] = workspace[i].floating;
        }
        return LINEARS_OK;
    }
    int exponent = LINEARS_EXPONENT_MIN;
    int any = 0;
    for (size_t i = 0; i < count; ++i) {
        if (workspace[i].fixed.mantissa != 0) {
            any = 1;
            if (workspace[i].fixed.exponent > exponent) exponent = workspace[i].fixed.exponent;
        }
    }
    if (!any) exponent = 0;
    if (kind == Q15) {
        for (size_t i = 0; i < count; ++i) {
            int16_t ignored;
            if (linears_fixed_to_q15(workspace[i].fixed, exponent, &ignored) != LINEARS_OK) {
                if (exponent == LINEARS_EXPONENT_MAX) return LINEARS_ERROR_RANGE;
                ++exponent;
                break;
            }
        }
    }
    for (size_t i = 0; i < count; ++i) {
        if (kind == Q15)
            (void)linears_fixed_to_q15(workspace[i].fixed, exponent, &((int16_t *)out->data)[i]);
        else
            (void)linears_fixed_to_q31(workspace[i].fixed, exponent, &((int32_t *)out->data)[i]);
    }
    out->exponent = exponent;
    return LINEARS_OK;
}

static linears_status_t evaluate_fixed(matrix_op operation, numeric_kind kind,
                                      const linears_matrix_t *a, const linears_matrix_t *b,
                                      linears_matrix_t *out, linears_fixed_t alpha,
                                      linears_fixed_t beta, linears_workspace_unit_t *workspace) {
    for (size_t row = 0; row < out->rows; ++row) {
        for (size_t col = 0; col < out->cols; ++col) {
            size_t i = row * out->cols + col;
            linears_fixed_t result = {0, 0}, x, y, product;
            linears_status_t status = LINEARS_OK;
            if (operation == MUL || operation == MATVEC) {
                for (size_t k = 0; k < a->cols; ++k) {
                    (void)load_fixed(a, row * a->cols + k, kind, &x);
                    (void)load_fixed(b, k * b->cols + col, kind, &y);
                    status = linears_fixed_mul(x, y, &product);
                    if (status == LINEARS_OK) status = linears_fixed_add(result, product, &result);
                    if (status != LINEARS_OK) return status;
                }
            } else if (operation == TRANSPOSE) {
                (void)load_fixed(a, col * a->cols + row, kind, &result);
            } else {
                (void)load_fixed(a, i, kind, &x);
                if (b != NULL) (void)load_fixed(b, i, kind, &y);
                switch (operation) {
                    case ADD: status = linears_fixed_add(x, y, &result); break;
                    case SUB: status = linears_fixed_sub(x, y, &result); break;
                    case HADAMARD: status = linears_fixed_mul(x, y, &result); break;
                    case SCALE: status = linears_fixed_mul(alpha, x, &result); break;
                    default:
                        status = linears_fixed_mul(alpha, x, &product);
                        if (status == LINEARS_OK) status = linears_fixed_mul(beta, y, &result);
                        if (status == LINEARS_OK) status = linears_fixed_add(product, result, &result);
                        break;
                }
            }
            if (status != LINEARS_OK) return status;
            workspace[i].fixed = result;
        }
    }
    return LINEARS_OK;
}

static linears_status_t matrix_compute(matrix_op operation, numeric_kind kind,
                                      const linears_matrix_t *a, const linears_matrix_t *b,
                                      linears_matrix_t *out, linears_workspace_unit_t alpha,
                                      linears_workspace_unit_t beta,
                                      linears_workspace_unit_t *workspace, size_t workspace_bytes) {
    size_t acount, bcount = 0, outcount;
    linears_status_t status = matrix_storage(a, kind, &acount);
    if (status == LINEARS_OK && b != NULL) status = matrix_storage(b, kind, &bcount);
    if (status == LINEARS_OK) status = matrix_storage(out, kind, &outcount);
    if (status != LINEARS_OK) return status;
    int binary = operation == ADD || operation == SUB || operation == HADAMARD ||
                 operation == MUL || operation == MATVEC || operation == AXPBY;
    if (binary && b == NULL) return LINEARS_ERROR_ARGUMENT;
    if (operation == MUL || operation == MATVEC) {
        if (a->cols != b->rows || out->rows != a->rows || out->cols != b->cols ||
            (operation == MATVEC && b->cols != 1)) return LINEARS_ERROR_DIMENSION;
    } else if (operation == TRANSPOSE) {
        if (out->rows != a->cols || out->cols != a->rows) return LINEARS_ERROR_DIMENSION;
    } else {
        if (out->rows != a->rows || out->cols != a->cols ||
            (b != NULL && (b->rows != a->rows || b->cols != a->cols))) return LINEARS_ERROR_DIMENSION;
    }
    status = matrix_aliases(a, acount, b, bcount, out, outcount, kind);
    if (status == LINEARS_OK)
        status = scratch_storage(workspace, workspace_bytes, outcount, a, acount, b, bcount, out, kind);
    if (status == LINEARS_OK) status = input_values(a, acount, kind);
    if (status == LINEARS_OK && b != NULL) status = input_values(b, bcount, kind);
    if (status != LINEARS_OK) return status;
    if (outcount == 0) return commit_result(out, 0, workspace, kind);
    if (kind == F32)
        status = evaluate_f32(operation, a, b, out, alpha.f32, beta.f32, workspace);
    else if (kind == F64)
        status = evaluate_f64(operation, a, b, out, alpha.floating, beta.floating, workspace);
    else
        status = evaluate_fixed(operation, kind, a, b, out, alpha.fixed, beta.fixed, workspace);
    return status == LINEARS_OK ? commit_result(out, outcount, workspace, kind) : status;
}

static linears_status_t copy_matrix(numeric_kind kind, const linears_matrix_t *a,
                                   linears_matrix_t *out) {
    size_t acount, outcount;
    linears_status_t status = matrix_storage(a, kind, &acount);
    if (status == LINEARS_OK) status = matrix_storage(out, kind, &outcount);
    if (status != LINEARS_OK) return status;
    if (a->rows != out->rows || a->cols != out->cols) return LINEARS_ERROR_DIMENSION;
    status = matrix_aliases(a, acount, NULL, 0, out, outcount, kind);
    if (status == LINEARS_OK) status = input_values(a, acount, kind);
    if (status != LINEARS_OK) return status;
    if (acount != 0) memmove(out->data, a->data, acount * element_size(kind));
    if (kind == Q15 || kind == Q31) out->exponent = a->exponent;
    return LINEARS_OK;
}

static linears_status_t fill_matrix(numeric_kind kind, linears_matrix_t *out, int identity) {
    size_t count;
    linears_status_t status = matrix_storage(out, kind, &count);
    if (status != LINEARS_OK) return status;
    if (identity && out->rows != out->cols) return LINEARS_ERROR_DIMENSION;
    status = matrix_aliases(NULL, 0, NULL, 0, out, count, kind);
    if (status != LINEARS_OK) return status;
    for (size_t i = 0; i < count; ++i) {
        int diagonal = identity && i / out->cols == i % out->cols;
        switch (kind) {
            case F32: ((float *)out->data)[i] = diagonal ? 1.0f : 0.0f; break;
            case F64: ((double *)out->data)[i] = diagonal ? 1.0 : 0.0; break;
            case Q15: ((int16_t *)out->data)[i] = diagonal ? INT16_C(16384) : 0; break;
            case Q31: ((int32_t *)out->data)[i] = diagonal ? INT32_C(1073741824) : 0; break;
        }
    }
    if (kind == Q15 || kind == Q31) out->exponent = identity && count != 0 ? 1 : 0;
    return LINEARS_OK;
}

static linears_status_t reduce_fixed(reduction_op operation, numeric_kind kind,
                                    const linears_matrix_t *a, const linears_matrix_t *b,
                                    size_t count, linears_fixed_t *out) {
    linears_fixed_t sum = {0, 0}, scale = {0, 0};
    if (operation == NORM) {
        for (size_t i = 0; i < count; ++i) {
            linears_fixed_t x;
            (void)load_fixed(a, i, kind, &x);
            (void)linears_fixed_abs(x, &x);
            if (linears_fixed_compare(x, scale) > 0) scale = x;
        }
        if (scale.mantissa == 0) { *out = scale; return LINEARS_OK; }
    }
    for (size_t i = 0; i < count; ++i) {
        size_t index = operation == TRACE ? i * a->cols + i : i;
        linears_fixed_t x, y, term;
        (void)load_fixed(a, index, kind, &x);
        linears_status_t status = LINEARS_OK;
        if (operation == NORM) {
            if (x.mantissa == 0) continue;
            status = linears_fixed_div(x, scale, &y);
            if (status == LINEARS_OK) status = linears_fixed_mul(y, y, &term);
        } else if (operation == DOT) {
            (void)load_fixed(b, i, kind, &y);
            status = linears_fixed_mul(x, y, &term);
        } else term = x;
        if (status == LINEARS_OK) status = linears_fixed_add(sum, term, &sum);
        if (status != LINEARS_OK) return status;
    }
    if (operation == NORM) {
        linears_status_t status = linears_fixed_sqrt(sum, &sum);
        if (status == LINEARS_OK) status = linears_fixed_mul(scale, sum, &sum);
        if (status != LINEARS_OK) return status;
    }
    *out = sum;
    return LINEARS_OK;
}

static linears_status_t reduction(reduction_op operation, numeric_kind kind,
                                  const linears_matrix_t *a, const linears_matrix_t *b, void *out) {
    size_t acount, bcount = 0;
    linears_status_t status = matrix_storage(a, kind, &acount);
    if (status == LINEARS_OK && b != NULL) status = matrix_storage(b, kind, &bcount);
    if (status != LINEARS_OK) return status;
    if (operation == DOT && b == NULL) return LINEARS_ERROR_ARGUMENT;
    if ((operation == TRACE && a->rows != a->cols) ||
        (b != NULL && (a->rows != b->rows || a->cols != b->cols))) return LINEARS_ERROR_DIMENSION;
    size_t outsize = kind == Q15 ? sizeof(linears_q15_t) : kind == Q31 ? sizeof(linears_q31_t) : element_size(kind);
    size_t alignment = kind == Q15 ? offsetof(struct basic_align_scalar_q15, value) :
                       kind == Q31 ? offsetof(struct basic_align_scalar_q31, value) : element_alignment(kind);
    if (out == NULL || (uintptr_t)out % alignment != 0) return LINEARS_ERROR_ARGUMENT;
    if (overlap(out, outsize, a, sizeof(*a)) || (b != NULL && overlap(out, outsize, b, sizeof(*b))))
        return LINEARS_ERROR_ALIAS;
    status = matrix_aliases(a, acount, b, bcount, NULL, 0, kind);
    if (status == LINEARS_OK) status = input_values(a, acount, kind);
    if (status == LINEARS_OK && b != NULL) status = input_values(b, bcount, kind);
    if (status != LINEARS_OK) return status;
    size_t count = operation == TRACE ? a->rows : acount;
    if (kind == F32) {
        float value;
        status = reduce_f32(operation, a, b, count, &value);
        if (status == LINEARS_OK) *(float *)out = value;
    } else if (kind == F64) {
        double value;
        status = reduce_f64(operation, a, b, count, &value);
        if (status == LINEARS_OK) *(double *)out = value;
    } else {
        linears_fixed_t value;
        status = reduce_fixed(operation, kind, a, b, count, &value);
        if (status == LINEARS_OK && kind == Q15) {
            linears_q15_t result;
            status = linears_fixed_export_q15(&value, 1, &result.mantissa, &result.exponent);
            if (status == LINEARS_OK) *(linears_q15_t *)out = result;
        } else if (status == LINEARS_OK) {
            *(linears_q31_t *)out = value;
        }
    }
    return status;
}

static linears_workspace_unit_t empty_scalar(numeric_kind kind) {
    linears_workspace_unit_t value;
    if (kind == Q15 || kind == Q31) {
        value.fixed.mantissa = 0;
        value.fixed.exponent = 0;
    } else if (kind == F32) value.f32 = 0;
    else value.floating = 0;
    return value;
}

/* Public variants keep format selection out of the matrix descriptor. */
#define DEFINE_COMMON(SUFFIX, KIND, SCALAR) \
linears_status_t linears_add_##SUFFIX(const linears_matrix_t *a, const linears_matrix_t *b, linears_matrix_t *out, linears_workspace_unit_t *w, size_t bytes) { \
    linears_workspace_unit_t unused = empty_scalar(KIND); return matrix_compute(ADD, KIND, a, b, out, unused, unused, w, bytes); } \
linears_status_t linears_sub_##SUFFIX(const linears_matrix_t *a, const linears_matrix_t *b, linears_matrix_t *out, linears_workspace_unit_t *w, size_t bytes) { \
    linears_workspace_unit_t unused = empty_scalar(KIND); return matrix_compute(SUB, KIND, a, b, out, unused, unused, w, bytes); } \
linears_status_t linears_hadamard_##SUFFIX(const linears_matrix_t *a, const linears_matrix_t *b, linears_matrix_t *out, linears_workspace_unit_t *w, size_t bytes) { \
    linears_workspace_unit_t unused = empty_scalar(KIND); return matrix_compute(HADAMARD, KIND, a, b, out, unused, unused, w, bytes); } \
linears_status_t linears_mul_##SUFFIX(const linears_matrix_t *a, const linears_matrix_t *b, linears_matrix_t *out, linears_workspace_unit_t *w, size_t bytes) { \
    linears_workspace_unit_t unused = empty_scalar(KIND); return matrix_compute(MUL, KIND, a, b, out, unused, unused, w, bytes); } \
linears_status_t linears_matvec_##SUFFIX(const linears_matrix_t *a, const linears_matrix_t *b, linears_matrix_t *out, linears_workspace_unit_t *w, size_t bytes) { \
    linears_workspace_unit_t unused = empty_scalar(KIND); return matrix_compute(MATVEC, KIND, a, b, out, unused, unused, w, bytes); } \
linears_status_t linears_transpose_##SUFFIX(const linears_matrix_t *a, linears_matrix_t *out, linears_workspace_unit_t *w, size_t bytes) { \
    linears_workspace_unit_t unused = empty_scalar(KIND); return matrix_compute(TRANSPOSE, KIND, a, NULL, out, unused, unused, w, bytes); } \
linears_status_t linears_copy_##SUFFIX(const linears_matrix_t *a, linears_matrix_t *out) { return copy_matrix(KIND, a, out); } \
linears_status_t linears_zero_##SUFFIX(linears_matrix_t *out) { return fill_matrix(KIND, out, 0); } \
linears_status_t linears_identity_##SUFFIX(linears_matrix_t *out) { return fill_matrix(KIND, out, 1); } \
linears_status_t linears_dot_##SUFFIX(const linears_matrix_t *a, const linears_matrix_t *b, SCALAR *out) { return reduction(DOT, KIND, a, b, out); } \
linears_status_t linears_norm_##SUFFIX(const linears_matrix_t *a, SCALAR *out) { return reduction(NORM, KIND, a, NULL, out); } \
linears_status_t linears_trace_##SUFFIX(const linears_matrix_t *a, SCALAR *out) { return reduction(TRACE, KIND, a, NULL, out); }

DEFINE_COMMON(f32, F32, float)
DEFINE_COMMON(f64, F64, double)
DEFINE_COMMON(q15, Q15, linears_q15_t)
DEFINE_COMMON(q31, Q31, linears_q31_t)

#define DEFINE_FLOAT_SCALE(SUFFIX, KIND, TYPE, MEMBER) \
linears_status_t linears_axpby_##SUFFIX(TYPE alpha, const linears_matrix_t *a, TYPE beta, const linears_matrix_t *b, linears_matrix_t *out, linears_workspace_unit_t *w, size_t bytes) { \
    if (!isfinite(alpha) || !isfinite(beta)) return LINEARS_ERROR_RANGE; \
    linears_workspace_unit_t x, y; x.MEMBER = alpha; y.MEMBER = beta; \
    return matrix_compute(AXPBY, KIND, a, b, out, x, y, w, bytes); } \
linears_status_t linears_scale_##SUFFIX(TYPE alpha, const linears_matrix_t *a, linears_matrix_t *out, linears_workspace_unit_t *w, size_t bytes) { \
    if (!isfinite(alpha)) return LINEARS_ERROR_RANGE; \
    linears_workspace_unit_t x; x.MEMBER = alpha; \
    return matrix_compute(SCALE, KIND, a, NULL, out, x, x, w, bytes); }

DEFINE_FLOAT_SCALE(f32, F32, float, f32)
DEFINE_FLOAT_SCALE(f64, F64, double, floating)

#define DEFINE_FIXED_SCALE(SUFFIX, KIND, TYPE) \
linears_status_t linears_axpby_##SUFFIX(TYPE alpha, const linears_matrix_t *a, TYPE beta, const linears_matrix_t *b, linears_matrix_t *out, linears_workspace_unit_t *w, size_t bytes) { \
    linears_workspace_unit_t x, y; \
    linears_status_t status = linears_fixed_from_##SUFFIX(alpha, &x.fixed); \
    if (status == LINEARS_OK) status = linears_fixed_from_##SUFFIX(beta, &y.fixed); \
    return status == LINEARS_OK ? matrix_compute(AXPBY, KIND, a, b, out, x, y, w, bytes) : status; } \
linears_status_t linears_scale_##SUFFIX(TYPE alpha, const linears_matrix_t *a, linears_matrix_t *out, linears_workspace_unit_t *w, size_t bytes) { \
    linears_workspace_unit_t x; linears_status_t status = linears_fixed_from_##SUFFIX(alpha, &x.fixed); \
    return status == LINEARS_OK ? matrix_compute(SCALE, KIND, a, NULL, out, x, x, w, bytes) : status; }

DEFINE_FIXED_SCALE(q15, Q15, linears_q15_t)
DEFINE_FIXED_SCALE(q31, Q31, linears_q31_t)
