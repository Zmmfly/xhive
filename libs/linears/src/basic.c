#include "linears.h"
#include "linears_numeric.h"
#include "linears_work.h"
#include "numeric_internal.h"

#include <float.h>
#include <math.h>
#include <string.h>

/* C99 alignment probes. Scratch is an array of actual union objects: each
 * element is accessed through its member, never as a reinterpreted scalar array. */
struct basic_align_f32 { char padding; float value; };
struct basic_align_f64 { char padding; double value; };
struct basic_align_q15 { char padding; int16_t value; };
struct basic_align_q31 { char padding; int32_t value; };
struct basic_align_work { char padding; int64_t value; };
struct basic_align_matrix { char padding; linears_matrix_t value; };
struct basic_align_workspace { char padding; linears_workspace_unit_t value; };

/* The raw-product accumulator proof covers both supported size_t widths. */
typedef char linears_size_t_fits_64[(sizeof(size_t) <= sizeof(uint64_t)) ? 1 : -1];

typedef enum { F32, F64, Q15, Q31, WORK } numeric_kind;
typedef enum { ADD, SUB, HADAMARD, MUL, MATVEC, TRANSPOSE, AXPBY, SCALE } matrix_op;
typedef enum { DOT, NORM, TRACE } reduction_op;

static size_t element_size(numeric_kind kind) {
    switch (kind) {
        case F32: return sizeof(float);
        case F64: return sizeof(double);
        case Q15: return sizeof(int16_t);
        case Q31: return sizeof(int32_t);
        default: return sizeof(int64_t);
    }
}

static size_t element_alignment(numeric_kind kind) {
    switch (kind) {
        case F32: return offsetof(struct basic_align_f32, value);
        case F64: return offsetof(struct basic_align_f64, value);
        case Q15: return offsetof(struct basic_align_q15, value);
        case Q31: return offsetof(struct basic_align_q31, value);
        default: return offsetof(struct basic_align_work, value);
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

static linears_status_t input_values(const linears_matrix_t *a, size_t count, numeric_kind kind) {
    if (kind == F32) {
        for (size_t i = 0; i < count; ++i)
            if (!isfinite(((const float *)a->data)[i])) return LINEARS_ERROR_RANGE;
    } else if (kind == F64) {
        for (size_t i = 0; i < count; ++i)
            if (!isfinite(((const double *)a->data)[i])) return LINEARS_ERROR_RANGE;
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

#define Q_TYPE int16_t
#define Q_BITS 15
#define Q_SUFFIX q15
#include "basic_q_impl.h"

#define Q_TYPE int32_t
#define Q_BITS 31
#define Q_SUFFIX q31
#include "basic_q_impl.h"

/* Every staged Q element has already passed exact range checking and its sole
 * quantization. Committing typed members cannot introduce a new failure. */
static void commit_result(linears_matrix_t *out, size_t count,
                          linears_workspace_unit_t *workspace, numeric_kind kind) {
    for (size_t i = 0; i < count; ++i) {
        switch (kind) {
            case F32: ((float *)out->data)[i] = workspace[i].f32; break;
            case F64: ((double *)out->data)[i] = workspace[i].floating; break;
            case Q15: ((int16_t *)out->data)[i] = workspace[i].q15; break;
            case Q31: ((int32_t *)out->data)[i] = workspace[i].q31; break;
            case WORK: ((int64_t *)out->data)[i] = workspace[i].fixed; break;
        }
    }
}

static linears_status_t evaluate_work(matrix_op operation,
                                     const linears_matrix_t *a, const linears_matrix_t *b,
                                     linears_matrix_t *out, linears_fixed_t alpha,
                                     linears_fixed_t beta, linears_workspace_unit_t *workspace) {
    const int64_t *adata = (const int64_t *)a->data;
    const int64_t *bdata = b == NULL ? NULL : (const int64_t *)b->data;
    for (size_t row = 0; row < out->rows; ++row) {
        for (size_t col = 0; col < out->cols; ++col) {
            size_t i = row * out->cols + col;
            linears_status_t status;
            if (operation == TRANSPOSE) {
                workspace[i].fixed = adata[col * a->cols + row];
                continue;
            }
            if (operation == ADD || operation == SUB) {
                status = operation == ADD ? linears_fixed_add(adata[i], bdata[i], &workspace[i].fixed)
                                          : linears_fixed_sub(adata[i], bdata[i], &workspace[i].fixed);
            } else {
                linears_accum_t sum = {0, 0};
                if (operation == MUL) {
                    for (size_t k = 0; k < a->cols; ++k) {
                        status = ln_add_checked(sum, ln_product(adata[row * a->cols + k],
                                                               bdata[k * b->cols + col]), &sum);
                        if (status != LINEARS_OK) return status;
                    }
                } else {
                    status = ln_add_checked(ln_product(alpha, adata[i]), ln_product(beta, bdata[i]), &sum);
                    if (status != LINEARS_OK) return status;
                }
                status = linears_accum_to_fixed(sum, &workspace[i].fixed);
            }
            if (status != LINEARS_OK) return status;
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
    if (outcount == 0) return LINEARS_OK;
    if (kind == F32)
        status = evaluate_f32(operation, a, b, out, alpha.f32, beta.f32, workspace);
    else if (kind == F64)
        status = evaluate_f64(operation, a, b, out, alpha.floating, beta.floating, workspace);
    else if (kind == Q15)
        status = evaluate_q15(operation, a, b, out, alpha.fixed, beta.fixed, workspace);
    else if (kind == Q31)
        status = evaluate_q31(operation, a, b, out, alpha.fixed, beta.fixed, workspace);
    else
        status = evaluate_work(operation, a, b, out, alpha.fixed, beta.fixed, workspace);
    if (status == LINEARS_OK) commit_result(out, outcount, workspace, kind);
    return status;
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
            case Q15: ((int16_t *)out->data)[i] = diagonal ? INT16_MAX : 0; break;
            case Q31: ((int32_t *)out->data)[i] = diagonal ? INT32_MAX : 0; break;
            case WORK: ((int64_t *)out->data)[i] = diagonal ? LINEARS_FIXED_ONE : 0; break;
        }
    }
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
    size_t outsize = element_size(kind);
    size_t alignment = element_alignment(kind);
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
    } else if (kind == Q15) status = reduce_q15(operation, a, b, count, (int16_t *)out);
    else status = reduce_q31(operation, a, b, count, (int32_t *)out);
    return status;
}

static linears_workspace_unit_t empty_scalar(numeric_kind kind) {
    linears_workspace_unit_t value;
    if (kind == Q15 || kind == Q31 || kind == WORK) value.fixed = 0;
    else if (kind == F32) value.f32 = 0;
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

#define DEFINE_FIXED_SCALE(SUFFIX, KIND) \
linears_status_t linears_axpby_##SUFFIX(linears_fixed_t alpha, const linears_matrix_t *a, linears_fixed_t beta, const linears_matrix_t *b, linears_matrix_t *out, linears_workspace_unit_t *w, size_t bytes) { \
    linears_workspace_unit_t x, y; x.fixed = alpha; y.fixed = beta; \
    return matrix_compute(AXPBY, KIND, a, b, out, x, y, w, bytes); } \
linears_status_t linears_scale_##SUFFIX(linears_fixed_t alpha, const linears_matrix_t *a, linears_matrix_t *out, linears_workspace_unit_t *w, size_t bytes) { \
    linears_workspace_unit_t x; x.fixed = alpha; \
    return matrix_compute(SCALE, KIND, a, NULL, out, x, x, w, bytes); }

DEFINE_FIXED_SCALE(q15, Q15)
DEFINE_FIXED_SCALE(q31, Q31)

linears_status_t linears_add_fixed(const linears_matrix_t *a, const linears_matrix_t *b,
                                 linears_matrix_t *out, linears_workspace_unit_t *workspace,
                                 size_t workspace_bytes) {
    linears_workspace_unit_t unused = empty_scalar(WORK);
    return matrix_compute(ADD, WORK, a, b, out, unused, unused, workspace, workspace_bytes);
}

linears_status_t linears_sub_fixed(const linears_matrix_t *a, const linears_matrix_t *b,
                                 linears_matrix_t *out, linears_workspace_unit_t *workspace,
                                 size_t workspace_bytes) {
    linears_workspace_unit_t unused = empty_scalar(WORK);
    return matrix_compute(SUB, WORK, a, b, out, unused, unused, workspace, workspace_bytes);
}

linears_status_t linears_mul_fixed(const linears_matrix_t *a, const linears_matrix_t *b,
                                 linears_matrix_t *out, linears_workspace_unit_t *workspace,
                                 size_t workspace_bytes) {
    linears_workspace_unit_t unused = empty_scalar(WORK);
    return matrix_compute(MUL, WORK, a, b, out, unused, unused, workspace, workspace_bytes);
}

linears_status_t linears_transpose_fixed(const linears_matrix_t *a, linears_matrix_t *out,
                                       linears_workspace_unit_t *workspace, size_t workspace_bytes) {
    linears_workspace_unit_t unused = empty_scalar(WORK);
    return matrix_compute(TRANSPOSE, WORK, a, NULL, out, unused, unused, workspace, workspace_bytes);
}

linears_status_t linears_axpby_fixed(linears_fixed_t alpha, const linears_matrix_t *a,
                                   linears_fixed_t beta, const linears_matrix_t *b,
                                   linears_matrix_t *out, linears_workspace_unit_t *workspace,
                                   size_t workspace_bytes) {
    linears_workspace_unit_t x, y;
    x.fixed = alpha;
    y.fixed = beta;
    return matrix_compute(AXPBY, WORK, a, b, out, x, y, workspace, workspace_bytes);
}

linears_status_t linears_identity_fixed(linears_matrix_t *out) {
    return fill_matrix(WORK, out, 1);
}
