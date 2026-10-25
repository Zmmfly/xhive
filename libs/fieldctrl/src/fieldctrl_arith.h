/* Internal arithmetic glue. q variants delegate all scalar arithmetic to
 * linears; CORDIC supplies only turn reduction and fixed-count rotations. */
#ifndef FIELDCTRL_ARITH_H
#define FIELDCTRL_ARITH_H

#include <math.h>
#include <stdint.h>
#include <stddef.h>
#include "linears_numeric.h"

static inline int fc_overlap(const void *a, size_t na, const void *b, size_t nb) {
    uintptr_t x = (uintptr_t)a, y = (uintptr_t)b;
    return x <= y ? y - x < na : x - y < nb;
}

#define FC_FLOAT_HELPERS(P, T, SQRT, FLOOR, HYPOT, SIN, COS, ATAN2, FREXP, LDEXP) \
    static inline T P##_load(T x, linears_status_t *s) { \
        if (*s != LINEARS_OK) return (T)0; \
        if (!isfinite(x)) { *s = LINEARS_ERROR_RANGE; return (T)0; } \
        return x; \
    } \
    static inline void P##_store(T x, T *out, linears_status_t *s) { \
        x = P##_load(x, s); if (*s == LINEARS_OK) *out = x; \
    } \
    static inline T P##_i(int64_t x, linears_status_t *s) { \
        return P##_load((T)x, s); \
    } \
    static inline T P##_add(T a, T b, linears_status_t *s) { \
        return *s == LINEARS_OK ? P##_load(a + b, s) : (T)0; \
    } \
    static inline T P##_sub(T a, T b, linears_status_t *s) { \
        return *s == LINEARS_OK ? P##_load(a - b, s) : (T)0; \
    } \
    static inline T P##_mul(T a, T b, linears_status_t *s) { \
        return *s == LINEARS_OK ? P##_load(a * b, s) : (T)0; \
    } \
    static inline T P##_div(T a, T b, linears_status_t *s) { \
        if (*s != LINEARS_OK) return (T)0; \
        if (b == (T)0) { *s = LINEARS_ERROR_DOMAIN; return (T)0; } \
        return P##_load(a / b, s); \
    } \
    static inline T P##_neg(T a, linears_status_t *s) { return P##_load(-a, s); } \
    static inline T P##_abs(T a, linears_status_t *s) { return P##_load(a < 0 ? -a : a, s); } \
    static inline T P##_sqrt(T a, linears_status_t *s) { \
        if (*s != LINEARS_OK) return (T)0; \
        if (a < (T)0) { *s = LINEARS_ERROR_DOMAIN; return (T)0; } \
        return P##_load(SQRT(a), s); \
    } \
    static inline T P##_hypot(T a, T b, linears_status_t *s) { \
        return *s == LINEARS_OK ? P##_load(HYPOT(a,b), s) : (T)0; \
    } \
    static inline T P##_rescale(T a, T numerator, T denominator, linears_status_t *s) { \
        int ea, en, ed; \
        T ma, mn, md; \
        if (*s != LINEARS_OK) return (T)0; \
        /* The caller supplies a positive nonzero magnitude denominator. \
         * Only the final exponent is applied, so neither multiplication \
         * nor division order discards a representable limited component. */ \
        ma = FREXP(a, &ea); mn = FREXP(numerator, &en); md = FREXP(denominator, &ed); \
        return P##_load(LDEXP((ma * mn) / md, ea + en - ed), s); \
    } \
    static inline int P##_cmp(T a, T b) { return (a > b) - (a < b); } \
    static inline T P##_wrap(T x, linears_status_t *s) { \
        if (*s != LINEARS_OK) return (T)0; \
        x -= FLOOR(x); return x == (T)0 || x >= (T)1 ? (T)0 : x; \
    } \
    static inline void P##_sincos(T a, T *sn, T *cs, linears_status_t *s) { \
        T w = P##_wrap(a, s), r, ss, cc; \
        int quadrant; \
        if (*s != LINEARS_OK) return; \
        /* Reduce turns before multiplying by pi: |r| <= pi/4, avoiding \
         * the f32 argument-quantization error of a full [0,2*pi) angle. */ \
        quadrant = (int)(w * (T)4 + (T)0.5); \
        r = (w - (T)quadrant * (T)0.25) * (T)6.283185307179586476925286766559; \
        ss = SIN(r); cc = COS(r); \
        switch (quadrant & 3) { \
        case 0: *sn = ss; *cs = cc; break; \
        case 1: *sn = cc; *cs = -ss; break; \
        case 2: *sn = -ss; *cs = -cc; break; \
        default: *sn = -cc; *cs = ss; break; \
        } \
        if (*sn == (T)0) *sn = (T)0; \
        if (*cs == (T)0) *cs = (T)0; \
    } \
    static inline T P##_atan2(T y, T x, linears_status_t *s) { \
        if (*s != LINEARS_OK) return (T)0; \
        if (x == (T)0 && y == (T)0) { *s = LINEARS_ERROR_DOMAIN; return (T)0; } \
        return P##_wrap(ATAN2(y,x) / (T)6.283185307179586476925286766559, s); \
    }
FC_FLOAT_HELPERS(fc32, float, sqrtf, floorf, hypotf, sinf, cosf, atan2f, frexpf, ldexpf)
FC_FLOAT_HELPERS(fc64, double, sqrt, floor, hypot, sin, cos, atan2, frexp, ldexp)
#undef FC_FLOAT_HELPERS

static inline linears_fixed_t fcq_zero(void) { return 0; }
#define FC_Q_BINARY(N) \
    static inline linears_fixed_t fcq_##N(linears_fixed_t a, linears_fixed_t b, linears_status_t *s) { \
        linears_fixed_t r = 0; \
        if (*s == LINEARS_OK) { *s = linears_fixed_##N(a, b, &r); } return r; \
    }
FC_Q_BINARY(add)
FC_Q_BINARY(sub)
FC_Q_BINARY(mul)
FC_Q_BINARY(div)
#undef FC_Q_BINARY
#define FC_Q_UNARY(N) \
    static inline linears_fixed_t fcq_##N(linears_fixed_t a, linears_status_t *s) { \
        linears_fixed_t r = 0; \
        if (*s == LINEARS_OK) { *s = linears_fixed_##N(a, &r); } return r; \
    }
FC_Q_UNARY(neg)
FC_Q_UNARY(abs)
FC_Q_UNARY(sqrt)
#undef FC_Q_UNARY
static inline linears_fixed_t fcq_i(int64_t x, linears_status_t *s) {
    linears_fixed_t r = 0;
    if (*s == LINEARS_OK) *s = linears_fixed_from_int(x, &r);
    return r;
}
static inline linears_fixed_t fcq15_load(linears_q15_t x, linears_status_t *s) {
    return *s == LINEARS_OK ? linears_fixed_from_q15(x) : 0;
}
static inline linears_fixed_t fcq31_load(linears_q31_t x, linears_status_t *s) {
    return *s == LINEARS_OK ? linears_fixed_from_q31(x) : 0;
}
static inline linears_fixed_t fcq_work_load(linears_fixed_t x, linears_status_t *s) {
    return *s == LINEARS_OK ? x : 0;
}
static inline void fcq_work_store(linears_fixed_t x, linears_fixed_t *out, linears_status_t *s) {
    if (*s == LINEARS_OK) *out = x;
}
static inline void fcq15_store(linears_fixed_t x, linears_q15_t *out, linears_status_t *s) {
    if (*s == LINEARS_OK) *s = linears_fixed_to_q15(x, out);
}
static inline void fcq31_store(linears_fixed_t x, linears_q31_t *out, linears_status_t *s) {
    if (*s == LINEARS_OK) *s = linears_fixed_to_q31(x, out);
}
static inline int fcq_cmp(linears_fixed_t a, linears_fixed_t b) {
    return linears_fixed_compare(a, b);
}
/* Compute a*numerator/denominator in the shared exact wide-product core. */
static inline linears_fixed_t fcq_rescale(linears_fixed_t a, linears_fixed_t numerator,
                                         linears_fixed_t denominator, linears_status_t *s) {
    linears_fixed_t r = 0;
    if (*s == LINEARS_OK) *s = linears_fixed_muldiv(a, numerator, denominator, &r);
    return r;
}
/* Ratios stay in [0,1]; small ratios round at the invariant F31 binary point. */
static inline linears_fixed_t fcq_hypot(linears_fixed_t a, linears_fixed_t b, linears_status_t *s) {
    linears_fixed_t aa = fcq_abs(a, s), bb = fcq_abs(b, s);
    linears_fixed_t hi = fcq_cmp(aa, bb) >= 0 ? aa : bb;
    linears_fixed_t lo = fcq_cmp(aa, bb) >= 0 ? bb : aa;
    linears_fixed_t ratio, sum, root;
    if (hi == 0 || *s != LINEARS_OK) return 0;
    ratio = fcq_div(lo, hi, s);
    sum = fcq_mul(ratio, ratio, s);
    sum = fcq_add(LINEARS_FIXED_ONE, sum, s);
    root = fcq_sqrt(sum, s);
    return fcq_mul(hi, root, s);
}

/* Internal CORDIC services: bounded integer-only operations, no allocation. */
linears_fixed_t fcq_wrap(linears_fixed_t x, linears_status_t *status);
void fcq_sincos(linears_fixed_t a, linears_fixed_t *sn, linears_fixed_t *cs,
               linears_status_t *status);
linears_fixed_t fcq_atan2(linears_fixed_t y, linears_fixed_t x,
                        linears_status_t *status);

#endif
