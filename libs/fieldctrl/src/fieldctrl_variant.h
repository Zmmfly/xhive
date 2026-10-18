/* Intentionally no include guard: select FC_VARIANT=32,64,15,31 before each
 * inclusion, then include fieldctrl_variant_end.h after the implementation. */
#include "fieldctrl_arith.h"
#define FC_CAT_(a, b) a##b
#define FC_CAT(a, b) FC_CAT_(a, b)
#if FC_VARIANT == 32
#define FC_T float
#define FC_W float
#define FC_PREFIX fieldctrl_f32_
#define FC_A fc32
#define FC_L fc32
#define FC_SUFFIX f32
#elif FC_VARIANT == 64
#define FC_T double
#define FC_W double
#define FC_PREFIX fieldctrl_f64_
#define FC_A fc64
#define FC_L fc64
#define FC_SUFFIX f64
#elif FC_VARIANT == 15
#define FC_T linears_q15_t
#define FC_W linears_fixed_t
#define FC_PREFIX fieldctrl_q15_
#define FC_A fcq
#define FC_L fcq15
#define FC_SUFFIX q15
#elif FC_VARIANT == 31
#define FC_T linears_q31_t
#define FC_W linears_fixed_t
#define FC_PREFIX fieldctrl_q31_
#define FC_A fcq
#define FC_L fcq31
#define FC_SUFFIX q31
#else
#error Invalid fieldctrl numeric variant
#endif
#define FC_N(n) FC_CAT(FC_PREFIX, n)
#define FC_TYPE(n) FC_N(FC_CAT(n, _t))
#define FC_LOAD(x) FC_CAT(FC_L, _load)((x), &status)
#define FC_STORE(x, p) FC_CAT(FC_L, _store)((x), (p), &status)
#define FC_I(x) FC_CAT(FC_A, _i)((x), &status)
#define FC_ADD(a,b) FC_CAT(FC_A, _add)((a), (b), &status)
#define FC_SUB(a,b) FC_CAT(FC_A, _sub)((a), (b), &status)
#define FC_MUL(a,b) FC_CAT(FC_A, _mul)((a), (b), &status)
#define FC_DIV(a,b) FC_CAT(FC_A, _div)((a), (b), &status)
#define FC_RATIO(n,d) FC_DIV(FC_I(n), FC_I(d))
#define FC_NEG(a) FC_CAT(FC_A, _neg)((a), &status)
#define FC_ABS(a) FC_CAT(FC_A, _abs)((a), &status)
#define FC_SQRT(a) FC_CAT(FC_A, _sqrt)((a), &status)
#define FC_HYPOT(a,b) FC_CAT(FC_A, _hypot)((a), (b), &status)
#define FC_RESCALE(a,n,d) FC_CAT(FC_A, _rescale)((a), (n), (d), &status)
#define FC_CMP(a,b) FC_CAT(FC_A, _cmp)((a), (b))
#define FC_WRAP(a) FC_CAT(FC_A, _wrap)((a), &status)
#define FC_SINCOS(a,s,c) FC_CAT(FC_A, _sincos)((a), (s), (c), &status)
#define FC_ATAN2(y,x) FC_CAT(FC_A, _atan2)((y), (x), &status)
#define FC_ACC_T FC_W
#define FC_COEFF_T FC_W
#if FC_VARIANT == 15 || FC_VARIANT == 31
#define FC_ACC_LOAD(x) fcq_work_load((x), &status)
#define FC_ACC_STORE(x,p) fcq_work_store((x), (p), &status)
#else
#define FC_ACC_LOAD(x) FC_LOAD(x)
#define FC_ACC_STORE(x,p) FC_STORE((x),(p))
#endif
#define FC_COEFF_LOAD(x) FC_ACC_LOAD(x)
#if FC_VARIANT == 15
/* Circular nearest rounding: the upper half-cell is zero turns, not +MAX. */
#define FC_ANGLE_STORE(x,p) do { \
    FC_W fc_angle_work = FC_WRAP(x); \
    if (status == LINEARS_OK) { \
        if (fc_angle_work >= LINEARS_FIXED_ONE - INT64_C(32768)) *(p) = 0; \
        else FC_STORE(fc_angle_work, (p)); \
    } \
} while (0)
#else
#define FC_ANGLE_STORE(x,p) do { \
    FC_T fc_angle_temp; \
    FC_W fc_angle_work = FC_WRAP(x); \
    FC_STORE(fc_angle_work, &fc_angle_temp); \
    if (status == LINEARS_OK) { \
        fc_angle_work = FC_LOAD(fc_angle_temp); \
        if (FC_CMP(fc_angle_work, FC_I(1)) >= 0) FC_STORE(FC_I(0), &fc_angle_temp); \
        if (status == LINEARS_OK) *(p) = fc_angle_temp; \
    } \
} while (0)
#endif
