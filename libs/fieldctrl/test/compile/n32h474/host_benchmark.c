/* Host-only before/after measurements, not MCU cycles or a numerical test suite.
 * BENCH_BFP selects only the old/new public representation adapters. Timed
 * workloads, values, iterations and postconditions are otherwise identical.
 * Build library objects separately with -O2 -fstrict-aliasing, without LTO or
 * sanitizers. Initialization, validation and printing are outside each timer.
 */
#define _POSIX_C_SOURCE 200809L
#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "linears.h"
#include "linears_numeric.h"
#include "filters.h"
#include "fieldctrl_base.h"

#ifndef BENCH_BFP
#error Define BENCH_BFP as 0 (raw fixed-Q) or 1 (old BFP)
#endif

#if BENCH_BFP
#define MODE_NAME "bfp"
typedef linears_q15_t coefficient_q15_t;
typedef linears_q31_t coefficient_q31_t;
#define DEFINE_ADAPTER(suffix, raw_type, fraction) \
static linears_##suffix##_t sample_##suffix(raw_type raw) { \
    linears_##suffix##_t result = {raw, 0}; return result; \
} \
static coefficient_##suffix##_t coefficient_##suffix(unsigned shift) { \
    return sample_##suffix((raw_type)(UINT64_C(1) << ((fraction) - shift))); \
} \
static long double value_##suffix(linears_##suffix##_t value) { \
    return ldexpl((long double)value.mantissa, value.exponent - (fraction)); \
} \
static long double matrix_value_##suffix(const linears_matrix_t *matrix, size_t index) { \
    return ldexpl((long double)((const raw_type *)matrix->data)[index], matrix->exponent - (fraction)); \
}
static linears_matrix_t matrix_view(void *data, size_t rows, size_t cols) {
    linears_matrix_t result = {data, rows, cols, 0};
    return result;
}
#else
#define MODE_NAME "fixed"
typedef linears_fixed_t coefficient_q15_t;
typedef linears_fixed_t coefficient_q31_t;
#define DEFINE_ADAPTER(suffix, raw_type, fraction) \
static linears_##suffix##_t sample_##suffix(raw_type raw) { return raw; } \
static coefficient_##suffix##_t coefficient_##suffix(unsigned shift) { \
    return (linears_fixed_t)(UINT64_C(1) << (31 - shift)); \
} \
static long double value_##suffix(linears_##suffix##_t value) { \
    return ldexpl((long double)value, -(fraction)); \
} \
static long double matrix_value_##suffix(const linears_matrix_t *matrix, size_t index) { \
    return ldexpl((long double)((const raw_type *)matrix->data)[index], -(fraction)); \
}
static linears_matrix_t matrix_view(void *data, size_t rows, size_t cols) {
    linears_matrix_t result = {data, rows, cols};
    return result;
}
#endif

DEFINE_ADAPTER(q15, int16_t, 15)
DEFINE_ADAPTER(q31, int32_t, 31)
#undef DEFINE_ADAPTER

enum workload { MATRIX4, MATRIX16, DOT256, FIR16, PI_STEP, VVVF_STEP };
struct observation { uint64_t elapsed_ns; long double value; int status; int valid; };

static uint64_t clock_ns(void) {
    struct timespec value;
    if (clock_gettime(CLOCK_MONOTONIC, &value) != 0) {
        perror("clock_gettime");
        exit(EXIT_FAILURE);
    }
    return (uint64_t)value.tv_sec * UINT64_C(1000000000) + (uint64_t)value.tv_nsec;
}

#define REQUIRE(call) do { \
    result.status = (call); if (result.status != LINEARS_OK) return result; \
} while (0)
#define TIMED(call) do { \
    const uint64_t begin = clock_ns(); \
    for (size_t iteration = 0; iteration < iterations; ++iteration) { \
        result.status = (call); \
        if (result.status != LINEARS_OK) break; \
    } \
    result.elapsed_ns = clock_ns() - begin; \
    if (result.status != LINEARS_OK) return result; \
} while (0)

#define DEFINE_BENCHMARK(suffix, raw_type, fraction) \
static struct observation benchmark_##suffix(enum workload which, size_t iterations) { \
    struct observation result = {0, 0, LINEARS_OK, 0}; \
    raw_type a_data[256], b_data[256], output[256] = {0}; \
    linears_workspace_unit_t workspace[LINEARS_WORKSPACE_UNITS(LINEARS_BASIC_WORKSPACE_BYTES(16, 16))]; \
    const raw_type sixteenth = (raw_type)(UINT64_C(1) << ((fraction) - 4)); \
    const raw_type sixty_fourth = (raw_type)(UINT64_C(1) << ((fraction) - 6)); \
    for (size_t i = 0; i < 256; ++i) { a_data[i] = sixteenth; b_data[i] = sixteenth; } \
    if (which == MATRIX4 || which == MATRIX16) { \
        const size_t n = which == MATRIX4 ? 4 : 16; \
        linears_matrix_t a = matrix_view(a_data, n, n); \
        linears_matrix_t b = matrix_view(b_data, n, n); \
        linears_matrix_t out = matrix_view(output, n, n); \
        for (size_t i = 0; i < 32; ++i) REQUIRE(linears_mul_##suffix(&a, &b, &out, workspace, sizeof(workspace))); \
        TIMED(linears_mul_##suffix(&a, &b, &out, workspace, sizeof(workspace))); \
        const long double expected = (long double)n / 256.0L; \
        result.valid = 1; \
        for (size_t i = 0; i < n * n; ++i) { \
            if (matrix_value_##suffix(&out, i) != expected) result.valid = 0; \
        } \
        result.value = matrix_value_##suffix(&out, 0); \
    } else if (which == DOT256) { \
        for (size_t i = 0; i < 256; ++i) { a_data[i] = sixty_fourth; b_data[i] = sixty_fourth; } \
        linears_matrix_t a = matrix_view(a_data, 1, 256); \
        linears_matrix_t b = matrix_view(b_data, 1, 256); \
        linears_##suffix##_t out = sample_##suffix(0); \
        for (size_t i = 0; i < 32; ++i) REQUIRE(linears_dot_##suffix(&a, &b, &out)); \
        TIMED(linears_dot_##suffix(&a, &b, &out)); \
        result.value = value_##suffix(out); result.valid = result.value == 0.0625L; \
    } else if (which == FIR16) { \
        coefficient_##suffix##_t coefficients[16]; \
        linears_##suffix##_t state[16], out = sample_##suffix(0); \
        filters_fir_##suffix##_t context; \
        const linears_##suffix##_t input = sample_##suffix((raw_type)(UINT64_C(1) << ((fraction) - 2))); \
        for (size_t i = 0; i < 16; ++i) coefficients[i] = coefficient_##suffix(5); \
        REQUIRE(filters_fir_##suffix##_init(&context, coefficients, 16, state, 16)); \
        for (size_t i = 0; i < 32; ++i) REQUIRE(filters_fir_##suffix##_step(&context, input, &out)); \
        TIMED(filters_fir_##suffix##_step(&context, input, &out)); \
        result.value = value_##suffix(out); result.valid = result.value == 0.125L; \
    } else if (which == PI_STEP) { \
        fieldctrl_##suffix##_pi_t context; \
        const raw_type half = (raw_type)(UINT64_C(1) << ((fraction) - 1)); \
        const raw_type error = (raw_type)(UINT64_C(1) << ((fraction) - 3)); \
        const linears_##suffix##_t dt = sample_##suffix((raw_type)(UINT64_C(1) << ((fraction) - 8))); \
        linears_##suffix##_t out = sample_##suffix(0); \
        REQUIRE(fieldctrl_##suffix##_pi_init(&context, coefficient_##suffix(1), coefficient_##suffix(2), \
            sample_##suffix((raw_type)-half), sample_##suffix(half), sample_##suffix(0))); \
        REQUIRE(fieldctrl_##suffix##_pi_step(&context, sample_##suffix(error), dt, &out)); \
        if (value_##suffix(out) != 0.0626220703125L) return result; \
        REQUIRE(fieldctrl_##suffix##_pi_step(&context, sample_##suffix((raw_type)-error), dt, &out)); \
        if (value_##suffix(out) != -0.0625L) return result; \
        for (size_t i = 0; i < 32; ++i) \
            REQUIRE(fieldctrl_##suffix##_pi_step(&context, sample_##suffix((i & 1) ? (raw_type)-error : error), dt, &out)); \
        TIMED(fieldctrl_##suffix##_pi_step(&context, sample_##suffix((iteration & 1) ? (raw_type)-error : error), dt, &out)); \
        result.value = value_##suffix(out); result.valid = result.value == -0.0625L; \
    } else if (which == VVVF_STEP) { \
        fieldctrl_##suffix##_vvvf_t context; \
        fieldctrl_##suffix##_vvvf_output_t out; \
        const linears_##suffix##_t frequency = sample_##suffix((raw_type)(UINT64_C(1) << ((fraction) - 3))); \
        const linears_##suffix##_t amplitude = sample_##suffix((raw_type)(UINT64_C(1) << ((fraction) - 2))); \
        const linears_##suffix##_t bus = sample_##suffix((raw_type)(UINT64_C(1) << ((fraction) - 1))); \
        const linears_##suffix##_t dt = sample_##suffix((raw_type)(UINT64_C(1) << ((fraction) - 8))); \
        REQUIRE(fieldctrl_##suffix##_vvvf_init(&context, sample_##suffix(0))); \
        for (size_t i = 0; i < 32; ++i) \
            REQUIRE(fieldctrl_##suffix##_vvvf_step(&context, frequency, amplitude, bus, dt, &out)); \
        REQUIRE(fieldctrl_##suffix##_vvvf_init(&context, sample_##suffix(0))); \
        TIMED(fieldctrl_##suffix##_vvvf_step(&context, frequency, amplitude, bus, dt, &out)); \
        result.value = value_##suffix(out.duty.a); \
        result.valid = value_##suffix(out.angle) == 0.0L && \
            value_##suffix(out.voltage.alpha) == 0.25L && value_##suffix(out.voltage.beta) == 0.0L && \
            result.value == 0.875L && value_##suffix(out.duty.b) == 0.125L && \
            value_##suffix(out.duty.c) == 0.125L && !out.saturated; \
    } \
    return result; \
}

DEFINE_BENCHMARK(q15, int16_t, 15)
DEFINE_BENCHMARK(q31, int32_t, 31)
#undef DEFINE_BENCHMARK
#undef TIMED
#undef REQUIRE

int main(void) {
    static const struct { enum workload id; const char *name; size_t iterations; } workloads[] = {
        {MATRIX4, "matrix4", 20000}, {MATRIX16, "matrix16", 1000},
        {DOT256, "dot256", 10000}, {FIR16, "fir16", 100000},
        {PI_STEP, "pi_step", 200000}, {VVVF_STEP, "vvvf_step", 20480}
    };
    puts("mode,format,workload,iterations,elapsed_ns,ns_per_call,result");
    for (size_t i = 0; i < sizeof(workloads) / sizeof(workloads[0]); ++i) {
        for (unsigned format = 0; format < 2; ++format) {
            struct observation value = format == 0
                ? benchmark_q15(workloads[i].id, workloads[i].iterations)
                : benchmark_q31(workloads[i].id, workloads[i].iterations);
            if (value.status != LINEARS_OK || !value.valid || value.elapsed_ns == 0) {
                fprintf(stderr, "%s q%u %s invalid: status=%d valid=%d result=%.12Lf\n",
                        MODE_NAME, format == 0 ? 15U : 31U, workloads[i].name,
                        value.status, value.valid, value.value);
                return EXIT_FAILURE;
            }
            printf("%s,q%u,%s,%zu,%" PRIu64 ",%.3f,%.12Lf\n", MODE_NAME,
                   format == 0 ? 15U : 31U, workloads[i].name,
                   workloads[i].iterations, value.elapsed_ns,
                   (double)value.elapsed_ns / (double)workloads[i].iterations, value.value);
        }
    }
    return EXIT_SUCCESS;
}
