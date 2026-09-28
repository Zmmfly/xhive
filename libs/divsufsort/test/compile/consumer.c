/*
 * C consumer fixture for the divsufsort public headers.
 *
 * This single translation unit is built twice by the test declarations: once
 * against the 32-bit-index API and once against the 64-bit-index API, where
 * the test target defines DIVSUFSORT_TEST_INDEX64. Only the local aliases
 * below change; public names and types are used exactly as declared.
 *
 * It is deliberately not tied to BUILD_DIVSUFSORT64, which selects how the
 * library itself is compiled, and it must not depend on Kconfig output,
 * generated headers, or GoogleTest. A zero exit code is the pass condition.
 *
 * Build requirements: C99, warnings-as-errors.
 */

/* Both public headers are included twice in 32-bit-then-64-bit order to prove
 * that their include guards and the shared divsufsort_types.h stay idempotent
 * and immune to declaration-order effects. */
#include <divsufsort.h>
#include <divsufsort64.h>
#include <divsufsort.h>
#include <divsufsort64.h>

#include <stdio.h>
#include <string.h>

/* Compile-time width contract for the shared and per-variant public types:
 * an unsigned byte, a 32-bit status value, and 32-/64-bit indices. */
typedef char byte_width[(sizeof(sauchar_t) == 1) ? 1 : -1];
typedef char status_width[(sizeof(saint_t) == 4) ? 1 : -1];
typedef char index32_width[(sizeof(saidx_t) == 4) ? 1 : -1];
typedef char index64_width[(sizeof(saidx64_t) == 8) ? 1 : -1];

/* Select the index width under test without touching the public API names. */
#if defined(DIVSUFSORT_TEST_INDEX64)
typedef saidx64_t test_index_t;
#define TEST_SORT(T, SA, N) divsufsort64((T), (SA), (N))
#define TEST_SUFCHECK(T, SA, N, V) sufcheck64((T), (SA), (N), (V))
#else
typedef saidx_t test_index_t;
#define TEST_SORT(T, SA, N) divsufsort((T), (SA), (N))
#define TEST_SUFCHECK(T, SA, N, V) sufcheck((T), (SA), (N), (V))
#endif

/* Defined in headers.cpp with C++ linkage hidden behind extern "C"; calling it
 * here verifies that the mixed C/C++ link resolves. */
int reverse_header_check(void);

int main(void) {
    const sauchar_t text[] = "banana";
    test_index_t sa[6];
    const test_index_t expected[6] = {5, 3, 1, 0, 4, 2};
    char formatted[64];
    int i;

    /* 1: the C++ translation unit agrees that all public type widths hold. */
    if (reverse_header_check() != 0) return 1;
    /* 2: suffix array construction reports success. */
    if (TEST_SORT(text, sa, 6) != 0) return 2;
    /* 3: the constructed suffix array is the known answer for "banana". */
    for (i = 0; i < 6; ++i) {
        if (sa[i] != expected[i]) return 3;
    }
    /* 4: the library's own validator accepts that suffix array. */
    if (TEST_SUFCHECK(text, sa, 6, 0) != 0) return 4;
    /* 5: PRIdSAINT_T formats a negative saint_t value as plain decimal. */
    snprintf(formatted, sizeof(formatted), "%" PRIdSAINT_T, (saint_t)-17);
    if (strcmp(formatted, "-17") != 0) return 5;
    return 0;
}
