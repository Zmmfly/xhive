/**
 * @file bench_result.h
 * @brief Stable little-endian RAM mailbox for the N32H474 benchmark runner.
 */
#ifndef FILTERS_BENCH_RESULT_H
#define FILTERS_BENCH_RESULT_H
#include <stdint.h>

/** @brief ABI identifier checked by the host reader. */
#define BENCH_MAGIC UINT32_C(0x51424e43)
/** @brief Number of independent transactions retained after warmup. */
#define BENCH_TRIALS 9u
/** @brief Maximum records: two models times ten dimension/particle cases. */
#define BENCH_MAX_CASES 20u

/** @brief One algorithm/model/dimension case; format is selected per image. */
typedef struct {
    uint32_t family; /**< 0=UKF predict+update, 1=PF predict+update+estimate, 2=UPF step. */
    uint32_t nonlinear; /**< Zero selects the linear callback, one the quadratic callback. */
    uint32_t n; /**< State dimension; measurement dimension equals n. */
    uint32_t particles; /**< One for UKF, otherwise 32 or 128. */
    int32_t status; /**< First nonzero linears_status_t, or zero on success. */
    uint32_t cycles[BENCH_TRIALS]; /**< Unsubtracted DWT cycles per restored transaction. */
    uint32_t persistent_bytes; /**< Persistent signal arrays only. */
    uint32_t descriptor_bytes; /**< Filter state structure. */
    uint32_t workspace_bytes; /**< Reusable explicit operation workspace. */
    uint32_t output_bytes; /**< PF estimate output arrays. */
    uint32_t checksum_lo; /**< Low word of the IEEE binary64 numerical checksum. */
    uint32_t checksum_hi; /**< High word of the IEEE binary64 numerical checksum. */
    uint32_t output_hash; /**< Raw output hash for repeatability checks. */
} bench_case_t;

/** @brief One firmware execution; only this test program owns mutable global state. */
typedef struct {
    uint32_t magic;
    uint32_t version; /**< Currently 1. */
    uint32_t format; /**< 0=f32, 1=f64, 2=q15, 3=q31. */
    uint32_t phase; /**< 0=boot, 1=ready, 2=running, 3=complete, 4=failed. */
    uint32_t command; /**< Host writes 1 to start; firmware clears it before running. */
    uint32_t core_hz;
    uint32_t cpacr;
    uint32_t cycle_read_overhead;
    uint32_t case_count;
    uint32_t active_case;
    uint32_t fault_cfsr;
    uint32_t fault_hfsr;
    bench_case_t cases[BENCH_MAX_CASES];
} bench_results_t;

#ifdef __cplusplus
extern "C" {
#endif
/** @brief Single-writer RAM mailbox, read over SWD outside timed regions. */
extern volatile bench_results_t g_bench_results;
#ifdef __cplusplus
}
#endif
#endif
