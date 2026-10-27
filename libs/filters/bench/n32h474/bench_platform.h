/**
 * @file bench_platform.h
 * @brief N32H474-only benchmark timing interface, not a filter library API.
 */
#ifndef FILTERS_BENCH_PLATFORM_H
#define FILTERS_BENCH_PLATFORM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Configure the board clock, selected FPU mode, GPIO markers and DWT. */
void bench_platform_init(void);

/** @brief Return the free-running 32-bit DWT cycle counter. */
uint32_t bench_cycles(void);

/**
 * @brief Set one of the twelve analyzer-connected GPIO markers.
 * @param[in] channel Zero-based marker index, 0 through 11.
 * @param[in] high Zero clears the marker; nonzero sets it.
 */
void bench_marker(uint32_t channel, uint32_t high);

/** @brief Return the configured processor clock frequency in Hz. */
uint32_t bench_clock_hz(void);

/** @brief Return SCB CPACR to verify CP10/CP11 access at measurement time. */
uint32_t bench_cpacr(void);

/** @brief Run the fixed, allocation-free benchmark workload and publish results. */
void bench_run(void);

#ifdef __cplusplus
}
#endif
#endif
