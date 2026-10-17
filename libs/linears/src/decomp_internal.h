/**
 * @file decomp_internal.h
 * @brief Private bounds and range checks shared by decomposition templates.
 *
 * This header does not allocate or retain state. Typed template helpers assume
 * this boundary validation has succeeded before reading caller storage.
 */
#ifndef LINEARS_DECOMP_INTERNAL_H
#define LINEARS_DECOMP_INTERNAL_H

#include "linears_decomp.h"
#include "linears_numeric.h"
#include <float.h>
#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @brief Private C99 alignment carriers, never instantiated as workspace.
 *
 * Named structures keep offsetof standard C99; defining a type inside
 * offsetof is a compiler extension and is deliberately avoided.
 */
struct linears_decomp_align_float { char prefix; float member; };
/** @brief Alignment carrier for native double storage. */
struct linears_decomp_align_double { char prefix; double member; };
/** @brief Alignment carrier for q15 raw storage. */
struct linears_decomp_align_int16_t { char prefix; int16_t member; };
/** @brief Alignment carrier for q31 raw storage. */
struct linears_decomp_align_int32_t { char prefix; int32_t member; };
/** @brief Alignment carrier for normalized integer scratch. */
struct linears_decomp_align_linears_fixed_t { char prefix; linears_fixed_t member; };
/** @brief Alignment carrier for matrix descriptors. */
struct linears_decomp_align_linears_matrix_t { char prefix; linears_matrix_t member; };
/** @brief Alignment carrier for row permutation entries. */
struct linears_decomp_align_size_t { char prefix; size_t member; };
/** @brief Alignment carrier for row-swap parity. */
struct linears_decomp_align_int { char prefix; int member; };
/** @brief Alignment carrier for symmetric pivot-block metadata. */
struct linears_decomp_align_uint8_t { char prefix; uint8_t member; };
/** @brief Expand an already selected scalar type into its named carrier. */
#define D_ALIGNOF_INNER(type) offsetof(struct linears_decomp_align_##type, member)
/** @brief C99 natural alignment for a supported decomposition storage type. */
#define D_ALIGNOF(type) D_ALIGNOF_INNER(type)

/**
 * @brief Validate one address range before overlap comparisons or access.
 * @param[in] pointer Range start, NULL allowed only for bytes zero.
 * @param[in] bytes Number of bytes in the range.
 * @param[in] alignment Required natural alignment in bytes, nonzero.
 * @return linears_status_t OK, ARGUMENT for NULL/alignment, or SIZE for address overflow.
 */
static inline linears_status_t linears_decomp_range(const void *pointer,
                                                   size_t bytes, size_t alignment)
{
    uintptr_t address = (uintptr_t)pointer;
    if (bytes == 0) return LINEARS_OK;
    if (pointer == NULL || address % alignment != 0) return LINEARS_ERROR_ARGUMENT;
    if (bytes > UINTPTR_MAX - address) return LINEARS_ERROR_SIZE;
    return LINEARS_OK;
}

/**
 * @brief Test overlap of two previously validated byte ranges.
 * @param[in] a First range start, possibly NULL only for zero a_bytes.
 * @param[in] a_bytes First byte count.
 * @param[in] b Second range start, possibly NULL only for zero b_bytes.
 * @param[in] b_bytes Second byte count.
 * @return int Nonzero on overlap; empty ranges never overlap.
 */
static inline int linears_decomp_overlap(const void *a, size_t a_bytes,
                                        const void *b, size_t b_bytes)
{
    uintptr_t first = (uintptr_t)a, second = (uintptr_t)b;
    return a_bytes != 0 && b_bytes != 0 &&
           first < second + b_bytes && second < first + a_bytes;
}

#endif
