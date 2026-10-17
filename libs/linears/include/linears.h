/**
 * @file linears.h
 * @brief Allocation-free dense, row-major linear algebra contracts.
 *
 * Matrices own no storage. Every data range is contiguous and contains
 * rows * cols elements of the API's selected numeric type. The caller owns
 * all descriptors, data and scratch storage and serializes overlapping calls.
 * Independent calls are reentrant; no function accesses mutable global state.
 */
#ifndef LINEARS_H
#define LINEARS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Status shared by linears and its consumers.
 *
 * Each API documents its applicable failures and output commit behavior.
 */
typedef enum {
    LINEARS_OK = 0,                         /**< The operation completed. */
    LINEARS_ERROR_ARGUMENT = -1,           /**< Invalid pointer or scalar argument. */
    LINEARS_ERROR_DIMENSION = -2,          /**< Incompatible matrix shapes. */
    LINEARS_ERROR_SIZE = -3,               /**< A storage size cannot fit size_t. */
    LINEARS_ERROR_WORKSPACE = -4,          /**< Insufficient or misaligned scratch storage. */
    LINEARS_ERROR_ALIAS = -5,              /**< A prohibited storage overlap was found. */
    LINEARS_ERROR_RANGE = -6,              /**< Numeric overflow, nonfinite value or exponent range. */
    LINEARS_ERROR_DOMAIN = -7,             /**< Division by zero or invalid function domain. */
    LINEARS_ERROR_SINGULAR = -8,           /**< The required pivot is zero at working precision. */
    LINEARS_ERROR_NOT_POSITIVE_DEFINITE = -9, /**< A positive-definite factorization failed. */
    LINEARS_ERROR_NO_CONVERGENCE = -10      /**< The iteration budget was exhausted. */
} linears_status_t;

/**
 * @brief Caller-owned dense row-major matrix descriptor.
 *
 * q15 values are int16_t mantissas * 2^(exponent - 15); q31 values are
 * int32_t mantissas * 2^(exponent - 31). Floating APIs ignore exponent.
 * There is one exponent for the entire fixed-point matrix, not per element.
 * A descriptor has no stride or capacity field: the caller guarantees the
 * actual allocation covers the validated rows * cols element count.
 */
typedef struct {
    void *data;    /**< Naturally aligned element storage; NULL only for an empty matrix. */
    size_t rows;   /**< Row count; zero denotes an empty matrix. */
    size_t cols;   /**< Column count; zero denotes an empty matrix. */
    int exponent;  /**< Shared fixed-point binary exponent; ignored by floating APIs. */
} linears_matrix_t;

/**
 * @brief q15 scalar with an explicit binary exponent.
 */
typedef struct {
    int16_t mantissa; /**< Signed fractional numerator with denominator 2^15. */
    int exponent;    /**< Binary scale; the value is mantissa * 2^(exponent - 15). */
} linears_q15_t;

/**
 * @brief q31 scalar with an explicit binary exponent.
 */
typedef struct {
    int32_t mantissa; /**< Signed fractional numerator with denominator 2^31. */
    int exponent;    /**< Binary scale; the value is mantissa * 2^(exponent - 31). */
} linears_q31_t;

/**
 * @brief Storage unit providing the alignment required by linears workspaces.
 *
 * Declare an array of these units and access intermediates through the active
 * member of each unit. A byte buffer merely cast to this type is not a valid
 * substitute for an actual typed array. Units have no persistent state.
 */
typedef union {
    double floating;       /**< Alignment for floating intermediates. */
    int64_t integer;       /**< Alignment for wide integer intermediates. */
    linears_q31_t fixed;   /**< Alignment for normalized fixed-point intermediates. */
} linears_workspace_unit_t;

/**
 * @brief Lowest accepted fixed-point binary exponent, inclusive.
 */
#define LINEARS_EXPONENT_MIN (-16384)
/**
 * @brief Highest accepted fixed-point binary exponent, inclusive.
 */
#define LINEARS_EXPONENT_MAX 16384

/**
 * @brief Overflow-checked size multiplication, yielding SIZE_MAX on overflow.
 *
 * Arguments must be nonnegative size_t-compatible expressions without side
 * effects; this constant-expression macro can evaluate them more than once.
 * SIZE_MAX is an invalid workspace size, not a usable allocation request.
 */
#define LINEARS_SIZE_MUL(a, b) \
    ((size_t)(b) != 0 && (size_t)(a) > SIZE_MAX / (size_t)(b) \
         ? SIZE_MAX : (size_t)(a) * (size_t)(b))

/**
 * @brief Overflow-checked size addition, yielding SIZE_MAX on overflow.
 *
 * Arguments must be nonnegative size_t-compatible expressions without side
 * effects; this constant-expression macro can evaluate them more than once.
 */
#define LINEARS_SIZE_ADD(a, b) \
    ((size_t)(a) > SIZE_MAX - (size_t)(b) \
         ? SIZE_MAX : (size_t)(a) + (size_t)(b))

/**
 * @brief Maximum scratch bytes for any basic operation producing rows by cols.
 *
 * All four numeric formats use the same bound. Empty output needs zero bytes.
 * A result of SIZE_MAX means overflow. Scratch must be a real array of
 * linears_workspace_unit_t, disjoint from every matrix's data and descriptor.
 * Arguments must be nonnegative size_t-compatible expressions without effects.
 */
#define LINEARS_BASIC_WORKSPACE_BYTES(rows, cols) \
    LINEARS_SIZE_MUL(LINEARS_SIZE_MUL((rows), (cols)), sizeof(linears_workspace_unit_t))

/**
 * @brief Number of aligned workspace units covering bytes, or SIZE_MAX on overflow.
 *
 * The argument must have no side effects. SIZE_MAX propagates as invalid.
 */
#define LINEARS_WORKSPACE_UNITS(bytes) \
    ((size_t)(bytes) == SIZE_MAX ? SIZE_MAX : \
     (size_t)(bytes) / sizeof(linears_workspace_unit_t) + \
     ((size_t)(bytes) % sizeof(linears_workspace_unit_t) != 0))

#ifdef __cplusplus
}
#endif
#endif
