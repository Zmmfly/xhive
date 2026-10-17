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
    double floating;       /**< Native f64 intermediates and their alignment. */
    float f32;             /**< Native f32 intermediates without double conversion. */
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
 * The zero-factor adjustment keeps literal-zero/runtime-factor combinations
 * warning-free: it only changes the comparison when the product is zero.
 * Every evaluated divisor is positive, and a wrapped product is never evaluated.
 */
#define LINEARS_SIZE_MUL(a, b) \
    (((size_t)(a) | (size_t)((size_t)(b) == 0)) > \
         SIZE_MAX / ((size_t)(b) != 0 ? (size_t)(b) : (size_t)1) \
         ? SIZE_MAX : (size_t)(a) * (size_t)(b))

/**
 * @brief Overflow-checked size addition, yielding SIZE_MAX on overflow.
 *
 * Arguments must be nonnegative size_t-compatible expressions without side
 * effects; this constant-expression macro can evaluate them more than once.
 */
#define LINEARS_SIZE_ADD(a, b) \
    (((size_t)(a) | (size_t)((size_t)(b) == 0)) > SIZE_MAX - (size_t)(b) \
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

/**
 * @defgroup linears_basic Basic matrix operations
 * @brief Native f32/f64 and integer-only block q15/q31 operations.
 *
 * Every matrix has contiguous, naturally aligned row-major storage. Output
 * rows/cols are supplied by the caller and never changed. A zero row or column
 * makes an empty matrix; NULL data is then permitted. Shape checks still apply.
 * Empty inner products return zero; (m x 0)(0 x n) produces an m x n zero block.
 *
 * Floating inputs and coefficients must be finite. f32 uses native float
 * arithmetic, f64 native double arithmetic. Floating exponent fields are ignored
 * and never changed. Computed q outputs use one automatically selected block
 * exponent; an all-zero block has exponent zero. Copy alone preserves the raw
 * representation exactly. Fixed inputs must have accepted block exponents and
 * values representable by the normalized core described in linears_numeric.h.
 *
 * On any failure output data, scalar outputs and output descriptor fields are
 * unchanged. Scratch contents are unspecified after a call, including failure.
 * Input/output data ranges may overlap arbitrarily, including exact in-place
 * operation. Descriptors may be identical but must not partially overlap each
 * other or any data. Scratch must be disjoint from all descriptors and data.
 * The caller guarantees actual buffer capacities; descriptor fields cannot
 * establish an allocation's real extent. Independent calls are reentrant.
 *
 * Shared errors: ARGUMENT for invalid pointer/alignment, DIMENSION for shape
 * mismatch, SIZE for count/byte/address arithmetic overflow, WORKSPACE for
 * insufficient or misaligned scratch, ALIAS for prohibited overlap, and RANGE
 * for nonfinite floating input/intermediate or unrepresentable fixed exponents.
 * No operation saturates. Underflow caused by a shared block exponent rounds
 * tiny output elements to zero. Dot products and matrix products round each
 * multiply/add in working precision; they are not exact superaccumulators.
 *
 * A scratch-taking call needs LINEARS_BASIC_WORKSPACE_BYTES(out->rows,
 * out->cols) bytes in an actual linears_workspace_unit_t array. An empty output
 * needs no scratch and accepts NULL. Workspace is not retained after return.
 * @{
 */

/**
 * @brief Compute the elementwise sum a + b.
 *
 * @param[in] a First input matrix, not NULL.
 * @param[in] b Second input matrix, not NULL; same shape as a.
 * @param[in,out] out Destination descriptor, not NULL; same shape as a.
 * @param[in,out] workspace Typed scratch array, or NULL for an empty output.
 * @param[in] workspace_bytes Available scratch bytes; see linears_basic.
 * @return linears_status_t OK or an applicable shared basic-operation error.
 */
linears_status_t linears_add_f32(const linears_matrix_t *a, const linears_matrix_t *b,
                               linears_matrix_t *out, linears_workspace_unit_t *workspace,
                               size_t workspace_bytes);
/**
 * @copydoc linears_add_f32
 */
linears_status_t linears_add_f64(const linears_matrix_t *a, const linears_matrix_t *b,
                               linears_matrix_t *out, linears_workspace_unit_t *workspace,
                               size_t workspace_bytes);
/**
 * @copydoc linears_add_f32
 */
linears_status_t linears_add_q15(const linears_matrix_t *a, const linears_matrix_t *b,
                               linears_matrix_t *out, linears_workspace_unit_t *workspace,
                               size_t workspace_bytes);
/**
 * @copydoc linears_add_f32
 */
linears_status_t linears_add_q31(const linears_matrix_t *a, const linears_matrix_t *b,
                               linears_matrix_t *out, linears_workspace_unit_t *workspace,
                               size_t workspace_bytes);

/**
 * @brief Compute the elementwise difference a - b.
 *
 * @param[in] a First input matrix, not NULL.
 * @param[in] b Matrix to subtract, not NULL; same shape as a.
 * @param[in,out] out Destination descriptor, not NULL; same shape as a.
 * @param[in,out] workspace Typed scratch array, or NULL for an empty output.
 * @param[in] workspace_bytes Available scratch bytes; see linears_basic.
 * @return linears_status_t OK or an applicable shared basic-operation error.
 */
linears_status_t linears_sub_f32(const linears_matrix_t *a, const linears_matrix_t *b,
                               linears_matrix_t *out, linears_workspace_unit_t *workspace,
                               size_t workspace_bytes);
/**
 * @copydoc linears_sub_f32
 */
linears_status_t linears_sub_f64(const linears_matrix_t *a, const linears_matrix_t *b,
                               linears_matrix_t *out, linears_workspace_unit_t *workspace,
                               size_t workspace_bytes);
/**
 * @copydoc linears_sub_f32
 */
linears_status_t linears_sub_q15(const linears_matrix_t *a, const linears_matrix_t *b,
                               linears_matrix_t *out, linears_workspace_unit_t *workspace,
                               size_t workspace_bytes);
/**
 * @copydoc linears_sub_f32
 */
linears_status_t linears_sub_q31(const linears_matrix_t *a, const linears_matrix_t *b,
                               linears_matrix_t *out, linears_workspace_unit_t *workspace,
                               size_t workspace_bytes);

/**
 * @brief Compute the elementwise (Hadamard) product of a and b.
 *
 * @param[in] a First input matrix, not NULL.
 * @param[in] b Second input matrix, not NULL; same shape as a.
 * @param[in,out] out Destination descriptor, not NULL; same shape as a.
 * @param[in,out] workspace Typed scratch array, or NULL for an empty output.
 * @param[in] workspace_bytes Available scratch bytes; see linears_basic.
 * @return linears_status_t OK or an applicable shared basic-operation error.
 */
linears_status_t linears_hadamard_f32(const linears_matrix_t *a, const linears_matrix_t *b,
                                    linears_matrix_t *out, linears_workspace_unit_t *workspace,
                                    size_t workspace_bytes);
/**
 * @copydoc linears_hadamard_f32
 */
linears_status_t linears_hadamard_f64(const linears_matrix_t *a, const linears_matrix_t *b,
                                    linears_matrix_t *out, linears_workspace_unit_t *workspace,
                                    size_t workspace_bytes);
/**
 * @copydoc linears_hadamard_f32
 */
linears_status_t linears_hadamard_q15(const linears_matrix_t *a, const linears_matrix_t *b,
                                    linears_matrix_t *out, linears_workspace_unit_t *workspace,
                                    size_t workspace_bytes);
/**
 * @copydoc linears_hadamard_f32
 */
linears_status_t linears_hadamard_q31(const linears_matrix_t *a, const linears_matrix_t *b,
                                    linears_matrix_t *out, linears_workspace_unit_t *workspace,
                                    size_t workspace_bytes);

/**
 * @brief Compute the matrix product a * b in row/column/inner-index order.
 *
 * @param[in] a Left matrix, not NULL, with shape m by k.
 * @param[in] b Right matrix, not NULL, with shape k by n.
 * @param[in,out] out Destination, not NULL, with shape m by n.
 * @param[in,out] workspace Typed scratch array, or NULL for an empty output.
 * @param[in] workspace_bytes Available scratch bytes; see linears_basic.
 * @return linears_status_t OK or an applicable shared basic-operation error.
 */
linears_status_t linears_mul_f32(const linears_matrix_t *a, const linears_matrix_t *b,
                               linears_matrix_t *out, linears_workspace_unit_t *workspace,
                               size_t workspace_bytes);
/**
 * @copydoc linears_mul_f32
 */
linears_status_t linears_mul_f64(const linears_matrix_t *a, const linears_matrix_t *b,
                               linears_matrix_t *out, linears_workspace_unit_t *workspace,
                               size_t workspace_bytes);
/**
 * @copydoc linears_mul_f32
 */
linears_status_t linears_mul_q15(const linears_matrix_t *a, const linears_matrix_t *b,
                               linears_matrix_t *out, linears_workspace_unit_t *workspace,
                               size_t workspace_bytes);
/**
 * @copydoc linears_mul_f32
 */
linears_status_t linears_mul_q31(const linears_matrix_t *a, const linears_matrix_t *b,
                               linears_matrix_t *out, linears_workspace_unit_t *workspace,
                               size_t workspace_bytes);

/**
 * @brief Multiply a matrix by a column vector.
 *
 * @param[in] a Matrix, not NULL, with shape m by k.
 * @param[in] b Column vector, not NULL, with shape k by 1.
 * @param[in,out] out Column-vector destination, not NULL, with shape m by 1.
 * @param[in,out] workspace Typed scratch array, or NULL for an empty output.
 * @param[in] workspace_bytes Available scratch bytes; see linears_basic.
 * @return linears_status_t OK or an applicable shared basic-operation error.
 */
linears_status_t linears_matvec_f32(const linears_matrix_t *a, const linears_matrix_t *b,
                                  linears_matrix_t *out, linears_workspace_unit_t *workspace,
                                  size_t workspace_bytes);
/**
 * @copydoc linears_matvec_f32
 */
linears_status_t linears_matvec_f64(const linears_matrix_t *a, const linears_matrix_t *b,
                                  linears_matrix_t *out, linears_workspace_unit_t *workspace,
                                  size_t workspace_bytes);
/**
 * @copydoc linears_matvec_f32
 */
linears_status_t linears_matvec_q15(const linears_matrix_t *a, const linears_matrix_t *b,
                                  linears_matrix_t *out, linears_workspace_unit_t *workspace,
                                  size_t workspace_bytes);
/**
 * @copydoc linears_matvec_f32
 */
linears_status_t linears_matvec_q31(const linears_matrix_t *a, const linears_matrix_t *b,
                                  linears_matrix_t *out, linears_workspace_unit_t *workspace,
                                  size_t workspace_bytes);

/**
 * @brief Transpose a matrix, including overlapping rectangular input/output.
 *
 * @param[in] a Input descriptor, not NULL, with shape m by n.
 * @param[in,out] out Destination descriptor, not NULL, with shape n by m.
 * @param[in,out] workspace Typed scratch array, or NULL for an empty output.
 * @param[in] workspace_bytes Available scratch bytes; see linears_basic.
 * @return linears_status_t OK or an applicable shared basic-operation error.
 */
linears_status_t linears_transpose_f32(const linears_matrix_t *a, linears_matrix_t *out,
                                     linears_workspace_unit_t *workspace, size_t workspace_bytes);
/**
 * @copydoc linears_transpose_f32
 */
linears_status_t linears_transpose_f64(const linears_matrix_t *a, linears_matrix_t *out,
                                     linears_workspace_unit_t *workspace, size_t workspace_bytes);
/**
 * @copydoc linears_transpose_f32
 */
linears_status_t linears_transpose_q15(const linears_matrix_t *a, linears_matrix_t *out,
                                     linears_workspace_unit_t *workspace, size_t workspace_bytes);
/**
 * @copydoc linears_transpose_f32
 */
linears_status_t linears_transpose_q31(const linears_matrix_t *a, linears_matrix_t *out,
                                     linears_workspace_unit_t *workspace, size_t workspace_bytes);

/**
 * @brief Compute the weighted sum alpha * a + beta * b.
 *
 * @param[in] alpha Scalar multiplier in this function's numeric format.
 * @param[in] a First input matrix, not NULL.
 * @param[in] beta Scalar multiplier in this function's numeric format.
 * @param[in] b Second input matrix, not NULL; same shape as a.
 * @param[in,out] out Destination, not NULL; same shape as a.
 * @param[in,out] workspace Typed scratch array, or NULL for an empty output.
 * @param[in] workspace_bytes Available scratch bytes; see linears_basic.
 * @return linears_status_t OK or an applicable shared basic-operation error.
 * @note Fixed-point products and sum round separately. Floating contraction
 *       follows the compiler's floating-point settings.
 */
linears_status_t linears_axpby_f32(float alpha, const linears_matrix_t *a,
                                 float beta, const linears_matrix_t *b, linears_matrix_t *out,
                                 linears_workspace_unit_t *workspace, size_t workspace_bytes);
/**
 * @copydoc linears_axpby_f32
 */
linears_status_t linears_axpby_f64(double alpha, const linears_matrix_t *a,
                                 double beta, const linears_matrix_t *b, linears_matrix_t *out,
                                 linears_workspace_unit_t *workspace, size_t workspace_bytes);
/**
 * @copydoc linears_axpby_f32
 */
linears_status_t linears_axpby_q15(linears_q15_t alpha, const linears_matrix_t *a,
                                 linears_q15_t beta, const linears_matrix_t *b, linears_matrix_t *out,
                                 linears_workspace_unit_t *workspace, size_t workspace_bytes);
/**
 * @copydoc linears_axpby_f32
 */
linears_status_t linears_axpby_q31(linears_q31_t alpha, const linears_matrix_t *a,
                                 linears_q31_t beta, const linears_matrix_t *b, linears_matrix_t *out,
                                 linears_workspace_unit_t *workspace, size_t workspace_bytes);

/**
 * @brief Multiply every element by a scalar.
 *
 * @param[in] alpha Multiplier in this function's numeric format.
 * @param[in] a Input descriptor, not NULL.
 * @param[in,out] out Destination, not NULL; same shape as a.
 * @param[in,out] workspace Typed scratch array, or NULL for an empty output.
 * @param[in] workspace_bytes Available scratch bytes; see linears_basic.
 * @return linears_status_t OK or an applicable shared basic-operation error.
 */
linears_status_t linears_scale_f32(float alpha, const linears_matrix_t *a, linears_matrix_t *out,
                                 linears_workspace_unit_t *workspace, size_t workspace_bytes);
/**
 * @copydoc linears_scale_f32
 */
linears_status_t linears_scale_f64(double alpha, const linears_matrix_t *a, linears_matrix_t *out,
                                 linears_workspace_unit_t *workspace, size_t workspace_bytes);
/**
 * @copydoc linears_scale_f32
 */
linears_status_t linears_scale_q15(linears_q15_t alpha, const linears_matrix_t *a, linears_matrix_t *out,
                                 linears_workspace_unit_t *workspace, size_t workspace_bytes);
/**
 * @copydoc linears_scale_f32
 */
linears_status_t linears_scale_q31(linears_q31_t alpha, const linears_matrix_t *a, linears_matrix_t *out,
                                 linears_workspace_unit_t *workspace, size_t workspace_bytes);

/**
 * @brief Copy a matrix representation exactly with memmove overlap semantics.
 *
 * @param[in] a Source descriptor, not NULL; finite/representable values required.
 * @param[in,out] out Destination descriptor, not NULL; same shape as a.
 * @return linears_status_t OK or ARGUMENT, DIMENSION, SIZE, ALIAS, RANGE.
 * @note Needs no scratch. Fixed-point exponent is copied without normalization;
 *       floating exponent is not changed. Input/output data overlap is supported.
 */
linears_status_t linears_copy_f32(const linears_matrix_t *a, linears_matrix_t *out);
/**
 * @copydoc linears_copy_f32
 */
linears_status_t linears_copy_f64(const linears_matrix_t *a, linears_matrix_t *out);
/**
 * @copydoc linears_copy_f32
 */
linears_status_t linears_copy_q15(const linears_matrix_t *a, linears_matrix_t *out);
/**
 * @copydoc linears_copy_f32
 */
linears_status_t linears_copy_q31(const linears_matrix_t *a, linears_matrix_t *out);

/**
 * @brief Fill all matrix elements with zero without scratch storage.
 *
 * @param[in,out] out Destination descriptor, not NULL; any rectangular shape.
 * @return linears_status_t OK or ARGUMENT, SIZE, ALIAS.
 * @note Sets fixed exponent to zero; leaves floating exponent unchanged.
 */
linears_status_t linears_zero_f32(linears_matrix_t *out);
/**
 * @copydoc linears_zero_f32
 */
linears_status_t linears_zero_f64(linears_matrix_t *out);
/**
 * @copydoc linears_zero_f32
 */
linears_status_t linears_zero_q15(linears_matrix_t *out);
/**
 * @copydoc linears_zero_f32
 */
linears_status_t linears_zero_q31(linears_matrix_t *out);

/**
 * @brief Fill a square identity matrix without scratch storage.
 *
 * @param[in,out] out Square destination descriptor, not NULL; 0 by 0 is valid.
 * @return linears_status_t OK or ARGUMENT, DIMENSION, SIZE, ALIAS.
 * @note Nonempty fixed identity uses exponent 1 and mantissa 2^14/2^30 on the
 *       diagonal; empty fixed identity uses exponent 0. Floating exponent stays.
 */
linears_status_t linears_identity_f32(linears_matrix_t *out);
/**
 * @copydoc linears_identity_f32
 */
linears_status_t linears_identity_f64(linears_matrix_t *out);
/**
 * @copydoc linears_identity_f32
 */
linears_status_t linears_identity_q15(linears_matrix_t *out);
/**
 * @copydoc linears_identity_f32
 */
linears_status_t linears_identity_q31(linears_matrix_t *out);

/**
 * @brief Sum corresponding products in row-major order without scratch storage.
 *
 * @param[in] a First input, not NULL; any rectangular shape.
 * @param[in] b Second input, not NULL; same shape as a.
 * @param[out] out Non-NULL scalar destination in the selected numeric format;
 *                 may overlap input data, but not either matrix descriptor.
 * @return linears_status_t OK or ARGUMENT, DIMENSION, SIZE, ALIAS, RANGE.
 * @note This is a vector dot product or, for matrices, the Frobenius inner
 *       product. Empty inputs return zero. Fixed scalar output is normalized.
 */
linears_status_t linears_dot_f32(const linears_matrix_t *a, const linears_matrix_t *b, float *out);
/**
 * @copydoc linears_dot_f32
 */
linears_status_t linears_dot_f64(const linears_matrix_t *a, const linears_matrix_t *b, double *out);
/**
 * @copydoc linears_dot_f32
 */
linears_status_t linears_dot_q15(const linears_matrix_t *a, const linears_matrix_t *b, linears_q15_t *out);
/**
 * @copydoc linears_dot_f32
 */
linears_status_t linears_dot_q31(const linears_matrix_t *a, const linears_matrix_t *b, linears_q31_t *out);

/**
 * @brief Compute the Euclidean/Frobenius norm using scaled sum of squares.
 *
 * @param[in] a Input, not NULL; any rectangular shape.
 * @param[out] out Non-NULL nonnegative scalar destination; may overlap input
 *                 data but not the descriptor. Empty inputs produce zero.
 * @return linears_status_t OK or ARGUMENT, SIZE, ALIAS, RANGE.
 * @note Needs no scratch. Scaling avoids squaring large inputs directly.
 */
linears_status_t linears_norm_f32(const linears_matrix_t *a, float *out);
/**
 * @copydoc linears_norm_f32
 */
linears_status_t linears_norm_f64(const linears_matrix_t *a, double *out);
/**
 * @copydoc linears_norm_f32
 */
linears_status_t linears_norm_q15(const linears_matrix_t *a, linears_q15_t *out);
/**
 * @copydoc linears_norm_f32
 */
linears_status_t linears_norm_q31(const linears_matrix_t *a, linears_q31_t *out);

/**
 * @brief Sum a square matrix's diagonal in increasing index order.
 *
 * @param[in] a Square input descriptor, not NULL; 0 by 0 is valid.
 * @param[out] out Non-NULL scalar destination; may overlap input data but not
 *                 the descriptor. Empty input produces zero.
 * @return linears_status_t OK or ARGUMENT, DIMENSION, SIZE, ALIAS, RANGE.
 * @note Needs no scratch. Fixed scalar output is normalized.
 */
linears_status_t linears_trace_f32(const linears_matrix_t *a, float *out);
/**
 * @copydoc linears_trace_f32
 */
linears_status_t linears_trace_f64(const linears_matrix_t *a, double *out);
/**
 * @copydoc linears_trace_f32
 */
linears_status_t linears_trace_q15(const linears_matrix_t *a, linears_q15_t *out);
/**
 * @copydoc linears_trace_f32
 */
linears_status_t linears_trace_q31(const linears_matrix_t *a, linears_q31_t *out);

/** @} */

#ifdef __cplusplus
}
#endif
#endif
