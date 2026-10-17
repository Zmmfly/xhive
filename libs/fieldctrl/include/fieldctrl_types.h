/**
 * @file fieldctrl_types.h
 * @brief Scalar and coordinate contracts for allocation-free motor algorithms.
 *
 * The caller owns every context and serializes calls using the same context.
 * Independent contexts are reentrant. No API accesses hardware or allocates.
 * Unless explicitly documented otherwise, every angle is an electrical turn
 * (one turn = 2*pi radians), positive from alpha toward beta, and is wrapped
 * into [0,1). Frequencies are electrical turns/second, not radians/second.
 * Physical units are SI or a consistent caller-selected per-unit system.
 * q15/q31 values retain the explicit binary exponent from linears.h; they are
 * not limited to [-1,1). Returned q scalars are independently normalized.
 */
#ifndef FIELDCTRL_TYPES_H
#define FIELDCTRL_TYPES_H

#include "linears.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Three phase quantities in a,b,c phase order, using float storage.
 */
typedef struct {
    float a; /**< Phase a value, in the caller's current or voltage units. */
    float b; /**< Phase b value in the same units as a. */
    float c; /**< Phase c value in the same units as a. */
} fieldctrl_f32_abc_t;

/**
 * @brief Stationary orthogonal alpha/beta quantities using float storage.
 */
typedef struct {
    float alpha; /**< Alpha component aligned with phase a. */
    float beta;  /**< Beta component, positive 90 electrical degrees from alpha. */
} fieldctrl_f32_ab_t;

/**
 * @brief Rotor-aligned d/q quantities using float storage.
 */
typedef struct {
    float d; /**< Direct axis aligned with the supplied rotor angle. */
    float q; /**< Quadrature axis, positive 90 electrical degrees from d. */
} fieldctrl_f32_dq_t;

/**
 * @brief Three phase quantities in a,b,c phase order, using double storage.
 */
typedef struct {
    double a; /**< Phase a value in caller-selected units. */
    double b; /**< Phase b value in the same units. */
    double c; /**< Phase c value in the same units. */
} fieldctrl_f64_abc_t;

/**
 * @brief Stationary orthogonal alpha/beta quantities using double storage.
 */
typedef struct {
    double alpha; /**< Alpha component aligned with phase a. */
    double beta;  /**< Beta component, positive 90 electrical degrees from alpha. */
} fieldctrl_f64_ab_t;

/**
 * @brief Rotor-aligned d/q quantities using double storage.
 */
typedef struct {
    double d; /**< Direct axis aligned with the supplied rotor angle. */
    double q; /**< Quadrature axis, positive 90 electrical degrees from d. */
} fieldctrl_f64_dq_t;

/**
 * @brief Three phase quantities with independently scaled q15 components.
 */
typedef struct {
    linears_q15_t a; /**< Phase a value in caller-selected units. */
    linears_q15_t b; /**< Phase b value in the same units. */
    linears_q15_t c; /**< Phase c value in the same units. */
} fieldctrl_q15_abc_t;

/**
 * @brief Stationary orthogonal quantities with independent q15 exponents.
 */
typedef struct {
    linears_q15_t alpha; /**< Alpha component aligned with phase a. */
    linears_q15_t beta;  /**< Beta component, positive 90 electrical degrees. */
} fieldctrl_q15_ab_t;

/**
 * @brief Rotor-aligned quantities with independently scaled q15 components.
 */
typedef struct {
    linears_q15_t d; /**< Direct component aligned with the rotor angle. */
    linears_q15_t q; /**< Quadrature component, positive 90 electrical degrees. */
} fieldctrl_q15_dq_t;

/**
 * @brief Three phase quantities with independently scaled q31 components.
 */
typedef struct {
    linears_q31_t a; /**< Phase a value in caller-selected units. */
    linears_q31_t b; /**< Phase b value in the same units. */
    linears_q31_t c; /**< Phase c value in the same units. */
} fieldctrl_q31_abc_t;

/**
 * @brief Stationary orthogonal quantities with independent q31 exponents.
 */
typedef struct {
    linears_q31_t alpha; /**< Alpha component aligned with phase a. */
    linears_q31_t beta;  /**< Beta component, positive 90 electrical degrees. */
} fieldctrl_q31_ab_t;

/**
 * @brief Rotor-aligned quantities with independently scaled q31 components.
 */
typedef struct {
    linears_q31_t d; /**< Direct component aligned with the rotor angle. */
    linears_q31_t q; /**< Quadrature component, positive 90 electrical degrees. */
} fieldctrl_q31_dq_t;

#ifdef __cplusplus
}
#endif
#endif
