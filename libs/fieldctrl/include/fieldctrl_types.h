/**
 * @file fieldctrl_types.h
 * @brief Scalar and coordinate contracts for allocation-free motor algorithms.
 *
 * The caller owns every context and serializes calls using the same context.
 * Independent contexts are reentrant. No API accesses hardware or allocates.
 * Unless explicitly documented otherwise, every angle is an electrical turn
 * (one turn = 2*pi radians), positive from alpha toward beta, and is wrapped
 * into [0,1). Frequencies are electrical turns/second, not radians/second.
 * Floating APIs use SI units unless a component explicitly permits per-unit
 * quantities. Q APIs use raw int16_t/2^15 or int32_t/2^31 signals in [-1,1).
 * The caller chooses Vbase, Ibase and Tbase; no base is stored by this library.
 * Voltage is V/Vbase, current is I/Ibase, time is t/Tbase, frequency is f*Tbase,
 * resistance is R*Ibase/Vbase and inductance is L*Ibase/(Vbase*Tbase).
 * Flux is psi/(Vbase*Tbase); torque is T/(Vbase*Ibase*Tbase). Angles remain
 * electrical turns, independent of these bases. Explicit wide coefficients,
 * state and diagnostics are int64_t/2^31, with no dynamic binary point.
 * Q outputs check the true [-1,+1] domain; +1 rounds to INT_MAX and -1 to
 * INT_MIN. Raw INT_MAX is still 1 minus one signal LSB, not exactly one.
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
 * @brief Three phase Q1.15 per-unit signal components, raw int16_t/2^15.
 */
typedef struct {
    linears_q15_t a; /**< Phase a value in caller-selected units. */
    linears_q15_t b; /**< Phase b value in the same units. */
    linears_q15_t c; /**< Phase c value in the same units. */
} fieldctrl_q15_abc_t;

/**
 * @brief Stationary orthogonal Q1.15 per-unit components.
 */
typedef struct {
    linears_q15_t alpha; /**< Alpha component aligned with phase a. */
    linears_q15_t beta;  /**< Beta component, positive 90 electrical degrees. */
} fieldctrl_q15_ab_t;

/**
 * @brief Rotor-aligned Q1.15 per-unit components.
 */
typedef struct {
    linears_q15_t d; /**< Direct component aligned with the rotor angle. */
    linears_q15_t q; /**< Quadrature component, positive 90 electrical degrees. */
} fieldctrl_q15_dq_t;

/**
 * @brief Three phase Q1.31 per-unit signal components, raw int32_t/2^31.
 */
typedef struct {
    linears_q31_t a; /**< Phase a value in caller-selected units. */
    linears_q31_t b; /**< Phase b value in the same units. */
    linears_q31_t c; /**< Phase c value in the same units. */
} fieldctrl_q31_abc_t;

/**
 * @brief Stationary orthogonal Q1.31 per-unit components.
 */
typedef struct {
    linears_q31_t alpha; /**< Alpha component aligned with phase a. */
    linears_q31_t beta;  /**< Beta component, positive 90 electrical degrees. */
} fieldctrl_q31_ab_t;

/**
 * @brief Rotor-aligned Q1.31 per-unit components.
 */
typedef struct {
    linears_q31_t d; /**< Direct component aligned with the rotor angle. */
    linears_q31_t q; /**< Quadrature component, positive 90 electrical degrees. */
} fieldctrl_q31_dq_t;

/**
 * @brief Wide rotor-frame state or diagnostic, each int64_t raw divided by 2^31.
 *
 * Used only where an unbounded PI request or internal integral needs headroom.
 * The physical per-unit meaning follows the owning API; this is not Q signal
 * storage and must not be implicitly narrowed to int16_t or int32_t.
 */
typedef struct {
    linears_fixed_t d; /**< Direct component at the invariant F31 binary point. */
    linears_fixed_t q; /**< Quadrature component at the invariant F31 binary point. */
} fieldctrl_fixed_dq_t;

#ifdef __cplusplus
}
#endif
#endif
