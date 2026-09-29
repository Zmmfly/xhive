/*
 * Shared implementation body of the scalar PID variants.
 *
 * This is not a public header: each src/pid_<suffix>.c includes it exactly once
 * with EZC_PID_TYPE and EZC_PID_SUFFIX defined, and it expands to one complete
 * typed API. A new scalar variant therefore needs a one-line instantiation file
 * instead of a copy of the algorithm.
 */

#if !defined(EZC_PID_TYPE) || !defined(EZC_PID_SUFFIX)
#error "pid_impl.h requires EZC_PID_TYPE and EZC_PID_SUFFIX"
#endif

#include <stddef.h>

#define EZC_PID_CAT_(a, b) a##b
#define EZC_PID_CAT(a, b) EZC_PID_CAT_(a, b)

/* ezc_pid_f32, ezc_pid_f64, ... */
#define EZC_PID_NAME EZC_PID_CAT(ezc_pid_, EZC_PID_SUFFIX)
/* ezc_pid_f32_t, ezc_pid_f32_cfg_t, ... */
#define EZC_PID_CONTEXT EZC_PID_CAT(EZC_PID_NAME, _t)
#define EZC_PID_CONFIG EZC_PID_CAT(EZC_PID_NAME, _cfg_t)
#define EZC_PID_RESULT EZC_PID_CAT(EZC_PID_NAME, _result_t)
#define EZC_PID_CALLBACK EZC_PID_CAT(EZC_PID_NAME, _cb_t)
/* ezc_pid_f32_init, ezc_pid_f32_update, ... */
#define EZC_PID_FN(name) EZC_PID_CAT(EZC_PID_NAME, name)

/*
 * Return a context to the documented zero state: all gains zero, no limit, no
 * anti-windup, no callback and no controller state.
 */
static void EZC_PID_FN(_zero)(EZC_PID_CONTEXT *ctx)
{
    ctx->kp = (EZC_PID_TYPE)0;
    ctx->ki = (EZC_PID_TYPE)0;
    ctx->kd = (EZC_PID_TYPE)0;
    ctx->integral = (EZC_PID_TYPE)0;
    ctx->prev_error = (EZC_PID_TYPE)0;
    ctx->dt_cached = (EZC_PID_TYPE)0;
    ctx->dt_inv_cached = (EZC_PID_TYPE)0;
    ctx->out_min = (EZC_PID_TYPE)0;
    ctx->out_max = (EZC_PID_TYPE)0;
    ctx->limit_enabled = false;
    ctx->anti_windup_enabled = false;
    ctx->callback = NULL;
    ctx->callback_ctx = NULL;
}

ezc_status_t EZC_PID_FN(_init)(EZC_PID_CONTEXT *ctx, const EZC_PID_CONFIG *cfg)
{
    if (ctx == NULL) {
        return EZC_ERR_INVALID_ARG;
    }

    /*
     * A missing configuration is rejected, and so is an enabled limit range
     * that is inverted or not comparable: NaN limits fail the comparison and
     * are therefore rejected as well.
     */
    if ((cfg == NULL) || (cfg->limit_enabled && !(cfg->out_min <= cfg->out_max))) {
        EZC_PID_FN(_zero)(ctx);
        return EZC_ERR_INVALID_ARG;
    }

    EZC_PID_FN(_zero)(ctx);
    ctx->kp = cfg->kp;
    ctx->ki = cfg->ki;
    ctx->kd = cfg->kd;
    ctx->out_min = cfg->out_min;
    ctx->out_max = cfg->out_max;
    ctx->limit_enabled = cfg->limit_enabled;
    ctx->anti_windup_enabled = cfg->anti_windup_enabled;
    ctx->callback = cfg->callback;
    ctx->callback_ctx = cfg->callback_ctx;
    return EZC_OK;
}

ezc_status_t EZC_PID_FN(_set_gains)(EZC_PID_CONTEXT *ctx, EZC_PID_TYPE kp,
                                    EZC_PID_TYPE ki, EZC_PID_TYPE kd)
{
    if (ctx == NULL) {
        return EZC_ERR_INVALID_ARG;
    }
    ctx->kp = kp;
    ctx->ki = ki;
    ctx->kd = kd;
    return EZC_OK;
}

ezc_status_t EZC_PID_FN(_set_limit)(EZC_PID_CONTEXT *ctx, EZC_PID_TYPE out_min,
                                    EZC_PID_TYPE out_max)
{
    if ((ctx == NULL) || !(out_min <= out_max)) {
        return EZC_ERR_INVALID_ARG;
    }
    ctx->out_min = out_min;
    ctx->out_max = out_max;
    ctx->limit_enabled = true;
    return EZC_OK;
}

ezc_status_t EZC_PID_FN(_clear_limit)(EZC_PID_CONTEXT *ctx)
{
    if (ctx == NULL) {
        return EZC_ERR_INVALID_ARG;
    }
    ctx->limit_enabled = false;
    return EZC_OK;
}

ezc_status_t EZC_PID_FN(_set_anti_windup)(EZC_PID_CONTEXT *ctx, bool enable)
{
    if (ctx == NULL) {
        return EZC_ERR_INVALID_ARG;
    }
    ctx->anti_windup_enabled = enable;
    return EZC_OK;
}

ezc_status_t EZC_PID_FN(_set_callback)(EZC_PID_CONTEXT *ctx, EZC_PID_CALLBACK callback,
                                       void *user_ctx)
{
    if (ctx == NULL) {
        return EZC_ERR_INVALID_ARG;
    }
    ctx->callback = callback;
    ctx->callback_ctx = user_ctx;
    return EZC_OK;
}

void EZC_PID_FN(_reset)(EZC_PID_CONTEXT *ctx)
{
    if (ctx == NULL) {
        return;
    }
    ctx->integral = (EZC_PID_TYPE)0;
    ctx->prev_error = (EZC_PID_TYPE)0;
}

/*
 * Return 1 / dt and keep it cached while the caller repeats the same dt.
 *
 * Zero is returned for every dt that cannot be inverted: zero, negative, NaN
 * and positive infinity all fail the comparisons below, and the caller treats
 * that zero as a zero-time step. The comparison is bit-exact on purpose,
 * because a fixed-rate loop passes the very same value every step.
 */
static EZC_PID_TYPE EZC_PID_FN(_inverse_dt)(EZC_PID_CONTEXT *ctx, EZC_PID_TYPE dt)
{
    EZC_PID_TYPE inv_dt;

    if (dt == ctx->dt_cached) {
        return ctx->dt_inv_cached;
    }
    if (!(dt > (EZC_PID_TYPE)0)) {
        return (EZC_PID_TYPE)0;
    }
    inv_dt = (EZC_PID_TYPE)1 / dt;
    if (!(inv_dt > (EZC_PID_TYPE)0)) {
        return (EZC_PID_TYPE)0;
    }
    ctx->dt_cached = dt;
    ctx->dt_inv_cached = inv_dt;
    return inv_dt;
}

EZC_PID_TYPE EZC_PID_FN(_update)(EZC_PID_CONTEXT *ctx, EZC_PID_TYPE setpoint,
                                 EZC_PID_TYPE measure, EZC_PID_TYPE dt)
{
    EZC_PID_RESULT result;
    EZC_PID_TYPE error;
    EZC_PID_TYPE p_term;
    EZC_PID_TYPE d_term;
    EZC_PID_TYPE pd_term;
    EZC_PID_TYPE i_term;
    EZC_PID_TYPE inv_dt;
    EZC_PID_TYPE raw;
    EZC_PID_TYPE output;

    if (ctx == NULL) {
        return (EZC_PID_TYPE)0;
    }

    error = setpoint - measure;
    p_term = ctx->kp * error;
    i_term = ctx->integral;
    d_term = (EZC_PID_TYPE)0;

    /*
     * A finite positive dt advances the time-dependent terms. Anything else is
     * a zero-time step: the integral holds, the derivative rate is zero, and
     * prev_error is still moved to this sample so that the next valid step
     * sees the error change of a single sample interval.
     */
    inv_dt = EZC_PID_FN(_inverse_dt)(ctx, dt);
    if (inv_dt > (EZC_PID_TYPE)0) {
        i_term = ctx->integral + (ctx->ki * error * dt);
        d_term = ctx->kd * (error - ctx->prev_error) * inv_dt;
    }

    pd_term = p_term + d_term;
    raw = pd_term + i_term;
    output = raw;

    if (ctx->limit_enabled) {
        result.limited = (raw < ctx->out_min) || (raw > ctx->out_max);

        if (result.limited && ctx->anti_windup_enabled) {
            /*
             * The integral may advance up to the saturation boundary, which is
             * the value of the integral term that places the unclamped output
             * exactly on a limit. The limiter only stops the integral from
             * moving further into saturation; it never pulls the integral back
             * towards its previous value and therefore never manufactures
             * integral state of its own.
             */
            const EZC_PID_TYPE i_min = ctx->out_min - pd_term;
            const EZC_PID_TYPE i_max = ctx->out_max - pd_term;

            if ((i_term > i_max) && (i_term > ctx->integral)) {
                i_term = (i_max > ctx->integral) ? i_max : ctx->integral;
            } else if ((i_term < i_min) && (i_term < ctx->integral)) {
                i_term = (i_min < ctx->integral) ? i_min : ctx->integral;
            }
            raw = pd_term + i_term;
        }

        output = raw;
        if (output < ctx->out_min) {
            output = ctx->out_min;
        } else if (output > ctx->out_max) {
            output = ctx->out_max;
        }
    } else {
        result.limited = false;
    }

    ctx->integral = i_term;
    ctx->prev_error = error;

    result.output = output;
    result.setpoint = setpoint;
    result.measure = measure;
    result.error = error;
    result.p_term = p_term;
    result.i_term = i_term;
    result.d_term = d_term;

    if (ctx->callback != NULL) {
        ctx->callback(ctx->callback_ctx, &result);
    }
    return output;
}

#undef EZC_PID_FN
#undef EZC_PID_CALLBACK
#undef EZC_PID_RESULT
#undef EZC_PID_CONFIG
#undef EZC_PID_CONTEXT
#undef EZC_PID_NAME
#undef EZC_PID_CAT
#undef EZC_PID_CAT_
