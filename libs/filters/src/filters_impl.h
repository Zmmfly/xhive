/* Shared finite-state filter algorithms, instantiated once per scalar format.
 * Every step computes candidate scalars/buffers first and commits only after
 * all arithmetic and quantization succeed. Scratch is not persistent state. */
#include "filters_internal.h"

static linears_status_t F_LOCAL(output)(const void *ctx, size_t bytes, const F_S *output)
{
    if (ctx == NULL || output == NULL) return LINEARS_ERROR_ARGUMENT;
    if (filters_overlap(ctx, bytes, output, sizeof(*output))) return LINEARS_ERROR_ALIAS;
    return LINEARS_OK;
}

static linears_status_t F_LOCAL(array_size)(size_t count, size_t element_size, size_t *bytes)
{
    if (count == 0) return LINEARS_ERROR_ARGUMENT;
    if (count > SIZE_MAX / element_size) return LINEARS_ERROR_SIZE;
    *bytes = count * element_size;
    return LINEARS_OK;
}

linears_status_t F_FN(lowpass, init)(F_CTX(lowpass) *ctx, F_S alpha, F_S initial)
{
    F_CTX(lowpass) result;
    if (ctx == NULL) return LINEARS_ERROR_ARGUMENT;
    F_TRY(F_LOCAL(weight)(alpha, &result.alpha));
    F_TRY(F_LOCAL(canonical)(initial, &result.value));
    *ctx = result;
    return LINEARS_OK;
}

linears_status_t F_FN(lowpass, reset)(F_CTX(lowpass) *ctx, F_S initial)
{
    F_S value;
    if (ctx == NULL) return LINEARS_ERROR_ARGUMENT;
    F_TRY(F_LOCAL(canonical)(initial, &value));
    ctx->value = value;
    return LINEARS_OK;
}

linears_status_t F_FN(lowpass, step)(F_CTX(lowpass) *ctx, F_S input, F_S *output)
{
    F_W alpha, previous, sample, candidate;
    F_S result;
    F_TRY(F_LOCAL(output)(ctx, sizeof(*ctx), output));
    F_TRY(F_LOCAL(load)(input, &sample));
    F_TRY(F_LOCAL(load)(ctx->alpha, &alpha));
    F_TRY(F_LOCAL(load)(ctx->value, &previous));
    F_TRY(F_LOCAL(blend)(alpha, previous, sample, &candidate));
    F_TRY(F_LOCAL(store)(candidate, &result));
    ctx->value = result;
    *output = result;
    return LINEARS_OK;
}

linears_status_t F_FN(ema, init)(F_CTX(ema) *ctx, F_S alpha, F_S initial)
{
    return F_FN(lowpass, init)(ctx, alpha, initial);
}

linears_status_t F_FN(ema, reset)(F_CTX(ema) *ctx, F_S initial)
{
    return F_FN(lowpass, reset)(ctx, initial);
}

linears_status_t F_FN(ema, step)(F_CTX(ema) *ctx, F_S input, F_S *output)
{
    return F_FN(lowpass, step)(ctx, input, output);
}

linears_status_t F_FN(highpass, init)(F_CTX(highpass) *ctx, F_S alpha, F_S initial_input,
                                     F_S initial_output)
{
    F_CTX(highpass) result;
    if (ctx == NULL) return LINEARS_ERROR_ARGUMENT;
    F_TRY(F_LOCAL(weight)(alpha, &result.alpha));
    F_TRY(F_LOCAL(canonical)(initial_input, &result.previous_input));
    F_TRY(F_LOCAL(canonical)(initial_output, &result.value));
    *ctx = result;
    return LINEARS_OK;
}

linears_status_t F_FN(highpass, reset)(F_CTX(highpass) *ctx, F_S initial_input,
                                      F_S initial_output)
{
    F_S previous, output;
    if (ctx == NULL) return LINEARS_ERROR_ARGUMENT;
    F_TRY(F_LOCAL(canonical)(initial_input, &previous));
    F_TRY(F_LOCAL(canonical)(initial_output, &output));
    ctx->previous_input = previous;
    ctx->value = output;
    return LINEARS_OK;
}

linears_status_t F_FN(highpass, step)(F_CTX(highpass) *ctx, F_S input, F_S *output)
{
    F_W sample, previous_input, previous_output, alpha, delta, sum, candidate;
    F_S stored_input, result;
    F_TRY(F_LOCAL(output)(ctx, sizeof(*ctx), output));
    F_TRY(F_LOCAL(load)(input, &sample));
    F_TRY(F_LOCAL(store)(sample, &stored_input));
    F_TRY(F_LOCAL(load)(ctx->alpha, &alpha));
    F_TRY(F_LOCAL(load)(ctx->previous_input, &previous_input));
    F_TRY(F_LOCAL(load)(ctx->value, &previous_output));
    if (F_CMP(alpha, F_LOCAL(zero)()) == 0) candidate = F_LOCAL(zero)();
    else {
        F_TRY(F_SUB(sample, previous_input, &delta));
        F_TRY(F_ADD(previous_output, delta, &sum));
        F_TRY(F_MUL(alpha, sum, &candidate));
    }
    F_TRY(F_LOCAL(store)(candidate, &result));
    ctx->previous_input = stored_input;
    ctx->value = result;
    *output = result;
    return LINEARS_OK;
}

linears_status_t F_FN(complementary, init)(F_CTX(complementary) *ctx, F_S alpha,
                                          F_S sample_period, F_S initial)
{
    F_CTX(complementary) result;
    F_W period;
    if (ctx == NULL) return LINEARS_ERROR_ARGUMENT;
    F_TRY(F_LOCAL(weight)(alpha, &result.alpha));
    F_TRY(F_LOCAL(load)(sample_period, &period));
    if (F_CMP(period, F_LOCAL(zero)()) <= 0) return LINEARS_ERROR_DOMAIN;
    F_TRY(F_LOCAL(store)(period, &result.sample_period));
    F_TRY(F_LOCAL(canonical)(initial, &result.value));
    *ctx = result;
    return LINEARS_OK;
}

linears_status_t F_FN(complementary, reset)(F_CTX(complementary) *ctx, F_S initial)
{
    F_S value;
    if (ctx == NULL) return LINEARS_ERROR_ARGUMENT;
    F_TRY(F_LOCAL(canonical)(initial, &value));
    ctx->value = value;
    return LINEARS_OK;
}

linears_status_t F_FN(complementary, step)(F_CTX(complementary) *ctx, F_S rate,
                                          F_S measurement, F_S *output)
{
    F_W derivative, observed, previous, period, alpha, increment, prediction, candidate;
    F_S result;
    F_TRY(F_LOCAL(output)(ctx, sizeof(*ctx), output));
    F_TRY(F_LOCAL(load)(rate, &derivative));
    F_TRY(F_LOCAL(load)(measurement, &observed));
    F_TRY(F_LOCAL(load)(ctx->alpha, &alpha));
    F_TRY(F_LOCAL(load)(ctx->sample_period, &period));
    F_TRY(F_LOCAL(load)(ctx->value, &previous));
    if (F_CMP(alpha, F_LOCAL(zero)()) == 0) candidate = observed;
    else {
        F_TRY(F_MUL(derivative, period, &increment));
        F_TRY(F_ADD(previous, increment, &prediction));
        F_TRY(F_LOCAL(blend)(alpha, observed, prediction, &candidate));
    }
    F_TRY(F_LOCAL(store)(candidate, &result));

    ctx->value = result;
    *output = result;
    return LINEARS_OK;
}

linears_status_t F_FN(fir, init)(F_CTX(fir) *ctx, const F_S *coefficients, size_t taps,
                                F_S *state, size_t state_capacity)
{
    size_t bytes, i;
    F_W coefficient;
    F_S zero = {0};
    if (ctx == NULL || coefficients == NULL || state == NULL) return LINEARS_ERROR_ARGUMENT;
    F_TRY(F_LOCAL(array_size)(taps, sizeof(F_S), &bytes));
    if (state_capacity < taps) return LINEARS_ERROR_WORKSPACE;
    if (filters_overlap(ctx, sizeof(*ctx), coefficients, bytes) ||
        filters_overlap(ctx, sizeof(*ctx), state, bytes) ||
        filters_overlap(coefficients, bytes, state, bytes)) return LINEARS_ERROR_ALIAS;
    for (i = 0; i < taps; ++i) F_TRY(F_LOCAL(load)(coefficients[i], &coefficient));
    for (i = 0; i < taps; ++i) state[i] = zero;
    ctx->coefficients = coefficients;
    ctx->state = state;
    ctx->taps = taps;
    ctx->head = 0;
    return LINEARS_OK;
}

linears_status_t F_FN(fir, reset)(F_CTX(fir) *ctx)
{
    size_t i;
    F_S zero = {0};
    if (ctx == NULL) return LINEARS_ERROR_ARGUMENT;
    for (i = 0; i < ctx->taps; ++i) ctx->state[i] = zero;
    ctx->head = 0;
    return LINEARS_OK;
}

linears_status_t F_FN(fir, step)(F_CTX(fir) *ctx, F_S input, F_S *output)
{
    size_t i, index, bytes;
    F_W sample, delayed, coefficient, term, sum = F_LOCAL(zero)();
    F_S stored_input, result;
    F_TRY(F_LOCAL(output)(ctx, sizeof(*ctx), output));
    bytes = ctx->taps * sizeof(F_S);
    if (filters_overlap(output, sizeof(*output), ctx->state, bytes) ||
        filters_overlap(output, sizeof(*output), ctx->coefficients, bytes)) return LINEARS_ERROR_ALIAS;
    F_TRY(F_LOCAL(load)(input, &sample));
    F_TRY(F_LOCAL(store)(sample, &stored_input));
    index = ctx->head;
    for (i = 0; i < ctx->taps; ++i) {
        if (i == 0) delayed = sample;
        else {
            index = index == 0 ? ctx->taps - 1 : index - 1;
            F_TRY(F_LOCAL(load)(ctx->state[index], &delayed));
        }
        F_TRY(F_LOCAL(load)(ctx->coefficients[i], &coefficient));
        F_TRY(F_MUL(coefficient, delayed, &term));
        F_TRY(F_ADD(sum, term, &sum));
    }
    F_TRY(F_LOCAL(store)(sum, &result));
    ctx->state[ctx->head] = stored_input;
    ctx->head = ctx->head + 1 == ctx->taps ? 0 : ctx->head + 1;
    *output = result;
    return LINEARS_OK;
}

linears_status_t F_FN(biquad, init)(F_CTX(biquad) *ctx,
                                   const F_FN(biquad, coefficients_t) *coefficients,
                                   size_t sections, F_S *state, size_t state_capacity,
                                   F_S *workspace, size_t workspace_capacity)
{
    size_t state_count, state_bytes, coefficient_bytes, i;
    F_W value;
    F_S zero = {0};
    if (ctx == NULL || coefficients == NULL || state == NULL || workspace == NULL)
        return LINEARS_ERROR_ARGUMENT;
    state_count = FILTERS_BIQUAD_STATE_COUNT(sections);
    if (state_count == SIZE_MAX) return LINEARS_ERROR_SIZE;
    F_TRY(F_LOCAL(array_size)(state_count, sizeof(F_S), &state_bytes));
    F_TRY(F_LOCAL(array_size)(sections, sizeof(*coefficients), &coefficient_bytes));
    if (state_capacity < state_count || workspace_capacity < state_count)
        return LINEARS_ERROR_WORKSPACE;
    if (filters_overlap(ctx, sizeof(*ctx), coefficients, coefficient_bytes) ||
        filters_overlap(ctx, sizeof(*ctx), state, state_bytes) ||
        filters_overlap(ctx, sizeof(*ctx), workspace, state_bytes) ||
        filters_overlap(coefficients, coefficient_bytes, state, state_bytes) ||
        filters_overlap(coefficients, coefficient_bytes, workspace, state_bytes) ||
        filters_overlap(state, state_bytes, workspace, state_bytes)) return LINEARS_ERROR_ALIAS;
    for (i = 0; i < sections; ++i) {
        F_TRY(F_LOCAL(load)(coefficients[i].b0, &value));
        F_TRY(F_LOCAL(load)(coefficients[i].b1, &value));
        F_TRY(F_LOCAL(load)(coefficients[i].b2, &value));
        F_TRY(F_LOCAL(load)(coefficients[i].a1, &value));
        F_TRY(F_LOCAL(load)(coefficients[i].a2, &value));
    }
    for (i = 0; i < state_count; ++i) state[i] = zero;
    ctx->coefficients = coefficients;
    ctx->state = state;
    ctx->workspace = workspace;
    ctx->sections = sections;
    return LINEARS_OK;
}

linears_status_t F_FN(biquad, reset)(F_CTX(biquad) *ctx)
{
    size_t i;
    F_S zero = {0};
    if (ctx == NULL) return LINEARS_ERROR_ARGUMENT;
    for (i = 0; i < 2 * ctx->sections; ++i) ctx->state[i] = zero;
    return LINEARS_OK;
}

linears_status_t F_FN(biquad, step)(F_CTX(biquad) *ctx, F_S input, F_S *output)
{
    size_t i, bytes;
    F_W sample, b0, b1, b2, a1, a2, s1, s2, term, feedback, y, next1, next2;
    F_S result;
    F_TRY(F_LOCAL(output)(ctx, sizeof(*ctx), output));
    bytes = 2 * ctx->sections * sizeof(F_S);
    if (filters_overlap(output, sizeof(*output), ctx->state, bytes) ||
        filters_overlap(output, sizeof(*output), ctx->workspace, bytes) ||
        filters_overlap(output, sizeof(*output), ctx->coefficients,
                        ctx->sections * sizeof(*ctx->coefficients))) return LINEARS_ERROR_ALIAS;
    F_TRY(F_LOCAL(load)(input, &sample));
    /* Successful init guarantees at least one section. */
    i = 0;
    do {
        F_TRY(F_LOCAL(load)(ctx->coefficients[i].b0, &b0));
        F_TRY(F_LOCAL(load)(ctx->coefficients[i].b1, &b1));
        F_TRY(F_LOCAL(load)(ctx->coefficients[i].b2, &b2));
        F_TRY(F_LOCAL(load)(ctx->coefficients[i].a1, &a1));
        F_TRY(F_LOCAL(load)(ctx->coefficients[i].a2, &a2));
        F_TRY(F_LOCAL(load)(ctx->state[2 * i], &s1));
        F_TRY(F_LOCAL(load)(ctx->state[2 * i + 1], &s2));
        F_TRY(F_MUL(b0, sample, &term));
        F_TRY(F_ADD(term, s1, &y));
        F_TRY(F_MUL(b1, sample, &term));
        F_TRY(F_MUL(a1, y, &feedback));
        F_TRY(F_SUB(term, feedback, &next1));
        F_TRY(F_ADD(next1, s2, &next1));
        F_TRY(F_MUL(b2, sample, &term));
        F_TRY(F_MUL(a2, y, &feedback));
        F_TRY(F_SUB(term, feedback, &next2));
        F_TRY(F_LOCAL(store)(next1, &ctx->workspace[2 * i]));
        F_TRY(F_LOCAL(store)(next2, &ctx->workspace[2 * i + 1]));
        /* Quantize section output as if each section were called separately. */
        F_TRY(F_LOCAL(store)(y, &result));
        F_TRY(F_LOCAL(load)(result, &sample));
    } while (++i < ctx->sections);
    for (i = 0; i < 2 * ctx->sections; ++i) ctx->state[i] = ctx->workspace[i];
    *output = result;
    return LINEARS_OK;
}

linears_status_t F_FN(moving_average, init)(F_CTX(moving_average) *ctx, size_t window,
                                           F_S *state, size_t state_capacity)
{
    size_t bytes, i;
    F_S zero = {0};
    if (ctx == NULL || state == NULL) return LINEARS_ERROR_ARGUMENT;
    F_TRY(F_LOCAL(array_size)(window, sizeof(F_S), &bytes));
    if (state_capacity < window) return LINEARS_ERROR_WORKSPACE;
    if (filters_overlap(ctx, sizeof(*ctx), state, bytes)) return LINEARS_ERROR_ALIAS;
    for (i = 0; i < window; ++i) state[i] = zero;
    ctx->state = state;
    ctx->window = window;
    ctx->head = 0;
    return LINEARS_OK;
}

linears_status_t F_FN(moving_average, reset)(F_CTX(moving_average) *ctx)
{
    size_t i;
    F_S zero = {0};
    if (ctx == NULL) return LINEARS_ERROR_ARGUMENT;
    for (i = 0; i < ctx->window; ++i) ctx->state[i] = zero;
    ctx->head = 0;
    return LINEARS_OK;
}

linears_status_t F_FN(moving_average, step)(F_CTX(moving_average) *ctx, F_S input,
                                           F_S *output)
{
    size_t i;
    F_W sample, value, divisor, term, sum = F_LOCAL(zero)();
    F_S stored_input, result;
    F_TRY(F_LOCAL(output)(ctx, sizeof(*ctx), output));
    if (filters_overlap(output, sizeof(*output), ctx->state, ctx->window * sizeof(F_S)))
        return LINEARS_ERROR_ALIAS;
    F_TRY(F_LOCAL(load)(input, &sample));
    F_TRY(F_LOCAL(store)(sample, &stored_input));
    F_TRY(F_LOCAL(count)(ctx->window, &divisor));
    /* Scale before summation so a finite average need not overflow its sum. */
    for (i = 0; i < ctx->window; ++i) {
        if (i == ctx->head) value = sample;
        else F_TRY(F_LOCAL(load)(ctx->state[i], &value));
        F_TRY(F_DIV(value, divisor, &term));
        F_TRY(F_ADD(sum, term, &sum));
    }
    F_TRY(F_LOCAL(store)(sum, &result));
    ctx->state[ctx->head] = stored_input;
    ctx->head = ctx->head + 1 == ctx->window ? 0 : ctx->head + 1;
    *output = result;
    return LINEARS_OK;
}

linears_status_t F_FN(median, init)(F_CTX(median) *ctx, size_t window, F_S *state,
                                   size_t state_capacity, F_S *workspace,
                                   size_t workspace_capacity)
{
    size_t bytes, i;
    F_S zero = {0};
    if (ctx == NULL || state == NULL || workspace == NULL) return LINEARS_ERROR_ARGUMENT;
    F_TRY(F_LOCAL(array_size)(window, sizeof(F_S), &bytes));
    if (state_capacity < window || workspace_capacity < window) return LINEARS_ERROR_WORKSPACE;
    if (filters_overlap(ctx, sizeof(*ctx), state, bytes) ||
        filters_overlap(ctx, sizeof(*ctx), workspace, bytes) ||
        filters_overlap(state, bytes, workspace, bytes)) return LINEARS_ERROR_ALIAS;
    for (i = 0; i < window; ++i) state[i] = zero;
    ctx->state = state;
    ctx->workspace = workspace;
    ctx->window = window;
    ctx->head = 0;
    return LINEARS_OK;
}

linears_status_t F_FN(median, reset)(F_CTX(median) *ctx)
{
    size_t i;
    F_S zero = {0};
    if (ctx == NULL) return LINEARS_ERROR_ARGUMENT;
    for (i = 0; i < ctx->window; ++i) ctx->state[i] = zero;
    ctx->head = 0;
    return LINEARS_OK;
}

linears_status_t F_FN(median, step)(F_CTX(median) *ctx, F_S input, F_S *output)
{
    size_t i, j, bytes;
    F_W sample, key, compared, left, right, two, candidate;
    F_S stored_input, result, key_scalar;
    F_TRY(F_LOCAL(output)(ctx, sizeof(*ctx), output));
    bytes = ctx->window * sizeof(F_S);
    if (filters_overlap(output, sizeof(*output), ctx->state, bytes) ||
        filters_overlap(output, sizeof(*output), ctx->workspace, bytes)) return LINEARS_ERROR_ALIAS;
    F_TRY(F_LOCAL(load)(input, &sample));
    F_TRY(F_LOCAL(store)(sample, &stored_input));
    for (i = 0; i < ctx->window; ++i)
        ctx->workspace[i] = i == ctx->head ? stored_input : ctx->state[i];
    /* Insertion sort is bounded by window^2 comparisons and needs no stack. */
    for (i = 1; i < ctx->window; ++i) {
        key_scalar = ctx->workspace[i];
        F_TRY(F_LOCAL(load)(key_scalar, &key));
        j = i;
        while (j != 0) {
            F_TRY(F_LOCAL(load)(ctx->workspace[j - 1], &compared));
            if (F_CMP(compared, key) <= 0) break;
            ctx->workspace[j] = ctx->workspace[j - 1];
            --j;
        }
        ctx->workspace[j] = key_scalar;
    }
    result = ctx->workspace[ctx->window / 2];
    if (ctx->window % 2 == 0) {
        F_TRY(F_LOCAL(load)(ctx->workspace[ctx->window / 2 - 1], &left));
        F_TRY(F_LOCAL(load)(result, &right));
        F_TRY(F_LOCAL(count)(2, &two));
        F_TRY(F_DIV(left, two, &left));
        F_TRY(F_DIV(right, two, &right));
        F_TRY(F_ADD(left, right, &candidate));
        F_TRY(F_LOCAL(store)(candidate, &result));
    }
    ctx->state[ctx->head] = stored_input;
    ctx->head = ctx->head + 1 == ctx->window ? 0 : ctx->head + 1;
    *output = result;
    return LINEARS_OK;
}
