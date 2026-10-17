# Filters

Allocation-free C99 signal filtering and state estimation in four numeric
variants: `f32`, `f64`, `q15`, and `q31`. Fixed-point paths use only the shared
`linears` integer arithmetic; their exponent-bearing scalars represent values
outside [-1,1] without hidden saturation.

## Stable low-pass interface

Include `filters_lowpass.h` (or the complete `filters.h`). For each suffix,
`filters_lowpass_SUFFIX_t` contains the caller-owned coefficient and output
history; `init(context, alpha, initial)`, `reset(context, initial)` and
`step(context, input, output)` return `linears_status_t`.

```c
filters_lowpass_f32_t filter;
float filtered;
if (filters_lowpass_f32_init(&filter, 0.125f, 0.0f) == LINEARS_OK) {
    /* One call per sample; output may not point inside filter. */
    (void)filters_lowpass_f32_step(&filter, 2.0f, &filtered);
}
```

The recurrence is `y = (1-alpha)*previous_y + alpha*input`, with dimensionless
`alpha` in [0,1]. `alpha=0` holds the output; `alpha=1` passes the input.
For a backward-Euler RC model choose `alpha=dt/(tau+dt)` yourself, with `dt`
and `tau` in the same time units. Sample rate conversion is not implicit.

All contexts must be initialized. Do not edit context fields or retained state
arrays between API calls. Serialize access to each context, including interrupt
callers. Independent contexts with disjoint writable storage are reentrant.
No function allocates memory or performs I/O. Every rejected sample preserves
persistent state and output; documented scratch buffers may change.

## Filter families and storage

Every function name ends in `_f32`, `_f64`, `_q15`, or `_q31` before the
operation suffix: for example `filters_fir_q15_step`. `filters.h` exposes all
families; `filters_lowpass.h` is a smaller include for control-loop consumers.

| Family | State and configuration | Scratch | Equation / cost per sample |
| --- | --- | --- | --- |
| FIR | `taps` scalars; immutable `taps` coefficients | None | `sum h[k]*x[n-k]`; O(taps) |
| Biquad/SOS | `2*sections` scalars; immutable `sections` coefficient records | `2*sections` scalars | Transposed direct form II; O(sections) |
| Low-pass | Context only; `alpha`, initial output | None | `(1-alpha)*y_previous+alpha*x`; O(1) |
| High-pass | Context only; `alpha`, initial input/output | None | `alpha*(y_previous+x-x_previous)`; O(1) |
| Moving average | `window` scalars | None | Fixed-window, zero-padded mean; O(window) |
| EMA | Context only; `alpha`, initial output | None | Same recurrence and implementation as low-pass; O(1) |
| Median | `window` scalars | `window` scalars | Insertion-sorted median; O(window²), even windows average the central pair |
| Complementary | Context only; `alpha`, positive sample period, initial estimate | None | `alpha*(y_previous+rate*dt)+(1-alpha)*measurement`; O(1) |
| Scalar Kalman | Estimate and variance in context | None | Separate predict/update; Joseph variance |
| Matrix Kalman | Caller-owned state vector and covariance matrix | Three explicitly typed arrays | See [KALMAN.md](KALMAN.md) |

`FILTERS_FIR_STATE_COUNT`, `FILTERS_BIQUAD_STATE_COUNT`,
`FILTERS_BIQUAD_WORKSPACE_COUNT`, `FILTERS_MOVING_AVERAGE_STATE_COUNT`,
`FILTERS_MEDIAN_STATE_COUNT`, and `FILTERS_MEDIAN_WORKSPACE_COUNT` give required
element counts. Supply actual array capacities to init; counts are not bytes.
Multiplicative macros return `SIZE_MAX` on overflow, which is not an allocation
request. Initialization additionally checks that required byte counts fit.
There is no arbitrary maximum tap/window/order cap and no hidden storage.

FIR, SOS, moving-average and median histories initially contain zeros, and reset
restores that same state. The moving average always divides by the full window,
including during startup. Low-pass, high-pass, EMA and complementary resets use
explicit caller-selected initial values.

```c
/* Fully caller-owned FIR: h[0] multiplies the current input. */
const float taps[3] = {0.25f, 0.5f, 0.25f};
float history[FILTERS_FIR_STATE_COUNT(3)];
filters_fir_f32_t fir;
float result;
if (filters_fir_f32_init(&fir, taps, 3, history, 3) == LINEARS_OK) {
    (void)filters_fir_f32_step(&fir, 1.0f, &result); /* result = 0.25 */
}

/* Pure-integer Q15 low-pass: alpha=1/8, initial=0, input=3. */
filters_lowpass_q15_t integer_filter;
linears_q15_t alpha = {16384, -2};
linears_q15_t zero = {0, 0};
linears_q15_t sample = {24576, 2};
linears_q15_t filtered_q15;
if (filters_lowpass_q15_init(&integer_filter, alpha, zero) == LINEARS_OK) {
    (void)filters_lowpass_q15_step(&integer_filter, sample, &filtered_q15);
}
```

SOS coefficient records contain `{b0,b1,b2,a1,a2}` with `a0=1`:

```text
y    = b0*x + s1
s1'  = b1*x - a1*y + s2
s2'  = b2*x - a2*y
```

Convert coefficient sets that store negated feedback terms before using them.
Each stage output and each state is rounded to the selected storage precision;
a cascade therefore matches successive calls to individual sections. Pole
stability, coefficient design and section ordering are the caller's choices.
A successful finite calculation is not a stability guarantee.

## Numeric, ownership and failure contracts

- `f32` and `f64` use native float/double operations. They reject nonfinite
  inputs and nonfinite intermediates with `LINEARS_ERROR_RANGE`; compilation
  must not use `-ffast-math` or another mode that removes finite-value checks.
- A Q scalar represents `mantissa * 2^(exponent-15)` or
  `mantissa * 2^(exponent-31)`. Coefficients, samples and scalar states each carry
  an exponent. Matrix Kalman instead uses one exponent per matrix block.
- Q arithmetic is integer-only, through `linears_numeric.h`: normalized
  31-significant-bit working values, nearest rounding with halfway cases away
  from zero, checked exponents, and no clipping or wraparound. Final Q15 states
  are rounded to Q15 after each step, not silently retained as hidden Q31 state.
- The valid exponent interval is `LINEARS_EXPONENT_MIN..LINEARS_EXPONENT_MAX`.
  Inputs, intermediates and committed outputs must all fit it. For example,
  dividing the smallest normalized input by a multi-sample mean window can
  report `RANGE` before a mathematically representable final sum is formed.
  There is no implied gradual underflow below the integer core's range.
- Low-pass alpha endpoints and constant-input updates are exact; alpha zero
  bypasses integration in the complementary filter and emits zero in the
  high-pass. Invalid supplied samples still fail validation even at endpoints.
- Init rejects NULL pointers, zero counts, undersized state/scratch capacities,
  size overflow, invalid scalar values and overlapping retained live ranges.
  Required prefixes—not unused capacity tails—are owned by a context.
- FIR/SOS coefficient arrays remain immutable and alive while a context uses
  them. No state array or context field is caller-editable after init. Distinct
  contexts may share immutable coefficients but not writable state/scratch.
- Step output must not overlap a context, coefficient prefix, state prefix, or
  scratch prefix. Exact and partial forbidden overlaps return `ALIAS` before
  state writes. Input samples are passed by value, so an ordinary local sample
  variable may also receive output.
- A failing init/reset/step preserves persistent context, state and output.
  SOS/median scratch contents may change on failure and are not a checkpoint.
  Correct the rejected input or configuration and retry; no hidden recovery
  step is required. All functions complete synchronously and require caller
  serialization if used from both task and interrupt contexts.

## Host tests

Tests are separated into `fir.cc`, `biquad.cc`, `lowpass.cc`, `highpass.cc`,
`moving_average.cc`, `ema.cc`, `median.cc`, `complementary.cc`,
`kalman_scalar.cc` and `kalman_matrix.cc`. Every category uses all four numeric
formats. Independent difference equations or sorted-window oracles use the
quantized public inputs; matrix Kalman uses Eigen only in tests.

The suites cover seeded random and long sequences, impulse/step and applicable
frequency responses, signed and above-one gains, reset/reentrancy, exact
capacities and guards, alias rejection, nonfinite inputs, exponent failures,
state preservation and fixed-point quantization. Tolerances are derived from
mantissa precision and recurrence/operation error bounds, not relaxed after a
failed implementation result.

From the SDK root, after the normal host-test configuration:

```sh
xmake test -g xhive -v 'filters_test_*/*'
xmake test -g xhive -v 'filters_test_lowpass/lowpass'
```

Root build/configuration and accepted command results are recorded in the
parent-owned `docs/fieldctrl-worklog.md`. Host tests are not hardware validation.

## Algorithms and references

- FIR convolution, direct-form-II-transposed biquads, moving average and one-pole
  filters follow the standard difference equations in Steven W. Smith,
  *The Scientist and Engineer's Guide to Digital Signal Processing*,
  [Chapters 6, 15 and 19](https://www.dspguide.com/pdfbook.htm).
- SOS coefficient order and feedback signs are stated explicitly, not inferred
  from another library's storage convention. See also
  [SciPy sosfilt documentation](https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.sosfilt.html).
- The median is the central order statistic; for even windows this implementation
  returns the arithmetic mean of the two central values. See
  [NIST median definition](https://www.itl.nist.gov/div898/handbook/eda/section3/eda351.htm).
- Complementary filtering combines integrated rate with an absolute measurement
  using complementary first-order weights, as described in the
  [AHRS complementary-filter exposition](https://ahrs.readthedocs.io/en/latest/filters/complementary.html).
  No angle wrapping or manifold state interpretation is implied.
- Kalman filtering and its Joseph covariance update are documented in `KALMAN.md`.

The implementation is original C code derived from these published algorithms,
not copied library source. Host tests are not a claim of on-target validation.
