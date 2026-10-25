# Filters

Allocation-free C99 signal filtering and state estimation in four numeric
variants: `f32`, `f64`, `q15`, and `q31`. Q signals are ordinary signed integers
with a fixed binary point: `int16_t/2^15` or `int32_t/2^31`. All Q arithmetic
uses the shared integer-only `linears` core; no per-value or per-block scale
metadata, floating conversion, or silent saturation is present.

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

The recurrence is `y = (1-alpha)*previous_y + alpha*input`. Floating `alpha`
is dimensionless in [0,1]; zero holds and one passes through. Q alpha is a
nonnegative raw signal in [0,1): `INT16_MAX`/`INT32_MAX` mean **1-LSB, not 1**.
For a backward-Euler RC model choose `alpha=dt/(tau+dt)` yourself, with `dt`
and `tau` in the same time units. Sample-rate conversion is not implicit.

Q low-pass, EMA, high-pass and complementary contexts retain their recursive
`value` as `linears_fixed_t`: an `int64_t` with exactly 31 fractional bits.
This preserves sub-Q15-LSB history instead of repeatedly discarding it. Output
is separately quantized to the selected signal format after a successful step.

`filters_lowpass_work_step(state, alpha_q31, input)` exposes that same integer
recurrence for an already caller-owned F31 working state, such as an observer's
unlocked speed estimate. Its state/input may exceed the Q1 signal interval;
they keep the same fixed 31-bit fraction. There is no extra context or init/reset
API. The typed Q low-pass wrappers use a local candidate, call this recurrence,
then narrow successfully before committing their context.

```c
linears_fixed_t speed_history = 8 * LINEARS_FIXED_ONE;
(void)filters_lowpass_work_step(&speed_history, INT32_C(1073741824),
                               2 * LINEARS_FIXED_ONE);
/* speed_history is now 5 * LINEARS_FIXED_ONE, still constant F31. */
```

All contexts must be initialized. Do not manually edit context fields or
retained arrays. The working low-pass entry may explicitly advance a Q context's
`value` field; a later raw step can reject that wider history if its candidate
output exceeds Q1, without losing the history. Serialize each context, including interrupt
callers. Independent contexts with disjoint writable storage are reentrant.
No function allocates memory or performs I/O. Every rejected sample preserves
persistent state and output; documented scratch buffers may change.

## Filter families and storage

Typed family functions use `_f32`, `_f64`, `_q15`, or `_q31` before the
operation suffix: for example `filters_fir_q15_step`. `filters.h` exposes all
families; `filters_lowpass.h` is a smaller include for control-loop consumers.

| Family | State and configuration | Scratch | Equation / cost per sample |
| --- | --- | --- | --- |
| FIR | `taps` signal elements; immutable coefficient array | None | `sum h[k]*x[n-k]`; O(taps) |
| Biquad/SOS | `2*sections` states, F31 for Q; immutable coefficient records | `2*sections` states of the same type | Transposed direct form II; O(sections) |
| Low-pass | Context only; `alpha`, initial output | None | `(1-alpha)*y_previous+alpha*x`; O(1) |
| High-pass | Context only; `alpha`, initial input/output | None | `alpha*(y_previous+x-x_previous)`; O(1) |
| Moving average | `window` signal elements; Q contexts also hold a 16-byte exact sum | None | Fixed-window, zero-padded mean; Q O(1), float O(window) |
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
For **both Q variants**, FIR/SOS coefficients and complementary sample periods
use `linears_fixed_t`, not the signal type: every such value is int64 raw/2^31.
This expresses ordinary SOS feedback coefficients near -2 and gains above one.
Q biquad state and scratch arrays also contain `linears_fixed_t` elements.

Q moving averages transactionally subtract the outgoing sample and add the new
sample in an exact signed 128-bit F62 sum, then divide by the integer window
count and round once. There is no integer running-sum drift or per-term LSB
loss. Floating means retain the finite-window recomputation to avoid drift.
The additional 16-byte Q context state is intentional; not every context or
workspace becomes smaller. Matrix Kalman working buffers can also grow; see
its explicit type-specific storage formulas in [KALMAN.md](KALMAN.md).

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

/* Pure-integer Q15 low-pass: alpha=1/8, initial=0, input=1/4. */
filters_lowpass_q15_t integer_filter;
linears_q15_t filtered_q15;
if (filters_lowpass_q15_init(&integer_filter, INT16_C(4096), 0) == LINEARS_OK) {
    (void)filters_lowpass_q15_step(&integer_filter, INT16_C(8192), &filtered_q15);
    /* filtered_q15 = 1024, meaning 1/32. */
}

/* Q15 FIR coefficients are wide F31, even though its delay ring is raw F15. */
const linears_fixed_t q_taps[3] = {
    LINEARS_FIXED_ONE / 4, LINEARS_FIXED_ONE / 2, LINEARS_FIXED_ONE / 4
};
linears_q15_t q_history[FILTERS_FIR_STATE_COUNT(3)];
filters_fir_q15_t q_fir;
(void)filters_fir_q15_init(&q_fir, q_taps, 3, q_history, 3);
```

SOS coefficient records contain `{b0,b1,b2,a1,a2}` with `a0=1`:

```text
y    = b0*x + s1
s1'  = b1*x - a1*y + s2
s2'  = b2*x - a2*y
```

Convert coefficient sets that store negated feedback terms before using them.
Q FIR and each SOS product sum use the shared exact signed 128-bit F62 MAC;
they do not round every product independently. Each Q section output is checked
against the signal interval and quantized directly from that sum; section
states are rounded to wide F31. A cascade therefore matches successive calls
to individual sections. An out-of-Q1 intermediate section output fails even
if a later section would reduce it. Pole stability, signal units, coefficient
design and section ordering are caller choices; success is not a stability
guarantee.

## Numeric, ownership and failure contracts

- `f32` and `f64` use native float/double operations. They reject nonfinite
  inputs and nonfinite intermediates with `LINEARS_ERROR_RANGE`; compilation
  must not use `-ffast-math` or another mode that removes finite-value checks.
- Q1.15 and Q1.31 signals lie in `[-1, 1-2^-15]` and `[-1, 1-2^-31]`.
  Choose physical signal units so inputs and required outputs fit this fixed
  range. Every raw integer bit pattern is valid; there is no Q NaN encoding.
- `linears_fixed_t` has an invariant 31-bit fraction and a signed 64-bit raw
  integer. `LINEARS_FIXED_ONE` is exactly 2147483648. Its wide integer part is
  used only where coefficients, recursive state or intermediate values need
  headroom; public signal buffers remain raw Q1.15/Q1.31.
- Arithmetic rounds to nearest with halfway cases away from zero. Signal export
  checks its supplied numeric value against `[-1,+1]` before narrowing. Exact +1,
  or an in-range value whose rounding reaches +1, maps to MAX with at most one
  signal LSB of error. An out-of-range supplied value returns
  `LINEARS_ERROR_RANGE`; there is no implicit change of scale or clipping.
- Direct final MAC paths (including FIR/SOS and the scalar Kalman final
  expressions) check their exact F62 sums before rounding. Matrix Kalman is a
  compound F31-working algorithm: shared working matrix operations round to F31,
  and its final range check applies to the **computed F31 estimate/covariance**,
  not an ideal real-arithmetic chain. For example, a final exact dot equal to
  `1+2^-33` can round to F31 ONE and then export as Q31 MAX. The working rounding
  error (`2^-33` in this example) and endpoint representation error (`2^-31`)
  are separate. Direct `linears_mul_q31` checks that same dot in F62 and rejects
  it. See [KALMAN.md](KALMAN.md) for the explicit regression and contract.
- Working operations also return `RANGE` on int64/F31 or signed-128/F62
  overflow. Success does not promise exact real-arithmetic results: finite
  fractional precision still rounds small terms. Tiny positive noise that
  quantizes to raw zero is zero and fails a strictly positive-noise contract.
- Alpha zero holds the low-pass, bypasses integration in the complementary
  filter, and emits zero in the high-pass. A raw MAX alpha never takes a
  floating passthrough shortcut. Nonfinite floating samples fail even at
  endpoints; raw Q samples need no invalid-encoding check.
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
quantized public inputs; matrix Kalman uses Eigen only in tests. The exact fixed
FIR oracle uses the host compiler's native `__int128`, deliberately independent
of the portable production two-limb implementation. These tests are verified
with Linux GCC/Clang; MSVC test compatibility is not claimed.

The suites cover seeded random and long sequences, impulse/step and applicable
frequency responses, signed and above-one gains, reset/reentrancy, exact
capacities and guards, alias rejection, nonfinite floating inputs, true range failures,
state preservation and fixed-point quantization. Tolerances are derived from
signal/F31 LSBs and recurrence/operation error bounds, not relaxed after a
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
