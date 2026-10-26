# Filters implementation work log

## Scope and ownership

- This node owns `include/filters.h`, `include/filters_ema.h`, `include/filters_fir.h`, `include/filters_iir.h`, `include/filters_lpf.h`, `include/filters_hpf.h`, `include/filters_bpf.h`, the basic filter sources, their classified GoogleTest cases, `README.md`, and this log. The dedicated Kalman node owns `include/filters_kalman.h`, `src/kalman*`, its two test categories and documentation.
- Build integration, SDK configuration and Git commits belong to the parent node. No root Xmake configuration is run here.

## Design decisions

- The current user contract supersedes the initial scaled-scalar design: Q signals are raw int16/F15 and int32/F31. `linears_fixed_t` is int64 with an invariant 31-bit fraction; no scale metadata, floating conversion or normalization convention remains in the Q algorithms.
- Reuse `linears_status_t`, the fixed scalar operations and signed-128/F62 MAC/mean helpers. Do not create a second numeric kernel in filters.
- Both Q variants use wide F31 coefficients only where required: FIR taps, SOS coefficients, and complementary sample periods. Signals remain Q1. Wide SOS feedback can express -1.9; a coefficient's type does not vary its binary point.
- EMA uses `y = (1-alpha)*y_previous + alpha*x`; it is the former alpha-parameterized low-pass, renamed when `filters_lowpass.h` was removed. Floating alpha includes exact one; Q alpha is raw nonnegative Q1 and MAX means 1-LSB. Callers choose their discretization and physical signal units.
- Q EMA, LPF/HPF/BPF, complementary and SOS histories retain F31 precision, including Q15's sub-signal-LSB history; Kalman publishes raw Q estimates/covariances after wide working calculations. The single `filters_ema_work_step` entry may explicitly advance a Q EMA value field and is shared with the SMO speed estimator because its unlocked rate can exceed Q1; it adds no work-context lifecycle or general fifth API family.
- LPF/HPF/BPF are designed from frequencies in Hz with the prewarped bilinear transform: order one is the RC (LPF/HPF) or series-RLC (BPF) response, order two is Butterworth, and the order-two BPF is the low-pass-to-band-pass transform split into two stagger-tuned sections. Contexts embed direct-form-I sections, so `set` swaps coefficients without reinterpreting history and init/reset start from a held input. LPF numerators are derived from the rounded denominators (exact DC gain one); HPF/BPF taps sum to exactly zero. Q accepts float Hz, designs in double and rounds coefficients to F31 once; a Q output outside [-1,+1] returns RANGE without committing. The alpha high-pass was removed; `filters_hpf_*` replaces it.
- FIR design helpers use an odd-length Hamming-windowed sinc. Q taps are rounded to F31 and the center tap absorbs the rounding, so the taps sum exactly to ONE (low-pass) or zero (high-pass, band-pass).
- FIR coefficients are in increasing delay order. Q FIR and SOS stages accumulate exact products in shared F62 storage and quantize only at the stage boundary. SOS uses a0=1 and transposed direct form II; states and tentative states are F31, while each section output is checked and narrowed to the signal type.
- Q moving averages retain a 16-byte exact running sum and transactionally subtract the old/add the new sample before one exact sum/count conversion: O(1) without drift or per-term LSB loss. Floating moving averages retain O(window) recomputation. Median uses caller-owned insertion-sort scratch; fixed even-window means add first in wide F31, then divide once.
- Complementary filtering uses a caller-selected fixed sample period and `alpha*(estimate + rate*dt)+(1-alpha)*measurement`; it does not perform angle wrapping.
- Every successful step commits state and output together; argument, alias, range and arithmetic failures leave persistent state and output unchanged. Scratch contents are unspecified after an attempted step. Output must not overlap the context or any retained buffer. Initialized contexts and retained arrays are not user-editable between calls.
- Parent-approved matrix Kalman semantics check the computed F31 estimate/covariance at final Q export, not the ideal real-valued algorithm chain. Its n=5 regression distinguishes an exact dot of 1+2^-33, F31 rounding to ONE, and Q31 endpoint export to MAX. The direct Q matrix/MAC contracts retain their stronger exact-F62 final-domain check. No mixed-output matrix API or interval arithmetic was added to hide this finite-precision boundary.
- No dynamic allocation, variable-length arrays, alloca, recursion, or mutable static storage. Storage count macros use size_t element counts; arithmetic that cannot be represented returns SIZE_MAX and init rejects it before accessing arrays.
- Separate contexts with disjoint mutable buffers are reentrant. A single context requires caller serialization, including interrupt callers; no lock, I/O, or hardware access is performed.

## Validation records

- The parent-owned `docs/fieldctrl-worklog.md` is the authoritative command and acceptance log. This file records only module-level design decisions.
- Existing floating domains and tests are retained. Fixed fixtures explicitly select representable signals and decode actual raw coefficients for their independent references; they do not silently clip a mathematical reference.
- Removed invalid-encoding cases have been replaced by true output/working overflow, exact half-way rounding, MAX-not-one, wide-coefficient/state, raw-LSB history, and transactional state cases. Every raw Q bit pattern is valid.
- A new tiny-alpha floating regression uses a high-precision closed form: repeatedly rounding `1-alpha` in the original double oracle introduced reference drift. The acceptance tolerance was not widened.
- No root Xmake, MCU, hardware, flash or interrupt-timing validation is performed by this node; integration validation remains parent-owned.
