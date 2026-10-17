# Filters implementation work log

## Scope and ownership

- This node owns `include/filters.h`, `include/filters_lowpass.h`, the basic filter sources, their classified GoogleTest cases, `README.md`, and this log. The dedicated Kalman node owns `include/filters_kalman.h`, `src/kalman*`, its two test categories and documentation.
- Build integration, SDK configuration and Git commits belong to the parent node. No root Xmake configuration is run here.

## Design decisions

- Reuse `linears_status_t`, `linears_q15_t`, `linears_q31_t`, and the pure-integer normalized `linears_fixed_t` arithmetic. Do not create a second fixed-point numeric implementation.
- Publish separate f32/f64/q15/q31 APIs. Scalar fixed-point values include a binary exponent, so signed coefficients and magnitudes above one are representable without implicit clipping.
- The low-pass coefficient is dimensionless alpha in [0, 1], with `y = (1-alpha)*y_previous + alpha*x`. EMA names expose the same recurrence. Callers select alpha once using their discretization; no hidden sampling-time conversion is performed.
- A high-pass uses `y = alpha*(y_previous + x-x_previous)`. Its caller supplies both initial input and initial output.
- FIR coefficients are in increasing delay order. A biquad section is normalized to a0=1 and uses transposed direct form II; a caller-owned tentative-state buffer prevents partial cascade commits.
- Moving average recomputes the finite zero-padded window instead of carrying a drifting running sum. Median uses caller-owned insertion-sort scratch; even windows return the mean of the two central values.
- Complementary filtering uses a caller-selected fixed sample period and `alpha*(estimate + rate*dt)+(1-alpha)*measurement`; it does not perform angle wrapping.
- Every successful step commits state and output together; argument, alias, range and arithmetic failures leave persistent state and output unchanged. Scratch contents are unspecified after an attempted step. Output must not overlap the context or any retained buffer. Initialized contexts and retained arrays are not user-editable between calls.
- No dynamic allocation, variable-length arrays, alloca, recursion, or mutable static storage. Storage count macros use size_t element counts; arithmetic that cannot be represented returns SIZE_MAX and init rejects it before accessing arrays.
- Separate contexts with disjoint mutable buffers are reentrant. A single context requires caller serialization, including interrupt callers; no lock, I/O, or hardware access is performed.

## Validation records

- The parent-owned `docs/fieldctrl-worklog.md` is the authoritative command and acceptance log. This file records only module-level design decisions.
- Early test development corrected the 32-step low-pass reference to the finite-time response, not its asymptotic limit. No tolerance was widened to hide that oracle error.
