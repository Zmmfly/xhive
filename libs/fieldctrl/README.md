# fieldctrl

Allocation-free C99 software algorithms for BLDC/PMSM drives, provided as
`f32`, `f64`, `q15` and `q31` entry points. This library does not access PWM,
ADCs, GPIO, interrupts or a motor. Hardware operation is not validated here.

## Numeric and coordinate conventions

- `linears_q15_t` is `int16_t` with **15 fixed fraction bits**: raw/32768.
  `linears_q31_t` is `int32_t` with **31 fixed fraction bits**: raw/2147483648.
  Their exact signal ranges are `[-1,1-2^-15]` and `[-1,1-2^-31]`. No scalar,
  vector or context carries a variable binary point. Q paths use the shared
  integer-only `linears_numeric.h` core, never a floating-point wrapper.
- Explicitly wide Q gains/state/diagnostics use `linears_fixed_t`, an `int64_t`
  **always divided by 2^31**, including in the q15 API. Its range is
  `[-2^32,2^32-2^-31]`. Do not pass a Q15 raw gain to a wide-coefficient argument.
- Angles are **electrical turns**, positive from alpha to beta. One turn equals
  `2*pi` radians. Wrapped angles lie in `[0,1)`. Floating frequency arguments
  are electrical turns/second (Hz); Q frequencies are `f*Tbase`. Mechanical
  speed interfaces explicitly divide electrical frequency by pole pairs.
- Clarke is amplitude-invariant: alpha = `(2a-b-c)/3`, beta = `(b-c)/sqrt(3)`.
  It removes zero sequence. The inverse produces balanced phase values.
- Park uses d = alpha*cos + beta*sin and q = -alpha*sin + beta*cos.
- Floating APIs document SI units. Q APIs require the per-unit conversion
  below; raw signal storage cannot represent an unscaled 50 Hz or 24 V value.
- All contexts are caller-owned. Calls on separate contexts are reentrant.
  The caller serializes calls sharing a context. There is no mutable global
  state, heap allocation, VLA, alloca or recursive algorithm.
- Calls use fixed-size scalar/vector/context locals. No dimension-dependent
  workspaces are required by this library.

## Q per-unit contract

The caller chooses positive physical bases `Vbase` (V), `Ibase` (A) and `Tbase`
(s). These are application-side conversion constants, **not context fields**.
Use one consistent selection for connected controllers and observers:

| Quantity | Q signal or coefficient value |
|---|---|
| Voltage / DC bus | `V / Vbase` |
| Current / current reference | `I / Ibase` |
| Sample time | `dt / Tbase` |
| Electrical or mechanical frequency | `f * Tbase` |
| Resistance | `R * Ibase / Vbase` |
| Inductance | `L * Ibase / (Vbase*Tbase)` |
| Stator or magnet flux linkage | `psi / (Vbase*Tbase)` |
| Torque | `T / (Vbase*Ibase*Tbase)` |
| Current-loop proportional gain | `kp_SI * Ibase / Vbase` |
| Current-loop integral gain | `ki_SI * Ibase*Tbase / Vbase` |
| Speed-loop proportional gain | `kp_SI / (Ibase*Tbase)` |
| Speed-loop integral gain | `ki_SI / Ibase` |
| Generic PID `kp` / `ki` / `kd` | `kp_SI*E/U` / `ki_SI*E*Tbase/U` / `kd_SI*E/(U*Tbase)` |
| PID derivative filter time constant | `Tf / Tbase` |

`E` and `U` are the bases of the PID error and output; the current and speed
rows are this rule with `E=Ibase, U=Vbase` and `E=1/Tbase, U=Ibase`.

All ordinary Q arguments must fit their fixed signal domain. In particular,
encoded `dt/Tbase` must be positive, and positive frequencies cannot exceed the
largest raw signal. Only PID/FOC gains, the PID derivative filter time constant
and the PID setpoint weight are wide F31 coefficients; resistance,
inductance, thresholds, actual voltages, currents, duty and angle remain bounded
signals. Gain range errors are not silently saturated. Choose bases against the
actual operating envelope and resolution, not merely to encode one nominal point.

The voltage-model equations remain `dpsi_pu = dt_pu*(v_pu-R_pu*i_pu)` and
`torque_pu = 1.5*pole_pairs*cross(psi_pu,i_pu)`. Arbitrary independent torque/flux
normalizations would change their coefficients and are not this API's contract.
For the speed/current cascade, two valid speed signals can differ by more than
one. Its private shared PID entry keeps the unrounded mechanical speed and that
error in F31 until actuator limiting; it does not narrow or clip the
intermediate error. The public PID setpoint and measurement are Q1 signals;
their difference is likewise formed in F31.

Q conversion checks the actual closed domain `[-1,+1]` before rounding.
Mathematical `+1`, or an in-domain Q15 value whose nearest rounding reaches
`32768`, maps to `INT_MAX`; this upper cell has at most one signal LSB error.
`-1` maps exactly to `INT_MIN`; actual values outside the domain return RANGE.
Raw `INT_MAX` still means `1-LSB`, including filter weights, not exactly one.
**Angles are periodic:** a Q15 computed angle in the upper half-LSB cell rounds
to zero turns instead of MAX. Q31 phases first wrap to `[0,2^31)` raw counts,
then store at the same F31 binary point.

## Control primitives

`fieldctrl_base.h` provides coordinate transforms, trigonometry, a bounded PID
with boundary anti-windup, radial voltage limiting, SVPWM duty calculation and
VVVF.

### State precision and failures

The q15 public values are quantized at the API boundary, but PID integrals and
VVVF phase retain int64 F31 state. A sub-Q15-LSB update can therefore accumulate
before the signal is rounded. Q31 already has 31 fraction bits: its wider state
adds integer headroom, not hidden fractional bits. Both Q paths round products
nearest with halfway cases away from zero. An increment below half an F31 LSB
can round to zero; there is no remainder accumulator. f32/f64 keep native state.

Q multiplication/division and exact F62 multiply-accumulates use the shared
portable two-uint64-limb core. CORDIC uses at most 32 raw integer rotations with
constant Q60 coordinates and Q62 turns. Its input ratios and final conversion
reuse integer core division/muldiv; no variable binary point is propagated.
The angle table is nearest-rounded `atan(2^-i)/(2*pi)` in Q62 turns.

Invalid arguments, nonfinite floating inputs and actual work/output range
errors leave every output and mutable context unchanged. Intentional
voltage/PID clipping is successful calculation, not a range error. Outputs may
not overlap mutable contexts; multi-output sine/cosine ranges must be disjoint.
Initialize contexts through their init entry point and do not edit their fields.

Radial component rescaling evaluates `component*limit/magnitude` without
prematurely discarding a representable result. Floating variants retain native
`frexp`/`ldexp` decomposition. Q variants use `linears_fixed_muldiv`, with an
exact wide raw product and one final rounded integer quotient. Neither an
underflowing intermediate ratio nor an overflowing intermediate product needs
to be constructed first.
The general voltage limiter and FOC do materialize the vector norm at working
precision: two sufficiently large finite components can have an unrepresentable
norm. They report RANGE without committing, even if a clipped vector would fit.
SVPWM avoids that particular magnitude overflow by normalizing before its norm.

### PID and modulation

`fieldctrl_S_pid_init(&ctx, &config, integral)` copies a configuration and an
initial integral (for a bumpless start); `fieldctrl_S_pid_step(&ctx, setpoint,
measurement, dt, &output)` returns the bounded command together with the P, I
and D terms and a `saturated` flag; `fieldctrl_S_pid_reset(&ctx, integral)`
clears the derivative history and keeps the tuning. A zero-initialized
configuration plus `kp`, `ki`, `lower` and `upper` is a plain PI; `kd`, the
derivative source and filter, and the `FIELDCTRL_PID_OPT_*` options (setpoint
weight, integral separation, independent integral limit) are opt-in. The
equations are listed once in `fieldctrl_base.h`. There are no setters or
callbacks: `fieldctrl_S_pid_retune(&ctx, &config)` validates a complete new
configuration and replaces it without restarting the controller.

- `kp`, `ki` and `kd` share one sign: all nonnegative is direct action, all
  nonpositive is reverse action (more output lowers the measurement). The step
  equations are linear in the gains, so no input negation is needed; mixed
  signs return DOMAIN. The speed cascade still requires nonnegative gains,
  because its cross-loop anti-windup assumes a direct-acting speed loop.
- Retuning keeps the state: the integral is clamped into the new integral
  bounds and the derivative history continues, so a limit that follows the bus
  voltage or a gain schedule may be applied every sample. Only a zero `kd`
  clears the stored derivative term, and the `dt` cache is invalidated only
  when `ki`, `kd` or the derivative filter change.
- Anti-windup advances a saturating integral at most to the value that puts
  the unclamped command exactly on the crossed bound, never beyond it and
  never back past its previous value. Rejecting the whole increment, the former
  conditional integration, could leave the integral short of that boundary.
- The first step after init or reset only records the error and measurement,
  so a nonzero initial value causes no derivative kick.
- `ki*dt`, `kd/dt` and `dt/(Tf+dt)` are cached and recomputed only when `dt`
  changes, which keeps both divisions out of fixed-rate loops.
- Non-positive `dt` returns DOMAIN; the primitive enforces representable
  arithmetic, not a plant-specific stability criterion.

This structure follows the generic `ezctrl` PID, adapted to fieldctrl's
status codes, atomic failures and Q per-unit contract. The speed cascade uses
the same PID with `kd=0`.

SVPWM first applies the circular linear-modulation bound `Vdc/sqrt(3)`, converts
to balanced phase voltages, subtracts `(maximum+minimum)/2` common mode, and
returns `0.5+phase/Vdc`. A final [0,1] clamp handles arithmetic roundoff. This is
linear SVPWM only, not overmodulation or six-step gate sequencing. Arithmetic
is performed in dimensionless bus-normalized coordinates before converting
physical voltage outputs, so even subnormal positive bus voltages preserve
modulation ratios. If a requested component exceeds the bus, saturation is
certain; scaling by that largest component avoids an overflowing voltage/bus
ratio while retaining direction. Duty output does not incorporate dead time,
minimum pulse widths or sensing windows.

### VVVF use

VVVF accepts a signed electrical frequency and nonnegative voltage amplitude
each call. It advances its working phase by `frequency*dt`, wraps to [0,1),
and generates a voltage vector and corresponding SVPWM duties at the new phase.
It rejects `abs(frequency)*dt > 1/2` turn to prevent sampling ambiguity. Zero
frequency holds the phase; zero amplitude yields three 0.5 duties. Amplitude
clipping is indicated in the result. Voltage-frequency laws and ramps remain
caller policy rather than an implicit motor-specific choice.

### Complete Q15 caller-side conversion example

This C99 program requests a 50 Hz electrical field, 8 V amplitude and a 24 V
bus, using `Vbase=32 V` and `Tbase=0.005 s`. A 100 us sample gives `dt_pu=0.02`,
rounded to raw 655. Consequently the represented interval is about 99.945 us;
that input quantization is not silently repaired. The context stores none of
these physical bases. This example is checked by `VvvfPerUnit` in `test/vvvf.cc`.

```c
#include "fieldctrl.h"

int main(void) {
    fieldctrl_q15_vvvf_t state;
    fieldctrl_q15_vvvf_output_t output;
    linears_status_t status = fieldctrl_q15_vvvf_init(&state, 0);
    if (status != LINEARS_OK) return 1;

    /* Initialize once. Reuse state for subsequent control samples. */
    status = fieldctrl_q15_vvvf_step(&state,
        8192,   /* 50 Hz * 0.005 s = 0.25, encoded Q1.15 */
        8192,   /* 8 V / 32 V = 0.25 */
        24576,  /* 24 V / 32 V = 0.75 */
        655,    /* (100 us / 0.005 s) * 32768, nearest integer */
        &output);
    return status == LINEARS_OK ? 0 : 1;
}
```

The first working phase is exactly raw `10731520` in F31 and the Q15 angle
output is raw `164`. There is no gate-drive action. For a gain above one, use
the different wide-coefficient format explicitly, for example
`linears_fixed_t kp = INT64_C(4) * LINEARS_FIXED_ONE;`; raw Q15 `4*32768`
is neither the correct format nor representable as a Q15 signal.

## FOC and sliding-mode observer

The umbrella `fieldctrl.h` also exposes `fieldctrl_foc.h` for d/q current control
and the mechanical-speed/current cascade, and `fieldctrl_smo.h` for the
sliding-mode current/back-EMF observer. See the
[FOC/SMO design and validation notes](README_foc_smo.md) for the motor model,
anti-windup rules, signed electrical-angle reconstruction, discrete low-pass
phase compensation, locking criteria and software-only rotating handover tests.
The observer is not a standstill startup procedure. In the documented dynamic
SMO software case, Q15 sensor quantization misses the original 0.25 Hz peak
frequency-error target under both tested caller scalings. Agreement with the
quantized-input arithmetic oracle does not make that physical target pass.
Use Q31 or a floating variant when that tested target is required; the companion
notes separate raw physical error, input quantization and working-arithmetic error.

## Direct torque control

The umbrella also exposes `fieldctrl_dtc.h` for voltage-model flux estimation,
flux/torque hysteresis and six-sector switching-table control. It returns a
requested inverter state in `0..7`, not a duty ratio or an executed gate action.
The [DTC design and validation notes](README_dtc.md) document the complete
positive/negative torque table, zero-vector selection, initial-flux requirements,
integrator drift, sample timing and the software motor-model tests.

## Safety boundary

The caller is responsible for current/voltage/temperature limits, ADC offset
calibration, dead-time and minimum-pulse handling, overcurrent trips, bus loss,
encoder validity, watchdogs, safe contactor sequencing and emergency shutdown.
The outputs are mathematical commands, not a claim that a motor can run safely.
Controller gains, observer gain/boundary layers and sample rate require tuning
against the actual resistance, inductance, flux linkage, inertia and sensing
latency. Software tests are not electrical or mechanical safety validation.

The VVVF generator is open loop: a commanded field frequency is not a measured
rotor speed. Back-EMF observers cannot identify rotor angle at standstill or
when induced voltage is below the sensing/model-error floor. A separate startup
strategy and handover criteria are required; this library does not claim
sensorless standstill self-start. Fixed signal quantization also changes model
parameters and sampled inputs; a pure DTC voltage integrator can accumulate the
resulting bias. The DTC notes report both a Q15 scaling choice that misses the
previous physical-flux error target and an engineering-selected nominal set.
These are documented limitations and unit-selection results, not hidden observer
corrections or a claim that quantization has disappeared.

## Algorithm references

- R. H. Park, “Two-Reaction Theory of Synchronous Machines,” AIEE Transactions,
  vol. 48, 1929, pp. 716–727: rotating-frame transformation.
- J. E. Volder, “The CORDIC Trigonometric Computing Technique,” IRE Transactions
  on Electronic Computers, EC-8, 1959, pp. 330–334: finite rotation/vectoring.
- K. J. Åström and T. Hägglund, *PID Controllers: Theory, Design, and Tuning*,
  2nd ed., ISA, 1995: setpoint weighting, derivative filtering and integrator
  anti-windup.
- H. W. van der Broeck, H.-C. Skudelny and G. V. Stanke, “Analysis and
  Realization of a Pulsewidth Modulator Based on Voltage Space Vectors,” IEEE
  Transactions on Industry Applications, 24(1), 1988, pp. 142–150: space-vector
  PWM. The implementation uses the equivalent max/min common-mode offset.

## Validation

The category tests (`transforms`, `pid_pwm`, `vvvf`) instantiate f32, f64, q15
and q31. They use fixed-seed independent equations with quantized reference
inputs, raw MIN/MAX/LSB and periodic endpoints, real output-domain failures,
alias/failure atomicity checks, a 12,000-sample physical RL plant with caller-side
per-unit conversion, saturation and direction changes, 20,000 integral updates,
F31 halfway/sub-LSB behavior and a 16,000-sample frequency/amplitude ramp. PID
cases use exact binary fractions for derivative sources, the derivative filter,
the dt cache, boundary anti-windup, setpoint weight, integral separation and
limits, reset and configuration validation, plus a 16,384-sample saturated
mass-position loop with derivative on measurement. Original
floating subnormal and extreme-scale counterexamples remain tests. The VVVF
voltage oracle uses long-double trigonometry so a rounded full-turn double
argument does not dominate the f64 error budget. None are hardware-in-the-loop
tests. SDK build/test commands and results are recorded in the integration log.
