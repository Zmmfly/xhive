# fieldctrl

Allocation-free C99 software algorithms for BLDC/PMSM drives, provided as
`f32`, `f64`, `q15` and `q31` entry points. This library does not access PWM,
ADCs, GPIO, interrupts or a motor. Hardware operation is not validated here.

## Numeric and coordinate conventions

- Scalar q values use `linears_q15_t`/`linears_q31_t`: value = mantissa times
  `2^(exponent - 15)` or `2^(exponent - 31)`. Each scalar has its own exponent.
  Fixed-point algorithms use the integer-only `linears_numeric.h` core;
  they do not convert to floating point.
- Angles are **electrical turns**, positive from alpha to beta. One turn equals
  `2*pi` radians. Wrapped angles lie in `[0,1)`. Frequencies are electrical
  turns/second (Hz). Mechanical speed interfaces state their pole-pair mapping.
- Clarke is amplitude-invariant: alpha = `(2a-b-c)/3`, beta = `(b-c)/sqrt(3)`.
  It removes zero sequence. The inverse produces balanced phase values.
- Park uses d = alpha*cos + beta*sin and q = -alpha*sin + beta*cos.
- Voltage/current/flux use SI units or one mutually consistent per-unit system.
  Gains and sample time must use the corresponding units.
- All contexts are caller-owned. Calls on separate contexts are reentrant.
  The caller serializes calls sharing a context. There is no mutable global
  state, heap allocation, VLA, alloca or recursive algorithm.
- Calls use fixed-size scalar/vector/context locals. No dimension-dependent
  workspaces are required by this library.

## Control primitives

`fieldctrl_base.h` provides coordinate transforms, trigonometry, conditional
anti-windup PI, radial voltage limiting, SVPWM duty calculation and VVVF.

### State precision and failures

The q15 public values are quantized at the API boundary, but PI integrals and
VVVF phase retain the canonical Q31 working representation. This prevents a
small `ki*dt*error` or frequency increment from disappearing at every q15 state
update. q31 uses the same working scalar. f32/f64 use native float/double state.
The precision is finite: increments smaller than half a working ULP can still
round away. All q arithmetic is 64-bit integer arithmetic, including division,
square root, normalization, and at most 32 CORDIC rotations/vectoring steps.
There is no hidden floating conversion in q entry points. The CORDIC constants
are nearest-rounded `atan(2^-i)/(2*pi)` in Q62 turns and gain in Q60.

Invalid arguments, nonfinite floating inputs, invalid exponents and arithmetic
range errors leave every output and mutable context unchanged. Intentional
voltage/PI clipping is successful calculation, not a range error. Outputs may
not overlap mutable contexts; multi-output sine/cosine ranges must be disjoint.
Initialize contexts through their init entry point and do not edit their fields.

Radial component rescaling separates mantissas from exponents before evaluating
`component*limit/magnitude`. Floating variants use native `frexp`/`ldexp`; q
variants multiply/divide normalized mantissas with linears, combine exponents
in int64, then validate the final exponent. Neither an underflowing `limit/norm`
nor an underflowing `component/norm` may erase a representable final component.
The general voltage limiter and FOC do materialize the vector norm at working
precision: two sufficiently large finite components can have an unrepresentable
norm. They report RANGE without committing, even if a clipped vector would fit.
SVPWM avoids that particular magnitude overflow by normalizing before its norm.

### PI and modulation

The PI forms `I_candidate=clamp(I+ki*dt*error,lower,upper)`. If
`kp*error+I_candidate` exceeds a limit and the integration increment drives
further into that limit, it rejects the increment and recomputes from old I.
It then clamps the command. This is conditional integration, not back-calculation.
The caller selects a stable sample period; the primitive enforces positive dt
and representable arithmetic, not a plant-specific stability criterion.

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
sensorless standstill self-start.

## Algorithm references

- R. H. Park, “Two-Reaction Theory of Synchronous Machines,” AIEE Transactions,
  vol. 48, 1929, pp. 716–727: rotating-frame transformation.
- J. E. Volder, “The CORDIC Trigonometric Computing Technique,” IRE Transactions
  on Electronic Computers, EC-8, 1959, pp. 330–334: finite rotation/vectoring.
- K. J. Åström and T. Hägglund, *PID Controllers: Theory, Design, and Tuning*,
  2nd ed., ISA, 1995: conditional integration as anti-windup.
- H. W. van der Broeck, H.-C. Skudelny and G. V. Stanke, “Analysis and
  Realization of a Pulsewidth Modulator Based on Voltage Space Vectors,” IEEE
  Transactions on Industry Applications, 24(1), 1988, pp. 142–150: space-vector
  PWM. The implementation uses the equivalent max/min common-mode offset.

## Validation

The category tests (`transforms`, `pi_pwm`, `vvvf`) instantiate f32, f64, q15
and q31. They use fixed-seed independent equations with quantized reference
inputs, cardinal/zero and invalid-value cases, alias/failure atomicity checks,
a 12,000-sample closed-loop RL plant with saturation and direction changes,
20,000 integral updates and a 16,000-sample frequency/amplitude ramp. The VVVF
voltage oracle uses long-double trigonometry so a rounded full-turn double
argument does not dominate the f64 error budget. None are hardware-in-the-loop
tests. SDK build/test commands and results are recorded in the integration log.
