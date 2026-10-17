# Voltage-model direct torque control

`fieldctrl_dtc.h` provides allocation-free C99 `f32`, `f64`, `q15` and `q31`
implementations of a six-sector, hysteresis-based direct torque controller.
It returns a discrete two-level-inverter request, **not** a PWM duty cycle and
not evidence that a gate driver executed the request. All persistent state is
in a caller-owned context. No implementation allocates, recurses, uses a VLA or
accesses hardware.

This implementation uses the **basic switching table (BST)** analyzed for PMSM
in [1, Sections 2.1–2.3 and Table 1], not that paper's proposed flexible table
(FST). BST originated in induction-motor DTC, but its use with PMSM and its
limitations are explicitly discussed in [1]. Its ancestry does not establish
stability for every surface/interior PMSM, speed, load angle or supply voltage.

## Coordinates, units and equations

Use amplitude-invariant alpha/beta quantities: alpha is aligned with phase a,
beta is positive 90 electrical degrees from alpha. Positive rotation is alpha
toward beta. Angles, where describing sectors, are **electrical turns**, with
one turn equal to `2*pi` radians. Electrical and mechanical speed differ by the
integer pole-pair count `p`.

SI units are A, V, ohm, Wb, N*m and s. A per-unit conversion is allowed only if
it preserves the scaling of both equations below; arbitrary independent
current, voltage and torque normalizations do not preserve the factor `3/2`.

For each accepted sample:

```text
psi_alpha_next = psi_alpha + dt * (applied_voltage_alpha - R * current_alpha)
psi_beta_next  = psi_beta  + dt * (applied_voltage_beta  - R * current_beta)
flux_magnitude = hypot(psi_alpha_next, psi_beta_next)
torque = (3/2) * p * (psi_alpha_next * current_beta
                     - psi_beta_next * current_alpha)
```

The current sample is held constant over the Euler interval. Estimates and
switching decisions use the **newly integrated** stator flux, then state and
output commit together. Flux magnitude uses scaled `hypot` arithmetic rather
than unscaled squaring that can overflow unnecessarily.

The torque cross product is the electromagnetic torque relation for a balanced
sinusoidal PMSM in these coordinates; it is not an induction-machine-only
formula. For a nonsalient surface PMSM, substituting
`psi_s = L*i_s + psi_pm*[cos(theta), sin(theta)]` gives
`torque = (3/2)*p*psi_pm*i_q`. For a salient machine the stator-flux relation also
contains the d/q inductance difference. Correct torque estimation alone does
not prove that the same fixed switching table has the desired torque derivative
at every salient-machine operating point.

## Interface and state

For `S = f32, f64, q15, q31`:

```text
fieldctrl_S_dtc_config_t { resistance, flux_band, torque_band, pole_pairs }
fieldctrl_S_dtc_t        { config, flux_alpha, flux_beta,
                          flux_state, torque_state, last_switch }
fieldctrl_S_dtc_result_t { stator_flux, flux_magnitude, torque,
                          sector, switch_state, flux_state, torque_state }

fieldctrl_S_dtc_init(ctx, config, initial_flux)
fieldctrl_S_dtc_step(ctx, current, applied_voltage,
                    flux_reference, torque_reference, dt, out)
```

- `config` and two-component samples are passed by value. `ctx` and `out` must
  be non-NULL, valid objects with disjoint storage.
- `R >= 0`; both hysteresis bands are strictly positive; `pole_pairs` is
  `1..65535`; `flux_reference > flux_band`; torque references are signed.
- `dt` is the actual elapsed interval in seconds and must be strictly positive.
  The API does not choose a stable sample interval or enforce a maximum current.
- Initial stator flux must be nonzero and known. For an SPMSM it can be seeded
  from a known initial rotor position and initial current. A deliberately aligned
  rotor at zero current has known PM flux orientation. An arbitrary zero seed
  is not a standstill estimator or a rotor-alignment procedure.
- Initialization sets flux memory to `+1`, torque memory to `0` and last
  requested switch to `0`. It does not energize or de-energize the inverter.
- Members are inspectable; do not edit a live context between calls. Reinitialize
  explicitly when replacing the initial flux estimate or configuration.
- Each context is reentrant relative to independent contexts. The caller must
  serialize all calls using the same context/output. Interrupt use requires
  exclusive storage and a separately established execution-time budget.
- Nonfinite floating values, invalid fixed-point exponents and arithmetic/export
  overflow fail. A zero updated flux fails with `LINEARS_ERROR_DOMAIN` because
  its sector is undefined. Invalid input scalar bounds return `ARGUMENT`,
  storage overlap returns `ALIAS`, and numerical range errors return `RANGE`.
  On **every failure, both the context and output retain their previous bytes**;
  integration is not partially committed. No fallback switching command is
  issued. Handling the still-active previous hardware state is the caller's job.

### Voltage feedback and sample timing

`applied_voltage` is the caller's actual average terminal voltage for the elapsed
interval. It may come from measured voltage, or from a hardware-validated
reconstruction using actual switch states and measured DC bus voltage with
appropriate dead-time/device-drop correction. A request returned on the previous
call is not sufficient proof of this voltage: a gate inhibit, current trip,
missed update or bus collapse changes what was actually applied.

`last_switch` is used **only** to choose between equivalent zero vectors with
fewer requested bit changes. It is never used as observer voltage feedback. If
hardware rejects a request, zero-vector selection may not minimize actual gate
transitions; flux integration still depends solely on the reported voltage.

A typical caller advances hardware for the last interval, samples current and
actual voltage consistently with that interval, calls `dtc_step`, then schedules
its returned request for the next interval. Sensor/filter and execution delays
must be included in system design; the library does not compensate them.

## Hysteresis and the switching table

Let `e_flux = flux_reference - flux_magnitude` and
`e_torque = torque_reference - torque`.

Flux is a two-state comparator:

- `e_flux >= +flux_band`: state `+1` (increase).
- `e_flux <= -flux_band`: state `-1` (decrease).
- Strictly inside these thresholds: retain the previous state.

Torque is a three-state hysteresis comparator, not a memoryless dead zone:

- `e_torque >= +torque_band`: state `+1`.
- `e_torque <= -torque_band`: state `-1`.
- Inside the outer thresholds, retain `+1` only while error remains positive,
  or `-1` only while it remains negative. At/reaching/crossing zero, return `0`.
  A neutral state stays neutral until an outer threshold is reached.

The six sectors are centered on the six active voltage vectors:
`center(k) = (k-1)/6` electrical turns, for `k = 1..6`. Sector k covers
`[center(k)-1/12, center(k)+1/12)`, modulo one turn. Thus sector 1 straddles the
positive alpha axis; these are **not** the `[0,1/6)`-style SVPWM sectors.
The implementation first checks signs and whether `|beta| >= |alpha|`; otherwise
it compares the bounded ratio `|beta|/|alpha|` with `1/sqrt(3)`. This avoids both
public-q15 angle quantization and a scale-dependent subnormal threshold such as
`|alpha|/sqrt(3)`. A boundary follows the half-open comparison at working precision;
scalar quantization can move a physical point across it.

Active vectors have these upper-switch bit patterns:

| Vector | V1 | V2 | V3 | V4 | V5 | V6 |
|---|---:|---:|---:|---:|---:|---:|
| `(Sa Sb Sc)` | 100 | 110 | 010 | 011 | 001 | 101 |
| Returned integer | 4 | 6 | 2 | 3 | 1 | 5 |
| Electrical turns | 0 | 1/6 | 2/6 | 3/6 | 4/6 | 5/6 |

The complete table below is in **returned bit-pattern integers**, not V indices:

| Flux decision | Torque decision | Sector 1 | 2 | 3 | 4 | 5 | 6 |
|---|---|---:|---:|---:|---:|---:|---:|
| +1 | +1 | 6 | 2 | 3 | 1 | 5 | 4 |
| +1 | 0  | Z | Z | Z | Z | Z | Z |
| +1 | -1 | 5 | 4 | 6 | 2 | 3 | 1 |
| -1 | +1 | 2 | 3 | 1 | 5 | 4 | 6 |
| -1 | 0  | Z | Z | Z | Z | Z | Z |
| -1 | -1 | 1 | 5 | 4 | 6 | 2 | 3 |

Equivalently, active-vector indices modulo six are `V(k+1)`, `V(k-1)`,
`V(k+2)` and `V(k-2)` for the four non-neutral rows above. **Negative torque
error selects a reverse active vector**, not a zero vector. This matters for
braking and speed reversal.

`Z` selects real state `0 = 000` or `7 = 111`, whichever differs from the
previous request in fewer bits. A previous zero state is retained. For active
states, one high bit selects 0 and two high bits select 7. This minimal-transition
choice is the zero-vector rule described in [1, Section 3]; it does not otherwise
replace BST by that paper's FST algorithm. Under an ideal floating-neutral load,
both zero states produce zero alpha/beta voltage. They do not freeze flux or
torque: `dpsi/dt = -R*i`, and PMSM rotor motion changes the load angle. Their torque
derivative depends on direction and operating point. Dead time and semiconductor
drops can make their actual average voltage nonzero, which the caller must report.

## Integer-only versions and numerical range

A q15 input/output scalar is `mantissa * 2^(exponent-15)`; q31 uses
`mantissa * 2^(exponent-31)`. Components have independent exponents, not a common
vector scale, and are not restricted to `[-1,1)`. Every raw and normalized
exponent must lie in `LINEARS_EXPONENT_MIN..LINEARS_EXPONENT_MAX`. Intermediate
values must also fit; neither saturating nor wrapping arithmetic is used.

Both q versions call the integer `linears_numeric.h` arithmetic. Their working
values and **persistent flux accumulators** are canonical `linears_q31_t`, with
31 significant magnitude bits. q15 quantization happens only at the public
output boundary, not at every integrator update. Sector/hysteresis decisions use
these working estimates, so they need not equal a decision recomputed from the
coarser displayed q15 estimates. The implementation does not convert q values
through `float`, `double`, floating trigonometry or a floating observer.

Wide state does not give unlimited precision. If an increment is smaller than
half a working flux unit, it can round away. Select signal scaling, `dt` and bands
so physically significant increments and errors are resolvable. Do not interpret
an exactly unchanged numerical flux as a hardware measurement. Normalization may
reject an exponent even when the raw exponent is legal; q15 final rounding can
also require an out-of-range exponent. These failures are atomic.

## Drift, low-speed limitations and commissioning

This is a **pure voltage integrator**, without a leaky integrator, current-model
correction, offset identification, saturation clamp or sensor-fusion observer.
If the voltage reading has bias `delta_v`, the current reading has bias `delta_i`
and the configured resistance differs by `delta_R`, the estimated flux error
contains

```text
initial_flux_error + integral(delta_v - R_est*delta_i - delta_R*i_true) dt.
```

A wrong initial flux persists even with perfect later samples; a constant voltage
or current bias accumulates with time. At low speed, back EMF is small compared
with resistive-drop and sensor-offset errors, so estimated angle/flux/torque can
be misleading while all numerical operations continue to return `LINEARS_OK`.
Resistance temperature changes, inverter drops and timing misalignment can cause
the same problem. The API has no standstill observability or automatic drift
rejection. Reinitialization requires an independently justified flux estimate;
blindly resetting to the target magnitude is not a correction of rotor direction.

For a practical installation, establish at least:

1. Correct phase order, transform normalization, pole pairs, resistance and an
   initial flux orientation obtained from alignment or a position/current model.
2. A feasible flux/torque envelope for the motor. For the surface-PMSM software
   test, `flux_ref = sqrt(psi_pm^2 + (L*Tref/(1.5*p*psi_pm))^2)` is supplied by the
   test caller. This is not an implemented MTPA/field-weakening controller and is
   not a salient-PMSM MTPA formula.
3. A sampling interval and hysteresis bands that account for voltage headroom,
   maximum torque/flux slopes, sensor quantization/noise and execution delay.
   Switching frequency is variable; smaller bands do not remove one-sample
   overshoot, and cannot repair a loss of voltage headroom or observability.
4. Independent cycle-by-cycle current protection, DC-bus limits, thermal limits,
   inverter interlocks/dead time, gate inhibit and a safe failure response. None
   of these protections is implemented by returning a switch bit pattern.
5. Operating-point validation, including positive/negative speed and torque,
   regeneration, large load angles, startup, field weakening, device drops and
   resistance drift. This BST offers no proof of stability across those regions.

The literature also warns that some zero-vector-only torque-reduction tables
cannot reverse a PMSM correctly [1, Sections 2.3–4]. This implementation avoids
that specific mistake by using active vectors for both signs of torque error;
it does **not** claim to eliminate the other voltage-model or fixed-table limits.

## Validation represented by the tests

`test/dtc.cc` is one discovered GoogleTest binary with four numeric type suites.
Its tests cover independently evaluated physical equations using quantized inputs
and a fixed random seed; all six sectors times two flux states times three torque
states; both zero vectors; exact hysteresis thresholds and memory; boundary sides;
sector invariance across floating subnormal/normal and fixed exponent scales;
32,768-step sub-q15-output-unit integration; explicit initial/sensor-bias drift;
actual voltage differing from the requested switch state; maximum pole-pair
count; independent-instance interleaving; exponent endpoints, NaN/Inf, numeric
and output-export overflow; and unchanged state/output on failures.

Numerical equation assertions propagate working-operation rounding error and add
half an output unit. Long binary integration and bias-drift cases use exact
expectations, not a percentage tolerance. Random references are evaluated **after
input quantization**, so measurement quantization is not incorrectly attributed
to arithmetic error.

The mechanical closed-loop test uses an independent RK4 surface-PMSM plant:
`R=0.4 ohm`, `L=3 mH`, `psi_pm=0.08 Wb`, `p=3`, `J=0.001 kg*m^2`,
viscous friction `0.01 N*m*s/rad`, and a 48 V DC bus. It runs both initial speeds
`+20` and `-20 rad/s`, commands `+/-0.4 N*m`, then reverses the torque command after
5,000 of 20,000 samples at approximately 10 us/sample. The plant consumes the
previous inverter request, so the simulation includes one-interval actuation
timing. Current, actual voltage and the quantized interval are the controller's
only per-step plant inputs. **The true rotor angle is not fed back into the DTC
observer**; it is used by the physical plant, diagnostics and initial flux seed.
Assertions include both speed reversals, torque tracking, current/flux bounds and
flux-observer error below 1% of the flux hysteresis half-band.

These are host software tests for this specified, initialized, nonsalient motor
model. They are not hardware measurements, startup-from-unknown-angle validation,
sensorless closed-loop certification, or proof of stability over a PMSM operating
map. No board was accessed or programmed.

## Sources

1. A. Nasr, C. Gu, S. Bozhko and C. Gerada, **Performance Enhancement of Direct
   Torque-Controlled Permanent Magnet Synchronous Motor with a Flexible Switching
   Table**, *Energies* 13(8), 1907, 2020.
   DOI: [10.3390/en13081907](https://doi.org/10.3390/en13081907).
   [Open full text](https://www.mdpi.com/1996-1073/13/8/1907).
   Equations (1)–(5) give the voltage/flux/torque relations; Section 2 and Table 1
   identify and assess BST for PMSM; Section 3 supplies the minimum-transition
   zero-vector rule; Sections 2.3 and 4 discuss operating-point and reversal
   limitations. The FST proposed in this paper is **not** implemented here.
2. L. Zhong, M. F. Rahman, W. Y. Hu and K. W. Lim, **Analysis of Direct Torque
   Control in Permanent Magnet Synchronous Motor Drives**, *IEEE Transactions on
   Power Electronics* 12(3), 528–536, 1997.
   DOI: [10.1109/63.575680](https://doi.org/10.1109/63.575680).
   Historical PMSM-DTC background, also discussed as the active-vector-only table
   in [1]; it is not cited as the source of this implementation's zero rows.
