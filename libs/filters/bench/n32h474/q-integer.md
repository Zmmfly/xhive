# Selected Q integer-kernel optimization

The selected change accelerates PF and shares its integer helpers with UPF.
The UKF implementation and all public storage capacities remain unchanged.
Persistent Q15/Q31 arrays remain int16_t/int32_t. This is not complete
integerization and does not require lossless Q output: numerical deviations
are accepted within the existing test and workload tolerances.

## Changes

- Integer default moments, rational centering/division and final rounding.
- Bounded F40/F64 exponential/reweight work and integer normalization.
- Shared Gaussian integer MAC/triangular solve/norm with local precision retries,
  exact reciprocal caching, diagonal and power-of-two reductions.
- Integer normalized-mantissa log determinant and shared width/wide-product helpers.
- No repeated callbacks/samplers, extra particle-count arrays or API capacity changes.

## N32H474 results

Fresh f32/f64 FPU-on/off and Q15/Q31 FPU-off images ran on the same fixture
as the [committed baseline](fpu-modes.json). Each image ran 20 cases; GPIO
logic-analyzer timing agreed with DWT. Q images use zero VFP instructions.
Nominal core clock: 240 MHz; memory limits: 256 KiB Flash, 96 KiB RAM,
8 KiB reserved stack. Stack high-water was not measured.

Representative nonlinear n=m=4, N=128 results (milliseconds):

| Format | Family | Baseline | Selected | Elapsed-time change |
| --- | --- | ---: | ---: | ---: |
| q15 | PF | 4.751529 | 4.222629 | -11.13% |
| q15 | UPF | 82.606933 | 82.934042 | +0.40% |
| q31 | PF | 8.969454 | 6.998100 | -21.98% |
| q31 | UPF | 140.681750 | 138.875096 | -1.28% |

Negative time change is faster. Small UPF regressions are retained explicitly;
the selected package is not claimed faster for every case. All case timings,
hashes and numerical error summaries are in [q-integer.json](q-integer.json).

| Image | Flash bytes | Static RAM bytes |
| --- | ---: | ---: |
| f32 / FPU on | 76624 | 41760 |
| f32 / FPU off | 80844 | 41760 |
| f64 / FPU on | 81584 | 76968 |
| f64 / FPU off | 81288 | 76968 |
| q15 / FPU off | 91544 | 20744 |
| q31 / FPU off | 96424 | 38792 |

## Numerical and host validation

- SDK default build and all 76 host test entries pass.
- Strict C99 GCC O2, Clang O2 and Clang ASan/UBSan/float-cast-overflow:
  49 binaries, 1,055 passing GoogleTest cases and 8 existing skips per mode.
  Final production/test source hashes exactly match that verified snapshot.
- All 23,352 Q output elements match the frozen host workload; original
  baseline/floating comparison tolerances pass. PF estimates also pass exact
  integer-rational mean/covariance checks. Normalized weights are compared as
  probabilities, not by their common scale.
- The workload thresholds remain 4e-3 for Q15 and 2e-7 for Q31, multiplied
  by (1+abs(reference)); these are validation thresholds, not a new universal
  filter-accuracy promise.
- The four floating images retain all baseline per-case status/hash/checksum
  values. Their kernels were not changed.
- Host tests, cross-build inspection and board runs are distinct evidence.

## Scope decision

The experimental UKF native-covariance certificate was not selected. Its
initial unconditional scan regressed the ordinary workload; lazy scanning
removed that cost, but the new native path was not exercised by this workload
and still added 37,152 / 38,440 Flash bytes (q15/q31). Experimental snapshots
and failed results remain in the persistent artifact directory, not in this commit.
The selected shared PF/UPF implementation permits bounded numerical changes
rather than requiring raw-byte equality with the original Q baseline.

Cholesky, callbacks, some scalar boundaries and the existing mixed-Q UKF
arithmetic remain floating. No further lossless/performance exploration is
part of this delivery. Reproduction entry points and complete source/image
hashes are recorded in q-integer.json.
