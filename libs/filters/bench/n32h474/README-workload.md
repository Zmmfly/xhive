# N32H474 UKF / PF / UPF workload

`workload.cc` is the allocation-free transaction driver; it does not initialize
hardware, wait for a command, flash a device, or run an event loop. The platform
owns `g_bench_results`, clock/FPU setup and timing/GPIO functions from the frozen
`bench_platform.h` / `bench_result.h`. One call to `bench_run()` runs one complete
image-format sweep and returns. Production baseline for the initial validation:
`3ee417ab63d1ee49aaa48d12540f2adfe3700975`.

## Build contract

- C++17; define exactly one `BENCH_FORMAT`: `0=f32`, `1=f64`, `2=q15`, `3=q31`.
- Compile **the driver and production C kernels** with `-fno-fast-math
  -ffp-contract=off`. FMA contraction must not differ between the hard/soft builds.
- No `HOST_BENCH` definition in firmware. `-fno-exceptions -fno-rtti` are supported.
- Required includes: this directory, `libs/filters/include`, `libs/linears/include`.
  Link the filter/linears implementation and the target's math/runtime library.
- Platform policy selects Cortex-M4 hard ABI / FPU-on for f32/f64, soft ABI /
  FPU-off for q15/q31. The workload does not change CPACR or the clock. Q callbacks
  retain **float for q15 / double for q31** and the production kernels retain their
  original local floating/extended intermediates; this is not a reduced integer
  surrogate workload.
- All test state, initial-state backups, input arrays and explicit workspaces are
  one reusable static region plus a union of the three state descriptors. There
  are no vectors, heap allocations, runtime RNG, dynamic initializers, or
  per-case copies of those buffers in the firmware build.

See the platform-owned build instructions for actual linker/clock/pin settings.
This document and `host_oracle.py` do not grant permission to access a board.

## Cases and exact input provenance

Each image runs 20 cases in this order:

1. Linear model, then nonlinear model.
2. For each model: `n=m=2`, then `n=m=4`.
3. For each dimension: UKF; PF N=32; UPF N=32; PF N=128; UPF N=128.

The family IDs and indivisible timed transactions are:

| Family | Transaction |
|---|---|
| 0 | UKF predict + update |
| 1 | PF predict + update + estimate |
| 2 | UPF step |

The model, scaled-transform parameters, covariance/noise and measurement values
match `../nonlinear.cc`:

```text
alpha=1, beta=2, kappa=0
x0[j] = 0.0625 + 0.015625*j
z[j]  = 0.09375 + 0.015625*j
P0 = diag(0.015625), Q = diag(0.00390625), R = diag(0.03125)
f[j] = 0.875*x[j] + 0.0625*x[(j+1)%n]
h[j] = 0.75*x[j] + 0.125*x[(j+1)%n]
nonlinear: add 0.03125*x[j]^2 to f, 0.0625*x[j]^2 to h
```

`draws.h` freezes 512 standard-normal draws generated offline by the original
libstdc++ `std::mt19937(0x51f15u)` + `std::normal_distribution<double>(0,1)` and
rounded with `round(draw*32768)/32768`. Hexadecimal literals represent the same
exact values in float and double; values beyond [-1,+1] are **not** clipped.
Each case uses the first `N*n` entries, exactly as the host's per-case generator
restart does. The sampler only copies a contiguous row of already typed values
and updates/wraps its cursor. The constexpr float/double table lives in object
read-only data (2,048 / 4,096 bytes), not the RAM workspace.

After successful PF/UPF init, particles are reconstructed on the same q15 grid as
the host: `round((x0[j] + 0.125*draw[i*n+j])*32768)/32768`. This avoids different
init rounding creating different starting ensembles. Initial weights and UPF
covariances are copied from the real init result without modification. Only the
harness's starting-state construction is rounded to that common grid; the
production algorithms and callback arithmetic are not shortened or rescaled.

## Timing and result validity

Each case performs **two warmups and nine retained trials**. Every transaction,
including the first warmup, starts from the initial x/P or complete particle /
weight / per-particle covariance backup and from draw cursor zero. PF output
arrays are cleared before a transaction. Initialization, restoration, checksum,
hash, finite checks and mailbox publication are outside the timed region.
Callbacks, sampling, all constituent API calls and their error checks are inside.

The exact bracketing order is:

```text
bench_marker(family*4 + BENCH_FORMAT, 1)
before = bench_cycles()
transaction()
after = bench_cycles()
bench_marker(family*4 + BENCH_FORMAT, 0)
```

The marker channel is 0..11. Warmups also emit markers: 11 pulses per case,
220 per successful run. GPIO pulse width includes the cycle-read bracketing
costs and is not identical to `after-before`. Cycle counts are unsigned 32-bit
counter differences; no overhead is subtracted. `cycle_read_overhead` records one
adjacent pair of empty reads before the sweep. A transaction must finish within
one counter period for the difference to be unambiguous; interruption handling,
clock verification and this duration bound are platform/runner responsibilities.

Every init and transaction API status is checked. On error the driver stops at
that case, publishes the exact `linears_status_t`, clears **all nine timings of
that case** (including earlier successful trials), clears its checksum/hash,
sets `phase=4`, and returns with the marker low. Harness validation uses
`LINEARS_ERROR_RANGE` for nonfinite outputs and
`LINEARS_ERROR_NO_CONVERGENCE` for a repeated transaction producing a different
raw hash or binary64 checksum. These two meanings are harness diagnostics, not
new production API status definitions.

Only a run with `phase=3` and `case_count=20` is complete performance data.
`case_count` counts published finished/failed cases; a failure record is included.
`active_case` is zero-based and identifies the running/failed case; it remains 19
on success. At each invocation the driver clears old case records and resets
these counters. It preserves the platform's `core_hz`, `cpacr`, command and fault
fields. The platform handles the command handshake outside `bench_run()`.

### Checksums, raw hash and complete outputs

The numerical checksum preserves the host benchmark definition and summation
order:

- UKF: `sum(x) + sum(P)`.
- PF: `sum(estimated_mean) + sum(estimated_covariance)`.
- UPF: `sum(particles) + sum(per_particle_covariances)`.

Each individual sum accumulates decoded signals in binary64. `checksum_lo` /
`checksum_hi` store the low/high 32 bits of that IEEE binary64 value, not a
rounded decimal or two floating numbers.

`output_hash` is FNV-1a-32 (offset 2166136261, multiplier 16777619), processing
little-endian raw element bytes in these array orders:

- UKF: x, P.
- PF: particles, **weights**, estimated mean, estimated covariance.
- UPF: particles, **weights**, per-particle covariances.

All modified persistent/output arrays are covered; descriptor padding and
unchanged Q/R are not. Every element is checked for finiteness and every one of
the 11 transactions must match the first transaction's hash and checksum. The
sum and 32-bit hash are compact diagnostics, not a proof of numerical accuracy.
Use the full-element host comparison below and the existing numerical tests.
Cross-toolchain/libm implementations can differ in last-bit floating results;
within-image repeatability is mandatory, but a cross-machine hash mismatch must
be investigated rather than silently interpreted as either success or failure.

## Memory accounting

Case records report **logical per-instance storage**, not the maximum physical
harness allocation:

- `persistent_bytes`: x/P/Q/R, or particles/weights/Q/R plus UPF covariances,
  using exactly float/double/int16_t/int32_t for the selected signal type.
- `descriptor_bytes`: the actual target ABI `sizeof` of that filter state.
- `workspace_bytes`: componentwise maximum of the public operation bounds,
  including init, typed signal staging, aligned basic scratch and decomposition.
- `output_bytes`: only PF's estimated mean/covariance; zero for UKF/UPF.

The physical harness also needs backups, input data, fixed maximum capacities
and the result mailbox. Isolated Cortex-M4 object compilation with GCC 14.2.1
confirmed these static object sizes; mailbox size is 1,728 bytes:

| Format | Workload BSS bytes | Workload + mailbox bytes | Space left in 96 KiB before platform/stack |
|---|---:|---:|---:|
| f32 | 39,408 | 41,136 | 57,168 |
| f64 | 74,616 | 76,344 | 21,960 |
| q15 | 18,392 | 20,120 | 78,184 |
| q31 | 36,440 | 38,168 | 60,136 |

This is not an image-wide peak-RAM measurement: final linker padding, other
platform/runtime data and the full call-chain/interrupt stack remain additional.
A compile-time assertion reserves at least 16 KiB beyond these objects; the
platform's real 96 KiB linker layout must still reject an oversized image.
Largest **driver-only single-function** `.su` frames were 384 bytes (f32/f64)
and 440 bytes (q15/q31); these are not transitive stack bounds. The 64-bit host
has larger descriptors and reports 41,224 / 76,432 / 20,208 / 38,256 bytes for the
same object-plus-mailbox sum.

For example, N=128, n=m=4 UPF logical target bytes are:

| Format | Persistent | Descriptor | Workspace | PF outputs | Sum |
|---|---:|---:|---:|---:|---:|
| f32 | 10,880 | 96 | 13,548 | 0 | 24,524 |
| f64 | 21,760 | 112 | 26,808 | 0 | 48,680 |
| q15 | 5,440 | 96 | 7,064 | 0 | 12,600 |
| q31 | 10,880 | 112 | 14,128 | 0 | 25,120 |

## Reproducible host validation

From the SDK root, without configuring or running Xmake:

```sh
python3 libs/filters/bench/n32h474/host_oracle.py --revision 3ee417a
python3 libs/filters/bench/n32h474/host_oracle.py --revision 3ee417a --sanitize
```

Without `--revision`, the script snapshots current production headers/sources;
with it, `git archive` supplies immutable production files. The current driver
is copied in both modes. `--cc` / `--cxx` select installed host compilers; the
frozen normal sequence must still match libstdc++. No downloads or hardware
access occur. Both normal and sanitizer builds use `-ffp-contract=off`.

Each invocation:

1. Checks all 512 table values against the original generator in float/double.
2. Builds/runs the same `workload.cc` with `HOST_BENCH=1` for all four formats.
   Each process invokes `bench_run()` twice and checks markers, complete coverage,
   repeated raw hashes and binary64 checksums. Stub timer values are **host
   nanoseconds, not MCU cycles**; the script does not report them as board data.
3. Saves every modified state/output element, including weights, as both raw
   bits and decoded values. An independent copy of the original host driver's
   inputs/RNG/init/reset/transaction exports the same arrays, using a replaced
   untimed main only. All 46,704 raw elements across 80 format/model/cases must
   match exactly. This validates driver equivalence, not an independently
   implemented UKF/PF/UPF mathematical reference.
4. Tests all nine public init/transaction failure-return points in each format,
   plus repeatability failures in all four formats and nonfinite output failures
   in both floating formats: 42 injected-failure checks. Runtime failures occur
   after two valid retained timings, proving the invalid case clears all timings,
   publishes the concrete status, stops the sweep and lowers the marker.
5. Writes per-case equivalence and Q-vs-floating elementwise maximum/RMS errors.
   Weights are sum-normalized before cross-format comparisons because the Q
   API's persistent weight convention is max-scaled. This report does not invent
   a new accuracy acceptance threshold.

The printed `/tmp/filters-xhive-board-oracle-*` directory retains:

- `host_{0,1,2,3}.csv`: mailbox-equivalent logical bytes, checksum bits and hash.
- `host_*_elements.csv`, `reference_*_elements.csv`: complete output elements.
- `equivalence.csv`: element counts, raw mismatches and absolute errors.
- `quantization.csv`: paired Q-vs-floating decoded errors per array.
- `metadata.json`: production revision, source mode/hashes, compiler versions,
  exact commands and observed validation results.
- Source copies, draw-check/failure adapters and executables for reproduction.

The initial GCC optimized and ASan/UBSan runs against `3ee417a` passed the complete
80-case / 46,704-element comparison. These checks are **host executions and
compile-only Cortex-M4 checks**, not real-board measurements.
