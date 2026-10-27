# Q estimator measurements

These tools compare the dedicated q15/q31 UKF, PF and UPF paths with the
unchanged f32/f64 paths. They are separate from functional Xmake tests and
neither download tools nor access a board. Generated objects and logs stay in
retained temporary directories outside the SDK.

## Reproduce

From the SDK root:

```sh
python3 libs/filters/bench/run.py --cc gcc --cxx g++
python3 libs/filters/bench/run.py --cc clang --cxx clang++ --nonlinear
python3 libs/filters/bench/compare.py /path/printed/by/run.py/results.csv
python3 libs/filters/bench/coverage.py /path/printed/by/a/gcc/run
python3 libs/filters/bench/cross_audit.py

# Previous implementation, with immutable production sources exported by Git:
python3 libs/filters/bench/run.py --revision 470206b --cc gcc --cxx g++
```

`run.py` snapshots the current production sources before compiling and records
source hashes, compiler commands, CPU information and the measurement method.
`--nonlinear` adds `0.03125*x_j*x_j` to the transition and `0.0625*x_j*x_j` to
the observation. It does not use expensive trigonometric callbacks that would
hide library costs. Both modes use the same dyadic input matrices and
precomputed normal draws across formats; q15 is paired with f32, q31 with f64.

The timed operations are UKF predict+update, PF predict+update+estimate, and
UPF step. Timing includes callbacks, validation and commit. Allocation,
initialization, restoration between transactions and random-number generation
are outside the timed interval. Each scenario uses 16 independent instances,
12 batches per sample, 2 warmup samples and 9 reported samples. CSV files
retain median/min/max, output checksums and memory counts. No absolute timing
threshold is imposed on CI.

## Recorded results

The `current-*` records use the dedicated Q implementation, with production
source hashes matching the validated working tree. The UKF kernel hash is
`556c4aad42e8cc03788f2cc72f2bf79527bc25133ab352da1d052ee9bf3d577b`.
Their `revision` field identifies the base commit, not a commit containing the
uncommitted implementation; `source_mode` is `working-tree-copy`.
`baseline-470206b-*` records describe the previous floating-wrapper version.

Host: AMD Ryzen 9 7945HX, Linux CPU 0 affinity. Compilers: GCC 15.2 and
Clang 21.1.8, `-O3 -fno-fast-math`, without LTO. Each compiler/model combination
has 40 records: four formats, n=m=2/4, and N=32/128 for PF/UPF. All 160
benchmark scenarios completed without API failures. Across all paired Q
scenarios, explicitly counted memory is 26.65%–61.05% of the paired floating
version, a reduction of 38.95%–73.35%.

| Model | Compiler | Floating median time / Q median time |
| --- | --- | --- |
| Linear | GCC | 2.039–3.743x |
| Linear | Clang | 2.654–4.616x |
| Nonlinear | GCC | 1.941–4.084x |
| Nonlinear | Clang | 2.616–4.591x |

Representative nonlinear GCC medians, n=m=4, N=128 for PF/UPF, in microseconds:

| Transaction | f32 | q15 | f64 | q31 |
| --- | ---: | ---: | ---: | ---: |
| UKF predict+update | 4.992 | 2.230 | 4.802 | 2.473 |
| PF predict+update+estimate | 39.840 | 16.599 | 39.274 | 16.945 |
| UPF step | 772.964 | 290.499 | 739.941 | 331.553 |

Explicit caller-memory counts for the same dimensions, in bytes:

| Family | f32 | q15 | f64 | q31 |
| --- | ---: | ---: | ---: | ---: |
| UKF | 2328 | 1168 | 4240 | 2208 |
| PF | 13360 | 3560 | 22496 | 6736 |
| UPF | 24604 | 12680 | 48760 | 25200 |

Memory includes one instance's persistent arrays, state descriptor, the maximum
reusable workspace arrays across the measured operations, and PF estimate
outputs. It excludes stack, inputs, callback/RNG state, allocator overhead and
the benchmark's multiple instances. This is not a hardware peak-RAM measurement.

After these measurements, the driver's workspace aggregate initialization was
rewritten as explicit zero-initialization and field assignments to remove
`-Wextra` missing-field warnings. That code is outside timing; production
sources and timed operations did not change. Recorded `driver_sha256` values
remain the hashes actually measured, rather than being relabeled. A subsequent
40-scenario nonlinear GCC run with the warning-clean driver reproduced every
recorded checksum and byte count; its timing was not substituted into the CSVs.

## Integer execution and operation-count evidence

`current-coverage-{linear,nonlinear}.json` records actual GCC `-O1 --coverage`
execution, not timing. The recorded nonlinear q15 cross-covariance took the
integer branch 5,448,960 times and the local floating branch 1,351,680 times
(19.88% floating). q31 cross-covariance took the integer branch 6,800,640 times;
both covariance kernels took it 8,840,832 times each. Counts include warmup
and initialization. Thus the benchmark exercises real integer computation and
also a non-ideal local floating fallback, not merely dormant Q code.

The speedup is not attributed solely to integer instructions:

- Q PF Gaussian likelihoods reuse a Cholesky factor and use triangular forward
  substitution. The floating composition performs a general pivoted LU solve
  per particle, including repeated factorization: that phase changes from
  repeated O(N*m^3) work to O(m^3+N*m^2).
- Q UKF covariance computes only the symmetric half; integer cross-covariance
  uses n paired sigma differences instead of a general 2n+1-term dot.
- Q UKF reuses the innovation Cholesky factor for gain solves, avoids a second
  LU factorization and removes transposed matrices and expanded weight arrays.
- Raw Q staging and integer moments remove whole-particle floating copies.

## Cross-compilation and limits

`current-arm.json` records successful Cortex-M4 soft-float and hard-float
(`fpv4-sp-d16`) compilation with arm-none-eabi-gcc 14.2.1. Q UKF/UPF call edges
point to the dedicated Q work APIs, not the complete f32/f64 UKF kernels;
integer multiply/accumulate instructions are present. Local floating Cholesky
and range/precision expansion remain intentional.

These are object-code checks, not MCU execution or cycle measurements.
Compiler `.su` reports are individual stack frames, not complete call-chain
peak stack. Host `perf` hardware-counter access was denied
(`perf_event_paranoid=4`); no permissions were changed and no retired-instruction
or CPU-cycle results are claimed. Scheduling and turbo variation remain in
host wall-time measurements. Q format alone does not guarantee a speedup on
every CPU or for arbitrary callback costs.
