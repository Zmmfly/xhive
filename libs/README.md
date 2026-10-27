# Libraries

**NOTE: Need update below table when library is added or updated**

| Library Name |                  Version                   |        License         |                                  URL                                  |
| :----------: | :----------------------------------------: | :--------------------: | :-------------------------------------------------------------------: |
|  micro-ecc   | `541b3a78026420a3e369c4c9281c396b5e531113` | `BSD-2-Clause license` |          [repository](https://github.com/kmackay/micro-ecc)           |
|    cJSON     |                  `1.7.19`                  |     `MIT License`      |           [repository](https://github.com/DaveGamble/cJSON)           |
|  heatshrink  | `e4084caa5c570c5e733de6b12ee1c9000c52d1b4` |     `ISC license `     | [repository](https://github.com/atomicobject/heatshrink/tree/develop) |
|    xxhash    |                  `0.8.3`                   | `BSD-2-Clause license` |           [repository](https://github.com/Cyan4973/xxHash)            |

## SDK numerical and control libraries

The following libraries are maintained in this SDK and follow its license;
there is no separately versioned upstream package:

| Library | Purpose | Application Kconfig selection |
| --- | --- | --- |
| [linears](linears/README.md) | Allocation-free matrix/vector arithmetic and decompositions | `CONFIG_LIBS_ADD_LINEARS=y` |
| [filters](filters/README.md) | Streaming filters, scalar/matrix Kalman, unscented Kalman and particle/unscented particle estimation | `CONFIG_LIBS_ADD_FILTERS=y` (selects linears) |
| [fieldctrl](fieldctrl/README.md) | Motor-control numerical primitives, VVVF, FOC/SMO, and DTC | `CONFIG_LIBS_ADD_FIELDCTRL=y` (selects linears and filters) |

Each library provides `f32`, `f64`, `q15`, and `q31` APIs. q15/q31 signals are raw
signed Q1.15/Q1.31 integers without exponent fields; working values and wide
coefficients use explicit `int64_t` fixed-F31 storage. Q computation is integer-only.
Matrices contain only `data`, `rows`, and `cols`, replacing the former BFP ABI.
State and dimension-sized workspace belong to the caller; see each public header
for capacity, alignment, aliasing, endpoint and range-error contracts. These are
software algorithms, not hardware motor drivers or substitutes for board-level
protection.

The SDK's default host build includes all three. Their per-file GoogleTest
entries share group `xhive`; Eigen 3.4.0 is a pinned host-test reference only:

```sh
xmake f -P . -y -m debug --xhive_tests=y --xhive_cross_tests=n
xmake test -g xhive -v 'linears_test_*/*' 'filters_test_*/*' 'fieldctrl_test_*/*'
```

Applications and library-only builds with `--xhive_tests=n` do not require Eigen
or GoogleTest. See the [implementation and validation worklog](../docs/fieldctrl-worklog.md)
for recorded software-test and cross-compilation evidence; those checks do not
establish physical motor operation.
