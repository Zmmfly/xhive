# <div align="center">xHive</div>
<div align="center">
  
![GitHub stars](https://img.shields.io/github/stars/Zmmfly/xhive?style=social&logo=github) ![GitHub forks](https://img.shields.io/github/forks/Zmmfly/xhive?style=social&logo=github)

🚀 Modern embedded SDK for ARM/RISC-V MCUs with intelligent build system, xmake-based. 
</div>

English | [简体中文](README.zh_CN.md)

## Maintenance Notice

- This project is maintained by an individual developer during personal time. While usage and issue reporting are welcome, please understand that timely responses to all requests cannot be guaranteed.
- For commercial projects or applications requiring long-term support, commercial support options are recommended to ensure project stability and continuous maintenance.
- Currently, considering the early-stage maintenance costs of the project, external code contributions are not being accepted. If you have feature requests or improvement suggestions, please submit them via issues, and I will evaluate them for future integration.
- Due to limited personal availability, documentation and examples may be incomplete. Please understand that documentation updates and organization may take considerable time.

## Features

- **Multi-architecture MCU support**: ARM Cortex-M, RISC-V, and more
- **XMake-based build system integration**: Streamlined cross-platform compilation
- **Dual-namespace development**: Simultaneous PC-based unit testing for pure logic code and MCU firmware compilation using XMake namespaces
- **Automatic startup code generation**: Simplified project initialization
- **Automatic linker script generation**: Optimized memory management

## Requirements
- python >= 3.10
- xmake >= 3.0.0
- kconfiglib, with custom modifications, install by `pip install git+https://github.com/Zmmfly/kconfiglib.git@dev/rawstr`
- windows-curses, Python pip package (for Windows platform)

## SDK development and tests

The SDK root is a standalone **native host** project. It does not read a board
`.config`, run `genconfig`, load MCU/RTOS targets, or require an embedded toolchain.
The default build compiles the portable libraries selected by the SDK's host
aggregate, using their upstream defaults (without `BUILD_IN_XHIVE`). mbedTLS
remains an explicitly selected target, as in application builds.

```sh
cd /path/to/xhive
xmake f -P . -y -m debug --xhive_tests=y --xhive_cross_tests=n
xmake
xmake test -g xhive -v
```

The host test group contains the base64/bitmap/divsufsort/ezctrl GoogleTest cases,
the linears/filters/fieldctrl numerical and software-control tests, C2 MMIO mock
and C/C++ header/link checks, and the SDK entry/target-selection build-system
regressions. Test
targets are non-default: `xmake` builds libraries, while `xmake test` builds and
runs the tests. The C2 checks use Python 3 and GCC/Clang-compatible `cc` and `c++`
executables on `PATH`; the native test group does **not** invoke a RISC-V compiler.
GoogleTest and the Eigen 3.4.0 numerical reference are managed by Xmake and may
need downloading on the first configure. Eigen is a host-test dependency only;
production and embedded targets do not depend on it.
Kconfig tools from the application requirements above are not needed for these
host checks. Lua regression scripts also use Python 3 for temporary-directory
cleanup that does not traverse directory symlinks. Test workspaces must be
outside the SDK tree; cleanup refuses the SDK, its ancestors/descendants, and
linked workspace roots. This workflow is validated on Linux; other hosts need
compatible host compilers and have not been validated.

```sh
# Library test targets are <library>_test_<relative path>, the test case is the
# source basename, and all host cases share the xhive group.
xmake test -v 'base64_test_*/*'
xmake test -v 'divsufsort_test_*/*'
xmake test -v 'ezctrl_test_*/*'
xmake test -g xhive -v 'linears_test_*/*' 'filters_test_*/*' 'fieldctrl_test_*/*'
xmake test -v 'xhive_test_c2/host'
xmake test -v 'xhive_test_sdk_entry/sdk_entry'

# A libraries-only build does not require GoogleTest, Eigen, or Python.
xmake f -P . -y --xhive_tests=n --xhive_cross_tests=n
xmake
# Re-enable host tests.
xmake f -P . -y --xhive_tests=y --xhive_cross_tests=n
```

Libraries with host tests declare them in `libs/<library>/xmake.lua`, outside
its production target: each `test/**.cc` except the shared `test/main.cc` becomes
an independent non-default binary target with its own Xmake test case. The
`tests/` directory keeps only SDK build-system regressions (`sdk_entry`,
`target_selection`, `riscv_toolset`/`riscv_startup`) and is not the pattern for
new library tests. The C2 wrapper targets live in
`vendor/openecos/starrysky/c2/tests/xmake.lua` and keep the `xhive_test_c2/host`
and `xhive_test_c2_full/full` selectors.

### Standalone library tests (optional)

base64, bitmap, divsufsort, ezctrl, linears, filters, and fieldctrl each load their
own test definitions, so `-P` also works against a library directory. Only the
selected library's tests are registered; its production dependencies are loaded
without their tests. Use a subshell to isolate both the
working directory and the configuration cache (POSIX shell):

```sh
component="$PWD/libs/base64"   # or libs/bitmap, libs/divsufsort, libs/ezctrl
(cd "$component" && XMAKE_CONFIGDIR="$component" xmake f -P . -y -m debug && XMAKE_CONFIGDIR="$component" xmake test -P . -v)
```

Standalone mode takes host options only (`-y -m debug`); SDK-root-only options
such as `--xhive_tests` and `--xhive_cross_tests` do not apply there, and the
compile-only cross targets are registered only in the SDK-root project. Prefer
the SDK-root selectors above for day-to-day work.

### Numerical and motor-control libraries

[linears](libs/linears/README.md), [filters](libs/filters/README.md), and
[fieldctrl](libs/fieldctrl/README.md) provide C99 `f32`, `f64`, `q15`, and `q31`
APIs with caller-owned state/workspace and no dynamic allocation. Fixed-point
values carry explicit binary exponents and use integer arithmetic. Select them
in application Kconfig using `LIBS_ADD_LINEARS`, `LIBS_ADD_FILTERS`, or
`LIBS_ADD_FIELDCTRL`; filters selects linears, and fieldctrl selects both.
The SDK default host build includes all three.

These libraries do not access motor hardware or implement board-level protection.
The [worklog](docs/fieldctrl-worklog.md) distinguishes numerical tests, software
plant simulations, and ARM compile/link checks from hardware validation. A
[reproducible N32H474 compile/link fixture](libs/fieldctrl/test/compile/n32h474/README.md)
preserves all four formats' public algorithm symbols; copy it to a fresh external
application directory as documented there. It has no hardware-run instructions.
The independent fixed-point audit requires binary `long double` with at least
64 significand bits; the usual MSVC 53-bit `long double` does not meet that
reference-precision requirement. Linux results do not establish compatibility
with every host compiler.

### Optional cross-compiler checks

Cross checks are not registered by default. Enable them explicitly and supply a
RISC-V GCC executable supporting the tested RV32/RV64 multilib variants. Full C2
validation also needs Clang with a RISC-V backend.

```sh
xmake f -P . -y --xhive_tests=y --xhive_cross_tests=y \
    --xhive_riscv_gcc=/path/to/riscv-none-elf-gcc \
    --xhive_clang=clang
xmake test -g xhive_cross -v

# Return to the default host-only test set.
xmake f -P . -y --xhive_tests=y --xhive_cross_tests=n
```

`xhive_cross` covers RISC-V compiler flags, startup/linker templates (with and
without LTO), compile-only freestanding divsufsort object builds, and the full C2
software runner.
Enabling it makes an unfiltered
`xmake test` run both groups, and a selector such as `divsufsort_test_*/*` also
matches the cross object targets. Put options **before** selectors, e.g.
`xmake test -g xhive -v 'divsufsort_test_*/*'`, to keep host cases only.
Xmake 3.0.8 treats arguments after the first selector as additional selectors,
so trailing `-g`/`-v` options do not take effect.
Missing tools and subprocess failures are reported
as failures, not silently skipped or replaced with a host compiler. These tests
do not flash or access a board and are not hardware qualification.

The original script entry points remain available:

```sh
xmake l tests/sdk_entry.lua
xmake l tests/target_selection.lua
python3 vendor/openecos/starrysky/c2/tests/run_tests.py --host-only
python3 vendor/openecos/starrysky/c2/tests/run_tests.py --sanitize
```

The former `tests/divsufsort.lua` and `tests/riscv_divsufsort.lua` entry points no
longer exist: divsufsort host coverage runs through `divsufsort_test_*/*`, and its
freestanding cross check runs through the library's `xhive_cross` object targets.

### External applications and `-P`

Applications continue to use `includes(os.getenv("XHIVE_SDK_PATH"))` and the
`xhive.embed` rule. They retain their own Kconfig selection and both SDK
namespaces. Only `xhive_host` registers host tests; the embedded namespace never
registers duplicate GoogleTest binaries. A missing application `.config` is an
error with instructions to run `xmake menuconfig` in that application.

To operate an application while staying in the SDK directory, keep **both** the
project and configuration-cache roots on the application:

```sh
# POSIX shell example; set these per command, not globally for SDK development.
app=/absolute/path/to/application
XHIVE_SDK_PATH="$PWD" XMAKE_CONFIGDIR="$app" xmake -P "$app"
XHIVE_SDK_PATH="$PWD" XMAKE_CONFIGDIR="$app" xmake menuconfig -P "$app"
```

Xmake 3.0.8 can cache `-P` in the invoking directory's `.xmake` directory. Using
`XMAKE_CONFIGDIR` as above avoids redirecting later SDK commands to the
application or mixing its configuration with SDK tests. If an earlier bare
`-P` invocation already redirected the SDK cache, run `xmake f -P . -y` from the
SDK root to select it again. SDK-root mode deliberately has no firmware target;
use an application project rather than copying a board `.config` into the SDK.

## Toolchain support

- [x] `arm-none-eabi-gcc`
- [x] `ATfE`, [repository](https://github.com/arm/arm-toolchain)
- [ ] `riscv32-unknown-elf-gcc` (RISC-V support is under development)

## License

- **GPLv3**: For open source projects, commercial use is currently not permitted

### License Terms

1. **GPLv3 License**: This project is licensed under GPLv3. Any modifications to this project, whether for internal use or distribution with products, must be open-sourced under the GPLv3 terms. If you wish to use or integrate this project in commercial products without open-sourcing your modifications, please contact us for a commercial license.

2. **Scope**: This license does not cover referenced third-party libraries or vendor-provided driver code.

3. **Full License Text**: The complete GPLv3 license text can be found on the [GNU official website](https://www.gnu.org/licenses/gpl-3.0.html).

4. **Contributions**: This project accepts contributions under a CLA. All contributors must sign the CLA to ensure that code copyright and licensing comply with project requirements.
