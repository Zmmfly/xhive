# <div align="center">xHive</div>
<div align="center">
  
![GitHub stars](https://img.shields.io/github/stars/Zmmfly/xhive?style=social&logo=github) ![GitHub forks](https://img.shields.io/github/forks/Zmmfly/xhive?style=social&logo=github)

🚀 Modern embedded SDK for ARM/RISC-V MCUs with intelligent build system, xmake-based. 
</div>

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
xmake f -y -m debug
xmake
xmake test -g xhive -v
```

The host test group contains the base64/bitmap GoogleTest cases, C2 MMIO mock and
C/C++ header/link checks, and the SDK entry/target-selection regressions. Test
targets are non-default: `xmake` builds libraries, while `xmake test` builds and
runs the tests. The C2 checks use Python 3 and GCC/Clang-compatible `cc` and `c++`
executables on `PATH`; the native test group does **not** invoke a RISC-V compiler.
GoogleTest is managed by Xmake and may need downloading on the first configure.
Kconfig tools from the application requirements above are not needed for these
host checks. Lua regression scripts also use Python 3 for temporary-directory
cleanup that does not traverse directory symlinks. Test workspaces must be
outside the SDK tree; cleanup refuses the SDK, its ancestors/descendants, and
linked workspace roots. This workflow is validated on Linux; other hosts need
compatible host compilers and have not been validated.

```sh
# Existing component test names are preserved.
xmake test 'test_base64_decode/default' -v
xmake test 'xhive_test_c2/host' -v
xmake test 'xhive_test_sdk_entry/sdk_entry' -v

# A libraries-only build does not require GoogleTest or Python.
xmake f -y --xhive_tests=n
xmake
# Re-enable host tests.
xmake f -y --xhive_tests=y
```

### Optional cross-compiler checks

Cross checks are not registered by default. Enable them explicitly and supply a
RISC-V GCC executable supporting the tested RV32/RV64 multilib variants. Full C2
validation also needs Clang with a RISC-V backend.

```sh
xmake f -y --xhive_cross_tests=y \
    --xhive_riscv_gcc=/path/to/riscv-none-elf-gcc \
    --xhive_clang=clang
xmake test -g xhive_cross -v

# Return to the default host-only test set.
xmake f -y --xhive_cross_tests=n
```

`xhive_cross` covers RISC-V compiler flags, startup/linker templates (with and
without LTO), and the full C2 software runner. Enabling it makes an unfiltered
`xmake test` run both groups. Missing tools and subprocess failures are reported
as failures, not silently skipped or replaced with a host compiler. These tests
do not flash or access a board and are not hardware qualification.

The original script entry points remain available:

```sh
xmake l tests/sdk_entry.lua
xmake l tests/target_selection.lua
python3 vendor/openecos/starrysky/c2/tests/run_tests.py --host-only
python3 vendor/openecos/starrysky/c2/tests/run_tests.py --sanitize
```

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
