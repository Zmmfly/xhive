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

## Requires
- python >= 3.10
- xmake >= 3.0.0
- kconfiglib, Python pip package
- windows-curses, Python pip package (for Windows platform)

## Toolchain support

- [x] `arm-none-eabi-gcc`
- [x] `ATfE`, [repository](https://github.com/arm/arm-toolchain)
- [ ] `riscv32-unknown-elf-gcc` (RISC-V support is under development)

## License

This project is available under multiple licensing options:

- **GPLv3**: For open source projects
- **Commercial License**: For proprietary commercial use

### License Terms

1. **GPLv3 License**: This project is licensed under GPLv3. Any modifications to this project, whether for internal use or distribution with products, must be open-sourced under the GPLv3 terms. If you wish to use or integrate this project in commercial products without open-sourcing your modifications, please contact us for a commercial license.

2. **Scope**: This license does not cover referenced third-party libraries or vendor-provided driver code.

3. **Full License Text**: The complete GPLv3 license text can be found on the [GNU official website](https://www.gnu.org/licenses/gpl-3.0.html).

4. **Contributions**: This project accepts contributions under a CLA. All contributors must sign the CLA to ensure that code copyright and licensing comply with project requirements.
