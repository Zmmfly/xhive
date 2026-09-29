# xHive

[English](README.md) | 简体中文

基于 Xmake 的现代嵌入式 SDK，面向 ARM、RISC-V 等 MCU，提供主机测试、固件构建、启动代码及链接脚本生成能力。

## SDK 独立构建与测试：快速开始

在 **xhive 仓库根目录**执行：

```sh
cd /path/to/xhive

# 使用主机 Debug 配置，启用主机测试，关闭可选交叉检查。
xmake f -P . -y -m debug --xhive_tests=y --xhive_cross_tests=n

# 构建默认的可移植库。
xmake

# 构建并运行主机测试。
xmake test -g xhive -v
```

这套流程不需要具体板卡工程，不读取板卡 `.config`，也不需要先运行 `menuconfig`、安装 Kconfig 工具或安装嵌入式交叉编译器。

注意区分：

- `xmake`：构建默认库目标，**不会自动构建或运行测试**。
- `xmake test -g xhive -v`：构建并运行 `xhive` 主机测试组。
- SDK 独立模式不生成某个板卡的固件；构建固件仍需使用应用工程。
- 首次配置可能通过 Xmake 下载 GoogleTest，需保证依赖可获取。

当前独立开发流程已在 Linux 上验证。其他主机平台需要兼容的编译器，尚未完成同等验证；不能假定默认 MSVC 环境可直接运行使用 GCC/Clang 参数的 C2 测试脚本。

## 维护说明

- 本项目由个人开发者利用业余时间维护，欢迎使用和反馈问题，但无法保证及时响应所有请求。
- 商业项目或需要长期支持的应用，建议联系维护者了解商业支持方案。
- 目前考虑到项目早期的维护成本，暂不接收外部代码贡献。功能需求和改进建议请通过 Issue 提交，由维护者评估。
- 文档与示例可能尚不完整，补充和整理需要时间。

## 主要功能

- **多架构 MCU 支持**：包括 ARM Cortex-M、RISC-V 等。
- **Xmake 构建集成**：统一管理配置、编译及测试入口。
- **主机与嵌入式双命名空间**：在应用工程中通过 `xhive_host` 和 `xhive_embed` 隔离主机代码与固件代码。
- **SDK 独立开发模式**：无需板卡配置即可构建可移植库并运行软件回归测试。
- **启动代码及链接脚本生成**：减少应用工程的重复配置工作。

## 环境要求

### 仅构建 SDK 主机库

- Xmake ≥ 3.0.0。
- 可用的主机编译工具链。
- 关闭测试后，不要求 GoogleTest、Python 或嵌入式工具链。

### 运行主机测试

在上述环境基础上，还需要：

- Python ≥ 3.10，用于 C2 软件检查和 Lua 回归脚本的安全临时目录清理。
- `PATH` 中提供 GCC/Clang 兼容的 `cc`、`c++` 命令，供 C2 测试脚本使用。
- GoogleTest，由 Xmake 的包管理器管理。

主机测试组不会调用 RISC-V 编译器。它不依赖应用工程使用的 `genconfig` 或 `menuconfig`。

### 构建嵌入式应用

应用工程另需安装相应嵌入式工具链和项目使用的定制 Kconfig 库：

```sh
python3 -m pip install 'git+https://github.com/Zmmfly/kconfiglib.git@dev/rawstr'
```

Windows 平台使用相关终端配置界面时，还需要 Python 包 `windows-curses`。这些应用配置依赖不是 SDK 主机测试的前置条件。

## 主机测试的操作方式

### 运行全部主机测试

```sh
xmake test -g xhive -v
```

当前主机测试组包含：

| 内容 | 测试入口 |
| --- | --- |
| base64 解码 | `base64_test_base64_decode/base64_decode` |
| base64 编码 | `base64_test_base64_encode/base64_encode` |
| base64 输入检查 | `base64_test_is_base64/is_base64` |
| bitmap | `bitmap_test_bitmap/bitmap` |
| divsufsort 两套索引 API、排序/BWT/搜索 | `divsufsort_test_divsufsort/divsufsort` |
| divsufsort 32 位索引 C/C++ 消费者构建 | `divsufsort_test_consumer32/consumer` |
| divsufsort 64 位索引 C/C++ 消费者构建 | `divsufsort_test_consumer64/consumer` |
| ezctrl PID 初始化、参数校验与状态管理 | `ezctrl_test_pid_config/pid_config` |
| ezctrl PID 计算、限幅与防饱和 | `ezctrl_test_pid_update/pid_update` |
| ezctrl C++ 包装层 | `ezctrl_test_pid_cpp/pid_cpp` |
| ezctrl C/C++ 头文件及链接检查 | `ezctrl_test_consumer/consumer` |
| C2 MMIO mock、C/C++ 头文件及链接检查 | `xhive_test_c2/host` |
| SDK 入口、配置隔离、错误传播与清理检查 | `xhive_test_sdk_entry/sdk_entry` |
| 可选 MCU/RTOS 目标选择 | `xhive_test_target_selection/target_selection` |

例如，只运行某一项：

```sh
xmake test -v 'base64_test_base64_decode/base64_decode'
xmake test -v 'xhive_test_c2/host'
xmake test -v 'xhive_test_sdk_entry/sdk_entry'
```

测试目标均为非默认目标，不会因为执行普通 `xmake` 而自动运行。自动发现的库测试目标名为 `<库名>_test_<相对 test/ 的路径，去扩展名并将分隔符转下划线>`，用例名是源文件 basename，因此也可用通配符一次运行一个库的全部用例（`base64_test_*/*`、`bitmap_test_*/*`、`divsufsort_test_*/*`、`ezctrl_test_*/*`）。测试定义在各库 `libs/<库>/xmake.lua` 的生产目标之外；`tests/` 目录只保留 SDK 构建系统回归脚本（`sdk_entry`、`target_selection`、`riscv_toolset`/`riscv_startup`），不是新增库测试的模板。C2 的包装目标声明在 `vendor/openecos/starrysky/c2/tests/xmake.lua`，选择器仍为 `xhive_test_c2/host` 与 `xhive_test_c2_full/full`。

### 只用某个库自己的测试入口（可选）

base64、bitmap、divsufsort、ezctrl 都在自身上下文加载测试定义，因此也可用 `-P` 直接针对库目录测试。用子 shell 同时隔离当前目录和配置缓存（POSIX shell）：

```sh
component="$PWD/libs/base64"   # 可换成 libs/bitmap、libs/divsufsort、libs/ezctrl
(cd "$component" && XMAKE_CONFIGDIR="$component" xmake f -P . -y -m debug && XMAKE_CONFIGDIR="$component" xmake test -P . -v)
```

独立入口只传主机配置参数（`-y -m debug`），不接受 `--xhive_tests`、`--xhive_cross_tests` 等 SDK 根工程选项；交叉对象目标也只在 SDK 根工程下注册。日常开发优先使用上面的 SDK 根选择器方式。

### 只构建库，不启用测试

```sh
xmake f -P . -y -m debug --xhive_tests=n --xhive_cross_tests=n
xmake
```

重新启用主机测试：

```sh
xmake f -P . -y -m debug --xhive_tests=y --xhive_cross_tests=n
xmake test -g xhive -v
```

独立模式下，可移植库使用其上游主机默认配置，不定义 `BUILD_IN_XHIVE`，也不生成板卡配置头文件。mbedTLS 与应用构建时一样，不属于默认聚合选择，需要显式选择目标。

### 直接运行原有脚本

原有入口仍然保留：

```sh
# SDK 构建逻辑回归。
xmake l tests/sdk_entry.lua
xmake l tests/target_selection.lua

# C2 主机软件检查，不要求 RISC-V 编译器。
python3 vendor/openecos/starrysky/c2/tests/run_tests.py --host-only

# C2 主机软件检查，并启用 ASan/UBSan。
python3 vendor/openecos/starrysky/c2/tests/run_tests.py --host-only --sanitize

# C2 完整软件检查，额外要求带 RISC-V 后端的 Clang。
python3 vendor/openecos/starrysky/c2/tests/run_tests.py
```

C2 脚本还支持 `--cc`、`--cxx`、`--clang` 参数，可显式选择对应编译器。

`tests/divsufsort.lua` 与 `tests/riscv_divsufsort.lua` 已删除：divsufsort 的主机覆盖由 `divsufsort_test_*/*` 提供，裸机编译检查由该库在 `xhive_cross` 组注册的对象目标提供。

## 可选的 RISC-V 交叉检查

交叉检查默认关闭，与主机测试分组独立。启用前需准备：

- 支持测试所需 RV32/RV64 多库组合的 RISC-V GCC。
- 带有 RISC-V 后端的 Clang，用于 C2 完整软件检查。
- 主机测试所需的 Python 与主机编译器。

```sh
xmake f -P . -y -m debug --xhive_tests=y --xhive_cross_tests=y \
    --xhive_riscv_gcc=/path/to/bin/riscv-none-elf-gcc \
    --xhive_clang=clang

xmake test -g xhive_cross -v
```

`xhive_cross` 包含：

- RISC-V 编译参数、ABI 及非法配置拒绝检查。
- 启动代码与链接脚本检查，包括启用和关闭 LTO 的情况。
- divsufsort 在 RV32/RV64 裸机编译模式下的 32/64 位索引构建检查（只编译，不在主机上执行）。
- C2 完整软件检查，包括 RV32 编译及寄存器位域布局验证。

恢复为默认的主机测试配置：

```sh
xmake f -P . -y -m debug --xhive_tests=y --xhive_cross_tests=n
```

**启用交叉检查后，不带分组参数的 `xmake test` 会运行两个组中已注册的测试。** 只想运行主机测试时，继续使用 `xmake test -g xhive -v`；同理，`divsufsort_test_*/*` 之类的通配符也会匹配 divsufsort 的交叉对象目标，可用 `xmake test -g xhive -v 'divsufsort_test_*/*'` 只保留该库主机用例。

**`-g`、`-v` 等选项必须放在选择器前。** Xmake 3.0.8 会将首个选择器后的参数当作更多选择器，后置 `-g xhive` 并不能限制测试分组。

缺少工具、编译失败或子进程失败都会报告为失败，不会静默跳过，也不会用主机 GCC 替代指定的 RISC-V GCC。

上述检查不烧录、不访问实板，也不能代替外设时序、电气特性或板级功能验证。

## 常用配置选项

以下选项使用 `xmake f --选项=值` 设置：

| 选项 | 默认值 | 用途 |
| --- | --- | --- |
| `xhive_tests` | `y` | 启用主机测试；在支持的平台上引入 GoogleTest |
| `xhive_cross_tests` | `n` | 在 SDK 独立模式下注册可选交叉检查 |
| `xhive_riscv_gcc` | `riscv-none-elf-gcc` | 指定交叉检查使用的 RISC-V GCC 可执行文件 |
| `xhive_clang` | `clang` | 指定 C2 完整检查使用的 Clang 可执行文件 |

主机测试和交叉检查可以分别开启或关闭。修改选项后，以所需的完整配置重新执行 `xmake f`，可避免继续使用先前保存的测试选择。

## 在 SDK 目录操作外部应用

已有应用仍采用原来的接入方式：

```lua
includes(os.getenv("XHIVE_SDK_PATH"))

-- 在应用固件目标中添加：
-- add_rules("xhive.embed")
```

应用保留自己的 `.config` 和固件目标。SDK 的主机测试只注册在 `xhive_host` 中，不会在 `xhive_embed` 下重复注册 GoogleTest 可执行文件。

若希望停留在 SDK 根目录构建外部应用，应同时指定工程目录和配置缓存目录。以下为 POSIX shell 示例：

```sh
cd /path/to/xhive
sdk="$PWD"
app=/absolute/path/to/application

# 应用已有有效 .config 时，可跳过 menuconfig。
XHIVE_SDK_PATH="$sdk" XMAKE_CONFIGDIR="$app" xmake menuconfig -P "$app"

# 只构建固件时，可关闭 SDK 主机测试及其 GoogleTest 依赖。
XHIVE_SDK_PATH="$sdk" XMAKE_CONFIGDIR="$app" xmake f -P "$app" -y --xhive_tests=n
XHIVE_SDK_PATH="$sdk" XMAKE_CONFIGDIR="$app" xmake -P "$app"
```

这些环境变量应按命令设置，不要为了操作一个应用而全局修改 SDK 开发环境。

### 为什么不只写 `-P`？

Xmake 3.0.8 可能将 `-P` 指向的工程保存在**调用目录**的 `.xmake` 缓存中。之后在 SDK 目录运行命令，可能继续指向先前的应用工程。

使用上面的 `XMAKE_CONFIGDIR="$app"` 可以让应用使用自己的缓存，避免与 SDK 的主机测试配置相互影响。

如果之前只使用 `-P`，导致 SDK 缓存已经指向其他工程，可以回到 SDK 根目录重新选择当前工程：

```sh
cd /path/to/xhive
xmake f -P . -y -m debug --xhive_tests=y --xhive_cross_tests=n
```

不要将板卡 `.config` 复制到 SDK 根目录来替代应用工程。SDK 独立模式本身没有板卡固件目标。

## 构建产物与临时目录

- 默认构建产物位于 SDK 的 `build/` 目录。
- Xmake 配置缓存通常位于 `.xmake/`；外部应用操作应按上文显式隔离。
- 回归脚本创建自己的临时工作目录，必须位于 SDK 源码树之外。
- Lua 回归脚本通过 Python 清理临时目录，不遍历目录符号链接指向的内容。
- 清理会拒绝 SDK 自身、SDK 的祖先或后代目录，以及符号链接形式的工作目录根节点。

## 工具链支持说明

- `arm-none-eabi-gcc`：已集成。
- [Arm Toolchain for Embedded（ATfE）](https://github.com/arm/arm-toolchain)：已集成。
- RISC-V：当前交叉检查使用 `riscv-none-elf-gcc` 和 Clang 的 RISC-V 后端。项目中的相关启动代码、链接和编译参数用例已验证，但整体支持仍在完善，不代表所有 RISC-V 芯片和工具链组合均已验证。

## 许可证与使用约定

本仓库提供 [GPLv3 许可证文本](LICENSE)，也可查阅 [GNU 官方完整文本](https://www.gnu.org/licenses/gpl-3.0.html)。引用的第三方库和芯片厂商驱动不统一适用本项目的许可证，应分别核对各自许可。

维护者在[英文 README](README.md#license) 中还声明了商业使用限制、商业授权以及 CLA 相关要求。涉及商业使用、授权或贡献时，请一并阅读完整许可证及英文声明，并向维护者确认适用约定；本中文说明不额外增加许可条款。
