# xhive 应用工程入门指南（从零搭建）

本文面向第一次使用 xhive 的开发者，从零搭出一个**独立的嵌入式应用工程**，并得到固件构建产物（`.elf` / `.bin`）。

主线只使用一个具体示例：**Nation N32H474，裸机（无 RTOS），ARM GCC 工具链**。其他芯片、RTOS、USB 等在文末"更进一步"一节简述。文中的最小工程、接入 SDK 库（base64）与厂商头文件三条路径已在 Linux + Xmake 3.0.8 上完成编译与链接验证（生成 ELF/BIN、关键符号齐全、无未定义符号）；menuconfig 交互流程、首次工具链下载、Windows 命令与实板运行未做同等验证。

SDK 自身的主机测试（软件回归）见文末"SDK 主机测试"一节，与应用固件构建是两回事。

## 核心概念

开始前先建立三个概念，后面所有步骤都围绕它们：

1. **SDK 根目录是独立的主机工程**。`/path/to/xhive` 自己就是完整的 Xmake 工程，用于构建可移植库和跑主机测试。它**没有**某个板卡的固件目标，也不要把板卡 `.config` 复制进去。
2. **应用工程是另一个目录**。你的固件代码、`xmake.lua`、`.config` 都放在应用目录里，通过环境变量 `XHIVE_SDK_PATH` 引用 SDK。
3. **Kconfig 的 `.config` 管理工程的主要配置**：MCU 型号、CPU 核心、时钟来源、内存布局、编译器、启动文件和链接脚本是否使用 SDK 默认值。构建时会据此生成 `build/xhive_config.h`，并自动挑选厂商驱动源码。具体外设参数仍需在应用代码或厂商配置中设置。

引入 SDK 后，工程里有两套命名空间：`xhive_host`（主机上的库）和 `xhive_embed`（固件代码）。应用固件目标放在自己的命名空间（示例用 `app`）并挂 `xhive.embed` 规则即可，不需要关心其余细节。

## 第 0 步：准备环境

需要安装：

- Xmake ≥ 3.0.0（本文命令按 3.0.8 验证）。
- Python ≥ 3.10 与 pip，以及用于获取源码的 Git。
- 定制的 kconfiglib 分支（`menuconfig` / `genconfig` 工具，SDK 的 Kconfig 文件依赖该分支）。应用构建**必须**安装它，即使不打算用菜单界面：

  ```sh
  python3 -m pip install 'git+https://github.com/Zmmfly/kconfiglib.git@dev/rawstr'
  ```

- Windows 上若使用 Python Launcher，把上述 `python3` 换成 `py -3`；使用终端菜单界面还需 `py -3 -m pip install windows-curses`。

安装后确保当前终端能找到 `menuconfig` 和 `genconfig` 命令。若系统 Python 不允许直接安装包，可使用虚拟环境，或参考文末的 pipx 安装方式。

获取 SDK（已经克隆过的读者直接用自己的路径）。POSIX shell：

```sh
mkdir -p ~/prjs && cd ~/prjs
git clone https://github.com/Zmmfly/xhive.git
```

Windows PowerShell：

```powershell
New-Item -ItemType Directory -Force "$HOME\prjs" | Out-Null
Set-Location "$HOME\prjs"
git clone https://github.com/Zmmfly/xhive.git
```

最后准备两个环境变量。约定一个简单做法：**为应用工程开一个专用终端**，变量只在这个终端里设置，应用相关步骤做完就关闭它或按下文恢复；到"SDK 主机测试"一节另开一个不带这些变量的终端。**它们的写法在 Linux/macOS（POSIX shell）和 Windows PowerShell 中不同**。

POSIX shell（专用终端中）：

```sh
mkdir -p ~/prjs/my_app     # 应用目录，本文统一在此目录执行命令
cd ~/prjs/my_app
export XHIVE_SDK_PATH="$HOME/prjs/xhive"   # SDK 根目录的绝对路径，按实际位置修改
export XMAKE_CONFIGDIR="$PWD"             # 把 Xmake 缓存也指向应用目录
```

Windows PowerShell（专用窗口中）：

```powershell
New-Item -ItemType Directory -Force "$HOME\prjs\my_app" | Out-Null
Set-Location "$HOME\prjs\my_app"
$env:XHIVE_SDK_PATH = "$HOME\prjs\xhive"   # 按实际 SDK 位置修改
$env:XMAKE_CONFIGDIR = "$PWD"
```

用完恢复（也可直接关闭该终端/窗口）：

```sh
unset XHIVE_SDK_PATH XMAKE_CONFIGDIR
```

```powershell
Remove-Item Env:\XHIVE_SDK_PATH, Env:\XMAKE_CONFIGDIR
```

几点说明：

- `XHIVE_SDK_PATH` 供应用的 `xmake.lua` 找到 SDK；设置错误时 `includes()` 的报错不容易看懂，所以动手前先确认：POSIX 执行 `echo "$XHIVE_SDK_PATH"`，PowerShell 执行 `$env:XHIVE_SDK_PATH`。
- `XMAKE_CONFIGDIR` 用于隔离 Xmake 的工程缓存。Xmake 3.0.8 的 `-P` 可能把工程选择写进**调用目录**的 `.xmake` 缓存，之后 SDK 目录里的命令可能被"带偏"到别的工程。按上面方式为每个应用单独设置，不要全局、永久地修改这些变量。
- 若不想占用整个终端，POSIX 也可以用子 shell 包住单条命令：`( export XHIVE_SDK_PATH=... XMAKE_CONFIGDIR=...; xmake f -P . -y --xhive_tests=n )`，变量只在该子 shell 内生效。
- `VAR=value command` 这种"命令前缀"写法是 Bash 语法，PowerShell 中不可用。

可选变量：`XHIVE_TOOLS_PATH` 指定交叉工具链安装位置（默认 `~/.xhive/tools`）。

## 第 1 步：创建应用工程骨架

本文沿用第 0 步创建的 `~/prjs/my_app`（换其他位置时保持环境变量同步即可），最终结构如下：

```text
my_app/
├── xmake.lua      # 应用工程描述
├── .config        # Kconfig 配置（第 2 步创建）
└── src/
    └── main.c     # 应用代码
```

`xmake.lua` 内容：

```lua
includes(os.getenv("XHIVE_SDK_PATH"))

namespace("app", function()
    target("minimal")
        set_kind("binary")
        add_rules("xhive.embed")
        add_files("src/*.c")
    target_end()
end)
```

逐行解释：

- `includes(os.getenv("XHIVE_SDK_PATH"))`：把整个 SDK 加载进本工程。SDK 会自动注册主机命名空间 `xhive_host` 和固件命名空间 `xhive_embed`。注意 Xmake 3.0.8 的顶层描述域没有普通的 Lua `assert`，不要用它来检查变量；按第 0 步先用 `echo` 确认即可。
- `namespace("app", ...)`：应用自己的目标命名空间，避免与 SDK 目标重名。
- `add_rules("xhive.embed")`：关键一行。该规则会自动为这个目标设置交叉平台与 `xhive_toolchain` 工具链、把输出扩展名设为 `.elf`、生成配置头、挑选链接脚本，并自动注入 `xhive_embed::deps` 聚合依赖（包含库、厂商驱动、第三方组件）。**不要**在文件里写 `set_plat("cross")` 之类的全局设置，那会影响 SDK 的主机目标。
- 应用目录不需要和 SDK 是特定相对位置或刻意避开相同前缀；SDK 按目录边界判断目标是否在 SDK 外部。

## 第 2 步：写最小 `.config`

`.config` 放在应用根目录。这里先走**手写快速路径**（内容少、可控、便于理解每一项），交互式 `menuconfig` 见"用 menuconfig 修改配置"一节。

创建 `.config`：

```text
# MCU 厂商与型号：Nation N32H474
CONFIG_VENDOR_USE_NATION=y
CONFIG_NATION_USE_N32H47X_48X=y
CONFIG_NATION_N32H474=y

# CPU：Cortex-M4 + 单精度硬件 FPU（决定 -mcpu / -mfpu / -mfloat-abi）
CONFIG_CPU_ARM=y
CONFIG_CORE_ARM_CORTEX_M4=y
CONFIG_ARCH_ARM_FPU=y
CONFIG_ARCH_ARM_FPU_SINGLE=y

# 时钟：内部 HSI 经 PLL 倍频（见下文"时钟到底会怎么配"）
CONFIG_CLOCK_SYSCLK_PLL=y
CONFIG_CLOCK_PLL_ENABLE=y
CONFIG_CLOCK_PLL_SRC_HSI=y
CONFIG_CLOCK_HSI_ENABLE=y

# 工具链与启动/链接方式
CONFIG_COMPILER_ARM_GCC=y
CONFIG_NO_STD_STARTFILE=y
CONFIG_USE_DEFAULT_STARTUP=y
CONFIG_USE_DEFAULT_LD_SCRIPT=y

# 裸机，不使用 RTOS
CONFIG_THIRD_RTOS_NONE=y

# 显式关闭默认开启、但最小示例不需要的组件
# CONFIG_NATION_N32H47X_48X_USE_USBFS is not set
# CONFIG_THIRD_SEGGER_RTT is not set

# 内存布局：请按你手上型号的数据手册核对，不要照抄
CONFIG_FLASH_START=0x08000000
CONFIG_FLASH_LENGTH=256
CONFIG_RAM_START=0x20000000
CONFIG_RAM_LENGTH=96
```

几点说明：

- 关于省略选项：`build/xhive_config.h` 由 genconfig 按 Kconfig 全量生成，未写的项在**头文件**里取默认值（例如 `CONFIG_FLASH_START` 默认 `0x08000000`、`CONFIG_STACK_LENGTH` 默认 `0x400`，需要更大栈时再添加）。但 SDK 的 Xmake 脚本解析 `.config` 时只认显式写出的行，不会自动补默认，因此影响构建行为的项（时钟、启动、链接、工具链、USB 开关等）建议像上面一样显式写出，不要随手省略。
- `CONFIG_NATION_N32H47X_48X_USE_USBFS` 默认是 `y`（`USE_USBHS` 默认 `n`）；启用任一种 USB 驱动都要求额外提供 USB 板级配置目录（见"更进一步"），最小示例必须像上面那样把 USBFS 显式写成"not set"。
- `CONFIG_NO_STD_STARTFILE=y` 表示链接时加 `-nostartfiles`，启动代码完全由 SDK 生成的启动文件接管，这正是第 3 步需要 `_init` / `_fini` 的原因。
- Flash 256 KiB / RAM 96 KiB 只是本文示例取值；同系列不同封装容量可能不同，务必按数据手册调整。

首次构建时 SDK 会生成 `build/Kconfig`（配置入口，自动 source SDK 与应用的 Kconfig）和 `build/xhive_config.h`（C 宏形式的配置，编译时自动加入 include 路径，代码里 `#include <xhive_config.h>` 即可使用）。

### 时钟到底会怎么配

这是容易误解的地方，按当前源码如实说明：

- 选中 `CLOCK_SYSCLK_PLL` + `CLOCK_PLL_SRC_HSI` 时，SDK 只为厂商源码定义 `SYSCLK_SRC=SYSCLK_USE_HSI_PLL`，**不**定义 `HSI_VALUE`；此时 `HSI_VALUE` 用器件头文件里的默认值（8 MHz），并不读取 `CONFIG_CLOCK_HSI_FREQ`。
- 厂商的 `system_n32h47x_48x.c` 按 `SYSCLK_FREQ` 与 `HSI_VALUE` 自动推算 PLL 参数。默认主频随固件库版本变化：本次验证使用的 1.2.0 对 N32H474 默认 240 MHz（240/8=30 倍），仓库较早版本使用 200 MHz；请检查自己版本的源文件，不要把示例频率当作固定值。Nation 方案下 `CLOCK_PLLM/PLLN/PLLP` 等 Kconfig 选项被禁用，倍频参数不通过 Kconfig 设置。
- 只选 `CLOCK_SYSCLK_HSI` 时，SDK 定义的只有 `HSI_VALUE=<CONFIG_CLOCK_HSI_FREQ>`，同样**不会**定义 `SYSCLK_SRC=SYSCLK_USE_HSI`；厂商文件在未指定 `SYSCLK_SRC` 时仍走 HSI→PLL 默认路径，而且这个额外定义会与器件头文件中的 `HSI_VALUE` 重复。不要假设"选了 HSI 就运行在 8 MHz"。
- 仓库更新可能带来厂商文件变化，以 `vendor/nation/n32h47x_48x/firmware/CMSIS/device/system_n32h47x_48x.c` 为准。

## 第 3 步：写 `src/main.c`

```c
int main(void)
{
    for (;;) {
        /* 在这里写你的应用逻辑 */
    }
}

/*
 * C 运行库钩子，应用侧提供，SDK 不会自动生成：
 * 默认启动代码在进入 main 前后调用 newlib 的 __libc_init_array /
 * __libc_fini_array，它们引用 _init 和 _fini。这两个符号平时由标准
 * 启动文件 crti.o 提供；本工程 CONFIG_NO_STD_STARTFILE=y（-nostartfiles），
 * crti.o 不参与链接，所以需要应用自己给出空实现。
 * 仅在确无其他初始化钩子需求时这样写；不要额外重复定义其他 C 运行库
 * 启动符号。
 */
void _init(void) {}
void _fini(void) {}
```

如果省略这两个空函数，链接会因缺少 `_init` / `_fini` 失败。这不是 SDK 的自动行为，而是当前"默认启动 + `-nostartfiles` + nano.specs"组合下的应用侧要求（见"排错"第 5 条）。

## 第 4 步：安装交叉工具链

在**应用目录**执行：

```sh
xmake xhive_install -P .
```

该命令读取 `.config` 里的编译器选择（这里是 `arm-none-eabi-gcc`），按 `compiler/toolchains.json` 记录的官方地址下载、校验 SHA256 并安装到 `XHIVE_TOOLS_PATH`（默认 `~/.xhive/tools`）。已安装时会提示 already installed。工具链缺失时，配置/构建会直接报 `Please run 'xmake xhive_install' ...`。

注意顺序：这一步**需要 `.config` 已经存在**，所以放在第 2 步之后。

## 第 5 步：配置并构建

仍在应用目录（环境变量已按第 0 步设置）：

```sh
xmake f -P . -y --xhive_tests=n    # 配置工程；--xhive_tests=n 跳过 SDK 主机测试与 GoogleTest
xmake b -P .                       # 构建
```

如何确认成功：

- 链接完成后，SDK 规则会先打印三行，类似：

  ```text
  Generated binary file: /path/to/my_app/build/.../minimal.bin
  ROM used: ...
  RAM used: ...
  ```

  最后 Xmake 汇总显示 `build ok`。

- 按打印的路径找到 `minimal.elf` 与 `minimal.bin`（两者同目录，`.bin` 由 `objcopy` 从 ELF 转出）。
- `build/` 下还会有 `xhive_config.h`、`build/<target>_linkscript/link.ld`（由 `templates/link.ld` 预处理生成）和 `build/n32h474_startup.c`（由 `templates/startup_arm.c` 结合 N32H474 的中断表 JSON 生成）。

常用变体：

- 调试模式：配置时加 `-m debug`。注意 `-m` 是 Xmake 的模式开关；固件优化级别由 `.config` 中显式选择的 `OPTIMIZE_*` 选项决定（Kconfig 菜单默认选择 `OPTIMIZE_SMALLEST`，手写配置省略时不能假定 Xmake 已应用此值）。想完全不优化，应改选 `CONFIG_OPTIMIZE_NONE=y`，不要同时保留其他优化选项；调试符号则由 SDK 规则固定开启。
- 看完整编译/链接命令：`xmake b -P . -v app::minimal`。
- `--xhive_cross_tests` 是 **SDK 根工程专用**选项（只在 SDK 独立模式注册），不要用在应用工程。
- 修改 `.config` 后重新执行 `xmake f` 再构建，配置头会重新生成。

## 第 6 步：检查产物（建议）

Xmake 显示 build ok 只说明链接器返回了 0。GNU ld 在缺少入口符号等情况下可能只警告仍返回成功，所以交付固件前建议再做几项静态检查。`arm-none-eabi-nm` / `readelf` 不要求在 PATH 里，它们就在工具链安装目录的 `bin` 下（例如 `~/.xhive/tools/arm-none-eabi-gcc-linux-x86_64/bin/`，Windows 目录名相应变化）：

POSIX 示例（`$elf` 换成第 5 步打印的实际 `.elf` 路径）：

```sh
nm=~/.xhive/tools/arm-none-eabi-gcc-linux-x86_64/bin/arm-none-eabi-nm
elf=/path/to/my_app/build/.../minimal.elf

# 关键符号都应为 T（已定义）：入口、厂商时钟初始化、C 运行库钩子
"$nm" "$elf" | grep -E " (Reset_Handler|SystemInit|_init|_fini)$"

# 列出未定义符号：命令成功退出且没有任何输出，才算没有未定义符号
"$nm" --undefined-only "$elf"

# 查看入口地址：去掉 Thumb 标志最低位后，应与 Reset_Handler 的地址一致
~/.xhive/tools/arm-none-eabi-gcc-linux-x86_64/bin/arm-none-eabi-readelf -h "$elf" | grep Entry
```

Windows PowerShell 对应写法（`$elf` 同样换成实际 `.elf` 路径）：

```powershell
$nm = "$HOME\.xhive\tools\arm-none-eabi-gcc-windows-x86_64\bin\arm-none-eabi-nm.exe"
$elf = "C:\path\to\my_app\build\...\minimal.elf"
& $nm --undefined-only $elf
```

这些检查通过表示"启动与厂商对象正确链接、没有未定义符号"，仍不等于实板验证；烧录工具与板卡操作不在本文范围。

## 第 7 步（可选）：使用 SDK 库——以 base64 为例

SDK 自带一些可移植库（base64、bitmap、divsufsort、ezctrl 等）。接入分两个层面，推荐两处都写：

1. **在 `.config` 里选择组件**：

   ```text
   CONFIG_LIBS_ADD_BASE64=y
   ```

   它决定聚合目标 `xhive_embed::libs` 把 base64 编入固件聚合链（`xhive_embed::deps` 已自动注入应用目标）。

2. **在应用目标上声明直接依赖**：

   ```lua
   namespace("app", function()
       target("minimal")
           set_kind("binary")
           add_rules("xhive.embed")
           add_files("src/*.c")
           add_deps("xhive_embed::base64")   -- 新增
       target_end()
   end)
   ```

   直接 `add_deps` 会把该库目标作为依赖一并构建，并向你的源文件传播它的公开头文件目录。

两个层面各司其职：`.config` 的 `LIBS_ADD_*` 控制**聚合选择**；直接 `add_deps` 建立**直接构建依赖**并传递公开使用要求。只写 `add_deps` 其实也能把库编进来，但让 `.config` 与实际使用保持一致，工程意图更清晰。mbedTLS 不在聚合映射内，需按其文档显式声明。

两处都改好后，在 `main.c` 文件顶部加入头文件：

```c
#include <base64.h>
```

再把以下代码放到 `main()` 内部、`for (;;)` 之前，保留原有 `_init` / `_fini`：

```c
char enc[9];                    /* ((5 + 2) / 3) * 4 = 8 字符，留 1 字节结尾 */
size_t n = base64_encode("xhive", 5, enc);
enc[n] = '\0';                  /* 该 API 不写结尾 NUL，需要调用方自己补 */
```

接口与缓冲区要求见 `libs/base64/inc/base64.h`；重新配置并构建通过，说明这个调用的头文件和链接依赖已接好。编译成功不是运行自测；运行时预期 `enc` 为 `"eGhpdmU="`，需另行在调试器或测试程序中核对。

## 第 8 步（可选）：使用厂商驱动与头文件

N32H474 的标准外设库、CMSIS 头文件已随 `CONFIG_VENDOR_USE_NATION` 等选项编入固件（源码在 `vendor/nation/n32h47x_48x/firmware/` 下，由 vendor 目标按 `.config` 自动挑选）。要在应用里直接使用厂商 API：

```lua
add_deps("xhive_embed::nation_n32h47x_48x")   -- 传播厂商公开头文件目录与宏
```

```c
#include "n32h47x_48x.h"    /* 厂商器件头 */
```

型号宏 `N32H474` 与当前选择对应的 `SYSCLK_SRC` 也由该目标以公开宏形式传递；`HSI_VALUE` 的来源见前面的时钟说明。外设 API 的用法参考厂商固件库自带文档与示例，本文不展开。

## 用 menuconfig 修改配置

不想手写 `.config`、或之后想改配置时，用交互界面（在应用目录）：

```sh
xmake menuconfig -P .
```

首次运行的推荐顺序是：`xmake menuconfig`（生成 `.config`）→ `xmake xhive_install`（按选择装工具链）→ `xmake f` → `xmake b`。注意 `xhive_install` 和 `xmake f` 都依赖 `.config` 已存在，所以 menuconfig 要在最前面，否则会提示先运行 `xmake menuconfig`。

菜单里的对应路径：MCU Vendor Selection → Nation → N32H47x/48x → N32H474；Clock Configuration；Memory Configuration；Compiler Options；Libraries / Third-party configurations。保存退出后 `.config` 写回应用根目录，再 `xmake f` 生效。手写与菜单两种方式可以混用：菜单改完的 `.config` 也可以直接手工微调。

## 更进一步

以下能力本文只指路，不展开：

- **RTOS**：`.config` 中 `THIRD_RTOS_NONE` 换成 FreeRTOS / RT-Thread / RT-Thread Nano（选项定义见 `third-party/Kconfig`）。注意选择 RTOS 后启动行为不同：默认启动代码只在 `THIRD_RTOS_NONE` 下调用 `__libc_init_array` / `__libc_fini_array` 并进入 `main`；RT-Thread 且启用用户 main 时入口改为 `entry`。
- **USB（N32H47x/48x）**：`NATION_N32H47X_48X_USE_USBFS` 默认开启、`USE_USBHS` 默认关闭；启用任一种 USB 驱动时，都需要在 `NATION_N32H47X_48X_EXTERNAL_INC_DIR` 指定应用侧的 USB 板级配置目录，否则配置阶段报错。最小示例先显式关闭。
- **自定义启动文件**：`CONFIG_USE_DEFAULT_STARTUP=n`，然后把自己的启动源文件加入目标 `add_files`。
- **自定义链接脚本**：`CONFIG_USE_DEFAULT_LD_SCRIPT=n` 并设置 `CUSTOM_LD_SCRIPT_PATH`（相对应用根目录）。默认脚本 `templates/link.ld` 是先经 C 预处理器展开 `CONFIG_*` 宏再使用的；如需对自己的脚本做同样处理，打开 `CUSTOM_LD_SCRIPT_NEED_PROC`。完全接管时可使用 `DISABLE_STARTUP_FILE_OPTIONS` / `DISABLE_LD_SCRIPT_OPTIONS` 关闭 SDK 的自动注入。
- **SEGGER RTT**：`THIRD_SEGGER_RTT` 默认开启，最小示例中显式关闭；需要调试输出时再打开。

## SDK 主机测试（与应用固件无关）

在 **SDK 根目录**执行。请**另开一个新终端**运行本节命令，不要沿用应用专用终端里的两个环境变量；若只能用同一终端，先恢复（POSIX：`unset XHIVE_SDK_PATH XMAKE_CONFIGDIR`；PowerShell：`Remove-Item Env:\XHIVE_SDK_PATH, Env:\XMAKE_CONFIGDIR`）：

```sh
cd /path/to/xhive
xmake f -P . -y -m debug --xhive_tests=y --xhive_cross_tests=n
xmake
xmake test -g xhive -v
```

这是 SDK 自身的软件回归（可移植库、构建逻辑等），不需要板卡，也不验证你的固件。选项与选择器规则（`-g`、`-v` 必须放在选择器前）详见 [README.zh_CN.md](../README.zh_CN.md)。

## 排错

按报错关键词查找：

1. `Application configuration not found` 或 `Please run 'xmake menuconfig'`：应用目录缺 `.config`。手写（第 2 步）或先运行 `xmake menuconfig -P .`。
2. `menuconfig tool not found` / `genconfig tool not found`：没装定制 kconfiglib。执行第 0 步的 pip 命令；Windows 菜单界面另装 `windows-curses`。
3. `Please run 'xmake xhive_install'`：交叉工具链未安装。在应用目录执行 `xmake xhive_install -P .`（前提：`.config` 已存在）。
4. `External USB include directory not set`：USBFS 默认开启。最小示例在 `.config` 写 `# CONFIG_NATION_N32H47X_48X_USE_USBFS is not set`，或按"更进一步"配置板级目录。
5. 链接报 `undefined reference to _init / _fini`：见第 3 步，在应用侧提供空实现；不要定义其他 CRT 符号。
6. `No valid SYSCLK source selected`：时钟选项组合不完整，按第 2 步的时钟段落补齐 `CLOCK_SYSCLK_*` 与 `CLOCK_PLL_SRC_*`。
7. build ok 但固件不启动：先做第 6 步的符号检查。缺入口（如 `Reset_Handler`）时 GNU ld 可能只警告，Xmake 仍显示成功，必须用 `nm` / `readelf` 核实。
8. SDK 目录里的命令突然作用到别的工程：Xmake 3.0.8 的 `-P` 缓存污染。回到 SDK 根目录执行 `xmake f -P . -y -m debug --xhive_tests=y --xhive_cross_tests=n` 重新选中 SDK 工程；日常为每个应用设置 `XMAKE_CONFIGDIR`（第 0 步）可避免。
9. 改了 `.config` 没生效：重新 `xmake f -P .` 再构建，确认 `build/xhive_config.h` 已更新。
10. PowerShell 里 `VAR=value xmake ...` 报错：命令前缀是 Bash 语法。PowerShell 用 `$env:VAR = "value"` 设置后直接运行命令。

## 下一步

- 通读 [README.zh_CN.md](../README.zh_CN.md)：主机测试、可选 RISC-V 交叉检查、工具链支持范围。
- 环境安装备选方式（pipx）：[Environment_zh_CN.md](Environment_zh_CN.md)。
- 浏览 `kconfigs/` 与 `vendor/` 下各 `Kconfig`：所有可配置项都有 help 文本。
- 尝试第 7、8 步，把一个真实外设初始化加进 `main.c`，再用第 6 步的方法检查产物。
