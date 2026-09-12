# StarrySky C2 外设驱动

本目录提供寄存器定义和不依赖 OS、堆内存的 C11 基础驱动。头文件可由 C++11 使用。
统一入口为 `#include "c2.h"`，也可以只包含所需外设头文件。

## 使用约定

- 通过各外设头文件声明的 `c2_*` 函数访问。返回值为 `c2_status_t`；`C2_OK` 表示该函数规定的软件操作成功，不代表硬件已通过实板验证。
- API 显式接收寄存器指针，例如 `C2_I2C0`、`C2_UART0`、`C2_TIM1`。传入非空但无效的地址不会被驱动探测或保护。
- 时钟频率由调用者以 Hz 提供，不从 ARCHINFO 猜测。板卡默认晶振为 72 MHz，但程序应传入实际频率。
- `poll_limit` 是有限轮询次数，**不是微秒或毫秒**。预算是整次调用还是单个阶段，以及零预算的语义，以对应头文件为准；不得把它当作硬实时截止时间。已经发出的硬件操作可能在软件超时后继续执行。
- 驱动不安装中断处理函数、不操作全局中断控制器，也不提供内部互斥锁。调用者必须串行化同一外设事务，防止线程/ISR 同时访问。GPIO 的软件影子状态需要独占管理。
- 全部 MMIO 通过 `c2_mmio_read32()` / `c2_mmio_write32()` 完成，使用完整的 32 位访问；RISC-V 路径包含 I/O fence。测试可覆写 `C2_MMIO_READ32` / `C2_MMIO_WRITE32`。
- 不要直接修改 MMIO 的 `BITS` 成员。位域读写可能变成读改写或窄访问；如需自行组装寄存器，使用本地值，再一次写入 `WORD`。FIFO、读清零状态只读一次后处理快照。
- 含 `UNKNOWN` 或“推断”注释的寄存器字段不等于已公开的硬件规范。本实现不会把参考 IP 独有的寄存器加入 C2。

## 特性与限制

| 外设 | 关键约束 |
| --- | --- |
| GPIO | 16 路；DDR 的 1 为输入、0 为输出。DR/DDR 有读冒险，输出、方向及 toggle 必须使用软件影子状态。 |
| SYS_UART | 分频为 `clock_hz / baud`，没有 TX-ready、线路状态或硬件 flush 寄存器，不能提供等同 HP_UART 的能力。 |
| HP_UART | 与 SYS_UART 布局不同；分频包含减一。LSR 读取会清中断挂起状态；发送入 FIFO 与按 TEMT 等待物理发送完成是不同操作。 |
| Timer | 两个实例；通过 DATA 读取递减计数，不能套用参考 IP 的 PSCR/CMP/IRQ。延时与时基不可同时占用同一计时器。 |
| PWM | 四路共用周期；`period = CMP`、`divider = PSCR + 1`，输出高电平在 `CNT >= CR`，占空比方向不能写反。CNT 无软件读通路。STAT 读清零。 |
| I2C | 基础 7 位主机事务；按 works 当前约定，ACK 位 1 回 ACK、0 回 NACK。参考 OpenCores 控制器与 pad 的有效电平定义存在矛盾，需在实板确认中间 ACK/最后 NACK 波形。固定引脚不支持 GPIO 总线恢复。 |
| QSPI | SDK FIFO 写法与启动裸事务的对齐方式不同；BUSY 与启动完成值也不能混用。未公开的接收/FIFO/长度字段不作猜测。 |
| PSRAM | 初始化影响共享 QSPI 和 GPIO15 路由，必须按启动条件调用，不能在正在使用 PSRAM 时重初始化。 |
| RNG | 32 位 LFSR 伪随机数，**不用于密码学**。CTRL.EN 仅允许写种子；置 0 不停止生成。零种子会锁在零状态。 |
| PS2 | 仅接收扫描码，不发送键盘命令、不翻译扫描码。DATA=0 与空 FIFO 无法可靠区分；STAT 读清 ITF。 |
| ARCHINFO | 可读出原始快照、解码 BCD 和显式写入元数据。不是可信芯片身份，也不设置真实时钟或 SRAM 容量；硅片写入是否生效需读回确认。 |

时钟、电源没有独立控制寄存器。Flash 擦写涉及 XIP 安全及器件命令集，不等同于 QSPI
控制器的基础接口；本目录不会在初始化时擦写 Flash 或执行破坏性 PSRAM 测试。

## HP_UART 的最小使用边界

HP_UART 保持同步轮询实现，不分配内存、不安装 ISR、不持有软件收发队列，也不执行
隐式等待、重试或恢复。单个字节的非阻塞接口可用于主循环；需要等待时使用已有的
有限轮询接口。不要将轮询次数解释为毫秒，长时间等待发送也不会顺便处理接收数据。

- 一个外设只由一个 context 管理，调用者串行化访问。普通轮询使用 `irq_enable_mask=0`；
  即使仅发送，轮询 LSR 也会清除接收等中断的挂起状态，不能与独立 ISR 随意混用。
- `write()` 成功表示已入 TX FIFO。需要确认最后一个停止位发完时，显式调用
  `c2_hp_uart_wait_tx_complete()`；超时不会取消已入队数据，重试时必须考虑 `written`。
- `init()` / `configure()` 不等待空闲，会改写帧格式并清空双方 FIFO。保留待发数据时，
  应先等待 TX 完成；接收侧需要对端停止发送，并明确处理现有 RX 数据后再重配。
- `flush()` 仅丢弃指定 FIFO 的内容，不等于等待发送或复位线路上的当前字符。
  `disable()` 仅关闭外设中断使能和软件访问，不停止硬件收发。
- 接收返回 `C2_ERROR_IO` 时，校验错误字节已经被取出并存入输出。批量接口立即停止，
  且 `read_count` 包含这个最后的错误字节；它不能作为可信数据继续解析。
- 合法 context 下，零长度批量收发返回 `C2_OK`、计数为 0，不访问 MMIO，允许数据指针
  为 `NULL`。非零长度配零预算返回超时；参数校验失败时，输出计数保持原值。

协议分帧、重试策略和并发调度由上层负责。HP_UART 的实板双向收发、FIFO 压力及线路
时序仍需单独验证，不能用 SYS_UART 的 Hello 输出或主机 MMIO 模拟代替。

## PSRAM 的 C 初始化

`c2_psram_init()` 按官方 `start.S` 的已观察序列执行：

1. QSPI `STATUS=0x10 → 0`，清 `INTCFG`、`DUM`，设置 `CLKDIV=1`。
2. 用 GPIO 软件影子状态将 GPIO15 拉低并设为输出，保留其它引脚的状态。
3. 保守时序 `WC=18`、`CHD=4`。
4. 依次发送 `0x66`、`0x99`、`0x35`；每条命令使用 `LEN=0x80000`、左对齐 `TXFIFO`、`STATUS=0x102`，按**完整 STATUS 等于 1**等待完成。
5. 全部成功后才拉高 GPIO15，最后设置运行时序 `WC=8`、`CHD=0`。

这些是 C2 参考工程的观察值，不会随主频自动换算；更换晶振频率或 PSRAM 器件时，
必须重新核对分频、等待周期及复位恢复时间。

这里没有用 SDK FIFO 模式的 `BUSY==0` 替代启动完成条件，也不会在失败后继续切换
到运行模式。GPIO context 必须由应用独占，并在之后继续用于 GPIO 操作，不能另建
失去影子状态的 context。**PSRAM 使用期间必须保留 GPIO15 的高电平和输出方向，
不能再次调用全端口 GPIO 初始化，也不能通过整端口写入覆盖该位。** 初始化不改写
mapped PSRAM 的内容，也不会自动做容量测试。

**必须满足的启动条件：**

- 只在冷启动阶段、首次使用 PSRAM 前调用。已在用的 PSRAM 中不能存放当前栈、context、数据或代码。
- 调用代码及其依赖（包括 GPIO 函数、回调）、常量、栈均须放在已经可用的存储中；通常需要内部 SRAM/ROM。不能在共享 QSPI 的 Flash XIP 路径上执行该重配置序列。
- 本库不擅自修改应用链接脚本或启动入口，不会自动搬运 `.ramfunc`。应用必须先提供内部 SRAM 栈，并通过链接/复制安排满足上述执行条件，再初始化 PSRAM，之后才使用外部 RAM 的 `.data`/`.bss`/堆栈。
- `poll_limit` 是三条命令共享的 STATUS 读取总预算，至少为 3；不是时间保证。回调可为各阶段实现板载 PSRAM 数据手册所需的等待时间。传 `NULL` 会复现官方代码没有显式软件延时的行为，不代表任何替代料都不需要延时。
- 失败意味着部分命令可能已经发出；没有自动回滚，也不保证立即重试安全。返回成功后才可访问 `C2_PSRAM_MAPPED_BASE`（8 MiB）。

仅展示调用关系，以下函数及其依赖仍需由应用安排到安全的执行区域：

```c
c2_status_t board_memory_cold_start(c2_gpio_t *gpio_in_sram,
                                    c2_psram_delay_fn board_delay,
                                    void *delay_context_in_sram)
{
    c2_status_t status;
    /* 示例只将 GPIO15 设为输出；实际应提供板级初始输出/方向值。 */
    status = c2_gpio_init(gpio_in_sram, C2_GPIO0,
                          C2_PSRAM_QPI_MODE_GPIO_MASK, 0u);
    if (status != C2_OK) {
        return status;
    }
    return c2_psram_init(C2_PSRAM0, C2_QSPI0, gpio_in_sram, 100000u,
                         board_delay, delay_context_in_sram);
}
```

## 小型外设示例

```c
#include "c2.h"

void example(void)
{
    uint32_t sample;
    c2_archinfo_snapshot_t raw;
    c2_archinfo_info_t info;
    uint8_t scan;
    size_t received;

    if (c2_rng_init(C2_RNG0, UINT32_C(0x12345678)) == C2_OK) {
        (void)c2_rng_read(C2_RNG0, &sample); /* 仅示例，不是安全随机种子 */
    }
    if (c2_archinfo_read(C2_ARCHINFO0, &raw) == C2_OK) {
        (void)c2_archinfo_decode(&raw, &info);
    }
    (void)c2_ps2_init(C2_PS20, false); /* 轮询模式，不开启 IRQ */
    (void)c2_ps2_receive(C2_PS20, &scan, 1, &received, 10000);
}
```

## 构建与软件验证

现有 `openecos_starrysky` XMake 目标通过 `c2/src/**.c` 收集实现、公开 `c2/inc`。
本轮只修正该脚本两处空表初始化的 Lua 语法（`[]` 改为 `{}`），保留原有配置逻辑。
具体芯片/板卡选择、启动代码和链接脚本仍由应用工程配置。

独立验证不需要连接板卡，也不依赖上述资料目录：

```sh
python3 vendor/openecos/starrysky/c2/tests/run_tests.py
python3 vendor/openecos/starrysky/c2/tests/run_tests.py --sanitize
```

脚本需要本机 `cc`、`c++` 和带 RISC-V 后端的 `clang`；也可通过 `--cc`、`--cxx`、
`--clang` 指定工具。产物放在临时目录，不写入固件或板卡。

验证包含各外设的 MMIO 脚本模拟、C11/C++11 头文件检查、所有公开驱动函数的 C++
链接检查、独立寄存器布局/位域测试、只读寄存器写入拒绝检查，以及 RV32IM/ILP32
freestanding 目标编译。`--sanitize` 额外开启本机 ASan/UBSan。

频率和时间换算使用 64 位整数防止溢出；RV32 固件正常链接时可能需要工具链的
`libgcc` / `compiler-rt` 整数运算辅助函数。目标对象编译和本机链接检查不等于完整
应用固件已完成交叉链接；还需使用实际启动文件、运行库和链接脚本生成固件。

## 资料来源

- `/home/zmmfly/prjs/boards/StarrySky-C2/works/docs/periphs/*.md`
- `/home/zmmfly/prjs/boards/StarrySky-C2/repos/embedded-sdk/board/StarrySkyC2/`
- `/home/zmmfly/prjs/boards/StarrySky-C2/repos/starrysky-mc-server/firmware/`

文档整理、官方 SDK 和参考工程之间存在差异；各驱动中的注释说明采用的行为和限制。
Host MMIO 模拟验证只能检查软件序列、参数及错误路径，不能验证 APB 时序、真实
FIFO 深度、外设线路波形、Flash XIP 仲裁或 PSRAM 的电气时序。
