# N32H47x/48x 算法库逆向：common 公共库寄存器与函数语义（v1.2.0）

本文档描述厂商闭源库 `n32h47x_48x_algo_lib` v1.2.0 中 `n32h47x_48x_common`
模块（GCC 版 `lib/n32h47x_48x_common.a`，armclang 版 `.lib` 成员
`n32h47x_48x_common.o`）的硬件寄存器访问与全部 15 个导出符号的语义。

所有结论均标注证据（函数名+对象内偏移）。未确认内容明确标注 **推测**。

## 1. 证据源与构建指纹

| 项 | 值 | 证据 |
| --- | --- | --- |
| GCC 版编译器 | GNU C17 10.2.1 20201103 (release)，`-mcpu=cortex-m4 -mthumb -mfloat-abi=soft -march=armv7e-m -g -gdwarf-2 -Os -ffunction-sections -fdata-sections` | DWARF `DW_AT_producer` |
| 厂商源文件 | `firmware/n32h47x_48x_algo_lib/lib/n32h47x_48x_common.c`（未随 SDK 发布） | DWARF `DW_AT_name` |
| 构建目录 | `E:\NS3608\Nations.N32H47x_48x_Library.0.8.0\projects\n32h47x_48x_EVAL\examples\ALGO\ALGO_DEMO\GCC` | DWARF `DW_AT_comp_dir` |
| armclang 交叉验证 | `.lib` 内对象含 `.rev16_text/.revsh_text/.rrx_text` 等 armclang 特有节 | 对象节表 |
| 厂商源行号 | CLEAR_ARAM_DONE_CHECK@114、RandomSort@184、Cpy_U8@210、Cpy_U32@228、XOR_U8@265、XOR_U32@283、SetZero_U8@300、SetZero_U32@317、ReverseBytesInWord_U8@418、ReverseBytesInWord_U32@458、Cmp_U32@625、Cmp_U8@651、BufToFIFO@680、FIFOToBuf@742 | DWARF `DW_AT_decl_line` |
| 厂商私有头 | `SAC_Common.h`（与公开头文件同目录，未随 SDK 发布；提供 `SAC_TypeDef`） | `.debug_line` 文件表 entry 4 |

DWARF 与反汇编冲突时以反汇编实际行为为准；本次未发现二者冲突。

## 2. 涉及的硬件地址

common 库只访问一个外设：SAC（Security Algorithm Coprocessor，算法加速器）。

- `SAC_BASE = 0x4002A000`。证据：CMSIS `n32h47x_48x.h` 中
  `#define SAC_BASE (AHBPERIPH_BASE + 0xA000U)`（`AHBPERIPH_BASE = PERIPH_BASE +
  0x20000U`，即 0x40020000）；common 库代码字面量 `0x4002A000`
  （`CLEAR_ARAM_DONE_CHECK+0x28` 处 `.word 0x4002a000`）。
- `SAC_SRAM_BASE = 0x4002A400`（ARAM，加速器内部 SRAM）。证据：CMSIS 头
  `#define SAC_SRAM_BASE (AHBPERIPH_BASE + 0xA400U)`。common 库本身不直接访问
  该地址；函数名 `CLEAR_ARAM_DONE_CHECK` 中的 ARAM 即指该存储块（命名对应关系
  为 **推测**，依据是唯一被轮询的标志与"clear"语义）。

### 2.1 SAC 寄存器布局（DWARF `SAC_TypeDef`，定义于厂商私有头 `SAC_Common.h`）

| 偏移 | 名称 | 类型 | 说明 |
| --- | --- | --- | --- |
| 0x00 | SACCR | volatile uint32_t | 控制寄存器（命名展开 **推测**：SAC Control Register） |
| 0x04 | SACSR | volatile uint32_t | 状态寄存器（命名展开 **推测**：SAC Status Register） |
| 0x08 | RESERVED0 | uint32_t | 保留 |
| 0x0C | SACSFR1 | volatile uint32_t | 功能寄存器 1（含义未知，common 未用） |
| 0x10 | SACSFR2 | volatile uint32_t | 功能寄存器 2（common 未用） |
| 0x14 | SACSFR3 | volatile uint32_t | 功能寄存器 3（common 未用） |
| 0x18–0x2C | RESERVED1–6 | uint32_t×6 | 保留 |
| 0x30 | P1ADR | volatile uint32_t | 参数/数据指针 1 端口（FIFO 式访问，**推测**；aes/des/sm4/hash 反汇编中出现 0x4002A030 字面量） |
| 0x34 | P2ADR | volatile uint32_t | 参数/数据指针 2 端口（同上，des/hash/sm4 使用） |
| 0x38 | P3ADR | volatile uint32_t | 参数/数据指针 3 端口（同上，des/sm4 使用） |
| 0x3C | KADR | volatile uint32_t | Key 端口（**推测**；aes/des/sm4/hash 均写 0x4002A03C） |
| 0x40 | IADR | volatile uint32_t | Input 数据端口（**推测**；hash 使用 0x4002A040） |
| 0x44 | OADR | volatile uint32_t | Output 数据端口（**推测**；hash 使用 0x4002A044） |

结构体总大小 72 字节（0x48）。成员名与偏移为 DWARF 直接证据；各端口寄存器的
具体语义由 aes/des/sm4/hash 各文档负责，此处只列布局与 common 库证据。

注意：本 SDK 发布的 `CMSIS/device/n32h47x_48x.h` 只有 `SAC_BASE`/`SAC` 宏，
没有 `SAC_Module` 定义；上表布局全部来自库内 DWARF。

## 3. common 库访问的寄存器位

### 3.1 SACCR（0x4002A000）bit 7（掩码 0x80）

- 访问方式：只读轮询。
- 语义：非 0 表示"ARAM 清除操作仍在进行"（busy）。依据：`CLEAR_ARAM_DONE_CHECK`
  在该位非 0 时自旋等待，清 0 后才认为清除完成（`CLEAR_ARAM_DONE_CHECK+0x08..+0x0e`，
  armclang 版 `i.CLEAR_ARAM_DONE_CHECK+0x14..+0x1c` 一致）。
- bit 7 是否同时指示其他操作的 busy：**推测**（common 库证据不足以区分；仅知
  本函数语境下它代表 ARAM 清除忙标志）。

### 3.2 SACSR（0x4002A004）bit 0（掩码 0x01）

- 访问方式：**读-改-写**（read-modify-write）。先回读 SACSR，再写回
  `回读值 | 0x01`。证据：`CLEAR_ARAM_DONE_CHECK+0x10..+0x16`
  （`ldr r2,[r3,#4]; orr r2,r2,#1; str r2,[r3,#4]`）；armclang 版
  `+0x1e..+0x28` 完全一致。
- 语义：ARAM 清除完成后的"完成确认/应答"位。依据：仅当 SACCR bit7 已经清 0
  （清除完成）后才置位，且函数名 `CLEAR_ARAM_DONE_CHECK`。
- 该位是 RW 控制型还是 write-1-to-clear 型状态位：**推测**为前者（厂商代码
  刻意做 RMW 保留其他位；若为 W1C 型，回读写回会误清其它标志，厂商应不会这样写）。
- 超时路径**不写**该寄存器：SACCR bit7 持续为 1 且计数超限时直接返回
  `Time_Out(2)`（`CLEAR_ARAM_DONE_CHECK+0x20`），硬件可能仍在忙。

### 3.3 CLEAR_ARAM_DONE_CHECK 的完整 MMIO 序列

```text
timeout = Alg_TimeOut_Counter        // GCC 版仅函数入口读一次（+0x00..+0x02）
                                    // armclang 版每轮循环重读一次（+0x06..+0x08）
i = 0
loop:
    if ((*(volatile uint32_t*)0x4002A000 & 0x80) == 0)   // +0x08..+0x0e
        goto done
    i++
    if (i > timeout) return 2                            // +0x1a..+0x22
    goto loop
done:
    *(volatile uint32_t*)0x4002A004 =                    // +0x10..+0x16
        *(volatile uint32_t*)0x4002A004 | 0x01
    return 0
```

超时边界：设 `Alg_TimeOut_Counter = N`，SACCR 最多被读 N+1 次；全部为忙则返回
`Time_Out`。N=0 时允许 1 次忙观察。返回 0 与枚举 `SetZero_OK`/`Cpy_OK` 同值，
返回 2 即 `Time_Out`（DWARF 枚举：Cpy_OK/SetZero_OK/XOR_OK/Reverse_OK/
Cmp_EQUAL=0，Cmp_UNEQUAL=1，Time_Out=2）。

## 4. 导出符号语义总表

除 `Alg_TimeOut_Counter` 外，函数第三长度参数的单位：`*Len` 后缀为字节数的
用"字节"，`wordLen` 为 32 位字数（即字节数 ×4）。参数名取自 DWARF。

| 符号 | 签名（DWARF） | 语义 | 返回值 | 关键证据 |
| --- | --- | --- | --- | --- |
| `Alg_TimeOut_Counter` | `uint32_t`（.data，初值 0x000186A0=100000） | 硬件轮询超时预算（次数，非时间） | — | `.data.Alg_TimeOut_Counter` 内容 `a0860100` |
| `CLEAR_ARAM_DONE_CHECK` | `uint32_t (void)` | 见 §3.3 | 0=完成并应答；2=超时 | `CLEAR_ARAM_DONE_CHECK+0x00..+0x28` |
| `Cpy_U8` | `uint32_t (uint8_t *dst, uint8_t *src, uint32_t byteLen)` | 逐字节复制 | 恒 0（Cpy_OK） | `Cpy_U8+0x00..+0x12` |
| `Cpy_U32` | `uint32_t (uint32_t *dst, const uint32_t *src, uint32_t wordLen)` | `(dst\|src)&3 != 0` 时整段按字节复制（wordLen×4 字节）；否则按字复制（4 字/组 + 余量） | 恒 0（未对齐路径返回 Cpy_U8 的返回值，同为 0） | `Cpy_U32+0x02..+0x16`（+0x16 为 `R_ARM_THM_JUMP24 → Cpy_U8` 尾跳） |
| `XOR_U8` | `uint32_t (uint8_t *a, uint8_t *b, uint8_t *c, uint32_t byteLen)` | `c[i]=a[i]^b[i]` | 恒 0 | `XOR_U8+0x0c..+0x16` |
| `XOR_U32` | `uint32_t (uint32_t *a, uint32_t *b, uint32_t *c, uint32_t wordLen)` | `c[i]=a[i]^b[i]`（字） | 恒 0 | `XOR_U32+0x0c..+0x1a` |
| `SetZero_U8` | `uint32_t (uint8_t *dst, uint32_t byteLen)` | `memset(dst,0,byteLen)` | 恒 0 | `SetZero_U8+0x06 → memset`（重定位） |
| `SetZero_U32` | `uint32_t (uint32_t *dst, uint32_t wordLen)` | 先清 `wordLen>>2` 组×4 字（strd 对），再对尾部 `(wordLen&3)` 字调 `memset` | 恒 0 | `SetZero_U32+0x24..+0x2c`、`+0x1c → memset` |
| `ReverseBytesInWord_U8` | `uint32_t (uint8_t *dst, const uint8_t *src, uint32_t wordLen)` | 每个 4 字节组内字节倒序；dst==src 原地交换 0↔3、1↔2，否则 `dst[4i+k]=src[4i+3-k]`。**第三参数按 4 字节组计数（wordLen），不是字节数** | 恒 0 | `ReverseBytesInWord_U8+0x00..+0x66` |
| `ReverseBytesInWord_U32` | `uint32_t (uint32_t *dst, const uint32_t *src, uint32_t wordLen)` | 调 U8 版（参数透传），丢弃返回值后返回 0。非尾调用（bl 后独立 return 0） | 恒 0 | `ReverseBytesInWord_U32+0x02 → ReverseBytesInWord_U8`（重定位） |
| `Cmp_U32` | `int32_t (const uint32_t *a, uint32_t aWordLen, const uint32_t *b, uint32_t bWordLen)` | 先比长度，再逐字比较 | 0=相等；1=不等 | `Cmp_U32+0x00..+0x22` |
| `Cmp_U8` | `int32_t (const uint8_t *a, uint32_t aByteLen, const uint8_t *b, uint32_t bByteLen)` | 先比长度，再逐字节比较 | 0=相等；1=不等 | `Cmp_U8+0x00..+0x1e` |
| `RandomSort` | `uint32_t (uint8_t *order, const uint8_t *rand, uint32_t len)` | 见 §6 | 恒 0 | `RandomSort+0x14..+0x34` |
| `BufToFIFO` | `uint32_t (volatile uint32_t *dst, const void *src, uint32_t wordLen)` | 见 §5.1（未在公开头文件声明） | 恒 0 | `BufToFIFO+0x00..+0xa2` |
| `FIFOToBuf` | `uint32_t (void *dst, volatile uint32_t *src, uint32_t wordLen)` | 见 §5.2（未在公开头文件声明） | 恒 0 | `FIFOToBuf+0x00..+0xae` |

注意：`BufToFIFO`、`FIFOToBuf`、`ReverseBytesInWord_U8` 是导出符号但
`inc/n32h47x_48x_algo_common.h` 未声明，属库内部接口（供 aes/des/sm4/hash 等
模块链接使用，**推测**）。

## 5. FIFO 数据搬运函数

### 5.1 BufToFIFO(volatile uint32_t *dst, const void *src, uint32_t wordLen)

- `dst`：FIFO 写口（固定地址，每写一个 32 位字自动入队一个字，小端原样写入，
  不做字节序变换）。调用方传入的应为 SAC 的数据/参数端口之一
  （P1ADR/P2ADR/P3ADR/KADR/IADR 一类，见 §2.1；具体由各算法库决定，**推测**）。
- `src`：内存源缓冲区，需可读 `wordLen*4` 字节。
- `wordLen`：**32 位字数**（重要：不是字节数。`wordLen>>2` 组、每组搬 16 字节 =
  4 字，尾部再搬 `wordLen&3` 字；共写入 `wordLen` 个字）。
- 对齐处理（检查对象是 `src`，`BufToFIFO+0x08` `lsls r2,r1,#30`）：
  - 4 对齐：直接 `*dst = w[i]`，4 字/组展开 + 尾部逐字（`+0x88..+0x30`）。
  - 非 4 对齐：每轮用 `Cpy_U8(tmp, src, 16)` 把 16 字节暂存到栈上 `tmp[4]`
    （`+0x32..+0x48`，重定位 `+0x36 → Cpy_U8`），再把 tmp[0..3] 写入 FIFO；
    尾部 `(wordLen&3)` 字同样经 tmp 中转；结束后 `SetZero_U32(tmp, 4)` 清零
    16 字节暂存区（`+0x6e → SetZero_U32`）。
- 返回 0。无长度/空指针检查。
- armclang 版（`i.BufToFIFO+0x0c..+0x14`）逻辑一致。

### 5.2 FIFOToBuf(void *dst, volatile uint32_t *src, uint32_t wordLen)

- **参数顺序与 BufToFIFO 不对称**：第 1 参数是内存目的缓冲区（DWARF 名
  `dst`，类型 `void*`），第 2 参数才是 FIFO 读口（DWARF 名 `src`，类型
  `volatile uint32_t*`）。证据：反汇编中读侧固定地址 `[r4]`（第 2 参数，
  `FIFOToBuf+0x2e/+0x36`），写侧指针递增（第 1 参数，`+0x30/+0x94`）；
  armclang 版 `i.FIFOToBuf` 相同。
- `wordLen`：同样为字数，共读出 `wordLen` 个字。
- 对齐怪癖（厂商行为，如实复刻）：对齐检查作用在 `src`（FIFO 指针）而非
  `dst`（`FIFOToBuf+0x04` `lsls r3,r1,#30`；armclang `+0x0c` `and r0,r4,#3`）。
  FIFO 寄存器地址恒为 4 对齐，因此中转路径实际不可达，字永远直接写入 `dst`。
  **推测**：厂商原意应是检查 `dst`，属源码笔误；因硬件地址恒对齐而未暴露。
- 中转路径（不可达分支）行为：每轮从 FIFO 读 4 字进 `tmp[4]`，`Cpy_U8(dst,
  tmp, 16)`；尾部 `(wordLen&3)` 字先进 tmp 再 `Cpy_U8`；最后
  `SetZero_U32(tmp, 4)`。
- 返回 0。

## 6. RandomSort 与随机源

```text
for (i = 0; i < len; i++) order[i] = i;          // +0x14..+0x18
for (i = 0; i < len; i++) {                      // +0x1a..+0x34
    index = i + rand[i] % (len - i);             // udiv/mls：r%((len-i))
    swap(order[i], order[index]);
}
return 0;
```

- 标准 Fisher–Yates 前向洗牌：第 i 轮从 `[i, len)` 均匀选下标（前提是
  `rand[i]` 均匀，模偏差存在但厂商未做消偏）。
- `order` 被完全覆写为 `0..len-1` 的一个排列；`rand` 需提供至少 `len` 字节。
- `rand` 的来源：common 库内无任何对 RNG 外设的访问（rng 库对象中 0x4002A000
  字面量与 common 无重定位关联），**推测**由 RNG 模块产出随机字节流后作参数
  传入（调用点在其它库中，超出本文档范围）。
- 头文件声称返回 `RandomSort_OK`，但公开头文件未定义该枚举，二进制恒返回 0
  （与 `Cpy_OK` 同值）。疑似头文件文档缺陷，仅记录，不修改 `inc/`。

## 7. Alg_TimeOut_Counter

- 定义：`uint32_t Alg_TimeOut_Counter = 100000;`（0x000186A0），位于 .data
  （`.data.Alg_TimeOut_Counter`，4 字节，初值即上述常量）。
- 语义：轮询**次数**预算，不是时间。`CLEAR_ARAM_DONE_CHECK` 及其它库的等待
  循环以它为上界。
- 边界（本函数）：N 值 → 最多 N+1 次状态读；0 → 仅 1 次读。
- 并发：DWARF 类型为普通 `uint32_t`（非 volatile）。GCC 版在函数入口快照一次
  （`CLEAR_ARAM_DONE_CHECK+0x00..+0x02`），armclang 版每轮重读
  （`+0x06..+0x08`）。若在中断中修改该变量，两个版本行为不同；调用方若依赖
  运行中调整超时，应以 armclang 版行为为准（**推测**：厂商两版未刻意区分，
  差异源于编译器选择）。

## 8. 不确定点汇总

| 编号 | 内容 | 状态 |
| --- | --- | --- |
| U1 | SACCR/SACSR 缩写展开（Control/Status Register） | 推测（成员名与偏移是 DWARF 实证） |
| U2 | SACCR bit7 是否还指示 ARAM 清除以外的 busy | 推测 |
| U3 | SACSR bit0 的寄存器型（RW vs W1C）及"clear-done acknowledge"命名 | 推测（RMW 行为是实证） |
| U4 | P1ADR/P2ADR/P3ADR/KADR/IADR/OADR 的端口语义 | 推测（偏移是 DWARF 实证；使用者为兄弟库） |
| U5 | FIFOToBuf 对齐检查对象疑似厂商笔误（应查 dst 却查 src） | 推测（检查 src 是两编译器一致的实证） |
| U6 | RandomSort 的 rand 来源（RNG 库）与调用方 | 推测 |
| U7 | `RandomSort_OK` 未在公开头定义 | 实证（头文件内容 vs 二进制返回值），疑似厂商文档缺陷 |
| U8 | BufToFIFO/FIFOToBuf 传入的具体 FIFO 端口寄存器 | 推测（由各算法库决定） |

## 9. 证据偏移索引（GCC 对象 `n32h47x_48x_common.o`）

| 函数 | 大小 | 关键偏移 |
| --- | --- | --- |
| CLEAR_ARAM_DONE_CHECK | 0x2c | +0x00 加载 Alg_TimeOut_Counter；+0x08..+0x0e 轮询 SACCR&0x80；+0x10..+0x16 SACSR RMW 置 bit0；+0x1a..+0x1e 计数比较；+0x20 返回 2；+0x24 重定位 Alg_TimeOut_Counter；+0x28 字面量 0x4002A000 |
| RandomSort | 0x36 | +0x14..+0x18 初始化；+0x22/+0x26 udiv/mls；+0x2e..+0x32 交换 |
| Cpy_U8 | 0x14 | +0x0c..+0x12 字节循环 |
| Cpy_U32 | 0x6e | +0x02..+0x06 对齐判断；+0x14..+0x16 尾跳 Cpy_U8；+0x40..+0x5c 4 字组；+0x64..+0x68 尾字 |
| XOR_U8 | 0x18 | +0x0c..+0x14 |
| XOR_U32 | 0x1e | +0x0c..+0x1a |
| SetZero_U8 | 0x0e | +0x06 → memset |
| SetZero_U32 | 0x30 | +0x24..+0x28 strd×2；+0x1c → memset |
| ReverseBytesInWord_U8 | 0x68 | +0x00..+0x08 分支；+0x16..+0x32 非原地；+0x3a..+0x56 原地 |
| ReverseBytesInWord_U32 | 0x0a | +0x02 → ReverseBytesInWord_U8；+0x06 返回 0 |
| Cmp_U32 | 0x24 | +0x00 长度比较；+0x10..+0x1e 逐字 |
| Cmp_U8 | 0x20 | +0x00 长度比较；+0x10..+0x1a 逐字节 |
| BufToFIFO | 0xa4 | +0x08 src 对齐判断；+0x88..+0xa2 对齐 4 字组；+0x2a..+0x30 尾字；+0x32..+0x54 中转组；+0x36/+0x62 → Cpy_U8；+0x6e → SetZero_U32；+0x72 返回 0 |
| FIFOToBuf | 0xb0 | +0x04 src 对齐判断；+0x94..+0xac 对齐 4 字组；+0x2e..+0x34 尾字；+0x36..+0x76 中转路径；+0x4c/+0x6e → Cpy_U8；+0x76 → SetZero_U32；+0x7a 返回 0 |
