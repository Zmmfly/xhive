# N32H47x/48x 算法加速器（SAC）AES 寄存器与厂商库行为逆向文档

依据 `n32h47x_48x_algo_lib/lib/n32h47x_48x_aes.a`（GCC arm，带 DWARF）逆向整理。
术语保留英文（SAC、SACCR、FIFO 等），正文中文。

## 1. 范围与证据源

- 逆向对象：`lib/n32h47x_48x_aes.a`（成员 `n32h47x_48x_aes.o`，GCC arm，DWARF2）。
  - 编译器（DWARF `DW_AT_producer`）：`GNU C17 10.2.1 20201103 (release) -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -march=armv7e-m -g -gdwarf-2 -Os -ffunction-sections -fdata-sections`。
  - 源文件（DWARF）：`../../../../../../firmware/n32h47x_48x_algo_lib/lib/n32h47x_48x_aes.c`，
    comp_dir `E:\NS3608\Nations.N32H47x_48x_Library.0.8.0\projects\n32h47x_48x_EVAL\examples\ALGO\ALGO_DEMO\GCC`。
  - 对象内只有 4 个函数节（`-ffunction-sections`），无内部 static 函数，全部逻辑内联在公共函数内：
    `AES_Init`(0x148 字节, 源行 40–125)、`AES_Crypto`(0x1e0, 源行 128–239)、`AES_Close`(0x38, 源行 246–258)、`AES_Version`(0x1c, 源行 270–278)。
- 交叉验证：armclang 版 `n32h47x_48x_aes.lib` 的 `n32h47x_48x_aes_1.o`，关键常量与流程一致
  （SACCR=0xD0、keylen 编码 8/16、`SACSFR1 |= 0x80`、Close 的 0x100/~0x10、Version 常量 5/0/25/3/19/0x10）。
- 外部依赖（common 库，另一 Agent 负责，本文直接采信其已确认语义）：
  - `uint32_t Alg_TimeOut_Counter`（初值 100000）；
  - `CLEAR_ARAM_DONE_CHECK()`：轮询 `0x4002A000` bit7（0x80）直到 0，随后对 `0x4002A004` 读-改-写置 bit0=1 并返回 0；超过 `Alg_TimeOut_Counter` 次返回 2（`Time_Out`）。它本身不置位 bit7，触发由调用方写 SACCR 完成；
  - `BufToFIFO(fifo_addr, data, length)`：把 `data` 起 `length` 个 32-bit 字（`length*4` 字节，小端）逐字写入 `*(volatile uint32_t*)fifo_addr`，源地址非对齐/非整字经栈缓冲处理，返回 0（长度单位是**字**，依据 common 库反汇编 `r7 = len>>2`、尾部 `(len&3)*4` 字节）。
- 引用约定：`AES_Init+0x12` 指 `.a` 内该函数节内的相对偏移（`objdump -d` 地址）；"源行 N"指 DWARF `decodedline` 映射到的厂商源码行号。
- 冲突处理原则：DWARF 与反汇编冲突时以反汇编实际行为为准（本文所有寄存器访问均直接来自反汇编指令）。

## 2. 模块概述

- AES 硬件挂在 **SAC**（DWARF 类型名 `SAC_TypeDef`，来自厂商 SAC 头文件，结构体行 192–212）上，基址 **0x4002A000**（`AES_Init+0x128`、`AES_Crypto+0x1c8` 等字面量）。
- SAC 是多算法共享控制器：`AES_Init` 写 `SACCR=0xD0`，同库 `DES_Init` 写 `0xD1`、`SM4_Init` 写 `0xD4`，即 bits[3:0] 为算法选择位（AES=0b0000、DES=0b0001、SM4=0b0100，DES/SM4 编码来自兄弟库对象，对 AES 文档属旁证）。
- 时钟：RCC 基址 0x40021000，`AHB1PCLKEN`（DWARF `RCC_Module.AHB1PCLKEN`，结构体偏移 0x3C，即 0x4002103C）bit15 置 1 开 SAC 时钟。旧逆向源码把该寄存器称为 `RCC_AHB2ENR`：地址与位相同，但 DWARF 结构体成员名为 `AHB1PCLKEN`，以 DWARF 为准。
- 复位值：全部寄存器复位值在库中无证据，一律未标注（见第 9 节）。

## 3. 寄存器总表

| 偏移 | 地址 | 名称（DWARF） | 访问 | 复位值 | 作用 / 副作用 |
| --- | --- | --- | --- | --- | --- |
| 0x00 | 0x4002A000 | SACCR | RW | 未知 | 算法选择、使能、ARAM 清除触发；bit7 为清除 busy（自清零） |
| 0x04 | 0x4002A004 | SACSR | RW（标志 W1C） | 未知 | 状态标志：bit0 ARAM 清除完成、bit1 单次运算完成；软件写 1 清零（实现为 RMW OR） |
| 0x08 | — | RESERVED0 | — | — | 未使用 |
| 0x0C | 0x4002A00C | SACSFR1 | RW | 未知 | AES 功能配置：方向、密钥长度、链接模式、START 触发 |
| 0x10 | 0x4002A010 | SACSFR2 | RW | 未知 | AES 库从未访问（推测供其他算法用） |
| 0x14 | 0x4002A014 | SACSFR3 | RW | 未知 | AES 库从未访问（推测供其他算法用） |
| 0x18–0x2C | — | RESERVED1–6 | — | — | 未使用 |
| 0x30 | 0x4002A030 | P1ADR | W（FIFO 写口） | — | AES：IV/CTR 初值写入口，一次写 4 字 |
| 0x34 | 0x4002A034 | P2ADR | W | — | AES 未使用（推测其他算法参数口） |
| 0x38 | 0x4002A038 | P3ADR | W | — | AES 未使用 |
| 0x3C | 0x4002A03C | KADR | W（FIFO 写口） | — | 密钥写入口，一次写 keyWordLen（4/6/8）字 |
| 0x40 | 0x4002A040 | IADR | W（FIFO 写口） | — | 数据块输入口，每块写 4 字 |
| 0x44 | 0x4002A044 | OADR | R（FIFO 读口） | — | 结果输出口，连续读依次弹出；每块读 4 字，CTR 尾部只读 remaind 字 |

DWARF `SAC_TypeDef` 成员布局（行 192–212）与上表一一对应：`SACCR@0, SACSR@4, SACSFR1@0xC, SACSFR2@0x10, SACSFR3@0x14, P1ADR@0x30, P2ADR@0x34, P3ADR@0x38, KADR@0x3C, IADR@0x40, OADR@0x44`。

## 4. 寄存器位域

### 4.1 SACCR（0x4002A000，控制寄存器）

| bit | 名称 | 取值 / 含义 | 证据 |
| --- | --- | --- | --- |
| 3:0 | ALGSEL | 0b0000 = AES；0b0001 = DES；0b0100 = SM4；其余未观察到 | `AES_Init+0x10` 写 0xD0；`DES_Init` 写 0xD1、`SM4_Init` 写 0xD4（兄弟库） |
| 4 | ENABLE | 1 = 算法模块已使能/初始化；库用它当"已 Init"标志读回判断 | `AES_Crypto+0x3e`、`AES_Close+0xe` 测试 `& 0x10`；`AES_Init` 写 0xD0 置位 |
| 6 | 未命名 | 未知。三个算法的 Init 值（0xD0/0xD1/0xD4）均置 1，属公共配置位；推测与算法 RAM/ARAM 映射相关 | 同上 |
| 7 | ARAMCLR | 写 1 启动 ARAM（算法 RAM）清除；清除期间读回 1，完成后自动回 0（busy/自清零） | `AES_Init` 写 0xD0 后调用 `CLEAR_ARAM_DONE_CHECK()`（common 库轮询 bit7 至 0，见 common 反汇编 `ands r0,r0,#128`） |
| 8 | DEINIT（推测） | Close 序列置 1 后立即清 bit4；推测为清除密钥/复位算法模块的触发位 | `AES_Close+0x12` `orr #256`，`+0x1a` `bic #16` |
| 其余位 | — | 未观察到 | — |

### 4.2 SACSR（0x4002A004，状态寄存器）

| bit | 名称 | 取值 / 含义 | 证据 |
| --- | --- | --- | --- |
| 0 | ARAM_DONE | ARAM 清除完成标志；软件写 1 清零（common 库实现为 RMW OR 1） | common `CLEAR_ARAM_DONE_CHECK+0x10` |
| 1 | DONE | 单次运算完成标志：密钥装载完成或一个 16 字节数据块完成时硬件置 1；软件写 1 清零（库实现为 RMW OR 2） | `AES_Init+0x86` 轮询 `lsls r1,r1,#30`（bit1），`+0x8a` 置 2 清除；`AES_Crypto+0x106`/`+0x10c` 同 |
| 其余位 | — | 未观察到（旧源码所称 KEYVALID/bit4 等在 0x4002A004 无证据） | — |

### 4.3 SACSFR1（0x4002A00C，AES 功能配置寄存器）

| bit | 名称 | 取值 / 含义 | 证据 |
| --- | --- | --- | --- |
| 1:0 | DIR | 0b01 = 加密（`En_De==AES_ENC` 或 `Mode==AES_CTR` 时强制 01，CTR 恒为加密方向）；0b10 = 解密 | `AES_Init+0xa2`（`(x&~0x3)\|1`）、`+0xd8`（`(x&0xFF)\|2`） |
| 3 | KEY192 | 1 = 192-bit 密钥 | `AES_Init+0xc2`：keyWordLen==6 → SACSFR1 直写 8 |
| 4 | KEY256 | 1 = 256-bit 密钥 | `AES_Init+0xc6`：keyWordLen==8 → 直写 16 |
| 5 | CBC | 1 = CBC 链接；0 = ECB/CTR | CBC：`AES_Init+0xfc` `(x&0xFF)\|0x20`；ECB/CTR：`+0xb8` `&= ~0x20` |
| 7 | START | 写 1 触发一次运算：先用于密钥装载/扩展，之后每写一个数据块触发一次块运算 | `AES_Init+0x78`（key 装载触发）、`AES_Crypto+0xf6`（主循环每块）、`+0x9e`（CTR 尾块） |
| 其余位 | — | 未观察到 | — |

注：`AES_Init` 对 SACSFR1 的第一次写是**直写**密钥长度值（0/8/16，`+0x34`），等效整体复位该寄存器；之后均为 RMW。

### 4.4 FIFO 端口（P1ADR/KADR/IADR/OADR）

- 每次总线访问搬运一个 32-bit 小端字；连续写/读按先后顺序进出 FIFO。
- 密钥顺序：`BufToFIFO(0x4002A03C, key, keyWordLen)`（`AES_Init+0x70`，`r0=0x4002A03C` 为 DWARF 记录的调用点常量）。
- IV/CTR 初值：仅 CBC 模式在 Init 时写，`BufToFIFO(0x4002A030, iv, 4)`（`AES_Init+0x106`）。ECB/CTR 不写 P1ADR；CTR 的计数器副本由软件维护（`AES_Crypto` 栈上 `iv_buf[4]`，DWARF 变量）。
- 数据块：每块 4 字写 IADR（`AES_Crypto+0xda`…`+0xf0`）；结果每块 4 字读 OADR（`+0x118`…`+0x12e`）。
- CTR 尾块（remaind<4 字时）：只读 remaind 个字并与输入字异或（`+0xbc`…`+0xcc`，每次循环重新读 OADR）；FIFO 中剩余未读的 keystream 字被丢弃（推测：下一次 START 会复位 FIFO，无板上证据）。

## 5. RCC 时钟开关

| 寄存器 | 地址 | 位 | 操作 | 证据 |
| --- | --- | --- | --- | --- |
| RCC->AHB1PCLKEN | 0x4002103C（RCC 基址 0x40021000 + DWARF 偏移 0x3C） | bit15 | Init 置 1（`AES_Init+0x6` RMW `orr #32768`）；Close 清 0（`AES_Close+0x28`）；Crypto 读回判 UnInit（`AES_Crypto+0x34`） |

任务里问"RCC 时钟开关（AHB2ENR?）"：地址与位（0x4002103C bit15）与 STM32 风格命名 AHB2ENR 巧合相同，但 DWARF 结构体成员名为 **AHB1PCLKEN**，本文采用 DWARF 命名。

## 6. 厂商库实际操作流程（附反汇编证据）

### 6.1 AES_Init（源行 40–125）

1. `RCC->AHB1PCLKEN |= 0x8000`（行 44，`+0x6`）。
2. `SAC->SACCR = 0x000000D0`：选择 AES + ENABLE + bit6 + 启动 ARAM 清除（行 46，`+0x10` 直写）。
3. `ret = CLEAR_ARAM_DONE_CHECK()`；`ret == Time_Out(2)` → 返回 `AES_TimeOutError(0x5a5a5a60)`（行 47–49，`+0x16`/`+0x1a`）。
4. 按 `parm->keyWordLen` 直写 SACSFR1：4→0x00、6→0x08、8→0x10；其他值返回 `AES_Crypto_KeyLengthError(0x5a5a5a5e)`（行 51–67，`+0x22`…`+0x10c`）。注意：直写发生在 key 空指针检查**之前**（`+0x34` 在 `+0x36` 前）。
5. `parm->key == NULL` → `AES_Crypto_ParaNull(0x5a5a5a5c)`（行 76–78，`+0x36`）。Init **不检查** `parm` 本身与 `parm->out`/`parm->in`。
6. `Mode` 为 CBC(0x22222222) 或 CTR(0x33333333) 时要求 `parm->iv != NULL`，否则 ParaNull（`+0x3a`…`+0x4c`）。
7. `Mode` 不是 ECB/CBC/CTR → `AES_Crypto_ModeError(0x5a5a5a5a)`（行 80–82，`+0x5e`→`+0x118`）。
8. `En_De` 不是 ENC(0x44444444)/DEC(0x55555555) → `AES_Crypto_EnOrDeError(0x5a5a5a5b)`（行 85–87，`+0x6c`）。
9. `BufToFIFO(0x4002A03C /*KADR*/, key, keyWordLen)`（行 90，`+0x70`）。
10. `SACSFR1 |= 0x80` 触发密钥装载（行 93，`+0x78`）。
11. 轮询 `SACSR & 0x2`，超时预算 `Alg_TimeOut_Counter`（预算外再轮询一次后判超时，见 6.5）；超时返回 `AES_TimeOutError`（行 95–98，`+0x84`…`+0xd0`）。
12. `SACSR |= 0x2` 清 DONE（行 103，`+0x8a`）。
13. 方向位：`En_De==ENC || Mode==CTR` → `SACSFR1 = (x & ~0x3) | 0x1`；否则（DEC）→ `SACSFR1 = (x & 0xFF) | 0x2`（行 105–111，`+0xa2`/`+0xd8`）。
14. 链接位：ECB 或 CTR → `SACSFR1 &= ~0x20`（行 115–117，`+0xb8`）；CBC → `SACSFR1 = (x & 0xFF) | 0x20` 并 `BufToFIFO(0x4002A030 /*P1ADR*/, iv, 4)`（行 119–122，`+0xfc`/`+0x106`）。
15. 返回 `AES_Init_OK(0)`（行 124，`+0xbe`）。

### 6.2 AES_Crypto（源行 128–239）

1. `parm->in == NULL || parm->out == NULL` → `AES_Crypto_ParaNull`（行 136–138，`+0xc`/`+0x12`）。**不检查 `parm` 本身**（`+0x4` 直接解引用）。
2. 长度校验（`inWordLen` 单位为字）：`inWordLen == 0`，或 `inWordLen % 4 != 0 且 Mode != CTR` → `AES_Crypto_LengthError(0x5a5a5a5d)`（行 141–143，`+0x1a`…`+0x2c`）。
3. 初始化检查：`(RCC->AHB1PCLKEN & 0x8000) == 0` 或 `(SAC->SACCR & 0x10) == 0` → `AES_Crypto_UnInitError(0x5a5a5a5f)`（行 145–147，`+0x34`/`+0x3e`）。注意判的是 **SACCR bit4**，不是状态寄存器。
4. CTR 模式：把 `parm->iv[0..3]` 复制到栈上 `iv_buf[4]`（无 NULL 检查，Init 已保证）（行 149–154，`+0x4c`…`+0x5c`）。
5. 主循环 `while (inWordLen != 0)`（整字块，行 156）：
   - CTR：写 `iv_buf[0..3]` 到 IADR（4 字，行 168–171，`+0x13a`…`+0x148`）；随后对 `iv_buf` 做 16 字节**大端计数器递增**（行 172–177，`+0x14a`…`+0x170`：从 byte15 往 byte0 找第一个非 0xFF 字节加 1，途经 0xFF 清 0，全 0xFF 回绕）。递增发生在 START **之前**（先写计数器、再自增、再触发）。
   - ECB/CBC：写当前 4 字输入块到 IADR（行 161–164，`+0xda`…`+0xf0`）。
   - `SACSFR1 |= 0x80` 触发块运算（行 186，`+0xf6`）。
   - 轮询 `SACSR & 0x2`，**每块独立**装满 `Alg_TimeOut_Counter` 预算；超时返回 `AES_TimeOutError`（行 188–193，`+0xfc`…`+0x17a`）。
   - `SACSR |= 0x2` 清 DONE（行 196，`+0x10c`）。
   - 输出（行 199–211）：CTR → `out[i] = OADR ^ in[i]`（4 次 OADR 读与输入异或，`+0x182`…`+0x1ac`）；ECB/CBC → `out[i] = OADR`（4 次读，`+0x118`…`+0x12e`）。
   - 指针推进 4 字，`inWordLen -= 4`（行 215，`+0x132`）。
6. 尾块：`remaind = 原始 inWordLen & 3`，`remaind != 0 且 Mode == CTR` 时（行 217，`+0x7c`/`+0x86`）：
   - 写当前 `iv_buf[0..3]` 到 IADR（行 219–222，`+0x88`…`+0x98`，不再自增）；
   - `SACSFR1 |= 0x80`、轮询+清 DONE（同上，行 224–234，`+0x9a`…`+0xb8`）；
   - `for (i = 0; i < remaind; i++) out[i] = OADR ^ in[i]`——**按字**异或 remaind 次，每次循环重新读 OADR（行 236，`+0xbc`…`+0xcc`）。
7. 返回 `AES_Crypto_OK(0)`（行 239）。

### 6.3 AES_Close（源行 246–258）

```
if ((RCC->AHB1PCLKEN & 0x8000) != 0 && (SAC->SACCR & 0x10) != 0) {   // +0x4…+0x10
    SAC->SACCR |= 0x100;                    // 行 251, +0x12
    SAC->SACCR &= ~0x10;                    // 行 252, +0x1a
    (void)CLEAR_ARAM_DONE_CHECK();          // 行 253, +0x22（返回值被忽略）
    RCC->AHB1PCLKEN &= ~0x8000;             // 行 256, +0x26
}
```

- 条件为两个检查的**与**：任一不满足则完全不动（不关时钟）。SM4/DES 的 Close 是同样流程，只是判 `(SACCR & 0x14)==0x14` / `(SACCR & 0x11)==0x11`。
- Close 中没有再置 SACCR bit7，`CLEAR_ARAM_DONE_CHECK` 在 bit7 已为 0 时立即成功并写 SACSR bit0=1；若 bit8 置位会重新拉高 bit7（busy），则该调用等待其完成——两种解释无法从静态证据区分，见第 9 节。
- Close 不触碰 SACSFR1/SACSR 其他位。

### 6.4 AES_Version（源行 270–278）

四个输出指针**无 NULL 检查**，直接写（`+0x4`…`+0x18`）：

| 输出 | 值 | 含义 |
| --- | --- | --- |
| `*type` | 0x05 | bit0=C(Commercial) + bit2=N(Normal)，即 CN 版（编码定义见头文件注释） |
| `*customer` | 0x00 | 标准版 |
| `date[0..2]` | 25, 3, 19 | 2025-03-19 |
| `*version` | 0x10 | 按头文件约定即 v1.0 |

注意：SDK 目录把该库标为 v1.2.0，但库自报 v1.0（GCC 与 armclang 一致）；两者差异仅作记录，不据此改动。

### 6.5 超时语义（三个轮询点一致）

Init 的 key 装载轮询（`+0x84`）、Crypto 主循环每块（`+0x102`）、Crypto 尾块（`+0xaa`）以及 common 的 `CLEAR_ARAM_DONE_CHECK` 均为同一模式：

```
i = 0;
while ((标志 & mask) == 0) {
    i++;
    if (i > Alg_TimeOut_Counter)   /* 继续条件是 CNT >= i */
        return TimeOut/AES_TimeOutError;
}
```

即每次预算最多进行 `Alg_TimeOut_Counter + 1` 次失败观测；返回超时前不写任何清除位。Crypto 主循环**每个数据块**重新装载预算。

## 7. .data / .rodata 常量

`readelf -S`：该对象 `.data`、`.bss`、`.rodata` 长度均为 0，无任何静态数据；全部常量以立即数或 literal pool 形式存在于代码中：

| 字面量 | 出现位置 | 含义 |
| --- | --- | --- |
| 0x40021000 | Init+0x124 / Crypto+0x1c4 / Close+0x30 | RCC 基址 |
| 0x4002A000 | Init+0x128 / Crypto+0x1c8 / Close+0x34 | SAC 基址 |
| 0x4002A03C | Init+0x130 | KADR（密钥 FIFO） |
| 0x4002A030 | Init+0x138 | P1ADR（IV FIFO） |
| 0x5a5a5a5a…0x5a5a5a60 | Init/Crypto 各错误路径 | 错误码（与 `inc/n32h47x_48x_aes.h` 枚举一致） |
| 0xD0 | Init+0x10 | SACCR 初始化值 |
| `Alg_TimeOut_Counter`（R_ARM_ABS32 重定位） | Init+0x134 / Crypto+0x1cc | common 库全局变量地址 |

## 8. 错误码表（反汇编与 inc 头文件一致性核对）

| 枚举 | 值 | 触发点（反汇编证据） |
| --- | --- | --- |
| AES_Crypto_OK / AES_Init_OK | 0x0 | 正常路径 |
| AES_Crypto_ModeError | 0x5a5a5a5a | Init：Mode 不属 ECB/CBC/CTR（`+0x118`） |
| AES_Crypto_EnOrDeError | 0x5a5a5a5b | Init：En_De 不属 ENC/DEC（`+0x114`） |
| AES_Crypto_ParaNull | 0x5a5a5a5c | Init：key NULL / CBC·CTR 时 iv NULL（`+0x110`）；Crypto：in 或 out NULL（`+0x1b8`） |
| AES_Crypto_LengthError | 0x5a5a5a5d | Crypto：inWordLen==0 或（%4!=0 且非 CTR）（`+0x1bc`） |
| AES_Crypto_KeyLengthError | 0x5a5a5a5e | Init：keyWordLen 不属 4/6/8（`+0x10c`） |
| AES_Crypto_UnInitError | 0x5a5a5a5f | Crypto：时钟未开或 SACCR bit4 未置（`+0x1c0`） |
| AES_TimeOutError | 0x5a5a5a60 | Init：ARAM 清除或 key 装载超时；Crypto：任一块/尾块超时 |

Crypto 里**没有** Mode/En_De/KeyLength 复查——这些只在 Init 中校验；Crypto 只查指针、长度、初始化状态。

## 9. 推测与未确认项

| 项 | 结论 | 依据与不确定性 |
| --- | --- | --- |
| SACCR bit6 | 推测：公共配置位（与 ARAM/算法 RAM 相关） | 仅知三个算法 Init 都置 1，功能未知 |
| SACCR bit8 (0x100) | 推测：DEINIT/密钥清除触发 | Close 序列 `|=0x100; &=~0x10` 后调用 ARAM 轮询；也可能只是无害写 0 |
| SACCR bit7 自清零 | 推测：busy 自清零 | common 库只轮询不手清；无"手清 bit7"的代码路径 |
| SACSFR1 START(bit7) 是否自清 | 未知 | 库每次都重新 `|= 0x80`，不依赖其当前值 |
| 各寄存器复位值 | 未知 | 库从不依赖复位值：SACCR 直写 0xD0，SACSFR1 直写 keylen |
| OADR FIFO 深度/溢出行为 | 未知 | CTR 尾块弃读剩余 keystream 字后下一块照常工作，推测每次 START 复位 FIFO |
| P2ADR/P3ADR/SACSFR2/SACSFR3 用途 | 推测：其他算法（DES/SM4/HASH）使用 | AES 库零访问；DWARF 只给成员名 |
| KADR/P1ADR 名称含义 | KADR=密钥口、P1ADR=IV 口为**用法推断**，名字来自 DWARF | Init 两次 BufToFIFO 的实参 |
| IV 字节序 | 推测：按小端字写入，板上需按标准向量字节序以小端组字 | FIFO 为 32-bit 小端字口；无实板向量验证 |
| `parm == NULL` | v1.2.0 不检查（Init/Crypto 直接解引用） | `+0x4`/`+0xc` 首指令即 load；属厂商行为，非本库可修缺陷 |

## 10. 建议板上测试向量（公开标准向量）

调用约定提醒：key/iv/in/out 均为 `uint32_t` 数组，硬件按小端 32-bit 字收发；下列十六进制为**字节序列**，填充数组时按小端组字（如 `6bc1bee2...` → `{0xe2bec16b, ...}`）。ECB/CBC 输入长度必须是 4 的倍数字（16 字节整数倍）；CTR 任意非零字数。

1. **AES-128 ECB 加密**（FIPS-197 C.1）
   - key：`000102030405060708090a0b0c0d0e0f`
   - in：`00112233445566778899aabbccddeeff`
   - 期望 out：`69c4e0d86a7b0430d8cdb78070b4c55a`
2. **AES-128 ECB 解密**：同 key，in 取上式密文，期望 out 为明文。
3. **AES-192 ECB 加密**（FIPS-197 C.2）
   - key：`000102030405060708090a0b0c0d0e0f1011121314151617`
   - in：`00112233445566778899aabbccddeeff`
   - 期望 out：`dda97ca4864cdfe06eaf70a0ec0d7191`
4. **AES-256 ECB 加密**（FIPS-197 C.3）
   - key：`000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f`
   - in：`00112233445566778899aabbccddeeff`
   - 期望 out：`8ea2b7ca516745bfeafc49904b496089`
5. **AES-128 CBC 加密 4 块**（SP 800-38A F.2.1）
   - key：`2b7e151628aed2a6abf7158809cf4f3c`；IV：`000102030405060708090a0b0c0d0e0f`
   - in：`6bc1bee22e409f96e93d7e117393172a ae2d8a571e03ac9c9eb76fac45af8e51 30c81c46a35ce411e5fbc1191a0a52ef f69f2445df4f9b17ad2b417be66c3710`
   - 期望 out：`7649abac8119b246cee98e9b12e9197d 5086cb9b507219ee95db113a917678b2 73bed6b8e3c1743b7116e69e22229516 3ff1caa1681fac09120eca307586e1a7`
6. **AES-128 CBC 解密**：同 key/IV，in 取上式密文，期望 out 为明文。
7. **AES-192 CBC 加密（首块）**（SP 800-38A F.2.3）
   - key：`8e73b0f7da0e6452c810f32b809079e562f8ead2522c6b7b`；IV 同上
   - in 首块：`6bc1bee22e409f96e93d7e117393172a` → 期望 out：`4f021db243bc633d7178183a9fa071e8`
8. **AES-256 CBC 加密（首块）**（SP 800-38A F.2.5）
   - key：`603deb1015ca71be2b73aef0857d77811f352c073b6108d72d9810a30914dff4`；IV 同上
   - in 首块：`6bc1bee22e409f96e93d7e117393172a` → 期望 out：`f58c4c04d6e5f1ba779eabfb5f7bfbd6`
9. **AES-128 CTR 加密 4 块**（SP 800-38A F.5.1；库对 CTR 无解密方向，DEC 也走加密路径）
   - key：`2b7e151628aed2a6abf7158809cf4f3c`；IV/初值计数器：`f0f1f2f3f4f5f6f7f8f9fafbfcfdfeff`
   - in：`6bc1bee22e409f96e93d7e117393172a ae2d8a571e03ac9c9eb76fac45af8e51 30c81c46a35ce411e5fbc1191a0a52ef f69f2445df4f9b17ad2b417be66c3710`
   - 期望 out：`874d6191b620e3261bef6864990db6ce 9806f66b7970fdff8617187bb9fffdff 5ae4df3edbd5d35e5b4f09020db03eab 1e031dda2fbe03d1792170a0f3009cee`
   - 该向量同时验证库的大端计数器递增（fefdff00 → fefdff01 → fefdff02）。
10. **AES-192 CTR 加密（首块）**（SP 800-38A F.5.3）
    - key：`8e73b0f7da0e6452c810f32b809079e562f8ead2522c6b7b`；初值计数器同上
    - in 首块：`6bc1bee22e409f96e93d7e117393172a` → 期望 out：`1abc932417521ca24f2b0459fe7e6e0b`
11. **AES-256 CTR 加密（首块）**（SP 800-38A F.5.5）
    - key：`603deb1015ca71be2b73aef0857d77811f352c073b6108d72d9810a30914dff4`；初值计数器同上
    - in 首块：`6bc1bee22e409f96e93d7e117393172a` → 期望 out：`601ec313775789a5b7a7f504bbf3d228`
12. **CTR 非整块（字级尾部）**：取 F.5.1 的 key/IV，in 只给 5 个字（20 字节，前 16 字节 + 4 字节），期望前 16 字节同 C1，第 5 字 = C2 低 32-bit 与对应输入字的异或（由库的尾部字级 XOR 决定；具体期望值按小端字序计算）。
13. 边界用例：ECB 下 `inWordLen%4 != 0` 应返回 `AES_Crypto_LengthError`；未 Init 直接 Crypto 应返回 `AES_Crypto_UnInitError`；`keyWordLen=5` 应返回 `AES_Crypto_KeyLengthError`；CBC 下 `iv=NULL` 应返回 `AES_Crypto_ParaNull`。
