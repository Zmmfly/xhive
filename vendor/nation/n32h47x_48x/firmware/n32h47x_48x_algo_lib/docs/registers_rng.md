# N32H47x/48x 算法加速器（SAC）RNG 寄存器与厂商库行为逆向文档

依据 `n32h47x_48x_algo_lib/lib/n32h47x_48x_rng.a`（GCC arm，带 DWARF）逆向整理。
术语保留英文（SAC、SACCR、TRNG、PRNG 等），正文中文。
本文与 `registers_aes.md` 同源同风格（同一 SAC 外设，RNG 视角）。

## 1. 范围与证据源

- 逆向对象：`lib/n32h47x_48x_rng.a`（成员 `n32h47x_48x_rng.o`，GCC arm，DWARF2）。
  - 编译器（DWARF `DW_AT_producer`）：`GNU C17 10.2.1 20201103 (release) -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -march=armv7e-m -g -gdwarf-2 -Os -ffunction-sections -fdata-sections`。
  - 源文件（DWARF）：`../../../../../../firmware/n32h47x_48x_algo_lib/lib/n32h47x_48x_rng.c`，
    comp_dir `E:\…\Nations.N32H47x_48x_Library.0.8.0\projects\n32h47x_48x_EVAL\examples\ALGO\ALGO_DEMO\GCC`（与 AES 对象同一构建树）。
  - 函数节（`-ffunction-sections`，无内部 static 函数）：
    `GetTrueRand_U32`（0x238 字节，源行 74–198）、`GetPseudoRand_U32`（0x130，源行 209–312 附近）、`RNG_Version`（0x1c）。
- 交叉验证：armclang 版 `n32h47x_48x_rng.lib` 成员 `n32h47x_48x_rng_1.o`（`startup_n32h487.o` 无关，忽略）。
  关键常量与流程全部一致：SACCR=0x16、SACSFR1 写序列（0x02/0x100/0x80 与 0x01）、SACSR 0x800/0x80、
  种子双写 SACSFR2、熵检查 16/8/4/2 级、错误码 0x7a9db86c/0x311ecf50/0x7a9db86e/0x7a9db86d、
  `upseedcount` u16 回绕 >1000→0、`RNG_Version` 字节 1/0/25/3/19/16。armclang 以
  `literal 0x7a9db86c + 2`、`+ 1` 的方式复用超时/攻击码字面量，语义与 GCC 独立字面量一致。
- 外部依赖（common 库，另一 Agent 负责，直接采信）：
  - `uint32_t Alg_TimeOut_Counter`（初值 100000）；
  - `Cpy_U32(dst, src, wordLen)`：按 32-bit 字拷贝；
  - `SetZero_U32(dst, wordLen)`：按字清零；
  - `memset`（libc）。
  RNG 库**不使用** `BufToFIFO`/`CLEAR_ARAM_DONE_CHECK`（`rng.a` 未定义符号仅上列 4 个），
  数据口访问全部为本库内联 MMIO。
- 引用约定：`GetTrueRand_U32+0x26` 指该函数节内相对偏移（`objdump -d` 地址）；"源行 N"指 DWARF
  `decodedline` 映射到的厂商源码行号。
- 冲突处理原则：DWARF 与反汇编冲突时以反汇编实际行为为准。本文所有寄存器访问均直接来自反汇编指令。

## 2. 模块概述

- RNG 真随机/伪随机功能位于 **SAC**（DWARF 类型名 `SAC_TypeDef`，decl 于厂商 SAC 头文件行 192–212，
  结构体 72 字节）上，基址 **0x4002A000**（`GetTrueRand_U32+0x224`、`GetPseudoRand_U32+0x114` 字面量）。
  与 CMSIS `n32h47x_48x.h` 的 `SAC_BASE = AHBPERIPH_BASE(0x40020000) + 0xA000` 一致；`SAC_SRAM_BASE`
  （0x4002A400）RNG 库未使用。
- SAC 是多算法共享控制器：AES 库写 `SACCR=0xD0`、DES `0xD1`、SM4 `0xD4`（见 `registers_aes.md`），
  RNG 两条路径均写 `SACCR=0x16`。对照推测 bits[3:0] 为算法选择（RNG=0b0110；旁证：AES=0、DES=1、SM4=4），
  bit4 含义未知。TRNG 与 PRNG 的区分不在 SACCR，而在 SACSFR1 的初始化写法（见 §4.3）。
- 时钟/配置三步（两条路径相同，顺序固定）：
  1. `RCC->AHBPCLKEN`（0x40021018）`|= 0x10`——CMSIS `RCC_AHBPCLKEN_RNGCEN`（RNGC clock enable）；
  2. `RCC->CFG3`（0x4002100C）`|= 0x04`——CMSIS `RCC_CFG3_TRNG1MEN`（TRNG 1M entropy clock enable）；
  3. `RCC->AHB1PCLKEN`（0x4002103C）`|= 0x8000`——CMSIS `RCC_AHB1PCLKEN_SACEN`（SAC clock enable）。
  三步均为读-改-写、不清除、无检查（`GetTrueRand_U32+0x26..0x3c`、`GetPseudoRand_U32+0x1c..0x30`，源行 86–90 / 234–238）。
- 复位值：全部 SAC 寄存器复位值在库中无证据，一律未标注。

## 3. 寄存器总表（RNG 库视角）

| 偏移 | 地址 | 名称（DWARF） | 访问 | 复位值 | 作用 / 副作用 |
| --- | --- | --- | --- | --- | --- |
| 0x00 | 0x4002A000 | SACCR | RW | 未知 | 算法/模式选择；RNG 库整体写 0x16，返回前整体写回保存值 |
| 0x04 | 0x4002A004 | SACSR | RW | 未知 | 命令位：bit11 FLUSH、bit7 START（RNG 库只写不读） |
| 0x08 | — | RESERVED0 | — | — | 未使用 |
| 0x0C | 0x4002A00C | SACSFR1 | RW | 未知 | RNG 状态/触发：读 bit7 为 busy；写 bit7 触发一轮采样；TRNG 初始化置 bit1/bit8；PRNG 装载种子置 bit0 |
| 0x10 | 0x4002A010 | SACSFR2 | W（RNG 视角） | 未知 | 种子写入口：连续两次 32-bit 写装载 64-bit 种子 |
| 0x14 | 0x4002A014 | SACSFR3 | R（RNG 视角） | 未知 | 随机数据读出口：每轮 kick+START 后读一次 |
| 0x18–0x2C | — | RESERVED1–6 | — | — | 未使用 |
| 0x30–0x44 | — | P1ADR/P2ADR/P3ADR/KADR/IADR/OADR | — | — | 0x4002A400 SRAM 模式地址指针；RNG 库从不访问 |

## 4. 寄存器位域

### 4.1 SACCR（0x4002A000）

| 位 | 名称 | 说明 |
| --- | --- | --- |
| 3:0 | 算法选择（推测） | 仅观察到整体写 0x16 → 0b0110。与 AES(0)/DES(1)/SM4(4) 编码并存，推测为 RNG 算法号 |
| 4 | 未知 | 0x16 中该位为 1；TRNG/PRNG 相同，不参与两路径区分 |
| 31:5 | 未观察 | 无证据 |

### 4.2 SACSR（0x4002A004）

| 位 | 名称 | 说明 |
| --- | --- | --- |
| 7 | START | 每次 kick 完成后读-改-写置 1，随后立即读 SACSFR3；RNG 库从不读 SACSR（`+0x80`、`+0xa8` 等） |
| 11 | FLUSH | 仅 TRNG 初始化时置 1 一次，随后轮询 SACSFR1 busy 直至 0（源行 101–107） |
| 其他 | 未观察 | AES 库将 bit0/bit1 用作 W1C 完成标志（见 registers_aes.md），RNG 路径不触碰 |

### 4.3 SACSFR1（0x4002A00C）

| 位 | 名称 | 说明 |
| --- | --- | --- |
| 7 (0x80) | busy/触发（读写双义） | 读 1 = 一轮采样进行中；写 1 = 触发（kick）新一轮采样。所有轮询均为"等待读值变 0" |
| 0 (0x01) | PRNG 种子装载触发（推测自清零命令位） | PRNG 路径在写完 SACSFR2 种子前置 1（`GetPseudoRand_U32+0x64`，源行 265） |
| 1 (0x02) | TRNG 配置（未知） | 仅 TRNG 初始化置 1（源行 96） |
| 8 (0x100) | TRNG 配置（未知） | 仅 TRNG 初始化置 1（源行 97） |
| 其他 | 未观察 | — |

TRNG 初始化净效果：`SACSFR1 = 0 → 0x02 → 0x102 → 0x182`（源行 95–98）；
PRNG 初始化净效果：`SACSFR1 = 0 → 0x01`（源行 244、265）。库返回前整体写回进入时的保存值。

### 4.4 SACSFR2（0x4002A010）与 SACSFR3（0x4002A014）

SACSFR2：`GetPseudoRand_U32+0x70/+0x76` 连续两次写 `seedbuf[0]`、`seedbuf[1]`（源行 267–268）。
两次写同一地址而非 0x10/0x14 分立寄存器，故按"移位/FIFO 式种子口"理解：先写的字为低 32 位还是
高 32 位无证据（未观察到回读），标注未知。

SACSFR3：唯一数据出口。每轮 `kick(SACSFR1|=0x80) → 等 busy=0 → SACSR|=0x80(START) → 读 SACSFR3`
产出一个 32-bit 随机字。TRNG/PRNG 两路径的取数动作完全相同。

## 5. 厂商库实际操作流程

### 5.1 单字取数节拍（两路径共用）

```
SACSFR1 |= 0x80;                        /* kick（触发一轮采样） */
while (SACSFR1 & 0x80) {                /* 等待 busy 清零 */
    if (++counter > Alg_TimeOut_Counter)
        return RNG_TimeOutError;        /* 0x7A9DB86E，错误路径不恢复寄存器 */
}
SACSR   |= 0x80;                        /* START */
word = SACSFR3;                         /* 读出随机字 */
```

GCC 与 armclang 的计数比较结构略有差异（`cmp r0,r2/bcs` vs `bls`），均为"计数值超过
`Alg_TimeOut_Counter` 即报错"，即最多允许 `Alg_TimeOut_Counter + 1` 次轮询。

### 5.2 GetTrueRand_U32(rand, wordLen) —— TRNG 路径

1. `memset(tempBuf[16], 0, 64)`（栈上 16 字缓冲，源行 74）。
2. 参数校验：`rand == NULL → ADDRNULL(0x7A9DB86C)`（源行 78）；`wordLen == 0 → LENError(0x311ECF50)`（源行 82）。
3. RCC 三步时钟（§2，源行 86–90）。
4. 保存 `SACCR`、`SACSFR1` 到局部（DWARF 名 `rRNGCTRL_back`、`rSACCRback`）；写 `SACCR = 0x16`；
   `SACSFR1 = 0`（源行 92–95）。
5. `SACSFR1 |= 0x02 | 0x100 | 0x80`（净 0x182，源行 96–98）；`SACSR |= 0x800`（FLUSH）；
   等 `SACSFR1` busy 清零（超时→`RNG_TimeOutError`）；`SACSR |= 0x80`（源行 101–110）。
6. **首次调用健康检查**（`counterInner == 0` 时执行一次，源行 111–180）：
   - 16 轮取数填 `tempBuf[0..15]`；若 `tempBuf[15] != 0` → 通过；
   - 否则 `SetZero_U32(tempBuf, 8)`，8 轮取数填 `tempBuf[0..7]`；若 `tempBuf[7] != 0` → 通过；
   - 否则 `SetZero_U32(tempBuf, 4)`，4 轮取数填 `tempBuf[0..3]`；若 `tempBuf[3] != 0` → 通过；
   - 否则 `SetZero_U32(tempBuf, 2)`，再 2 轮 **仅 kick/等待/START、不读 SACSFR3**（GCC `+0x1a0..+0x1ee`、
     armclang `+0x1c8..+0x206` 均无 `[#20]` 访问）；随后判 `tempBuf[1] == 0` → 返回攻击码
     `0x7A9DB86D`（源行 160–175）。
   - 通过后 `counterInner = 1`，本次及后续调用跳过健康检查（源行 180）。
   - **注意（厂商缺陷，保持对齐不改）**：第 4 级循环没有数据读回，`tempBuf[1]` 必为 `SetZero_U32`
     刚写入的 0，该判定恒真——即一旦前 3 级（合计 28 字）全零，必然返回攻击码。
7. `wordLen` 轮取数写入 `rand[i]`（源行 182–193）。
8. 恢复：先 `SACSFR1 = 保存值`，再 `SACCR = 保存值`（源行 195–196，顺序固定）；返回 `RNG_OK(0x5A5A5A5A)`。
   **所有错误返回（超时/攻击）都不恢复寄存器**，SAC 停留在 RNG 模式。

### 5.3 GetPseudoRand_U32(rand, wordLen, seed) —— PRNG 路径

1. 参数校验同上（源行 215–219）。
2. RCC 三步时钟；保存 `SACCR`/`SACSFR1`；`SACCR = 0x16`（与 TRNG 相同）；`SACSFR1 = 0`（源行 234–244）。
   **不做** FLUSH、不写 0x02/0x100。
3. 种子决策（源行 247–260，`upseedcount` 为 u16 静态变量）：
   - `upseedcount == 0 && seed == NULL`：调 `GetTrueRand_U32(seedbuf, 2)` 现场取 TRNG 种子；
     **仅当返回值等于 `RNG_TimeOutError` 时透传返回**（`+0x4e..+0x56` 与 armclang `literal+2` 技巧），
     其他错误（含攻击码）被忽略、继续执行（seedbuf 保持原值）；
   - `seed != NULL`（无论 `upseedcount`）：`Cpy_U32(seedbuf, seed, 2)` 显式重播种；
   - `upseedcount != 0 && seed == NULL`：直接复用内部 `seedbuf`。
4. `SACSFR1 |= 0x01`（源行 265）；`SACSFR2 = seedbuf[0]; SACSFR2 = seedbuf[1];`（源行 267–268）。
5. `upseedcount = (uint16_t)(upseedcount + 1); if (upseedcount > 1000) upseedcount = 0;`
   （`ldrh/strh/uxth`，u16 语义，`+0x78..+0x88`，源行 270–271）。
6. `wordLen` 轮取数写 `rand[i]`（§5.1 节拍，源行 273 起）。
7. 再取 2 轮，`seedbuf[j] ^= SACSFR3`（j=0,1）作为下次默认种子（`+0xb8..+0xe6`，源行 284 附近）。
8. 恢复 `SACSFR1`→`SACCR`，返回 `RNG_OK`。错误路径不恢复；步骤 3 的超时早退同样不恢复。

### 5.4 RNG_Version(type, customer, date, version)

无条件直写四个输出（`+0x02..+0x18`，两工具链一致）：`*type = 1`、`*customer = 0`、
`date = {25, 3, 19}`（2025-03-19）、`*version = 16 (0x10)`。按头文件注释"0x12 即 v1.2"的编码，
0x10 表示 **v1.0**——与库包 v1.2.0 版本号不符，记录为事实。无任何 NULL 检查（传 NULL 即硬错误）。

## 6. 错误码对照（重要）

| 名称 | 公开头文件 `inc/n32h47x_48x_rng.h` | 库二进制实际返回 | 库内 DWARF 枚举 |
| --- | --- | --- | --- |
| RNG_OK | 0x5A5A5A5A | 0x5A5A5A5A | 0x5a5a5a5a |
| LENError | 0x311ECF50 | 0x311ECF50 | 0x311ecf50 |
| ADDRNULL | 0x7A9DB86C | 0x7A9DB86C | 0x7a9db86c |
| RNGATTACKED | **0x7A9DB86F** | **0x7A9DB86D** | **0x7a9db86d** |
| RNG_TimeOutError | 0x7A9DB86E | 0x7A9DB86E | 0x7a9db86e |

即：**库二进制实际返回 0x7A9DB86D**。库内 DWARF 枚举与反汇编字面量（`GetTrueRand_U32+0x22c`、
armclang `literal 0x7a9db86c + 1`）互相印证。厂商原包的枚举为自增式
（`ADDRNULL = 0x7A9DB86C` 后接 `RNGATTACKED, RNG_TimeOutError`），按 C 枚举规则正确展开即为
0x7A9DB86D/0x7A9DB86E；SDK 中 `inc/n32h47x_48x_rng.h` 曾被旧适配手工展开为
`RNGATTACKED = 0x7A9DB86F`（偏移一位），与库行为不一致，**本次已将 inc/ 头文件修正为
0x7A9DB86D**，重建源码直接使用头文件枚举 `RNGATTACKED`，两者现已一致。

## 7. 库内状态与数据

| 符号 | 节 | 大小 | 初值 | 用途 |
| --- | --- | --- | --- | --- |
| `seedbuf` | .data（GCC `.data.seedbuf.0`；armclang 偏移 8） | 8 B | `0x17B03B21, 0x346963FD`（小端实测） | PRNG 默认 64-bit 种子；每次调用末尾 XOR 再播种 |
| `upseedcount` | GCC .bss / armclang .data | 2 B（u16） | 0 | TRNG 取种仅在第 1 次（==0）执行；>1000 回绕 0 |
| `counterInner` | GCC .bss / armclang .data | 4 B | 0 | TRNG 首调健康检查只做一次（==0 判定，完成后置 1） |

- 状态跨调用延续：`GetPseudoRand_U32(NULL seed)` 第 2 次起复用 XOR 更新过的 `seedbuf`；
  `GetTrueRand_U32` 第 2 次起跳过健康检查。
- 两函数均无重入/并发保护；错误路径不恢复 SACCR/SACSFR1，也不回滚上述状态。
- 修改 `Alg_TimeOut_Counter` 可全局调整所有轮询上限（每轮独立计数，非总预算）。

## 8. 不确定点（推测，均注明依据）

1. SACCR=0x16 的位义：仅"整体写值"有证据；bits[3:0]=算法选择为跨库类比推测（AES/DES/SM4 文档旁证）。
2. SACSFR1 bit0/bit1/bit8 的确切硬件含义：只有写法证据（bit0 与写种子伴生、bit1/bit8 仅 TRNG 初始化），
   "种子装载触发/TRNG 采样使能"为推测。
3. SACSFR2 两次写入的高低字顺序：无回读证据，未知。
4. SACSFR3 读是否弹 FIFO、START 是否产下一拍：库内 kick→等→START→读 的节拍固定，但硬件内部
   流水关系无证据，按黑盒记录。
5. busy 位写 1 触发采样的自清零行为：由"写 1 后立即轮询等 0"的用法反推（否则首次即退出），推测成立。
6. 第 4 级熵检查恒判攻击：两工具链均无该级数据读回，判定恒真；推测为厂商源码漏写取数语句（缺陷），
   本文按实际行为记录。
7. armclang 将 `counterInner`/`upseedcount` 放 .data、GCC 放 .bss：仅节归属差异，初值与语义一致
   （armclang .data 内容为全 0，见其 symtab 偏移布局）。

## 9. 板上验证建议（无固定测试向量）

1. **版本一致性**：`RNG_Version` 应返回 type=1、customer=0、date={25,3,19}、version=0x10。
2. **TRNG 连续性**：上电后首次 `GetTrueRand_U32(buf, N)`（N≥1024）应返回 RNG_OK；样本非全零、
   非恒定；1/0 位计数比应在 N*32 的一半附近（|ones−N*16| 约几个 σ，σ=√(N*32)/2）；
   相邻两次调用输出不同（第 2 次跳过健康检查仍应正常）。
3. **PRNG 确定性**：同一种子 `{s0,s1}`、复位后两次 `GetPseudoRand_U32(buf, N, seed)` 输出应完全一致；
   换种子输出应改变。`seed=NULL` 首调（TRNG 取种）两次输出应不同。
4. **攻击路径**（可选，破坏性/依赖环境）：仅当 TRNG 持续输出全零（如熵源时钟未起振）时才会触发
   0x7A9DB86D；正常板无法自然触发，可用逻辑分析仪抓 SACSFR1/SACSR 时序验证节拍，或
   在 HAL 层 mock 寄存器做主机单测验证返回值 0x7A9DB86D 而非头文件的 0x7A9DB86F。
5. **超时路径**：把 `Alg_TimeOut_Counter` 置 0 或 1 并在调试器下冻结 SAC 时钟（清除 RNGCEN 后再调
   用应能在健康检查或取数轮询处返回 0x7A9DB86E）。
6. **寄存器恢复**：调用前后 dump SACCR/SACSFR1，成功路径应恢复原值；人为制造超时后 SACCR 应保持
   0x16（不恢复），验证与库一致。
