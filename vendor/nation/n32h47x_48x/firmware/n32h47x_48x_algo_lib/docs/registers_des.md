# N32H47x/48x SAC 加速器 DES/TDES 寄存器与行为文档（逆向）

本文档描述厂商闭源算法库 `n32h47x_48x_algo_lib` v1.2.0 中 DES/TDES 库
（`DES_Init` / `DES_Crypto` / `DES_Close` / `DES_Version`）对 SAC
（Security Algorithm Coprocessor）外设的实际 MMIO 访问序列、寄存器位域、
错误码路径与超时行为。所有结论均给出反汇编或 DWARF 证据出处；无法确认的
内容明确标注"推测"。

## 1. 证据来源

| 证据 | 位置 | 说明 |
| --- | --- | --- |
| GCC 目标文件反汇编 | `lib/n32h47x_48x_des.a::n32h47x_48x_des.o`（`.text.DES_*`） | GNU Arm Embedded 10.2.1（`.comment`），`-Os`（`Tag_ABI_optimization_goals: Aggressive Size`），Thumb-2。下文引用形如 `GCC DES_Init+0x04` |
| GCC 目标文件 DWARF | 同上（`.debug_info` / `.debug_line`） | `SAC_TypeDef` 结构（寄存器名与偏移）、`DES_PARM` 布局、`enum DES` 常量值、BufToFIFO 调用点参数、厂商源文件 `lib/n32h47x_48x_des.c` 的行号映射 |
| armclang 交叉验证 | `.lib` 成员 `n32h47x_48x_des_1.o`（节 `i.DES_*`） | 与 GCC 逐行为对照，两者逻辑一致；下文引用形如 `armclang DES_Init+0x06` |
| CMSIS 设备头 | `CMSIS/device/n32h47x_48x.h` | `SAC_BASE = AHBPERIPH_BASE + 0xA000 = 0x4002A000`、`SAC_SRAM_BASE = 0x4002A400`、`RCC_AHB1PCLKEN(+0x3C)`、`RCC_AHB1PCLKEN_SACEN = 0x8000`（位定义注释 "SAC clock enabled"） |
| API 契约 | `inc/n32h47x_48x_des.h` | 错误码枚举值、`DES_PARM` 布局、宏值（本文不改该文件） |
| 公共库依赖 | `inc/n32h47x_48x_algo_common.h` + `common.disasm` + common 库最终逆向结论（父 Agent 转达） | `CLEAR_ARAM_DONE_CHECK`（轮询 SACCR bit7→0，成功后 RMW 置 SACSR bit0）、`Alg_TimeOut_Counter`、`BufToFIFO(fifo_addr, data, wordLen)`（见 §7） |
| 姊妹库旁证 | `aes.disasm`、`sm4.disasm` | AES 写 SACCR=0xD0、SM4 写 SACCR=0xD4，用于推断算法选择位 |

GCC 版目标文件 `.data`/`.bss` 段大小为 0，无任何库内静态数据；所有常量
（MMIO 地址、错误码、0xD1）均为立即数或 `.text` 字面池（`DES_Init` 字面池
`+0xf8..+0x11c`：`40021000/4002a000/4002a034/4002a030/4002a038/4002a03c/
5a5a5a60/5a5a5a5c/5a5a5a5b/5a5a5a5e`；`DES_Crypto` 字面池含 `Alg_TimeOut_Counter`
重定位 `R_ARM_ABS32`）。

## 2. 外设概述

- SAC 基址 `0x4002A000`（GCC `DES_Init+0x06` 字面池 `4002a000`；CMSIS `SAC_BASE`）。
- SAC 时钟由 RCC `AHB1PCLKEN`（`0x40021000+0x3C`）bit15（`RCC_AHB1PCLKEN_SACEN=0x8000`）控制。
  注：DWARF `RCC_Module` 中 +0x3C 成员名为 `AHB1PCLKEN`（旧版逆向源码误称 AHB2ENR）。
- SAC 内部 ARAM（算法 RAM）位于 `0x4002A400`（CMSIS `SAC_SRAM_BASE`），由
  `CLEAR_ARAM_DONE_CHECK()` 清理（common 库职责）。
- 同一 SAC 被 AES（SACCR=0xD0）、SM4（SACCR=0xD4）、HASH 库共用，算法通过 SACCR 低 3 位选择（推测，见 §4.1）。

## 3. 寄存器总表（SAC，基址 0x4002A000）

| 偏移 | 地址 | 名称（DWARF `SAC_TypeDef`） | 访问 | 复位值 | 副作用 / 说明 |
| --- | --- | --- | --- | --- | --- |
| 0x00 | 0x4002A000 | `SACCR` | RW | 未知 | 控制寄存器。位 7 写 1 触发 ARAM clear（自清零，由 common 轮询）；位 8 在 Close 中置 1。详见 §4.1 |
| 0x04 | 0x4002A004 | `SACSR` | RW（部分位 rc_w1 类） | 未知 | 状态寄存器。bit2=OFNE（输出 FIFO 非空），读观测、写 1 清除（见 §4.2）；bit0 由 common 库在 ARAM 完成后写 1 |
| 0x08 | 0x4002A008 | `RESERVED0[]` | — | — | DWARF 为数组保留，DES 库未访问 |
| 0x0C | 0x4002A00C | `SACSFR1` | RW | 未知（Init 写 0） | DES/TDES 运行配置（链接模式/密钥长度/方向/启动）。详见 §4.3 |
| 0x10 | 0x4002A010 | `SACSFR2` | RW | 未知 | DES 库从未访问；其他算法库用途未知 |
| 0x14 | 0x4002A014 | `SACSFR3` | RW | 未知 | 同上 |
| 0x18–0x2C | — | `RESERVED1..6` | — | — | 未访问 |
| 0x30 | 0x4002A030 | `P1ADR` | W（FIFO push） | — | 参数口 1：DES_Init 在 CBC 模式下写入 IV（2 字）。写一次压入一个 32-bit 字 |
| 0x34 | 0x4002A034 | `P2ADR` | W（FIFO push） | — | 参数口 2：所有密钥模式下写入 key words[1:0]（2 字） |
| 0x38 | 0x4002A038 | `P3ADR` | W（FIFO push） | — | 参数口 3：TDES_2KEY/TDES_3KEY 写入 key words[3:2]（2 字） |
| 0x3C | 0x4002A03C | `KADR` | W（FIFO push） | — | 密钥口：TDES_3KEY 写入 key words[5:4]（2 字） |
| 0x40 | 0x4002A040 | `IADR` | W（FIFO push） | — | 数据输入 FIFO：DES_Crypto 每块写入 2 字 |
| 0x44 | 0x4002A044 | `OADR` | R（FIFO pop） | — | 数据输出 FIFO：DES_Crypto 每块读出 2 字，读一次弹出一个字 |

> 名称 `P1ADR/P2ADR/P3ADR/KADR/IADR/OADR` 取自 DWARF `SAC_TypeDef`（decl 于厂商
> 外设头，第 192–212 行）。SDK 的 CMSIS 头引用了 `SAC_Module` 但未提供该类型
> 定义，故 DWARF 是寄存器命名的唯一权威来源。
> 复位值：库代码从未读回未写位的初始值，无法从二进制推出，留白。

相关 RCC 寄存器：

| 偏移 | 地址 | 名称 | 位 | 说明 |
| --- | --- | --- | --- | --- |
| +0x3C | 0x4002103C | `RCC_AHB1PCLKEN` | bit15 = `RCC_AHB1PCLKEN_SACEN`(0x8000) | `DES_Init` 置 1（GCC `DES_Init+0x04..+0x0c`，读-或-写）；`DES_Close` 清 0（`DES_Close+0x2a..+0x30`） |

## 4. 位域明细

### 4.1 SACCR（0x4002A000，控制寄存器）

| 位 | 名称（本文命名） | 值/行为 | 证据与确定度 |
| --- | --- | --- | --- |
| [2:0] | `ALGO[2:0]`（推测） | DES=001（0xD1），AES=000（0xD0），SM4=100（0xD4） | 三个库 Init 的立即数对比（GCC `DES_Init+0x0e` movs #209；`aes.disasm AES_Init+0x10` movs #208；`sm4.disasm SM4_Init+0x10` movs #212）。**推测**：低 3 位是算法选择，具体编码只观测到这三种组合 |
| bit4 | `CFG`（暂名） | 恒为 1（三个库均置位）；与 bit0 共同构成 `& 0x11 == 0x11` 的"已初始化"签名 | GCC `DES_Crypto+0x26..+0x2e`、`DES_Close+0x0c..+0x14` |
| bit6 | 未知 | DES/AES/SM4 的 Init 值均置 1 | 立即数 0xD0/0xD1/0xD4 都含 bit6。用途不明 |
| bit7 | `ARAM_CLR`（推测） | 写 1 启动 ARAM 清理，随后被 `CLEAR_ARAM_DONE_CHECK()` 轮询至 0 | Init 写 0xD1 后立即调用 common 轮询（GCC `DES_Init+0x0e..+0x18`）。**推测**：该位自清零（轮询语义要求），DES 库自身从不读它 |
| bit8 | `CLOSE`（推测） | `DES_Close` 中先 `|= 0x100` 再 `&= ~0x11`，随后再执行一次 ARAM clear + 关时钟 | GCC `DES_Close+0x16..+0x24`。**推测**：关闭/收尾动作，具体硬件语义未观测 |
| 其余位 | — | 未观察到读写 | — |

### 4.2 SACSR（0x4002A004，状态寄存器）

| 位 | 名称 | 行为 | 证据 |
| --- | --- | --- | --- |
| bit0 | ARAM done 应答位（common 库职责） | `CLEAR_ARAM_DONE_CHECK()` 成功后写 1（RMW） | common 库语义（父 Agent 已确认）；DES 库不直接访问 |
| bit2 | `OFNE`（输出 FIFO 非空） | 硬件完成一块后置 1；DES_Crypto 轮询该位，随后读-改-写回 1 清除，再从 OADR 连读 2 字 | GCC `DES_Crypto+0x56`（`ldr r8,[r3,#4]; tst r8,#4`）、`+0x60..+0x66`（`orr #4` 写回）、`+0x68..+0x72`（读 OADR 两次） |
| 其余位 | — | 未观察 | — |

> "写 1 清除"是根据写回序列的推测；也可能硬件在 OADR 弹空后自动清零，
> 库的写回只是冗余。无法从二进制区分，板上验证时可用逻辑分析仪/循环读确认。

### 4.3 SACSFR1（0x4002A00C，DES/TDES 运行配置）

`DES_Init` 在 ARAM 清理完成后先整写 0（GCC `DES_Init+0x1c..+0x20`），
随后按下表 RMW 置位：

| 位 | 名称 | 值 | 证据 |
| --- | --- | --- | --- |
| bit1 | `CBC` | 0x02：CBC 链接（否则 ECB） | `DES_Init+0x6c..+0x78`（CBC 分支 `orr r3,#2`） |
| bit4 | `TDES_3KEY`（推测命名） | 0x10：仅 TDES_3KEY 组合中出现 | 3-key 写 0x30 = 0x20\|0x10（`DES_Init+0xd0..+0xd6`），2-key 只写 0x20。**推测**：bit5 表示 TDES，bit4 表示第三把密钥 |
| bit5 | `TDES`（推测命名） | 0x20：TDES_2KEY / TDES_3KEY 均置位 | `DES_Init+0xa0..+0xa6`（`orr #32`）、`+0xd0..+0xd6`（`orr #48`） |
| bit6 | `DECRYPT` | 0x40：解密方向（`En_De == 0x44444444` 时置位；加密 0x33333333 不置位） | `DES_Init+0x52..+0x62`（条件 `itttt eq` + `orr #64`） |
| bit7 | `GO` | 0x80：启动一个 64-bit 块运算。每块由 DES_Crypto 置位一次 | `DES_Crypto+0x48..+0x4e`（`orr #128`）。库从不清除该位；是否自清零未观测（推测自动，否则无法解释多块连续运算） |
| 其余位 | — | 未观察 | — |

密钥长度编码汇总（`DES_Init` switch，GCC `+0x42..+0xd8`）：

| keyMode | BufToFIFO 写入口（各 2 字） | SACSFR1 置位 |
| --- | --- | --- |
| `DES_KEY`(0x55555555) | P2ADR ← key[0:1] | 无 |
| `TDES_2KEY`(0x66666666) | P2ADR ← key[0:1]；P3ADR ← key[2:3] | `\| 0x20` |
| `TDES_3KEY`(0x77777777) | P2ADR ← key[0:1]；P3ADR ← key[2:3]；KADR ← key[4:5] | `\| 0x30` |
| 其它 | — | 返回 `DES_Crypto_KeyError`(0x5A5A5A5E) |

CBC 模式额外动作：`SACSFR1 |= 0x02` 后 `BufToFIFO(P1ADR=0x4002A030, iv, 2)`
（GCC `DES_Init+0x6c..+0x7c`）。注意 IV 与密钥写入端口在旧版逆向源码中被
错位（见 §8）。

### 4.4 FIFO 端口（P1ADR/P2ADR/P3ADR/KADR/IADR/OADR）

- 每个端口是 32-bit push/pop 窗口：`BufToFIFO`/直写每写一次压入一个字；
  OADR 每读一次弹出一个字（GCC `DES_Crypto+0x68..+0x72` 连续两次读
  `[r3,#0x44]` 得到一块的两个字）。
- 字节序：按小端 32-bit 字写入（`BufToFIFO` 为 `ldr/str` 字循环，见 §7）。
  硬件在 64-bit 块内的位/字节排列无法从库代码确定（板上向量验证，见 §9）。

## 5. 错误码（与 `inc/n32h47x_48x_des.h` 枚举一致）

| 枚举值 | 数值 | 返回点（GCC 反汇编地址） |
| --- | --- | --- |
| `DES_Crypto_OK` / `DES_Init_OK` | 0x00000000 | `DES_Init+0x80`、`DES_Crypto+0x7a` |
| `DES_Crypto_ModeError` | 0x5A5A5A5A | `DES_Init+0xf0`（Mode 非 ECB/CBC） |
| `DES_Crypto_EnOrDeError` | 0x5A5A5A5B | `DES_Init+0xe2`（En_De 非 0x33333333/0x44444444） |
| `DES_Crypto_ParaNull` | 0x5A5A5A5C | `DES_Init+0xde`（key==NULL 或 CBC 且 iv==NULL）；`DES_Crypto+0x88`（in/out==NULL） |
| `DES_Crypto_LengthError` | 0x5A5A5A5D | `DES_Crypto+0x8c`（inWordLen==0 或为奇数） |
| `DES_Crypto_KeyError` | 0x5A5A5A5E | `DES_Init+0xe6`（keyMode 三者之外） |
| `DES_Crypto_UnInitError` | 0x5A5A5A5F | `DES_Crypto+0x90`（RCC 时钟位未开或 SACCR 签名≠0x11） |
| `DES_TimeOutError` | 0x5A5A5A60 | `DES_Init+0xda`（ARAM 清理超时）；`DES_Crypto+0x84`（块轮询超时） |

DWARF `enum DES`（decl 行 53–64）与 armclang 版（错误码由 0x5A5A5A60 基值
±偏移合成，如 `subs #4`→ParaNull、`adds #1`→LengthError）完全一致。

参数校验顺序（DES_Init，两套工具链一致）：
ARAM 清理 → SACSFR1=0 → key NULL → CBC iv NULL → Mode 合法 → En_De 合法 →
keyMode 分发。即 key 为 NULL 且 Mode 非法时返回 ParaNull（先查 key）。

DES_Crypto 校验顺序：in/out NULL → 长度 → 硬件就绪（不检查 iv；iv 由 Init 保证）。

## 6. 厂商库操作流程

### 6.1 DES_Init（`.text.DES_Init`，0x120 字节，DWARF decl 行 40）

```text
1. RCC_AHB1PCLKEN |= 0x8000                // GCC +0x04..+0x0c (读[0x4002103C] 或 0x8000 写回)
2. SACCR = 0xD1                            // +0x0e..+0x10
3. if (CLEAR_ARAM_DONE_CHECK() == 2/*Time_Out*/)
       return DES_TimeOutError;            // +0x14..+0x1a → +0xda
4. SACSFR1 = 0;                            // +0x1c..+0x20
5. if (parm->key == NULL)                  // parm+0x0C
       return DES_Crypto_ParaNull;         // +0x22 → +0xde
6. if (parm->Mode == DES_CBC && parm->iv == NULL)   // parm+0x18 / parm+0x04
       return DES_Crypto_ParaNull;
7. if (parm->Mode != DES_ECB && != DES_CBC)
       return DES_Crypto_ModeError;        // +0xea..+0xf4
8. if (parm->En_De != DES_ENC && != DES_DEC)
       return DES_Crypto_EnOrDeError;      // +0x34..+0x40 → +0xe2
9. switch (parm->keyMode) { … 见 §4.3 表 }  // +0x42..+0xd8
10. if (parm->En_De == DES_DEC) SACSFR1 |= 0x40;   // +0x52..+0x62
11. if (parm->Mode == DES_CBC) { SACSFR1 |= 0x02;  // +0x64..+0x78
        BufToFIFO(0x4002A030, parm->iv, 2); }      // +0x7c
12. return DES_Init_OK;                             // +0x80
```

DWARF 行号佐证：厂商源 `lib/n32h47x_48x_des.c` 行 44（RCC 使能）、行 45–47
（ARAM 检查/超时）、行 50–54（SACSFR1=0 与 key/iv 判空）、行 57–59（Mode）、
行 62–64（En_De）、行 67–88（keyMode switch）、行 91–99（方向与 CBC）、
行 102–103（返回）。

### 6.2 DES_Crypto（`.text.DES_Crypto`，0xB0 字节，DWARF decl 行 112，局部变量 `inWordLen`、`i`、`counter`）

```text
1. if (parm->in == NULL || parm->out == NULL)      // +0x04..+0x0e
       return DES_Crypto_ParaNull;                 // → +0x88
2. inWordLen = parm->inWordLen;                    // parm+0x10
   if (inWordLen == 0 || (inWordLen & 1))          // +0x12..+0x1a
       return DES_Crypto_LengthError;              // → +0x8c
3. if (!(RCC_AHB1PCLKEN & 0x8000)                  // +0x1c..+0x22
       || (SACCR & 0x11) != 0x11)                  // +0x24..+0x2e
       return DES_Crypto_UnInitError;              // → +0x90
4. for (i = 0; i < inWordLen; i += 2) {            // +0x30..+0x3a, +0x74..+0x78
       IADR = in[i];  IADR = in[i+1];              // +0x3c..+0x46  (0x4002A040 ×2)
       SACSFR1 |= 0x80;  /* GO */                  // +0x48..+0x4e
       counter = 0;                                // +0x50..+0x54
       while (!(SACSR & 0x04)) {                   // +0x56..+0x5e
           counter++;
           if (counter > Alg_TimeOut_Counter)      // +0x7e..+0x82 (r9=*Alg_TimeOut_Counter)
               return DES_TimeOutError;            // → +0x84
       }
       SACSR |= 0x04;                              // +0x60..+0x66
       out[i] = OADR;  out[i+1] = OADR;            // +0x68..+0x72  (0x4002A044 ×2)
   }
5. return DES_Crypto_OK;                           // +0x7a
```

超时细节：GCC 将 `Alg_TimeOut_Counter` 的值在每个块开始时读入 `r9` 一次；
轮询为 `cmp r9,counter; bcs`（counter ≤ 计数值则继续），即允许的轮询次数
约为 `Alg_TimeOut_Counter + 1`。armclang 版每次轮询都重读全局变量
（`armclang DES_Crypto+0x6c..+0x72`），两者语义等价（该变量非原子修改场景下）。
复位值 100000（0x186A0）来自 common 库。

### 6.3 DES_Close（`.text.DES_Crypto` 之后，0x3C 字节，DWARF decl 行 162）

```text
1. if (!(RCC_AHB1PCLKEN & 0x8000)) return;         // +0x04..+0x08
2. if ((SACCR & 0x11) != 0x11) return;             // +0x0c..+0x14
3. SACCR |= 0x100;                                 // +0x16..+0x1c
4. SACCR &= ~0x11;                                 // +0x1e..+0x24
5. CLEAR_ARAM_DONE_CHECK();  // 返回值被忽略      // +0x26
6. RCC_AHB1PCLKEN &= ~0x8000;                      // +0x2a..+0x30
```

### 6.4 DES_Version（0x1C 字节，DWARF decl 行 187）

```text
*type = 0x05;  *customer = 0x00;
date[0] = 0x19(25);  date[1] = 0x04(4);  date[2] = 0x12(18);
*version = 0x11;    // 1.1
```

按头文件约定（"bytes are 18,9,13 → September 13, 2018"，即 年/月/日），
date = {25, 4, 18} 表示 2025-04-18；GCC 与 armclang 两个版本字节完全一致。
（另一种 读法 25 日/4 月/2018 年 与头文件示例矛盾，不采用。）

## 7. 与 common 库的协作点

| 调用点 | 传参 | 证据 |
| --- | --- | --- |
| `DES_Init` → `CLEAR_ARAM_DONE_CHECK()` | 无参；返回 2 (`Time_Out`) 视为失败 | 重定位 `.rel.text.DES_Init +0x14 R_ARM_THM_CALL`；common DWARF `enum{...Time_Out=2}` |
| `DES_Close` → `CLEAR_ARAM_DONE_CHECK()` | 无参；返回值忽略 | 重定位 `.rel.text.DES_Close +0x26` |
| `DES_Init` → `BufToFIFO(fifo_addr, data, wordLen)` ×6 | IV→`0x4002A030`(P1ADR)；key1→`0x4002A034`(P2ADR)；key2→`0x4002A038`(P3ADR)；key3→`0x4002A03C`(KADR)；wordLen 恒为 **2（32-bit 字，即 8 字节）** | DWARF `DW_TAG_GNU_call_site_parameter`：`DW_OP_const4u 0x4002A034/0x4002A030/0x4002A038/0x4002A03C` 与 `DW_OP_lit2`；重定位 6×`R_ARM_THM_CALL BufToFIFO` |
| `DES_Crypto` → `Alg_TimeOut_Counter` | 每块超时上界 | 重定位 `.rel.text.DES_Crypto +0x9c R_ARM_ABS32 Alg_TimeOut_Counter` |

关于 `BufToFIFO` 的长度单位（common 库最终逆向结论确认）：第三参数为
**32 位字数 wordLen**（DWARF 参数名即 wordLen）；common 库反汇编中块循环为
4 字一组（`lsrs r7,r2,#2` 计组数、`and r2,r4,#3` 计剩余**字**数、循环体
`ldr r3,[r1],#4; str r3,[fifo]` 每次写一个 32-bit 字），共写入恰好 wordLen
个字；非对齐路径经 16 字节栈缓冲 `Cpy_U8` 中转（无递归），结束后
`SetZero_U32(tmp,4)` 清擦。AES 库将 `AES_PARM.keyWordLen`（头文件注释
"length(by word) of key"，合法值 4/6/8）原样传给 `BufToFIFO`，DES 传 2
（= 64-bit 密钥/IV），两个库交叉印证该单位。

另注（common 库职责，仅记录）：`FIFOToBuf(buffer, fifo_addr, wordLen)` 的
参数顺序与 `BufToFIFO` 不对称——目标缓冲区为第 1 参数、FIFO 地址为第 2
参数；DES 库不使用该函数。

## 8. 旧版逆向源码（修正前）与 v1.2.0 的行为差异

对 `src/n32h47x_48x_des.c`（本次已修正）：

1. **RCC 寄存器名与位**：旧码称 `RCC_AHB2ENR`；实际是 `RCC_AHB1PCLKEN`
   （+0x3C）bit15。位值 0x8000 相同，地址相同，行为等价但命名误导。
2. **Init 清零寄存器错**：旧码在 ARAM 检查后写 `CR(+0x00)=0`，会清掉
   SACCR 的 0x11 签名，使随后的 `DES_Crypto` 必然返回 UnInitError。
   v1.2.0 清的是 `SACSFR1(+0x0C)=0`。
3. **密钥/IV FIFO 端口整体错位一格**：旧码单 DES key→0x30、2KEY 0x30/0x34、
   3KEY 0x30/0x34/0x38、IV→0x3C；v1.2.0 为 key1→0x34、key2→0x38、key3→0x3C、
   IV→0x30。旧码单 DES 实际把密钥写进了 IV 口。
4. **GO 位写错寄存器**：旧码置 `CR(+0x00) |= 0x80`（会重新触发 ARAM 清理
   序列）；v1.2.0 置 `SACSFR1(+0x0C) |= 0x80`。
5. **数据 FIFO 偏移错**：旧码 DIN=+0x08/DOUT=+0x10；v1.2.0 输入 IADR=+0x40、
   输出 OADR=+0x44。
6. **超时语义缺失**：旧码 `WaitForDESOperationComplete(0)` 以传入的 0 为上界，
   且从不引用 `Alg_TimeOut_Counter`；v1.2.0 用 `counter > Alg_TimeOut_Counter`
   判超时（全局变量初值 100000）。
7. **轮询后清标志的顺序**：旧码在等待前/后写 SR 的方式与 v1.2.0 的
   `SACSR |= 0x04`（写回 1）不一致；已对齐。
8. 旧码自带的 `*_rev` 错误码宏与头文件枚举重复；已改用枚举常量（数值经
   DWARF/反汇编核对一致）。

已记录、未修改的疑似缺陷（厂商 v1.2.0 自身）：

- `DES_Init` 不检查 `parm` 指针本身为 NULL 的情况（两套工具链一致，直接解引用）。
- `DES_Crypto` 以字为单位处理 in/out，未检查 `in`/`out` 缓冲长度与别名越界。
- `DES_Close` 忽略 `CLEAR_ARAM_DONE_CHECK()` 的超时返回值，直接关时钟。
- `DES_Crypto` 单块超时后不复位 `SACSFR1.GO`，直接返回；硬件状态未知。

## 9. 建议板上测试向量（公开标准/参考实现生成）

以下期望值由主机 `openssl enc`（legacy provider，`-nopad`）生成，第 1 组
同时与公开文献中的经典 DES KAT（J. Orlin Grabbe, "The DES Algorithm
Illustrated"）一致。注意：`DES_PARM` 的 in/iv/key/out 均为 `uint32_t*`，
每 64-bit 块为 2 个小端 32-bit 字；硬件在块内的字节/位序尚未从二进制确认
（推测与常规密码加速器一致，按向量字节序即 MSB 字节在前 组字，
见文末说明）。

**T1 单 DES ECB 加密**（keyMode=DES_KEY, Mode=DES_ECB, En_De=DES_ENC）

- key = `13 34 57 79 9B BC DF F1`
- IV：无（ECB，iv 可为 NULL）
- in (8 B) = `01 23 45 67 89 AB CD EF`
- 期望 out = `85 E8 13 54 0F 0A B4 05`

**T2 单 DES ECB 解密**（keyMode=DES_KEY, Mode=DES_ECB, En_De=DES_DEC）

- 同 T1 密钥；in = T1 的密文；期望 out = `01 23 45 67 89 AB CD EF`。

**T3 单 DES CBC 加密**（keyMode=DES_KEY, Mode=DES_CBC, En_De=DES_ENC）

- key = `13 34 57 79 9B BC DF F1`；IV = `11 11 11 11 11 11 11 11`
- in (8 B) = `01 23 45 67 89 AB CD EF`
- 期望 out = `56 E6 BD 45 88 6B 1D DA`

**T4 3DES(EDE) 3KEY ECB 加密**（keyMode=TDES_3KEY, Mode=DES_ECB, En_De=DES_ENC）

- key = `01 23 45 67 89 AB CD EF` `23 45 67 89 AB CD EF 01` `45 67 89 AB CD EF 01 23`
- in (24 B) = `"now is the time for all "` = `6E 6F 77 20 69 73 20 74 68 65 20 74 69 6D 65 20 66 6F 72 20 61 6C 6C 20`
- 期望 out = `F3 FB 48 0D 28 E5 80 BF 43 62 76 0C C1 3B A7 DA FF 55 C5 F8 0F AA AC 45`

**T5 3DES(EDE) 3KEY ECB 解密**（keyMode=TDES_3KEY, En_De=DES_DEC）

- 同 T4 密钥；in = T4 密文；期望 out = T4 明文。

**T6 3DES(EDE) 2KEY ECB 加密**（keyMode=TDES_2KEY, Mode=DES_ECB, En_De=DES_ENC）

- key = `01 23 45 67 89 AB CD EF` `23 45 67 89 AB CD EF 01`
- in 同 T4；期望 out = `E7 6E 5E 81 0B D8 93 B4 5D 27 31 A8 D9 B4 01 62 3D D3 FC 69 A0 8C C6 D9`

**T7 3DES 3KEY CBC 加密**（keyMode=TDES_3KEY, Mode=DES_CBC, En_De=DES_ENC）

- key 同 T4；IV = `07 07 07 07 07 07 07 07`
- in 同 T4；**标准**期望 out = `8D F0 79 BE 6B AF 8C B2 E1 F8 45 40 B6 3A F6 90 61 A7 8B 4F A8 9D 28 0C`
- ⚠ 实测警告：见 §10——实板 TDES_3KEY+CBC 块间链接与标准不符（块 1 正确，块 2 起链值非上一块密文）；单钥 CBC 不受影响。

**T8 退化一致性**（数学恒等式，无需 openssl）

- keyMode=TDES_3KEY（或 2KEY），K1=K2=K3=`13 34 57 79 9B BC DF F1`，
  ECB 加密 T1 明文 → 期望与 T1 单 DES 结果完全一致
  （EDE 等密钥退化为单 DES）。若不符，优先怀疑密钥端口映射或 TDES 位。

**字序探针**：若 T1 结果不符，将 key/in/out 每个字按大端解释重试
（即 word[0]=0x13345779, word[1]=0x9BBCDFF1 改为 0x01234567/0x89ABCDEF
对应的两种打包顺序分别测试）。此为唯一无法从库二进制确认的自由度。

## 10. 实板验证结果与 TDES+CBC 链接异常（N32H474 实板，2026-10）

在 N32H474 实板（110 轮投票）上对本仓库重建源码的验证结果：

| 组合 | 结果 |
| --- | --- |
| 单钥 DES ECB（厂商 ALGO_DEMO word 向量，16 words） | PASS |
| 单钥 DES CBC（厂商 word 向量，8 块链） | PASS |
| TDES_3KEY ECB、TDES_2KEY ECB（标准向量 LE 打包） | PASS |
| 各模式 roundtrip | PASS |
| **TDES_3KEY + CBC** | **FAIL（本节）** |

### 10.1 异常现象（实测数据）

测试向量：key = `0123456789abcdef` `23456789abcdef01` `456789abcdef0123`，
IV = `0^8`，pt = P1||P2，P1 = P2 = `0123456789abcdef`（LE 打包同 ECB 测试）：

- 板上 C1 = `f2afd84ee809e2b5`，与标准 3DES-CBC 第一块一致（正确）。
- 板上 C2 = `326310a3c41bd2fd`，标准期望 `db7860f547d16c54`（主机 openssl 双工具一致）。
- 精确复现（openssl 逐阶段验证）：板上 C2 = `E_K3(D_K2(E_K1(P2 ⊕ X)))`，
  其中链值 X = `76648e7d6ea89b5f`，既非 C1 也非其字交换，也不是任何可识别的
  DES 中间量（见 10.3）。
- 解密方向自洽：板解 C2 时同样以 X 作为链值（`D1(E2(D3(C2))) ⊕ X = P2`），
  故 roundtrip 通过。

### 10.2 库侧排查结论（厂商二进制定点复查）

对 `n32h47x_48x_des.a` 的 TDES_3KEY+CBC 路径逐指令复查：

1. `DES_Init` 3KEY 分支精确时序：`BufToFIFO(P2ADR←key[1:0])` →
   `BufToFIFO(P3ADR←key[3:2])` → `BufToFIFO(KADR←key[5:4])` →
   `SACSFR1 |= 0x30` → 公共尾（`En_De==DEC` 时 `SACSFR1 |= 0x40`）→ CBC 分支
   `SACSFR1 |= 0x02` → `BufToFIFO(P1ADR←iv, 2)`。**IV 写入在 TDES 位置位之后、
   全部密钥之后；3KEY 下无任何额外端口写入**（GCC `DES_Init+0xaa..+0x7c`；
   6 个 `BufToFIFO` 重定位全部对账）。
2. `DES_Crypto` 循环中与 Mode/keyMode 相关的分支 MMIO：**无**。循环体仅访问
   IADR(+0x40，写 2 字)、SACSFR1(+0x0C，`|= 0x80`)、SACSR(+0x04，轮询/写 1 清
   bit2)、OADR(+0x44，读 2 字)。
3. 2KEY+CBC 与 3KEY+CBC 的 Init 路径结构完全相同（同一公共尾，仅密钥端口数与
   SACSFR1 位不同）；单钥 CBC 无 TDES 位且实测通过。→ 若 3KEY+CBC 异常，
   2KEY+CBC 存在同样风险，**尚未实测**。
4. 本仓库重建源码与厂商二进制在该路径上指令序列一致（同字面池、同 RMW 顺序），
   无遗漏指令或错端口。→ **异常不可能由库代码差异引起，属硬件级行为**（推测：
   硅片在 TDES+CBC 下块间链寄存器更新了非标准值，或需要某个库未触及的
   未文档化配置）。

### 10.3 链值 X 的排除清单（openssl + 纯 Python DES 全部不符）

对 X = `76648e7d6ea89b5f` 已排除：C1/字交换/字节反转；K1/K2/K3 任一单阶段
E/D 作用于 P、IV、stage1/2 输出、C1（含 ws/brev 变体）；任意两阶段组合；
2KEY 风格混合阶段组合（E1D2E1/E2D3E2/E3D2E3/E1D3E1 及正逆序）；完整解密
方向组合；纯 Python DES 内部状态（st1/st2/st3 的 L16||R16、R16||L16 即
pre-FP、post-IP）。X 的代数来源未定，但不影响"硬件以 X 链接、确定性可复现"
的结论。

### 10.4 厂商 ALGO_DEMO 权威 word 向量（供上板复测）

来源：`Nations.N32H47x_48x_Library.1.2.0/projects/n32h47x_48x_EVAL/examples/
ALGO/ALGO_DEMO/src/algo_demo.c` 的 `TDES_3Key_test()` / `TDES_2Key_test()`。
word→64-bit 块打包：每块两字按小端拼字节（源码注释自带 BE 十六进制互证）。
**openssl 验证：demo 的 2KEY/3KEY CBC 期望值均为标准 EDE CBC 链接（链值 =
上一块密文）**——与实板行为矛盾，需上板裁决（见 10.5）。

3KEY CBC（keyMode=TDES_3KEY, iv=iv1, Mode=DES_CBC, En_De=DES_ENC，
期望值 openssl 8/8 块一致）：

```c
uint32_t in1[16] = {0x3C7EB08D,0xAFD2FDE9,0x22245D10,0x148AE53D,0xC70F11D1,0x0813FEDF,
                    0xED8A71D7,0xA66B2FAA,0x137DAC5A,0x9A7850D6,0xFDE9C4AB,0xC1C6856E,
                    0x05CDB663,0xF7D812E4,0x86341DEB,0xBA52B237};
uint32_t key1[6] = {0x675BE5D2,0x1641A6AD,0x14531A6B,0xEBFA006E,0x90DFD0CD,0x2D029B93};
uint32_t iv1[2]  = {0xB5CC3A62,0xC96EF050};
uint32_t TDES_CBC_EN[16] = {0x048BD8AD,0xF98F2C51,0x5F6FD563,0xA26A1038,
                            0x8017FC81,0xBBD5AF4C,0x0A7AEEFF,0xB7D428A1,
                            0x316E31F7,0xD8F283E1,0xDDD4395F,0x8076C2D0,
                            0x0434D1E9,0xD1A94D4D,0xFF3E3B5E,0x77C93116};
uint32_t TDES_CBC_DE[16] = {0xD24C9D38,0xAB82EA98,0xEC4AAF78,0x8DB239A7,
                            0xD0565899,0xA4615EDD,0x78EF88CC,0x16B472C3,
                            0x573F4CD7,0x45910A7C,0x874D72AE,0x5E1D01CA,
                            0x1374E950,0x56502FB2,0x4A32593B,0xE0F51246};
```

2KEY CBC（keyMode=TDES_2KEY，期望值 openssl 8/8 块一致，标准 EDE2-CBC）：

```c
uint32_t key1[4] = {0x81F08C18,0x5C6BE38C,0x4D6A6563,0xFF220031};
uint32_t iv1[2]  = {0xB5CC3A62,0xC96EF050};
uint32_t in1[16] = { /* 同上 in1 */ };
uint32_t TDES_CBC_EN[16] = {0x3723A485,0x3E2EEB10,0x9E5434C4,0x2692C8FD,
                            0x978D5743,0x10CBCFD7,0x873A396C,0xD9CF6AEB,
                            0x5C8953FC,0xD62F3744,0xDE2D0B60,0x1DA22B35,
                            0x00793D6F,0x543CD424,0x833BE660,0x05703F52};
uint32_t TDES_CBC_DE[16] = {0xED617A1E,0xBFDACE87,0x1FCAFD57,0x8D3ECA85,
                            0x72154F73,0xF84F987F,0xE8AABC7B,0xEFB3677F,
                            0xC5F7CC4C,0x9F3AD2C8,0x40779B72,0x00D7F205,
                            0xF1A8B424,0x9A389EA2,0x3EEC58F4,0x1546667E};
```

注：demo 中 CBC_DE 子项是"加密得密文→解密比对明文"的自洽用法；
TDES_CBC_DE 数组是解密输出期望（即明文），不是另一组密文。

### 10.5 建议的裁决实验（板上）

1. 烧录厂商原库 + 直接跑 `TDES_3Key_test()`：若 flag3 失败 → 硬件怪癖实锤
   （厂商库同样非标准）；若通过 → 对比测试环境（打包/调用序列）找差异。
2. 用 12.1 的最小向量实测 TDES_2KEY+CBC（键 = 前两把），验证同容是否异常。
3. 4 块探针：IV=0，P||P||P||P（3KEY+CBC 加密），报出 C1..C4；反解每块
   链值 X2..X4，判断非标准更新函数 Xᵢ = G(Cᵢ₋₁) 是否固定可循，并提供样本
   作进一步代数识别。

## 11. 未确认点汇总（均标"推测"）

| 项 | 推测内容 | 依据 |
| --- | --- | --- |
| SACCR[2:0] | 算法选择：AES=0, DES=1, SM4=4 | 三库 Init 立即数 0xD0/0xD1/0xD4 |
| SACCR bit6 | 恒置 1，用途未知 | 三库 Init 值均含该位 |
| SACCR bit7 | ARAM 清理触发、自清零 | Init 写入后立即轮询该位到 0 |
| SACCR bit8 | Close 收尾动作（复位/flush） | 仅 Close 观测到 `|= 0x100` |
| SACSFR1 bit4/bit5 | bit5=TDES、bit4=第 3 把密钥 | 2KEY=0x20、3KEY=0x30 的组合 |
| SACSFR1 bit7 (GO) | 完成后自清零 | 库从不清除却支持多块连续运算 |
| SACSR bit2 清除方式 | 写 1 清除（rc_w1） | 库的 RMW 写回序列；也可能是读清 |
| 块内字节序 | 向量字节序 = MSB 字节在前 | 密码加速器惯例；T1/T8 板上确认（已证实） |
| TDES+CBC 块间链值 | 3KEY+CBC 实板块 2 起链值非上一块密文（§10）；代数来源未定；2KEY+CBC 未测 | 实板数据 + 库侧逐指令复查（无库代码差异） |
| 复位值 | 所有寄存器复位值未知 | 库从不读回未写状态 |

## 12. 版本信息

`DES_Version` 返回：type=0x05、customer=0x00、date={0x19,0x04,0x12}
（按头文件约定为 2025-04-18）、version=0x11（即 1.1）。GCC 与 armclang
两个发行版字节一致（GCC `DES_Version+0x02..+0x18`；armclang 同偏移）。

---
生成于 DES 库逆向任务；分析中间产物位于 `/tmp/des-re/`（含 DWARF dump 与
行号表）。`src/n32h47x_48x_des.c` 已按本文档对齐 v1.2.0 行为。
