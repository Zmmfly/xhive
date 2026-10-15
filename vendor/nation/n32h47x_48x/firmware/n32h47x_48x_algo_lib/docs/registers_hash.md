# N32H47x/48x SAC 加速器 HASH 寄存器与行为文档（逆向）

本文档描述厂商闭源算法库 `n32h47x_48x_algo_lib` v1.2.0 中 HASH 库
（`HASH_Init` / `HASH_Start` / `HASH_Update` / `HASH_Complete` /
`HASH_Close` / 五个 one-shot `XX_Hash` / `HASH_Version`，共 11 个导出
函数）对 SAC（Security Algorithm Coprocessor）外设的实际 MMIO 访问
序列、寄存器位域、算法选择编码、流式状态机、padding 流程、错误码路径
与超时行为。所有结论均给出反汇编或 DWARF 证据出处；无法确认的内容明确
标注"推测"。

## 1. 证据来源

| 证据 | 位置 | 说明 |
| --- | --- | --- |
| GCC 目标文件反汇编 | `lib/n32h47x_48x_hash.a::n32h47x_48x_hash.o`（`.text.HASH_*`） | GNU Arm Embedded 10.2.1，`-Os`，Thumb-2，带重定位。下文引用形如 `GCC HASH_Init+0x2C` |
| GCC 目标文件 DWARF | 同上（`.debug_info` / `.debug_line`） | 函数签名与参数名（`ctx`/`in`/`byteLen`/`out`/`wordLen`/`counter` 等）、`HASH_CTX`(224B) 与 `HASH_ALG`(56B) 布局、`SAC_TypeDef` 寄存器名、`RCC_Module` 布局（AHB1PCLKEN@+0x3C）、厂商源文件 `lib/n32h47x_48x_hash.c` 行号映射（ByteLenPlus1@186、ProcMsgBuf@208、PadMsg1@234、Init@322、Start@361、Update@401、Complete@526、Close@568、SM3_Hash@594、SHA1_Hash@616、SHA224_Hash@637、SHA256_Hash@658、MD5_Hash@679、Version@702） |
| 重定位表 | `readelf -r n32h47x_48x_hash.o` | `HASH_Update` 内两处 `bl` 解析为 **HASH_ProcMsgBuf**（非自递归）；one-shot 的 5 处 `bl` 解析为 HASH_Init/Start/Update/Complete/Close；`HASH_Init+0x94` 字面池指向 `.data.MD5_S` |
| 目标文件数据 | `.data.MD5_K/.MD5_S/.MD5_V/.SHA1_IV/.SHA1_K/.SHA224_IV/.SHA256_IV/.SHA256_K/.SM3_K/.SM3_V` 与 `.rodata.HASH_ALG_*` | 全部常量的逐字节内容（§5、§8），已与新源码编译产物逐字节比对一致 |
| armclang 交叉验证 | `.lib` 成员 `n32h47x_48x_hash_1.o`（节 `i.*`） | 与 GCC 逐行为对照一致（含 PadMsg1 的 MD5 字交换、>55 双块流程、HASH_Version 四个字节）；下文引用形如 `armclang HASH_PadMsg1+0x52` |
| CMSIS 设备头 | `CMSIS/device/n32h47x_48x.h` | `SAC_BASE = 0x4002A000`、`SAC_SRAM_BASE = 0x4002A400`、`RCC_AHB1PCLKEN(+0x3C)`、`RCC_AHB1PCLKEN_SACEN = 0x8000`（位注释 "SAC clock enabled"） |
| API 契约 | `inc/n32h47x_48x_hash.h` | 错误码枚举值、`HASH_CTX`/`HASH_ALG` 布局、`HASH_SEQUENCE_TRUE/FALSE`（本文不改该文件） |
| 公共库 | `lib/n32h47x_48x_common.a` + common 库最终逆向结论（父 Agent 转达） | `CLEAR_ARAM_DONE_CHECK`（轮询 SACCR bit7→0，成功后 RMW 置 SACSR bit0）、`Alg_TimeOut_Counter`（.data 初值 100000）、`BufToFIFO(fifo_addr, data, wordLen)` 与 `FIFOToBuf(buffer, fifo_addr, wordLen)`（**长度为 32 位字数**；BufToFIFO 非对齐源经 16 字节栈缓冲中转） |
| 姊妹库旁证 | `registers_des.md`、aes/sm4 库 | AES 写 SACCR=0xD0、DES=0xD1、SM4=0xD4，用于推断 SACCR 算法位（§4.1） |

## 2. 外设概述

- SAC 基址 `0x4002A000`（GCC `HASH_Init+0x2A` 字面池 `4002a000`；CMSIS `SAC_BASE`）。
- SAC 时钟由 RCC `AHB1PCLKEN`（`0x40021000+0x3C`）bit15（`RCC_AHB1PCLKEN_SACEN=0x8000`）
  控制。DWARF `RCC_Module` 中 +0x3C 成员名即 `AHB1PCLKEN`
  （GCC `HASH_Init+0x2C..+0x32` 读-或-写；`HASH_Close+0x30..+0x36` 清零）。
- SAC 内部 ARAM（算法 RAM）起始 `0x4002A400`（CMSIS `SAC_SRAM_BASE`）。
  HASH 库不直接访问 `0x4002A400`，而是通过 §3 的参数/数据窗口寄存器
  （P1ADR/P2ADR/KADR/IADR/OADR）按字压入/弹出。
- HASH 引擎为逐块压缩模型：软件每次向 IADR 写满 16 个字（64 字节），
  向 SACSFR1 写 bit7 启动一次压缩，轮询 SACSR bit3 等待完成。
- 同一 SAC 被 AES（SACCR=0xD0）、DES（0xD1）、SM4（0xD4）与
  HASH（0xD2/0x92）共用；HASH 与其他算法的低 3 位编码相同差异见 §4.1。

## 3. 寄存器总表（SAC，基址 0x4002A000）

| 偏移 | 地址 | 名称（DWARF `SAC_TypeDef`） | 访问 | 复位值 | 副作用 / 说明 |
| --- | --- | --- | --- | --- | --- |
| 0x00 | 0x4002A000 | `SACCR` | RW | 未知 | 控制寄存器。HASH 初始化整体写入 `HASH_SACCR|0x80`（0xD2/0x92）；bit7 写 1 触发 ARAM clear（自清零，common 轮询）；Close 置 bit8 后清 0x12。详见 §4.1 |
| 0x04 | 0x4002A004 | `SACSR` | RW（部分位写 1 清除） | 未知 | 状态寄存器。bit3 = 单块压缩完成标志（硬件置位，软件 RMW 写 1 清除）；bit0 由 common 库在 ARAM clear 完成后写 1 确认。详见 §4.2 |
| 0x08 | 0x4002A008 | `RESERVED0[]` | — | — | HASH 库未访问 |
| 0x0C | 0x4002A00C | `SACSFR1` | RW | 未知 | HASH 运行配置：低 5 位为算法编码（`HASH_HASHCTRL`），bit7 = 启动单块压缩。详见 §4.3 |
| 0x10 | 0x4002A010 | `SACSFR2` | RW | 未知 | HASH 库从未访问（DES 等其他库使用） |
| 0x14 | 0x4002A014 | `SACSFR3` | RW | 未知 | 同上 |
| 0x18–0x2C | — | `RESERVED1..6` | — | — | 未访问 |
| 0x30 | 0x4002A030 | `P1ADR` | W（FIFO push） | — | 参数口 1：IV/中间状态上传。`HASH_Start(SEQ_FALSE)` 上传表 IV；save-IV 模式下每次 Update/Complete 前回存 ctx->IV |
| 0x34 | 0x4002A034 | `P2ADR` | W（FIFO push） | — | 参数口 2：仅 MD5 在 `HASH_Init` 上传 16 字旋转量表 MD5_S（GCC `HASH_Init+0x5C..+0x62`，字面池重定位 `.data.MD5_S`） |
| 0x38 | 0x4002A038 | `P3ADR` | W（FIFO push） | — | HASH 库未访问（AES/DES 使用） |
| 0x3C | 0x4002A03C | `KADR` | W（FIFO push） | — | 常数口：`HASH_Init` 上传 K 表（SHA1 4 字 / SHA224/SHA256 64 字 / MD5 64 字 / SM3 2 字） |
| 0x40 | 0x4002A040 | `IADR` | W（FIFO push） | — | 消息输入口：每块写 16 字；padding 零字与长度字也经此写入（含直接 `*IADR = 0` / `*IADR = word` 存储） |
| 0x44 | 0x4002A044 | `OADR` | R（FIFO pop） | — | 摘要/状态输出口：`HASH_Complete` 读 `DigestWordLen` 字作摘要；save-IV 模式 `HASH_Update` 尾部读 `IVLen` 字回存 ctx->IV |

> 名称 `P1ADR/P2ADR/P3ADR/KADR/IADR/OADR` 取自 DWARF `SAC_TypeDef`
> （厂商私有 SAC_Common.h，decl 第 192–212 行左右；`registers_des.md`
> 同源）。SDK 的 CMSIS 头未提供该类型定义，DWARF 是命名的唯一权威来源。
> 各窗口"写一次压入一个 32-bit 字 / 读一次弹出一个字"的 FIFO 语义由
> 使用方式推断（连续写入有序数据、BufToFIFO/FIFOToBuf 的流式用法；
> OADR 读回与 P1ADR 写入的可逆性由 save-IV 状态机要求），标注为**推测**。
> 复位值：库代码从未读回未写位的初始值，无法从二进制推出，留白。

相关 RCC 寄存器：

| 偏移 | 地址 | 名称 | 位 | 说明 |
| --- | --- | --- | --- | --- |
| +0x3C | 0x4002103C | `RCC_AHB1PCLKEN` | bit15 = `RCC_AHB1PCLKEN_SACEN`(0x8000) | `HASH_Init` 置 1（GCC `HASH_Init+0x2C..+0x32`）；`HASH_Update/Complete` 检查（`+0x46..+0x4C` / `+0x36..+0x3C`）；`HASH_Close` 清 0（`+0x30..+0x36`） |

## 4. 位域明细

### 4.1 SACCR（0x4002A000，控制寄存器）

`HASH_Init` 整体写入 `ctx->hashAlg->HASH_SACCR | 0x80`，即
SHA1/SHA224/SHA256/SM3 写 **0x000000D2**、MD5 写 **0x00000092**
（GCC `HASH_Init+0x34..+0x3A`；表内容见 §5）。0xD2/0x92 本身已含
bit7，故 `|0x80` 为冗余写法。

| 位 | 名称（本文命名） | 值/行为 | 证据与确定度 |
| --- | --- | --- | --- |
| [2:0] | `ALGO[2:0]`（推测） | HASH 引擎 = `010`（0xD2&7 = 0x92&7 = 2）。姊妹库：AES=000（0xD0）、DES=001（0xD1）、SM4=100（0xD4） | 四个库 Init 立即数对比。**推测**：低 3 位为 SAC 子引擎选择，仅观测到上述组合 |
| bit1 | `HASH_CFG_A`（暂名） | HASH 恒置 1；与 bit4 共同构成 `(SACCR & 0x12) == 0x12` 的"已配置 HASH"签名，被 Update/Complete/Close 检查 | GCC `HASH_Update+0x50..+0x5A`、`HASH_Complete+0x3E..+0x48`、`HASH_Close+0x0C..+0x16`（`and #18; cmp #18`） |
| bit4 | `HASH_CFG_B`（暂名） | 同上恒置 1（0x12 的组成部分） | 同上；DES 库对应签名为 0x11（bit0|bit4），位选择随算法族不同 |
| bit6 | `HASH_BE`（推测） | SHA1/SHA224/SHA256/SM3 = 1，MD5 = 0。与 §6 的大端/小端字通道分组完全对应（0xD2 有 bit6，0x92 无） | `.rodata.HASH_ALG_*` 立即数。**推测**：bit6 即字通道字节序选择（1=大端 lane），与 SACSFR1 算法编码配合生效；独立作用未在板上分离验证 |
| bit7 | `ARAM_CLR` | 写 1 触发 ARAM clear，随后自清零；common 的 `CLEAR_ARAM_DONE_CHECK()` 轮询此位至 0（最多 Alg_TimeOut_Counter+1 次读），成功后 RMW 置 SACSR bit0，超时返回 2 | GCC `HASH_Init+0x34..+0x42`（写后立即调用）；common `CLEAR_ARAM_DONE_CHECK+0x08..+0x1E`。**推测**：自清零（轮询语义要求）；HASH 库自身从不读此位 |
| bit8 | `CLOSE`（推测） | `HASH_Close` 先 `SACCR |= 0x100` 再 `&= ~0x12`，随后再 ARAM clear + 关时钟 | GCC `HASH_Close+0x18..+0x26`。**推测**：关闭/收尾动作（可能触发 ARAM 或内部状态清除），具体硬件语义未观测 |
| 其余位 | — | 未观察到读写 | — |

### 4.2 SACSR（0x4002A004，状态寄存器）

| 位 | 名称 | 行为 | 证据 |
| --- | --- | --- | --- |
| bit3 | `HASH_DONE` | 单块压缩完成：软件置 SACSFR1 bit7 后轮询此位，`1` 表示完成；完成后软件读回 SACSR 并 `|= 0x08` 写回（读-改-写，即写 1 清除/应答）。等待预算为 `Alg_TimeOut_Counter`（初值 100000）：计数在每次失败轮询后 +1，`Alg_TimeOut_Counter >= counter` 时继续，超出返回 `HASH_TimeOut_ERROR`(0x01044404) | GCC `HASH_ProcMsgBuf+0x1C..+0x3C`、`HASH_Update+0x15C..+0x178`、`HASH_PadMsg1+0x66..+0xD4`/`+0xA0..+0xE4`；armclang `HASH_PadMsg1+0xA2..+0xBE`。写 1 清除的语义为**推测**（RMW 模式与常见 rc_w1 寄存器一致，未观测到读清或条件写差异） |
| bit0 | `CLRDONE_ACK` | common 库在 ARAM clear 完成（SACCR bit7 自清）后 RMW 置 1，作为完成确认 | common `CLEAR_ARAM_DONE_CHECK+0x10..+0x16`；HASH 库不直接访问 |
| 其余位 | — | 未观察到读写（DES 库观测到 bit2=OFNE，与 HASH 无关） | — |

### 4.3 SACSFR1（0x4002A00C，HASH 运行配置/启动寄存器）

`HASH_Init` 在 ARAM clear 完成后整体写入 `ctx->hashAlg->HASH_HASHCTRL`
（GCC `HASH_Init+0x44..+0x4A`），此后每块压缩前 `|= 0x80` 启动。

| 位 | 名称 | 值/行为 | 证据与确定度 |
| --- | --- | --- | --- |
| [4:0] | `HASH_ALG`（推测） | 算法编码（= 表字段 HASH_HASHCTRL）：SHA-1=`0x00`，SHA-224=`0x02`，SHA-256=`0x02`，SM3=`0x0F`，MD5=`0x10`。SHA-224 与 SHA-256 编码相同（SHA-224 = SHA-256 内核 + 不同 IV + 软件截断 7 字） | `.rodata.HASH_ALG_*` 内容 + `HASH_Init+0x44..+0x4A`。编码含义为**推测**（仅观测到写入值；0x0F 与 0x10 的内部位拆分未验证） |
| bit7 | `HASH_START` | 写 1 启动一次单块压缩（硬件对 IADR 中当前 16 字组做压缩并更新状态）；完成后置 SACSR bit3 | `HASH_ProcMsgBuf+0x14..+0x1A` 等全部块处理点；armclang `HASH_PadMsg1+0x8E..+0x9C` |
| 其余位 | — | 未观察到读写 | — |

## 5. 算法参数表（`.rodata.HASH_ALG_*`，各 56 字节）

| 字段（偏移） | SHA1 | SHA224 | SHA256 | MD5 | SM3 | SM3_160† | SM3_192† |
| --- | --- | --- | --- | --- | --- | --- | --- |
| HashAlgID(+0x00) | 0x0004 | 0x000A | 0x000B | 0x000C | 0x0012 | 0x0012 | 0x0012 |
| K 指针(+0x04) | SHA1_K | SHA256_K | SHA256_K | MD5_K | SM3_K | SM3_K | SM3_K |
| KLen 字(+0x08) | 4 | 64 | 64 | 64 | 2 | 2 | 2 |
| IV 指针(+0x0C) | SHA1_IV | SHA224_IV | SHA256_IV | MD5_V | SM3_V | SM3_V | SM3_V |
| IVLen 字(+0x10) | 5 | 8 | 8 | 4 | 8 | 8 | 8 |
| HASH_SACCR(+0x14) | 0xD2 | 0xD2 | 0xD2 | 0x92 | 0xD2 | 0xD2 | 0xD2 |
| HASH_HASHCTRL(+0x18) | 0x00 | 0x02 | 0x02 | 0x10 | 0x0F | 0x0F | 0x0F |
| BlockByteLen(+0x1C) | 64 | 64 | 64 | 64 | 64 | 64 | 64 |
| BlockWordLen(+0x20) | 16 | 16 | 16 | 16 | 16 | 16 | 16 |
| DigestByteLen(+0x24) | 20 | 28 | 32 | 16 | 32 | 20 | 24 |
| DigestWordLen(+0x28) | 5 | 7 | 8 | 4 | 8 | 5 | 6 |
| Cycle(+0x2C) | 6 | 6 | 6 | 6 | 6 | 6 | 6 |
| ByteLenPlus(+0x30) | HASH_ByteLenPlus1（全部相同） | | | | | | |
| PadMsg(+0x34) | HASH_PadMsg1（全部相同） | | | | | | |

† `HASH_ALG_SM3_160/192` 为二进制中导出但公共头未声明、且不被任何
公共函数引用的截断摘要变体（摘要取 SM3 前 20/24 字节）；Init/Start/
Update/Complete 的合法性检查只比较五个公共表指针，SM3_160/192 会被
拒绝（GCC `HASH_Init+0x08..+0x26` 比较链）。

注：`Cycle` 字段实际用途是 `log2(BlockByteLen)`（=6），`HASH_Update`
用它作移位计算完整块数：`cycle = (msgIdx + byteLen) >> Cycle`
（GCC `HASH_Update+0xAE` `lsr.w r8, sl, r8`），并非头文件注释的
"iteration times"（轮数）。消息长度上限由 `HASH_ByteLenPlus1` 检查
`msgByteLen[0] < 0x20000000` 给出，即 2^61 字节（SHA-2 规范消息长度
2^64 bit）。

## 6. 字节序与字通道（word lane）模型

对 `.data` 常量表与 IADR 推送序列的完整交叉验证给出如下一致性模型
（除标注外为二进制事实，模型整体为高置信度**推断**，建议按 §10 板上
确认）：

- 64 位消息长度计数器存于 `msgByteLen[0]=高 32 位, [1]=低 32 位`
  （`HASH_ByteLenPlus1` 在 [1] 上做加法、进位进 [0]）。
- `HASH_PadMsg1` 将其左移 3 位转为位长度后：
  - **SHA-1/SHA-224/SHA-256/SM3**：`ReverseBytesInWord_U32(msgByteLen,
    msgByteLen, 2)` 逐字字节反转，然后按 `[0]`（=BE(高字)）、`[1]`
    （=BE(低字)）顺序推入 IADR —— 大端 64 位长度
    （GCC `HASH_PadMsg1+0xBA..+0xC6`，写入点 `+0x90..+0x96`）。
  - **MD5**：`strd` 交换两字，使 `[0]=低字, [1]=高字`，原样（小端
    本机序）推入 —— 小端 64 位长度，低字在前
    （GCC `HASH_PadMsg1+0x3C` `strd r3, r2`；armclang
    `+0x52..+0x58` 显式交换）。
- `.data` 常量表（LE 字值）：
  - MD5_K = 标准 T 表（0xD76AA478, …）；MD5_V = 标准 MD5 IV
    （0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476）。
  - SHA1_IV/SHA224_IV/SHA256_IV/SHA256_K/SHA1_K/SM3_K/SM3_V 均为
    **对应标准常量的逐字节反转（bswap）**，例：SHA256_IV[0]
    = 0x67E6096A = bswap(0x6A09E667)，SM3_V[0] = 0x6F168073 =
    bswap(0x7380166F)，SHA1_IV = {0x01234567, 0x89ABCDEF,
    0xFEDCBA98, 0x76543210, 0xF0E1D2C3}。
- 消息字：`BufToFIFO(IADR, in, 16)` 原样推送本机（LE 打包）字，无
  软件字节反转（GCC `HASH_Update+0x13E..+0x144`）。
- 综合模型（**推测**）：SAC 的 ARAM 窗口在 SHA/SM3 模式（SACCR bit6=1）
  下按**大端字通道**解释每个写入/读出字——硬件对输入字做一次字节交换，
  故软件必须预存/预反转成 bswap(标准值)（IV、K、长度字皆然；消息字
  LE 打包经硬件交换即得 SHA 要求的 BE 打包 W）；MD5 模式（bit6=0）
  按小端字通道直通（表存标准值、长度低字在前）。OADR 读出的摘要/状态
  字与 P1ADR 写入同通道（save-IV 状态机的读回-再写入要求二者互逆），
  因此 `FIFOToBuf` 原样写入 out 即得标准字节序摘要。此模型解释了所有
  观测编码，但单个硬件数据通路的精确行为需板上向量确认。

## 7. 厂商库实际操作流程

### 7.1 one-shot 路径（SHA1_Hash/SHA224_Hash/SHA256_Hash/MD5_Hash/SM3_Hash）

五个 one-shot 是完全相同的 0x44 字节包装（GCC 各函数 + 重定位表）：
在栈上建一个 224 字节 `HASH_CTX`，`ctx.hashAlg = 对应表`，
`ctx.sequence = HASH_SEQUENCE_FALSE(0x010A5A5A)`，依次调用
`HASH_Init` → `HASH_Start` → `HASH_Update(ctx,in,byteLen)` →
`HASH_Complete(ctx,out)` → `HASH_Close()`，**全部返回值被丢弃，恒返回 0**
（GCC `SM3_Hash+0x14..+0x34` 五个 `bl` 及 `movs r0,#0`）。

完整寄存器序列（含证据）：

1. `HASH_Init`（GCC `HASH_Init+0x28..+0x66`）：
   1. `RCC_AHB1PCLKEN |= 0x8000`（开 SAC 时钟）；
   2. `SACCR = HASH_SACCR | 0x80`（0xD2/0x92，触发 ARAM clear）；
   3. `CLEAR_ARAM_DONE_CHECK()`：轮询 SACCR bit7→0，成功后 SACSR bit0
      置 1；返回 2（超时）则 `HASH_Init` 返回 0x01044404；
   4. `SACSFR1 = HASH_HASHCTRL`（算法编码，整体写）；
   5. `BufToFIFO(KADR, K, KLen)`（K 常数，字数）；
   6. 仅 MD5（`HashAlgID == 0x000C`）：`BufToFIFO(P2ADR, MD5_S, 16)`
      （旋转量表）。
   注：Init **不上传 IV**、**不复位**任何 ctx 计数器。
2. `HASH_Start`（GCC `HASH_Start+0x2A..+0x62`）：
   `SetZero_U32(msgByteLen,4)`；`msgIdx = 0`；
   - `sequence == 0x0105A5A5`（save IV）：`Cpy_U32(ctx->IV, 表IV, IVLen)`
     —— IV 存入 ctx，供多次 Update 间保存/恢复中间状态；
   - `sequence == 0x010A5A5A`：`BufToFIFO(P1ADR, 表IV, IVLen)`
     —— IV 直接下发硬件（one-shot 用此模式）；
   - 其他值 → 返回 0x01044401。
3. `HASH_Update`（见 7.2）。
4. `HASH_Complete`（GCC `HASH_Complete+0x08..+0x90`）：
   1. `memset(栈上 64 字节 tmpBuf, 0)`；
   2. `out == NULL` 或表指针非法 → 返回 **0x01044402**
      （HASH_Update_ERROR；库从不返回 HASH_Complete_ERROR）；
   3. 时钟未开或 `(SACCR & 0x12) != 0x12` → 返回 **0x01044400**；
   4. save-IV 模式：`BufToFIFO(P1ADR, ctx->IV, IVLen)`（恢复中间状态）；
   5. 调用 `PadMsg(ctx)`（见 7.3），**返回值被忽略**（疑似缺陷：padding
      超时后仍读摘要）；
   6. out 4 字节对齐：`FIFOToBuf(tmpBuf, OADR, DigestWordLen)` +
      `Cpy_U8(out, tmpBuf, DigestWordLen*4)`；不对齐：直接
      `FIFOToBuf(out, OADR, DigestWordLen)`（common 的 FIFOToBuf 支持
      非对齐目标）。
5. `HASH_Close`（GCC `HASH_Close+0x02..+0x3E`）：
   时钟未开或 `(SACCR & 0x12) != 0x12` → 直接返回 0；否则
   `SACCR |= 0x100`；`SACCR &= ~0x12`；`CLEAR_ARAM_DONE_CHECK()`
   超时 → 0x01044404；成功后 `RCC_AHB1PCLKEN &= ~0x8000`，返回 0。

### 7.2 流式路径与状态机（HASH_Update，GCC `HASH_Update+0x0A..+0x1BC`）

前置检查（顺序即返回语义）：
1. `ctx == NULL` 或表指针非法 → `HASH_Update_ERROR`(0x01044402)；
2. `byteLen == 0` → 返回 0（不检查 in）；
3. `in == NULL` → **返回 0 且丢弃数据**（疑似缺陷）；
4. 时钟未开或 `(SACCR & 0x12) != 0x12` → `HASH_Init_ERROR`(0x01044400)。

主流程（局部名取自 DWARF：cycle/begin/remainder/BlockByteLen/i/ret/counter）：
1. `ret = ByteLenPlus(msgByteLen, byteLen)`：非 0 → 原样返回
   （超长时为 `HASH_ByteLenPlus_ERROR`=0x01044403）。
2. `begin = msgIdx + byteLen`；`begin < 64`：仅拷贝入 `msgBuf[msgIdx..]`
   并推进 `msgIdx`，返回 0。
3. 需要成块时：save-IV 模式先 `BufToFIFO(P1ADR, ctx->IV, IVLen)`
   恢复中间状态（`+0x98..+0xAA`）。
4. `cycle = begin >> 6`（完整块数，Cycle=6 作移位）。
5. `msgIdx != 0`：`Cpy_U8(msgBuf+msgIdx, in, 64-msgIdx)` 补满块 →
   `HASH_ProcMsgBuf(ctx)`（msgBuf 16 字 → IADR，SACSFR1|=0x80，等
   SACSR bit3 并应答，`msgIdx=0`）；仅当返回 0x01044404 时中止；
   `cycle--`；`in += (64-msgIdx)`。
6. 循环 `i < cycle` 逐块：
   - `in` 4 字节对齐：`BufToFIFO(IADR, in, 16)`；`SACSFR1 |= 0x80`；
     内联等待 SACSR bit3（同上超时语义）并 RMW 置 bit3；
   - 不对齐：`Cpy_U8(msgBuf, in, 64)` 经 msgBuf 走 `HASH_ProcMsgBuf`；
   - `in += 64`。
7. `msgIdx = begin & 63`；`Cpy_U8(msgBuf, in, msgIdx)`（无条件，0 长度
   也调用）。
8. save-IV 模式：`FIFOToBuf(ctx->IV, OADR, IVLen)` 把最新中间状态读回
   ctx（`+0x100..+0x116`）。
9. 返回 0。

多段 Update（save-IV 模式）状态机：每次调用在成块路径前后成对执行
"OADR→ctx->IV（读回）"与"ctx->IV→P1ADR（恢复）"，硬件每块从最近下发
的 IV 续算 —— 这使单个 SAC 可被多个 HASH_CTX 交错复用。
`HASH_SEQUENCE_FALSE` 模式下无读回，硬件状态连续累积，两个 ctx 交错
使用会互相破坏（one-shot 每次完整重配，故无此问题）。

### 7.3 padding 流程（HASH_PadMsg1，GCC `+0x00..+0xE4`）

1. `wordLen = (msgIdx + 4) >> 2`（覆盖 0x80 所在字所需字数）；
   在 `msgBuf[msgIdx..msgIdx+3]` 写 `0x80,0,0,0`（msgBuf 的 132 字节
   冗余吸收 msgIdx≤63 时的越界 3 字节）。
2. 位长度转换与字节序处理（见 §6）。
3. `msgIdx > 55`（无空间放长度字）：
   `BufToFIFO(IADR, msgBuf, wordLen)` → 补零至 16 字 →
   `SACSFR1|=0x80` → 等 bit3 → 应答；`wordLen = 0`。
4. 最终块：`BufToFIFO(IADR, msgBuf, wordLen)` → 补零至 **14** 字 →
   `*IADR = msgByteLen[0]` → `*IADR = msgByteLen[1]`（长度字直接写）→
   `SACSFR1|=0x80` → 等 bit3 → 应答；成功返回 0，超时返回 0x01044404。

## 8. `.data`/`.rodata` 数据表汇总

| 符号 | 大小 | 含义 |
| --- | --- | --- |
| `MD5_K` | 0x100（64 字） | MD5 T 常数，标准值（小端通道） |
| `MD5_S` | 0x40（16 字） | MD5 旋转量表 {7,12,17,22,5,9,14,20,4,11,16,23,6,10,15,21}，仅 Init 上传 P2ADR |
| `MD5_V` | 0x10（4 字） | MD5 IV，标准值 |
| `SHA1_IV`/`SHA1_K` | 0x14/0x10 | SHA-1 IV/K，bswap(标准值) |
| `SHA224_IV`/`SHA256_IV` | 0x20/0x20 | SHA-224/256 IV，bswap(标准值) |
| `SHA256_K` | 0x100（64 字） | SHA-256 K，bswap(标准值) |
| `SM3_K`/`SM3_V` | 0x8/0x20 | SM3 T/IV，bswap(标准值) |
| `HASH_ALG_*`（7 个） | 各 0x38 | §5 参数表；厂商将 K/S/V 放在**可写 .data**（本仓库重建源码改用 `static const`，行为等价） |

新源码编译产物已与上述厂商字节逐位比对一致（10/10 表匹配）。

## 9. 错误码汇总（实际使用值）

| 值 | 枚举名 | 返回点 |
| --- | --- | --- |
| 0 | *_OK | 全部成功路径 |
| 0x01044400 | HASH_Init_ERROR | Init：ctx NULL/表非法；Update/Complete：时钟未开或 SACCR 签名不符（GCC `HASH_Update+0x17E`、`HASH_Complete+0x96`） |
| 0x01044401 | HASH_Start_ERROR | Start：ctx NULL/表非法/sequence 非法 |
| 0x01044402 | HASH_Update_ERROR | Update：ctx NULL/表非法；**Complete**：out NULL/表非法（厂商复用） |
| 0x01044403 | HASH_ByteLenPlus_ERROR | Update：ByteLenPlus 溢出（消息 ≥ 2^61 字节） |
| 0x01044404 | HASH_TimeOut_ERROR | Init/Update/Complete(padding)/Close：SAC 轮询超时 |
| 0x01044405..0x0104440D | HASH_Complete_ERROR … MD5_Hash_ERROR | **v1.2.0 二进制从不返回**（仅存在于头文件枚举；Complete 用 0x01044402/0x01044400，one-shot 恒返回 0） |

## 10. 不确定点（推测）与板上验证建议

推测项（均有二进制依据，但硬件侧未验证）：
1. SACCR bit6 = 字通道字节序选择（§4.1/§6）；bit7 自清零；bit8 关闭语义。
2. SACSFR1 [4:0] 算法编码含义及 0x0F/0x10 的位拆分（§4.3）。
3. SACSR bit3/bi0 为写 1 清除（RMW 观测，未区分其他清除方式）。
4. P1ADR/KADR/IADR/OADR 的 FIFO 自增与 OADR/P1ADR 通道互逆性（§6）。
5. OADR 摘要字使 `FIFOToBuf` 直写 out 即得标准字节序（§6 模型推论）。
6. SM3_160/192 的摘要 = SM3 前 20/24 字节（表内容推断，公共 API 不可达）。

建议板上测试向量（空串与 "abc" 标准摘要，十六进制，直接比对 one-shot
输出；再以流式 save-IV 模式分段喂入验证状态机）：

| 算法 | 输入 | 期望摘要（十六进制） |
| --- | --- | --- |
| MD5 | "" | d41d8cd98f00b204e9800998ecf8427e |
| MD5 | "abc" | 900150983cd24fb0d6963f7d28e17f72 |
| SHA-1 | "" | da39a3ee5e6b4b0d3255bfef95601890afd80709 |
| SHA-1 | "abc" | a9993e364706816aba3e25717850c26c9cd0d89d |
| SHA-224 | "" | d14a028c2a3a2bc9476102bb288234c415a2b01f828ea62ac5b3e42f |
| SHA-224 | "abc" | 23097d223405d8228642a477bda255b32aadbce4bda0b3f7e36c9da7 |
| SHA-256 | "" | e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855 |
| SHA-256 | "abc" | ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad |
| SM3 | "" | 1ab21d8355cfa17f8e61194831e81a8f22bec8c728fefb747ed035eb5082aa2b |
| SM3 | "abc" | 66c7f0f462eeedd9d1f2d46bdc10e4e24167c4875cf2f7a2297da02b8f4ba8e0 |

边界补充向量：对每种算法测试长度 55/56/63/64/65/119/128（覆盖
padding 单双块切换与块循环）；非对齐输入缓冲（`in&3 != 0` 路径）；
`HASH_Complete` 非对齐 out；save-IV 模式下 1 字节 × N 次分段 Update
与 100 字节一次 Update 结果一致性。
