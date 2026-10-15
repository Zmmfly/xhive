# N32H47x/48x 算法加速器固件逆向文档总览

本目录是对厂商闭源算法库 `n32h47x_48x_algo_lib` v1.2.0 逆向工程的文档集。
逆向对象为随 SDK 发布的静态库原件（不依赖任何板卡运行环境）：

- GCC arm 版：`lib/n32h47x_48x_{common,aes,des,sm4,hash,rng}.a`
  （GNU ARM 10.2.1，`-Os -g -gdwarf-2`，含完整 DWARF 调试信息）。
- armclang 版：同目录 `.lib`（交叉验证用；其中 `startup_n32h487.o` 成员与本
  逆向无关）。

对每个模块，`src/` 下维护与二进制行为对齐的重建源码（`inc/` 公开头文件为
API 契约，一律不改动）。

## 文档索引

| 文档 | 内容 | 覆盖库 |
| --- | --- | --- |
| [registers_common.md](registers_common.md) | SAC 寄存器布局、CLEAR_ARAM_DONE_CHECK 的 MMIO 序列与超时语义、15 个导出符号（BufToFIFO/FIFOToBuf/Cpy/XOR/SetZero/ReverseBytes/Cmp/RandomSort/Alg_TimeOut_Counter）的参数、对齐与返回值语义 | common |
| [registers_aes.md](registers_aes.md) | AES 加速器寄存器与操作序列 | aes |
| [registers_des.md](registers_des.md) | DES/3DES 加速器寄存器与操作序列 | des |
| [registers_sm4.md](registers_sm4.md) | SM4 加速器寄存器与操作序列 | sm4 |
| [registers_hash.md](registers_hash.md) | HASH（SHA1/SHA224/SHA256/MD5/SM3）加速器寄存器与操作序列 | hash |
| [registers_rng.md](registers_rng.md) | RNG 随机数发生器寄存器与操作序列 | rng |

## 算法加速器（SAC）概述

所有算法模块共用一个 AHB 外设 SAC（Security Algorithm Coprocessor，算法
加速器），基址 `SAC_BASE = 0x4002A000`（CMSIS `n32h47x_48x.h`：
`AHBPERIPH_BASE + 0xA000`），其内部 SRAM（ARAM）位于 `SAC_SRAM_BASE =
0x4002A400`。common 库负责对外设的公共访问：超时等待（CLEAR_ARAM_DONE_
CHECK）、FIFO 字搬运（BufToFIFO/FIFOToBuf）与纯内存数据处理；各算法库在
其上编排具体的寄存器序列。

### 已知算法外设地址总表

| 地址 | 寄存器（DWARF `SAC_TypeDef`） | 证据来源 |
| --- | --- | --- |
| 0x4002A000 | SACCR：bit7 = ARAM 清除 busy（common 轮询） | common 库字面量 + 轮询行为；CMSIS `SAC_BASE` |
| 0x4002A004 | SACSR：bit0 = 清除完成应答（common RMW 置位） | common 库 RMW 行为 |
| 0x4002A00C/10/14 | SACSFR1/2/3（功能寄存器，common 未用） | DWARF 布局 |
| 0x4002A030 | P1ADR（参数/数据端口 1，推测） | DWARF 布局；aes/des/sm4/hash 反汇编字面量 |
| 0x4002A034 | P2ADR（参数/数据端口 2，推测） | DWARF 布局；des/sm4/hash 反汇编字面量 |
| 0x4002A038 | P3ADR（参数/数据端口 3，推测） | DWARF 布局；des/sm4 反汇编字面量 |
| 0x4002A03C | KADR（Key 端口，推测） | DWARF 布局；aes/des/sm4/hash 反汇编字面量 |
| 0x4002A040 | IADR（Input 端口，推测） | DWARF 布局；hash 反汇编字面量 |
| 0x4002A044 | OADR（Output 端口，推测） | DWARF 布局；hash 反汇编字面量 |
| 0x4002A400 | SAC SRAM（ARAM） | CMSIS `SAC_SRAM_BASE` |

各寄存器位段与操作时序详见对应文档；表中"推测"项的判定依据见
[registers_common.md](registers_common.md) §8。

## Alg_TimeOut_Counter 超时机制

`uint32_t Alg_TimeOut_Counter = 100000;`（common 库 .data 段导出变量）是
全部算法库共用的**轮询次数预算**（不是时间）：

- 以 `CLEAR_ARAM_DONE_CHECK` 为例：设变量值为 N，则状态寄存器最多被读
  N+1 次；若忙标志一直不清 0，返回 `Time_Out(2)`；成功路径在忙标志清 0 后
  对 SACSR bit0 做读-改-写置位并返回 0。
- N=0 时允许 1 次忙观察后立即超时；默认 100000 次。
- GCC 版对象在函数入口将该变量快照一次，armclang 版每轮循环重读；若应用
  需要在等待期间动态调整该值，两个发行版的实际行为不同（详见
  [registers_common.md](registers_common.md) §7）。
- 超时只表示"轮询预算耗尽"，不代表外设复位：超时路径不写任何确认位，
  硬件可能仍在忙，调用方须自行恢复后再启动新操作。

## 证据与写作约定

- 每条硬件相关结论标注 `函数名+偏移`（对象内偏移），DWARF 与反汇编冲突时
  以反汇编为准。
- 无法从二进制确证的内容一律标注 **推测**；各文档末尾汇总不确定点。
- 逆向仅基于发布工件（.a/.lib + DWARF + CMSIS 头交叉引用），未做实板验证；
  所有"行为"均指指令级语义，未经硬件运行确认。
