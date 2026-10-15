# N32H47x/48x SM4 算法加速器寄存器逆向文档（v1.2.0）

本文档描述 Nations N32H47x/48x 算法加速器外设中 SM4 部分的寄存器行为，依据厂商闭源库
`lib/n32h47x_48x_sm4.a`（v1.2.0，GCC arm-none-eabi 10.2.1 `-Os -gdwarf-2`，含 DWARF）
的反汇编与调试信息逆向，并用同目录 armclang 版 `.lib`（成员 `n32h47x_48x_sm4_1.o`）
交叉验证。除标明"推测/采信"外，所有结论均有反汇编证据；DWARF 与反汇编冲突时以反汇编为准。

## 1. 证据源与方法

| 证据 | 位置 |
| --- | --- |
| GCC 版反汇编 | `/tmp/xhive-re-algo/gcc/sm4.disasm`（4 个函数各自独立 section，偏移从 0 起） |
| GCC 版段/数据 | `/tmp/xhive-re-algo/gcc/sm4.sections`（含 `.data.SM4_CK`、`.data.SM4_FK` 内容） |
| GCC 版符号表 | `/tmp/xhive-re-algo/gcc/sm4.symtab` |
| armclang 交叉验证 | `/tmp/xhive-re-algo/armclang/sm4/n32h47x_48x_sm4_1.o.disasm` |
| DWARF（info/decodedline） | 从 `.a` 提取的 `n32h47x_48x_sm4.o`，工具 `arm-none-eabi-readelf --debug-dump=info/decodedline` |
| 公共库协作函数 | `/tmp/xhive-re-algo/gcc/common.disasm`（BufToFIFO 等） |
| API 契约 | `inc/n32h47x_48x_sm4.h`（错误码与 `SM4_PARM` 布局，勿改） |

DWARF 关键信息：

- 厂商源文件 `firmware/n32h47x_48x_algo_lib/lib/n32h47x_48x_sm4.c`，编译于
  `Nations.N32H47x_48x_Library.0.8.0 ... ALGO_DEMO/GCC` 工程。
- 函数与形参名：`SM4_Init(SM4_PARM *parm)`（局部 `iv`、`counter`）、
  `SM4_Crypto(SM4_PARM *parm)`（局部 `inWordLen`、`counter`、`in`、`out`）、
  `SM4_Close(void)`、`SM4_Version(type, customer, date, version)`。
- `SAC_TypeDef`（decl 于厂商私有头，SDK 公开头文件中无此类型定义）给出全部寄存器名。
- `RCC_Module` 同样来自厂商私有头；SDK 的 `CMSIS/device/n32h47x_48x.h` 亦定义
  `RCC_AHB1PCLKEN_SACEN = 0x00008000`（"SAC clock enabled"）与 `SAC_BASE`，可与反汇编互相印证。
- `SM4_PARM` 成员偏移：`in`=0、`iv`=4、`out`=8、`key`=12、`inWordLen`=16、
  `EnDeMode`=20、`workingMode`=24，与 `inc/n32h47x_48x_sm4.h` 一致。

外设命名：算法加速器在 SDK CMSIS 头中称 **SAC**（`SAC_BASE = AHBPERIPH_BASE + 0xA000
= 0x4002A000`；`SAC_SRAM_BASE = 0x4002A400`，即 common 库所说的 ARAM）。

## 2. 寄存器总表

SAC 基址 `0x4002A000`。访问属性一栏：驱动只做 32-bit 读写；"FIFO 写口/读口"指同一地址
重复写入/读取时按队列顺序生效。

| 偏移 | 地址 | 名称（DWARF） | 访问（SM4 驱动视角） | 复位值 | 用途 |
| --- | --- | --- | --- | --- | --- |
| 0x00 | 0x4002A000 | `SACCR` | RW | 未知 | 全局控制：算法选择、使能、ARAM 清除触发/忙位 |
| 0x04 | 0x4002A004 | `SACSR` | RW（混合状态） | 未知 | 状态：完成标志、完成应答位 |
| 0x08 | — | `RESERVED0` | — | — | 保留 |
| 0x0C | 0x4002A00C | `SACSFR1` | RW | 未知 | 加解密方向、块模式、启动位 |
| 0x10 | 0x4002A010 | `SACSFR2` | RW | 未知 | 算法选择（SM4 = 0x7E） |
| 0x14 | — | `SACSFR3` | — | — | SM4 驱动未触碰 |
| 0x18–0x2C | — | `RESERVED1`–`RESERVED6` | — | — | 保留 |
| 0x30 | 0x4002A030 | `P1ADR` | W（FIFO 写口） | — | IV 装载（CBC，4 字） |
| 0x34 | 0x4002A034 | `P2ADR` | W（FIFO 写口） | — | FK 系统参数装载（4 字） |
| 0x38 | 0x4002A038 | `P3ADR` | W（FIFO 写口） | — | 用户密钥装载（4 字） |
| 0x3C | 0x4002A03C | `KADR` | W（FIFO 写口） | — | CK 轮常数装载（32 字） |
| 0x40 | 0x4002A040 | `IADR` | W（FIFO 写口） | — | 数据输入（每块 4 字） |
| 0x44 | 0x4002A044 | `OADR` | R（FIFO 读口） | — | 数据输出（每块 4 字） |

RCC 侧（基址 `0x40021000`）：

| 偏移 | 地址 | 名称 | 位 | 用途 |
| --- | --- | --- | --- | --- |
| 0x3C | 0x4002103C | `AHB1PCLKEN` | bit15（`RCC_AHB1PCLKEN_SACEN`，0x8000） | SAC 时钟使能；Init 置位（读-改-写），Close 清位 |

## 3. 位域明细

### 3.1 SACCR（0x4002A000）

字段划分采信 common/AES 兄弟库最终结论（父 Agent 转达，DWARF + AES_Init 写 0xD0
佐证）；位名以本文代称标注。

| 位 | 名称 | 观察到的行为 | 证据 |
| --- | --- | --- | --- |
| [3:0] | ALGO_SEL | 算法选择：0=AES、1=DES、4=SM4；SM4_Init 写入的 0xD4 低 nibble=4 | common/AES 最终结论 + Init+0x10 |
| 4（0x10） | ENABLE | 加速器使能；Init 置 1，Close 清 0 | common/AES 最终结论 + Close+0x1e |
| 5（0x20） | （未知） | 未观察 | — |
| 6（0x40） | （未知） | 0xD4 与 AES 的 0xD0 均含此位，单独含义未观察 | Init+0x10、AES_Init |
| 7（0x80） | ARAM_CLEAR/BUSY | 写 0xD4 后为 1（触发/忙）；`CLEAR_ARAM_DONE_CHECK` 轮询此位至 0 | common 最终结论 + Init 先写 0xD4 再调用该函数 |
| 8（0x100） | （推测：关闭时 ARAM 清除触发） | Close 先置 1，随后立刻调 `CLEAR_ARAM_DONE_CHECK` | Close+0x16–0x26 |
| 其它 | — | 未观察 | — |

`SM4_Crypto`/`SM4_Close` 以 `(SACCR & 0x14) == 0x14` 判定可用状态（Crypto+0x24、
Close+0xc）：0x14 = ENABLE（bit4）| SM4 算法选择位（nibble 值 4 的 bit2），即确认
"加速器已使能且选中 SM4"。Init 写入的 0xD4 = `ALGO_SEL=4 | 0x10 | 0x40 | 0x80`
恰好满足；Close 清掉 bit4/bit2 后即失效。该检查无法区分低 nibble 中同样含 bit2
的其它取值（如 5/6/7），厂商驱动即如此实现。

### 3.2 SACSR（0x4002A004）

| 位 | 名称（本文代称） | 观察到的行为 | 证据 |
| --- | --- | --- | --- |
| 0（0x01） | DONE_ACK | `CLEAR_ARAM_DONE_CHECK` 成功路径写 1，作为 ARAM 清除完成应答 | common 库语义（采信父 Agent 结论） |
| 5（0x20） | OP_DONE | 密钥扩展/每数据块完成标志：驱动轮询至 1，然后 `SACSR = SACSR \| 0x20` 清除（推断为写 1 清零 W1C） | Init+0x84–0x90、Crypto+0x56–0x64 |
| 其它 | — | 未观察 | — |

### 3.3 SACSFR1（0x4002A00C）

| 位 | 名称 | 编码 | 证据 |
| --- | --- | --- | --- |
| 0（0x01） | 方向 | 1 = 解密（SM4_DEC），0 = 加密（SM4_ENC） | Init+0x9a/0xc6 |
| 1（0x02） | 块模式 | 1 = CBC（SM4_CBC），0 = ECB（SM4_ECB） | Init+0xaa/0xda |
| 6（0x40） | （基值） | Init 写基值 0x40 后再置 bit7 启动密钥扩展；bit6 单独含义未观察 | Init+0x72–0x7c |
| 7（0x80） | 启动 | Init：密钥扩展启动；Crypto：每个 16 字节块启动 | Init+0x76–0x7c、Crypto+0x4c–0x54 |
| bit8–31 | — | GCC 版对置位操作先 `uxtb` 截断到低 8 位，armclang 版保留全部位；实际值 ≤0xFF，两者等价 | Init+0xc8（uxtb）vs armclang+0xe8（`&0xFE`） |

厂商驱动只依赖低 8 位；"仅低 8 位有效"为推断（依据 GCC 的 `uxtb` 掩码习惯），未在手册层面证实。

### 3.4 SACSFR2（0x4002A010）

| 值 | 含义 |
| --- | --- |
| 0x7E | 选择 SM4 算法（Init+0x6e–0x70 写入；其它取值属 AES/DES 等，本次未观察） |

### 3.5 参数/数据 FIFO（P1ADR/P2ADR/P3ADR/KADR/IADR/OADR）

均为队列式访问口：向同一地址连续写 N 个 32-bit 字即按序入队；`OADR` 连续读即按序出队。
SM4 各口的写入内容与字数：

| 口 | 偏移 | 写入内容 | 字数 | 时机 |
| --- | --- | --- | --- | --- |
| P3ADR | 0x38 | `parm->key` | 4 | Init，密钥装载 |
| KADR | 0x3C | `SM4_CK[]` | 32 | Init，紧随 key |
| P2ADR | 0x34 | `SM4_FK[]` | 4 | Init，最后装载 |
| P1ADR | 0x30 | `parm->iv` | 4 | Init 且 CBC 时，方向/模式位设置之后 |
| IADR | 0x40 | 明文/密文块 | 4 | Crypto，每块 |
| OADR | 0x44 | 密文/明文块 | 4 | Crypto，每块，完成标志清除后读 |

## 4. 厂商库实际操作流程

引用格式：`函数+偏移` 指 `/tmp/xhive-re-algo/gcc/sm4.disasm` 中该函数 section 内的
半字偏移；armclang 侧对应行为相同，不再逐条列出（差异单列于 4.6）。

### 4.1 SM4_Init（SM4_Init_OK = 0）

1. RCC 时钟开：`AHB1PCLKEN |= 0x8000`（读-改-写）。（Init+0x4–0xc）
2. `SACCR = 0xD4`（= ALGO_SEL=SM4(4) | ENABLE(0x10) | 0x40 | ARAM_CLEAR(0x80)）。（Init+0xe–0x12）
3. 调 `CLEAR_ARAM_DONE_CHECK()`；返回 `Time_Out`（=2）则直接返回 `SM4_TimeOutError`
   （0x27A90E3A）。（Init+0x16–0x20）
4. 参数校验（顺序固定）：
   - `workingMode` ∉ {SM4_ECB=0x11111111, SM4_CBC=0x22222222} → `SM4_ModeErr`（0x27A90E36）。（Init+0x22–0x2e）
   - `EnDeMode` ∉ {SM4_ENC=0x33333333, SM4_DEC=0x44444444} → `SM4_EnDeErr`（0x27A90E37）。（Init+0x30–0x3c）
   - `key == NULL` → `SM4_ADRNULL`（0x27A90E35）。（Init+0x3e–0x42）
   - CBC 且 `iv == NULL` → `SM4_ADRNULL`。（Init+0x44–0x4e）
   - 注意：`parm` 本身不做 NULL 检查。
5. 装载（DWARF call-site 证实实参）：
   `BufToFIFO(0x4002A038, key, 4)` → `BufToFIFO(0x4002A03C, SM4_CK, 32)` →
   `BufToFIFO(0x4002A034, SM4_FK, 4)`。（Init+0x50–0x68）
6. `SACSFR2 = 0x7E`（选择 SM4）。（Init+0x6c–0x70）
7. `SACSFR1 = 0x40`；再读-改-写 `SACSFR1 |= 0x80` 启动密钥扩展。（Init+0x72–0x7c）
8. 轮询 `SACSR & 0x20`：`counter` 从 0 起，每次未置位 `counter++`，
   `counter > Alg_TimeOut_Counter` 时返回 `SM4_TimeOutError`；置位后
   `SACSR = SACSR | 0x20` 清除。（Init+0x7e–0x90、0xb8–0xbe）
9. 方向位：ENC → `SACSFR1 &= ~0x01`；DEC → `SACSFR1 = (SACSFR1 & 0xFF) | 0x01`。
   （Init+0x9a/0xc6–0xce）
10. 块模式位：ECB → `SACSFR1 &= ~0x02`；CBC → `SACSFR1 = (SACSFR1 & 0xFF) | 0x02` 且
    随后 `BufToFIFO(0x4002A030, iv, 4)`。（Init+0xaa/0xda–0xe8）
11. 返回 `SM4_Init_OK`（0）。

### 4.2 SM4_Crypto（SM4_Crypto_OK = 0）

1. 校验（顺序固定）：`out == NULL || in == NULL` → `SM4_ADRNULL`；（Crypto+0x6–0xc）
   `inWordLen == 0 || (inWordLen & 3) != 0` → `SM4_LengthErr`（0x27A90E38）；（Crypto+0xe–0x18）
   `!(AHB1PCLKEN & 0x8000) || (SACCR & 0x14) != 0x14` → `SM4_UnInitError`（0x27A90E39）。
   （Crypto+0x1a–0x2c）
2. 按 16 字节块循环（`inWordLen` 为 32-bit 字数，块 = 4 字）：
   1. `IADR = in[0..3]`（4 次写）。（Crypto+0x34–0x4a）
   2. `SACSFR1 = (SACSFR1 & 0xFF) | 0x80` 启动本块。（Crypto+0x4c–0x54）
   3. 轮询 `SACSR & 0x20` 同 Init；超时返回 `SM4_TimeOutError`（此时本块及之前块
      的输出可能已部分写入 `out`，属部分完成语义）。（Crypto+0x56–0x64、0x8c–0x94）
   4. `SACSR = SACSR | 0x20` 清除完成标志。（Crypto+0x5e–0x64）
   5. `out[0..3] = OADR`（4 次读）。（Crypto+0x66–0x7a）
3. 返回 `SM4_Crypto_OK`（GCC 版用递减到 0 的 `inWordLen` 兼作返回值 0）。

### 4.3 SM4_Close（void）

1. `(AHB1PCLKEN & 0x8000) == 0` 直接返回（未开时钟视为无需关闭）。（Close+0x4–0x8）
2. `(SACCR & 0x14) != 0x14` 直接返回（未使能或未选中 SM4）。（Close+0xa–0x14）
3. `SACCR |= 0x100`（推测触发 ARAM 清除）。（Close+0x16–0x1c）
4. 重新读 `SACCR`，`SACCR &= ~0x14`（清 ENABLE 与 SM4 算法选择位）。（Close+0x1e–0x24）
5. 调 `CLEAR_ARAM_DONE_CHECK()`，返回值被忽略。（Close+0x26）
6. `AHB1PCLKEN &= ~0x8000` 关时钟。（Close+0x2a–0x30）

### 4.4 SM4_Version

两套编译器结果一致：`*type = 0x01`（Commercial）、`*customer = 0x00`（标准版）、
`date = {0x19, 0x03, 0x13}`（按头文件注释格式为 2019-03-25）、`*version = 0x10`
（按头文件注释格式为 1.0）。注意：发行包标注 v1.2.0，但库内版本字节仍报 1.0 /
2019-03-25 —— 版本字节未随包版本更新，属厂商库自身事实，两种工具链一致。

### 4.5 与 common 库的协作点

- `CLEAR_ARAM_DONE_CHECK()`：轮询 `SACCR bit7` 至 0，成功后对 `SACSR` 做读-改-写
  置位 bit0 应答（common 库最终结论）。Init 在写 `SACCR=0xD4` 之后调用（等待
  ARAM 清除完成），Close 在置 `SACCR bit8` 并清 ENABLE/算法选择位之后调用。
  超时返回值 2 被映射为 `SM4_TimeOutError`（Init）；Close 忽略返回值。
- `BufToFIFO(fifo_addr, data, wordLen)`：第三参数为 **32-bit 字数**（common 库最终
  结论，DWARF 形参名即 `wordLen`；GCC 反汇编亦吻合：`len>>2` 组、每组 4 字，加
  `len&3` 个尾字，总传输量恰为 `len` 个字；AES 库给 IV 传 4 同样吻合）。非对齐
  源缓冲经 16 字节栈缓冲由 `Cpy_U8` 中转（common 库最终结论）。SM4 的 4 个调用点
  传入的均为 4 字节对齐的 `uint32_t*`，全部按字顺序入队。
- `Alg_TimeOut_Counter`（common，初值 100000）：Init 的密钥扩展轮询预算
  （每次调用 counter 归零）；Crypto 的轮询预算 **跨块累计**（GCC 的 r5/armclang 的
  r5 均无块间复位），即一次 Crypto 调用全部失败轮询的总预算。

### 4.6 GCC 与 armclang 的等价性

- 全部 MMIO 地址、写入值、分支条件、错误码、FIFO 次序一致。
- 置位写法差异：GCC 对 SACSFR1 置位用 `uxtb`（截断低 8 位），armclang 用
  `& 0x7F`/`& 0xFE` 后加值；寄存器实际值 ≤0xFF，无行为差异。
- `Alg_TimeOut_Counter` 读取时机：GCC 每块轮询前读一次，armclang 每次轮询都读；
  常量配置下等价（ISR 中途修改变量属未定义使用场景）。

## 5. `.data` 常量（字节级提取，含板上实测修正记录）

`SM4_FK[4]` 与 `SM4_CK[32]` 为**非 const 的外部可见 `.data` 符号**
（`.data.SM4_FK`、`.data.SM4_CK`，symtab 类型 D；重定位证据：Init+0x58 调用的
32 字表 → `.data.SM4_CK`，Init+0x62 调用的 4 字表 → `.data.SM4_FK`）。

**字节序约定（新证据：N32H474 板上实测 + od 字节级复验）**：`.data` 的原始内存
字节即 GB/T 32907-2016 打印序（如 FK 首字节 a3 b1 ba c6、CK 首字节 00 07 0e 15）。
小端 ARM 上 C 数组元素的数值因此是 GB/T 记法的**逐字字节反转**：

```text
SM4_FK = { 0xC6BAB1A3, 0x5033AA56, 0x97917D67, 0xDC2270B2 }
SM4_CK = { 0x150E0700, 0x312A231C, 0x4D463F38, 0x69625B54,
           0x857E7770, 0xA19A938C, 0xBDB6AFA8, 0xD9D2CBC4,
           0xF5EEE7E0, 0x110A03FC, 0x2D261F18, 0x49423B34,
           0x655E5750, 0x817A736C, 0x9D968F88, 0xB9B2ABA4,
           0xD5CEC7C0, 0xF1EAE3DC, 0x0D06FFF8, 0x29221B14,
           0x453E3730, 0x615A534C, 0x7D766F68, 0x99928B84,
           0xB5AEA7A0, 0xD1CAC3BC, 0xEDE6DFD8, 0x0902FBF4,
           0x251E1710, 0x413A332C, 0x5D564F48, 0x79726B64 }
```

语义推断：硬件把 FIFO 字的内存字节按升序当作大端 SM4 常量/数据使用
（等效于硬件内部对每个 32-bit 字做字节反转）。

> **勘误记录（板上实测判定的重建缺陷）**：早期版本曾把 .data 字节序列直接当作
> 大端字面量写入 C 表（如 0xA3B1BAC6/0x00070E15），导致内存镜像逐字反转、
> 轮密钥全错。板上表现：加解密 roundtrip 全过（对称自洽），GB/T 标准向量与
> 厂商 ALGO_DEMO word 向量全挂（同固件 DES/AES 向量全过，方法学已交叉证实）；
> 标准向量实测输出 7448a6d5488c42aedb96494a791b791b ≠ 期望 681edf34d206965e86b3e94f536e4246。
> 修正为上表后与厂商 .data 字节一致。

## 6. 错误码（与 inc/n32h47x_48x_sm4.h 逐一对齐）

| 枚举 | 值 | 反汇编触发点 |
| --- | --- | --- |
| SM4_Crypto_OK / SM4_Init_OK | 0 | 正常返回 |
| SM4_ADRNULL | 0x27A90E35 | Init：key==NULL 或（CBC 且 iv==NULL）；Crypto：in==NULL 或 out==NULL |
| SM4_ModeErr | 0x27A90E36 | Init：workingMode 非法 |
| SM4_EnDeErr | 0x27A90E37 | Init：EnDeMode 非法 |
| SM4_LengthErr | 0x27A90E38 | Crypto：inWordLen 为 0 或非 4 的倍数 |
| SM4_UnInitError | 0x27A90E39 | Crypto：SAC 时钟未开或 SACCR 判活掩码 0x14 不符 |
| SM4_TimeOutError | 0x27A90E3A | Init：ARAM 清除或密钥扩展轮询超时；Crypto：块轮询超时 |

## 7. 推测与未观察项

- 所有复位值未知：静态分析无上电采样证据。
- SACCR 字段划分（[3:0] 算法选择、bit4 ENABLE、bit7 ARAM_CLEAR）采信 common/AES
  兄弟库最终结论（父 Agent 转达）；SM4 侧反汇编（0xD4 与 0x14 掩码）与之自洽。
- SACCR bit6（0x40）单独含义未知，AES/SM4 初始化值均包含该位。
- SACCR bit8（0x100）＝"关闭时触发 ARAM 清除"为推测，依据是 Close 置位后立即
  调用 `CLEAR_ARAM_DONE_CHECK`。
- SACSR bit5 的 W1C（写 1 清零）为推断：驱动读-或-写回 0x20 后继续读 OADR 数据仍正确。
- SACSR bit0 完成应答、SACCR bit7 忙位语义采信 common 库最终结论（父 Agent 转达）。
- SACSFR1 仅低 8 位有效为推断（GCC `uxtb` 习惯），未证实。
- SACSFR2 除 0x7E 外的编码（AES/DES 等）未观察。
- SM4 字节序（板上证据佐证）：硬件把每个 32-bit FIFO 字的内存字节按升序当作
  大端 SM4 数据/常数（等效内部字节反转；FK/CK 的 .data 镜像即此约定的直接证据）。
  因此 GB/T 大端打印向量应按**小端打包**成字：字节串 01 23 45 67 → 字 0x67452301。
- `SM4_Init` 不检查 `parm == NULL`（会直接解引用崩溃），与厂商行为一致，属保留缺陷。
- `SM4_Crypto` 超时中途返回时可能已写部分输出（部分完成），厂商无回滚。

## 8. 板上测试建议（GB/T 32907-2016）

调用序列：`SM4_Init(&parm)` → `SM4_Crypto(&parm)` → `SM4_Close()`；
CBC 时 `parm.iv` 有效，ECB 可为 NULL。

1. 单块加密（标准向量，字值按小端打包，见 §5/§7 字节序约定）：
   - key 字 = `{0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476}`
     （即 0123456789abcdeffedcba9876543210）
   - in 字 = 同 key 字（inWordLen = 4）
   - 期望 out 字 = `{0x34DF1E68, 0x5E9606D2, 0x4FE9B386, 0x46426E53}`
     （即 681edf34d206965e86b3e94f536e4246）（SM4_ECB + SM4_ENC）
2. 单块解密（同 key，SM4_ECB + SM4_DEC）：
   - in 字 = 第 1 项的 out 字，期望还原第 1 项的 in 字
3. 厂商 ALGO_DEMO word 向量（word 对 word 直接比对，无字节序转换）：
   - key1 = `{0x84853E30, 0xB3D3154D, 0x9A887F49, 0xDC65910A}`，
     in1 = `{0x4B551C70, ...}`，期望 SM4_ECB_EN = `{0xD61A389C, ...}`（以 demo 源为准）；
   - CBC 用 demo 的 IV 与期望值同样 word 级比对。
3. 双块 ECB 加密（inWordLen = 8，ECB 性质可无计算推导）：
   - in = P||P（P 同向量 1 明文），期望 out = C||C（C 同向量 1 密文）
4. CBC 单块（IV = 0^128，inWordLen = 4）：
   - 期望首块密文与 ECB 相同（C1 = E(P1 ⊕ 0) = C），可先验 IV 通路
5. CBC 解密回环：用第 4 步结果作 in（SM4_CBC + SM4_DEC，同 IV），期望还原 P1；
   CBC 多块加密的 C2 = E(P2 ⊕ C1) 依赖硬件轮函数，建议同时跑解密回环校验
6. 建议附带检查：`SM4_Crypto` 在未 `SM4_Init` 时返回 `SM4_UnInitError`；
   `inWordLen = 5` 返回 `SM4_LengthErr`；错误 workingMode/EnDeMode 返回对应错误码；
   `SM4_Version` 返回 `{0x01, 0x00, {0x19,0x03,0x13}, 0x10}`。

## 附录 A：证据偏移索引

| 结论 | GCC 反汇编位置 |
| --- | --- |
| RCC 0x4002103C bit15 置位 | SM4_Init+0x4–0xc |
| SACCR=0xD4（ALGO=SM4|ENABLE|ARAM_CLEAR） | SM4_Init+0xe–0x12 |
| ARAM 清除超时→SM4_TimeOutError | SM4_Init+0x16–0x20 |
| 校验顺序与错误码字面量 | SM4_Init+0x22–0x4e、0xee/0xf2/0xf6 |
| BufToFIFO(0x38/0x3C/0x34, …, 4/32/4) | SM4_Init+0x50–0x68（DWARF call-site 参数证实） |
| SACSFR2=0x7E | SM4_Init+0x6c–0x70 |
| SACSFR1=0x40、\|=0x80 | SM4_Init+0x72–0x7c |
| 轮询 SACSR bit5 与超时 | SM4_Init+0x7e–0x90、0xb8–0xbe |
| 方向/模式位编码 | SM4_Init+0x9a–0xce、0xd0–0xe8 |
| IV → 0x4002A030 | SM4_Init+0xd6–0xe8 |
| Crypto 校验序列与判活掩码 0x14 | SM4_Crypto+0x6–0x2c、0x96/0x9a/0x9e |
| IADR/OADR = +0x40/+0x44 | SM4_Crypto+0x34–0x4a、0x66–0x7a |
| 每块启动写法 | SM4_Crypto+0x4c–0x54 |
| Crypto 超时跨块累计 | SM4_Crypto+0x56（r7 重载）与 0x8c（r5 无复位） |
| Close 序列 | SM4_Close+0x4–0x30 |
| Version 字节 | SM4_Version 全函数（0x2–0x18） |
| BufToFIFO len 单位=字 | common.disasm BufToFIFO+0x6/0x88/0xa0/0x1a |
