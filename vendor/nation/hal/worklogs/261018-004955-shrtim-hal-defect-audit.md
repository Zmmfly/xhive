# SHRTIM HAL 软件缺陷审查（Astra max 只读源码审计）

- **日期**: 2026-10-18
- **审计对象**: `vendor/nation/hal/src/nation_shrtim.c`（1988 行）+ 契约头文件（602 行），提交 `e2bb0ac`
- **审计方法**: 逐函数源码审查 + vendor std-periph / CMSIS 位定义跟踪 + RAGFlow 全库社区检索 + 只读 Python 常量审计（ADC 表项、寄存器偏移、master 错位掩码、EXEV 移位映射及数值边界）
- **审计员**: shrtim-defect-hunt（gpt-6-astra, max reasoning）
- **前提**: T01-T23 全量回归 PASS ≠ HAL 正确——本报告列出测试未覆盖路径中的缺陷

## 缺陷总览

| 严重程度 | 数量 | 代表项 |
| --- | --- | --- |
| Blocker（条件性） | 1 | #1 SHRTPLL 所有权 |
| Major | 18 | #12 ADC 错位、#15 master 初始化、#10 fault 恢复 |
| Minor | 5 | #20 fault 枚举校验、#22 MST_SYNC 路由 |
| Nit | 1 | #25 死区上限文档 |

## 最高优先级修复清单

### S1 ADC 路由三连修（#12/#13/#14）

**#12 表序错位**：HAL 枚举 `A=0..F=5, MASTER=6`；vendor 表 `MASTER,A,B,C,D,E,F`。`table[unit]` 对 MASTER 请求读到 F 行，对 A 请求读到 MASTER 行——所有单元偏一位。T14 传 C(2) 实际路由到 B。
**修复**：`row = (unit == SHRTIM_UNIT_MASTER) ? 0 : (unsigned)unit + 1`。

**#13 vendor 偏移表错误**：ADC6 和 ADC5 都指向 ADTGEX1 低 6 位；ADC9 和 ADC10 都指向 ADTGEX2 高 6 位。写 6 会覆盖 5 的配置，写 9 会覆盖 10。
**修复**：HAL 层适配表纠正偏移映射。

**#14 跨组残留**：trig1-4 换 group 后旧 SRC 寄存器未清空。
**修复**：路由替换时先清旧组再提交新组。

### S2 master 初始化修复（#15）

`center_master_setup()` 给 MASTER 调 `SetUpdateTrig(RESET)`——vendor 契约 MASTER 只允许 NONE/REPETITION。宏展开 `0x40000<<12 = 0x40000000` = `MCTRL.BRSTDMA[0]`，不是 `MREPTUEN(bit29)`。且全文件从未给 MASTER 设 CONT=1（连续模式），复位默认 CONT=0 单发。
**修复**：使用 REPETITION 更新语义 + 显式设置 MASTER.CONT。

### S3 fault 恢复契约修复（#10/#11）

**#10**：`fault_reset()` 只清 INTCLR，不写 OEN。UM 原文：故障后输出保持故障态直到软件重新使能。HAL 头文件宣称"clear flag → resume"不成立。
**修复**：保存故障→单元→输出映射和原使能状态，reset 时有条件重新使能 OEN。

**#11**：fault_config/attach 在运行中先清全部 FALTEN 再启用，有失保护窗口。
**修复**：安全顺序——先编程输出安全态再一次性提交掩码。

### S4 安全/lifecycle 修复（#1/#2/#3/#4）

**#1**：`init()/deinit()` 不检查 SHRTPLL 是否被 SYSCLK 依赖——条件性 Blocker。
**#2**：`deinit()` 不禁 NVIC、不清回调/状态——生命周期残留。
**#3**：`pwm_stop()` 只停计数器不关输出——"pins at idle" 契约未实现。
**#4**：`out_polarity()` 运行中写 POL——UM 明确禁止。

### S5 运行时安全修复（#5/#6/#16/#17/#18）

**#5**：exev_config 允许 fast+edge、fast+filter 等硬件禁止组合。
**#6**：solve_timing 只查 16 位容量不查 UM 表 9-8 合法时序范围。
**#16**：运行中 freq/duty 更新非事务性（未屏蔽自动更新）。
**#17**：运行中改 CKPSC（跨档调频）违反官方禁改条件。
**#18**：pwm_init 先写死区再停机（H7 在 HAL 层未防护）。

## 中低优先级

| # | 函数 | 问题 | 级别 |
| --- | --- | --- | --- |
| 7 | capture_read | SW 触发与 ISR 争用 CPT 标志 | Major |
| 8 | irq_hook | callback/user 两次发布无原子性 | Major |
| 9 | isr_fault | 无 IE 门控（timer/master 有） | Major |
| 19 | init | 重复 init 无安全重配/失败收敛 | Major |
| 20 | fault_attach | 非法枚举静默映射为 NO_ACTION | Minor |
| 21 | out_force | 非持续覆盖（一次性 SWT 事件） | Minor |
| 22 | irq_hook | MST_SYNC 注册/分发单元不一致 | Minor |
| 23 | master_period | period=0/>65535 静默丢弃 | Minor |
| 24 | capture | EXEV 捕获无无副作用读取入口 | Minor |
| 25 | deadtime | 文档上限矛盾（16µs vs 34µs） | Nit |

## 对 worklog H1-H9 的影响

| worklog 项 | 影响 |
| --- | --- |
| H9 MCNT 恒 0 | #15 master 从未正确连续运行——先修软件再评估硅缺陷 |
| H4 master 时基 | #15 错误 master 配置可能部分解释观测 |
| H5/H6 跨单元相位 | #17 运行中改 CKPSC 未消除——须先排除软件因素 |
| H7 死区锁存 | #18 HAL 本体未防护（demo 碰巧规避） |

## 社区/第三方已知问题

RAGFlow 全库检索（qwen3-reranker）**未命中可核验的社区帖子或 N32H47x 专用勘误表**。N32L40x 勘误表无 SHRTIM 专章。SHRTIM UG 有 DAC 复位触发延迟（192ns 消隐窗口）但非 SHRTIM PWM 缺陷。

## 建议的新测试方向

- 冷启动 master MCTRL 寄存器验证（CONT/MREPTUEN/CKPSC 实际值）
- ADC 触发全矩阵（10 触发 × 6 单元 × 5 事件，交叉验证寄存器）
- fault 释放后 OEN/ODISSTS 状态 + 波形恢复
- pwm_stop 后引脚电平（而非仅频率消失）
- 运行中 out_polarity 应返回错误
- EXEV 非 SW 捕获配合 IRQ
