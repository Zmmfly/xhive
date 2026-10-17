# FOC 电流/速度控制与反电势 SMO

本模块提供 C99 的 `f32`、`f64`、`q15`、`q31` 四套接口。算法状态全部由调用者持有，没有动态分配、VLA、`alloca`、递归、硬件访问或隐式单例。独立 context 可重入；同一 context 的线程/中断调用必须串行化。

**适用模型是平衡、正弦反电势、非凸极 PMSM/SPMSM。** SMO 使用单一标量电感 `L=Ld=Lq`；不提供 IPMSM 的凸极补偿，也不声称对显著磁饱和、非正弦/梯形 BLDC 反电势保持相同精度。FOC 不包含交叉耦合/反电势前馈、弱磁、MTPA、无感启动或故障状态机。

## 接口和数值约定

公开契约见 [`fieldctrl_foc.h`](include/fieldctrl_foc.h)、[`fieldctrl_smo.h`](include/fieldctrl_smo.h)。下表的 `S` 为 `f32/f64/q15/q31`：

| 接口 | 作用 |
| --- | --- |
| `fieldctrl_S_foc_init/step` | d/q 电流 PI、二维圆形限压、联合抗积分饱和、逆 Park、SVPWM |
| `fieldctrl_S_speed_foc_init/step` | 机械速度 PI 与电流环的同步组合，`id*=0` |
| `fieldctrl_S_smo_init/step` | 电流预测、边界层滑模注入、低通反电势、角度/电频率估计和锁定诊断 |

- 电流 A、电压 V、电阻 Ω、电感 H、时间 s；配置增益单位在公开头逐项注明。
- 所有角度为 **electrical turns**，范围 `[0,1)`，正方向 `alpha -> beta`；一电周为 `2*pi rad`。
- SMO 和速度组合反馈使用带符号的 **electrical turns/s**。速度组合的参考值为 **mechanical turns/s**，内部除以 `pole_pairs` 得到机械反馈；不能把 rad/s 或 rpm 直接传入。
- FOC 的角度输入必须已归一化到 `[0,1)`。量化后恰好成为 `1` 的角度应重新 wrap；SMO 输出对此做了处理。
- 每个 q 标量为 `mantissa * 2^(exponent-15)` 或 `mantissa * 2^(exponent-31)`，不是固定范围 `[-1,1)`。参数/结果的指数独立，沿用 linears 的指数范围。
- q15/q31 生产路径使用共享 linears 整数算术及整数 CORDIC，**没有转成 float/double 执行的包装**。q15 的 FOC 积分、SMO 电流预测、角度历史和滤波状态保留 Q31 工作精度；SMO q15 实际复用 `filters_lowpass_q31_*`。仅在公共输入/输出边界作 q15 转换。f32/f64 分别使用原生单/双精度。
- 有限精度仍有边界：比当前工作状态 LSB 更小的增量仍可能舍入消失。宽状态解决的是每拍压到 q15 导致的过早冻结，不是无限精度积分。
- 所有错误保留 context 与输出的原字节；输出对象不得与 context 重叠。成功仅表示软件命令或观测结果已算出，不表示 PWM 已生效或电机已安全。

## 电流 FOC 与二维抗积分饱和

每拍将测量电流 Park 到转子坐标系，然后计算：

```text
e = i_reference - i_dq
DeltaI = diag(ki_d,ki_q) * e * dt
I_candidate = I_old + DeltaI
v_request = diag(kp_d,kp_q) * e + I_candidate
Vmax = min(config.voltage_limit, Vdc/sqrt(3))
v_limited = v_request * min(1, Vmax/|v_request|)
```

这里的约束是 **d/q 联合圆形边界**，不是两轴各自裁剪。两个分量都未超出单轴上限，矢量模长仍可能饱和。

若发生圆形饱和且 `DeltaI dot (v_request-v_limited) > 0`，本拍两轴积分增量一起拒绝，再用旧积分重新计算并限压。反向解除饱和的积分可以保留。点积只需要符号：实现将增量与请求电压分别除以自己的最大绝对分量后判断，避免物理量乘积在正常有限输入下仍上溢/下溢。零增量不需要作点积。

限幅乘积/商通过共享 `FC_RESCALE` 保留中间指数：既不先算可能下溢的 `Vmax/|v|`，也不先算可能丢掉小轴的 `v_axis/|v|`。q 版本仍然只调用整数核心并合成最终指数。

最终依次进行逆 Park 与基础 SVPWM。SVPWM 使用最大/最小相电压中点的公共模注入，输出三相 `[0,1]` 占空比，不进行过调制。最终公共标量舍入可轻微改变矢量模长；SVPWM 还会按实际 DC bus 重新约束。电压输出与占空比重构电压之间因此存在已量化的输出误差，不应当把未限幅 PI 请求送给观测器。

速度组合为：

```text
mechanical_feedback = electrical_speed / pole_pairs
speed_error = mechanical_reference - mechanical_feedback
iq_reference = PI_speed(speed_error, dt), limited to +/-current_limit
id_reference = 0
FOC_current(..., reference, ..., dt)
```

外环首先采用基础 PI 的条件积分抗饱和。如果内环的最终 q 电压饱和残差与速度误差非零且同号，还撤销本拍速度积分更新，避免电压不足时外环继续累积。该同号判断不做物理量相乘。本拍已经生成的 q 电流参考保留，撤销后的积分影响下一拍。

`speed_foc_step` 每调用一次同时运行两环，二者共用传入的 `dt`，没有隐藏降采样。若要不同环路频率，应由调用者分别调度基础 PI 与独立电流 `foc_step`，并管理参考值保持和相应的抗饱和反馈。

## SMO：真实电流预测与反电势提取

采用静止坐标系电压模型，`psi` 为永磁磁链、`f_e` 为带符号电频率：

```text
di/dt = (v - R*i - e)/L
e_alpha = -2*pi*f_e*psi*sin(2*pi*theta)
e_beta  =  2*pi*f_e*psi*cos(2*pi*theta)
```

`step` 在时刻 `n` 读取 `i[n]`，所传 `v[n]` 是区间 `[n,n+1)` 实际施加的平均电压。注入和预测严格采用：

```text
current_error = i_hat[n] - i[n]
z[n] = K * sat(current_error / boundary)       # 两轴独立，sat 范围 [-1,1]
i_hat[n+1] = i_hat[n] + dt/L * (v[n] - R*i_hat[n] - z[n])
e_hat[n] = (1-alpha)*e_hat[n-1] + alpha*z[n]
```

`e_hat` 直接使用 filters 的一阶低通实现，不另造滤波内核。输出 `current_hat` 对应下一拍预测；角度和反电势对应当前采样时刻，仍含观测器固有延迟。滤波反电势未作幅值反卷积。

初始化校验正数/增益/方向，以及：

```text
0 < alpha_emf <= 1
0 < alpha_speed <= 1
0 < dt*(R + K/boundary)/L < 2
max_speed*dt < 1/4 electrical turn
0 < min_speed < max_speed
```

第三条来自边界层内误差线性化和显式 Euler 的稳定区间，不是对参数失配、延迟、采样噪声的全局稳定保证。`K` 必须覆盖目标运行范围内的反电势和建模误差；增大 K 同时减小 boundary 会收紧采样周期约束。初始化通过不代表参数已完成电机整定。

### 方向、角度与滤波相移

对指定方向 `sigma = +1/-1`：

```text
theta_filtered = atan2(-sigma*e_hat_alpha, sigma*e_hat_beta)/(2*pi)
f_raw = wrapped_difference(theta_filtered[n], theta_filtered[n-1]) / dt
f_hat = lowpass(f_raw, alpha_speed)
```

负转时反电势本身反号，若省略 `sigma`，角度会错半电周。方向在初始化时显式给定；反转必须由调用者用独立控制流程跨过零速，再重新初始化对应方向。这里没有自动方向识别。

反电势低通实际传递函数为：

```text
H(z) = alpha / (1-(1-alpha)*z^-1)
Omega = 2*pi*f_hat*dt
phase_lead = atan2((1-alpha)*sin(Omega),
                  alpha + (1-alpha)*(1-cos(Omega))) / (2*pi)
theta = wrap(theta_filtered + phase_lead)
```

此式使用带符号电频率和 **离散** 频率响应，正反转分别加相应的相位补偿。分母采用等价的无消减形式，以保留非常小的正 `alpha`；不能用会把 `1-(1-alpha)` 消成零的写法。

只补偿低通在定频附近的相位，不补偿 SMO 电流误差反馈、边界层、ADC/PWM 延迟和加速度造成的动态误差。特别是滤波越慢，突变时估计频率越滞后；补偿公式不能消除该暂态。

### observable 与 locked 的含义

`LINEARS_OK` 与“角度可用”是两件事。每次必须检查输出标志：

- 反电势模长至少为 `min_emf`；否则保持上次角度、清空速度/差分历史及锁定计数。
- 具备前一有效角度，原始和滤波速度符号均符合配置方向，二者绝对值都处于 `[min_speed,max_speed]`。
- 两轴预测电流误差都处于 `boundary` 内，确认正在边界层内运行。
- 以上同时成立才置 `observable`；连续 `lock_samples` 拍成立才置 `locked`，任一条件不满足立即清锁。

这些是基于内部信号的接管诊断条件，不是外部磁链角真值或硬件安全认证。噪声、错误参数和电压重构误差能制造伪反电势，必须在实际系统中设置阈值、接管一致性检查和故障策略。

**静止时不存在反电势，不能凭此算法自启动，也不能把 `LINEARS_OK` 或保持的旧角度当成有效位置。** 低速、停转、换向和失锁需要调用者的开环、编码器或其它可观测方案。

## 调度和板级责任

建议先单独校准相序、电流符号、Clarke/Park 标定、极对数、磁链角零点及同步采样。SMO 的电压输入应从实际占空比与实测 DC bus 重构，并按硬件需要计入死区和压降，而不是直接使用尚未限压的控制量。

软件测试中的因果调度为：

1. 当前电流与用于本拍的角度送 FOC，得到占空比。
2. 从占空比重构本区间电压，连同当前电流送 SMO，得到本拍估计和下一拍电流预测。
3. 下一拍采用上拍 SMO 角度，加上 **SMO 自己的频率**乘采样间隔进行时间戳外推。

这项外推在调用者/测试侧显式完成，不是 SMO 内置的采样延迟补偿。真正硬件的采样点与 PWM 更新点可能不同，必须根据时间戳处理。限流、过流硬关断、欠压/过压、过温、门极互锁、最小脉宽和看门狗均不由本库实现。

## 测试与证据边界

作者测试按四版展开，`test/foc.cc` 与 `test/smo.cc` 使用独立 double 公式/模型作为参考，先量化公开输入再算参考，不用未量化理想输入掩盖 q 误差。测试独立固定种子，工作状态和输出量化误差分开计入预算；CORDIC 的两次旋转预算来自 32 轮残差和 Q31 分量舍入。

覆盖：

- 电流 PI、二维抗饱和、逆变换和占空比重构公式；联合圆形约束不等价于单轴约束的反例。
- 极小/极大正常有限物理量下的抗饱和谓词、双向量程限幅、小轴保留；q 单轴跨完整公开指数范围的严格结果；小步长下宽积分状态不提前冻结。
- 正负旋转的长序列电机电流环、母线饱和后恢复；包含机械惯量的速度级联加减速/反转模型。
- SMO 预测及注入饱和递推；离散 SMO 与低通级联的解析频响，正负转相位补偿和速度跟踪。
- 加减速电机模型、无反电势、静态偏置、错误方向、反电势丢失及连续有效样本重新锁定。
- 极小正低通权重、参数/输入失败的 state/output 原子性、两个独立 context 交错执行。
- 正反转下先有感 warmup，再经 `observable && locked` 接管；接管后 FOC 的反馈角只来自 SMO，电机真角仅用于物理模型和误差断言。

最后一项使用理想轴速源维持旋转，是**旋转时无感电流环接管仿真**，不是无感零速启动或无感机械速度全闭环实测。测试没有把真角偷偷继续送入“无感”控制。

作者在 SDK 外的独占临时目录中完成了以下验证（GoogleTest 为主机现有包，无根 Xmake 配置改动）：

- C 生产文件使用 `cc -std=c99 -Wall -Wextra -Wpedantic -Werror`；测试使用 `c++ -std=c++17` 和相同警告选项。
- `-O2` 构建运行 **74 个 GoogleTest 用例全部通过**：FOC 38 个、SMO 36 个，没有跳过。两个 q 全指数范围用例直接比较 mantissa/exponent，不转换为 double。
- 同一组用例以 `-O1 -g -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer` 重新编译运行，**74 个全部通过**。
- 作者文件的 UTF-8、末尾换行、无尾随空白检查以及限定文件范围的 `git diff --check` 通过。

这些结果只属于 Linux 主机软件检查。没有实板、逆变器、电流采样或电机热/机械极限验证。SDK 统一构建/测试由上层集成流程运行；这些作者检查不替代硬件验证。

## 算法来源

1. Simon Strobl, imperix **TN111: Field oriented control (FOC) of permanent magnet synchronous machine (PMSM)**，2021。提供各向同性 PMSM 的 d/q 电流控制和电压模型背景。[公开文档](https://imperix.com/doc/implementation/field-oriented-control-of-pmsm)。本实现省略其解耦网络，不照搬其硬件性能结论。
2. imperix **Discrete PI controller implementation**。列出条件积分、反算等成熟抗饱和方法。[公开文档](https://imperix.com/doc/implementation/pi-controller)。本实现将条件积分的“继续推向饱和”判断扩展为二维圆形边界的径向点积；具体离散规则以上面的公式为准。
3. Simon Strobl, imperix **TN136: Sensorless motor control using a back-EMF Sliding-Mode Observer**，2021。说明电流误差注入、反电势提取、低通相移及低速不可观测性。[公开文档](https://imperix.com/doc/implementation/sensorless-motor-control)。本实现选择连续分段线性的 `sat` 边界层与角差分低通，不实现该文的 Q-PLL。

文献说明算法依据；库内定点表示、事务提交、离散相移公式和明确的方向/锁定契约由本实现及对应测试定义。参考文献中的实验结果不构成本仓库的实板证据。
