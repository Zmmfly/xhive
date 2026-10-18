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

- f32/f64 保留原生浮点和 SI 单位：电流 A、电压 V、电阻 Ω、电感 H、时间 s。所有浮点输入必须有限。
- q15/q31 普通信号分别是 **`int16_t raw/2^15`、`int32_t raw/2^31`**，编码范围为 `[-1,1)`，全部位型有效。没有逐标量指数、context 缩放字段或动态二进制点。
- 两种 Q API 的 **全部 `kp/ki` 系数都是 `linears_fixed_t=int64_t raw/2^31`**，不是 Q15 API 就用 15 位小数。系数可以大于 1。例如系数 `1.0` 必须编码成 `2147483648`，而不是 `32768`。
- Q 积分、电流预测、角度历史和滤波历史都使用恒定 F31 的 `int64_t`。工作范围为 `[-2^32,2^32-2^-31]`；不是把指数改名。整数核心及 CORDIC 不转成 float/double 执行。
- Q `requested_voltage` 是显式 `fieldctrl_fixed_dq_t` 宽 F31 诊断；未限幅请求可以大于 1。Q SMO `electrical_speed` 也是宽 F31 诊断，未锁定角跳变可以产生超过 Q1 的频率。其他公开信号仍是 Q1，不统一放大接口域。
- 数学结果恰好 `+1` 按 linears 端点策略导出为 `INT16_MAX/INT32_MAX`，误差为一个信号 LSB；`-1` 精确导出为 `MIN`。真实结果超出 `[-1,+1]` 则返回 `RANGE`，不通过普通导出静默截断。`MAX` 输入仍表示 `1-LSB`，尤其不能把 `MAX alpha` 当作精确 1。
- 所有角度仍是 **electrical turns**，范围 `[0,1)`，正方向 `alpha -> beta`，一电周为 `2*pi rad`。Q15 角度专门进行周期最近舍入：上端半个量化单元舍入到零，不受普通 `+1 -> MAX` 导出规则替代。
- Q15 比其 F31 工作状态少 16 位小数，因此 sub-Q15 增量能在状态中积累。Q31 与工作状态的小数位均为 31；小于半个 F31 LSB 的逐拍增量仍会消失，不能声称具有无限精度积分。
- 所有错误保留 context 与输出的原字节；输出对象不得与 context 重叠。成功仅表示软件命令或观测结果已算出，不表示 PWM 已生效或电机已安全。

### Q API 的 per-unit 换算

这是 Q 接口与 context 布局的不兼容迁移：调用者必须改用定宽 raw 信号/F31 系数，不能复用旧对象布局或直接填入未换算的 SI 数值。f32/f64 的接口和数值域保持不变。

调用者选择正的 `Vbase/Ibase/Tbase`，**不将基值存入生产 config/context**。先按下表换算，再量化到声明的 Q1 或 F31 类型：

| 物理量 | Q 接口数值 |
| --- | --- |
| 电压、电流、时间 | `Vpu=V/Vbase`、`Ipu=I/Ibase`、`tpu=t/Tbase` |
| 电频、机械频率 | `fpu=f*Tbase`；机械反馈仍由电频除 `pole_pairs` |
| 电阻、电感 | `Rpu=R*Ibase/Vbase`、`Lpu=L*Ibase/(Vbase*Tbase)` |
| 磁链、转矩（软件 plant） | `psi_pu=psi/(Vbase*Tbase)`、`torque_pu=torque/(Vbase*Ibase*Tbase)` |
| 电流环比例、积分系数 | `kp_pu=kp_SI*Ibase/Vbase`、`ki_pu=ki_SI*Ibase*Tbase/Vbase` |
| 速度环比例、积分系数 | `kp_pu=kp_SI/(Ibase*Tbase)`、`ki_pu=ki_SI/Ibase` |

Q 信号不能直接编码 `50 Hz`：例如 `Tbase=0.005 s` 时传入 `fpu=0.25`。频率不乘 `2*pi`；只有正弦反电势模型将频率转换为角速度。`dt` 和频率必须使用同一 `Tbase`，从而 `fpu*dtpu` 仍是一拍的电角度增量。

选择基值时须给坐标旋转和预测留余量：两个合法 Q1 电流分量旋转后可能超出 Q1，或合法电压使下一拍电流预测超出 Q1；此时接口返回 `RANGE` 并保持状态，而不是捏造一个夹紧的观测量。速度误差在工作域计算，可达 `(-2,2)`，不在进入 PI 前额外窄化。

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

若发生圆形饱和且 `DeltaI dot (v_request-v_limited) > 0`，本拍两轴积分增量一起拒绝，再用旧积分重新计算并限压。反向解除饱和的积分可以保留。点积只需要符号，请求电压与理想圆形饱和残差同向：

- f32/f64 将两个向量分别除以自身最大绝对分量后判断，避免正常有限物理量的乘积上溢/下溢。
- Q 路径用 linears 的两个精确 F62 MAC 累加，然后直接检查双 limb 的符号位和非零位。**不先除小向量，也不把点积舍入回 F31**，否则真实正乘积可能变成零并错误地允许积分继续增长。

限幅使用共享 `FC_RESCALE(a,limit,norm)`。Q 路径通过固定 F31 宽乘除计算 `a_raw*limit_raw/norm_raw`，只在最后舍入，不存储任何指数或缩放信息。浮点路径保留原来的量程安全计算，既避免 `limit/norm` 下溢，也避免先计算小轴 `a/norm` 导致可表示结果丢失。

最终依次进行逆 Park 与基础 SVPWM。SVPWM 使用最大/最小相电压中点的公共模注入，数学占空比为三相 `[0,1]`，Q 的正端点按前述规则编码为 `MAX`，不进行过调制。最终公共标量舍入可轻微改变矢量模长；SVPWM 还会按实际 DC bus 重新约束。电压输出与占空比重构电压之间因此存在已量化的输出误差，不应当把未限幅 PI 请求送给观测器。

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

`e_hat` 直接使用 filters 的一阶低通实现，不另造滤波内核。Q SMO 用 `filters_lowpass_q31_t` 持有固定的 Q1.31 alpha/F31 history，调用 `filters_lowpass_work_step(&state, alpha, input)`：该共用内核执行精确 F62 两项积和，再一次舍入到 F31，允许未锁频率超出 Q1 而不丢失诊断。f32/f64 继续调用原生低通入口。

Q 注入先与 `+/-boundary` 比较，只有在线性区才做除法；f32/f64 保留原有除法与溢出失败行为。这样 Q 路径中合法的 `i_hat=+1`、`i=-1`、Q31 `boundary=1 raw` 不会先形成不可表示的 `2^32` 商；正确的 `sat` 本来就是 `+1`。

输出 `current_hat` 对应下一拍预测；角度和反电势对应当前采样时刻，仍含观测器固有延迟。滤波反电势未作幅值反卷积。

初始化校验正数/增益/方向，以及：

```text
0 < alpha_emf <= 1
0 < alpha_speed <= 1
0 < dt*(R + K/boundary)/L < 2
max_speed*dt < 1/4 electrical turn
0 < min_speed < max_speed
```

Q 的 alpha 上限实际上是 `1-LSB`，不是精确 1；稳定性和角步长边界在声明的工作精度下检查。第三条来自边界层内误差线性化和显式 Euler 的稳定区间，不是对参数失配、延迟、采样噪声的全局稳定保证。`K` 必须覆盖目标运行范围内的反电势和建模误差；增大 K 同时减小 boundary 会收紧采样周期约束。初始化通过不代表参数已完成电机整定。

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
- 保留 f32/f64 极小/极大正常数的抗饱和、双向量程限幅和小轴保留反例；Q 改为 RAW_MIN/MAX/LSB、显式宽系数/请求、超 Q1 输出原子失败、工作溢出、精确 F62 微小正点积、sub-Q15 累积及 F31 逐拍舍入界。
- 正负旋转的长序列电机电流环、母线饱和后恢复；包含机械惯量的速度级联加减速/反转模型。
- SMO 预测及注入饱和递推；离散 SMO 与低通级联的解析频响，正负转相位补偿和速度跟踪。
- 加减速电机模型、无反电势、静态偏置、错误方向、反电势丢失及连续有效样本重新锁定。
- 浮点极小正低通权重，Q 的最小正 raw alpha 与非精确 1 的 MAX alpha；宽未锁频率、数学正负端点导出、先饱和后除法，以及 state/output 原子性和多实例交错执行。
- 正反转下先有感 warmup，再经 `observable && locked` 接管；接管后 FOC 的反馈角只来自 SMO，电机真角仅用于物理模型和误差断言。

最后一项使用理想轴速源维持旋转，是**旋转时无感电流环接管仿真**，不是无感零速启动或无感机械速度全闭环实测。测试没有把真角偷偷继续送入“无感”控制。

Q 物理模型的显式测试基值分别为：电流过载场景 `Vbase=128 V, Ibase=32 A, Tbase=0.005 s`；速度场景 `64 V, 8 A, 0.005 s`；SMO/旋转接管 `32 V, 4 A, 0.005 s`。这些值只在 host oracle 中转换数据，不是生产状态的字段。参考模型使用实际量化的参数、信号和采样周期。Q 误差按绝对信号 LSB、固定 F31 运算舍入、观测器收缩率和低通增益分别计算，不能沿用带动态量程的相对误差预算。

### 已确认的 Q15 动态速度噪声边界

对本文 12→50→8 Hz 加减速 plant，保持电机、滑模增益和滤波系数不变，Q15 的电流采样量化经角差分放大。独立 double observer 使用**实际量化的配置、电压和电流**作数值参考；另一条对照只去除电流 sensor 量化，仍使用同一配置与实际施加的电压。已观察到以下正负转对称的结果（均值对应最终 8 Hz 平台）：

| Vbase / Ibase / Tbase | 32 V / 4 A / 0.005 s | 唯一额外工程候选：16 V / 2 A / 0.0064 s |
| --- | ---: | ---: |
| Q15 实现速度峰值误差 | 0.533496 Hz | 0.317164 Hz |
| 去除 sensor 量化后的参考峰值 | 0.091699 Hz | 0.091694 Hz |
| sensor 量化单独造成的峰值扰动 | 0.533498 Hz | 0.240274 Hz |
| 整数实现相对量化输入 reference 的最大误差 | 0.00002393 Hz | 0.00001058 Hz |
| 最后单拍误差 | 0.034133 Hz | 0.104364 Hz |
| 最后 1000 拍平均速度 | 7.999973 Hz | 7.999578 Hz |
| 模型使用的量化后采样周期 | 99.945068 us | 100 us（精确） |
| 实际输入电流最大绝对分量（pu） | 0.447205 | 0.894409 |
| 物理电流矢量峰值 / Ibase | 0.447225 | 0.894458 |
| 实际施加电压最大绝对分量（pu） | 0.116364 | 0.232697 |
| plant 电频峰值 `f*Tbase` | 0.250000 | 0.320000 |
| 实际配置 `min_speed/max_speed`（pu） | 0.010010 / 0.500000 | 0.012787 / 0.640015 |

以上实际输入域均经过断言；`Tag::make` 同时严格拒绝超 `[-1,+1]` 的测试信号。这里是 SMO-only 电压/电流观测测试，没有 DC bus 参数或速度参考输入，不虚构这两项。完整实际配置、输入峰值和 `meets_legacy_peak=false`、`meets_legacy_last_sample=false` 记录在测试 properties 中。

**两组 Q15 均未达到 0.25 Hz 的峰值误差目标，也未达到最后单拍 0.03 Hz 指标。** 均值良好不代替峰值合格结论。需要该峰值指标时选择 Q31/f32，不能简单调大测试容差、继续搜索基值或把 Q15 输入送浮点计算。

额外候选只按工程范围选择一次：目标 `Ts=100 us` 在 `Tbase=0.0064 s` 下恰好为 `dtpu=1/64`，`Rpu=.125/Lpu=.1953125/Kpu=.5` 均精确；实际峰值电流为 `1.788916 A`，测试断言 `<1.8 A`，证明相对 `Ibase=2 A` 仍有至少 10% 余量。它改善部分量化噪声但仍不满足上述峰值目标，未继续调基值或增益。

测试同时保留浮点原峰值阈值、Q31 峰值阈值、未量化 sensor 对照的原动态阈值，以及 Q 实现对量化输入 reference 的严格 F31 运算误差界；Q15 场景明确记录量化限制，不伪称性能合格。

本轮无指数实现的实际验证结果：

- C99 生产文件、C++17 测试以 `-Wall -Wextra -Wpedantic -Werror -Wvla` 编译通过；额外的 `-fsyntax-only` 检查通过。
- 独占临时目录内链接真实 linears 数值核心、fieldctrl 基础/FOC/SMO 和 filters 共用低通，`-O2` 运行 **88 个 GoogleTest 用例全部通过**（FOC 42 个、SMO 46 个），无跳过。
- 相同用例以 `-O1 -g -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer` 重新编译运行，**88 个全部通过**，未报告 sanitizer 错误。
- 上述通过包含量化噪声限制的诊断回归，**不代表 Q15 物理动态性能达到 0.25 Hz 峰值或 0.03 Hz 最后单拍指标**。峰值、参考误差及真实输入域也记录为 GoogleTest properties，可用 XML 输出查看。
- 未运行 SDK 根 Xmake，未创建提交；该独立验证不冒充 SDK 全量构建结果。

这些检查只属于 Linux 主机软件证据。没有实板、逆变器、电流采样或电机热/机械极限验证。SDK 统一构建/测试由上层集成流程运行；作者检查不替代硬件验证。

## 算法来源

1. Simon Strobl, imperix **TN111: Field oriented control (FOC) of permanent magnet synchronous machine (PMSM)**，2021。提供各向同性 PMSM 的 d/q 电流控制和电压模型背景。[公开文档](https://imperix.com/doc/implementation/field-oriented-control-of-pmsm)。本实现省略其解耦网络，不照搬其硬件性能结论。
2. imperix **Discrete PI controller implementation**。列出条件积分、反算等成熟抗饱和方法。[公开文档](https://imperix.com/doc/implementation/pi-controller)。本实现将条件积分的“继续推向饱和”判断扩展为二维圆形边界的径向点积；具体离散规则以上面的公式为准。
3. Simon Strobl, imperix **TN136: Sensorless motor control using a back-EMF Sliding-Mode Observer**，2021。说明电流误差注入、反电势提取、低通相移及低速不可观测性。[公开文档](https://imperix.com/doc/implementation/sensorless-motor-control)。本实现选择连续分段线性的 `sat` 边界层与角差分低通，不实现该文的 Q-PLL。

文献说明算法依据；库内定点表示、事务提交、离散相移公式和明确的方向/锁定契约由本实现及对应测试定义。参考文献中的实验结果不构成本仓库的实板证据。
