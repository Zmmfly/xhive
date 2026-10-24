---
name: xhive-fieldctrl
description: 使用 xhive fieldctrl 的四版 PID、VVVF、FOC/速度环、SMO 与 DTC：核对坐标和采样时序，正确进行无指数 Q 格式与标幺换算，接入调用者状态、SDK 依赖及主机验证，明确硬件安全边界。
---

# xhive fieldctrl 使用技能

用于选型、编写调用代码、集成或排查 `libs/fieldctrl`；不是电机整定、板卡驱动或无感启动方案。
先读当前头文件和调用处，不套用旧 BFP 示例。本技能不扩大文件、网络、烧录或硬件权限。
以下链接均指向当前 checkout；不要求用户安装任何全局技能。

## 1. 入口与接口地图

统一包含 `#include "fieldctrl.h"`；接口名中的 `S` 替换为 `f32/f64/q15/q31`，不是可调用的泛型 API。

| 头文件 | 提供的能力 |
| --- | --- |
| [fieldctrl.h](../../../libs/fieldctrl/include/fieldctrl.h) | umbrella，包含下列全部组件 |
| [fieldctrl_types.h](../../../libs/fieldctrl/include/fieldctrl_types.h) | `abc/ab/dq` 类型、坐标、单位、宽 F31 诊断类型 |
| [fieldctrl_base.h](../../../libs/fieldctrl/include/fieldctrl_base.h) | wrap/sincos/atan2、Clarke/Park 及逆变换、PID（`pid_init/reset/retune/step`）、限压、SVPWM、VVVF |
| [fieldctrl_foc.h](../../../libs/fieldctrl/include/fieldctrl_foc.h) | `foc_init/step`、`speed_foc_init/step` |
| [fieldctrl_smo.h](../../../libs/fieldctrl/include/fieldctrl_smo.h) | `smo_init/step`、方向枚举及 observable/locked |
| [fieldctrl_dtc.h](../../../libs/fieldctrl/include/fieldctrl_dtc.h) | `dtc_init/step`、磁链/转矩估计和离散逆变器请求 |

- f32/f64：原生 `float/double`，按头文件使用 SI 单位，输入必须有限；不以浮点包装实现 Q。
- q15/q31：整数信号、显式标幺化；前者存储较小，后者信号分辨率更高，但都需量程和精度预算。
- FOC/SMO 的模型是平衡正弦反电势、非凸极 PMSM，SMO 用单一 `L=Ld=Lq`。
  不把它当成任意梯形 BLDC、凸极 IPMSM、磁饱和或弱磁模型。
- VVVF 是开环旋转电压场；FOC 需要可信角度；SMO 只在反电势可观测时提供接管诊断；DTC 是基本开关表控制。

## 2. 坐标、角度与频率先统一

- `abc` 是 a/b/c 相序；alpha 对齐 a 相，beta 比 alpha 正向超前 90 电角度。
- Clarke 为幅值不变形式：`alpha=(2a-b-c)/3`、`beta=(b-c)/sqrt(3)`，去除零序；逆变换产生平衡相量。
- Park：`d=alpha*cos+beta*sin`、`q=-alpha*sin+beta*cos`；d 轴对齐传入的转子磁链角。
- 角度是 **electrical turns**，一电周为 `2*pi rad`，正方向 alpha→beta，控制接口角度要求 `[0,1)`。
  `wrap` 可用于规范化，但不能替代相序、角度零点和符号标定。
- 浮点电频率是 electrical turn/s（Hz），不是 rad/s、RPM 或机械转速。
  `f_e=p*f_m`，`f_m=RPM/60`，电角度由机械角乘极对数并加实际标定偏置后 wrap。
- 速度级联输入是**机械频率命令 + 电频率反馈**；API 内部用反馈除 `pole_pairs`，不要在外部再除一次。
  Q 的两种频率都再乘同一 `Tbase`；速度 PI 的 SI 系数也必须以 turn/s 而非 RPM/rad/s 定义。

## 3. 状态生命周期、失败与并发

1. 调用者持有 context 和输出，成功 `init` 一次后按样本调用 `step`；不要每拍重新初始化。
2. 初始化后的成员由库维护，不直接写积分、相位、滤波历史或复制配置去“在线修改”状态。
   明确需要更换配置/初值时再重新初始化，并处理控制切换带来的状态不连续。
3. 独立 context 可重入；同一 context 或输出的访问需调用者串行化，包括线程与 ISR。
4. API 无动态分配、MMIO、阻塞等待或锁；ISR 中还需独占存储、正确采样时间戳与实际执行时间预算。
5. 输出不得与可变 context 重叠；多输出接口如 sincos 的目的存储也须互不重叠。
6. 错误不改变 context/output；**保留旧字节不等于旧命令仍安全**。失败后不得把旧输出当成本拍有效结果驱动硬件。
7. `LINEARS_OK` 仅代表本次软件计算成功。主动限压/PID 饱和可正常返回 OK；SMO 的有效性另看标志。

## 4. Q 格式与 per-unit：不能只把 f32 改名为 q15

`linears_q15_t=int16_t` 表示 raw/2^15，`linears_q31_t=int32_t` 表示 raw/2^31；没有 exponent。
信号范围分别为 `[-1,1-2^-15]`、`[-1,1-2^-31]`。
`linears_fixed_t=int64_t` 永远为 raw/2^31，范围 `[-2^32,2^32-2^-31]`，**q15 的宽系数也是 F31**。
`LINEARS_FIXED_ONE` 是原始整数 2147483648；增益 4 写成 `INT64_C(4) * LINEARS_FIXED_ONE`，不是 `4*32768`。

调用者选正的 `Vbase/Ibase/Tbase`，关联控制器/观测器使用一致基值；基值不存入库的 config/context。
先按表得到数值，再量化到参数声明的 Q1 信号或宽 F31 系数：

| 量 | 标幺数值 |
| --- | --- |
| 电压/母线 | `V/Vbase` |
| 电流/电流参考 | `I/Ibase` |
| 采样间隔 | `dt/Tbase` |
| 电频/机械频率 | `f*Tbase`，不乘 `2*pi` |
| 电阻 | `R*Ibase/Vbase` |
| 电感 | `L*Ibase/(Vbase*Tbase)` |
| 磁链 | `psi/(Vbase*Tbase)` |
| 转矩 | `torque/(Vbase*Ibase*Tbase)` |
| 电流 PI 比例/积分系数 | `kp_SI*Ibase/Vbase` / `ki_SI*Ibase*Tbase/Vbase` |
| 速度 PI 比例/积分系数 | `kp_SI/(Ibase*Tbase)` / `ki_SI/Ibase` |
| 通用 PID `kd` / 微分滤波时间常数 | `kd_SI*E/(U*Tbase)` / `Tf/Tbase`；E、U 为误差和输出基值 |

- 普通信号仍须 fit Q1：R/L、阈值、SMO sliding_gain、alpha、母线、电流、磁链/转矩输出都不能随意扩大到宽域。
  只有头文件明确声明的 PID/FOC 系数（含 PID 微分滤波时间常数和设定值权重）、状态和诊断使用宽 F31；合法分量经过旋转或求模仍可能超域。
- 量化前检查实际物理量和中间值范围，再做最近舍入，中点远离零；不能先强转窄整数后检查。
  负数不能用 `(int)(x+0.5)` 实现此规则，也不能左移负的有符号数。
  精确的 -0.25 Q15 可直接写 `-8192`；提升到 F31 用 `linears_fixed_from_q15(raw)`，不要 `raw << 16`。
- 已有 F31 值用 `linears_fixed_to_q15/q31(value, &out)` 并检查状态，详见
  [整数核心](../../../libs/linears/include/linears_numeric.h)；例如 `-LINEARS_FIXED_ONE/4` 安全表示宽 -0.25。
- `INT16_MAX/INT32_MAX` 输入表示 `1-LSB`，不是精确 1；尤其 MAX 低通权重不是完全直通。
  普通输出先检查真实工作值在 `[-1,+1]`，+1 或域内上端舍入到 +1 时映射到 MAX；真实超域返回 RANGE。
  Q15 **角度**是例外：上端半 LSB 舍入到整周时回到 0，不饱和成 MAX。
- `dt/Tbase` 量化后必须为正；不能把 50 Hz、24 V 等 SI 数值直接塞进 Q1，也不能忽略量化后真实采样间隔。
- Q15 的 F31 状态能累积 sub-Q15-LSB 增量；Q31 宽状态只增加整数余量，不增加小数位。
  小于半个 F31 LSB 的逐拍增量仍可能变零，没有隐藏的余数累加器。
- Q FOC `requested_voltage` 是宽 F31 的 `fieldctrl_fixed_dq_t`；Q SMO `electrical_speed` 也是宽 F31。
  后者进入 raw-Q `speed_foc_step` 前，先验证观测有效性，再显式调用对应 `linears_fixed_to_q*` 并检查结果。
  禁止强转截断；`requested_voltage` 即使可缩窄也不是实际施加电压，不能直接喂 SMO/DTC。

## 5. VVVF：完整 Q15 C99 调用例

流程是 `vvvf_init(ctx, initial_angle)`，然后每拍 `vvvf_step(ctx, frequency, amplitude, dc_bus, dt, out)`。
先推进相位，再产生该新相位的电压和占空比；要求 `abs(frequency)*dt <= 1/2` 电周。
频率可正负/为零；幅值须非负、母线和 dt 须正。零幅值给三路 0.5 duty，**不是门极关闭**。
线性 SVPWM 将幅值限制到 `Vdc/sqrt(3)`，`saturated` 表示限幅；V/f 曲线、斜坡、同步与限流由调用者负责。

以下是可单独编译的主机 consumer，不访问硬件。`Vbase=32 V`、`Tbase=0.005 s`，命令为电频 50 Hz、幅值 8 V、母线 24 V。
目标 100 us 得到 dt raw=655；实际表示 `655/32768*0.005 = 99.945068359375 us`，不是精确 100 us。
循环仅模拟四次 API 调用，不等待真实时间；实际系统须把一次 init 放在采样循环外。

```c
#include "fieldctrl.h"

int main(void)
{
    fieldctrl_q15_vvvf_t state;
    fieldctrl_q15_vvvf_output_t output;
    linears_status_t status = fieldctrl_q15_vvvf_init(&state, 0);
    if (status != LINEARS_OK) return 1;

    for (unsigned sample = 0; sample < 4; ++sample) {
        status = fieldctrl_q15_vvvf_step(&state,
            8192,   /* 50 Hz * 0.005 s = 0.25 */
            8192,   /* 8 V / 32 V = 0.25 */
            24576,  /* 24 V / 32 V = 0.75 */
            655,    /* quantized dt/Tbase, raw Q1.15 */
            &output);
        if (status != LINEARS_OK) return 2;
        if (sample == 0 && output.angle != 164) return 3;
        if (output.saturated || output.duty.a < 0 ||
            output.duty.b < 0 || output.duty.c < 0) return 4;
        /* Consume this sample only after success; no gate drive here. */
    }
    return 0;
}
```

对应回归见 [test/vvvf.cc](../../../libs/fieldctrl/test/vvvf.cc) 的 `VvvfPerUnit`。
这只演示编码、状态复用和命令输出，电场命令频率不是已测转子速度。

## 6. 基础 PID、FOC 与机械速度外环

- 基础 PID：`pid_init(&ctx,&config,integral)` 复制配置，`integral` 为初始积分（无扰切换）；
  `pid_step(&ctx,setpoint,measurement,dt,&out)` 输出限幅命令、P/I/D 各项与 `saturated`。
  `config` 零初始化即全部可选功能关闭；`kd`、微分源/滤波和 `FIELDCTRL_PID_OPT_*` 均需显式开启。
  配置无 setter/回调：`pid_retune(&ctx,&config)` 校验完整新配置后在线替换，保留积分（收进新积分范围）与微分历史，
  可每拍调用（随母线变化的限幅、增益调度）；仅 `kd=0` 清除已存 D 项，`ki`/`kd`/滤波时间常数变化才使 dt 缓存失效。
  `kp`/`ki`/`kd` 须同号：全非负为正作用，全非正为反作用（输出增大使测量减小），异号返回 DOMAIN；速度级联仍要求非负。
  `pid_reset` 只清微分历史并设定积分。
- 首拍（init/reset 后）只记录误差和测量，D=0；积分最多推进到使未限幅命令恰好落在限值上，不越过也不回拉。
  `ki*dt`、`kd/dt`、`dt/(Tf+dt)` 按 dt 缓存，dt 变化才重算；Q 版 `kd/dt` 超出 F31 宽域返回 RANGE。
- `fieldctrl_S_foc_init(&ctx, &config)` 接收 **config 指针**并复制；`step` 的测量输入是 **ab**，参考输入是 **dq**。
  若采集到 abc，先用同版本 Clarke。输入电流、转子磁链角、母线和 dt 必须时间一致。
- 电流 PI 的电压约束是 `min(config.voltage_limit,dc_bus/sqrt(3))` 的 d/q **联合圆形**边界，不是各轴单独裁剪。
  推向饱和的两轴积分增量一起拒绝；输出有未限幅请求、限幅电压和 duty，不能混用。
- 速度级联 `speed_foc_step(ctx,current,mechanical_reference,electrical_speed,electrical_angle,dc_bus,dt,out)`：
  外环生成受 `current_limit` 约束的 iq 参考，id=0；内环电压饱和也参与速度积分抗饱和。
- 每次调用两环都运行，**共用 dt、没有自动分频**。不同环频率需调用者分别调度基础 PID 与电流 FOC，保持参考并处理抗饱和反馈。
- 不提供解耦/反电势前馈、MTPA、弱磁、无感启动或板级故障状态机。

第二个完整 C99 consumer 使用 f32 接口和合成零电流，验证参数形状与状态生命周期。
这些 kp/ki/限压只是软件示例，不是任何真实电机的整定建议：

```c
#include "fieldctrl.h"

int main(void)
{
    const fieldctrl_f32_foc_config_t config = {
        .kp_d = 1.0f, .ki_d = 0.0f,
        .kp_q = 1.0f, .ki_q = 0.0f, .voltage_limit = 6.0f
    };
    const fieldctrl_f32_ab_t measured = {0.0f, 0.0f};
    const fieldctrl_f32_dq_t reference = {0.0f, 0.25f};
    fieldctrl_f32_foc_t state;
    fieldctrl_f32_foc_output_t output;
    linears_status_t status = fieldctrl_f32_foc_init(&state, &config);
    if (status != LINEARS_OK) return 1;

    for (unsigned sample = 0; sample < 4; ++sample) {
        status = fieldctrl_f32_foc_step(&state, measured, reference,
            0.0f, 24.0f, 0.0001f, &output); /* turns, volts, seconds */
        if (status != LINEARS_OK) return 2;
        if (output.saturated) return 3;
    }
    return 0;
}
```

## 7. SMO：采样时序、有效性和接管

`fieldctrl_S_smo_init(&ctx,&config,initial_current)`；随后 `smo_step(&ctx,current,voltage,&output)`。
采样周期在 config 中固定，step 不接受临时 dt；必须按 `config.dt` 调度。

- 离散约定是 **current=i[n]，voltage 为 `[n,n+1)` 的实际平均施加电压**；预测输出 `current_hat` 对应 n+1。
  从实际开关/占空比与实测母线重构电压并处理死区、压降、更新失效；不是未限幅 PI 请求。
- 角度/反电势输出属于当前观测时刻并有固有延迟；内置补偿仅针对低通的离散稳态相移，不能消除 ADC/PWM 延迟或加速度暂态。
- 初始化需 R≥0、L/dt/K/boundary>0、正 min_emf/lock_samples、`0<min_speed<max_speed`，并检查
  `0<dt*(R+K/boundary)/L<2`、`max_speed*dt<1/4`、低通 alpha 在 `(0,1]`（Q 的上限实际为 MAX<1）。
  这是局部 Euler/角步长必要检查，不是含噪声、模型误差、延迟的整机稳定保证。
- `LINEARS_OK` **不等于 observable 或 locked**。反电势过低时会保持旧角度并清速度历史/锁定，不是有效位置测量。
- observable 要求反电势达阈值、存在前一有效角度、原始和滤波频率方向/幅值均符合配置、两轴电流误差在 boundary 内。
  连续 `lock_samples` 拍才 locked；任一条件不满足就清锁。仍需外部一致性检查和安全接管/失锁策略。
- 方向显式为 `FIELDCTRL_SMO_FORWARD/REVERSE`；换向必须由外部启动/反转流程跨过零速，再重新 init，不能直接改 live config。

旋转接管的因果链：当前电流/本拍可用角度 → FOC → 本区间实际电压重构 → SMO → 下一拍角度预测。
若使用上一拍 SMO 角度，按实际时间戳用 **SMO 自身频率**外推并 wrap；这是调用者逻辑，不是隐含补偿。
Q 外推也需在正确 F31/turns 单位内运算，不能把宽诊断当 raw-Q 信号。
静止没有反电势；开环/有感启动、低速、失锁和换向方案均在库外，不宣称 sensorless standstill startup。

[当前 FOC/SMO 说明](../../../libs/fieldctrl/README_foc_smo.md) 记录了 Q15 动态量化限制：
已测两组基值都未达 0.25 Hz 峰值和 0.03 Hz 末拍误差目标；需要这些指标时用 Q31/f32。
与量化输入参考一致、均值良好或测试通过都不代表这些物理性能门槛达标，不用放宽容差掩盖限制。

## 8. DTC：实际电压、已知初始磁链与开关位

`fieldctrl_S_dtc_init(&ctx, config, initial_flux)` 的 **config 按值传递**，不要照搬 FOC 的 config 指针。
随后调用 `dtc_step(&ctx,current,applied_voltage,flux_reference,torque_reference,dt,&result)`。

1. 初始定子磁链必须已知且非零：由可信转子位置/电流模型或已验证对齐获得，不能任意置零或仅写目标幅值。
2. R≥0，两滞环带>0，`pole_pairs` 为整数 1..65535；`flux_reference>flux_band`、dt>0，转矩参考有正负。
3. `applied_voltage` 是**已过去 dt 区间的实际平均端电压**；电流需与该区间时间对齐，Euler 模型将其区间内保持不变。
   上次请求、`last_switch`、门极被禁止后的理想电压都不能冒充实测/校验重构值。
4. 先积分 `psi += dt*(v-R*i)`，再以新磁链计算转矩、扇区与滞环，成功后一起提交状态/结果。
   新磁链为零时 DOMAIN，Q 磁链模长/转矩超域时 RANGE，失败不会部分提交或自动给安全开关命令。
5. 下一采样区间是否执行返回请求由应用决定；电压反馈仍必须反映实际执行，包括母线变化、保护跳闸和更新丢失。

- `switch_state` 是 0..7：**bit2=a、bit1=b、bit0=c 上桥臂状态**；V1..V6 依次为 4、6、2、3、1、5。
- 0=000 和 7=111 是零矢量，不是占空比，也不等于硬件关断。死区、互锁、最小脉宽与保护由板级负责。
- 扇区 1..6 的中心为 `(sector-1)/6` turns，覆盖中心±1/12 的半开区间；不是从零开始的普通六等分索引。
- 转矩正/负误差可请求正/反向有功矢量；中性转矩状态才选零矢量。完整滞环/开关表见
  [DTC 说明](../../../libs/fieldctrl/README_dtc.md)，不要用自行猜的表替代。
- 这是纯电压积分器：传感器偏置、R 温漂、错误初值会累积/保留，没有自动漂移消除、低速可观测性或全工况稳定保证。
  重置磁链需要独立依据；Q 基值改变也会影响漂移，不能把某组软件模型精度推广为任意工况保证。

## 9. 接入应用与 SDK 主机验证

应用应已在 SDK 外按 [入门指南](../../../docs/Getting_Started_zh_CN.md) 接入 SDK 和 `xhive.embed`。
应用 `.config` 启用 `CONFIG_LIBS_ADD_FIELDCTRL=y`；[Kconfig](../../../libs/Kconfig) 会 select filters/linears。
这是 **Kconfig 求解并保存配置**的行为；SDK 的 Lua 解析只读显式行，不自动补 select/default。
手写配置时也显式写 `CONFIG_LIBS_ADD_FILTERS=y`、`CONFIG_LIBS_ADD_LINEARS=y`，或先经配置工具物化保存。

在实际固件 target 域添加直接依赖以传播公开头文件与链接依赖：

```lua
add_deps("xhive_embed::fieldctrl")
```

[fieldctrl/xmake.lua](../../../libs/fieldctrl/xmake.lua) 已 public 依赖 filters/linears，不要复制源码或重建一套私有数学实现。
SDK 根主机构建不读板卡 `.config`；使用 `xhive_host`，不是固件的 `xhive_embed`，也不验证板卡配置。
SDK 目录不明时先按用户指定路径/当前 checkout 确认，不硬编码用户名。以下 POSIX 命令从真实 SDK 根执行：

```sh
cd /absolute/path/to/xhive
xmake f -P . -y -m debug --xhive_tests=y --xhive_cross_tests=n
xmake b
xmake test -g xhive -v 'fieldctrl_test_*/*'
```

当前精确类别选择器与 [test/](../../../libs/fieldctrl/test/) 文件对应：

```sh
xmake test -g xhive -v 'fieldctrl_test_transforms/transforms' 'fieldctrl_test_pid_pwm/pid_pwm'
xmake test -g xhive -v 'fieldctrl_test_vvvf/vvvf'
xmake test -g xhive -v 'fieldctrl_test_foc/foc' 'fieldctrl_test_smo/smo'
xmake test -g xhive -v 'fieldctrl_test_dtc/dtc'
xmake test -g xhive -v 'fieldctrl_test_audit_*/*'
# 涉及公共数值核心、依赖或构建规则时扩大回归。
xmake test -g xhive -v
```

- `-g/-v` 均在选择器前，通配符加引号；`nothing to test` 不是通过。
- 普通 `xmake b` 不运行测试；测试目标非默认。区分 Xmake 入口数与 GoogleTest 用例数，不把历史数量当不变预期。
- 生产 C99，注册测试 C++17；GoogleTest/Eigen 3.4.0 是主机测试依赖，不进入固件。完整主机依赖和缓存隔离见
  [SDK README](../../../README.zh_CN.md)，不要为了这里的主机测试安装交叉工具链或运行 menuconfig。
- 直接主机 consumer 可用真实 `libs/{fieldctrl,filters,linears}/src/*.c`、三个 include 目录和宿主 `-lm` 编译链接；
  不把 consumer 烟测当全量数值测试。临时文件放 SDK 外的独占 `*-xhive-*` 目录，使用安全临时目录清理。

## 10. 交付前确认边界

- 参数整定需要真实 R/L、磁链、极对数、惯量、母线范围、采样延迟和噪声数据；缺这些就说明缺口，不编造电机参数。
- 本库不实现过流硬关断、欠压/过压、过温、ADC 偏置校准、门极互锁、看门狗或急停。
- 没有授权就不烧录、不连接串口/调试器、不运行电机；主机模型通过不是实板、电气或机械安全验证。
- ARM 编译/链接和单函数静态栈帧不是整条调用链、峰值 RAM、ISR 嵌套或 WCET 证明。
- 记录实际运行的命令、环境、结果/失败和未做项；使用当前 [库 README](../../../libs/fieldctrl/README.md) 与头文件作为调用契约。
