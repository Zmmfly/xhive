/**
 * @file shrtim_example_half_pfc_full_llc.c
 * @brief Single-chip combo: half-bridge PFC (2 PWM) + full-bridge LLC (4 PWM).
 *
 * Unit / switch mapping (fixed silicon pins):
 *
 * | Unit | Output | Pin  | Power switch            | Notes                     |
 * | ---- | ------ | ---- | ----------------------- | ------------------------- |
 * | A    | A1     | PA8  | PFC high-side MOSFET    | 65 kHz edge-aligned        |
 * | A    | A2     | PA9  | PFC low-side MOSFET     | 100 ns dead-time           |
 * | B    | B1     | PA10 | LLC Q1 (leg 1 high)     | CENTER mode, 50 %+150 ns DT|
 * | B    | B2     | PA11 | LLC Q3 (leg 1 low)      |                            |
 * | C    | C1     | PB12 | LLC Q2 (leg 2 high)     | CENTER + output SWAP       |
 * | C    | C2     | PB13 | LLC Q4 (leg 2 low)      |                            |
 *
 * LLC legs run CENTER-aligned: shrtim_pwm_init()/shrtim_pwm_freq() then
 * configure and keep following the shared master anchor internally
 * (hardware-validated path, see center_master_setup() in the HAL source),
 * so no hand-computed master period is needed here. The second leg is
 * output-swapped (shrtim_pwm_swap), which makes its drive anti-phase to
 * leg 1 and yields the diagonal conduction pairs Q1(B1)+Q4(C2) and
 * Q2(C1)+Q3(B2) (swap verified on hardware, demo test T08).
 *
 * PA9/PA10 double as USART1 pads. To reclaim BOTH console pins the PFC
 * must move too (e.g. PFC -> E/F, LLC -> C/D), because unit A drives PA9.
 * When remapping, the shrtim_pinmux() output routing and the
 * shrtim_fault_attach() unit arguments move with the units; the FLT1
 * input pin itself is fixed silicon and stays put (only a different
 * fault channel / board wiring would change it).
 */

/*
 * ============================== 中文说明 ==============================
 * 拓扑：半桥 PFC（2 路 PWM，单元 A）+ 全桥 LLC（4 路 PWM，单元 B/C）。
 *
 * 单元 / 开关映射：
 *   单元 A  A1(PA8) → PFC 上管；A2(PA9) → PFC 下管
 *          —— 65 kHz 边沿对齐互补 + 100 ns 死区，占空比由电流环在线调节
 *   单元 B  B1(PA10) → LLC Q1（桥臂 1 上管）；B2(PA11) → LLC Q3（桥臂 1 下管）
 *   单元 C  C1(PB12) → LLC Q2（桥臂 2 上管）；C2(PB13) → LLC Q4（桥臂 2 下管）
 *          —— 中心对齐（CENTER）固定 50% + 150 ns 死区，调频（PFM）稳输出
 *
 * 要点：
 *  - LLC 两桥臂用 CENTER 模式：pwm_init()/pwm_freq() 内部自动配置并跟随
 *    共享 master 锚周期（硬件实测验证的路径），示例不需要手写 master 周期。
 *    跨桥臂反相存在启动时确定的相位残差（实测 0~350ns，整轮恒定）；桥臂内
 *    不重叠由每单元死区保证。对伏秒对称敏感的功率级需计入该残差。
 *  - 第二桥臂经 shrtim_pwm_swap() 交换输出后与桥臂 1 反相，得到对角导通
 *    对 Q1(B1)+Q4(C2) / Q2(C1)+Q3(B2)（swap 已实测，见 demo T08）。
 *  - 约束：CENTER 模式共享同一个 master，所有 CENTER 单元必须同频（本例
 *    两腿始终同频）。PFC 为边沿模式且不锁相，不占用 master。
 *  - 在线调频：llc_freq() 依次更新两腿（每次都会搬 master 锚）；两次调用
 *    之间有短暂过渡窗口，失败回滚期间也不保证一致——“同频”是成功提交后
 *    的稳态约束。要求严格无毛刺时应在停波状态下改频。
 *  - PA9/PA10 兼作 USART1：要同时保留两个串口引脚，PFC 也要迁走（PFC→E/F、
 *    LLC→C/D 之类），且 pinmux/故障引脚需随单元一起迁移。
 * =====================================================================
 */

#include "nation_shrtim.h"

#define PFC_UNIT        SHRTIM_UNIT_A
#define LLC_LEG1_UNIT   SHRTIM_UNIT_B
#define LLC_LEG2_UNIT   SHRTIM_UNIT_C

#define PFC_FREQ_HZ     65000u   /**< PFC switching frequency.            */
#define PFC_DT_NS       100u     /**< PFC dead-time, both edges.          */
#define LLC_FREQ_HZ     100000u  /**< LLC nominal switching frequency.    */
#define LLC_FREQ_MIN_HZ 60000u   /**< LLC sweep lower bound (validated).  */
#define LLC_FREQ_MAX_HZ 150000u  /**< LLC sweep upper bound (soft start). */
#define LLC_DT_NS       150u     /**< LLC dead-time, both edges.          */
#define OCP_FAULT_CH    1u       /**< FLT1 pin: over-current comparator.  */

/**
 * @brief Point an ADC trigger at the nominal logical midpoint of the x1
 *        pulse.
 *
 * CMP1 is the x1 moving (turn-off) compare, so the ideal on-window spans
 * [set anchor, CMP1] and its nominal midpoint is CMP1/2. Dead-time shifts
 * the real turn-on edge and gate drivers add delay - the true electrical
 * sampling point must be calibrated per power stage. The free compare
 * CMP3 is programmed with that value and routed to @p trig. Must be
 * refreshed after every duty or frequency change (the hooks below do
 * this). Verified register-level (demo T20: cmp3 == cmp1/2); the ADC
 * routing itself is only API-return-checked (rc == 0, as demo T14) and
 * its electrical trigger position is not LA-observable.
 *
 * @param[in] trig ADC trigger index 1..10.
 * @param[in] unit Unit whose current is sampled.
 * @return #SHRTIM_OK or #SHRTIM_ERR_PARAM.
 */
static int adc_midpoint_setup(uint8_t trig, shrtim_unit_t unit)
{
    shrtim_compare_raw(unit, 3u, shrtim_compare_get(unit, 1u) / 2u);
    return shrtim_adc_trigger(trig, unit, SHRTIM_EVT_CMP3);
}

/**
 * @brief Configure pin muxing, PWM units, OCP fault and ADC triggers for
 *        the PFC + LLC combo. Units are left stopped.
 *
 * @return #SHRTIM_OK, or the first negative error code that failed.
 */
int shrtim_example_half_pfc_full_llc_init(void)
{
    shrtim_pwm_cfg_t cfg = SHRTIM_PWM_CFG_DEFAULT;
    shrtim_fault_cfg_t flt;
    int rc;

    if (shrtim_clk_hz() == 0u)
    {
        rc = shrtim_init(0u);              /* 0 -> default 240 MHz */
        if (rc != SHRTIM_OK)
        {
            return rc;
        }
    }

    /* --- PFC 桥臂（单元 A）：边沿对齐互补对，不锁相、不占 master --- */
    (void)shrtim_pinmux(SHRTIM_OUT_A1);
    (void)shrtim_pinmux(SHRTIM_OUT_A2);
    cfg.freq_hz    = PFC_FREQ_HZ;
    cfg.duty_x1000 = 100u;                 /* 软启动起始占空比 */
    cfg.dt_rise_ns = PFC_DT_NS;
    cfg.dt_fall_ns = PFC_DT_NS;
    rc = shrtim_pwm_init(PFC_UNIT, &cfg);
    if (rc != SHRTIM_OK)
    {
        return rc;
    }

    /* --- LLC（单元 B/C）：CENTER 模式两桥臂，master 锚由 HAL 自动管理 --- */
    (void)shrtim_pinmux(SHRTIM_OUT_B1);
    (void)shrtim_pinmux(SHRTIM_OUT_B2);
    (void)shrtim_pinmux(SHRTIM_OUT_C1);
    (void)shrtim_pinmux(SHRTIM_OUT_C2);
    cfg.freq_hz    = LLC_FREQ_HZ;
    cfg.duty_x1000 = 500u;                 /* LLC 固定 50% 占空比 */
    cfg.align      = SHRTIM_ALIGN_CENTER;  /* 锚周期随配置/调频自动更新 */
    cfg.dt_rise_ns = LLC_DT_NS;
    cfg.dt_fall_ns = LLC_DT_NS;
    rc = shrtim_pwm_init(LLC_LEG1_UNIT, &cfg);
    if (rc != SHRTIM_OK)
    {
        return rc;
    }
    rc = shrtim_pwm_init(LLC_LEG2_UNIT, &cfg);
    if (rc != SHRTIM_OK)
    {
        return rc;
    }
    /* 第二桥臂交换输出 -> 与桥臂 1 反相 -> 对角导通（实测：demo T08）。 */
    shrtim_pwm_swap(LLC_LEG2_UNIT, true);

    /* --- 过流保护：FLT1 低有效 + 锁存，所有桥臂故障时输出无效电平 --- */
    (void)shrtim_pinmux_fault(OCP_FAULT_CH);
    flt.active_high = false;               /* 过流比较器输出低有效 */
    flt.filter      = 3u;
    flt.latch       = true;                /* 锁存直到 fault_reset() */
    (void)shrtim_fault_config(OCP_FAULT_CH, &flt);
    (void)shrtim_fault_attach(PFC_UNIT, 0x01u,
                              SHRTIM_FAULT_OUT_INACTIVE,
                              SHRTIM_FAULT_OUT_INACTIVE);
    (void)shrtim_fault_attach(LLC_LEG1_UNIT, 0x01u,
                              SHRTIM_FAULT_OUT_INACTIVE,
                              SHRTIM_FAULT_OUT_INACTIVE);
    (void)shrtim_fault_attach(LLC_LEG2_UNIT, 0x01u,
                              SHRTIM_FAULT_OUT_INACTIVE,
                              SHRTIM_FAULT_OUT_INACTIVE);
    /* 注：pinmux/fault/ADC 参数均为固定合法常量，失败只可能来自参数
     * 错误，故说明性忽略 (void)；pwm_init 等可因时序求解失败，必须检查。 */

    /* 电流采样：PFC 电感电流 + LLC 谐振腔电流，均取导通窗口电中点。 */
    rc = adc_midpoint_setup(1u, PFC_UNIT);
    if (rc != SHRTIM_OK)
    {
        return rc;
    }
    return adc_midpoint_setup(2u, LLC_LEG1_UNIT);
}

/**
 * @brief Start both converters (LLC first, then the PFC).
 *
 * CENTER-mode units need no manual master handling: the anchor counter is
 * already configured and running from shrtim_pwm_init(). Cross-leg
 * anti-phase holds within a startup-dependent residual phase offset
 * (estimated 0..350 ns at 100 kHz from 200 ns-resolution LA samples over
 * finite windows - an observed range, not a guaranteed bound, constant
 * per run); within-leg non-overlap is guaranteed per-unit by the
 * dead-time generator. Budget the residual when the power stage cares
 * about volt-second symmetry.
 */
void shrtim_example_half_pfc_full_llc_start(void)
{
    shrtim_pwm_start(LLC_LEG1_UNIT);
    shrtim_pwm_start(LLC_LEG2_UNIT);
    shrtim_pwm_start(PFC_UNIT);
}

/**
 * @brief Stop all units; pins go to their configured idle levels.
 *
 * @note The shared master anchor keeps running by HAL design (other CENTER
 *       units may depend on it); it is re-anchored by the next pwm_init()
 *       or pwm_freq() call for this group.
 */
void shrtim_example_half_pfc_full_llc_stop(void)
{
    shrtim_pwm_stop(PFC_UNIT);
    shrtim_pwm_stop(LLC_LEG1_UNIT);
    shrtim_pwm_stop(LLC_LEG2_UNIT);
}

/**
 * @brief Apply a new PFC duty from the current/voltage control loop.
 *
 * Refreshes the PFC current-sampling midpoint afterwards.
 *
 * @param[in] duty_x1000 PFC x1 duty in 0.1 % steps.
 * @return #SHRTIM_OK, or #SHRTIM_ERR_PARAM outside 0..1000.
 */
int shrtim_example_half_pfc_full_llc_pfc_duty(uint32_t duty_x1000)
{
    int rc;

    /* Degenerate 0/1000 switch the outputs to fixed levels WITHOUT
     * rewriting CMP1 (HAL contract), so the midpoint helper would keep
     * refreshing a stale compare. Keep this sampling example inside
     * 1..999; note that even 1..999 does not guarantee a pulse wider
     * than the configured dead-time. */
    if ((duty_x1000 == 0u) || (duty_x1000 >= 1000u))
    {
        return SHRTIM_ERR_PARAM;
    }
    rc = shrtim_pwm_duty(PFC_UNIT, duty_x1000);
    if (rc == SHRTIM_OK)
    {
        rc = adc_midpoint_setup(1u, PFC_UNIT);
    }
    return rc;
}

/**
 * @brief Sweep both LLC legs to a new frequency (PFM output regulation).
 *
 * CENTER-mode pwm_freq() follows the master anchor internally for each
 * leg. The two legs are updated one after the other, so a short transient
 * window exists between the calls; change the frequency while stopped when
 * a strictly glitch-free transition is required.
 *
 * @param[in] hz New LLC switching frequency in Hz (60..150 kHz).
 * @return #SHRTIM_OK, #SHRTIM_ERR_PARAM outside the allowed band, or
 *         #SHRTIM_ERR_FREQ when unreachable.
 */
int shrtim_example_half_pfc_full_llc_llc_freq(uint32_t hz)
{
    int rc;

    if ((hz < LLC_FREQ_MIN_HZ) || (hz > LLC_FREQ_MAX_HZ))
    {
        return SHRTIM_ERR_PARAM;           /* 应用允许的调频范围限幅 */
    }
    rc = shrtim_pwm_freq(LLC_LEG1_UNIT, hz);
    if (rc == SHRTIM_OK)
    {
        rc = shrtim_pwm_freq(LLC_LEG2_UNIT, hz);
    }
    if (rc == SHRTIM_OK)
    {
        rc = adc_midpoint_setup(2u, LLC_LEG1_UNIT);
    }
    return rc;
}
