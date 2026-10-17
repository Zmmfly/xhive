/**
 * @file shrtim_example_full_pfc_half_llc.c
 * @brief Single-chip combo: two NON-INTERLEAVED PFC fast legs (4 PWM) +
 *        half-bridge LLC (2 PWM).
 *
 * (File name kept for history: an earlier revision labelled the PFC part
 * "full-bridge PFC"; see the topology note below for what it actually
 * configures.)
 *
 * Unit / switch mapping (fixed silicon pins):
 *
 * | Unit | Output | Pin  | Power switch              | Notes                    |
 * | ---- | ------ | ---- | ------------------------- | ------------------------ |
 * | A    | A1     | PA8  | PFC leg 1 high-side       | 65 kHz, 50 ns dead-time  |
 * | A    | A2     | PA9  | PFC leg 1 low-side        |                          |
 * | B    | B1     | PA10 | PFC leg 2 high-side       | same freq/duty as leg 1  |
 * | B    | B2     | PA11 | PFC leg 2 low-side        |                          |
 * | C    | C1     | PB12 | LLC high-side MOSFET      | 100 kHz, 50 %+150 ns DT  |
 * | C    | C2     | PB13 | LLC low-side MOSFET       |                          |
 *
 * The two PFC legs run identical complementary PWM at the same frequency
 * and duty: two INDEPENDENT, non-interleaved fast-leg gate patterns. This
 * is NOT a complete four-switch full-bridge PFC modulator - no cross-leg
 * phase relationship is established, the legs must each feed their own
 * inductor / converter channel, and the two midpoints must never be
 * hard-paralleled (the per-unit dead-time does not protect a cross-leg
 * short). Back-to-back starting leaves a constant but uncontrolled
 * inter-leg phase offset; do not read "same duty" as "co-phase".
 *
 * NOTE: a same-frequency unit's complement output is 1-D, so it only
 * equals a half-period shift at exactly 50 % duty; true 180 deg
 * interleaving at arbitrary duty needs a per-unit compare offset PLUS
 * matching set/reset event routing (in EDGE mode the set event stays at
 * the period, so moving CMP1 alone changes duty, not phase) - raw API
 * territory outside the simplified HAL.
 *
 * The half-bridge LLC is a single complementary pair (no cross-leg phase
 * requirement, hence no master lock needed) regulated by frequency sweep.
 * This example deliberately uses no master-timer APIs at all.
 *
 * PA9/PA10 double as USART1 pads: with a console UART on them, remap the
 * PFC legs (e.g. to E/F) and the LLC (e.g. to D), pinmux calls move with
 * the units.
 */

/*
 * ============================== 中文说明 ==============================
 * 拓扑：两路非交错 PFC 快桥臂（4 路 PWM，单元 A/B）+ 半桥 LLC（2 路 PWM，
 * 单元 C）。注意：这不是完整四开关全桥 PFC 调制——不建立跨桥臂相位关系，
 * 两路需各自接独立的电感/通道，严禁将两桥臂中点直接并联（每单元死区
 * 不保护跨桥短路）；文件名保留历史命名。
 *
 * 单元 / 开关映射：
 *   单元 A  A1(PA8) → PFC 桥臂 1 上管；A2(PA9) → 桥臂 1 下管
 *   单元 B  B1(PA10) → PFC 桥臂 2 上管；B2(PA11) → 桥臂 2 下管
 *          —— 65 kHz 互补 + 50 ns 死区；两桥臂同频同占空比
 *   单元 C  C1(PB12) → LLC 上管；C2(PB13) → LLC 下管
 *          —— 标称 100 kHz（软启动自 150 kHz 起步，范围 60..150 kHz），
 *             固定 50% + 150 ns 死区
 *
 * 要点：
 *  - PFC 两桥臂为同频同占空比的独立快桥臂门极模式：背靠背启动留下恒定
 *    但不受控的相位差，“同占空比”≠“同相”。这不是 180° 交错——同频单元的
 *    互补输出是 1-D，仅 50% 占空比时等效半周期移相；真交错需 compare 偏置
 *    加配套 set/reset 事件路由（EDGE 下仅改 CMP1 变占空比不变相位），
 *    属 raw API 范畴。
 *  - 半桥 LLC 只有一个桥臂，无跨桥臂相位需求，因此本例完全不使用
 *    master/锁相 API。
 *  - PA9/PA10 兼作 USART1：需要串口时 PFC 迁往 E/F、LLC 迁往 D（pinmux
 *    随单元一起迁移）。
 * =====================================================================
 */

#include "nation_shrtim.h"

#define PFC_LEG1_UNIT   SHRTIM_UNIT_A
#define PFC_LEG2_UNIT   SHRTIM_UNIT_B
#define LLC_UNIT        SHRTIM_UNIT_C

#define PFC_FREQ_HZ     65000u   /**< PFC switching frequency.             */
#define PFC_DT_NS       50u      /**< PFC dead-time, both edges.           */
#define LLC_FREQ_HZ     100000u  /**< LLC nominal switching frequency.     */
#define LLC_FREQ_MIN_HZ 60000u   /**< LLC sweep lower bound (validated).   */
#define LLC_FREQ_MAX_HZ 150000u  /**< LLC sweep upper bound (soft start).  */
#define LLC_DT_NS       150u     /**< LLC dead-time, both edges.           */
#define OCP_FAULT_CH    1u       /**< FLT1 pin: over-current comparator.   */

/**
 * @brief Point an ADC trigger at the nominal logical midpoint of a unit's
 *        x1 pulse.
 *
 * CMP1 is the x1 turn-off compare, so the ideal on-window is [period,
 * CMP1] and its nominal midpoint is CMP1/2, programmed into the free
 * compare CMP3. Dead-time and gate-drive delay shift the true electrical
 * point - calibrate per power stage. Must be refreshed after every duty
 * or frequency change. Verified register-level (demo T20: cmp3 ==
 * cmp1/2); the ADC routing itself is only API-return-checked (as demo
 * T14) and its electrical trigger position is not LA-observable.
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
 * @brief Configure pin muxing, both PFC legs, the LLC leg, OCP fault and
 *        ADC triggers. All units are left stopped.
 *
 * @return #SHRTIM_OK, or the first negative error code that failed.
 */
int shrtim_example_full_pfc_half_llc_init(void)
{
    shrtim_pwm_cfg_t cfg = SHRTIM_PWM_CFG_DEFAULT;
    shrtim_fault_cfg_t flt;
    int rc;

    if (shrtim_clk_hz() == 0u)
    {
        rc = shrtim_init(0u);
        if (rc != SHRTIM_OK)
        {
            return rc;
        }
    }

    /* PFC 桥臂（单元 A/B）：两路同频互补对，同频同占空比运行。
     * 真 180° 交错需 compare 偏置（简化 API 未封装，见文件头说明）。 */
    (void)shrtim_pinmux(SHRTIM_OUT_A1);
    (void)shrtim_pinmux(SHRTIM_OUT_A2);
    (void)shrtim_pinmux(SHRTIM_OUT_B1);
    (void)shrtim_pinmux(SHRTIM_OUT_B2);
    cfg.freq_hz    = PFC_FREQ_HZ;
    cfg.duty_x1000 = 100u;                 /* 软启动起始占空比 */
    cfg.dt_rise_ns = PFC_DT_NS;
    cfg.dt_fall_ns = PFC_DT_NS;
    rc = shrtim_pwm_init(PFC_LEG1_UNIT, &cfg);
    if (rc != SHRTIM_OK)
    {
        return rc;
    }
    rc = shrtim_pwm_init(PFC_LEG2_UNIT, &cfg);
    if (rc != SHRTIM_OK)
    {
        return rc;
    }

    /* LLC 半桥（单元 C）：一路互补对，固定 50% + 死区；单桥臂无跨桥相位
     * 需求，不使用 master 锁相。 */
    (void)shrtim_pinmux(SHRTIM_OUT_C1);
    (void)shrtim_pinmux(SHRTIM_OUT_C2);
    cfg.freq_hz    = LLC_FREQ_MAX_HZ;      /* 软启动：从最高频率起步 */
    cfg.duty_x1000 = 500u;
    cfg.dt_rise_ns = LLC_DT_NS;
    cfg.dt_fall_ns = LLC_DT_NS;
    rc = shrtim_pwm_init(LLC_UNIT, &cfg);
    if (rc != SHRTIM_OK)
    {
        return rc;
    }

    /* 过流保护：覆盖所有变换器桥臂。 */
    (void)shrtim_pinmux_fault(OCP_FAULT_CH);
    flt.active_high = false;
    flt.filter      = 3u;
    flt.latch       = true;
    (void)shrtim_fault_config(OCP_FAULT_CH, &flt);
    (void)shrtim_fault_attach(PFC_LEG1_UNIT, 0x01u,
                              SHRTIM_FAULT_OUT_INACTIVE,
                              SHRTIM_FAULT_OUT_INACTIVE);
    (void)shrtim_fault_attach(PFC_LEG2_UNIT, 0x01u,
                              SHRTIM_FAULT_OUT_INACTIVE,
                              SHRTIM_FAULT_OUT_INACTIVE);
    (void)shrtim_fault_attach(LLC_UNIT, 0x01u,
                              SHRTIM_FAULT_OUT_INACTIVE,
                              SHRTIM_FAULT_OUT_INACTIVE);
    /* 注：pinmux/fault 参数为固定合法常量，说明性忽略；pwm_init 可因
     * 时序求解失败，必须检查返回值。 */

    /* 两路电流采样：PFC 桥臂 1 电流 + LLC 谐振腔电流，均取名义逻辑脉宽
     * 中点（CMP1/2，未补偿死区/驱动延迟，实际采样位置需功率级校准）。 */
    rc = adc_midpoint_setup(1u, PFC_LEG1_UNIT);
    if (rc != SHRTIM_OK)
    {
        return rc;
    }
    return adc_midpoint_setup(2u, LLC_UNIT);
}

/**
 * @brief Start both converters (PFC legs back-to-back, then the LLC).
 *
 * Starting the two PFC units from the same stopped state leaves their
 * inter-leg phase offset constant but uncontrolled: the legs are
 * same-frequency and non-interleaved, with no controlled cross-leg phase
 * relationship.
 */
void shrtim_example_full_pfc_half_llc_start(void)
{
    shrtim_pwm_start(PFC_LEG1_UNIT);
    shrtim_pwm_start(PFC_LEG2_UNIT);
    shrtim_pwm_start(LLC_UNIT);
}

/** @brief Stop both converters; pins go to their configured idle levels. */
void shrtim_example_full_pfc_half_llc_stop(void)
{
    shrtim_pwm_stop(LLC_UNIT);
    shrtim_pwm_stop(PFC_LEG1_UNIT);
    shrtim_pwm_stop(PFC_LEG2_UNIT);
}

/**
 * @brief Apply a new duty to both PFC legs (current loop).
 *
 * Both legs always share the duty so their (constant) phase offset is
 * preserved. The PFC sampling midpoint is refreshed afterwards.
 *
 * @param[in] duty_x1000 PFC x1 duty in 0.1 % steps (1..999; the range
 *                  does not guarantee a pulse wider than the dead-time).
 * @return #SHRTIM_OK, or #SHRTIM_ERR_PARAM outside 1..999.
 */
int shrtim_example_full_pfc_half_llc_pfc_duty(uint32_t duty_x1000)
{
    int rc;

    /* Degenerate 0/1000 keep a stale CMP1 (fixed-level outputs, HAL
     * contract); stay in 1..999 for this sampling example. */
    if ((duty_x1000 == 0u) || (duty_x1000 >= 1000u))
    {
        return SHRTIM_ERR_PARAM;
    }
    rc = shrtim_pwm_duty(PFC_LEG1_UNIT, duty_x1000);

    if (rc == SHRTIM_OK)
    {
        rc = shrtim_pwm_duty(PFC_LEG2_UNIT, duty_x1000);
    }
    if (rc == SHRTIM_OK)
    {
        rc = adc_midpoint_setup(1u, PFC_LEG1_UNIT);
    }
    return rc;
}

/**
 * @brief Sweep the LLC to a new frequency (PFM output regulation).
 *
 * The sampling midpoint is refreshed afterwards. Change the frequency
 * while stopped for a strictly glitch-free transition.
 *
 * @param[in] hz New LLC switching frequency in Hz (60..150 kHz).
 * @return #SHRTIM_OK, #SHRTIM_ERR_PARAM outside the allowed band, or
 *         #SHRTIM_ERR_FREQ when unreachable.
 */
int shrtim_example_full_pfc_half_llc_llc_freq(uint32_t hz)
{
    int rc;

    if ((hz < LLC_FREQ_MIN_HZ) || (hz > LLC_FREQ_MAX_HZ))
    {
        return SHRTIM_ERR_PARAM;           /* 应用允许的调频范围限幅 */
    }
    rc = shrtim_pwm_freq(LLC_UNIT, hz);
    if (rc == SHRTIM_OK)
    {
        rc = adc_midpoint_setup(2u, LLC_UNIT);
    }
    return rc;
}
