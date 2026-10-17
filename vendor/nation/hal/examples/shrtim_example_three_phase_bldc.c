/**
 * @file shrtim_example_three_phase_bldc.c
 * @brief Single-chip three-phase BLDC inverter configuration (6 PWM).
 *
 * Unit / switch mapping (fixed silicon pins):
 *
 * | Unit | Output | Pin  | Power switch      | Notes                          |
 * | ---- | ------ | ---- | ----------------- | ------------------------------ |
 * | C    | C1     | PB12 | Phase U high-side | 20 kHz CENTER, 50 %+200 ns DT  |
 * | C    | C2     | PB13 | Phase U low-side  |                                |
 * | D    | D1     | PB14 | Phase V high-side |                                |
 * | D    | D2     | PB15 | Phase V low-side  |                                |
 * | E    | E1     | PC8  | Phase W high-side |                                |
 * | E    | E2     | PC9  | Phase W low-side  |                                |
 *
 * FOC / SVPWM carrier configuration: three identical CENTER-aligned legs
 * at one frequency share the HAL-managed master anchor (all CENTER units
 * must run at the same frequency), so the three carriers start phase-
 * aligned within the startup residual (see the LLC examples; budget it
 * when volt-second symmetry across phases matters). Per-phase duties come
 * from the current loop; a real SVPWM modulation never reaches exactly
 * 0/1000. Two-shunt current sampling: ADC triggers on the nominal logical
 * midpoint (CMP3 = CMP1/2, dead-time and gate-drive uncompensated) of
 * phases U and V.
 *
 * Six-step (trapezoidal) variant: keep the same three legs, but commutate
 * each phase between "PWM", "low-side on" and "float" states per the
 * hall/BEMF sector, using shrtim_pwm_stop()/shrtim_pwm_start() plus
 * shrtim_out_force()/shrtim_out_enable() per leg - the sector logic is
 * application code on top of the same configuration.
 *
 * The mapping deliberately uses PB12..PB15 and PC8/PC9, leaving PA9/PA10
 * free for the USART1 console.
 */

/*
 * ============================== 中文说明 ==============================
 * 拓扑：单芯片三相 BLDC 逆变桥（6 路 PWM，单元 C/D/E），FOC/SVPWM 载波配置。
 *
 * 单元 / 开关映射：
 *   单元 C  C1(PB12) → U 相上管；C2(PB13) → U 相下管
 *   单元 D  D1(PB14) → V 相上管；D2(PB15) → V 相下管
 *   单元 E  E1(PC8)  → W 相上管；E2(PC9)  → W 相下管
 *          —— 20 kHz 中心对齐，固定 50% 载波 + 200 ns 死区
 *
 * 要点：
 *  - 三桥臂同频 CENTER 模式，共享 master 锚由 HAL 自动管理（CENTER 单元
 *    必须全部同频）；三路载波启动时相位对齐（存在启动相位残差，量级与
 *    LLC 示例相同，跨相伏秒敏感的应用需计入）。桥臂内不重叠由每单元
 *    死区硬件保证。
 *  - phase_duty() 由电流环每载波周期写入三相 SVPWM 占空比（1..999；
 *    真实调制不会触及 0/1000 端点）。在线改载波频率需停波或接受三腿
 *    顺序更新的过渡窗口。
 *  - 双电阻采样：U/V 相电流触发取名义逻辑脉宽中点（CMP3=CMP1/2，未补
 *    偿死区/驱动延迟，实际采样点由功率级校准）。
 *  - 六步梯形波变体：同样的三桥臂配置，按霍尔/BEMF 扇区用
 *    pwm_stop/start + out_force/out_enable 在“PWM/下管常开/悬空”间换相，
 *    扇区逻辑属应用层。
 *  - 重新配置（重新 init/改死区）时单元必须处于停止状态：死区使能与
 *    预分频/方向位同属运行锁存族，运行中重配不会生效（实测：运行中重
 *    新初始化的桥臂无死区窗口）。
 *  - 引脚刻意避开 PA9/PA10（USART1 控制台可用）。
 * =====================================================================
 */

#include "nation_shrtim.h"

#define PHASE_U_UNIT   SHRTIM_UNIT_C
#define PHASE_V_UNIT   SHRTIM_UNIT_D
#define PHASE_W_UNIT   SHRTIM_UNIT_E

#define CARRIER_HZ     20000u   /**< Carrier (switching) frequency.       */
#define CARRIER_MIN_HZ 8000u    /**< Carrier sweep lower bound.           */
#define CARRIER_MAX_HZ 40000u   /**< Carrier sweep upper bound.           */
#define LEG_DT_NS      200u     /**< Leg dead-time, both edges.           */
#define OCP_FAULT_CH   1u       /**< FLT1 pin: over-current comparator.   */

/**
 * @brief Point an ADC trigger at the nominal logical midpoint of one
 *        phase's x1 pulse.
 *
 * CMP1 is the x1 moving (turn-off) compare, so the ideal on-window spans
 * [set anchor, CMP1] and its nominal midpoint is CMP1/2 (dead-time and
 * gate-drive delay uncompensated - calibrate per power stage). The free
 * compare CMP3 is programmed with that value and routed to @p trig.
 * Verified register-level (demo T20: cmp3 == cmp1/2); the ADC routing
 * itself is only API-return-checked (as demo T14) and its electrical
 * trigger position is not LA-observable.
 *
 * @param[in] trig ADC trigger index 1..10.
 * @param[in] unit Phase unit whose current is sampled.
 * @return #SHRTIM_OK or #SHRTIM_ERR_PARAM.
 */
static int adc_midpoint_setup(uint8_t trig, shrtim_unit_t unit)
{
    shrtim_compare_raw(unit, 3u, shrtim_compare_get(unit, 1u) / 2u);
    return shrtim_adc_trigger(trig, unit, SHRTIM_EVT_CMP3);
}

/**
 * @brief Configure the three-phase bridge: pin muxing, CENTER carriers,
 *        OCP fault and the two-shunt current-sampling triggers. All units
 *        are left stopped.
 *
 * @note Re-configure units while STOPPED: like the prescaler and the
 *       counting direction, the dead-time enable does not re-latch on a
 *       running counter (LA-observed in the demo regression: a leg
 *       re-initialised while still running showed no dead-time gap).
 *
 * @return #SHRTIM_OK, or the first negative error code that failed.
 */
int shrtim_example_bldc_init(void)
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

    /* 三个桥臂（单元 C/D/E）：同频 CENTER 载波，master 锚由 HAL 自动管理。 */
    (void)shrtim_pinmux(SHRTIM_OUT_C1);
    (void)shrtim_pinmux(SHRTIM_OUT_C2);
    (void)shrtim_pinmux(SHRTIM_OUT_D1);
    (void)shrtim_pinmux(SHRTIM_OUT_D2);
    (void)shrtim_pinmux(SHRTIM_OUT_E1);
    (void)shrtim_pinmux(SHRTIM_OUT_E2);
    cfg.freq_hz    = CARRIER_HZ;
    cfg.duty_x1000 = 500u;                 /* 载波初始 50%，电流环再写 SVPWM */
    cfg.align      = SHRTIM_ALIGN_CENTER;
    cfg.dt_rise_ns = LEG_DT_NS;
    cfg.dt_fall_ns = LEG_DT_NS;
    rc = shrtim_pwm_init(PHASE_U_UNIT, &cfg);
    if (rc != SHRTIM_OK)
    {
        return rc;
    }
    rc = shrtim_pwm_init(PHASE_V_UNIT, &cfg);
    if (rc != SHRTIM_OK)
    {
        return rc;
    }
    rc = shrtim_pwm_init(PHASE_W_UNIT, &cfg);
    if (rc != SHRTIM_OK)
    {
        return rc;
    }

    /* 过流保护：任一故障关断三相桥臂。 */
    (void)shrtim_pinmux_fault(OCP_FAULT_CH);
    flt.active_high = false;               /* 过流比较器输出低有效 */
    flt.filter      = 3u;
    flt.latch       = true;                /* 锁存直到 fault_reset() */
    (void)shrtim_fault_config(OCP_FAULT_CH, &flt);
    (void)shrtim_fault_attach(PHASE_U_UNIT, 0x01u,
                              SHRTIM_FAULT_OUT_INACTIVE,
                              SHRTIM_FAULT_OUT_INACTIVE);
    (void)shrtim_fault_attach(PHASE_V_UNIT, 0x01u,
                              SHRTIM_FAULT_OUT_INACTIVE,
                              SHRTIM_FAULT_OUT_INACTIVE);
    (void)shrtim_fault_attach(PHASE_W_UNIT, 0x01u,
                              SHRTIM_FAULT_OUT_INACTIVE,
                              SHRTIM_FAULT_OUT_INACTIVE);
    /* 注：pinmux/fault 参数为固定合法常量，说明性忽略；pwm_init 可因
     * 时序求解失败，必须检查返回值。 */

    /* 双电阻采样：U/V 相电流，取名义逻辑脉宽中点。 */
    rc = adc_midpoint_setup(1u, PHASE_U_UNIT);
    if (rc != SHRTIM_OK)
    {
        return rc;
    }
    return adc_midpoint_setup(2u, PHASE_V_UNIT);
}

/**
 * @brief Start the three carriers (FOC current loop may then write duties).
 *
 * Right after the starts the inter-leg phase offset is still converging;
 * let the group settle before enabling the power stage (same residual
 * behaviour as the LLC examples).
 */
void shrtim_example_bldc_start(void)
{
    shrtim_pwm_start(PHASE_U_UNIT);
    shrtim_pwm_start(PHASE_V_UNIT);
    shrtim_pwm_start(PHASE_W_UNIT);
}

/** @brief Stop all three legs; pins go to their configured idle levels. */
void shrtim_example_bldc_stop(void)
{
    shrtim_pwm_stop(PHASE_U_UNIT);
    shrtim_pwm_stop(PHASE_V_UNIT);
    shrtim_pwm_stop(PHASE_W_UNIT);
}

/**
 * @brief Apply one SVPWM duty triple from the current loop.
 *
 * All three phases are updated one after another (transient window); a
 * current-loop ISR writing once per carrier period is the intended
 * serialisation. Degenerate 0/1000 are rejected: they switch the outputs
 * to fixed levels without rewriting CMP1 (HAL contract), which would
 * leave the sampling midpoints stale.
 *
 * @param[in] duty_u Phase U duty in 0.1 % steps (1..999).
 * @param[in] duty_v Phase V duty in 0.1 % steps (1..999).
 * @param[in] duty_w Phase W duty in 0.1 % steps (1..999).
 * @return #SHRTIM_OK, or #SHRTIM_ERR_PARAM when any duty is outside
 *         1..999.
 */
int shrtim_example_bldc_phase_duty(uint32_t duty_u, uint32_t duty_v,
                                   uint32_t duty_w)
{
    int rc;

    if ((duty_u == 0u) || (duty_u >= 1000u) ||
        (duty_v == 0u) || (duty_v >= 1000u) ||
        (duty_w == 0u) || (duty_w >= 1000u))
    {
        return SHRTIM_ERR_PARAM;           /* SVPWM 不会触及端点 */
    }
    rc = shrtim_pwm_duty(PHASE_U_UNIT, duty_u);
    if (rc != SHRTIM_OK)
    {
        return rc;
    }
    rc = shrtim_pwm_duty(PHASE_V_UNIT, duty_v);
    if (rc != SHRTIM_OK)
    {
        return rc;
    }
    rc = shrtim_pwm_duty(PHASE_W_UNIT, duty_w);
    if (rc != SHRTIM_OK)
    {
        return rc;
    }

    /* 刷新双电阻采样中点（名义值，随占空比移动）。 */
    rc = adc_midpoint_setup(1u, PHASE_U_UNIT);
    if (rc == SHRTIM_OK)
    {
        rc = adc_midpoint_setup(2u, PHASE_V_UNIT);
    }
    return rc;
}

/**
 * @brief Change the carrier frequency (e.g. audible-noise tuning).
 *
 * CENTER mode re-anchors the shared master inside pwm_freq per leg; the
 * three legs update one after another (transient window). Strictly
 * glitch-free changes must be made while stopped.
 *
 * @param[in] hz New carrier frequency in Hz (8..40 kHz).
 * @return #SHRTIM_OK, #SHRTIM_ERR_PARAM outside the band, or
 *         #SHRTIM_ERR_FREQ when unreachable.
 */
int shrtim_example_bldc_freq(uint32_t hz)
{
    int rc;

    if ((hz < CARRIER_MIN_HZ) || (hz > CARRIER_MAX_HZ))
    {
        return SHRTIM_ERR_PARAM;           /* 应用允许的载波范围限幅 */
    }
    rc = shrtim_pwm_freq(PHASE_U_UNIT, hz);
    if (rc == SHRTIM_OK)
    {
        rc = shrtim_pwm_freq(PHASE_V_UNIT, hz);
    }
    if (rc == SHRTIM_OK)
    {
        rc = shrtim_pwm_freq(PHASE_W_UNIT, hz);
    }
    if (rc == SHRTIM_OK)
    {
        rc = adc_midpoint_setup(1u, PHASE_U_UNIT);
    }
    if (rc == SHRTIM_OK)
    {
        rc = adc_midpoint_setup(2u, PHASE_V_UNIT);
    }
    return rc;
}
