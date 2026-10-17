/**
 * @file shrtim_example_bridgeless_pfc.c
 * @brief Single-chip totem-pole bridgeless PFC (4 outputs).
 *
 * Unit / switch mapping (fixed silicon pins):
 *
 * | Unit | Output | Pin  | Power switch                     | Notes        |
 * | ---- | ------ | ---- | -------------------------------- | ------------ |
 * | A    | A1     | PA8  | Fast-leg high-side (GaN/SiC)     | 65 kHz       |
 * | A    | A2     | PA9  | Fast-leg low-side (GaN/SiC)      | 50 ns dead-t.|
 * | B    | B1     | PA10 | Slow-leg switch (grid +)         | line freq.   |
 * | B    | B2     | PA11 | Slow-leg switch (grid -)         | line freq.   |
 *
 * The fast leg is a normal complementary PWM pair whose duty is trimmed by
 * the current loop. The slow leg commutates at the mains frequency and is
 * driven with shrtim_out_force() from the grid polarity detection: the
 * SHRTIM counter floor is ~916 Hz at a 240 MHz kernel clock, so 50/60 Hz
 * PWM is not reachable (and hard line-rate switching is undesirable
 * anyway). Commutation is a break/make pair the caller separates by its
 * own validated blanking interval (device turn-off + driver delay); the
 * LA-verified shape is demo test T21: 20 ms both-off gap keeps the two
 * outputs strictly non-overlapping (0 simultaneous-high samples).
 *
 * Interleaving note: wiring a second fast leg's high-side gate to a
 * same-frequency unit's COMPLEMENT output only yields anti-phase drive at
 * exactly 50 % duty — it is NOT a 180 deg phase-shifted copy of an
 * arbitrary duty cycle. True interleaving needs a per-unit compare offset
 * PLUS matching set/reset event routing (in EDGE mode the set event stays
 * at the period, so moving CMP1 alone changes duty, not phase) - raw API
 * territory, not a single call.
 */

/*
 * ============================== 中文说明 ==============================
 * 拓扑：单芯片图腾柱无桥 PFC（4 路输出）。
 *
 * 单元 / 开关映射：
 *   单元 A  A1(PA8) → 快桥臂上管；A2(PA9) → 快桥臂下管
 *          —— 65 kHz 互补 + 50 ns 死区（GaN/SiC 器件），占空比由电流环调节
 *   单元 B  B1(PA10) → 慢桥臂（电网正半周）；B2(PA11) → 慢桥臂（电网负半周）
 *          —— 工频换向，由电网极性检测软件驱动
 *
 * 要点：
 *  - SHRTIM 计数器频率下限约 916 Hz（240 MHz 内核时钟），50/60 Hz PWM
 *    不可达，且工频硬开关不期望；慢桥臂用 shrtim_out_force() 在零点检测
 *    任务里换向。两路慢桥臂输出仍走 SHRTIM，故障保护与空闲电平语义统一。
 *  - 换向为 break/make 两步：先调 slow_leg_break() 将两路都置为无效电平，
 *    由调用方保持空白区（覆盖器件关断时间 + 驱动延迟，应用所有），
 *    再调 slow_leg_make() 闭合目标开关。LA 已验证形态见 demo T21：
 *    20ms 双断窗口下 F1/F2 严格互斥（0 重叠采样）。强制路径与
 *    fault/死区的硬件优先级需实板验证，不能假设硬件兜底。
 *  - 交错变体注意：把第二快桥臂上管接到同频单元的互补输出，只有在
 *    占空比恰为 50% 时才是反相驱动，不等于任意占空比的 180° 移相；真
 *    交错需要 compare 偏移（raw API），不能用极性取反替代。
 *  - PA9/PA10 兼作 USART1：需要串口时将桥臂整体迁移（pinmux 随单元走）。
 * =====================================================================
 */

#include "nation_shrtim.h"

#define FAST_UNIT      SHRTIM_UNIT_A
#define SLOW_UNIT      SHRTIM_UNIT_B
#define FAST_FREQ_HZ   65000u   /**< Fast-leg switching frequency.         */
#define FAST_DT_NS     50u      /**< Fast-leg dead-time (GaN-class).       */
#define OCP_FAULT_CH   1u       /**< FLT1 pin: over-current comparator.    */

/**
 * @brief Point ADC trigger 1 at the nominal logical midpoint of the
 *        fast-leg x1 pulse.
 *
 * CMP1 is the x1 turn-off compare, so the ideal on-window is [period,
 * CMP1] and its nominal midpoint is CMP1/2, programmed into the free
 * compare CMP3. Dead-time and gate-drive delay shift the true electrical
 * sampling point - calibrate per power stage. Verified register-level
 * (demo T20: cmp3 == cmp1/2); the ADC routing itself is only
 * API-return-checked (as demo T14) and its electrical trigger position
 * is not LA-observable.
 *
 * @return #SHRTIM_OK or #SHRTIM_ERR_PARAM.
 */
static int adc_midpoint_setup(void)
{
    shrtim_compare_raw(FAST_UNIT, 3u,
                       shrtim_compare_get(FAST_UNIT, 1u) / 2u);
    return shrtim_adc_trigger(1u, FAST_UNIT, SHRTIM_EVT_CMP3);
}

/**
 * @brief Configure pin muxing, the fast PWM leg, the forced slow leg and
 *        over-current protection. The fast unit is left stopped.
 *
 * @return #SHRTIM_OK, or the first negative error code that failed.
 */
int shrtim_example_bridgeless_pfc_init(void)
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

    /* 快桥臂（单元 A）：互补 PWM，GaN 级死区。 */
    (void)shrtim_pinmux(SHRTIM_OUT_A1);
    (void)shrtim_pinmux(SHRTIM_OUT_A2);
    cfg.freq_hz    = FAST_FREQ_HZ;
    cfg.duty_x1000 = 100u;                 /* 软启动起始占空比 */
    cfg.dt_rise_ns = FAST_DT_NS;
    cfg.dt_fall_ns = FAST_DT_NS;
    rc = shrtim_pwm_init(FAST_UNIT, &cfg);
    if (rc != SHRTIM_OK)
    {
        return rc;
    }

    /* 慢桥臂（单元 B）：复用为 SHRTIM 输出，在电网极性明确前保持无效
     * 电平；工频换向通过 shrtim_out_force() 软件驱动（单元保持停止）。 */
    (void)shrtim_pinmux(SHRTIM_OUT_B1);
    (void)shrtim_pinmux(SHRTIM_OUT_B2);
    rc = shrtim_pwm_init(SLOW_UNIT, &cfg);
    if (rc != SHRTIM_OK)
    {
        return rc;                         /* 与其它 pwm_init 同样检查 */
    }
    shrtim_out_force(SHRTIM_OUT_B1, SHRTIM_LEVEL_INACTIVE);
    shrtim_out_force(SHRTIM_OUT_B2, SHRTIM_LEVEL_INACTIVE);

    /* 过流保护：两桥臂故障时输出无效电平。 */
    (void)shrtim_pinmux_fault(OCP_FAULT_CH);
    flt.active_high = false;
    flt.filter      = 3u;
    flt.latch       = true;
    (void)shrtim_fault_config(OCP_FAULT_CH, &flt);
    (void)shrtim_fault_attach(FAST_UNIT, 0x01u,
                              SHRTIM_FAULT_OUT_INACTIVE,
                              SHRTIM_FAULT_OUT_INACTIVE);
    (void)shrtim_fault_attach(SLOW_UNIT, 0x01u,
                              SHRTIM_FAULT_OUT_INACTIVE,
                              SHRTIM_FAULT_OUT_INACTIVE);
    /* 注：pinmux/fault 参数为固定合法常量，说明性忽略；pwm_init 可因
     * 时序求解失败，必须检查返回值。 */

    return adc_midpoint_setup();
}

/** @brief Start the fast leg (call after the grid polarity is settled). */
void shrtim_example_bridgeless_pfc_start(void)
{
    shrtim_pwm_start(FAST_UNIT);
}

/** @brief Stop switching; both legs go to their inactive levels. */
void shrtim_example_bridgeless_pfc_stop(void)
{
    shrtim_pwm_stop(FAST_UNIT);
    shrtim_out_force(SHRTIM_OUT_B1, SHRTIM_LEVEL_INACTIVE);
    shrtim_out_force(SHRTIM_OUT_B2, SHRTIM_LEVEL_INACTIVE);
}

/**
 * @brief Apply a new fast-leg duty from the current-loop PI output.
 *
 * Refreshes the current-sampling midpoint afterwards.
 *
 * @param[in] duty_x1000 Fast-leg x1 duty in 0.1 % steps.
 * @return #SHRTIM_OK, or #SHRTIM_ERR_PARAM outside 0..1000.
 */
int shrtim_example_bridgeless_pfc_fast_duty(uint32_t duty_x1000)
{
    int rc;

    /* Degenerate 0/1000 keep a stale CMP1 (fixed-level outputs, HAL
     * contract); stay in 1..999 for this sampling example. */
    if ((duty_x1000 == 0u) || (duty_x1000 >= 1000u))
    {
        return SHRTIM_ERR_PARAM;
    }
    rc = shrtim_pwm_duty(FAST_UNIT, duty_x1000);
    if (rc == SHRTIM_OK)
    {
        rc = adc_midpoint_setup();
    }
    return rc;
}

/**
 * @brief Break: force both slow-leg outputs inactive (commutation start).
 *
 * Call this first and HOLD the both-off state for the application's
 * validated blanking interval before calling _make(). LA-verified shape
 * (demo T21): a 20 ms both-off gap keeps the two outputs strictly
 * non-overlapping (0 simultaneous-high samples over 1.2 M); the interval
 * itself is application-owned (device turn-off + driver delay).
 */
void shrtim_example_bridgeless_pfc_slow_leg_break(void)
{
    shrtim_out_force(SHRTIM_OUT_B1, SHRTIM_LEVEL_INACTIVE);
    shrtim_out_force(SHRTIM_OUT_B2, SHRTIM_LEVEL_INACTIVE);
}

/**
 * @brief Make: close the target slow-leg switch for the mains half-cycle.
 *
 * Only valid after slow_leg_break() plus the validated blanking interval;
 * the two switches are then never requested active together.
 *
 * @param[in] grid_positive true: positive mains half-cycle, false: negative.
 */
void shrtim_example_bridgeless_pfc_slow_leg_make(bool grid_positive)
{
    if (grid_positive)
    {
        shrtim_out_force(SHRTIM_OUT_B1, SHRTIM_LEVEL_ACTIVE);
    }
    else
    {
        shrtim_out_force(SHRTIM_OUT_B2, SHRTIM_LEVEL_ACTIVE);
    }
}
