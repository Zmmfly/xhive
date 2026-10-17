/**
 * @file shrtim_example_full_llc.c
 * @brief Single-chip full-bridge LLC resonant converter (4 PWM).
 *
 * Unit / switch mapping (fixed silicon pins):
 *
 * | Unit | Output | Pin  | Power switch       | Notes                      |
 * | ---- | ------ | ---- | ------------------ | -------------------------- |
 * | A    | A1     | PA8  | Q1 (leg 1 high)    | CENTER mode, 50 %+150 ns DT|
 * | A    | A2     | PA9  | Q3 (leg 1 low)     |                            |
 * | B    | B1     | PA10 | Q2 (leg 2 high)    | CENTER + output SWAP       |
 * | B    | B2     | PA11 | Q4 (leg 2 low)     |                            |
 *
 * Both legs run CENTER-aligned at a fixed 50 % duty; leg 2 is output-swapped
 * (shrtim_pwm_swap), making it anti-phase to leg 1: the diagonal pairs
 * Q1(A1)+Q4(B2) and Q2(B1)+Q3(A2) conduct alternately (swap verified on
 * hardware, demo test T08). Output regulation is pure PFM; pwm_init() and
 * pwm_freq() manage the shared master anchor internally (validated path,
 * see center_master_setup() in the HAL source), so no hand-computed master
 * period appears in this example.
 *
 * PA9/PA10 double as USART1 pads — move both legs (e.g. to C/D or D/E,
 * pinmux calls move with them) when a console UART is required.
 */

/*
 * ============================== 中文说明 ==============================
 * 拓扑：单芯片全桥 LLC 谐振变换器（4 路 PWM，单元 A/B）。
 *
 * 单元 / 开关映射：
 *   单元 A  A1(PA8) → Q1（桥臂 1 上管）；A2(PA9) → Q3（桥臂 1 下管）
 *   单元 B  B1(PA10) → Q2（桥臂 2 上管）；B2(PA11) → Q4（桥臂 2 下管）
 *          —— 中心对齐（CENTER）固定 50% + 150 ns 死区，60..150 kHz 调频
 *
 * 要点：
 *  - 第二桥臂经 shrtim_pwm_swap() 交换输出后与桥臂 1 反相，得到对角导通
 *    对 Q1(A1)+Q4(B2) / Q2(B1)+Q3(A2)（swap 已实测，见 demo T08）。
 *  - CENTER 模式下 pwm_init()/pwm_freq() 内部自动配置并跟随共享 master
 *    锚周期（硬件实测路径），示例不手写 master 周期；约束：CENTER 单元
 *    必须同频（两腿始终同频，满足）。
 *  - 在线调频存在两腿先后更新的过渡窗口；严格无毛刺需求停波改频。
 *  - 软启动从最高频率（150 kHz）起步下降；调频钩子做了范围限幅。
 *  - PA9/PA10 兼作 USART1：需要串口时两桥臂整体迁移（pinmux 随单元走）。
 * =====================================================================
 */

#include "nation_shrtim.h"

#define LLC_LEG1_UNIT   SHRTIM_UNIT_A
#define LLC_LEG2_UNIT   SHRTIM_UNIT_B

#define LLC_FREQ_HZ     100000u  /**< Nominal (resonant) frequency.       */
#define LLC_FREQ_MAX_HZ 150000u  /**< Sweep upper bound (soft start).     */
#define LLC_FREQ_MIN_HZ 60000u   /**< Sweep lower bound (validated).      */
#define LLC_DT_NS       150u     /**< Dead-time, both edges.              */
#define OCP_FAULT_CH    1u       /**< FLT1 pin: over-current comparator.  */

/** 当前开关频率（两桥臂共享）；HAL 不提供周期读回，故本地记录。
 *  (Tracked locally because the HAL exposes no period read-back.) */
static uint32_t s_llc_freq_hz = LLC_FREQ_MAX_HZ;

/**
 * @brief Point ADC trigger 1 at the nominal logical midpoint of leg 1's
 *        x1 pulse.
 *
 * CMP1 is the moving (turn-off) compare, so the ideal on-window is
 * [set anchor, CMP1] and its nominal midpoint is CMP1/2, programmed into
 * the free compare CMP3. Dead-time and gate-drive delay shift the true
 * electrical point - calibrate per power stage. Verified register-level
 * (demo T20: cmp3 == cmp1/2); the ADC routing itself is only
 * API-return-checked (as demo T14) and its electrical trigger position is
 * not LA-observable.
 *
 * @return #SHRTIM_OK or #SHRTIM_ERR_PARAM.
 */
static int adc_midpoint_setup(void)
{
    shrtim_compare_raw(LLC_LEG1_UNIT, 3u,
                       shrtim_compare_get(LLC_LEG1_UNIT, 1u) / 2u);
    return shrtim_adc_trigger(1u, LLC_LEG1_UNIT, SHRTIM_EVT_CMP3);
}

/**
 * @brief Configure both LLC legs (CENTER mode), the leg-2 output swap,
 *        OCP fault and the tank-current ADC trigger. Units stay stopped.
 *
 * @return #SHRTIM_OK, or the first negative error code that failed.
 */
int shrtim_example_full_llc_init(void)
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

    (void)shrtim_pinmux(SHRTIM_OUT_A1);
    (void)shrtim_pinmux(SHRTIM_OUT_A2);
    (void)shrtim_pinmux(SHRTIM_OUT_B1);
    (void)shrtim_pinmux(SHRTIM_OUT_B2);

    cfg.freq_hz    = LLC_FREQ_MAX_HZ;      /* 软启动：从最高频率起步 */
    cfg.duty_x1000 = 500u;                 /* LLC 恒为 50% 占空比 */
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

    /* 过流保护：任一故障关断全部四管。 */
    (void)shrtim_pinmux_fault(OCP_FAULT_CH);
    flt.active_high = false;
    flt.filter      = 3u;
    flt.latch       = true;
    (void)shrtim_fault_config(OCP_FAULT_CH, &flt);
    (void)shrtim_fault_attach(LLC_LEG1_UNIT, 0x01u,
                              SHRTIM_FAULT_OUT_INACTIVE,
                              SHRTIM_FAULT_OUT_INACTIVE);
    (void)shrtim_fault_attach(LLC_LEG2_UNIT, 0x01u,
                              SHRTIM_FAULT_OUT_INACTIVE,
                              SHRTIM_FAULT_OUT_INACTIVE);
    /* 注：pinmux/fault 参数为固定合法常量，说明性忽略；可失败的时序求解
     * （pwm_init）必须检查返回值。 */

    rc = adc_midpoint_setup();
    if (rc != SHRTIM_OK)
    {
        return rc;
    }
    s_llc_freq_hz = LLC_FREQ_MAX_HZ;       /* 重新初始化后同步软件缓存 */
    return SHRTIM_OK;
}

/**
 * @brief Start both legs.
 *
 * CENTER mode needs no manual master handling: the anchor counter runs
 * from shrtim_pwm_init() and both units lock to it (demo T03/T04 pattern).
 * Cross-leg anti-phase holds within a startup-dependent residual phase
 * offset (estimated 0..350 ns at 100 kHz from 200 ns-resolution LA
 * samples over finite windows - an observed range, not a guaranteed
 * bound, constant per run); within-leg non-overlap is guaranteed
 * per-unit by the dead-time generator.
 */
void shrtim_example_full_llc_start(void)
{
    shrtim_pwm_start(LLC_LEG1_UNIT);
    shrtim_pwm_start(LLC_LEG2_UNIT);
}

/**
 * @brief Stop the converter; pins go to their configured idle levels.
 *
 * @note The shared master anchor keeps running by HAL design and is
 *       re-anchored by the next pwm_init()/pwm_freq() of this group.
 */
void shrtim_example_full_llc_stop(void)
{
    shrtim_pwm_stop(LLC_LEG1_UNIT);
    shrtim_pwm_stop(LLC_LEG2_UNIT);
}

/**
 * @brief Sweep the converter to a new frequency (PFM output regulation).
 *
 * CENTER-mode pwm_freq() follows the master anchor internally per leg; the
 * two legs update one after the other (short transient window in between —
 * change frequency while stopped for a strictly glitch-free transition).
 * Whether lower frequency means more power depends on the operating region
 * relative to the resonant point of the actual tank.
 *
 * @param[in] hz New switching frequency in Hz (60..150 kHz).
 * @return #SHRTIM_OK, #SHRTIM_ERR_PARAM outside the allowed band, or
 *         #SHRTIM_ERR_FREQ when unreachable.
 */
int shrtim_example_full_llc_freq_set(uint32_t hz)
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
        rc = adc_midpoint_setup();
    }
    if (rc == SHRTIM_OK)
    {
        s_llc_freq_hz = hz;                /* 只有全部成功才提交缓存 */
    }
    return rc;
}

/**
 * @brief Read back the last fully committed switching frequency.
 *
 * This is the bookkeeping value of the last request where BOTH legs (and
 * the shared anchor) were updated successfully - not a live measurement,
 * and during the transient window or after a failed/rolled-back request
 * the legs may not match it. Callers must serialise control updates and
 * must not let another CENTER-mode user re-configure the shared master at
 * a different frequency.
 *
 * @return Last committed frequency in Hz.
 */
uint32_t shrtim_example_full_llc_freq_get(void)
{
    return s_llc_freq_hz;
}
