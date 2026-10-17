/**
 * @file shrtim_example_pfc_llc_dcdc_chain.c
 * @brief Single-chip three-stage power chain: two-leg PFC + full-bridge
 *        LLC + synchronous-buck DC-DC (10 PWM, one unit spare).
 *
 * Unit / switch mapping (fixed silicon pins):
 *
 * | Stage | Unit | Output | Pin  | Role                     |
 * | ----- | ---- | ------ | ---- | ------------------------ |
 * | PFC   | A    | A1/A2  | PA8/PA9  | fast leg 1 (H/L)    |
 * | PFC   | B    | B1/B2  | PA10/PA11| fast leg 2 (H/L)    |
 * | BUCK  | C    | C1/C2  | PB12/PB13| sync buck (H/L)     |
 * | LLC   | D    | D1/D2  | PB14/PB15| leg 1 (Q1/Q3)       |
 * | LLC   | E    | E1/E2  | PC8/PC9  | leg 2 (Q2/Q4, SWAP) |
 * | spare | F    | F1/F2  | PC6/PC7  | free (or PFC slow legs via out_force) |
 *
 * PFC stage: two fast legs at the same frequency and duty (independent
 * parallel channels - NOT interleaved, see the accepted boundary; the
 * mains-frequency slow legs of a totem-pole PFC belong on GPIO or on the
 * spare unit F via shrtim_out_force()). LLC stage: CENTER-aligned D/E
 * pair (the hardware-verified diagonal pairing, demo T18/T23) with the
 * second leg output-swapped; pwm_init()/pwm_freq() manage the shared
 * master anchor automatically (all CENTER units must share one
 * frequency). NOTE on pairing: the D/E CENTER pair is the verified
 * diagonal in a dedicated two-unit context (demo T18); under FULL-CHAIN
 * conditions (edge units at other prescalers live around the shared
 * anchor) the inter-leg phase was boot-history dependent - measured
 * 0.98 %..12 % same-side overlap at 100 kHz across power cycles - and a
 * C+D pair measured a deterministic ~36 % offset. Root cause is not
 * isolated (suspected cross-prescaler coupling per the User Manual's
 * equal-prescaler warning): for a volt-second-critical full-bridge LLC,
 * verify the diagonal on each boot or budget the observed spread. Buck
 * stage: EDGE-aligned so it may run at a different frequency without
 * touching the LLC master anchor.
 *
 * Five units are used, so PA9/PA10 carry PWM and the console UART must
 * move elsewhere (or drop the PFC leg on unit B / the buck to free them).
 * Three ADC triggers sample PFC, LLC-tank and buck inductor currents at
 * their nominal logical pulse midpoints (CMP3 = CMP1/2). One OCP fault
 * input gates all five units.
 */

/*
 * ============================== 中文说明 ==============================
 * 拓扑：单芯片三级电源链——两桥臂 PFC（4 路 PWM，单元 A/B）+ 全桥 LLC
 * （4 路 PWM，单元 C/D）+ 同步降压 DC-DC（2 路 PWM，单元 E），共 10 路，
 * 单元 F 备用（或作 PFC 工频慢桥臂的 out_force 输出）。
 *
 * 单元 / 开关映射：
 *   单元 A  A1(PA8)/A2(PA9)   → PFC 快桥臂 1 上/下管（65 kHz EDGE）
 *   单元 B  B1(PA10)/B2(PA11) → PFC 快桥臂 2 上/下管（同频同占空比）
 *   单元 C  C1(PB12)/C2(PB13) → 同步降压上/下管（200 kHz EDGE + 20 ns 死区）
 *   单元 D  D1(PB14)/D2(PB15) → LLC 桥臂 1（100 kHz CENTER，Q1/Q3）
 *   单元 E  E1(PC8)/E2(PC9)   → LLC 桥臂 2（Q2/Q4，swap 后对角导通）
 *
 * 要点：
 *  - LLC 对用 D/E（专属双单元场景下已验证对角，demo T18）；但在全链条件
 *    （其它预分频档的 EDGE 单元围绕共享锚运行）下，跨腿相位随启动历史
 *    变化——同固件跨上电实测 0.98%~12% 同侧重叠（100 kHz），C+D 配对则
 *    实测确定性 ~36%。根因未定位（疑与用户手册“跨定时器事件需相等预分
 *    频比”警告有关）：伏秒敏感的全桥 LLC 应每次上电复核对角，或计入该
 *    偏差预算。
 *  - PFC 两桥臂为同频同占空比的独立并联通道（非交错；真交错需 compare
 *    偏置+事件路由，超出简化 API）。图腾柱的工频慢桥臂建议用 GPIO 或
 *    备用单元 F 的 out_force 输出。
 *  - LLC 对用 CENTER 模式共享 master 锚（HAL 自动管理，CENTER 单元必须
 *    同频）；降压级必须用 EDGE——不同频的 CENTER 单元会争抢共享锚。
 *  - 启动顺序 PFC → LLC → 降压（母线预充）；停止反向。
 *  - 三路电流采样（PFC/LLC 谐振腔/降压电感）均取名义逻辑脉宽中点。
 *  - 占用 5 个单元 → PA9/PA10 为 PWM 输出，串口需迁往其它 UART。
 *  - 重新配置任一级前先停该级（死区使能/预分频/方向均为运行锁存族，
 *    运行中重配不生效）。
 * =====================================================================
 */

#include "nation_shrtim.h"

#define PFC_LEG1_UNIT  SHRTIM_UNIT_A
#define PFC_LEG2_UNIT  SHRTIM_UNIT_B
#define BUCK_UNIT      SHRTIM_UNIT_C
#define LLC_LEG1_UNIT  SHRTIM_UNIT_D
#define LLC_LEG2_UNIT  SHRTIM_UNIT_E

#define PFC_FREQ_HZ    65000u   /**< PFC switching frequency.            */
#define PFC_DT_NS      50u      /**< PFC dead-time, both edges.          */
#define LLC_FREQ_HZ    100000u  /**< LLC nominal frequency.              */
#define LLC_FREQ_MIN_HZ 60000u  /**< LLC sweep lower bound.              */
#define LLC_FREQ_MAX_HZ 150000u /**< LLC sweep upper bound (soft start). */
#define LLC_DT_NS      150u     /**< LLC dead-time, both edges.          */
#define BUCK_FREQ_HZ   200000u  /**< Buck switching frequency.           */
#define BUCK_DT_NS     20u      /**< Buck dead-time, both edges.         */
#define OCP_FAULT_CH   1u       /**< FLT1 pin: over-current comparator.  */

/**
 * @brief Point an ADC trigger at the nominal logical midpoint of one
 *        unit's x1 pulse.
 *
 * CMP1 is the x1 turn-off compare, so the ideal on-window is [period,
 * CMP1] and its nominal midpoint is CMP1/2 (dead-time and gate-drive
 * delay uncompensated - calibrate per power stage). The free compare
 * CMP3 is programmed with that value and routed to @p trig. Verified
 * register-level (demo T20); the ADC routing itself is only
 * API-return-checked (as demo T14) and its electrical trigger position
 * is not LA-observable.
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
 * @brief Configure all three stages plus the shared OCP fault. All units
 *        are left stopped.
 *
 * @return #SHRTIM_OK, or the first negative error code that failed.
 */
int shrtim_example_chain_init(void)
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

    /* --- PFC：两路 EDGE 快桥臂（同频同占空比，非交错） --- */
    (void)shrtim_pinmux(SHRTIM_OUT_A1);
    (void)shrtim_pinmux(SHRTIM_OUT_A2);
    (void)shrtim_pinmux(SHRTIM_OUT_B1);
    (void)shrtim_pinmux(SHRTIM_OUT_B2);
    cfg.freq_hz    = PFC_FREQ_HZ;
    cfg.duty_x1000 = 100u;                 /* 软启动起始占空比 */
    cfg.align      = SHRTIM_ALIGN_EDGE;
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

    /* --- LLC：D/E CENTER 对 + E 腿 swap（已验证对角，锚自动管理）。
     * 先建 CENTER 对再配 EDGE 级。 --- */
    (void)shrtim_pinmux(SHRTIM_OUT_D1);
    (void)shrtim_pinmux(SHRTIM_OUT_D2);
    (void)shrtim_pinmux(SHRTIM_OUT_E1);
    (void)shrtim_pinmux(SHRTIM_OUT_E2);
    cfg.freq_hz    = LLC_FREQ_MAX_HZ;      /* 软启动：从最高频率起步 */
    cfg.duty_x1000 = 500u;
    cfg.align      = SHRTIM_ALIGN_CENTER;
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
    shrtim_pwm_swap(LLC_LEG2_UNIT, true);

    /* --- 降压级：EDGE（不同频，不得占用 CENTER 共享锚） --- */
    (void)shrtim_pinmux(SHRTIM_OUT_C1);
    (void)shrtim_pinmux(SHRTIM_OUT_C2);
    cfg.freq_hz    = BUCK_FREQ_HZ;
    cfg.duty_x1000 = 100u;
    cfg.align      = SHRTIM_ALIGN_EDGE;
    cfg.dt_rise_ns = BUCK_DT_NS;
    cfg.dt_fall_ns = BUCK_DT_NS;
    rc = shrtim_pwm_init(BUCK_UNIT, &cfg);
    if (rc != SHRTIM_OK)
    {
        return rc;
    }

    /* --- 共享过流保护：FLT1 关断全部五单元 --- */
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
    (void)shrtim_fault_attach(LLC_LEG1_UNIT, 0x01u,
                              SHRTIM_FAULT_OUT_INACTIVE,
                              SHRTIM_FAULT_OUT_INACTIVE);
    (void)shrtim_fault_attach(LLC_LEG2_UNIT, 0x01u,
                              SHRTIM_FAULT_OUT_INACTIVE,
                              SHRTIM_FAULT_OUT_INACTIVE);
    (void)shrtim_fault_attach(BUCK_UNIT, 0x01u,
                              SHRTIM_FAULT_OUT_INACTIVE,
                              SHRTIM_FAULT_OUT_INACTIVE);
    /* 注：pinmux/fault 参数为固定合法常量，说明性忽略；pwm_init 可因
     * 时序求解失败，必须检查返回值。 */

    /* 三级电流采样：PFC / LLC 谐振腔 / 降压电感。 */
    rc = adc_midpoint_setup(1u, PFC_LEG1_UNIT);
    if (rc != SHRTIM_OK)
    {
        return rc;
    }
    rc = adc_midpoint_setup(2u, LLC_LEG1_UNIT);
    if (rc != SHRTIM_OK)
    {
        return rc;
    }
    return adc_midpoint_setup(3u, BUCK_UNIT);
}

/**
 * @brief Start the chain in power sequence: PFC (bus precharge) first,
 *        then the LLC stage, then the buck.
 *
 * Let each stage settle before enabling the next (master-lock convergence
 * and soft-start ramps are application-controlled).
 */
void shrtim_example_chain_start(void)
{
    shrtim_pwm_start(PFC_LEG1_UNIT);
    shrtim_pwm_start(PFC_LEG2_UNIT);
    /* TODO(应用层): PFC 软启动/母线预充完成后，再启动后级。 */
    shrtim_pwm_start(LLC_LEG1_UNIT);
    shrtim_pwm_start(LLC_LEG2_UNIT);
    shrtim_pwm_start(BUCK_UNIT);
}

/** @brief Stop the chain in reverse order; pins go to idle levels. */
void shrtim_example_chain_stop(void)
{
    shrtim_pwm_stop(BUCK_UNIT);
    shrtim_pwm_stop(LLC_LEG1_UNIT);
    shrtim_pwm_stop(LLC_LEG2_UNIT);
    shrtim_pwm_stop(PFC_LEG1_UNIT);
    shrtim_pwm_stop(PFC_LEG2_UNIT);
}

/**
 * @brief Apply a new duty to both PFC legs (current loop).
 *
 * Both legs share the duty; degenerate 0/1000 keep a stale CMP1 (fixed
 * level path, HAL contract) and are rejected.
 *
 * @param[in] duty_x1000 PFC x1 duty in 0.1 % steps (1..999).
 * @return #SHRTIM_OK, or #SHRTIM_ERR_PARAM outside 1..999.
 */
int shrtim_example_chain_pfc_duty(uint32_t duty_x1000)
{
    int rc;

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
 * @brief Sweep the LLC pair to a new frequency (PFM regulation).
 *
 * CENTER-mode pwm_freq() follows the shared master anchor per leg; the
 * two legs update one after another (transient window). Strictly
 * glitch-free changes must be made while stopped. The buck stage (EDGE)
 * is unaffected.
 *
 * @param[in] hz New LLC frequency in Hz (60..150 kHz).
 * @return #SHRTIM_OK, #SHRTIM_ERR_PARAM outside the band, or
 *         #SHRTIM_ERR_FREQ when unreachable.
 */
int shrtim_example_chain_llc_freq(uint32_t hz)
{
    int rc;

    if ((hz < LLC_FREQ_MIN_HZ) || (hz > LLC_FREQ_MAX_HZ))
    {
        return SHRTIM_ERR_PARAM;
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

/**
 * @brief Apply a new buck duty (output voltage loop).
 *
 * @param[in] duty_x1000 Buck x1 duty in 0.1 % steps (1..999).
 * @return #SHRTIM_OK, or #SHRTIM_ERR_PARAM outside 1..999.
 */
int shrtim_example_chain_buck_duty(uint32_t duty_x1000)
{
    int rc;

    if ((duty_x1000 == 0u) || (duty_x1000 >= 1000u))
    {
        return SHRTIM_ERR_PARAM;
    }
    rc = shrtim_pwm_duty(BUCK_UNIT, duty_x1000);
    if (rc == SHRTIM_OK)
    {
        rc = adc_midpoint_setup(3u, BUCK_UNIT);
    }
    return rc;
}
