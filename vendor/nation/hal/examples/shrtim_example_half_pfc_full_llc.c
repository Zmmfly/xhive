/**
 * @file shrtim_example_half_pfc_full_llc.c
 * @brief Single-chip combo: half-bridge PFC (2 PWM) + full-bridge LLC (4 PWM).
 *
 * Unit / switch mapping (fixed silicon pins):
 *
 * | Unit | Output | Pin  | Power switch            | Notes                     |
 * | ---- | ------ | ---- | ----------------------- | ------------------------- |
 * | A    | A1     | PA8  | PFC high-side MOSFET    | 65 kHz complementary pair |
 * | A    | A2     | PA9  | PFC low-side MOSFET     | 100 ns dead-time          |
 * | B    | B1     | PA10 | LLC Q1 (leg 1 high)     | 100..150 kHz, 50 %+DT     |
 * | B    | B2     | PA11 | LLC Q3 (leg 1 low)      |                           |
 * | C    | C1     | PB12 | LLC Q2 (leg 2 high)     |                           |
 * | C    | C2     | PB13 | LLC Q4 (leg 2 low)      |                           |
 *
 * Both LLC legs run in phase, so the diagonal conduction pairs come from
 * the complementary x2 outputs: Q1(B1)+Q4(C2) conduct together, then
 * Q2(C1)+Q3(B2). No phase-shift API is needed.
 *
 * PA9/PA10 double as USART1 pads: when the console UART is needed on those
 * pins, move the LLC legs to other free units (e.g. D/E) in the mapping.
 *
 * Runtime hooks:
 *  - PFC duty is trimmed by the current/voltage control loop.
 *  - LLC output regulation is done by frequency sweep (both legs must
 *    always share one frequency).
 */

#include "nation_shrtim.h"

#define PFC_UNIT        SHRTIM_UNIT_A
#define LLC_LEG1_UNIT   SHRTIM_UNIT_B
#define LLC_LEG2_UNIT   SHRTIM_UNIT_C

#define PFC_FREQ_HZ     65000u   /**< PFC switching frequency.            */
#define PFC_DT_NS       100u     /**< PFC dead-time, both edges.          */
#define LLC_FREQ_HZ     100000u  /**< LLC nominal switching frequency.    */
#define LLC_FREQ_MIN_HZ 60000u   /**< LLC sweep lower bound.              */
#define LLC_FREQ_MAX_HZ 150000u  /**< LLC sweep upper bound.              */
#define LLC_DT_NS       150u     /**< LLC dead-time, both edges.          */
#define OCP_FAULT_CH    1u       /**< FLT1 pin: over-current comparator.  */

/**
 * @brief Configure pin muxing, PWM units, master sync, OCP fault and ADC
 *        trigger for the PFC + LLC combo. Units are left stopped.
 *
 * @return #SHRTIM_OK, or the first negative error code that failed.
 */
int shrtim_example_half_pfc_full_llc_init(void)
{
    shrtim_pwm_cfg_t cfg = SHRTIM_PWM_CFG_DEFAULT;
    shrtim_fault_cfg_t flt;
    uint32_t master_ticks;
    int rc;

    if (shrtim_clk_hz() == 0u)
    {
        rc = shrtim_init(0u);              /* 0 -> default 240 MHz */
        if (rc != SHRTIM_OK)
        {
            return rc;
        }
    }

    /* --- PFC leg: one complementary pair, duty controlled at runtime --- */
    (void)shrtim_pinmux(SHRTIM_OUT_A1);
    (void)shrtim_pinmux(SHRTIM_OUT_A2);
    cfg.freq_hz    = PFC_FREQ_HZ;
    cfg.duty_x1000 = 100u;                 /* soft-start pedestal          */
    cfg.dt_rise_ns = PFC_DT_NS;
    cfg.dt_fall_ns = PFC_DT_NS;
    rc = shrtim_pwm_init(PFC_UNIT, &cfg);
    if (rc != SHRTIM_OK)
    {
        return rc;
    }

    /* --- LLC: two identical 50 % legs, phase-locked to the master ------ */
    (void)shrtim_pinmux(SHRTIM_OUT_B1);
    (void)shrtim_pinmux(SHRTIM_OUT_B2);
    (void)shrtim_pinmux(SHRTIM_OUT_C1);
    (void)shrtim_pinmux(SHRTIM_OUT_C2);
    cfg.freq_hz    = LLC_FREQ_HZ;
    cfg.duty_x1000 = 500u;                 /* LLC is fixed 50 %            */
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
    shrtim_pwm_sync_to_master(LLC_LEG1_UNIT, true);
    shrtim_pwm_sync_to_master(LLC_LEG2_UNIT, true);

    /* --- Over-current protection on all legs --------------------------- */
    (void)shrtim_pinmux_fault(OCP_FAULT_CH);
    flt.active_high = false;               /* OCP comparator: active low   */
    flt.filter      = 3u;
    flt.latch       = true;                /* stays latched until reset    */
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

    /* Sample the PFC current at the middle of the on-time. */
    (void)shrtim_adc_trigger(1u, PFC_UNIT, SHRTIM_EVT_CMP1);

    master_ticks = shrtim_clk_hz() / LLC_FREQ_HZ - 1u;
    shrtim_master_period(master_ticks, false);   /* arm, do not start yet  */
    return SHRTIM_OK;
}

/**
 * @brief Release the phase-locked start: run the LLC legs, then the PFC.
 *
 * The master counter is started last so both LLC legs begin their period
 * at the same edge (sequence hardware-verified in the demo project).
 */
void shrtim_example_half_pfc_full_llc_start(void)
{
    shrtim_pwm_start(LLC_LEG1_UNIT);
    shrtim_pwm_start(LLC_LEG2_UNIT);
    shrtim_master_period(shrtim_clk_hz() / LLC_FREQ_HZ - 1u, true);
    shrtim_pwm_start(PFC_UNIT);
}

/** @brief Stop all converters; pins go to their configured idle levels. */
void shrtim_example_half_pfc_full_llc_stop(void)
{
    shrtim_pwm_stop(PFC_UNIT);
    shrtim_pwm_stop(LLC_LEG1_UNIT);
    shrtim_pwm_stop(LLC_LEG2_UNIT);
}

/**
 * @brief Apply a new PFC duty from the current/voltage control loop.
 *
 * @param[in] duty_x1000 PFC x1 duty in 0.1 % steps (x2 follows as the
 *                       complementary output).
 * @return #SHRTIM_OK, or #SHRTIM_ERR_PARAM outside 0..1000.
 */
int shrtim_example_half_pfc_full_llc_pfc_duty(uint32_t duty_x1000)
{
    return shrtim_pwm_duty(PFC_UNIT, duty_x1000);
}

/**
 * @brief Sweep both LLC legs to a new frequency (output regulation).
 *
 * Both legs always receive the same frequency so the diagonal phase
 * relationship is preserved.
 *
 * @param[in] hz New LLC switching frequency in Hz.
 * @return #SHRTIM_OK, or #SHRTIM_ERR_FREQ when unreachable.
 */
int shrtim_example_half_pfc_full_llc_llc_freq(uint32_t hz)
{
    int rc = shrtim_pwm_freq(LLC_LEG1_UNIT, hz);

    if (rc == SHRTIM_OK)
    {
        rc = shrtim_pwm_freq(LLC_LEG2_UNIT, hz);
    }
    return rc;
}
