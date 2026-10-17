/**
 * @file shrtim_example_full_pfc_half_llc.c
 * @brief Single-chip combo: full-bridge PFC (4 PWM) + half-bridge LLC (2 PWM).
 *
 * Unit / switch mapping (fixed silicon pins):
 *
 * | Unit | Output | Pin  | Power switch              | Notes                    |
 * | ---- | ------ | ---- | ------------------------- | ------------------------ |
 * | A    | A1     | PA8  | PFC leg 1 high-side       | 65 kHz, 50 ns dead-time  |
 * | A    | A2     | PA9  | PFC leg 1 low-side        |                          |
 * | B    | B2     | PA11 | PFC leg 2 high-side       | wired to the complement  |
 * | B    | B1     | PA10 | PFC leg 2 low-side        | for 180 deg interleave   |
 * | C    | C1     | PB12 | LLC high-side MOSFET      | 100 kHz, 50 %+150 ns DT  |
 * | C    | C2     | PB13 | LLC low-side MOSFET       |                          |
 *
 * The full-bridge (interleaved) PFC uses two complementary units. Wiring
 * the second leg's high-side drive to the B2 complement of the in-phase
 * unit B gives the inherent 180 deg interleave between the legs — the
 * input/inductor current ripple cancels without any phase-shift API.
 *
 * The half-bridge LLC is one plain complementary pair regulated by
 * frequency sweep; the duty stays at 50 %.
 *
 * PA9/PA10 double as USART1 pads: with a console UART on them, remap the
 * PFC legs onto units D/E and the LLC onto unit F.
 */

#include "nation_shrtim.h"

#define PFC_LEG1_UNIT   SHRTIM_UNIT_A
#define PFC_LEG2_UNIT   SHRTIM_UNIT_B
#define LLC_UNIT        SHRTIM_UNIT_C

#define PFC_FREQ_HZ     65000u   /**< PFC switching frequency.             */
#define PFC_DT_NS       50u      /**< PFC dead-time, both edges.           */
#define LLC_FREQ_HZ     100000u  /**< LLC nominal switching frequency.     */
#define LLC_DT_NS       150u     /**< LLC dead-time, both edges.           */
#define OCP_FAULT_CH    1u       /**< FLT1 pin: over-current comparator.   */

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

    /* PFC legs: two complementary pairs at one frequency; the 180 deg
     * interleave comes from wiring leg 2's high-side to B2 (complement). */
    (void)shrtim_pinmux(SHRTIM_OUT_A1);
    (void)shrtim_pinmux(SHRTIM_OUT_A2);
    (void)shrtim_pinmux(SHRTIM_OUT_B1);
    (void)shrtim_pinmux(SHRTIM_OUT_B2);
    cfg.freq_hz    = PFC_FREQ_HZ;
    cfg.duty_x1000 = 100u;                 /* soft-start pedestal          */
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
    shrtim_pwm_sync_to_master(PFC_LEG1_UNIT, true);
    shrtim_pwm_sync_to_master(PFC_LEG2_UNIT, true);

    /* LLC half-bridge: one complementary pair, fixed 50 % + dead-time. */
    (void)shrtim_pinmux(SHRTIM_OUT_C1);
    (void)shrtim_pinmux(SHRTIM_OUT_C2);
    cfg.freq_hz    = LLC_FREQ_HZ;
    cfg.duty_x1000 = 500u;
    cfg.dt_rise_ns = LLC_DT_NS;
    cfg.dt_fall_ns = LLC_DT_NS;
    rc = shrtim_pwm_init(LLC_UNIT, &cfg);
    if (rc != SHRTIM_OK)
    {
        return rc;
    }

    /* Over-current protection on every converter leg. */
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

    /* Two ADC triggers: interleaved PFC currents and the LLC tank. */
    (void)shrtim_adc_trigger(1u, PFC_LEG1_UNIT, SHRTIM_EVT_CMP1);
    (void)shrtim_adc_trigger(2u, LLC_UNIT, SHRTIM_EVT_CMP1);

    shrtim_master_period(shrtim_clk_hz() / PFC_FREQ_HZ - 1u, false);
    return SHRTIM_OK;
}

/**
 * @brief Start the converters: phase-locked PFC legs first, then the LLC.
 *
 * The master counter is released after both PFC units run so their period
 * boundaries coincide (hardware-verified sequence).
 */
void shrtim_example_full_pfc_half_llc_start(void)
{
    shrtim_pwm_start(PFC_LEG1_UNIT);
    shrtim_pwm_start(PFC_LEG2_UNIT);
    shrtim_master_period(shrtim_clk_hz() / PFC_FREQ_HZ - 1u, true);
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
 * @brief Apply a new duty to both interleaved PFC legs (current loop).
 *
 * Both legs always share the duty so the interleave ripple cancellation
 * is preserved.
 *
 * @param[in] duty_x1000 PFC x1 duty in 0.1 % steps.
 * @return #SHRTIM_OK, or #SHRTIM_ERR_PARAM outside 0..1000.
 */
int shrtim_example_full_pfc_half_llc_pfc_duty(uint32_t duty_x1000)
{
    int rc = shrtim_pwm_duty(PFC_LEG1_UNIT, duty_x1000);

    if (rc == SHRTIM_OK)
    {
        rc = shrtim_pwm_duty(PFC_LEG2_UNIT, duty_x1000);
    }
    return rc;
}

/**
 * @brief Sweep the LLC to a new frequency (PFM output regulation).
 *
 * @param[in] hz New LLC switching frequency in Hz.
 * @return #SHRTIM_OK, or #SHRTIM_ERR_FREQ when unreachable.
 */
int shrtim_example_full_pfc_half_llc_llc_freq(uint32_t hz)
{
    return shrtim_pwm_freq(LLC_UNIT, hz);
}
