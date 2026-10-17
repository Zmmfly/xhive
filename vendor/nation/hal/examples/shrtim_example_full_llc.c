/**
 * @file shrtim_example_full_llc.c
 * @brief Single-chip full-bridge LLC resonant converter (4 PWM).
 *
 * Unit / switch mapping (fixed silicon pins):
 *
 * | Unit | Output | Pin  | Power switch       | Notes                      |
 * | ---- | ------ | ---- | ------------------ | -------------------------- |
 * | A    | A1     | PA8  | Q1 (leg 1 high)    | 60..150 kHz sweep, 50 %+DT |
 * | A    | A2     | PA9  | Q3 (leg 1 low)     | 150 ns dead-time           |
 * | B    | B1     | PA10 | Q2 (leg 2 high)    | phase-locked to master     |
 * | B    | B2     | PA11 | Q4 (leg 2 low)     |                            |
 *
 * Both legs run 50 % complementary in phase; the diagonal conduction pairs
 * come from wiring the complement: Q1(A1)+Q4(B2) conduct together, then
 * Q2(B1)+Q3(A2). Output regulation is pure frequency sweep (PFM), the
 * duty stays at 50 %.
 *
 * PA9/PA10 double as USART1 pads — if a console UART is required, move the
 * legs to units C/D (PB12..PB15) or D/E instead.
 */

#include "nation_shrtim.h"

#define LLC_LEG1_UNIT   SHRTIM_UNIT_A
#define LLC_LEG2_UNIT   SHRTIM_UNIT_B

#define LLC_FREQ_HZ     100000u  /**< Nominal (resonant) frequency.       */
#define LLC_FREQ_MAX_HZ 150000u  /**< Sweep upper bound (light load).     */
#define LLC_FREQ_MIN_HZ 60000u   /**< Sweep lower bound (soft start floor).*/
#define LLC_DT_NS       150u     /**< Dead-time, both edges.              */
#define OCP_FAULT_CH    1u       /**< FLT1 pin: over-current comparator.  */

/** Current switching frequency (both legs share it); tracked locally
 *  because the HAL intentionally exposes no period read-back. */
static uint32_t s_llc_freq_hz = LLC_FREQ_MAX_HZ;

/**
 * @brief Configure both LLC legs, master phase lock, OCP fault and the
 *        resonant-current ADC trigger. Units are left stopped.
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

    cfg.freq_hz    = LLC_FREQ_MAX_HZ;      /* soft-start at max frequency  */
    cfg.duty_x1000 = 500u;                 /* LLC always 50 %              */
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

    /* Phase-lock both legs so the diagonal pairs split the period exactly. */
    shrtim_pwm_sync_to_master(LLC_LEG1_UNIT, true);
    shrtim_pwm_sync_to_master(LLC_LEG2_UNIT, true);

    /* Over-current protection: any fault disables all four switches. */
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

    /* Resonant tank current sample once per period (mid on-time of leg 1). */
    (void)shrtim_adc_trigger(1u, LLC_LEG1_UNIT, SHRTIM_EVT_CMP1);

    /* Arm the master anchor; started together with the units. */
    shrtim_master_period(shrtim_clk_hz() / LLC_FREQ_MAX_HZ - 1u, false);
    return SHRTIM_OK;
}

/**
 * @brief Start both legs phase-aligned (master released last).
 *
 * The hardware-verified sequence: both units start, then the master
 * counter kicks the synchronised group so period boundaries coincide.
 */
void shrtim_example_full_llc_start(void)
{
    shrtim_pwm_start(LLC_LEG1_UNIT);
    shrtim_pwm_start(LLC_LEG2_UNIT);
    shrtim_master_period(shrtim_clk_hz() / s_llc_freq_hz - 1u, true);
}

/** @brief Stop the converter; pins go to their configured idle levels. */
void shrtim_example_full_llc_stop(void)
{
    shrtim_pwm_stop(LLC_LEG1_UNIT);
    shrtim_pwm_stop(LLC_LEG2_UNIT);
}

/**
 * @brief Sweep the converter to a new frequency (PFM output regulation).
 *
 * Lower frequency -> more output power (approaching resonance). Both legs
 * always receive the same frequency so the diagonal phase relationship is
 * preserved. Ramp in steps from the control task for a clean soft start.
 *
 * @param[in] hz New switching frequency in Hz.
 * @return #SHRTIM_OK, or #SHRTIM_ERR_FREQ when unreachable.
 */
int shrtim_example_full_llc_freq_set(uint32_t hz)
{
    int rc = shrtim_pwm_freq(LLC_LEG1_UNIT, hz);

    if (rc == SHRTIM_OK)
    {
        rc = shrtim_pwm_freq(LLC_LEG2_UNIT, hz);
    }
    if (rc == SHRTIM_OK)
    {
        s_llc_freq_hz = hz;
    }
    return rc;
}

/**
 * @brief Read back the tracked switching frequency.
 *
 * @return Frequency in Hz (both legs always match by construction).
 */
uint32_t shrtim_example_full_llc_freq_get(void)
{
    return s_llc_freq_hz;
}
