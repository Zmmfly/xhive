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
 * anyway). Both slow-leg outputs are still SHRTIM outputs, so fault
 * protection and idle behaviour apply uniformly.
 *
 * Interleaved totem-pole variants simply allocate one more fast-leg unit
 * (e.g. unit C) and use the complementary x2 pin as the second leg's
 * high-side drive for the inherent 180 deg interleave.
 */

#include "nation_shrtim.h"

#define FAST_UNIT      SHRTIM_UNIT_A
#define SLOW_UNIT      SHRTIM_UNIT_B
#define FAST_FREQ_HZ   65000u   /**< Fast-leg switching frequency.         */
#define FAST_DT_NS     50u      /**< Fast-leg dead-time (GaN-class).       */
#define OCP_FAULT_CH   1u       /**< FLT1 pin: over-current comparator.    */

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

    /* Fast leg: complementary PWM with GaN-class dead-time. */
    (void)shrtim_pinmux(SHRTIM_OUT_A1);
    (void)shrtim_pinmux(SHRTIM_OUT_A2);
    cfg.freq_hz    = FAST_FREQ_HZ;
    cfg.duty_x1000 = 100u;                 /* soft-start pedestal          */
    cfg.dt_rise_ns = FAST_DT_NS;
    cfg.dt_fall_ns = FAST_DT_NS;
    rc = shrtim_pwm_init(FAST_UNIT, &cfg);
    if (rc != SHRTIM_OK)
    {
        return rc;
    }

    /* Slow leg: muxed SHRTIM outputs, kept inactive until the grid
     * polarity is known; commutation happens via shrtim_out_force(). */
    (void)shrtim_pinmux(SHRTIM_OUT_B1);
    (void)shrtim_pinmux(SHRTIM_OUT_B2);
    (void)shrtim_pwm_init(SLOW_UNIT, &cfg);   /* unit stays stopped       */
    shrtim_out_force(SHRTIM_OUT_B1, SHRTIM_LEVEL_INACTIVE);
    shrtim_out_force(SHRTIM_OUT_B2, SHRTIM_LEVEL_INACTIVE);

    /* Over-current protection on both legs. */
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

    /* Sample the inductor current in the middle of the on-time. */
    (void)shrtim_adc_trigger(1u, FAST_UNIT, SHRTIM_EVT_CMP1);
    return SHRTIM_OK;
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
 * @param[in] duty_x1000 Fast-leg x1 duty in 0.1 % steps.
 * @return #SHRTIM_OK, or #SHRTIM_ERR_PARAM outside 0..1000.
 */
int shrtim_example_bridgeless_pfc_fast_duty(uint32_t duty_x1000)
{
    return shrtim_pwm_duty(FAST_UNIT, duty_x1000);
}

/**
 * @brief Commutate the slow leg to the mains half-cycle.
 *
 * Call from the zero-crossing / grid polarity detection (software timer
 * or ADC comparator task). The two switches are never active together.
 *
 * @param[in] grid_positive true: positive mains half-cycle, false: negative.
 */
void shrtim_example_bridgeless_pfc_slow_leg(bool grid_positive)
{
    if (grid_positive)
    {
        shrtim_out_force(SHRTIM_OUT_B1, SHRTIM_LEVEL_ACTIVE);
        shrtim_out_force(SHRTIM_OUT_B2, SHRTIM_LEVEL_INACTIVE);
    }
    else
    {
        shrtim_out_force(SHRTIM_OUT_B1, SHRTIM_LEVEL_INACTIVE);
        shrtim_out_force(SHRTIM_OUT_B2, SHRTIM_LEVEL_ACTIVE);
    }
}
