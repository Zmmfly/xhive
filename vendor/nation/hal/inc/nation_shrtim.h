/**
 * @file nation_shrtim.h
 * @brief Modern high-level C API for the N32H47x/48x SHRTIM peripheral.
 *
 * The vendor std-periph driver exposes one function per register field, which
 * makes even a plain complementary PWM hard to write. This HAL wraps that
 * driver into a small, unit-centric API:
 *
 *  - shrtim_init()          bring the SHRTIM clock up once
 *  - shrtim_pwm_init()      one call per timing unit (frequency, duty,
 *                           alignment, dead-time, polarity, idle levels)
 *  - shrtim_pwm_*           cheap runtime tweaks (frequency, duty, ...)
 *  - shrtim_out_*           per-pin output control (enable, force, polarity)
 *  - shrtim_irq_*()         register callbacks; call the shrtim_isr_*()
 *                           entry points from the user's vector handlers
 *
 * Mapping to hardware:
 *  - One "unit" is one SHRTIM timing unit (A..F), it owns one PWM period and
 *    the complementary output pair x1/x2.
 *  - Output pins are fixed by silicon (PA8/PA9, PA10/PA11, PB12..PB15,
 *    PC6..PC9); shrtim_pinmux() programs the right AF for a given output.
 *
 * Setters that can fail return 0 on success and a negative @ref
 * shrtim_rc_t on error; the level and run helpers are void.
 */

#ifndef NATION_SHRTIM_HAL_H
#define NATION_SHRTIM_HAL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Return codes used by every function that can fail. */
typedef enum {
    SHRTIM_OK = 0,           /**< Operation completed. */
    SHRTIM_ERR_PARAM = -1,   /**< Invalid argument (null pointer, bad range). */
    SHRTIM_ERR_FREQ = -2,    /**< Requested frequency unreachable with any
                                  high-resolution prescaler. */
    SHRTIM_ERR_STATE = -3,   /**< Peripheral not initialised (call
                                  shrtim_init() first). */
    SHRTIM_ERR_CLOCK = -4,   /**< SHRTPLL could not be configured. */
} shrtim_rc_t;

/** SHRTIM timing units ("master" only drives sync/ADC triggers). */
typedef enum {
    SHRTIM_UNIT_A = 0,       /**< Timing unit A, outputs A1/A2. */
    SHRTIM_UNIT_B = 1,       /**< Timing unit B, outputs B1/B2. */
    SHRTIM_UNIT_C = 2,       /**< Timing unit C, outputs C1/C2. */
    SHRTIM_UNIT_D = 3,       /**< Timing unit D, outputs D1/D2. */
    SHRTIM_UNIT_E = 4,       /**< Timing unit E, outputs E1/E2. */
    SHRTIM_UNIT_F = 5,       /**< Timing unit F, outputs F1/F2. */
    SHRTIM_UNIT_MASTER = 6,  /**< Master timer (no PWM outputs). */
} shrtim_unit_t;

/** One of the 12 SHRTIM outputs. */
typedef enum {
    SHRTIM_OUT_A1 = 0,       /**< Unit A output 1 (PA8). */
    SHRTIM_OUT_A2 = 1,       /**< Unit A output 2 (PA9). */
    SHRTIM_OUT_B1 = 2,       /**< Unit B output 1 (PA10). */
    SHRTIM_OUT_B2 = 3,       /**< Unit B output 2 (PA11). */
    SHRTIM_OUT_C1 = 4,       /**< Unit C output 1 (PB12). */
    SHRTIM_OUT_C2 = 5,       /**< Unit C output 2 (PB13). */
    SHRTIM_OUT_D1 = 6,       /**< Unit D output 1 (PB14). */
    SHRTIM_OUT_D2 = 7,       /**< Unit D output 2 (PB15). */
    SHRTIM_OUT_E1 = 8,       /**< Unit E output 1 (PC8). */
    SHRTIM_OUT_E2 = 9,       /**< Unit E output 2 (PC9). */
    SHRTIM_OUT_F1 = 10,      /**< Unit F output 1 (PC6). */
    SHRTIM_OUT_F2 = 11,      /**< Unit F output 2 (PC7). */
} shrtim_out_t;

/** Counter alignment of a PWM unit. */
typedef enum {
    SHRTIM_ALIGN_EDGE = 0,   /**< Edge aligned: up-counting, set on period,
                                  reset on compare 1. */
    SHRTIM_ALIGN_CENTER = 1, /**< Center aligned: up-down counting, symmetric
                                  pulse around the valley. */
} shrtim_align_t;

/** Re-trigger behaviour of a unit's counter. */
typedef enum {
    SHRTIM_RUN_CONTINUOUS = 0, /**< Free-running, wraps at period. */
    SHRTIM_RUN_ONESHOT = 1,    /**< Single-shot, stops after one period. */
    SHRTIM_RUN_RETRIG = 2,     /**< Single-shot, retriggerable by reset event. */
} shrtim_run_t;

/** Logic level of an output while idle / forced. */
typedef enum {
    SHRTIM_LEVEL_INACTIVE = 0, /**< Output at its inactive level. */
    SHRTIM_LEVEL_ACTIVE = 1,   /**< Output at its active level. */
} shrtim_level_t;

/** Behaviour of an output while its unit is in fault state. */
typedef enum {
    SHRTIM_FAULT_OUT_NONE = 0,   /**< Output keeps operating. */
    SHRTIM_FAULT_OUT_ACTIVE = 1, /**< Output forced active. */
    SHRTIM_FAULT_OUT_INACTIVE = 2, /**< Output forced inactive. */
    SHRTIM_FAULT_OUT_HIGHZ = 3,  /**< Output tri-stated. */
} shrtim_fault_out_t;

/** PWM configuration, see @ref shrtim_pwm_cfg_default. */
typedef struct {
    uint32_t freq_hz;        /**< PWM frequency in Hz (1 .. fSHRTIM). */
    uint32_t duty_x1000;     /**< Output x1 duty in 0.1 % steps (0..1000). */
    uint32_t duty2_x1000;    /**< Output x2 duty in 0.1 % steps. 0 selects the
                                  automatic complement of x1 (with dead-time);
                                  a non-zero value enables asymmetric mode
                                  where x2 gets its own compare value. */
    shrtim_align_t align;    /**< Edge or center aligned counting. */
    shrtim_run_t run;        /**< Continuous / one-shot / retriggerable. */
    bool active_low1;        /**< Invert polarity of output x1. */
    bool active_low2;        /**< Invert polarity of output x2. */
    shrtim_level_t idle1;    /**< x1 level while the unit is stopped. */
    shrtim_level_t idle2;    /**< x2 level while the unit is stopped. */
    uint32_t dt_rise_ns;     /**< Dead-time on x1 rising edge, ns (0 = off). */
    uint32_t dt_fall_ns;     /**< Dead-time on x1 falling edge, ns (0 = off). */
    bool enable_outputs;     /**< Also drive the pins (OEN bits) on start. */
} shrtim_pwm_cfg_t;

/** Static initaliser for @ref shrtim_pwm_cfg_t: 20 kHz, 50 %, edge aligned,
 *  complementary pair, no dead-time, active-high, idle low. */
#define SHRTIM_PWM_CFG_DEFAULT { \
    .freq_hz = 20000u, .duty_x1000 = 500u, .duty2_x1000 = 0u, \
    .align = SHRTIM_ALIGN_EDGE, .run = SHRTIM_RUN_CONTINUOUS, \
    .active_low1 = false, .active_low2 = false, \
    .idle1 = SHRTIM_LEVEL_INACTIVE, .idle2 = SHRTIM_LEVEL_INACTIVE, \
    .dt_rise_ns = 0u, .dt_fall_ns = 0u, .enable_outputs = true }

/** Interrupt / callback events, one per hardware interrupt source. */
typedef enum {
    SHRTIM_EVT_UPDATE = 0,   /**< Timer update (preload transfer). */
    SHRTIM_EVT_REPT = 1,     /**< Repetition counter reached zero. */
    SHRTIM_EVT_CMP1 = 2,     /**< Compare 1 match. */
    SHRTIM_EVT_CMP2 = 3,     /**< Compare 2 match. */
    SHRTIM_EVT_CMP3 = 4,     /**< Compare 3 match. */
    SHRTIM_EVT_CMP4 = 5,     /**< Compare 4 match. */
    SHRTIM_EVT_CMP5 = 6,     /**< Compare 5 match (raw access helper). */
    SHRTIM_EVT_CPT1 = 7,     /**< Capture 1 done. */
    SHRTIM_EVT_CPT2 = 8,     /**< Capture 2 done. */
    SHRTIM_EVT_RSTRO = 9,    /**< Counter reset / roll-over. */
    SHRTIM_EVT_FAULT1 = 10,  /**< Fault input 1 became active. */
    SHRTIM_EVT_FAULT2 = 11,  /**< Fault input 2 became active. */
    SHRTIM_EVT_FAULT3 = 12,  /**< Fault input 3 became active. */
    SHRTIM_EVT_FAULT4 = 13,  /**< Fault input 4 became active. */
    SHRTIM_EVT_FAULT5 = 14,  /**< Fault input 5 became active. */
    SHRTIM_EVT_FAULT6 = 15,  /**< Fault input 6 became active. */
    SHRTIM_EVT_SYSFLT = 16,  /**< System fault (clock loss, lockup, ECC ...). */
    SHRTIM_EVT_MST_SYNC = 17, /**< Master timer synchronisation event. */
} shrtim_evt_t;

/**
 * @brief Callback invoked from the (user's) interrupt context.
 *
 * @param[in] unit  Unit that raised the event, SHRTIM_UNIT_MASTER for master
 *                  events or the fault events.
 * @param[in] evt   Event that fired; flags are already cleared when the
 *                  callback runs.
 * @param[in,out] user  Opaque pointer registered with the callback.
 */
typedef void (*shrtim_cb_t)(shrtim_unit_t unit, shrtim_evt_t evt, void *user);

/** External-event (EXEV) input configuration. */
typedef struct {
    bool from_gpio;          /**< true: GPIO-sourced event (needs AFIO pin
                                  routing done by the application), false:
                                  on-chip source selected by @p chip_src. */
    uint8_t chip_src;        /**< 0 comparator out, 1 ATIMx TRGO, 2 ADC AWD
                                  (only used when from_gpio == false). */
    bool active_low;         /**< Event polarity. */
    uint8_t sensitivity;     /**< 0 level, 1 rising, 2 falling, 3 both edges. */
    bool fast_mode;          /**< true: asynchronous low-latency path. */
    uint8_t filter;          /**< 0 = off, 1..15 = digital filter taps. */
} shrtim_exev_cfg_t;

/** Fault input configuration. */
typedef struct {
    bool active_high;        /**< Fault pin polarity (default active low). */
    uint8_t filter;          /**< 0 = off, 1..15 = digital filter taps. */
    bool latch;              /**< true: fault stays active after the source
                                  clears until shrtim_fault_reset(). */
} shrtim_fault_cfg_t;

/** Chopper (high-frequency carrier) configuration for one unit. */
typedef struct {
    bool enable;             /**< Chop both outputs with the carrier. */
    uint8_t duty_eighths;    /**< Carrier duty in 1/8 steps (0..7). */
    uint8_t prescaler_div;   /**< Carrier = fSHRTIM / (16 * (div+1)), div 0..15. */
} shrtim_chopper_cfg_t;

/* ---------------------------------------------------------------------- */
/* Global lifecycle                                                        */
/* ---------------------------------------------------------------------- */

/**
 * @brief Bring SHRTIM up: SHRTPLL, clocks, reset.
 *
 * Configures the dedicated SHRTPLL from HSE when available, else HSI, to the
 * requested frequency, enables the SHRTIM AHB clock and releases the
 * peripheral from reset. Safe to call again to change fSHRTIM.
 *
 * @param[in] fshrtim_hz Target SHRTIM kernel clock in Hz (0 selects 240 MHz).
 * @return #SHRTIM_OK on success, #SHRTIM_ERR_CLOCK if the PLL cannot lock.
 */
int shrtim_init(uint32_t fshrtim_hz);

/**
 * @brief Actual SHRTIM kernel clock in Hz after @ref shrtim_init().
 * @return Kernel clock in Hz, 0 when not initialised.
 */
uint32_t shrtim_clk_hz(void);

/**
 * @brief Turn everything off and reset the peripheral.
 *
 * Stops all units, disables outputs, resets SHRTIM and gates its clock.
 */
void shrtim_deinit(void);

/* ---------------------------------------------------------------------- */
/* Pin muxing                                                              */
/* ---------------------------------------------------------------------- */

/**
 * @brief Route one SHRTIM output to its fixed GPIO pin.
 *
 * PA8/PA9, PA10/PA11, PB12..PB15 and PC6..PC9 have one hard-wired AF each;
 * this helper programs it (AF6/AF12/AF10/AF11), so the application does not
 * need the AF numbers.
 *
 * @param[in] out Output to mux onto its pin.
 * @return #SHRTIM_OK, or #SHRTIM_ERR_PARAM for a bad output id.
 */
int shrtim_pinmux(shrtim_out_t out);

/**
 * @brief Route one fault input to its fixed GPIO pin as input.
 * @param[in] fault Fault channel 1..6 (5 uses PB0).
 * @return #SHRTIM_OK, or #SHRTIM_ERR_PARAM outside 1..6.
 */
int shrtim_pinmux_fault(uint8_t fault);

/* ---------------------------------------------------------------------- */
/* PWM: configure / run                                                    */
/* ---------------------------------------------------------------------- */

/**
 * @brief Configure one timing unit for PWM in a single call.
 *
 * Computes the high-resolution prescaler and period for @p cfg->freq_hz,
 * applies duty, alignment, dead-time, polarities and idle levels, arms the
 * preload shadow registers and forces one update so the values are live.
 * The unit is left stopped; call @ref shrtim_pwm_start().
 *
 * @param[in] unit Unit to configure (SHRTIM_UNIT_MASTER is rejected).
 * @param[in] cfg  Configuration, @ref SHRTIM_PWM_CFG_DEFAULT is a good start.
 * @return #SHRTIM_OK, #SHRTIM_ERR_PARAM, #SHRTIM_ERR_FREQ or
 *         #SHRTIM_ERR_STATE.
 */
int shrtim_pwm_init(shrtim_unit_t unit, const shrtim_pwm_cfg_t *cfg);

/**
 * @brief Change the PWM frequency of a configured unit at runtime.
 *
 * Period and compare values are recomputed for the current duty settings and
 * pushed through the preload mechanism (glitch-free while running).
 *
 * @param[in] unit Configured unit.
 * @param[in] hz   New frequency in Hz.
 * @return #SHRTIM_OK, or #SHRTIM_ERR_FREQ when unreachable.
 */
int shrtim_pwm_freq(shrtim_unit_t unit, uint32_t hz);

/**
 * @brief Change output x1 duty cycle at runtime (0.1 % units, 0..1000).
 *
 * With a zero duty2 the x2 output stays the automatic complement.
 *
 * @param[in] unit Configured unit.
 * @param[in] duty_x1000 Duty in 0.1 % steps.
 * @return #SHRTIM_OK, or #SHRTIM_ERR_PARAM outside 0..1000.
 */
int shrtim_pwm_duty(shrtim_unit_t unit, uint32_t duty_x1000);

/**
 * @brief Set an independent duty for output x2 (asymmetric PWM).
 *
 * A non-zero value gives x2 its own compare register (useful in center
 * aligned mode); zero restores the complementary behaviour.
 *
 * @param[in] unit Configured unit.
 * @param[in] duty_x1000 Duty in 0.1 % steps, 0 = complementary again.
 * @return #SHRTIM_OK, or #SHRTIM_ERR_PARAM.
 */
int shrtim_pwm_duty2(shrtim_unit_t unit, uint32_t duty_x1000);

/**
 * @brief Program the dead-time generator of a unit from nanoseconds.
 *
 * Picks the closest dead-time prescaler automatically; the value actually
 * programmed is quantised by that prescaler. Passing 0/0 disables dead-time.
 *
 * @param[in] unit     Configured unit.
 * @param[in] rise_ns  Dead-time before x1 goes active (ns).
 * @param[in] fall_ns  Dead-time after x1 goes inactive (ns).
 * @return #SHRTIM_OK, or #SHRTIM_ERR_PARAM when a value exceeds the range of
 *         every prescaler (approximately 16 us at 240 MHz).
 */
int shrtim_pwm_deadtime(shrtim_unit_t unit, uint32_t rise_ns, uint32_t fall_ns);

/**
 * @brief Start a unit's counter (outputs follow enable state).
 * @param[in] unit Unit to run.
 */
void shrtim_pwm_start(shrtim_unit_t unit);

/**
 * @brief Stop a unit's counter and leave the pins at their idle levels.
 * @param[in] unit Unit to halt.
 */
void shrtim_pwm_stop(shrtim_unit_t unit);

/** @brief Start every timing unit A..F. */
void shrtim_pwm_start_all(void);

/** @brief Stop every timing unit A..F. */
void shrtim_pwm_stop_all(void);

/**
 * @brief Re-arm a one-shot unit (single-shot or retriggerable mode).
 *
 * Resets the counter so the next period is emitted from the beginning.
 * @param[in] unit Unit configured with @ref SHRTIM_RUN_ONESHOT or
 *                 @ref SHRTIM_RUN_RETRIG.
 */
void shrtim_pwm_oneshot_arm(shrtim_unit_t unit);

/* ---------------------------------------------------------------------- */
/* Output level control                                                    */
/* ---------------------------------------------------------------------- */

/**
 * @brief Enable one output driver on its pin.
 * @param[in] out Output to enable.
 */
void shrtim_out_enable(shrtim_out_t out);

/**
 * @brief Disable one output driver (pin goes to its idle level).
 * @param[in] out Output to disable.
 */
void shrtim_out_disable(shrtim_out_t out);

/**
 * @brief Force an output to a logic level, bypassing the PWM crossbar.
 *
 * @param[in] out Output to force.
 * @param[in] level #SHRTIM_LEVEL_ACTIVE or #SHRTIM_LEVEL_INACTIVE.
 */
void shrtim_out_force(shrtim_out_t out, shrtim_level_t level);

/**
 * @brief Invert the polarity of one output at runtime.
 * @param[in] out Output to change.
 * @param[in] active_low true = active low.
 */
void shrtim_out_polarity(shrtim_out_t out, bool active_low);

/**
 * @brief Swap outputs x1 and x2 of a unit (crossbar swap).
 * @param[in] unit Unit whose outputs are exchanged.
 * @param[in] enable true to swap, false to restore.
 */
void shrtim_pwm_swap(shrtim_unit_t unit, bool enable);

/**
 * @brief Push-pull mode: x1 and x2 alternate periods instead of being
 *        complementary within one period.
 * @param[in] unit Unit to change.
 * @param[in] enable true to enable push-pull.
 */
void shrtim_pwm_pushpull(shrtim_unit_t unit, bool enable);

/**
 * @brief Chop both outputs of a unit with a high-frequency carrier
 *        (transformer gate drive).
 * @param[in] unit Unit to chop.
 * @param[in] cfg  Carrier settings, @p cfg->enable == false turns it off.
 * @return #SHRTIM_OK or #SHRTIM_ERR_PARAM.
 */
int shrtim_pwm_chopper(shrtim_unit_t unit, const shrtim_chopper_cfg_t *cfg);

/* ---------------------------------------------------------------------- */
/* Raw counter / compare access                                            */
/* ---------------------------------------------------------------------- */

/**
 * @brief Read the live counter of a unit.
 * @param[in] unit Unit (or SHRTIM_UNIT_MASTER) to read.
 * @return Counter value in high-resolution ticks.
 */
uint32_t shrtim_counter(shrtim_unit_t unit);

/**
 * @brief Overwrite the period (preload) of a unit in high-resolution ticks.
 * @param[in] unit Unit to change.
 * @param[in] ticks New period, 1..0xFFFF.
 */
void shrtim_period_raw(shrtim_unit_t unit, uint32_t ticks);

/**
 * @brief Overwrite one compare register (preload) in high-resolution ticks.
 * @param[in] unit Unit to change.
 * @param[in] cmp  Compare index 1..5.
 * @param[in] ticks New compare value.
 */
void shrtim_compare_raw(shrtim_unit_t unit, uint8_t cmp, uint32_t ticks);

/**
 * @brief Read back one compare register.
 * @param[in] unit Unit to read.
 * @param[in] cmp  Compare index 1..5.
 * @return Compare value in high-resolution ticks.
 */
uint32_t shrtim_compare_get(shrtim_unit_t unit, uint8_t cmp);

/* ---------------------------------------------------------------------- */
/* Master timer, sync and triggers                                         */
/* ---------------------------------------------------------------------- */

/**
 * @brief Configure the master timer period for synchronised operation.
 *
 * @param[in] period_ticks Master period in high-resolution ticks.
 * @param[in] start_now Also start the master counter.
 */
void shrtim_master_period(uint32_t period_ticks, bool start_now);

/**
 * @brief Reset-to-master-period: slave a unit's counter to the master.
 *
 * @param[in] unit Unit to slave.
 * @param[in] enable true = master period resets this unit.
 */
void shrtim_pwm_sync_to_master(shrtim_unit_t unit, bool enable);

/**
 * @brief Route one SHRTIM event to an ADC trigger output.
 *
 * @param[in] trig  ADC trigger 1..10 (matches SHRTIM_TRGO1..10).
 * @param[in] unit  Event owner unit (or SHRTIM_UNIT_MASTER).
 * @param[in] evt   Event to route: UPDATE, CMP1..CMP4 or RSTRO.
 * @return #SHRTIM_OK or #SHRTIM_ERR_PARAM.
 */
int shrtim_adc_trigger(uint8_t trig, shrtim_unit_t unit, shrtim_evt_t evt);

/* ---------------------------------------------------------------------- */
/* External events, faults, capture                                        */
/* ---------------------------------------------------------------------- */

/**
 * @brief Configure one of the ten external event channels.
 *
 * @param[in] ev  Event channel 1..10.
 * @param[in] cfg Event source, polarity, sensitivity and filtering.
 * @return #SHRTIM_OK or #SHRTIM_ERR_PARAM.
 */
int shrtim_exev_config(uint8_t ev, const shrtim_exev_cfg_t *cfg);

/**
 * @brief Make one external event reset a unit's counter (event-driven PWM).
 * @param[in] unit Unit to affect.
 * @param[in] ev   Event channel 1..10, 0 detaches all events.
 * @return #SHRTIM_OK or #SHRTIM_ERR_PARAM.
 */
int shrtim_exev_reset_unit(shrtim_unit_t unit, uint8_t ev);

/**
 * @brief Configure a fault input pin and its filtering.
 * @param[in] fault Fault channel 1..6.
 * @param[in] cfg   Polarity / filter / latch configuration.
 * @return #SHRTIM_OK or #SHRTIM_ERR_PARAM.
 */
int shrtim_fault_config(uint8_t fault, const shrtim_fault_cfg_t *cfg);

/**
 * @brief Attach fault channels to a unit and define per-output fault states.
 *
 * @param[in] unit  Unit to protect.
 * @param[in] mask  Bitmask of fault channels (bit0 = fault 1).
 * @param[in] out1  Fault behaviour of output x1.
 * @param[in] out2  Fault behaviour of output x2.
 * @return #SHRTIM_OK or #SHRTIM_ERR_PARAM.
 */
int shrtim_fault_attach(shrtim_unit_t unit, uint8_t mask,
                        shrtim_fault_out_t out1, shrtim_fault_out_t out2);

/**
 * @brief Clear a latched fault and let the outputs resume.
 * @param[in] fault Fault channel 1..6.
 */
void shrtim_fault_reset(uint8_t fault);

/**
 * @brief Select the capture trigger of a capture unit.
 *
 * @param[in] unit  Unit owning the capture.
 * @param[in] cpt   Capture unit 1 or 2.
 * @param[in] ev    External event channel 1..10 that latches the counter,
 *                  0 selects the software trigger.
 * @return #SHRTIM_OK or #SHRTIM_ERR_PARAM.
 */
int shrtim_capture_config(shrtim_unit_t unit, uint8_t cpt, uint8_t ev);

/**
 * @brief Latch a software-triggered capture and read both capture units.
 *
 * @param[in] unit Unit to capture.
 * @param[out] cpt1 Captured counter of capture unit 1 (may be NULL).
 * @param[out] cpt2 Captured counter of capture unit 2 (may be NULL).
 * @return #SHRTIM_OK, or #SHRTIM_ERR_STATE when no capture completed yet.
 */
int shrtim_capture_read(shrtim_unit_t unit, uint32_t *cpt1, uint32_t *cpt2);

/* ---------------------------------------------------------------------- */
/* Interrupt plumbing (manual ISR hookup)                                  */
/* ---------------------------------------------------------------------- */

/**
 * @brief Register a callback for one event of one unit.
 *
 * The callback runs from interrupt context via the @ref shrtim_isr_timer /
 * @ref shrtim_isr_master / @ref shrtim_isr_fault entry points that the
 * application calls from its vector handlers. Only one callback per
 * (unit, event) pair is stored; registering NULL removes it.
 *
 * @param[in] unit Owning unit; fault/system events accept any unit and are
 *                 delivered once per event from the fault ISR.
 * @param[in] evt  Event to subscribe to.
 * @param[in] cb   Callback, NULL to remove.
 * @param[in,out] user Opaque pointer passed back to the callback.
 * @return #SHRTIM_OK or #SHRTIM_ERR_PARAM.
 */
int shrtim_irq_hook(shrtim_unit_t unit, shrtim_evt_t evt,
                    shrtim_cb_t cb, void *user);

/**
 * @brief Enable or disable the interrupt source of one event.
 *
 * Also enables the NVIC line of the owning vector.
 *
 * @param[in] unit Owning unit.
 * @param[in] evt  Event to toggle.
 * @param[in] enable true = fire callbacks, false = mask.
 * @return #SHRTIM_OK or #SHRTIM_ERR_PARAM.
 */
int shrtim_irq_enable(shrtim_unit_t unit, shrtim_evt_t evt, bool enable);

/**
 * @brief ISR entry for SHRT1_MST_IRQHandler (master timer vector).
 *
 * Call unconditionally as the first statement of the master handler; it
 * checks/clears flags and dispatches registered callbacks.
 */
void shrtim_isr_master(void);

/**
 * @brief ISR entry for SHRT1_TIMx_IRQHandler (unit A..F vectors).
 *
 * @param[in] unit Unit whose vector fired (A..F).
 */
void shrtim_isr_timer(shrtim_unit_t unit);

/**
 * @brief ISR entry for SHRT1_FLT_IRQHandler (fault vector).
 *
 * Handles fault 1..6 and the system fault, clears flags and dispatches.
 */
void shrtim_isr_fault(void);

#ifdef __cplusplus
}
#endif

#endif /* NATION_SHRTIM_HAL_H */
