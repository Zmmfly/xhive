/**
 * @file nation_shrtim.c
 * @brief Implementation of the high-level N32H47x/48x SHRTIM HAL.
 *
 * Wraps the Nation std-periph driver (n32h47x_48x_shrtim.c and friends) into
 * the unit-centric API declared in nation_shrtim.h. The vendor driver is only
 * called, never modified. No dynamic memory is used; all state is static.
 *
 * Register-usage notes verified against the vendor sources:
 *  - SHRTIM_TIM_SetPrescaler/SetPeriod/SetCompare1..4/SetCounterMode/
 *    EnablePreload/GetCounter and the IT/flag helpers accept
 *    SHRTIM_TIMER_MASTER because they are based on sMasterRegs (index 0).
 *  - SHRTIM_TIM_SetCountingMode/SetCompare5/SetCaptureTrig/GetCapture
 *    and the DT/CHP/OUT functions are based on sTimerxRegs and accept A..F
 *    only.
 *  - Dead-time value fields DTR/DTF are 9 bits wide (SHRTIM_TXDT_DTR =
 *    0x1FF), so the maximum dead-time is 511 DTG ticks (~34.5 us at the
 *    DIV16 prescaler with fSHRTIM = 240 MHz).
 */

#include "nation_shrtim.h"

#include <stddef.h>

#include "n32h47x_48x_gpio.h"
#include "n32h47x_48x_rcc.h"
#include "n32h47x_48x_shrtim.h"

/* The vendor xmake target only defines HSE_VALUE when the HSE oscillator is
 * enabled in kconfig and HSI_VALUE when SYSCLK runs from HSI. Provide safe
 * fallbacks so this file builds in every configuration; the runtime source
 * selection below never picks HSE when the oscillator is not ready. */
#ifndef HSE_VALUE
#define HSE_VALUE 8000000UL
#endif
#ifndef HSI_VALUE
#define HSI_VALUE 8000000UL
#endif

#define SHRTIM_HAL_PERIOD_MAX 0xFFFFu
#define SHRTIM_HAL_DTV_MAX 0x1FFu /* 9-bit dead-time value fields */
#define SHRTIM_HAL_EVT_COUNT 18u
#define SHRTIM_HAL_UNITS 6u

/* ---------------------------------------------------------------------- */
/* Static state                                                            */
/* ---------------------------------------------------------------------- */

/** Vendor timer identifiers for units A..F (master is SHRTIM_TIMER_MASTER). */
static const uint32_t s_tmr_id[SHRTIM_HAL_UNITS] = {
    SHRTIM_TIMER_A, SHRTIM_TIMER_B, SHRTIM_TIMER_C,
    SHRTIM_TIMER_D, SHRTIM_TIMER_E, SHRTIM_TIMER_F,
};

/** Vendor output-enable identifiers for the 12 outputs. */
static const uint32_t s_out_id[12] = {
    SHRTIM_OUTPUT_TA1, SHRTIM_OUTPUT_TA2,
    SHRTIM_OUTPUT_TB1, SHRTIM_OUTPUT_TB2,
    SHRTIM_OUTPUT_TC1, SHRTIM_OUTPUT_TC2,
    SHRTIM_OUTPUT_TD1, SHRTIM_OUTPUT_TD2,
    SHRTIM_OUTPUT_TE1, SHRTIM_OUTPUT_TE2,
    SHRTIM_OUTPUT_TF1, SHRTIM_OUTPUT_TF2,
};

/** Per-unit timing configuration kept for runtime re-computation. */
typedef struct {
    uint32_t ratio;       /**< Programmed CKPSC value. */
    uint32_t period;      /**< Programmed period in HR ticks. */
    uint32_t duty;        /**< x1 duty in 0.1 % steps. */
    uint32_t duty2;       /**< x2 duty in 0.1 % steps, 0 = complementary. */
    uint8_t align;        /**< shrtim_align_t. */
    uint8_t ratio_idx;    /**< Index of ratio in s_ratio_hw (master anchor). */
    uint8_t configured;   /**< Non-zero after shrtim_pwm_init(). */
} unit_state_t;

static unit_state_t s_unit[SHRTIM_HAL_UNITS];
static uint32_t s_clk_hz; /**< Actual SHRTIM kernel clock, 0 = uninitialised. */

/** Callback storage: rows A..F plus one row (master) shared by master and
 * fault/system events. */
static shrtim_cb_t s_cb[SHRTIM_HAL_UNITS + 1u][SHRTIM_HAL_EVT_COUNT];
static void *s_cb_user[SHRTIM_HAL_UNITS + 1u][SHRTIM_HAL_EVT_COUNT];

/** Latch requests per fault channel (informational; see fault_config). */
static bool s_fault_latch[6];

/* ---------------------------------------------------------------------- */
/* Pin muxing tables                                                       */
/* ---------------------------------------------------------------------- */

typedef struct {
    GPIO_Module *port;
    uint8_t pos;   /**< Pin index 0..15 on the port. */
    uint32_t af;
    uint32_t rcc_clk;
} pin_map_t;

/** Fixed AF routing of the 12 SHRTIM outputs. */
static const pin_map_t s_out_pins[12] = {
    {GPIOA, 8u, GPIO_AF_SHRTIM_CHA1_PA8, RCC_AHB_PERIPHEN_GPIOA},
    {GPIOA, 9u, GPIO_AF_SHRTIM_CHA2_PA9, RCC_AHB_PERIPHEN_GPIOA},
    {GPIOA, 10u, GPIO_AF_SHRTIM_CHB1_PA10, RCC_AHB_PERIPHEN_GPIOA},
    {GPIOA, 11u, GPIO_AF_SHRTIM_CHB2_PA11, RCC_AHB_PERIPHEN_GPIOA},
    {GPIOB, 12u, GPIO_AF_SHRTIM_CHC1_PB12, RCC_AHB_PERIPHEN_GPIOB},
    {GPIOB, 13u, GPIO_AF_SHRTIM_CHC2_PB13, RCC_AHB_PERIPHEN_GPIOB},
    {GPIOB, 14u, GPIO_AF_SHRTIM_CHD1_PB14, RCC_AHB_PERIPHEN_GPIOB},
    {GPIOB, 15u, GPIO_AF_SHRTIM_CHD2_PB15, RCC_AHB_PERIPHEN_GPIOB},
    {GPIOC, 8u, GPIO_AF_SHRTIM_CHE1_PC8, RCC_AHB_PERIPHEN_GPIOC},
    {GPIOC, 9u, GPIO_AF_SHRTIM_CHE2_PC9, RCC_AHB_PERIPHEN_GPIOC},
    {GPIOC, 6u, GPIO_AF_SHRTIM_CHF1_PC6, RCC_AHB_PERIPHEN_GPIOC},
    {GPIOC, 7u, GPIO_AF_SHRTIM_CHF2_PC7, RCC_AHB_PERIPHEN_GPIOC},
};

/** Fixed AF routing of the 6 fault inputs (FLT5 uses PB0). */
static const pin_map_t s_flt_pins[6] = {
    {GPIOA, 12u, GPIO_AF_SHRTIM_FLT1_PA12, RCC_AHB_PERIPHEN_GPIOA},
    {GPIOA, 15u, GPIO_AF_SHRTIM_FLT2_PA15, RCC_AHB_PERIPHEN_GPIOA},
    {GPIOB, 10u, GPIO_AF_SHRTIM_FLT3_PB10, RCC_AHB_PERIPHEN_GPIOB},
    {GPIOB, 11u, GPIO_AF_SHRTIM_FLT4_PB11, RCC_AHB_PERIPHEN_GPIOB},
    {GPIOB, 0u, GPIO_AF_SHRTIM_FLT5_PB0, RCC_AHB_PERIPHEN_GPIOB},
    {GPIOC, 10u, GPIO_AF_SHRTIM_FLT6_PC10, RCC_AHB_PERIPHEN_GPIOC},
};

/* ---------------------------------------------------------------------- */
/* Lookup tables for value-mapped vendor macros                            */
/* ---------------------------------------------------------------------- */

static const uint32_t s_exev_flt[16] = {
    SHRTIM_EXEV_FILTER_NONE, SHRTIM_EXEV_FILTER_1, SHRTIM_EXEV_FILTER_2,
    SHRTIM_EXEV_FILTER_3, SHRTIM_EXEV_FILTER_4, SHRTIM_EXEV_FILTER_5,
    SHRTIM_EXEV_FILTER_6, SHRTIM_EXEV_FILTER_7, SHRTIM_EXEV_FILTER_8,
    SHRTIM_EXEV_FILTER_9, SHRTIM_EXEV_FILTER_10, SHRTIM_EXEV_FILTER_11,
    SHRTIM_EXEV_FILTER_12, SHRTIM_EXEV_FILTER_13, SHRTIM_EXEV_FILTER_14,
    SHRTIM_EXEV_FILTER_15,
};

static const uint32_t s_exev_sens[4] = {
    SHRTIM_EXEV_SENSITIVITY_LEVEL,
    SHRTIM_EXEV_SENSITIVITY_RISINGEDGE,
    SHRTIM_EXEV_SENSITIVITY_FALLINGEDGE,
    SHRTIM_EXEV_SENSITIVITY_BOTHEDGES,
};

static const uint32_t s_falt_flt[16] = {
    SHRTIM_FALT_FILTER_NONE, SHRTIM_FALT_FILTER_1, SHRTIM_FALT_FILTER_2,
    SHRTIM_FALT_FILTER_3, SHRTIM_FALT_FILTER_4, SHRTIM_FALT_FILTER_5,
    SHRTIM_FALT_FILTER_6, SHRTIM_FALT_FILTER_7, SHRTIM_FALT_FILTER_8,
    SHRTIM_FALT_FILTER_9, SHRTIM_FALT_FILTER_10, SHRTIM_FALT_FILTER_11,
    SHRTIM_FALT_FILTER_12, SHRTIM_FALT_FILTER_13, SHRTIM_FALT_FILTER_14,
    SHRTIM_FALT_FILTER_15,
};

static const uint32_t s_chp_duty[8] = {
    SHRTIM_CHP_DUTYCYCLE_0, SHRTIM_CHP_DUTYCYCLE_125, SHRTIM_CHP_DUTYCYCLE_250,
    SHRTIM_CHP_DUTYCYCLE_375, SHRTIM_CHP_DUTYCYCLE_500, SHRTIM_CHP_DUTYCYCLE_625,
    SHRTIM_CHP_DUTYCYCLE_750, SHRTIM_CHP_DUTYCYCLE_875,
};

static const uint32_t s_chp_psc[16] = {
    SHRTIM_CHP_PRESCALER_DIV16, SHRTIM_CHP_PRESCALER_DIV32,
    SHRTIM_CHP_PRESCALER_DIV48, SHRTIM_CHP_PRESCALER_DIV64,
    SHRTIM_CHP_PRESCALER_DIV80, SHRTIM_CHP_PRESCALER_DIV96,
    SHRTIM_CHP_PRESCALER_DIV112, SHRTIM_CHP_PRESCALER_DIV128,
    SHRTIM_CHP_PRESCALER_DIV144, SHRTIM_CHP_PRESCALER_DIV160,
    SHRTIM_CHP_PRESCALER_DIV176, SHRTIM_CHP_PRESCALER_DIV192,
    SHRTIM_CHP_PRESCALER_DIV208, SHRTIM_CHP_PRESCALER_DIV224,
    SHRTIM_CHP_PRESCALER_DIV240, SHRTIM_CHP_PRESCALER_DIV256,
};

/* ---------------------------------------------------------------------- */
/* ADC trigger source matrices                                             */
/* ---------------------------------------------------------------------- */

/* Event index inside the matrices: 0 = UPDATE (mapped to the period event),
 * 1..4 = CMP1..CMP4, 5 = RSTRO. A 0 entry means the silicon does not offer
 * that (trigger-group, unit, event) source; shrtim_adc_trigger() then
 * returns SHRTIM_ERR_PARAM. UPDATE is mapped to the timer period event,
 * which is the instant the preload update happens by default. */

#define ADTG_SRC_U 0u
#define ADTG_SRC_C1 1u
#define ADTG_SRC_C2 2u
#define ADTG_SRC_C3 3u
#define ADTG_SRC_C4 4u
#define ADTG_SRC_R 5u

/* ADC triggers 1 and 3: group1 covers master + A/B/C, group2 covers D/E/F. */
static const uint32_t s_adtg13[7][6] = {
    {SHRTIM_ADTG13_SOURCE_GROUP1_MPRD, SHRTIM_ADTG13_SOURCE_GROUP1_MCMP1,
     SHRTIM_ADTG13_SOURCE_GROUP1_MCMP2, SHRTIM_ADTG13_SOURCE_GROUP1_MCMP3,
     SHRTIM_ADTG13_SOURCE_GROUP1_MCMP4, 0},
    {SHRTIM_ADTG13_SOURCE_GROUP1_TAPRD, SHRTIM_ADTG13_SOURCE_GROUP1_TACMP1,
     SHRTIM_ADTG13_SOURCE_GROUP1_TACMP2, SHRTIM_ADTG13_SOURCE_GROUP1_TACMP3,
     SHRTIM_ADTG13_SOURCE_GROUP1_TACMP4, SHRTIM_ADTG13_SOURCE_GROUP1_TARSTRO},
    {SHRTIM_ADTG13_SOURCE_GROUP1_TBPRD, SHRTIM_ADTG13_SOURCE_GROUP1_TBCMP1,
     SHRTIM_ADTG13_SOURCE_GROUP1_TBCMP2, SHRTIM_ADTG13_SOURCE_GROUP1_TBCMP3,
     SHRTIM_ADTG13_SOURCE_GROUP1_TBCMP4, SHRTIM_ADTG13_SOURCE_GROUP1_TBRSTRO},
    {SHRTIM_ADTG13_SOURCE_GROUP1_TCPRD, SHRTIM_ADTG13_SOURCE_GROUP1_TCCMP1,
     SHRTIM_ADTG13_SOURCE_GROUP1_TCCMP2, SHRTIM_ADTG13_SOURCE_GROUP1_TCCMP3,
     SHRTIM_ADTG13_SOURCE_GROUP1_TCCMP4, 0},
    {SHRTIM_ADTG13_SOURCE_GROUP2_TDPRD, SHRTIM_ADTG13_SOURCE_GROUP2_TDCMP1,
     SHRTIM_ADTG13_SOURCE_GROUP2_TDCMP2, SHRTIM_ADTG13_SOURCE_GROUP2_TDCMP3,
     SHRTIM_ADTG13_SOURCE_GROUP2_TDCMP4, 0},
    {SHRTIM_ADTG13_SOURCE_GROUP2_TEPRD, SHRTIM_ADTG13_SOURCE_GROUP2_TECMP1,
     SHRTIM_ADTG13_SOURCE_GROUP2_TECMP2, SHRTIM_ADTG13_SOURCE_GROUP2_TECMP3,
     SHRTIM_ADTG13_SOURCE_GROUP2_TECMP4, 0},
    {SHRTIM_ADTG13_SOURCE_GROUP2_TFPRD, SHRTIM_ADTG13_SOURCE_GROUP2_TFCMP1,
     SHRTIM_ADTG13_SOURCE_GROUP2_TFCMP2, SHRTIM_ADTG13_SOURCE_GROUP2_TFCMP3,
     SHRTIM_ADTG13_SOURCE_GROUP2_TFCMP4, SHRTIM_ADTG13_SOURCE_GROUP2_TFRSTRO},
};

/* ADC triggers 2 and 4: group1 covers master + A/B/C, group2 covers D/E/F. */
static const uint32_t s_adtg24[7][6] = {
    {SHRTIM_ADTG24_SOURCE_GROUP1_MPRD, SHRTIM_ADTG24_SOURCE_GROUP1_MCMP1,
     SHRTIM_ADTG24_SOURCE_GROUP1_MCMP2, SHRTIM_ADTG24_SOURCE_GROUP1_MCMP3,
     SHRTIM_ADTG24_SOURCE_GROUP1_MCMP4, 0},
    {SHRTIM_ADTG24_SOURCE_GROUP1_TAPRD, SHRTIM_ADTG24_SOURCE_GROUP1_TACMP1,
     SHRTIM_ADTG24_SOURCE_GROUP1_TACMP2, SHRTIM_ADTG24_SOURCE_GROUP1_TACMP3,
     SHRTIM_ADTG24_SOURCE_GROUP1_TACMP4, 0},
    {SHRTIM_ADTG24_SOURCE_GROUP1_TBPRD, SHRTIM_ADTG24_SOURCE_GROUP1_TBCMP1,
     SHRTIM_ADTG24_SOURCE_GROUP1_TBCMP2, SHRTIM_ADTG24_SOURCE_GROUP1_TBCMP3,
     SHRTIM_ADTG24_SOURCE_GROUP1_TBCMP4, 0},
    {SHRTIM_ADTG24_SOURCE_GROUP1_TCPRD, SHRTIM_ADTG24_SOURCE_GROUP1_TCCMP1,
     SHRTIM_ADTG24_SOURCE_GROUP1_TCCMP2, SHRTIM_ADTG24_SOURCE_GROUP1_TCCMP3,
     SHRTIM_ADTG24_SOURCE_GROUP1_TCCMP4, SHRTIM_ADTG24_SOURCE_GROUP1_TCRSTRO},
    {SHRTIM_ADTG24_SOURCE_GROUP2_TDPRD, SHRTIM_ADTG24_SOURCE_GROUP2_TDCMP1,
     SHRTIM_ADTG24_SOURCE_GROUP2_TDCMP2, SHRTIM_ADTG24_SOURCE_GROUP2_TDCMP3,
     SHRTIM_ADTG24_SOURCE_GROUP2_TDCMP4, SHRTIM_ADTG24_SOURCE_GROUP2_TDRSTRO},
    /* No TEPRD in this group (silicon limitation): UPDATE on unit E is
     * rejected for ADC triggers 2 and 4. */
    {0, SHRTIM_ADTG24_SOURCE_GROUP2_TECMP1,
     SHRTIM_ADTG24_SOURCE_GROUP2_TECMP2, SHRTIM_ADTG24_SOURCE_GROUP2_TECMP3,
     SHRTIM_ADTG24_SOURCE_GROUP2_TECMP4, SHRTIM_ADTG24_SOURCE_GROUP2_TERSTRO},
    {SHRTIM_ADTG24_SOURCE_GROUP2_TFPRD, SHRTIM_ADTG24_SOURCE_GROUP2_TFCMP1,
     SHRTIM_ADTG24_SOURCE_GROUP2_TFCMP2, SHRTIM_ADTG24_SOURCE_GROUP2_TFCMP3,
     SHRTIM_ADTG24_SOURCE_GROUP2_TFCMP4, 0},
};

/* ADC triggers 5, 7 and 9 (single source register each). */
static const uint32_t s_adtg579[7][6] = {
    {SHRTIM_ADTG579_MPRD, SHRTIM_ADTG579_MCMP1, SHRTIM_ADTG579_MCMP2,
     SHRTIM_ADTG579_MCMP3, SHRTIM_ADTG579_MCMP4, 0},
    {SHRTIM_ADTG579_TAPRD, SHRTIM_ADTG579_TACMP1, SHRTIM_ADTG579_TACMP2,
     SHRTIM_ADTG579_TACMP3, SHRTIM_ADTG579_TACMP4, SHRTIM_ADTG579_TARSTRO},
    {SHRTIM_ADTG579_TBPRD, SHRTIM_ADTG579_TBCMP1, SHRTIM_ADTG579_TBCMP2,
     SHRTIM_ADTG579_TBCMP3, SHRTIM_ADTG579_TBCMP4, SHRTIM_ADTG579_TBRSTRO},
    {SHRTIM_ADTG579_TCPRD, SHRTIM_ADTG579_TCCMP1, SHRTIM_ADTG579_TCCMP2,
     SHRTIM_ADTG579_TCCMP3, SHRTIM_ADTG579_TCCMP4, 0},
    {SHRTIM_ADTG579_TDPRD, SHRTIM_ADTG579_TDCMP1, SHRTIM_ADTG579_TDCMP2,
     SHRTIM_ADTG579_TDCMP3, SHRTIM_ADTG579_TDCMP4, 0},
    {SHRTIM_ADTG579_TEPRD, SHRTIM_ADTG579_TECMP1, SHRTIM_ADTG579_TECMP2,
     SHRTIM_ADTG579_TECMP3, SHRTIM_ADTG579_TECMP4, 0},
    {SHRTIM_ADTG579_TFPRD, SHRTIM_ADTG579_TFCMP1, SHRTIM_ADTG579_TFCMP2,
     SHRTIM_ADTG579_TFCMP3, SHRTIM_ADTG579_TFCMP4, SHRTIM_ADTG579_TFRSTRO},
};

/* ADC triggers 6, 8 and 10 (single source register each). */
static const uint32_t s_adtg6810[7][6] = {
    {SHRTIM_ADTG6810_MPRD, SHRTIM_ADTG6810_MCMP1, SHRTIM_ADTG6810_MCMP2,
     SHRTIM_ADTG6810_MCMP3, SHRTIM_ADTG6810_MCMP4, 0},
    {SHRTIM_ADTG6810_TAPRD, SHRTIM_ADTG6810_TACMP1, SHRTIM_ADTG6810_TACMP2,
     SHRTIM_ADTG6810_TACMP3, SHRTIM_ADTG6810_TACMP4, 0},
    {SHRTIM_ADTG6810_TBPRD, SHRTIM_ADTG6810_TBCMP1, SHRTIM_ADTG6810_TBCMP2,
     SHRTIM_ADTG6810_TBCMP3, SHRTIM_ADTG6810_TBCMP4, 0},
    {SHRTIM_ADTG6810_TCPRD, SHRTIM_ADTG6810_TCCMP1, SHRTIM_ADTG6810_TCCMP2,
     SHRTIM_ADTG6810_TCCMP3, SHRTIM_ADTG6810_TCCMP4, SHRTIM_ADTG6810_TCRSTRO},
    {SHRTIM_ADTG6810_TDPRD, SHRTIM_ADTG6810_TDCMP1, SHRTIM_ADTG6810_TDCMP2,
     SHRTIM_ADTG6810_TDCMP3, SHRTIM_ADTG6810_TDCMP4, SHRTIM_ADTG6810_TDRSTRO},
    /* No TEPRD in this group (silicon limitation): UPDATE on unit E is
     * rejected for ADC triggers 6, 8 and 10. */
    {0, SHRTIM_ADTG6810_TECMP1, SHRTIM_ADTG6810_TECMP2,
     SHRTIM_ADTG6810_TECMP3, SHRTIM_ADTG6810_TECMP4, SHRTIM_ADTG6810_TERSTRO},
    {SHRTIM_ADTG6810_TFPRD, SHRTIM_ADTG6810_TFCMP1, SHRTIM_ADTG6810_TFCMP2,
     SHRTIM_ADTG6810_TFCMP3, SHRTIM_ADTG6810_TFCMP4, 0},
};

/* ---------------------------------------------------------------------- */
/* Internal helpers                                                        */
/* ---------------------------------------------------------------------- */

/** High-resolution prescaler candidates, highest resolution first. */
static const uint32_t s_ratio_hw[8] = {
    SHRTIM_PRESCALERRATIO_MUL32, SHRTIM_PRESCALERRATIO_MUL16,
    SHRTIM_PRESCALERRATIO_MUL8, SHRTIM_PRESCALERRATIO_MUL4,
    SHRTIM_PRESCALERRATIO_MUL2, SHRTIM_PRESCALERRATIO_DIV1,
    SHRTIM_PRESCALERRATIO_DIV2, SHRTIM_PRESCALERRATIO_DIV4,
};

/** Numerators of the s_ratio_hw[] candidates: fHRCK = fSHRTIM * num/den. */
static const uint16_t s_ratio_num[8] = {32u, 16u, 8u, 4u, 2u, 1u, 1u, 1u};

/** Denominators of the s_ratio_hw[] candidates. */
static const uint16_t s_ratio_den[8] = {1u, 1u, 1u, 1u, 1u, 1u, 2u, 4u};

/**
 * @brief Solve the high-resolution prescaler and period for a PWM frequency.
 *
 * Walks the prescaler candidates from the highest resolution (fHRCK =
 * fSHRTIM x 32) downwards and picks the first one whose period
 * fHRCK / (hz * mult) fits 1..0xFFFF. mult is 2 for center alignment
 * (one PWM cycle spans 2 * period counter ticks).
 *
 * @param[in] hz Requested PWM frequency in Hz.
 * @param[in] mult Cycle multiplier (1 edge, 2 center).
 * @param[out] ratio CKPSC value to program.
 * @param[out] period Period value to program.
 * @param[out] idx Index of the chosen candidate in s_ratio_hw.
 * @return SHRTIM_OK or SHRTIM_ERR_FREQ.
 */
static int solve_timing(uint32_t hz, uint32_t mult,
                        uint32_t *ratio, uint32_t *period, uint8_t *idx)
{
    unsigned i;

    if (hz == 0u) {
        return SHRTIM_ERR_FREQ;
    }
    for (i = 0u; i < 8u; i++) {
        /* period = round(fSHRTIM * num / (den * hz * mult)) */
        uint64_t div = (uint64_t)hz * (uint64_t)mult * (uint64_t)s_ratio_den[i];
        uint64_t p = ((uint64_t)s_clk_hz * (uint64_t)s_ratio_num[i] + div / 2u) / div;

        if ((p >= 1u) && (p <= (uint64_t)SHRTIM_HAL_PERIOD_MAX)) {
            *ratio = s_ratio_hw[i];
            *period = (uint32_t)p;
            *idx = (uint8_t)i;
            return SHRTIM_OK;
        }
    }
    return SHRTIM_ERR_FREQ;
}

/** Compare value for an edge-aligned duty: round(P * duty / 1000), clamped
 * to P-1 so the reset event never coincides with the period event. */
static uint32_t edge_compare(uint32_t period, uint32_t duty)
{
    uint64_t c = (((uint64_t)period * (uint64_t)duty) + 500u) / 1000u;

    if (c >= (uint64_t)period) {
        c = (uint64_t)period - 1u;
    }
    return (uint32_t)c;
}

/**
 * @brief Compare value for a center-aligned duty in the MASTER time base.
 *
 * LA + JLink evidence (round 7): while a slave counter is phase-locked to
 * the master (reset trigger = SHRTIM_RESETTRIG_MASTER_PER) this silicon
 * runs the slave in the MASTER's time base: compare events fire at
 * CMPx / fHRCK(master), once per master period, regardless of the slave's
 * own CKPSC/UPDOWNM fields (those registers read back as written but are
 * not effective while locked; dropping the lock mid-run does not restore
 * them either). With the master period register holding the SAME P as the
 * slave PRD (one PWM cycle), the output windows are:
 *
 *   duty <= 500 : c = round(P * duty / 1000)
 *                 x1: SET = MASTERPER, RST = CMP1 -> [per, c]  duty = c/P
 *   duty >  500 : c = round(P * (1000 - duty) / 1000)
 *                 x1: SET = CMP1, RST = MASTERPER -> [c, per]
 *                                                      duty = 1 - c/P
 * x2 mirrors the sources (exact complement). LA-verified: duty 300 ->
 * x1 = 30.00 %, x2 = 70.00 % at 10.02 kHz (CMP1 = 14400 = P * 300/1000).
 *
 * The User Manual documents the underlying behaviour (located after the
 * LA/JLink rounds): CN_UM_N32H47x_48x_Series_User_Manual_V1.1.0,
 * chapter 9 (SHRTIM), "Timer clock and prescaler":
 *  - "when several timers are enabled, the prescaler synchronizes with
 *    the prescaler of the timer that was started first" — the master is
 *    started first by center_master_setup(), so a locked slave follows
 *    the master time base;
 *  - "the prescaler cannot be modified once the counter is enabled
 *    (TxCNTEN/MCNTEN set)" — and the up-down section adds that UPDOWNM
 *    must not change while running;
 *  - the warning that events crossing timer boundaries (output set/reset,
 *    counter reset, update, ...) require EQUAL prescaler ratios in the
 *    coupled timers, otherwise "the results are unpredictable". This
 *    scheme couples slave and master (MASTERPER set/reset + MASTER_PER
 *    reset trigger) with unequal ratios; rounds 4-7 measured the outcome
 *    as the deterministic follow-the-master behaviour encoded above.
 *
 * @note The pulses are anchored to the master period edge (leading for
 *       duty <= 500, trailing above), not geometrically centered inside
 *       the PWM cycle. Rounds 3-7 found no crossbar combination on this
 *       silicon that yields geometrically centered pulses with exact
 *       duties; duty and frequency are exact and the pair is a true
 *       complementary partition of the cycle.
 *
 * @param[in] period Period P (programmed to both the slave PRD and the
 *                   master MPRD; one full PWM cycle in master ticks).
 * @param[in] duty Duty in 0.1 % steps (caller guarantees 1..999).
 * @return Compare value in master time-base ticks.
 */
static uint32_t center_compare(uint32_t period, uint32_t duty)
{
    uint64_t scaled;

    if (duty <= 500u) {
        scaled = (uint64_t)period * (uint64_t)duty;
    } else {
        scaled = (uint64_t)period * (1000u - (uint64_t)duty);
    }
    return (uint32_t)((scaled + 500u) / 1000u);
}

/**
 * @brief Program the MASTER timer as the center-alignment phase anchor.
 *
 * The master must complete one period per PWM cycle (= 2 * P slave ticks).
 * 2*P can exceed the 16-bit master period register (e.g. P = 48000 at
 * 10 kHz), so the master runs one prescaler step SLOWER than the slave
 * and its period register holds the SAME P: master cycle = P / (fHRCK/2)
 * = 2*P / fHRCK = one PWM cycle. The master counter is started here and
 * keeps running (center units share it; stopping one unit never stops the
 * master).
 *
 * Constraints (master is a single shared resource):
 *  - All center-aligned units must run at the same PWM frequency. When
 *    units with different frequencies are configured, the LAST
 *    shrtim_pwm_init()/shrtim_pwm_freq() for a center unit reprograms the
 *    master; earlier center units at other frequencies lose their anchor.
 *  - A later shrtim_master_period() call overrides the anchor period the
 *    same way (its own semantics are unchanged).
 *  - shrtim_pwm_sync_to_master()/shrtim_exev_reset_unit() write the same
 *    reset-trigger register; calling them on a center unit can break the
 *    phase lock.
 * Edge-aligned units are not affected.
 *
 * @note The master-vs-slave prescaler ratios are intentionally unequal
 *       here; the User Manual (see center_compare() for the citation)
 *       warns that events crossing timers with unequal prescalers are
 *       unpredictable in general — this scheme relies on the
 *       deterministic follow-the-master behaviour measured on this
 *       silicon and encoded in center_compare().
 *
 * @param[in] ratio_idx Slave prescaler index in s_ratio_hw (0..6; the DIV4
 *                      last step has no slower master counterpart and is
 *                      rejected by the callers).
 * @param[in] period Slave period P (also programmed to the master).
 */
static void center_master_setup(uint8_t ratio_idx, uint32_t period)
{
    uint32_t m_ratio = s_ratio_hw[(unsigned)ratio_idx + 1u];
    unsigned retry;

    for (retry = 0u; retry < 3u; retry++) {
        SHRTIM_TIM_SetPrescaler(SHRTIM1, SHRTIM_TIMER_MASTER, m_ratio);
        SHRTIM_TIM_EnablePreload(SHRTIM1, SHRTIM_TIMER_MASTER);
        SHRTIM_TIM_SetPeriod(SHRTIM1, SHRTIM_TIMER_MASTER, period);
        /* Roll-over generated update: keeps the master preload live and
         * raises master UPDATE events for interrupt users. */
        SHRTIM_TIM_SetUpdateTrig(SHRTIM1, SHRTIM_TIMER_MASTER,
                                 SHRTIM_UPDATETRIG_RESET);
        SHRTIM_ForceUpdate(SHRTIM1, SHRTIM_TIMER_MASTER);
        SHRTIM_TIM_CounterEnable(SHRTIM1, SHRTIM_TIMER_MASTER);
        if (SHRTIM_TIM_GetPrescaler(SHRTIM1, SHRTIM_TIMER_MASTER) == m_ratio) {
            break;
        }
    }
}

/**
 * @brief Program the set/reset crossbars and compare values of one unit.
 *
 * Central re-computation used by pwm_init / pwm_freq / pwm_duty / pwm_duty2.
 * Degenerate duties are handled without compare events:
 *  - duty 0   : x1 set source cleared, reset on period (permanently inactive)
 *  - duty 1000: x1 reset source cleared, set on period (permanently active)
 * A zero duty2 mirrors the x1 crossbars (set/reset swapped) which yields the
 * exact complement of x1 at all duty settings including the degenerate ones.
 */
static void apply_crossbars(unsigned unit)
{
    unit_state_t *st = &s_unit[unit];
    uint32_t tmr = s_tmr_id[unit];
    uint32_t out1 = s_out_id[2u * unit];
    uint32_t out2 = s_out_id[2u * unit + 1u];
    uint32_t set1 = SHRTIM_OUTPUTSET_NONE;
    uint32_t rst1 = SHRTIM_OUTPUTRESET_NONE;
    uint32_t set2;
    uint32_t rst2;

    if ((st->align == (uint8_t)SHRTIM_ALIGN_EDGE) || (st->duty == 0u) ||
        (st->duty >= 1000u)) {
        /* Edge aligned (also used for the degenerate center cases, where a
         * compare pair cannot express 0 % / 100 % symmetrically). */
        if (st->duty == 0u) {
            set1 = SHRTIM_OUTPUTSET_NONE;
            rst1 = SHRTIM_OUTPUTRESET_TIMPER;
        } else if (st->duty >= 1000u) {
            set1 = SHRTIM_OUTPUTSET_TIMPER;
            rst1 = SHRTIM_OUTPUTRESET_NONE;
        } else {
            set1 = SHRTIM_OUTPUTSET_TIMPER;
            rst1 = SHRTIM_OUTPUTRESET_TIMCMP1;
            SHRTIM_TIM_SetCompare1(SHRTIM1, tmr, edge_compare(st->period, st->duty));
        }

        if (st->duty2 == 0u) {
            set2 = rst1; /* mirror: x2 = complement of x1 */
            rst2 = set1;
        } else if (st->duty2 >= 1000u) {
            set2 = SHRTIM_OUTPUTSET_TIMPER;
            rst2 = SHRTIM_OUTPUTRESET_NONE;
        } else {
            set2 = SHRTIM_OUTPUTSET_TIMPER;
            rst2 = SHRTIM_OUTPUTRESET_TIMCMP2;
            SHRTIM_TIM_SetCompare2(SHRTIM1, tmr, edge_compare(st->period, st->duty2));
        }
    } else {
        /* Center aligned, 1..999 per-mille duty: master-period-anchored
         * single compare per output (see center_compare() and
         * center_master_setup() for the derivation and constraints). */
        uint32_t c = center_compare(st->period, st->duty);

        if (st->duty <= 500u) {
            set1 = SHRTIM_OUTPUTSET_MASTERPER;
            rst1 = SHRTIM_OUTPUTRESET_TIMCMP1;
        } else {
            set1 = SHRTIM_OUTPUTSET_TIMCMP1;
            rst1 = SHRTIM_OUTPUTRESET_MASTERPER;
        }
        SHRTIM_TIM_SetCompare1(SHRTIM1, tmr, c);

        if (st->duty2 == 0u) {
            set2 = rst1; /* mirror: x2 = complement of x1 */
            rst2 = set1;
        } else if (st->duty2 >= 1000u) {
            set2 = SHRTIM_OUTPUTSET_TIMPER;
            rst2 = SHRTIM_OUTPUTRESET_NONE;
        } else {
            uint32_t c2 = center_compare(st->period, st->duty2);

            if (st->duty2 <= 500u) {
                set2 = SHRTIM_OUTPUTSET_MASTERPER;
                rst2 = SHRTIM_OUTPUTRESET_TIMCMP3;
            } else {
                set2 = SHRTIM_OUTPUTSET_TIMCMP3;
                rst2 = SHRTIM_OUTPUTRESET_MASTERPER;
            }
            SHRTIM_TIM_SetCompare3(SHRTIM1, tmr, c2);
        }
    }

    SHRTIM_OUT_SetOutputSetSrc(SHRTIM1, out1, set1);
    SHRTIM_OUT_SetOutputResetSrc(SHRTIM1, out1, rst1);
    SHRTIM_OUT_SetOutputSetSrc(SHRTIM1, out2, set2);
    SHRTIM_OUT_SetOutputResetSrc(SHRTIM1, out2, rst2);
}

/**
 * @brief Program the dead-time generator of one unit from nanoseconds.
 *
 * The DTG prescaler candidates run from fSHRTIM x 8 (highest resolution)
 * down to fSHRTIM / 16. The first candidate whose tick counts for both
 * edges fit the 9-bit DTR/DTF fields is used. 0/0 disables dead-time.
 *
 * @return SHRTIM_OK or SHRTIM_ERR_PARAM when no prescaler covers the range.
 */
static int program_deadtime(unsigned unit, uint32_t rise_ns, uint32_t fall_ns)
{
    static const uint32_t psc[8] = {
        SHRTIM_DT_PRESCALER_MUL8, SHRTIM_DT_PRESCALER_MUL4,
        SHRTIM_DT_PRESCALER_MUL2, SHRTIM_DT_PRESCALER_DIV1,
        SHRTIM_DT_PRESCALER_DIV2, SHRTIM_DT_PRESCALER_DIV4,
        SHRTIM_DT_PRESCALER_DIV8, SHRTIM_DT_PRESCALER_DIV16,
    };
    static const uint16_t num[8] = {8u, 4u, 2u, 1u, 1u, 1u, 1u, 1u};
    static const uint16_t den[8] = {1u, 1u, 1u, 1u, 2u, 4u, 8u, 16u};
    uint32_t tmr = s_tmr_id[unit];
    unsigned i;

    if ((rise_ns == 0u) && (fall_ns == 0u)) {
        SHRTIM_TIM_DisableDeadTime(SHRTIM1, tmr);
        SHRTIM_DT_SetRisingValue(SHRTIM1, tmr, 0u);
        SHRTIM_DT_SetFallingValue(SHRTIM1, tmr, 0u);
        return SHRTIM_OK;
    }

    for (i = 0u; i < 8u; i++) {
        /* ticks = round(ns * fSHRTIM * num / (den * 1e9)) */
        uint64_t div = (uint64_t)den[i] * 1000000000u;
        uint64_t tr = ((uint64_t)rise_ns * (uint64_t)s_clk_hz * (uint64_t)num[i] + div / 2u) / div;
        uint64_t tf = ((uint64_t)fall_ns * (uint64_t)s_clk_hz * (uint64_t)num[i] + div / 2u) / div;

        if ((tr <= (uint64_t)SHRTIM_HAL_DTV_MAX) && (tf <= (uint64_t)SHRTIM_HAL_DTV_MAX)) {
            SHRTIM_DT_SetPrescaler(SHRTIM1, tmr, psc[i]);
            SHRTIM_DT_SetRisingValue(SHRTIM1, tmr, (uint32_t)tr);
            SHRTIM_DT_SetFallingValue(SHRTIM1, tmr, (uint32_t)tf);
            SHRTIM_TIM_EnableDeadTime(SHRTIM1, tmr);
            return SHRTIM_OK;
        }
    }
    return SHRTIM_ERR_PARAM;
}

/** Invoke the registered callback of one (unit, event) pair if any. */
static void dispatch(shrtim_unit_t unit, shrtim_evt_t evt)
{
    unsigned row = (unit == SHRTIM_UNIT_MASTER) ? SHRTIM_HAL_UNITS : (unsigned)unit;

    if (s_cb[row][evt] != NULL) {
        s_cb[row][evt](unit, evt, s_cb_user[row][evt]);
    }
}

/** Vendor timer id of a unit; master maps to SHRTIM_TIMER_MASTER. */
static uint32_t timer_of(shrtim_unit_t unit)
{
    if (unit == SHRTIM_UNIT_MASTER) {
        return SHRTIM_TIMER_MASTER;
    }
    return s_tmr_id[unit];
}

/** Per-timer register address, reusing the vendor offset table defined
 * at file scope in n32h47x_48x_shrtim.h (0 = master, 1..6 = A..F). This
 * is the exact addressing the LA-proven working EnableIT_* functions
 * use. */
static volatile uint32_t *tmr_reg(volatile uint32_t *base, unsigned idx)
{
    return (volatile uint32_t *)((uint32_t)base + REG_OFFSET_TAB_TIMER[idx]);
}

/** Slave-timer event scan order with the shared TxIDEN/TxINTSTS/TxINTCLR
 * bit masks (the three register triads use identical bit positions,
 * verified against the device header; note REP is bit5 on slaves, NOT the
 * master's bit4 that several vendor ClearFlag_* helpers write). */
static const shrtim_evt_t s_t_evt[9] = {
    SHRTIM_EVT_UPDATE, SHRTIM_EVT_REPT, SHRTIM_EVT_CMP1, SHRTIM_EVT_CMP2,
    SHRTIM_EVT_CMP3, SHRTIM_EVT_CMP4, SHRTIM_EVT_CPT1, SHRTIM_EVT_CPT2,
    SHRTIM_EVT_RSTRO,
};
static const uint32_t s_t_bit[9] = {
    SHRTIM_TXIDEN_UPDIEN,  SHRTIM_TXIDEN_REPTIEN, SHRTIM_TXIDEN_CMP1IEN,
    SHRTIM_TXIDEN_CMP2IEN, SHRTIM_TXIDEN_CMP3IEN, SHRTIM_TXIDEN_CMP4IEN,
    SHRTIM_TXIDEN_CPT1IEN, SHRTIM_TXIDEN_CPT2IEN, SHRTIM_TXIDEN_RSTROIEN,
};

/** Master event scan order with the MINTSTS/MIDEN/MINTCLR bit masks. */
static const shrtim_evt_t s_m_evt[7] = {
    SHRTIM_EVT_UPDATE,  SHRTIM_EVT_REPT,     SHRTIM_EVT_CMP1,
    SHRTIM_EVT_CMP2,    SHRTIM_EVT_CMP3,     SHRTIM_EVT_CMP4,
    SHRTIM_EVT_MST_SYNC,
};
static const uint32_t s_m_bit[7] = {
    SHRTIM_MIDEN_MUPDIEN,  SHRTIM_MIDEN_MREPTIEN, SHRTIM_MIDEN_MCMP1IEN,
    SHRTIM_MIDEN_MCMP2IEN, SHRTIM_MIDEN_MCMP3IEN, SHRTIM_MIDEN_MCMP4IEN,
    SHRTIM_MIDEN_SYNCINIEN,
};

/**
 * @brief Serve one pass of pending, interrupt-enabled timer events.
 *
 * Reads the unit's IDEN register once and serves ONLY events whose
 * interrupt-enable bit AND status flag are both set; flags of disabled
 * sources are left untouched for polling paths (e.g. software capture).
 * Flags are cleared with direct write-one-to-clear accesses (the vendor
 * ClearFlag_* family writes master bit masks into the slave registers -
 * their REP position differs - and does read-modify-writes). After each
 * clear the flag is re-read: if it is still set (ineffective clear or
 * immediate re-fire) a per-(unit, event) runaway counter increments;
 * 1000 consecutive occurrences auto-mask that source in IDEN and set the
 * degraded bitmap so a hardware quirk can never starve the main loop
 * again. Exactly one pass - no while-loop re-polling.
 *
 * @param[in] iden_base Master-side anchor of the IDEN register triad.
 * @param[in] sts_base Master-side anchor of the INTSTS register triad.
 * @param[in] clr_base Master-side anchor of the INTCLR register triad.
 * @param[in] idx Register-table index (0 = master, unit+1 = slave).
 * @param[in] unit Unit reported to the callbacks.
 * @param[in] storm Per-event consecutive re-fire counters (caller-owned).
 * @param[in] degraded Degraded (auto-masked) bitmap (caller-owned).
 * @param[in] evt_count Number of entries in evt/bit.
 * @param[in] evt Event list.
 * @param[in] bit Bit list matching evt.
 */
static void isr_pass(volatile uint32_t *iden_base,
                     volatile uint32_t *sts_base,
                     volatile uint32_t *clr_base, unsigned idx,
                     shrtim_unit_t unit, uint16_t *storm, uint16_t *degraded,
                     unsigned evt_count, const shrtim_evt_t *evt,
                     const uint32_t *bit)
{
    volatile uint32_t *iden = tmr_reg(iden_base, idx);
    volatile uint32_t *sts = tmr_reg(sts_base, idx);
    volatile uint32_t *clr = tmr_reg(clr_base, idx);
    uint32_t enable = *iden;
    uint32_t active = *sts;
    unsigned i;

    for (i = 0u; i < evt_count; i++) {
        uint32_t b = bit[i];

        if (((enable & b) == 0u) || ((active & b) == 0u)) {
            continue;
        }
        *clr = b;
        if ((*sts & b) != 0u) {
            if ((uint32_t)storm[i] + 1u >= 1000u) {
                *iden &= ~b; /* runaway source: auto-mask it */
                *degraded |= (uint16_t)(1u << i);
            } else {
                storm[i]++;
            }
        } else {
            storm[i] = 0u;
        }
        dispatch(unit, evt[i]);
    }
}

/**
 * @brief Enable or disable one unit event interrupt source.
 *
 * @return SHRTIM_OK, or SHRTIM_ERR_PARAM for events the silicon cannot
 * raise on the given unit (RSTRO/CPT on master, CMP5 anywhere).
 */
static int it_set(shrtim_unit_t unit, shrtim_evt_t evt, bool enable)
{
    uint32_t t = timer_of(unit);

    switch (evt) {
    case SHRTIM_EVT_UPDATE:
        if (enable) { SHRTIM_EnableIT_UPDATE(SHRTIM1, t); }
        else { SHRTIM_DisableIT_UPDATE(SHRTIM1, t); }
        break;
    case SHRTIM_EVT_REPT:
        if (enable) { SHRTIM_EnableIT_REP(SHRTIM1, t); }
        else { SHRTIM_DisableIT_REP(SHRTIM1, t); }
        break;
    case SHRTIM_EVT_CMP1:
        if (enable) { SHRTIM_EnableIT_CMP1(SHRTIM1, t); }
        else { SHRTIM_DisableIT_CMP1(SHRTIM1, t); }
        break;
    case SHRTIM_EVT_CMP2:
        if (enable) { SHRTIM_EnableIT_CMP2(SHRTIM1, t); }
        else { SHRTIM_DisableIT_CMP2(SHRTIM1, t); }
        break;
    case SHRTIM_EVT_CMP3:
        if (enable) { SHRTIM_EnableIT_CMP3(SHRTIM1, t); }
        else { SHRTIM_DisableIT_CMP3(SHRTIM1, t); }
        break;
    case SHRTIM_EVT_CMP4:
        if (enable) { SHRTIM_EnableIT_CMP4(SHRTIM1, t); }
        else { SHRTIM_DisableIT_CMP4(SHRTIM1, t); }
        break;
    case SHRTIM_EVT_CPT1:
        if (unit == SHRTIM_UNIT_MASTER) { return SHRTIM_ERR_PARAM; }
        if (enable) { SHRTIM_EnableIT_CPT1(SHRTIM1, t); }
        else { SHRTIM_DisableIT_CPT1(SHRTIM1, t); }
        break;
    case SHRTIM_EVT_CPT2:
        if (unit == SHRTIM_UNIT_MASTER) { return SHRTIM_ERR_PARAM; }
        if (enable) { SHRTIM_EnableIT_CPT2(SHRTIM1, t); }
        else { SHRTIM_DisableIT_CPT2(SHRTIM1, t); }
        break;
    case SHRTIM_EVT_RSTRO:
        if (unit == SHRTIM_UNIT_MASTER) { return SHRTIM_ERR_PARAM; }
        if (enable) { SHRTIM_EnableIT_RST(SHRTIM1, t); }
        else { SHRTIM_DisableIT_RST(SHRTIM1, t); }
        break;
    case SHRTIM_EVT_FAULT1:
    case SHRTIM_EVT_FAULT2:
    case SHRTIM_EVT_FAULT3:
    case SHRTIM_EVT_FAULT4:
    case SHRTIM_EVT_FAULT5:
    case SHRTIM_EVT_FAULT6: {
        unsigned f = (unsigned)evt - (unsigned)SHRTIM_EVT_FAULT1;

        switch (f) {
        case 0:
            if (enable) { SHRTIM_EnableIT_FALT1(SHRTIM1); }
            else { SHRTIM_DisableIT_FALT1(SHRTIM1); }
            break;
        case 1:
            if (enable) { SHRTIM_EnableIT_FALT2(SHRTIM1); }
            else { SHRTIM_DisableIT_FALT2(SHRTIM1); }
            break;
        case 2:
            if (enable) { SHRTIM_EnableIT_FALT3(SHRTIM1); }
            else { SHRTIM_DisableIT_FALT3(SHRTIM1); }
            break;
        case 3:
            if (enable) { SHRTIM_EnableIT_FALT4(SHRTIM1); }
            else { SHRTIM_DisableIT_FALT4(SHRTIM1); }
            break;
        case 4:
            if (enable) { SHRTIM_EnableIT_FALT5(SHRTIM1); }
            else { SHRTIM_DisableIT_FALT5(SHRTIM1); }
            break;
        default:
            if (enable) { SHRTIM_EnableIT_FALT6(SHRTIM1); }
            else { SHRTIM_DisableIT_FALT6(SHRTIM1); }
            break;
        }
        break;
    }
    case SHRTIM_EVT_SYSFLT:
        if (enable) { SHRTIM_EnableIT_SYSFALT(SHRTIM1); }
        else { SHRTIM_DisableIT_SYSFALT(SHRTIM1); }
        break;
    case SHRTIM_EVT_MST_SYNC:
        if (enable) { SHRTIM_EnableIT_SYNC(SHRTIM1); }
        else { SHRTIM_DisableIT_SYNC(SHRTIM1); }
        break;
    default:
        /* CMP5 has no interrupt-enable bit in the silicon. */
        return SHRTIM_ERR_PARAM;
    }
    return SHRTIM_OK;
}

/** Vendor fault-state encoding of shrtim_fault_out_t. */
static uint32_t fault_state_encode(shrtim_fault_out_t out)
{
    switch (out) {
    case SHRTIM_FAULT_OUT_ACTIVE:
        return SHRTIM_OUT_FAULTSTATE_ACTIVE;
    case SHRTIM_FAULT_OUT_INACTIVE:
        return SHRTIM_OUT_FAULTSTATE_INACTIVE;
    case SHRTIM_FAULT_OUT_HIGHZ:
        return SHRTIM_OUT_FAULTSTATE_HIGHZ;
    case SHRTIM_FAULT_OUT_NONE:
    default:
        return SHRTIM_OUT_FAULTSTATE_NO_ACTION;
    }
}

/* ---------------------------------------------------------------------- */
/* Global lifecycle                                                        */
/* ---------------------------------------------------------------------- */

int shrtim_init(uint32_t fshrtim_hz)
{
    uint32_t pll_src;
    uint64_t fin;
    volatile uint32_t spin;

    if (fshrtim_hz == 0u) {
        fshrtim_hz = 240000000u;
    }

    /* Prefer HSE when the oscillator is running, else HSI. */
    if (RCC_GetFlagStatus(RCC_FLAG_HSERDF) == SET) {
        pll_src = RCC_SHRTPLL_SRC_HSE;
        fin = (uint64_t)HSE_VALUE;
    } else {
        pll_src = RCC_SHRTPLL_SRC_HSI;
        fin = (uint64_t)HSI_VALUE;
    }

    if (RCC_ConfigSHRTPll(pll_src, fin, (uint64_t)fshrtim_hz, ENABLE) != SUCCESS) {
        return SHRTIM_ERR_CLOCK;
    }

    /* RCC_ConfigSHRTPll() does not wait for lock; poll the ready flag. */
    for (spin = 0u; spin < 2000000u; spin++) {
        if (RCC_GetFlagStatus(RCC_FLAG_SHRTPLLRDF) == SET) {
            break;
        }
    }
    if (RCC_GetFlagStatus(RCC_FLAG_SHRTPLLRDF) != SET) {
        return SHRTIM_ERR_CLOCK;
    }

    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPHEN_SHRTIM, ENABLE);
    RCC_EnableAHBPeriphReset(RCC_AHB_PERIPHRST_SHRTIM);

    /* Read back the real PLL output so period/duty math uses the achieved
     * frequency: f = fin * CLKF / 2^14 / (CLKR + 1) / 4 (vendor formula). */
    {
        uint32_t clkf = RCC->SHRTPLLCTRL1 & RCC_SHRTPLLCTRL1_CLKF;
        uint32_t clkr = (RCC->SHRTPLLCTRL2 & RCC_SHRTPLLCTRL2_CLKR) >> 26u;
        uint64_t f = ((fin * (uint64_t)clkf) + 8192u) / 16384u;

        f = f / ((uint64_t)clkr + 1u);
        s_clk_hz = (uint32_t)((f + 2u) / 4u);
    }

    /* A peripheral reset wiped every unit; drop stale state. */
    {
        unsigned u;

        for (u = 0u; u < SHRTIM_HAL_UNITS; u++) {
            s_unit[u].configured = 0u;
        }
    }
    return SHRTIM_OK;
}

uint32_t shrtim_clk_hz(void)
{
    return s_clk_hz;
}

void shrtim_deinit(void)
{
    unsigned i;

    SHRTIM_TIM_CounterDisable(SHRTIM1, SHRTIM_TIMER_ALL);
    for (i = 0u; i < 12u; i++) {
        SHRTIM_DisableOutput(SHRTIM1, s_out_id[i]);
    }

    RCC_EnableAHBPeriphReset(RCC_AHB_PERIPHRST_SHRTIM);
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPHEN_SHRTIM, DISABLE);
    (void)RCC_ConfigSHRTPll(RCC_SHRTPLL_SRC_HSI, 0u, 0u, DISABLE);

    for (i = 0u; i < SHRTIM_HAL_UNITS; i++) {
        s_unit[i].configured = 0u;
    }
    s_clk_hz = 0u;
}

/* ---------------------------------------------------------------------- */
/* Pin muxing                                                              */
/* ---------------------------------------------------------------------- */

/**
 * @brief Configure one pad for SHRTIM use with direct register writes.
 *
 * LA tests showed that re-muxing a pad previously owned by another
 * peripheral (PA10 was USART1_RX) did not take effect when routed through
 * GPIO_InitPeripheral(), while never-muxed pads worked. This helper
 * therefore rewrites every pad field itself - PMODE, AFSEL, PUPD, POTYPE,
 * SR and DS - with explicit clear+set operations, overriding any previous
 * owner unconditionally, and verifies the AFSEL/PMODE read-back with a
 * bounded retry.
 *
 * @param[in] pin Pad map entry (port, pin position, AF, RCC clock).
 * @param[in] pmode PMODE field value (GPIO_MODE_AF_PP or GPIO_MODE_INPUT).
 */
static void pin_apply(const pin_map_t *pin, uint32_t pmode)
{
    unsigned pos = pin->pos;
    uint32_t mask = 1u << pos;
    unsigned reg = pos >> 2;
    unsigned slot = (pos & 3u) * 8u;
    unsigned pair = pos * 2u;
    unsigned retry;

    RCC_EnableAHB1PeriphClk(pin->rcc_clk, ENABLE);

    for (retry = 0u; retry < 3u; retry++) {
        /* Output stage: push-pull, fast slew, 8 mA drive. */
        pin->port->POTYPE &= ~mask;
        pin->port->SR = (pin->port->SR & ~mask) | (GPIO_SLEW_RATE_FAST << pos);
        pin->port->DS = (pin->port->DS & ~(GPIO_DS_BIT_MASK << pair))
                        | (GPIO_DS_8mA << pair);
        /* Direction, pulls and the AF mux field: full clear+set. */
        pin->port->PMODE = (pin->port->PMODE & ~(GPIO_PMODE_BIT_MASK << pair))
                           | ((pmode & GPIO_PMODE_BIT_MASK) << pair);
        pin->port->PUPD &= ~(GPIO_PUPD_BIT_MASK << pair); /* no pull */
        pin->port->AFSEL[reg] = (pin->port->AFSEL[reg]
                                 & ~(GPIO_AFSEL0_AFSEL0 << slot))
                                | ((pin->af & GPIO_AFSEL0_AFSEL0) << slot);

        if ((((pin->port->AFSEL[reg] >> slot) & GPIO_AFSEL0_AFSEL0)
             == (pin->af & GPIO_AFSEL0_AFSEL0))
            && (((pin->port->PMODE >> pair) & GPIO_PMODE_BIT_MASK)
                == (pmode & GPIO_PMODE_BIT_MASK))) {
            break;
        }
    }
}

int shrtim_pinmux(shrtim_out_t out)
{
    if ((unsigned)out >= 12u) {
        return SHRTIM_ERR_PARAM;
    }
    pin_apply(&s_out_pins[out], GPIO_MODE_AF_PP);
    return SHRTIM_OK;
}

int shrtim_pinmux_fault(uint8_t fault)
{
    if ((fault < 1u) || (fault > 6u)) {
        return SHRTIM_ERR_PARAM;
    }
    /* Digital input with the AF field routed to the SHRTIM fault mux. */
    pin_apply(&s_flt_pins[fault - 1u], GPIO_MODE_INPUT);
    return SHRTIM_OK;
}

/* ---------------------------------------------------------------------- */
/* PWM: configure / run                                                    */
/* ---------------------------------------------------------------------- */

int shrtim_pwm_init(shrtim_unit_t unit, const shrtim_pwm_cfg_t *cfg)
{
    uint32_t ratio;
    uint32_t period;
    uint32_t mult;
    uint32_t mode;
    uint32_t tmr;
    uint32_t out1;
    uint32_t out2;
    uint8_t ratio_idx;
    int rc;

    if ((cfg == NULL) || (unit == SHRTIM_UNIT_MASTER) || ((unsigned)unit >= SHRTIM_HAL_UNITS)) {
        return SHRTIM_ERR_PARAM;
    }
    if ((cfg->freq_hz == 0u) || (cfg->duty_x1000 > 1000u) ||
        (cfg->duty2_x1000 > 1000u) ||
        ((uint32_t)cfg->align > (uint32_t)SHRTIM_ALIGN_CENTER) ||
        ((uint32_t)cfg->run > (uint32_t)SHRTIM_RUN_RETRIG)) {
        return SHRTIM_ERR_PARAM;
    }
    if (s_clk_hz == 0u) {
        return SHRTIM_ERR_STATE;
    }

    mult = (cfg->align == SHRTIM_ALIGN_CENTER) ? 2u : 1u;
    rc = solve_timing(cfg->freq_hz, mult, &ratio, &period, &ratio_idx);
    if (rc != SHRTIM_OK) {
        return rc;
    }
    if ((cfg->align == SHRTIM_ALIGN_CENTER) && (ratio_idx >= 7u)) {
        /* DIV4 is the slowest prescaler: no slower master ratio exists to
         * stretch one master period over the full 2*P PWM cycle. */
        return SHRTIM_ERR_FREQ;
    }

    rc = program_deadtime((unsigned)unit, cfg->dt_rise_ns, cfg->dt_fall_ns);
    if (rc != SHRTIM_OK) {
        return rc;
    }

    tmr = s_tmr_id[unit];
    out1 = s_out_id[2u * (unsigned)unit];
    out2 = s_out_id[2u * (unsigned)unit + 1u];

    /* Center alignment: program the shared master BEFORE the unit. The
     * JLink dump showed the unit counter ending up with the master's
     * prescaler value when the master was configured after it, so the
     * unit writes must come last. */
    if (cfg->align == SHRTIM_ALIGN_CENTER) {
        center_master_setup(ratio_idx, period);
    }

    SHRTIM_TIM_CounterDisable(SHRTIM1, tmr);
    /* Write the unit configuration with preload DISABLED: the JLink dump
     * proved the update-trigger field defaults to
     * SHRTIM_UPDATETRIG_NONE, so shadow registers are never transferred
     * in hardware and CKPSC/UPDOWNM/period must reach the active domain
     * directly. Preload is re-enabled afterwards for glitch-free runtime
     * updates. */
    SHRTIM_TIM_DisablePreload(SHRTIM1, tmr);
    SHRTIM_TIM_SetPrescaler(SHRTIM1, tmr, ratio);
    SHRTIM_TIM_SetPeriod(SHRTIM1, tmr, period);
    SHRTIM_TIM_SetCountingMode(SHRTIM1, tmr,
                               (cfg->align == SHRTIM_ALIGN_CENTER)
                                   ? SHRTIM_COUNTING_MODE_UP_DOWN
                                   : SHRTIM_COUNTING_MODE_UP);
    if (cfg->align == SHRTIM_ALIGN_CENTER) {
        /* Master-anchored center alignment: phase-lock this counter to the
         * master (the lock is what makes the duty deterministic). */
        SHRTIM_TIM_SetResetTrig(SHRTIM1, tmr, SHRTIM_RESETTRIG_MASTER_PER);
    } else {
        SHRTIM_TIM_SetResetTrig(SHRTIM1, tmr, SHRTIM_RESETTRIG_NONE);
    }

    switch (cfg->run) {
    case SHRTIM_RUN_ONESHOT:
        mode = SHRTIM_MODE_SINGLESHOT;
        break;
    case SHRTIM_RUN_RETRIG:
        mode = SHRTIM_MODE_RETRIGGERABLE;
        break;
    default:
        mode = SHRTIM_MODE_CONTINUOUS;
        break;
    }
    SHRTIM_TIM_SetCounterMode(SHRTIM1, tmr, mode);

    s_unit[unit].ratio = ratio;
    s_unit[unit].ratio_idx = ratio_idx;
    s_unit[unit].period = period;
    s_unit[unit].duty = cfg->duty_x1000;
    s_unit[unit].duty2 = cfg->duty2_x1000;
    s_unit[unit].align = (uint8_t)cfg->align;
    s_unit[unit].configured = 1u;

    apply_crossbars((unsigned)unit);

    /* Polarity, idle level; OUT_Config also clears the fault-state and
     * chopper bits of both output halves to their defaults. */
    SHRTIM_OUT_Config(SHRTIM1, out1,
                      (cfg->active_low1 ? SHRTIM_OUT_NEGATIVE_POLARITY
                                        : SHRTIM_OUT_POSITIVE_POLARITY) |
                          (cfg->idle1 == SHRTIM_LEVEL_ACTIVE
                               ? SHRTIM_OUT_IDLELEVEL_ACTIVE
                               : SHRTIM_OUT_IDLELEVEL_INACTIVE));
    SHRTIM_OUT_Config(SHRTIM1, out2,
                      (cfg->active_low2 ? SHRTIM_OUT_NEGATIVE_POLARITY
                                        : SHRTIM_OUT_POSITIVE_POLARITY) |
                          (cfg->idle2 == SHRTIM_LEVEL_ACTIVE
                               ? SHRTIM_OUT_IDLELEVEL_ACTIVE
                               : SHRTIM_OUT_IDLELEVEL_INACTIVE));

    /* Roll-over generated update: transfers the preload at every counter
     * reset/roll-over and raises the UPDATE event (fixes the permanently
     * clear UPDITF observed with the default NONE trigger).
     * @note LA/T11 observation: with UPDATETRIG_RESET the UPDATE interrupt
     *       fires 2-4x per PWM period on this silicon (both counter
     *       turnarounds / reset paths raise it), so UPDATE IRQ counts are
     *       a multiple of the PWM frequency - documented behaviour, not a
     *       bug in the counting path. */
    SHRTIM_TIM_SetUpdateTrig(SHRTIM1, tmr, SHRTIM_UPDATETRIG_RESET);
    SHRTIM_ForceUpdate(SHRTIM1, tmr);
    SHRTIM_TIM_EnablePreload(SHRTIM1, tmr);
    SHRTIM_ForceUpdate(SHRTIM1, tmr);

    /* Read back the fields the JLink session showed going stale and
     * rewrite them a bounded number of times if they did not stick. */
    {
        uint32_t want_cnt = (cfg->align == SHRTIM_ALIGN_CENTER)
                                ? SHRTIM_COUNTING_MODE_UP_DOWN
                                : SHRTIM_COUNTING_MODE_UP;
        unsigned retry;

        for (retry = 0u; retry < 3u; retry++) {
            if ((SHRTIM_TIM_GetPrescaler(SHRTIM1, tmr) == ratio)
                && (SHRTIM_TIM_GetCountingMode(SHRTIM1, tmr) == want_cnt)
                && (SHRTIM_TIM_GetPeriod(SHRTIM1, tmr) == period)) {
                break;
            }
            SHRTIM_TIM_DisablePreload(SHRTIM1, tmr);
            SHRTIM_TIM_SetPrescaler(SHRTIM1, tmr, ratio);
            SHRTIM_TIM_SetCountingMode(SHRTIM1, tmr, want_cnt);
            SHRTIM_TIM_SetPeriod(SHRTIM1, tmr, period);
            SHRTIM_TIM_SetUpdateTrig(SHRTIM1, tmr, SHRTIM_UPDATETRIG_RESET);
            SHRTIM_ForceUpdate(SHRTIM1, tmr);
            SHRTIM_TIM_EnablePreload(SHRTIM1, tmr);
            SHRTIM_ForceUpdate(SHRTIM1, tmr);
        }
    }

    if (cfg->enable_outputs) {
        SHRTIM_EnableOutput(SHRTIM1, out1 | out2);
    }
    return SHRTIM_OK;
}

int shrtim_pwm_freq(shrtim_unit_t unit, uint32_t hz)
{
    uint32_t ratio;
    uint32_t period;
    uint32_t mult;
    uint32_t tmr;
    uint8_t ratio_idx;
    int center;

    if ((unit == SHRTIM_UNIT_MASTER) || ((unsigned)unit >= SHRTIM_HAL_UNITS)) {
        return SHRTIM_ERR_PARAM;
    }
    if (s_clk_hz == 0u) {
        return SHRTIM_ERR_STATE;
    }
    if (s_unit[unit].configured == 0u) {
        return SHRTIM_ERR_STATE;
    }

    center = (s_unit[unit].align == (uint8_t)SHRTIM_ALIGN_CENTER);
    mult = center ? 2u : 1u;
    if (solve_timing(hz, mult, &ratio, &period, &ratio_idx) != SHRTIM_OK) {
        return SHRTIM_ERR_FREQ;
    }
    if (center && (ratio_idx >= 7u)) {
        return SHRTIM_ERR_FREQ; /* no slower master ratio (see pwm_init) */
    }

    tmr = s_tmr_id[unit];
    if (center) {
        /* Keep the shared master anchor in step with the new cycle time;
         * configured before the unit so its writes cannot land on the
         * unit registers (see shrtim_pwm_init()). */
        center_master_setup(ratio_idx, period);
    }
    SHRTIM_TIM_SetPrescaler(SHRTIM1, tmr, ratio);
    SHRTIM_TIM_SetPeriod(SHRTIM1, tmr, period);
    {
        unsigned retry;

        for (retry = 0u; retry < 3u; retry++) {
            if ((SHRTIM_TIM_GetPrescaler(SHRTIM1, tmr) == ratio)
                && (SHRTIM_TIM_GetPeriod(SHRTIM1, tmr) == period)) {
                break;
            }
            SHRTIM_TIM_DisablePreload(SHRTIM1, tmr);
            SHRTIM_TIM_SetPrescaler(SHRTIM1, tmr, ratio);
            SHRTIM_TIM_SetPeriod(SHRTIM1, tmr, period);
            SHRTIM_ForceUpdate(SHRTIM1, tmr);
            SHRTIM_TIM_EnablePreload(SHRTIM1, tmr);
            SHRTIM_ForceUpdate(SHRTIM1, tmr);
        }
    }
    s_unit[unit].ratio = ratio;
    s_unit[unit].ratio_idx = ratio_idx;
    s_unit[unit].period = period;
    apply_crossbars((unsigned)unit);
    SHRTIM_ForceUpdate(SHRTIM1, tmr);
    return SHRTIM_OK;
}

uint32_t shrtim_pwm_freq_get(shrtim_unit_t unit)
{
    uint32_t tmr;
    uint32_t ratio;
    uint32_t period;
    unsigned i;

    if ((unit == SHRTIM_UNIT_MASTER) || ((unsigned)unit >= SHRTIM_HAL_UNITS)) {
        return 0u;
    }
    if ((s_clk_hz == 0u) || (s_unit[unit].configured == 0u)) {
        return 0u;
    }

    tmr = s_tmr_id[unit];
    ratio = SHRTIM_TIM_GetPrescaler(SHRTIM1, tmr);
    period = SHRTIM_TIM_GetPeriod(SHRTIM1, tmr);
    if (period == 0u) {
        return 0u;
    }

    for (i = 0u; i < 8u; i++) {
        if (s_ratio_hw[i] == ratio) {
            uint32_t mult =
                (s_unit[unit].align == (uint8_t)SHRTIM_ALIGN_CENTER) ? 2u : 1u;
            uint64_t div = (uint64_t)mult * (uint64_t)period
                           * (uint64_t)s_ratio_den[i];

            /* freq = round(fSHRTIM * num / (mult * period * den)) */
            return (uint32_t)(((uint64_t)s_clk_hz * (uint64_t)s_ratio_num[i]
                               + div / 2u) / div);
        }
    }
    return 0u;
}

int shrtim_pwm_duty(shrtim_unit_t unit, uint32_t duty_x1000)
{
    if ((unit == SHRTIM_UNIT_MASTER) || ((unsigned)unit >= SHRTIM_HAL_UNITS)) {
        return SHRTIM_ERR_PARAM;
    }
    if (duty_x1000 > 1000u) {
        return SHRTIM_ERR_PARAM;
    }
    if ((s_clk_hz == 0u) || (s_unit[unit].configured == 0u)) {
        return SHRTIM_ERR_STATE;
    }

    s_unit[unit].duty = duty_x1000;
    apply_crossbars((unsigned)unit);
    SHRTIM_ForceUpdate(SHRTIM1, s_tmr_id[unit]);
    return SHRTIM_OK;
}

int shrtim_pwm_duty2(shrtim_unit_t unit, uint32_t duty_x1000)
{
    if ((unit == SHRTIM_UNIT_MASTER) || ((unsigned)unit >= SHRTIM_HAL_UNITS)) {
        return SHRTIM_ERR_PARAM;
    }
    if (duty_x1000 > 1000u) {
        return SHRTIM_ERR_PARAM;
    }
    if ((s_clk_hz == 0u) || (s_unit[unit].configured == 0u)) {
        return SHRTIM_ERR_STATE;
    }

    s_unit[unit].duty2 = duty_x1000;
    apply_crossbars((unsigned)unit);
    SHRTIM_ForceUpdate(SHRTIM1, s_tmr_id[unit]);
    return SHRTIM_OK;
}

int shrtim_pwm_deadtime(shrtim_unit_t unit, uint32_t rise_ns, uint32_t fall_ns)
{
    if ((unit == SHRTIM_UNIT_MASTER) || ((unsigned)unit >= SHRTIM_HAL_UNITS)) {
        return SHRTIM_ERR_PARAM;
    }
    if ((s_clk_hz == 0u) || (s_unit[unit].configured == 0u)) {
        return SHRTIM_ERR_STATE;
    }
    return program_deadtime((unsigned)unit, rise_ns, fall_ns);
}

void shrtim_pwm_start(shrtim_unit_t unit)
{
    uint32_t tmr;

    if ((unit == SHRTIM_UNIT_MASTER) || ((unsigned)unit >= SHRTIM_HAL_UNITS)) {
        return;
    }
    tmr = s_tmr_id[unit];
    SHRTIM_TIM_CounterEnable(SHRTIM1, tmr);
}

void shrtim_pwm_stop(shrtim_unit_t unit)
{
    if ((unit != SHRTIM_UNIT_MASTER) && ((unsigned)unit < SHRTIM_HAL_UNITS)) {
        SHRTIM_TIM_CounterDisable(SHRTIM1, s_tmr_id[unit]);
    }
}

void shrtim_pwm_start_all(void)
{
    shrtim_unit_t u;

    SHRTIM_TIM_CounterEnable(SHRTIM1, SHRTIM_TIMER_X);
    for (u = (shrtim_unit_t)0u; (unsigned)u < SHRTIM_HAL_UNITS; u++)
    {
        /* Same running-clock rewrite requirement as shrtim_pwm_start(). */
        shrtim_pwm_start(u);
    }
}

void shrtim_pwm_stop_all(void)
{
    SHRTIM_TIM_CounterDisable(SHRTIM1, SHRTIM_TIMER_X);
}

void shrtim_pwm_oneshot_arm(shrtim_unit_t unit)
{
    if ((unit != SHRTIM_UNIT_MASTER) && ((unsigned)unit < SHRTIM_HAL_UNITS)) {
        SHRTIM_CounterReset(SHRTIM1, s_tmr_id[unit]);
    }
}

/* ---------------------------------------------------------------------- */
/* Output level control                                                    */
/* ---------------------------------------------------------------------- */

void shrtim_out_enable(shrtim_out_t out)
{
    if ((unsigned)out < 12u) {
        SHRTIM_EnableOutput(SHRTIM1, s_out_id[out]);
    }
}

void shrtim_out_disable(shrtim_out_t out)
{
    if ((unsigned)out < 12u) {
        SHRTIM_DisableOutput(SHRTIM1, s_out_id[out]);
    }
}

void shrtim_out_force(shrtim_out_t out, shrtim_level_t level)
{
    if ((unsigned)out < 12u) {
        SHRTIM_OUT_ForceLevel(SHRTIM1, s_out_id[out],
                              (level == SHRTIM_LEVEL_ACTIVE)
                                  ? SHRTIM_OUT_LEVEL_ACTIVE
                                  : SHRTIM_OUT_LEVEL_INACTIVE);
    }
}

void shrtim_out_polarity(shrtim_out_t out, bool active_low)
{
    if ((unsigned)out < 12u) {
        SHRTIM_OUT_SetPolarity(SHRTIM1, s_out_id[out],
                               active_low ? SHRTIM_OUT_NEGATIVE_POLARITY
                                          : SHRTIM_OUT_POSITIVE_POLARITY);
    }
}

void shrtim_pwm_swap(shrtim_unit_t unit, bool enable)
{
    if ((unit != SHRTIM_UNIT_MASTER) && ((unsigned)unit < SHRTIM_HAL_UNITS)) {
        if (enable) {
            SHRTIM_EnableSwapOutputs(SHRTIM1, s_tmr_id[unit]);
        } else {
            SHRTIM_DisableSwapOutputs(SHRTIM1, s_tmr_id[unit]);
        }
        /* The swap bit lives in the common CTRL2 register and is applied
         * with the register update on this silicon: LA tests showed no
         * output exchange until an update was forced. */
        SHRTIM_ForceUpdate(SHRTIM1, s_tmr_id[unit]);
    }
}

void shrtim_pwm_pushpull(shrtim_unit_t unit, bool enable)
{
    if ((unit != SHRTIM_UNIT_MASTER) && ((unsigned)unit < SHRTIM_HAL_UNITS)) {
        if (enable) {
            SHRTIM_TIM_EnablePushPullMode(SHRTIM1, s_tmr_id[unit]);
        } else {
            SHRTIM_TIM_DisablePushPullMode(SHRTIM1, s_tmr_id[unit]);
        }
    }
}

int shrtim_pwm_chopper(shrtim_unit_t unit, const shrtim_chopper_cfg_t *cfg)
{
    uint32_t out1;
    uint32_t out2;

    if ((cfg == NULL) || (unit == SHRTIM_UNIT_MASTER) ||
        ((unsigned)unit >= SHRTIM_HAL_UNITS)) {
        return SHRTIM_ERR_PARAM;
    }
    if ((cfg->duty_eighths > 7u) || (cfg->prescaler_div > 15u)) {
        return SHRTIM_ERR_PARAM;
    }

    out1 = s_out_id[2u * (unsigned)unit];
    out2 = s_out_id[2u * (unsigned)unit + 1u];

    if (!cfg->enable) {
        SHRTIM_OUT_SetChopperMode(SHRTIM1, out1, SHRTIM_OUT_CHOPPERMODE_DISABLED);
        SHRTIM_OUT_SetChopperMode(SHRTIM1, out2, SHRTIM_OUT_CHOPPERMODE_DISABLED);
        return SHRTIM_OK;
    }

    SHRTIM_CHP_SetPrescaler(SHRTIM1, s_tmr_id[unit], s_chp_psc[cfg->prescaler_div]);
    SHRTIM_CHP_SetDutyCycle(SHRTIM1, s_tmr_id[unit], s_chp_duty[cfg->duty_eighths]);
    SHRTIM_CHP_SetPulseWidth(SHRTIM1, s_tmr_id[unit], SHRTIM_CHP_PULSEWIDTH_16);
    SHRTIM_OUT_SetChopperMode(SHRTIM1, out1, SHRTIM_OUT_CHOPPERMODE_ENABLED);
    SHRTIM_OUT_SetChopperMode(SHRTIM1, out2, SHRTIM_OUT_CHOPPERMODE_ENABLED);
    return SHRTIM_OK;
}

/* ---------------------------------------------------------------------- */
/* Raw counter / compare access                                            */
/* ---------------------------------------------------------------------- */

uint32_t shrtim_counter(shrtim_unit_t unit)
{
    if ((unsigned)unit <= (unsigned)SHRTIM_UNIT_MASTER) {
        return SHRTIM_TIM_GetCounter(SHRTIM1, timer_of(unit));
    }
    return 0u;
}

void shrtim_period_raw(shrtim_unit_t unit, uint32_t ticks)
{
    if (((unsigned)unit <= (unsigned)SHRTIM_UNIT_MASTER) &&
        (ticks >= 1u) && (ticks <= SHRTIM_HAL_PERIOD_MAX)) {
        SHRTIM_TIM_SetPeriod(SHRTIM1, timer_of(unit), ticks);
        if ((unit != SHRTIM_UNIT_MASTER) && (s_unit[unit].configured != 0u)) {
            s_unit[unit].period = ticks;
        }
    }
}

void shrtim_compare_raw(shrtim_unit_t unit, uint8_t cmp, uint32_t ticks)
{
    if (((unsigned)unit > (unsigned)SHRTIM_UNIT_MASTER) || (cmp < 1u) || (cmp > 5u)) {
        return;
    }
    if ((cmp == 5u) && (unit == SHRTIM_UNIT_MASTER)) {
        return; /* master only offers compare 1..4 */
    }
    if (ticks > SHRTIM_HAL_PERIOD_MAX) {
        return;
    }

    switch (cmp) {
    case 1u:
        SHRTIM_TIM_SetCompare1(SHRTIM1, timer_of(unit), ticks);
        break;
    case 2u:
        SHRTIM_TIM_SetCompare2(SHRTIM1, timer_of(unit), ticks);
        break;
    case 3u:
        SHRTIM_TIM_SetCompare3(SHRTIM1, timer_of(unit), ticks);
        break;
    case 4u:
        SHRTIM_TIM_SetCompare4(SHRTIM1, timer_of(unit), ticks);
        break;
    default:
        SHRTIM_TIM_SetCompare5(SHRTIM1, s_tmr_id[unit], ticks);
        break;
    }
}

uint32_t shrtim_compare_get(shrtim_unit_t unit, uint8_t cmp)
{
    if (((unsigned)unit > (unsigned)SHRTIM_UNIT_MASTER) || (cmp < 1u) || (cmp > 5u)) {
        return 0u;
    }
    if ((cmp == 5u) && (unit == SHRTIM_UNIT_MASTER)) {
        return 0u;
    }

    switch (cmp) {
    case 1u:
        return SHRTIM_TIM_GetCompare1(SHRTIM1, timer_of(unit));
    case 2u:
        return SHRTIM_TIM_GetCompare2(SHRTIM1, timer_of(unit));
    case 3u:
        return SHRTIM_TIM_GetCompare3(SHRTIM1, timer_of(unit));
    case 4u:
        return SHRTIM_TIM_GetCompare4(SHRTIM1, timer_of(unit));
    default:
        return SHRTIM_TIM_GetCompare5(SHRTIM1, s_tmr_id[unit]);
    }
}

/* ---------------------------------------------------------------------- */
/* Master timer, sync and ADC triggers                                     */
/* ---------------------------------------------------------------------- */

void shrtim_master_period(uint32_t period_ticks, bool start_now)
{
    if ((period_ticks >= 1u) && (period_ticks <= SHRTIM_HAL_PERIOD_MAX)) {
        SHRTIM_TIM_EnablePreload(SHRTIM1, SHRTIM_TIMER_MASTER);
        SHRTIM_TIM_SetPeriod(SHRTIM1, SHRTIM_TIMER_MASTER, period_ticks);
        SHRTIM_ForceUpdate(SHRTIM1, SHRTIM_TIMER_MASTER);
        if (start_now) {
            SHRTIM_TIM_CounterEnable(SHRTIM1, SHRTIM_TIMER_MASTER);
        }
    }
}

void shrtim_pwm_sync_to_master(shrtim_unit_t unit, bool enable)
{
    if ((unit != SHRTIM_UNIT_MASTER) && ((unsigned)unit < SHRTIM_HAL_UNITS)) {
        SHRTIM_TIM_SetResetTrig(SHRTIM1, s_tmr_id[unit],
                                enable ? SHRTIM_RESETTRIG_MASTER_PER
                                       : SHRTIM_RESETTRIG_NONE);
    }
}

int shrtim_adc_trigger(uint8_t trig, shrtim_unit_t unit, shrtim_evt_t evt)
{
    const uint32_t(*table)[6];
    unsigned evi;
    unsigned grp;
    uint32_t src_reg;
    uint32_t src;
    uint32_t update;

    if ((trig < 1u) || (trig > 10u) || ((unsigned)unit > (unsigned)SHRTIM_UNIT_MASTER)) {
        return SHRTIM_ERR_PARAM;
    }

    switch (evt) {
    case SHRTIM_EVT_UPDATE:
        evi = ADTG_SRC_U;
        break;
    case SHRTIM_EVT_CMP1:
        evi = ADTG_SRC_C1;
        break;
    case SHRTIM_EVT_CMP2:
        evi = ADTG_SRC_C2;
        break;
    case SHRTIM_EVT_CMP3:
        evi = ADTG_SRC_C3;
        break;
    case SHRTIM_EVT_CMP4:
        evi = ADTG_SRC_C4;
        break;
    case SHRTIM_EVT_RSTRO:
        evi = ADTG_SRC_R;
        break;
    default:
        return SHRTIM_ERR_PARAM;
    }

    switch (trig) {
    case 1u:
    case 3u:
        table = s_adtg13;
        grp = (unit == SHRTIM_UNIT_MASTER || (unsigned)unit <= 2u) ? 0u : 1u;
        src_reg = (grp == 0u)
                      ? ((trig == 1u) ? SHRTIM_ADCTRIG1_SOURCE_GROUP1
                                      : SHRTIM_ADCTRIG3_SOURCE_GROUP1)
                      : ((trig == 1u) ? SHRTIM_ADCTRIG1_SOURCE_GROUP2
                                      : SHRTIM_ADCTRIG3_SOURCE_GROUP2);
        break;
    case 2u:
    case 4u:
        table = s_adtg24;
        grp = (unit == SHRTIM_UNIT_MASTER || (unsigned)unit <= 2u) ? 0u : 1u;
        src_reg = (grp == 0u)
                      ? ((trig == 2u) ? SHRTIM_ADCTRIG2_SOURCE_GROUP1
                                      : SHRTIM_ADCTRIG4_SOURCE_GROUP1)
                      : ((trig == 2u) ? SHRTIM_ADCTRIG2_SOURCE_GROUP2
                                      : SHRTIM_ADCTRIG4_SOURCE_GROUP2);
        break;
    case 5u:
        table = s_adtg579;
        src_reg = SHRTIM_ADCTRIG5_SOURCE;
        break;
    case 7u:
        table = s_adtg579;
        src_reg = SHRTIM_ADCTRIG7_SOURCE;
        break;
    case 9u:
        table = s_adtg579;
        src_reg = SHRTIM_ADCTRIG9_SOURCE;
        break;
    case 6u:
        table = s_adtg6810;
        src_reg = SHRTIM_ADCTRIG6_SOURCE;
        break;
    case 8u:
        table = s_adtg6810;
        src_reg = SHRTIM_ADCTRIG8_SOURCE;
        break;
    default:
        table = s_adtg6810;
        src_reg = SHRTIM_ADCTRIG10_SOURCE;
        break;
    }

    src = table[(unsigned)unit][evi];
    if (src == 0u) {
        return SHRTIM_ERR_PARAM; /* source not available in this group */
    }

    update = (unit == SHRTIM_UNIT_MASTER) ? (uint32_t)SHRTIM_ADCTRIG_UPDATE_MASTER
                                          : (uint32_t)SHRTIM_ADCTRIG_UPDATE_TIMER_A +
                                                (uint32_t)unit;
    SHRTIM_ConfigADCTrig(SHRTIM1, src_reg, (uint32_t)(trig - 1u), update, src);
    return SHRTIM_OK;
}

/* ---------------------------------------------------------------------- */
/* External events                                                         */
/* ---------------------------------------------------------------------- */

int shrtim_exev_config(uint8_t ev, const shrtim_exev_cfg_t *cfg)
{
    uint32_t event;
    uint32_t src;
    uint32_t compsel;
    uint32_t pol;
    uint32_t sens;

    if ((cfg == NULL) || (ev < 1u) || (ev > 10u) ||
        (cfg->sensitivity > 3u) || (cfg->filter > 15u)) {
        return SHRTIM_ERR_PARAM;
    }
    if (!cfg->from_gpio && (cfg->chip_src > 2u)) {
        return SHRTIM_ERR_PARAM;
    }

    event = (uint32_t)1u << (ev - 1u); /* SHRTIM_EVENT_n bit value */

    if (cfg->from_gpio) {
        src = SHRTIM_EXEV_SOURCE_1;
        compsel = SHRTIM_EXEVXSRC2_COMP_NONE;
    } else if (cfg->chip_src == 0u) {
        src = SHRTIM_EXEV_SOURCE_2; /* comparator output */
        compsel = SHRTIM_EXEVXSRC2_COMP1_OUT;
    } else if (cfg->chip_src == 1u) {
        src = SHRTIM_EXEV_SOURCE_3; /* ATIMx TRGO */
        compsel = SHRTIM_EXEVXSRC2_COMP_NONE;
    } else {
        src = SHRTIM_EXEV_SOURCE_4; /* ADC AWD */
        compsel = SHRTIM_EXEVXSRC2_COMP_NONE;
    }

    pol = cfg->active_low ? SHRTIM_EXEV_POLARITY_LOW : SHRTIM_EXEV_POLARITY_HIGH;
    sens = s_exev_sens[cfg->sensitivity];

    SHRTIM_EXEV_Config(SHRTIM1, event, src, compsel, pol, sens,
                       cfg->fast_mode ? SHRTIM_EXEV_FASTMODE_ENABLE
                                      : SHRTIM_EXEV_FASTMODE_DISABLE,
                       s_exev_flt[cfg->filter], SHRTIM_EXEV_PRESCALER_DIV1);
    return SHRTIM_OK;
}

int shrtim_exev_reset_unit(shrtim_unit_t unit, uint8_t ev)
{
    uint32_t trig;

    if ((unit == SHRTIM_UNIT_MASTER) || ((unsigned)unit >= SHRTIM_HAL_UNITS) ||
        (ev > 10u)) {
        return SHRTIM_ERR_PARAM;
    }

    if (ev == 0u) {
        trig = SHRTIM_RESETTRIG_NONE;
    } else {
        trig = (uint32_t)SHRTIM_RESETTRIG_EXEV_1 << (ev - 1u);
    }
    SHRTIM_TIM_SetResetTrig(SHRTIM1, s_tmr_id[unit], trig);
    return SHRTIM_OK;
}

/* ---------------------------------------------------------------------- */
/* Faults                                                                  */
/* ---------------------------------------------------------------------- */

int shrtim_fault_config(uint8_t fault, const shrtim_fault_cfg_t *cfg)
{
    uint32_t flt;

    if ((cfg == NULL) || (fault < 1u) || (fault > 6u) || (cfg->filter > 15u)) {
        return SHRTIM_ERR_PARAM;
    }

    flt = (uint32_t)1u << (fault - 1u); /* SHRTIM_FAULT_n bit value */

    /* GPIO-sourced fault pin; polarity + source via Config (its mask covers
     * only POL/SRC0), filter via the dedicated setter. The silicon latches
     * the fault flag until it is cleared; the latch request is recorded for
     * shrtim_fault_reset() and reported, there is no latch-disable bit. */
    SHRTIM_FALT_Config(SHRTIM1, flt,
                       (cfg->active_high ? SHRTIM_FALT_POLARITY_HIGH
                                         : SHRTIM_FALT_POLARITY_LOW) |
                           SHRTIM_FALT_SRC_DIGITALINPUT,
                       SHRTIM_EXEVXSRC2_COMP_NONE);
    SHRTIM_FALT_SetFilter(SHRTIM1, flt, s_falt_flt[cfg->filter]);
    SHRTIM_FALT_Enable(SHRTIM1, flt);

    s_fault_latch[fault - 1u] = cfg->latch;
    return SHRTIM_OK;
}

int shrtim_fault_attach(shrtim_unit_t unit, uint8_t mask,
                        shrtim_fault_out_t out1, shrtim_fault_out_t out2)
{
    uint32_t bits = 0u;
    uint32_t fs1;
    uint32_t fs2;
    unsigned i;

    if ((unit == SHRTIM_UNIT_MASTER) || ((unsigned)unit >= SHRTIM_HAL_UNITS) ||
        (mask > 0x3Fu)) {
        return SHRTIM_ERR_PARAM;
    }

    for (i = 0u; i < 6u; i++) {
        if ((mask & (uint8_t)(1u << i)) != 0u) {
            bits |= (uint32_t)1u << i; /* SHRTIM_FAULT_(i+1) bit value */
        }
    }

    /* Replace the whole attachment: clear all, then enable the requested. */
    SHRTIM_TIM_DisableFault(SHRTIM1, s_tmr_id[unit],
                            SHRTIM_FAULT_1 | SHRTIM_FAULT_2 | SHRTIM_FAULT_3 |
                                SHRTIM_FAULT_4 | SHRTIM_FAULT_5 | SHRTIM_FAULT_6);
    if (bits != 0u) {
        SHRTIM_TIM_EnableFault(SHRTIM1, s_tmr_id[unit], bits);
    }

    fs1 = fault_state_encode(out1);
    fs2 = fault_state_encode(out2);
    SHRTIM_OUT_SetFaultState(SHRTIM1, s_out_id[2u * (unsigned)unit], fs1);
    SHRTIM_OUT_SetFaultState(SHRTIM1, s_out_id[2u * (unsigned)unit + 1u], fs2);
    return SHRTIM_OK;
}

void shrtim_fault_reset(uint8_t fault)
{
    if ((fault >= 1u) && (fault <= 6u)) {
        switch (fault) {
        case 1u:
            SHRTIM_ClearFlag_FALT1(SHRTIM1);
            break;
        case 2u:
            SHRTIM_ClearFlag_FALT2(SHRTIM1);
            break;
        case 3u:
            SHRTIM_ClearFlag_FALT3(SHRTIM1);
            break;
        case 4u:
            SHRTIM_ClearFlag_FALT4(SHRTIM1);
            break;
        case 5u:
            SHRTIM_ClearFlag_FALT5(SHRTIM1);
            break;
        default:
            SHRTIM_ClearFlag_FALT6(SHRTIM1);
            break;
        }
    }
}

/* ---------------------------------------------------------------------- */
/* Capture                                                                */
/* ---------------------------------------------------------------------- */

int shrtim_capture_config(shrtim_unit_t unit, uint8_t cpt, uint8_t ev)
{
    uint64_t trig;

    if ((unit == SHRTIM_UNIT_MASTER) || ((unsigned)unit >= SHRTIM_HAL_UNITS) ||
        (cpt < 1u) || (cpt > 2u) || (ev > 10u)) {
        return SHRTIM_ERR_PARAM;
    }

    if (ev == 0u) {
        trig = SHRTIM_CAPTURETRIG_SW;
    } else {
        trig = (uint64_t)SHRTIM_CAPTURETRIG_EXEV_1 << (ev - 1u);
    }
    SHRTIM_TIM_SetCaptureTrig(SHRTIM1, s_tmr_id[unit], (uint32_t)(cpt - 1u), trig);
    return SHRTIM_OK;
}

int shrtim_capture_read(shrtim_unit_t unit, uint32_t *cpt1, uint32_t *cpt2)
{
    uint32_t tmr;
    uint64_t saved1;
    uint64_t saved2;
    uint32_t f1 = 0u;
    uint32_t f2 = 0u;
    volatile uint32_t spin;

    if ((unit == SHRTIM_UNIT_MASTER) || ((unsigned)unit >= SHRTIM_HAL_UNITS)) {
        return SHRTIM_ERR_PARAM;
    }
    tmr = s_tmr_id[unit];

    saved1 = SHRTIM_TIM_GetCaptureTrig(SHRTIM1, tmr, SHRTIM_CAPTUREUNIT_1);
    saved2 = SHRTIM_TIM_GetCaptureTrig(SHRTIM1, tmr, SHRTIM_CAPTUREUNIT_2);

    /* Writing the SW trigger bit fires a software capture. */
    SHRTIM_TIM_SetCaptureTrig(SHRTIM1, tmr, SHRTIM_CAPTUREUNIT_1,
                              saved1 | SHRTIM_CAPTURETRIG_SW);
    SHRTIM_TIM_SetCaptureTrig(SHRTIM1, tmr, SHRTIM_CAPTUREUNIT_2,
                              saved2 | SHRTIM_CAPTURETRIG_SW);

    for (spin = 0u; spin < 10000u; spin++) {
        f1 = SHRTIM_IsActiveFlag_CPT1(SHRTIM1, tmr);
        f2 = SHRTIM_IsActiveFlag_CPT2(SHRTIM1, tmr);
        if ((f1 != 0u) && (f2 != 0u)) {
            break;
        }
    }

    /* Restore the user trigger selection (without the SW bit). */
    SHRTIM_TIM_SetCaptureTrig(SHRTIM1, tmr, SHRTIM_CAPTUREUNIT_1, saved1);
    SHRTIM_TIM_SetCaptureTrig(SHRTIM1, tmr, SHRTIM_CAPTUREUNIT_2, saved2);

    if ((f1 == 0u) || (f2 == 0u)) {
        return SHRTIM_ERR_STATE;
    }
    if (cpt1 != NULL) {
        *cpt1 = SHRTIM_TIM_GetCapture1(SHRTIM1, tmr);
    }
    if (cpt2 != NULL) {
        *cpt2 = SHRTIM_TIM_GetCapture2(SHRTIM1, tmr);
    }
    SHRTIM_ClearFlag_CPT1(SHRTIM1, tmr);
    SHRTIM_ClearFlag_CPT2(SHRTIM1, tmr);
    return SHRTIM_OK;
}

/* ---------------------------------------------------------------------- */
/* Interrupt plumbing                                                      */
/* ---------------------------------------------------------------------- */

int shrtim_irq_hook(shrtim_unit_t unit, shrtim_evt_t evt,
                    shrtim_cb_t cb, void *user)
{
    unsigned row;

    if (((unsigned)unit > (unsigned)SHRTIM_UNIT_MASTER) ||
        ((unsigned)evt >= SHRTIM_HAL_EVT_COUNT)) {
        return SHRTIM_ERR_PARAM;
    }

    /* Fault/system events are dispatched from the fault ISR with
     * SHRTIM_UNIT_MASTER regardless of the unit passed here. */
    if ((evt >= SHRTIM_EVT_FAULT1) && (evt <= SHRTIM_EVT_SYSFLT)) {
        row = SHRTIM_HAL_UNITS;
    } else {
        row = (unit == SHRTIM_UNIT_MASTER) ? SHRTIM_HAL_UNITS : (unsigned)unit;
    }

    s_cb[row][evt] = cb;
    s_cb_user[row][evt] = user;
    return SHRTIM_OK;
}

int shrtim_irq_enable(shrtim_unit_t unit, shrtim_evt_t evt, bool enable)
{
    IRQn_Type irq;

    if (((unsigned)unit > (unsigned)SHRTIM_UNIT_MASTER) ||
        ((unsigned)evt >= SHRTIM_HAL_EVT_COUNT)) {
        return SHRTIM_ERR_PARAM;
    }

    if ((evt >= SHRTIM_EVT_FAULT1) && (evt <= SHRTIM_EVT_SYSFLT)) {
        irq = SHRT1_FLT_IRQn;
    } else if (evt == SHRTIM_EVT_MST_SYNC) {
        irq = SHRT1_MST_IRQn;
    } else if (unit == SHRTIM_UNIT_MASTER) {
        if (evt == SHRTIM_EVT_CMP5) {
            return SHRTIM_ERR_PARAM;
        }
        irq = SHRT1_MST_IRQn;
    } else {
        irq = (IRQn_Type)(SHRT1_TIMA_IRQn + (int)unit);
    }

    if (it_set(unit, evt, enable) != SHRTIM_OK) {
        return SHRTIM_ERR_PARAM;
    }
    if (enable) {
        NVIC_EnableIRQ(irq);
    }
    return SHRTIM_OK;
}

void shrtim_isr_master(void)
{
    static uint16_t storm[7];
    static uint16_t degraded;

    isr_pass(&SHRTIM1->sMasterRegs.MIDEN, &SHRTIM1->sMasterRegs.MINTSTS,
             &SHRTIM1->sMasterRegs.MINTCLR, 0u, SHRTIM_UNIT_MASTER,
             storm, &degraded, 7u, s_m_evt, s_m_bit);
}

void shrtim_isr_timer(shrtim_unit_t unit)
{
    static uint16_t storm[SHRTIM_HAL_UNITS][9];
    static uint16_t degraded[SHRTIM_HAL_UNITS];

    if ((unit == SHRTIM_UNIT_MASTER) || ((unsigned)unit >= SHRTIM_HAL_UNITS)) {
        return;
    }
    isr_pass(&SHRTIM1->sMasterRegs.MIDEN, &SHRTIM1->sMasterRegs.MINTSTS,
             &SHRTIM1->sMasterRegs.MINTCLR, (unsigned)unit + 1u, unit,
             storm[unit], &degraded[unit], 9u, s_t_evt, s_t_bit);
}

void shrtim_isr_fault(void)
{
    unsigned f;

    for (f = 0u; f < 6u; f++) {
        uint32_t active = 0u;

        switch (f) {
        case 0u:
            active = SHRTIM_IsActiveFlag_FALT1(SHRTIM1);
            break;
        case 1u:
            active = SHRTIM_IsActiveFlag_FALT2(SHRTIM1);
            break;
        case 2u:
            active = SHRTIM_IsActiveFlag_FALT3(SHRTIM1);
            break;
        case 3u:
            active = SHRTIM_IsActiveFlag_FALT4(SHRTIM1);
            break;
        case 4u:
            active = SHRTIM_IsActiveFlag_FALT5(SHRTIM1);
            break;
        default:
            active = SHRTIM_IsActiveFlag_FALT6(SHRTIM1);
            break;
        }

        if (active != 0u) {
            switch (f) {
            case 0u:
                SHRTIM_ClearFlag_FALT1(SHRTIM1);
                break;
            case 1u:
                SHRTIM_ClearFlag_FALT2(SHRTIM1);
                break;
            case 2u:
                SHRTIM_ClearFlag_FALT3(SHRTIM1);
                break;
            case 3u:
                SHRTIM_ClearFlag_FALT4(SHRTIM1);
                break;
            case 4u:
                SHRTIM_ClearFlag_FALT5(SHRTIM1);
                break;
            default:
                SHRTIM_ClearFlag_FALT6(SHRTIM1);
                break;
            }
            dispatch(SHRTIM_UNIT_MASTER,
                     (shrtim_evt_t)((unsigned)SHRTIM_EVT_FAULT1 + f));
        }
    }

    if (SHRTIM_IsActiveFlag_SYSFALT(SHRTIM1) != 0u) {
        SHRTIM_ClearFlag_SYSFALT(SHRTIM1);
        dispatch(SHRTIM_UNIT_MASTER, SHRTIM_EVT_SYSFLT);
    }
}
