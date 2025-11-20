#include <stdint.h>
#include <string.h>
#include "n32h47x_48x_rng.h"
#include "n32h47x_48x_algo_common.h"

// External references from common library
extern uint32_t Cpy_U32(uint32_t *dst, const uint32_t *src, uint32_t wordLen);
extern uint32_t SetZero_U32(uint32_t *dst, uint32_t wordLen);

// Global variables for RNG state
static uint32_t seedbuf[2]      = {0};         // Seed buffer for pseudorandom generation
static uint32_t upseedcount     = 0;           // Counter for seed updates
static uint32_t counterInner    = 0;           // Inner counter for true random generation
static const uint32_t dword_444 = 0x12345678;  // Magic constant for seeding

// Hardware register definitions for N32H474/75 cryptographic accelerator
#define RCC_BASE_ADDR              0x40021000U
#define RCC_APB2ENR               (*(volatile uint32_t *)(RCC_BASE_ADDR + 0x18))
#define RCC_AHBENR                (*(volatile uint32_t *)(RCC_BASE_ADDR + 0x14))

#define CRYPTO_BASE_ADDR           0x4002A000U
#define CRYPTO_CR                 (*(volatile uint32_t *)(CRYPTO_BASE_ADDR + 0x00))
#define CRYPTO_SR                 (*(volatile uint32_t *)(CRYPTO_BASE_ADDR + 0x0C))
#define CRYPTO_DIN                (*(volatile uint32_t *)(CRYPTO_BASE_ADDR + 0x10))
#define CRYPTO_DOUT               (*(volatile uint32_t *)(CRYPTO_BASE_ADDR + 0x14))

// Control register bits
#define CRYPTO_CR_ALGOMODE_TRNG   0x0016U  // True Random Number Generator mode
#define CRYPTO_CR_ALGOMODE_PRNG   0x0001U  // Pseudorandom Number Generator mode
#define CRYPTO_CR_FLUSH           0x0800U  // Flush FIFO
#define CRYPTO_CR_START           0x0080U  // Start operation

// Status register bits
#define CRYPTO_SR_BUSY            0x0080U  // Operation busy flag
#define CRYPTO_SR_DRDY            0x0001U  // Data ready flag

// RCC enable bits
#define RCC_APB2ENR_CRYPEN        0x0010U  // Crypto peripheral enable
#define RCC_AHBENR_RNGEN          0x8000U  // RNG hardware enable

// RCC enable bits (alternative mapping)
#define RCC_AHBENR_CRYPTOEN       0x0004U  // Alternative crypto enable

/**
 * @brief Get pseudorandom number using hardware cryptographic accelerator
 * @param[out] rand Pointer to buffer to store random numbers
 * @param[in] wordLen Length of random numbers in 32-bit words
 * @param[in] seed Pointer to seed array [2 words], can be NULL for automatic seeding
 * @return RNG_OK: success; other error codes for failure
 * @note Uses hardware PRNG mode of the cryptographic accelerator
 * @discovery Address 0x10: Complex hardware interaction with multiple registers
 * @hardware Access to RCC and Crypto registers for hardware initialization and control
 */
uint32_t GetPseudoRand_U32(uint32_t *rand, uint32_t wordLen, uint32_t seed[2])
{
    uint32_t i, j;
    uint32_t timeout_counter;
    uint32_t saved_cr, saved_sr;

    // Input validation
    if (rand == NULL)
        return ADDRNULL;

    if (wordLen == 0)
        return LENError;

    // Hardware initialization - enable crypto peripherals
    RCC_APB2ENR |= RCC_APB2ENR_CRYPEN;   // Enable crypto peripheral clock
    RCC_AHBENR  |= RCC_AHBENR_CRYPTOEN;  // Enable AHB crypto interface
    RCC_AHBENR  |= RCC_AHBENR_RNGEN;     // Enable RNG hardware

    // Save current crypto module state
    saved_cr = CRYPTO_CR;
    CRYPTO_CR = CRYPTO_CR_ALGOMODE_PRNG;  // Set to PRNG mode

    saved_sr = CRYPTO_SR;
    CRYPTO_SR = 0;  // Clear status register

    // Seed management
    if (upseedcount == 0)
    {
        if (seed == NULL)
        {
            goto use_hardware_seed;
        }
    }
    else if (seed == NULL)
    {
        // Generate seed from true random number generator
        uint32_t result = GetTrueRand_U32(seedbuf, 2);
        if (result != RNG_OK)
            return result;
        goto use_hardware_seed;
    }
    else
    {
        // Copy provided seed
        Cpy_U32(seedbuf, seed, 2);
    }

use_hardware_seed:
    // Set the seed in crypto module
    CRYPTO_SR |= CRYPTO_SR_DRDY;
    CRYPTO_DIN = dword_444;  // Write seed/initialization constant

    // Update seed counter (wraps at 1000)
    if ((uint16_t)(++upseedcount) > 1000)
        upseedcount = 0;

    // Generate requested number of random words
    for (i = 0; i < wordLen; i++)
    {
        // Start random number generation
        CRYPTO_SR |= CRYPTO_SR_BUSY;

        // Wait for operation completion with timeout
        timeout_counter = 0;
        while ((CRYPTO_SR & CRYPTO_SR_BUSY) != 0)
        {
            if (++timeout_counter > Alg_TimeOut_Counter)
                return RNG_TimeOutError;
        }

        // Read generated random number
        CRYPTO_CR |= CRYPTO_CR_START;
        rand[i] = CRYPTO_DOUT;
    }

    // Update seed buffer with entropy from generated random numbers
    for (j = 0; j < 2; j++)
    {
        // Generate additional random data for seed update
        CRYPTO_SR |= CRYPTO_SR_BUSY;

        timeout_counter = 0;
        while ((CRYPTO_SR & CRYPTO_SR_BUSY) != 0)
        {
            if (++timeout_counter > Alg_TimeOut_Counter)
                return RNG_TimeOutError;
        }

        CRYPTO_CR |= CRYPTO_CR_START;
        seedbuf[j] ^= CRYPTO_DOUT;  // Update seed with XOR
    }

    // Restore crypto module state
    CRYPTO_SR = saved_sr;
    CRYPTO_CR = saved_cr;

    return RNG_OK;
}

/**
 * @brief Get true random number using hardware random number generator
 * @param[out] rand Pointer to buffer to store random numbers
 * @param[in] wordLen Length of random numbers in 32-bit words
 * @return RNG_OK: success; other error codes for failure
 * @note Uses hardware TRNG mode with entropy validation
 * @discovery Address 0x190: Complex entropy collection and validation logic
 * @hardware Multi-stage entropy collection with health checks
 */
uint32_t GetTrueRand_U32(uint32_t *rand, uint32_t wordLen)
{
    uint32_t i, j, k, m, n;
    uint32_t timeout_counter;
    uint32_t entropy_pool[16];  // 64 bytes of entropy collection
    uint32_t saved_cr, saved_sr;

    // Initialize entropy collection buffer
    memset(entropy_pool, 0, sizeof(entropy_pool));

    // Input validation
    if (rand == NULL)
        return ADDRNULL;

    if (wordLen == 0)
        return LENError;

    // Hardware initialization
    RCC_APB2ENR |= RCC_APB2ENR_CRYPEN;   // Enable crypto peripheral
    RCC_AHBENR  |= RCC_AHBENR_CRYPTOEN;  // Enable AHB crypto interface
    RCC_AHBENR  |= RCC_AHBENR_RNGEN;     // Enable RNG hardware

    // Configure crypto module for TRNG mode
    saved_cr = CRYPTO_CR;
    CRYPTO_CR = CRYPTO_CR_ALGOMODE_TRNG;  // Set to TRNG mode

    saved_sr = CRYPTO_SR;
    CRYPTO_SR = 386;  // Special TRNG initialization value

    // Flush and initialize the hardware
    CRYPTO_CR |= CRYPTO_CR_FLUSH;

    // Wait for hardware initialization
    timeout_counter = 0;
    while ((CRYPTO_SR & CRYPTO_SR_BUSY) != 0)
    {
        if (++timeout_counter > Alg_TimeOut_Counter)
            return RNG_TimeOutError;
    }

    CRYPTO_CR |= CRYPTO_CR_START;

    // One-time entropy collection and validation
    if (counterInner == 0)
    {
        // Collect initial entropy (16 words)
        for (i = 0; i < 16; i++)
        {
            CRYPTO_SR |= CRYPTO_SR_BUSY;

            timeout_counter = 0;
            while ((CRYPTO_SR & CRYPTO_SR_BUSY) != 0)
            {
                if (++timeout_counter > Alg_TimeOut_Counter)
                    return RNG_TimeOutError;
            }

            CRYPTO_CR |= CRYPTO_CR_START;
            entropy_pool[i] = CRYPTO_DOUT;
        }

        // Entropy validation - ensure non-zero entropy
        if (entropy_pool[15] == 0)
        {
            // Collect more entropy if validation fails
            SetZero_U32(entropy_pool, 8);

            for (j = 0; j < 8; j++)
            {
                CRYPTO_SR |= CRYPTO_SR_BUSY;

                timeout_counter = 0;
                while ((CRYPTO_SR & CRYPTO_SR_BUSY) != 0)
                {
                    if (++timeout_counter > Alg_TimeOut_Counter)
                        return RNG_TimeOutError;
                }

                CRYPTO_CR |= CRYPTO_CR_START;
                entropy_pool[j] = CRYPTO_DOUT;
            }

            // Second level validation
            if (entropy_pool[7] == 0)
            {
                SetZero_U32(entropy_pool, 4);

                for (k = 0; k < 4; k++)
                {
                    CRYPTO_SR |= CRYPTO_SR_BUSY;

                    timeout_counter = 0;
                    while ((CRYPTO_SR & CRYPTO_SR_BUSY) != 0)
                    {
                        if (++timeout_counter > Alg_TimeOut_Counter)
                            return RNG_TimeOutError;
                    }

                    CRYPTO_CR |= CRYPTO_CR_START;
                    entropy_pool[k] = CRYPTO_DOUT;
                }

                // Third level validation
                if (entropy_pool[3] == 0)
                {
                    SetZero_U32(entropy_pool, 2);

                    for (m = 0; m < 2; m++)
                    {
                        CRYPTO_SR |= CRYPTO_SR_BUSY;

                        timeout_counter = 0;
                        while ((CRYPTO_SR & CRYPTO_SR_BUSY) != 0)
                        {
                            if (++timeout_counter > Alg_TimeOut_Counter)
                                return RNG_TimeOutError;
                        }

                        CRYPTO_CR |= CRYPTO_CR_START;
                        entropy_pool[m] = CRYPTO_DOUT;
                    }

                    // Final validation
                    if (entropy_pool[1] == 0)
                        return RNGATTACKED;  // Hardware RNG appears to be compromised
                }
            }
        }

        counterInner = 1;  // Mark entropy collection as complete
    }

    // Generate requested random numbers
    for (n = 0; n < wordLen; n++)
    {
        CRYPTO_SR |= CRYPTO_SR_BUSY;

        timeout_counter = 0;
        while ((CRYPTO_SR & CRYPTO_SR_BUSY) != 0)
        {
            if (++timeout_counter > Alg_TimeOut_Counter)
                return RNG_TimeOutError;
        }

        CRYPTO_CR |= CRYPTO_CR_START;
        rand[n] = CRYPTO_DOUT;
    }

    // Restore crypto module state
    CRYPTO_SR = saved_sr;
    CRYPTO_CR = saved_cr;

    return RNG_OK;
}

/**
 * @brief Get RNG library version information
 * @param[out] type Pointer to library type information
 * @param[out] customer Pointer to customer ID information
 * @param[out] date Pointer to array [3] containing date information
 * @param[out] version Pointer to version information
 * @return Pointer to type parameter (for chaining)
 * @note Fixed version information embedded in the library
 * @discovery Address 0x41C: Simple static version information function
 * @version Type=1 (Commercial), Customer=0 (Standard), Date=2025-03-19, Version=1.6
 */
void RNG_Version(uint8_t *type, uint8_t *customer, uint8_t date[3], uint8_t *version)
{
    if (type != NULL)
        *type = 1;      // Commercial version

    if (customer != NULL)
        *customer = 0;  // Standard version

    if (date != NULL)
    {
        date[0] = 25;   // Year 2025 (25 + 2000)
        date[1] = 3;    // March
        date[2] = 19;   // Day 19
    }

    if (version != NULL)
        *version = 16;  // Version 1.6
}