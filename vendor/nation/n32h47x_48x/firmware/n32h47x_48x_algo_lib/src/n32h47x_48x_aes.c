#include "n32h47x_48x_aes.h"
#include "n32h47x_48x_algo_common.h"

/* Hardware register definitions for AES accelerator */
#define AES_CR_REG        (*((volatile uint32_t *)0x4002A000))  // AES Control Register
#define AES_SR_REG        (*((volatile uint32_t *)0x4002A004))  // AES Status Register  
#define AES_DINR_REG      (*((volatile uint32_t *)0x4002A040))  // AES Data Input Register
#define AES_DOUTR_REG     (*((volatile uint32_t *)0x4002A044))  // AES Data Output Register
#define AES_KEYR0_REG     (*((volatile uint32_t *)0x4002A030))  // AES Key Register 0-3
#define AES_KEYR1_REG     (*((volatile uint32_t *)0x4002A034))  // AES Key Register 1-3
#define AES_KEYR2_REG     (*((volatile uint32_t *)0x4002A038))  // AES Key Register 2-3
#define AES_KEYR3_REG     (*((volatile uint32_t *)0x4002A03C))  // AES Key Register 3-3
#define AES_IVR0_REG      (*((volatile uint32_t *)0x4002A048))  // AES IV Register 0-3
#define AES_IVR1_REG      (*((volatile uint32_t *)0x4002A04C))  // AES IV Register 1-3
#define AES_IVR2_REG      (*((volatile uint32_t *)0x4002A050))  // AES IV Register 2-3
#define AES_IVR3_REG      (*((volatile uint32_t *)0x4002A054))  // AES IV Register 3-3

/* RCC register definitions */
#define RCC_AHB2ENR_REG   (*((volatile uint32_t *)0x4002103C))  // RCC AHB2 peripheral clock enable register

/* AES Control Register bit definitions */
#define AES_CR_EN         0x00000001U    // AES enable
#define AES_CR_START      0x00000002U    // AES start processing
#define AES_CR_DMAOUTEN   0x00000008U    // DMA output enable
#define AES_CR_DMAINEN    0x00000010U    // DMA input enable
#define AES_CR_CCFC       0x00000080U    // Computation Complete Flag Clear
#define AES_CR_ERRC       0x00000100U    // Error Clear
#define AES_CR_CCIE       0x00000200U    // Computation Complete Interrupt Enable
#define AES_CR_ERRIE      0x00000400U    // Error Interrupt Enable
#define AES_CR_DATATYPE   0x00000C00U    // Data type selection
#define AES_CR_MODE       0x00006000U    // AES mode selection
#define AES_CR_CHMOD      0x00070000U    // AES chaining mode
#define AES_CR_KEYSIZE    0x00180000U    // AES key size selection
#define AES_CR_KEYSEL     0x00600000U    // AES key selection
#define AES_CR_NPBLB      0x0F000000U    // Number of padding bytes in last block

/* AES Status Register bit definitions */
#define AES_SR_CCF        0x00000001U    // Computation Complete Flag
#define AES_SR_RDERR      0x00000002U    // Read Error Flag
#define AES_SR_WRERR      0x00000004U    // Write Error Flag
#define AES_SR_BUSY       0x00000008U    // Busy Flag
#define AES_SR_KEYVALID   0x00000010U    // Key Valid Flag

/* RCC AHB2ENR bit definitions */
#define RCC_AHB2ENR_AESEN 0x00008000U    // AES clock enable

/* External function declaration for buffer to FIFO transfer */
extern uint32_t BufToFIFO(uint32_t addr, uint32_t *data, uint32_t length);

/**
 * @brief Initialize AES hardware accelerator
 * @param[in] parm pointer to AES parameter structure
 * @return AES_Init_OK on success, error code on failure
 * @note This function initializes the AES hardware with specified key, mode, and parameters
 */
uint32_t AES_Init(AES_PARM *parm)
{
    uint32_t timeout_counter;
    uint32_t key_length;
    uint32_t mode;
    uint32_t encrypt_decrypt;
    uint32_t chaining_mode;
    
    /* Validate input parameters */
    if (!parm || !parm->key || !parm->out) {
        return AES_Crypto_ParaNull;
    }
    
    /* Extract parameters from structure */
    key_length      = parm->keyWordLen;
    mode            = parm->Mode;
    encrypt_decrypt = parm->En_De;
    chaining_mode   = parm->Mode;
    
    /* Enable AES clock */
    RCC_AHB2ENR_REG |= RCC_AHB2ENR_AESEN;
    
    /* Disable AES before configuration */
    AES_CR_REG = 0xD0U;  // Reset value with disabled state
    
    /* Clear ARAM and wait for completion */
    if (CLEAR_ARAM_DONE_CHECK() == Time_Out) {
        return AES_TimeOutError;
    }
    
    /* Configure key size based on key length */
    switch (key_length) {
        case 4:  // 128-bit key
            AES_CR_REG = (AES_CR_REG & ~AES_CR_KEYSIZE) | 0x0U;
            break;
        case 6:  // 192-bit key  
            AES_CR_REG = (AES_CR_REG & ~AES_CR_KEYSIZE) | 0x2U;
            break;
        case 8:  // 256-bit key
            AES_CR_REG = (AES_CR_REG & ~AES_CR_KEYSIZE) | 0x4U;
            break;
        default:
            return AES_Crypto_KeyLengthError;
    }
    
    /* Validate input buffer and mode parameters */
    if (!parm->in || (encrypt_decrypt == AES_ENC || encrypt_decrypt == AES_DEC) && !parm->iv) {
        return AES_Crypto_ParaNull;
    }
    
    /* Validate working mode */
    if (mode != AES_ECB && mode != AES_CBC && mode != AES_CTR) {
        return AES_Crypto_ModeError;
    }
    
    /* Validate encryption/decryption mode */
    if (encrypt_decrypt != AES_ENC && encrypt_decrypt != AES_DEC) {
        return AES_Crypto_EnOrDeError;
    }
    
    /* Load key into hardware */
    BufToFIFO((uint32_t)AES_KEYR0_REG, parm->key, key_length);
    
    /* Enable key loading */
    AES_CR_REG |= AES_CR_CCFC;
    
    /* Wait for key loading to complete */
    timeout_counter = 0;
    while ((AES_SR_REG & AES_SR_CCF) == 0) {
        if (++timeout_counter > Alg_TimeOut_Counter) {
            return AES_TimeOutError;
        }
    }
    
    /* Clear computation complete flag */
    AES_SR_REG |= AES_SR_CCF;
    
    /* Configure chaining mode */
    if (mode == AES_ECB || encrypt_decrypt == AES_DEC) {
        /* ECB mode or decryption mode */
        AES_CR_REG = (AES_CR_REG & ~AES_CR_CHMOD) | 0x0U;
    } else if (mode == AES_CBC && encrypt_decrypt != AES_DEC) {
        /* CBC encryption mode */
        AES_CR_REG = (AES_CR_REG & ~AES_CR_CHMOD) | 0x2U;
    }
    
    /* Configure encryption/decryption */
    if (mode == AES_ECB || encrypt_decrypt == AES_ENC) {
        /* ECB mode or encryption */
        AES_CR_REG &= ~AES_CR_MODE;
    } else if (mode == AES_CBC && encrypt_decrypt != AES_ENC) {
        /* CBC decryption */
        AES_CR_REG = (AES_CR_REG & ~AES_CR_MODE) | 0x4U;
    }
    
    /* Load IV for CBC or CTR mode */
    if (mode == AES_CBC || mode == AES_CTR) {
        if (parm->iv) {
            BufToFIFO((uint32_t)AES_IVR0_REG, parm->iv, 4);
        }
    }
    
    /* Special handling for CTR mode */
    if (mode == AES_CTR) {
        AES_CR_REG &= ~AES_CR_CHMOD;  // Clear chaining mode bits
        AES_CR_REG |= 0x4U;           // Set CTR mode
        if (parm->iv) {
            BufToFIFO((uint32_t)AES_IVR0_REG, parm->iv, 4);
        }
    }
    
    return AES_Init_OK;
}

/**
 * @brief Perform AES encryption/decryption operation
 * @param[in] parm pointer to AES parameter structure
 * @return AES_Crypto_OK on success, error code on failure
 * @note Processes input data in 128-bit blocks
 */
uint32_t AES_Crypto(AES_PARM *parm)
{
    uint32_t *input_ptr;
    uint32_t *output_ptr;
    uint32_t blocks_remaining;
    uint32_t remaining_bytes;
    uint32_t timeout_counter;
    uint32_t i;
    uint32_t j;
    uint32_t ctr_counter[4];
    uint32_t *iv_ptr;
    
    /* Validate input parameters */
    if (!parm || !parm->in || !parm->out) {
        return AES_Crypto_ParaNull;
    }
    
    /* Check input length */
    if (!parm->inWordLen || ((parm->inWordLen % 4) != 0 && parm->Mode != AES_CTR)) {
        return AES_Crypto_LengthError;
    }
    
    /* Check if AES is initialized and enabled */
    if ((RCC_AHB2ENR_REG & RCC_AHB2ENR_AESEN) != RCC_AHB2ENR_AESEN ||
        (AES_SR_REG & AES_SR_KEYVALID) != AES_SR_KEYVALID) {
        return AES_Crypto_UnInitError;
    }
    
    /* Get pointers and calculate block count */
    input_ptr        = parm->in;
    output_ptr       = parm->out;
    blocks_remaining = parm->inWordLen >> 2;   // Convert words to 128-bit blocks
    remaining_bytes  = parm->inWordLen & 0x3;  // Get remaining bytes
    
    /* Special handling for CTR mode - copy IV for counter manipulation */
    if (parm->Mode == AES_CTR && parm->iv) {
        iv_ptr = (uint32_t *)parm->iv;
        ctr_counter[0] = iv_ptr[0];
        ctr_counter[1] = iv_ptr[1]; 
        ctr_counter[2] = iv_ptr[2];
        ctr_counter[3] = iv_ptr[3];
    }
    
    /* Process full 128-bit blocks */
    while (blocks_remaining > 0) {
        
        /* For CTR mode, write counter to DIN register */
        if (parm->Mode == AES_CTR) {
            AES_DINR_REG = ctr_counter[3];
            
            /* Increment counter (big-endian) */
            for (i = 0; i < 16; i++) {
                if (((uint8_t*)&ctr_counter)[15 - i] != 0xFF) {
                    ((uint8_t*)&ctr_counter)[15 - i]++;
                    break;
                }
                ((uint8_t*)&ctr_counter)[15 - i] = 0;
            }
        } else {
            /* For ECB/CBC modes, write input block to DIN register */
            AES_DINR_REG = *input_ptr;
            AES_DINR_REG = input_ptr[1];
            AES_DINR_REG = input_ptr[2]; 
            AES_DINR_REG = input_ptr[3];
        }
        
        /* Start encryption/decryption */
        AES_CR_REG |= AES_CR_START;
        
        /* Wait for operation to complete */
        timeout_counter = 0;
        while ((AES_SR_REG & AES_SR_CCF) == 0) {
            if (++timeout_counter > Alg_TimeOut_Counter) {
                return AES_TimeOutError;
            }
        }
        
        /* Clear computation complete flag */
        AES_SR_REG |= AES_SR_CCF;

        /* Read output from DOUT register - read all 4 words sequentially */
        if (parm->Mode == AES_CTR) {
            /* CTR mode: XOR output with input plaintext */
            *output_ptr = AES_DOUTR_REG ^ *input_ptr;
            output_ptr[1] = AES_DOUTR_REG ^ input_ptr[1];
            output_ptr[2] = AES_DOUTR_REG ^ input_ptr[2];
            output_ptr[3] = AES_DOUTR_REG ^ input_ptr[3];
        } else {
            /* ECB/CBC modes: direct output */
            *output_ptr = AES_DOUTR_REG;
            output_ptr[1] = AES_DOUTR_REG;
            output_ptr[2] = AES_DOUTR_REG;
            output_ptr[3] = AES_DOUTR_REG;
        }
        
        /* Advance pointers */
        input_ptr  += 4;
        output_ptr += 4;
        blocks_remaining--;
    }
    
    /* Process remaining bytes for CTR mode */
    if (remaining_bytes > 0 && parm->Mode == AES_CTR) {
        /* Write counter to DIN register */
        AES_DINR_REG = ctr_counter[3];
        
        /* Start encryption */
        AES_CR_REG |= AES_CR_START;
        
        /* Wait for operation to complete */
        timeout_counter = 0;
        while ((AES_SR_REG & AES_SR_CCF) == 0) {
            if (++timeout_counter > Alg_TimeOut_Counter) {
                return AES_TimeOutError;
            }
        }
        
        /* Clear computation complete flag */
        AES_SR_REG |= AES_SR_CCF;
        
        /* Process remaining bytes with XOR */
        for (j = 0; j < remaining_bytes; j++) {
            ((uint8_t*)output_ptr)[j] = ((uint8_t*)AES_DOUTR_REG)[j] ^ ((uint8_t*)input_ptr)[j];
        }
    }
    
    return AES_Crypto_OK;
}

/**
 * @brief Close AES hardware accelerator
 * @return none
 * @note Disables AES clock and clears hardware state
 */
void AES_Close(void)
{
    /* Check if AES is enabled */
    if ((RCC_AHB2ENR_REG & RCC_AHB2ENR_AESEN) == RCC_AHB2ENR_AESEN) {
        /* Check if AES peripheral is ready */
        if ((AES_SR_REG & AES_SR_KEYVALID) == AES_SR_KEYVALID) {
            /* Disable AES and clear key valid flag */
            AES_CR_REG |= AES_CR_ERRC;     // Clear errors
            AES_CR_REG &= ~AES_CR_EN;      // Disable AES
            AES_SR_REG &= ~AES_SR_KEYVALID; // Clear key valid flag
            
            /* Clear ARAM */
            CLEAR_ARAM_DONE_CHECK();
            
            /* Disable AES clock */
            RCC_AHB2ENR_REG &= ~RCC_AHB2ENR_AESEN;
        }
    }
}

/**
 * @brief Get AES library version information
 * @param[out] type pointer to library type information
 * @param[out] customer pointer to customer ID information  
 * @param[out] date pointer to array containing date information (3 bytes)
 * @param[out] version pointer to version information
 * @return none
 * @note Returns fixed version information for this AES library implementation
 */
void AES_Version(uint8_t *type, uint8_t *customer, uint8_t date[3], uint8_t *version)
{
    if (type) {
        *type = 0x05;  // Library type indicator
    }
    
    if (customer) {
        *customer = 0x00;  // Standard version
    }
    
    if (date) {
        date[0] = 25;  // Year: 2025
        date[1] = 3;   // Month: March  
        date[2] = 19;  // Day: 19th
    }
    
    if (version) {
        *version = 0x10;  // Version 1.0
    }
}