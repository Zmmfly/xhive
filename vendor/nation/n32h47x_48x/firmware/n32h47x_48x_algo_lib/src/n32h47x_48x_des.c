#include "n32h47x_48x_des.h"
#include "n32h47x_48x_algo_common.h"

#include <stddef.h>

// Hardware register base addresses
#define DES_CRYP_BASE          0x4002A000
#define DES_RCC_AHB2ENR_BASE   0x40021000

// Hardware register offsets
#define DES_CR                 0x00    // Control register
#define DES_SR                 0x04    // Status register  
#define DES_DIN                0x08    // Data input register
#define DES_DOUT               0x10    // Data output register
#define DES_KEY0_7             0x20    // Key registers 0-7
#define DES_IV0_3              0x40    // Initialization vector registers 0-3
#define DES_KEY0_1_OFFSET      0x30    // First key registers offset
#define DES_KEY2_3_OFFSET      0x34    // Second key registers offset
#define DES_KEY4_5_OFFSET      0x38    // Third key registers offset
#define DES_KEY6_7_OFFSET      0x3C    // Fourth key registers offset

// RCC AHB2 peripheral clock enable register offset
#define RCC_AHB2ENR_OFFSET      0x3C

// Control register bit definitions
#define DES_CR_ALGO_MASK       0x00000010  // Algorithm selection mask
#define DES_CR_ALGO_DES        0x00000000  // DES algorithm selection
#define DES_CR_ALGO_TDES       0x00000010  // TDES algorithm selection
#define DES_CR_DATATYPE_MASK   0x00000C00  // Data type mask
#define DES_CR_DATATYPE_NONE   0x00000000  // No data type swap
#define DES_CR_KEYSIZE_MASK    0x00000018  // Key size mask for TDES
#define DES_CR_KEYSIZE_DES     0x00000000  // DES key size
#define DES_CR_KEYSIZE_2KEY    0x00000008  // TDES 2-key size
#define DES_CR_KEYSIZE_3KEY    0x00000018  // TDES 3-key size
#define DES_CR_DECRYPTION      0x00000040  // Decryption mode bit
#define DES_CR_CHAIN_CBC       0x00000002  // CBC chaining mode

// Status register bit definitions
#define DES_SR_BUSY             0x00000010  // Busy flag
#define DES_SR_OFNE            0x00000004  // Output FIFO not empty flag

// RCC clock enable bit
#define RCC_AHB2ENR_CRYPEN      0x00008000  // Crypto module clock enable

// DES algorithm configuration constants
#define DES_CFG_VALUE           0xD1        // Configuration register value
#define DES_CONTROL_INIT        0x00        // Initial control register value

// DES error code constants from reverse engineering
#define DES_Crypto_ParaNull_rev       0x5A5A5A5C  // Parameter null error
#define DES_Crypto_LengthError_rev    0x5A5A5A5D  // Length error
#define DES_Crypto_UnInitError_rev    0x5A5A5A5F  // Uninitialized error
#define DES_TimeOutError_rev          0x5A5A5A60  // Timeout error
#define DES_Crypto_ModeError_rev      0x5A5A5A5A  // Mode error
#define DES_Crypto_EnOrDeError_rev    0x5A5A5A5B  // Encryption/decryption error
#define DES_Crypto_KeyError_rev       0x5A5A5A5E  // Key error

// Hardware timeout counter

/**
 * @brief Buffer to FIFO transfer function
 * @param[in] fifo_address FIFO address offset
 * @param[in] buffer Pointer to data buffer
 * @param[in] word_count Number of words to transfer
 * @note Internal function used to write data to hardware FIFO
 */
static void BufferToFIFO_Write(uint32_t fifo_offset, const uint32_t* buffer, uint32_t word_count)
{
    volatile uint32_t* fifo_reg = (volatile uint32_t*)(DES_CRYP_BASE + fifo_offset);
    
    for (uint32_t i = 0; i < word_count; i++) {
        *fifo_reg = buffer[i];
    }
}

/**
 * @brief Hardware register access helper functions
 */
static inline void WriteHardwareRegister(uint32_t offset, uint32_t value)
{
    *((volatile uint32_t*)(DES_CRYP_BASE + offset)) = value;
}

static inline uint32_t ReadHardwareRegister(uint32_t offset)
{
    return *((volatile uint32_t*)(DES_CRYP_BASE + offset));
}

static inline void WriteRCCRegister(uint32_t offset, uint32_t value)
{
    *((volatile uint32_t*)(DES_RCC_AHB2ENR_BASE + offset)) = value;
}

static inline uint32_t ReadRCCRegister(uint32_t offset)
{
    return *((volatile uint32_t*)(DES_RCC_AHB2ENR_BASE + offset));
}

/**
 * @brief Enable DES hardware clock
 * @note Enable RCC AHB2 clock for crypto module
 */
static void EnableDESHardwareClock(void)
{
    uint32_t rcc_value = ReadRCCRegister(RCC_AHB2ENR_OFFSET);
    WriteRCCRegister(RCC_AHB2ENR_OFFSET, rcc_value | RCC_AHB2ENR_CRYPEN);
}

/**
 * @brief Disable DES hardware clock  
 * @note Disable RCC AHB2 clock for crypto module
 */
static void DisableDESHardwareClock(void)
{
    uint32_t rcc_value = ReadRCCRegister(RCC_AHB2ENR_OFFSET);
    WriteRCCRegister(RCC_AHB2ENR_OFFSET, rcc_value & ~RCC_AHB2ENR_CRYPEN);
}

/**
 * @brief Check if DES hardware is properly initialized
 * @return 1 if ready, 0 if not ready
 */
static uint32_t IsDESHardwareReady(void)
{
    uint32_t rcc_clock_enabled = (ReadRCCRegister(RCC_AHB2ENR_OFFSET) & RCC_AHB2ENR_CRYPEN) == RCC_AHB2ENR_CRYPEN;
    uint32_t crypto_config = (ReadHardwareRegister(DES_CR) & 0x11) == 0x11;
    return rcc_clock_enabled && crypto_config;
}

/**
 * @brief Wait for DES hardware operation completion
 * @param[in] timeout_counter Timeout counter value
 * @return DES_TimeOutError if timeout, DES_Crypto_OK if success
 */
static uint32_t WaitForDESOperationComplete(uint32_t timeout_counter)
{
    uint32_t timeout = 0;
    
    while ((ReadHardwareRegister(DES_SR) & DES_SR_OFNE) == 0) {
        timeout++;
        if (timeout > timeout_counter) {
            return DES_TimeOutError;
        }
    }
    
    // Clear the output FIFO not empty flag
    uint32_t status = ReadHardwareRegister(DES_SR);
    WriteHardwareRegister(DES_SR, status | DES_SR_OFNE);
    
    return DES_Crypto_OK;
}

/**
 * @brief Initialize DES/TDES algorithm with parameters
 * @param[in] param Pointer to DES parameter structure
 * @return DES_Init_OK if success, other error codes if fail
 */
uint32_t DES_Init(DES_PARM* param)
{
    // Enable hardware clock
    EnableDESHardwareClock();
    
    // Configure crypto module for DES/TDES
    WriteHardwareRegister(DES_CR, DES_CFG_VALUE);
    
    // Wait for ARAM clear operation to complete
    if (CLEAR_ARAM_DONE_CHECK() == Time_Out) {
        return DES_TimeOutError;
    }
    
    // Initialize control register
    WriteHardwareRegister(DES_CR, DES_CONTROL_INIT);
    
    // Validate key pointer
    if (param->key == NULL) {
        return DES_Crypto_ParaNull_rev;
    }
    
    // For CBC mode, validate IV pointer
    if (param->Mode == DES_CBC && param->iv == NULL) {
        return DES_Crypto_ParaNull_rev;
    }
    
    // Validate working mode (ECB or CBC)
    if (param->Mode != DES_ECB && param->Mode != DES_CBC) {
        return DES_Crypto_ModeError_rev;
    }
    
    // Validate encryption/decryption mode
    if (param->En_De != DES_ENC && param->En_De != DES_DEC) {
        return DES_Crypto_EnOrDeError_rev;
    }
    
    // Configure key based on key mode
    switch (param->keyMode) {
        case DES_KEY:    // Single DES
            BufferToFIFO_Write(DES_KEY0_1_OFFSET, param->key, 2);
            break;
            
        case TDES_2KEY:  // Triple DES with 2 keys
            BufferToFIFO_Write(DES_KEY0_1_OFFSET, param->key, 2);
            BufferToFIFO_Write(DES_KEY2_3_OFFSET, param->key + 2, 2);
            WriteHardwareRegister(DES_CR, ReadHardwareRegister(DES_CR) | 0x20);
            break;
            
        case TDES_3KEY:  // Triple DES with 3 keys
            BufferToFIFO_Write(DES_KEY0_1_OFFSET, param->key, 2);
            BufferToFIFO_Write(DES_KEY2_3_OFFSET, param->key + 2, 2);
            BufferToFIFO_Write(DES_KEY4_5_OFFSET, param->key + 4, 2);
            WriteHardwareRegister(DES_CR, ReadHardwareRegister(DES_CR) | 0x30);
            break;
            
        default:
            return DES_Crypto_KeyError_rev;
    }
    
    // Configure decryption mode if needed
    if (param->En_De == DES_DEC) {
        WriteHardwareRegister(DES_CR, ReadHardwareRegister(DES_CR) | DES_CR_DECRYPTION);
    }
    
    // Configure CBC mode if needed
    if (param->Mode == DES_CBC) {
        WriteHardwareRegister(DES_CR, ReadHardwareRegister(DES_CR) | DES_CR_CHAIN_CBC);
        BufferToFIFO_Write(DES_KEY6_7_OFFSET, param->iv, 2);
    }
    
    return DES_Init_OK;
}

/**
 * @brief Perform DES/TDES encryption/decryption
 * @param[in] param Pointer to DES parameter structure
 * @return DES_Crypto_OK if success, other error codes if fail
 */
uint32_t DES_Crypto(DES_PARM* param)
{
    uint32_t timeout_counter = 0;
    uint32_t* input_ptr = param->in;
    uint32_t* output_ptr = param->out;
    uint32_t remaining_words = param->inWordLen;
    
    // Validate pointers
    if (input_ptr == NULL || output_ptr == NULL) {
        return DES_Crypto_ParaNull_rev;
    }
    
    // Validate input length (must be multiple of 2 for DES)
    if (remaining_words == 0 || (remaining_words & 0x01) != 0) {
        return DES_Crypto_LengthError_rev;
    }
    
    // Check if hardware is properly initialized
    if (!IsDESHardwareReady()) {
        return DES_Crypto_UnInitError_rev;
    }
    
    // Process data in blocks of 2 words (8 bytes for DES)
    while (remaining_words > 0) {
        // Write 2 words to input FIFO
        WriteHardwareRegister(DES_DIN, input_ptr[0]);
        WriteHardwareRegister(DES_DIN, input_ptr[1]);
        
        // Start encryption/decryption process
        WriteHardwareRegister(DES_CR, ReadHardwareRegister(DES_CR) | 0x80);
        
        // Wait for operation to complete
        uint32_t result = WaitForDESOperationComplete(timeout_counter);
        if (result != DES_Crypto_OK) {
            return result;
        }
        
        // Read 2 words from output FIFO
        output_ptr[0] = ReadHardwareRegister(DES_DOUT);
        output_ptr[1] = ReadHardwareRegister(DES_DOUT);
        
        // Move to next block
        input_ptr += 2;
        output_ptr += 2;
        remaining_words -= 2;
    }
    
    return DES_Crypto_OK;
}

/**
 * @brief Close DES/TDES algorithm and disable hardware
 * @note Clean up and disable DES hardware module
 */
void DES_Close(void)
{
    uint32_t rcc_clock_enabled = (ReadRCCRegister(RCC_AHB2ENR_OFFSET) & RCC_AHB2ENR_CRYPEN) == RCC_AHB2ENR_CRYPEN;
    
    if (rcc_clock_enabled) {
        uint32_t crypto_config = ReadHardwareRegister(DES_CR);
        if ((crypto_config & 0x11) == 0x11) {
            // Disable crypto operations
            WriteHardwareRegister(DES_CR, crypto_config | 0x100);
            WriteHardwareRegister(DES_CR, crypto_config & 0xFFFFFFEE);
            
            // Wait for ARAM clear to complete
            CLEAR_ARAM_DONE_CHECK();
            
            // Disable hardware clock
            DisableDESHardwareClock();
        }
    }
}

/**
 * @brief Get DES/TDES library version information
 * @param[out] type Library type information
 * @param[out] customer Customer information
 * @param[out] date Library date (3 bytes: day, month, year)
 * @param[out] version Library version information
 */
void DES_Version(uint8_t* type, uint8_t* customer, uint8_t date[3], uint8_t* version)
{
    *type = 0x05;        // Security version
    *customer = 0x00;    // Standard version
    
    date[0] = 0x19;      // Day: 25
    date[1] = 0x04;      // Month: 4
    date[2] = 0x12;      // Year: 18 (2018)
    
    *version = 0x11;     // Version 1.1
}