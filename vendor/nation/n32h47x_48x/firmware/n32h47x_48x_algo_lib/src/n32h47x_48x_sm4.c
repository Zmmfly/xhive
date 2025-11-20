#include "n32h47x_48x_sm4.h"
#include "n32h47x_48x_algo_common.h"

#include <stddef.h>

// Hardware register base addresses
#define SM4_CRYP_BASE          0x4002A000
#define SM4_RCC_AHB2ENR_BASE   0x40021000

// Hardware register offsets
#define SM4_CR                  0x00    // Control register
#define SM4_SR                  0x04    // Status register  
#define SM4_DIN                 0x08    // Data input register
#define SM4_DOUT                0x10    // Data output register
#define SM4_KEY0_7              0x20    // Key registers 0-7
#define SM4_IV0_3               0x40    // Initialization vector registers 0-3
#define SM4_CSGCMCCM0R          0x50    // Context swap GCM/GMAC or CCM registers
#define SM4_CSGCM0R             0x60    // Context swap GCM registers
#define SM4_CR_ALGO_OFFSET      0x30    // Algorithm selection offset
#define SM4_CR_DATATYPE_OFFSET  0x34    // Data type selection offset
#define SM4_CR_KEYSIZE_OFFSET   0x38    // Key size selection offset

// RCC AHB2 peripheral clock enable register offset
#define RCC_AHB2ENR_OFFSET      0x3C

// Control register bit definitions
#define SM4_CR_ALGO_MASK        0x80000080  // Algorithm selection mask
#define SM4_CR_ALGO_SM4         0x80000000  // SM4 algorithm selection
#define SM4_CR_DATATYPE_MASK    0x00000C00  // Data type mask
#define SM4_CR_DATATYPE_NONE    0x00000000  // No data type swap
#define SM4_CR_KEYSIZE_MASK     0x00000080  // Key size mask

// Status register bit definitions
#define SM4_SR_BUSY             0x00000010  // Busy flag
#define SM4_SR_OFFE             0x00000020  // Output FIFO empty flag

// RCC clock enable bit
#define RCC_AHB2ENR_CRYPEN      0x00008000  // Crypto module clock enable

// SM4 algorithm configuration constants
#define SM4_CFG_VALUE           0xD4        // Configuration register value
#define SM4_SM4_ALGO_VAL        0x7E        // SM4 algorithm register value
#define SM4_CONTROL_INIT        0x40        // Initial control register value

// SM4 FK constants (System parameters for key expansion)
const uint32_t SM4_FK[4] = {
    0xC6BAB1A3,  // FK[0]
    0x5033AA56,  // FK[1]
    0x97917D67,  // FK[2]
    0xDC2270B2   // FK[3]
};

// SM4 CK constants (Round constants)
const uint32_t SM4_CK[32] = {
    0x15121619,  // CK[0]
    0x31323334,  // CK[1]
    0x4D4E4F50,  // CK[2]
    0x696A6B6C,  // CK[3]
    0x85868788,  // CK[4]
    0xA1A2A3A4,  // CK[5]
    0xBDBEBFC0,  // CK[6]
    0xD9DADBDC,  // CK[7]
    0x11121314,  // CK[8]
    0x2D2E2F30,  // CK[9]
    0x494A4B4C,  // CK[10]
    0x65666768,  // CK[11]
    0x81828384,  // CK[12]
    0x9D9E9FA0,  // CK[13]
    0xB9BABBBC,  // CK[14]
    0xD5D6D7D8,  // CK[15]
    0x0D0E0F10,  // CK[16]
    0x292A2B2C,  // CK[17]
    0x45464748,  // CK[18]
    0x61626364,  // CK[19]
    0x7D7E7F80,  // CK[20]
    0x999A9B9C,  // CK[21]
    0xB5B6B7B8,  // CK[22]
    0xD1D2D3D4,  // CK[23]
    0x0A0B0C0D,  // CK[24]
    0x26272829,  // CK[25]
    0x42434445,  // CK[26]
    0x5E5F6061,  // CK[27]
    0x7A7B7C7D,  // CK[28]
    0x96979899,  // CK[29]
    0xB2B3B4B5,  // CK[30]
    0xCECFD0D1   // CK[31]
};

/**
 * @brief Buffer to FIFO transfer function
 * @param[in] fifo_address FIFO address offset
 * @param[in] buffer Pointer to data buffer
 * @param[in] word_count Number of words to transfer
 * @note Internal function used to write data to hardware FIFO
 */
static void BufferToFIFO_Write(uint32_t fifo_offset, const uint32_t* buffer, uint32_t word_count)
{
    volatile uint32_t* fifo_reg = (volatile uint32_t*)(SM4_CRYP_BASE + fifo_offset);
    
    for (uint32_t i = 0; i < word_count; i++) {
        *fifo_reg = buffer[i];
    }
}

/**
 * @brief Hardware register access helper functions
 */
static inline void WriteHardwareRegister(uint32_t offset, uint32_t value)
{
    *((volatile uint32_t*)(SM4_CRYP_BASE + offset)) = value;
}

static inline uint32_t ReadHardwareRegister(uint32_t offset)
{
    return *((volatile uint32_t*)(SM4_CRYP_BASE + offset));
}

static inline void WriteRCCRegister(uint32_t offset, uint32_t value)
{
    *((volatile uint32_t*)(SM4_RCC_AHB2ENR_BASE + offset)) = value;
}

static inline uint32_t ReadRCCRegister(uint32_t offset)
{
    return *((volatile uint32_t*)(SM4_RCC_AHB2ENR_BASE + offset));
}

/**
 * @brief Enable SM4 hardware clock
 * @note Enable RCC AHB2 clock for crypto module
 */
static void EnableSM4HardwareClock(void)
{
    uint32_t rcc_value = ReadRCCRegister(RCC_AHB2ENR_OFFSET);
    WriteRCCRegister(RCC_AHB2ENR_OFFSET, rcc_value | RCC_AHB2ENR_CRYPEN);
}

/**
 * @brief Disable SM4 hardware clock  
 * @note Disable RCC AHB2 clock for crypto module
 */
static void DisableSM4HardwareClock(void)
{
    uint32_t rcc_value = ReadRCCRegister(RCC_AHB2ENR_OFFSET);
    WriteRCCRegister(RCC_AHB2ENR_OFFSET, rcc_value & ~RCC_AHB2ENR_CRYPEN);
}

/**
 * @brief Check if SM4 hardware is properly initialized
 * @return 1 if ready, 0 if not ready
 */
static uint32_t IsSM4HardwareReady(void)
{
    uint32_t rcc_clock_enabled = (ReadRCCRegister(RCC_AHB2ENR_OFFSET) & RCC_AHB2ENR_CRYPEN) == RCC_AHB2ENR_CRYPEN;
    uint32_t crypto_config = (ReadHardwareRegister(SM4_CR) & 0x14) == 0x14;
    return rcc_clock_enabled && crypto_config;
}

/**
 * @brief Wait for SM4 hardware operation completion
 * @param[in] timeout_counter Timeout counter value
 * @return Time_Out if timeout, SM4_Crypto_OK if success
 */
static uint32_t WaitForSM4OperationComplete(uint32_t timeout_counter)
{
    uint32_t timeout = 0;
    
    while ((ReadHardwareRegister(SM4_SR) & SM4_SR_OFFE) == 0) {
        timeout++;
        if (timeout > timeout_counter) {
            return SM4_TimeOutError;
        }
    }
    
    // Clear the output FIFO empty flag
    uint32_t status = ReadHardwareRegister(SM4_SR);
    WriteHardwareRegister(SM4_SR, status | SM4_SR_OFFE);
    
    return SM4_Crypto_OK;
}

/**
 * @brief Initialize SM4 algorithm with parameters
 * @param[in] param Pointer to SM4 parameter structure
 * @return SM4_Init_OK if success, other error codes if fail
 */
uint32_t SM4_Init(SM4_PARM* param)
{
    uint32_t  timeout_counter = 0;
    uint32_t* iv_ptr          = param->iv;
    
    // Enable hardware clock
    EnableSM4HardwareClock();
    
    // Configure crypto module
    WriteHardwareRegister(SM4_CR, SM4_CFG_VALUE);
    
    // Wait for ARAM clear operation to complete
    if (CLEAR_ARAM_DONE_CHECK() == Time_Out) {
        return SM4_TimeOutError;
    }
    
    // Validate working mode (ECB or CBC)
    if (param->workingMode != SM4_ECB && param->workingMode != SM4_CBC) {
        return SM4_ModeErr;
    }
    
    // Validate encryption/decryption mode
    if (param->EnDeMode != SM4_ENC && param->EnDeMode != SM4_DEC) {
        return SM4_EnDeErr;
    }
    
    // Validate key pointer
    if (param->key == NULL) {
        return SM4_ADRNULL;
    }
    
    // For CBC mode, validate IV pointer
    if (param->workingMode == SM4_CBC && iv_ptr == NULL) {
        return SM4_ADRNULL;
    }
    
    // Load encryption key to hardware FIFO
    BufferToFIFO_Write(0x38, param->key, 4);
    
    // Load SM4 CK constants to hardware FIFO
    BufferToFIFO_Write(0x3C, SM4_CK, 32);
    
    // Load SM4 FK constants to hardware FIFO
    BufferToFIFO_Write(0x34, SM4_FK, 4);
    
    // Configure SM4 algorithm
    WriteHardwareRegister(SM4_CR_ALGO_OFFSET, SM4_SM4_ALGO_VAL);
    WriteHardwareRegister(SM4_CR, SM4_CONTROL_INIT);
    
    // Start key expansion process
    uint32_t control = ReadHardwareRegister(SM4_CR);
    WriteHardwareRegister(SM4_CR, control | 0x80);
    
    // Wait for key expansion to complete
    while ((ReadHardwareRegister(SM4_SR) & SM4_SR_OFFE) == 0) {
        timeout_counter++;
        if (timeout_counter > Alg_TimeOut_Counter) {
            return SM4_TimeOutError;
        }
    }
    
    // Clear the output FIFO empty flag
    uint32_t status = ReadHardwareRegister(SM4_SR);
    WriteHardwareRegister(SM4_SR, status | SM4_SR_OFFE);
    
    // Configure encryption/decryption mode
    if (param->EnDeMode == SM4_ENC) {
        uint32_t control_reg = ReadHardwareRegister(SM4_CR);
        WriteHardwareRegister(SM4_CR, control_reg & 0xFFFFFFFE);
    } else if (param->EnDeMode == SM4_DEC) {
        uint32_t control_reg = ReadHardwareRegister(SM4_CR);
        WriteHardwareRegister(SM4_CR, (control_reg & 0xFFFFFFFE) | 0x01);
    }
    
    // Configure working mode
    if (param->workingMode == SM4_ECB) {
        uint32_t control_reg = ReadHardwareRegister(SM4_CR);
        WriteHardwareRegister(SM4_CR, control_reg & 0xFFFFFFFD);
    } else if (param->workingMode == SM4_CBC) {
        uint32_t control_reg = ReadHardwareRegister(SM4_CR);
        WriteHardwareRegister(SM4_CR, (control_reg & 0xFFFFFFFD) | 0x02);
        
        // Load IV for CBC mode
        BufferToFIFO_Write(0x30, (const uint32_t*)iv_ptr, 4);
    }
    
    return SM4_Init_OK;
}

/**
 * @brief Perform SM4 encryption/decryption
 * @param[in] param Pointer to SM4 parameter structure
 * @return SM4_Crypto_OK if success, other error codes if fail
 */
uint32_t SM4_Crypto(SM4_PARM* param)
{
    uint32_t timeout_counter = 0;
    uint32_t* input_ptr = param->in;
    uint32_t* output_ptr = param->out;
    uint32_t remaining_words = param->inWordLen;
    
    // Validate pointers
    if (output_ptr == NULL || input_ptr == NULL) {
        return SM4_ADRNULL;
    }
    
    // Validate input length
    if (remaining_words == 0 || (remaining_words & 0x03) != 0) {
        return SM4_LengthErr;
    }
    
    // Check if hardware is properly initialized
    if (!IsSM4HardwareReady()) {
        return SM4_UnInitError;
    }
    
    // Process data in blocks of 4 words (16 bytes)
    while (remaining_words > 0) {
        // Write 4 words to input FIFO
        WriteHardwareRegister(SM4_DIN, input_ptr[0]);
        WriteHardwareRegister(SM4_DIN, input_ptr[1]);
        WriteHardwareRegister(SM4_DIN, input_ptr[2]);
        WriteHardwareRegister(SM4_DIN, input_ptr[3]);
        
        // Start encryption/decryption process
        uint32_t control = ReadHardwareRegister(SM4_CR);
        WriteHardwareRegister(SM4_CR, (control & 0x7F) | 0x80);
        
        // Wait for operation to complete
        uint32_t result = WaitForSM4OperationComplete(timeout_counter);
        if (result != SM4_Crypto_OK) {
            return result;
        }
        
        // Read 4 words from output FIFO
        output_ptr[0] = ReadHardwareRegister(SM4_DOUT);
        output_ptr[1] = ReadHardwareRegister(SM4_DOUT);
        output_ptr[2] = ReadHardwareRegister(SM4_DOUT);
        output_ptr[3] = ReadHardwareRegister(SM4_DOUT);
        
        // Move to next block
        input_ptr += 4;
        output_ptr += 4;
        remaining_words -= 4;
    }
    
    return SM4_Crypto_OK;
}

/**
 * @brief Close SM4 algorithm and disable hardware
 * @note Clean up and disable SM4 hardware module
 */
void SM4_Close(void)
{
    uint32_t rcc_clock_enabled = (ReadRCCRegister(RCC_AHB2ENR_OFFSET) & RCC_AHB2ENR_CRYPEN) == RCC_AHB2ENR_CRYPEN;
    
    if (rcc_clock_enabled) {
        uint32_t crypto_config = ReadHardwareRegister(SM4_CR);
        if ((crypto_config & 0x14) == 0x14) {
            // Disable crypto operations
            WriteHardwareRegister(SM4_CR, crypto_config | 0x100);
            WriteHardwareRegister(SM4_CR, crypto_config & 0xFFFFFFEB);
            
            // Wait for ARAM clear to complete
            CLEAR_ARAM_DONE_CHECK();
            
            // Disable hardware clock
            DisableSM4HardwareClock();
        }
    }
}

/**
 * @brief Get SM4 library version information
 * @param[out] type Library type information
 * @param[out] customer Customer information
 * @param[out] date Library date (3 bytes: day, month, year)
 * @param[out] version Library version information
 */
void SM4_Version(uint8_t* type, uint8_t* customer, uint8_t date[3], uint8_t* version)
{
    *type = 0x01;        // Commercial version
    *customer = 0x00;    // Standard version
    
    date[0] = 0x19;      // Day: 25
    date[1] = 0x03;      // Month: 3
    date[2] = 0x13;      // Year: 19 (2019)
    
    *version = 0x10;     // Version 1.0
}