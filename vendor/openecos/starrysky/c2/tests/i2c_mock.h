#ifndef TESTS_I2C_MOCK_H
#define TESTS_I2C_MOCK_H

#include <stdint.h>

uint32_t i2c_mock_read32(const volatile uint32_t *address);
void i2c_mock_write32(volatile uint32_t *address, uint32_t value);

#define C2_MMIO_READ32(address) i2c_mock_read32(address)
#define C2_MMIO_WRITE32(address, value) i2c_mock_write32((address), (value))

#endif /* TESTS_I2C_MOCK_H */
