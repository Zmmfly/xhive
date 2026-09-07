#ifndef TEST_TIMER_PWM_MOCK_H
#define TEST_TIMER_PWM_MOCK_H

#include <stdint.h>

uint32_t c2_test_mmio_read32(volatile const uint32_t *address);
void c2_test_mmio_write32(volatile uint32_t *address, uint32_t value);

#define C2_MMIO_READ32(address) c2_test_mmio_read32(address)
#define C2_MMIO_WRITE32(address, value) c2_test_mmio_write32((address), (value))

#endif /* TEST_TIMER_PWM_MOCK_H */
