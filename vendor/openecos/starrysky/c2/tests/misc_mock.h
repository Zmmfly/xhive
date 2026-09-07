#ifndef C2_TEST_MISC_MOCK_H
#define C2_TEST_MISC_MOCK_H
#include <stdint.h>
uint32_t c2_test_misc_read(volatile const uint32_t *address);
void c2_test_misc_write(volatile uint32_t *address, uint32_t value);
#define C2_MMIO_READ32(address) c2_test_misc_read(address)
#define C2_MMIO_WRITE32(address, value) c2_test_misc_write(address, value)
#endif
