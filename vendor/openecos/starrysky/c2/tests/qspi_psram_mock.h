#ifndef TESTS_QSPI_PSRAM_MOCK_H
#define TESTS_QSPI_PSRAM_MOCK_H

#include <stdint.h>

uint32_t qspi_psram_mock_read32(const volatile uint32_t *address);
void qspi_psram_mock_write32(volatile uint32_t *address, uint32_t value);

#define C2_MMIO_READ32(address) qspi_psram_mock_read32((address))
#define C2_MMIO_WRITE32(address, value) \
    qspi_psram_mock_write32((address), (uint32_t)(value))

#endif /* TESTS_QSPI_PSRAM_MOCK_H */
