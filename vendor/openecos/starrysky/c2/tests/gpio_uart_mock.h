#ifndef C2_GPIO_UART_MOCK_H
#define C2_GPIO_UART_MOCK_H

#include <stdint.h>

uint32_t c2_gpio_uart_mock_read32(volatile const uint32_t *address);
void c2_gpio_uart_mock_write32(volatile uint32_t *address, uint32_t value);

#define C2_MMIO_READ32(address) \
    c2_gpio_uart_mock_read32((volatile const uint32_t *)(address))
#define C2_MMIO_WRITE32(address, value) \
    c2_gpio_uart_mock_write32((volatile uint32_t *)(address), (uint32_t)(value))

#endif /* C2_GPIO_UART_MOCK_H */
