#include "c2_gpio.h"
#include "c2_uart.h"

#include <stdio.h>
#include <string.h>

#define MOCK_MAX_EVENTS 256u

typedef struct {
    volatile const uint32_t *address;
    uint32_t value;
} mock_read_t;

typedef struct {
    volatile uint32_t *address;
    uint32_t value;
} mock_write_t;

static mock_read_t mock_reads[MOCK_MAX_EVENTS];
static mock_write_t mock_writes[MOCK_MAX_EVENTS];
static size_t mock_read_count;
static size_t mock_read_index;
static size_t mock_write_count;
static unsigned mock_failures;
static volatile const uint32_t *mock_gpio_dr;
static volatile const uint32_t *mock_gpio_ddr;
static volatile const uint32_t *mock_sys_data;
static volatile const uint32_t *mock_hp_lsr;
static unsigned mock_gpio_dr_reads;
static unsigned mock_gpio_ddr_reads;
static unsigned mock_sys_data_reads;
static unsigned mock_hp_lsr_reads;
static unsigned test_failures;

static void mock_reset(void)
{
    mock_read_count = 0u;
    mock_read_index = 0u;
    mock_write_count = 0u;
    mock_failures = 0u;
    mock_gpio_dr_reads = 0u;
    mock_gpio_ddr_reads = 0u;
    mock_sys_data_reads = 0u;
    mock_hp_lsr_reads = 0u;
}

static void mock_push_read(volatile const uint32_t *address, uint32_t value)
{
    if (mock_read_count >= MOCK_MAX_EVENTS) {
        ++mock_failures;
        return;
    }
    mock_reads[mock_read_count].address = address;
    mock_reads[mock_read_count].value = value;
    ++mock_read_count;
}

uint32_t c2_gpio_uart_mock_read32(volatile const uint32_t *address)
{
    uint32_t value;

    if (address == mock_gpio_dr) {
        ++mock_gpio_dr_reads;
    }
    if (address == mock_gpio_ddr) {
        ++mock_gpio_ddr_reads;
    }
    if (address == mock_sys_data) {
        ++mock_sys_data_reads;
    }
    if (address == mock_hp_lsr) {
        ++mock_hp_lsr_reads;
    }

    if (mock_read_index < mock_read_count) {
        if (mock_reads[mock_read_index].address != address) {
            ++mock_failures;
        }
        value = mock_reads[mock_read_index].value;
        ++mock_read_index;
        return value;
    }
    return *address;
}

void c2_gpio_uart_mock_write32(volatile uint32_t *address, uint32_t value)
{
    if (mock_write_count >= MOCK_MAX_EVENTS) {
        ++mock_failures;
        return;
    }
    mock_writes[mock_write_count].address = address;
    mock_writes[mock_write_count].value = value;
    ++mock_write_count;
    *address = value;
}

#define CHECK_TRUE(expression) do { \
    if (!(expression)) { \
        fprintf(stderr, "%s:%d: check failed: %s\n", \
                __FILE__, __LINE__, #expression); \
        ++test_failures; \
    } \
} while (0)

#define CHECK_EQ_U32(expected, actual) do { \
    uint32_t check_expected_ = (uint32_t)(expected); \
    uint32_t check_actual_ = (uint32_t)(actual); \
    if (check_expected_ != check_actual_) { \
        fprintf(stderr, "%s:%d: expected 0x%08lx, got 0x%08lx\n", \
                __FILE__, __LINE__, (unsigned long)check_expected_, \
                (unsigned long)check_actual_); \
        ++test_failures; \
    } \
} while (0)

#define CHECK_EQ_SIZE(expected, actual) do { \
    size_t check_expected_ = (size_t)(expected); \
    size_t check_actual_ = (size_t)(actual); \
    if (check_expected_ != check_actual_) { \
        fprintf(stderr, "%s:%d: expected %lu, got %lu\n", \
                __FILE__, __LINE__, (unsigned long)check_expected_, \
                (unsigned long)check_actual_); \
        ++test_failures; \
    } \
} while (0)

static void check_write(size_t index,
                        volatile uint32_t *address,
                        uint32_t value)
{
    CHECK_TRUE(index < mock_write_count);
    if (index < mock_write_count) {
        CHECK_TRUE(mock_writes[index].address == address);
        CHECK_EQ_U32(value, mock_writes[index].value);
    }
}

static c2_hp_uart_config_t hp_default_config(void)
{
    c2_hp_uart_config_t config;

    config.baud_rate = UINT32_C(115200);
    config.irq_enable_mask = 0u;
    config.data_bits = 8u;
    config.parity = C2_UART_PARITY_NONE;
    config.stop_bits = C2_UART_STOP_BITS_1;
    config.rx_trigger = C2_HP_UART_RX_TRIGGER_14_BYTES;
    return config;
}

static void test_gpio_parameters_and_shadow(void)
{
    C2_GPIO_TypeDef regs = {0};
    c2_gpio_t gpio = {0};
    uint32_t input_mask = 0u;
    uint32_t output_bits = 0u;
    uint32_t sampled = 0u;

    mock_gpio_dr = &regs.DR.WORD;
    mock_gpio_ddr = &regs.DDR.WORD;
    mock_reset();

    CHECK_EQ_U32(C2_ERROR_INVALID_ARGUMENT,
                 c2_gpio_init(NULL, &regs, 0u, 0u));
    CHECK_EQ_U32(C2_ERROR_INVALID_ARGUMENT,
                 c2_gpio_init(&gpio, NULL, 0u, 0u));
    CHECK_EQ_U32(C2_ERROR_INVALID_ARGUMENT,
                 c2_gpio_init(&gpio, &regs, UINT32_C(0x10000), 0u));
    CHECK_EQ_U32(C2_ERROR_NOT_INITIALIZED,
                 c2_gpio_set_mask(&gpio, 1u));
    CHECK_EQ_SIZE(0u, mock_write_count);

    CHECK_EQ_U32(C2_OK,
                 c2_gpio_init(&gpio, &regs, UINT32_C(0x8001),
                              UINT32_C(0x8000)));
    CHECK_EQ_SIZE(3u, mock_write_count);
    check_write(0u, &regs.DDR.WORD, UINT32_C(0xffff));
    check_write(1u, &regs.DR.WORD, UINT32_C(0x8000));
    check_write(2u, &regs.DDR.WORD, UINT32_C(0x7ffe));
    CHECK_EQ_U32(0u, mock_gpio_dr_reads);
    CHECK_EQ_U32(0u, mock_gpio_ddr_reads);

    CHECK_EQ_U32(C2_OK, c2_gpio_set_mask(&gpio, 0u));
    CHECK_EQ_U32(C2_OK,
                 c2_gpio_set_direction_mask(&gpio, 0u,
                                            C2_GPIO_DIRECTION_INPUT));
    CHECK_EQ_U32(C2_OK, c2_gpio_read_inputs(&gpio, 0u, &sampled));
    CHECK_EQ_U32(0u, sampled);
    CHECK_EQ_SIZE(3u, mock_write_count);
    CHECK_EQ_U32(0u, mock_gpio_dr_reads);

    CHECK_EQ_U32(C2_OK, c2_gpio_clear_mask(&gpio, UINT32_C(0x8000)));
    CHECK_EQ_U32(C2_OK, c2_gpio_set_mask(&gpio, UINT32_C(0x0001)));
    CHECK_EQ_U32(C2_OK, c2_gpio_toggle_mask(&gpio, UINT32_C(0x8000)));
    CHECK_EQ_U32(C2_OK,
                 c2_gpio_set_direction_mask(&gpio, UINT32_C(0x0001),
                                            C2_GPIO_DIRECTION_INPUT));
    CHECK_EQ_U32(0u, mock_gpio_dr_reads);
    CHECK_EQ_U32(0u, mock_gpio_ddr_reads);
    check_write(3u, &regs.DR.WORD, 0u);
    check_write(4u, &regs.DR.WORD, 1u);
    check_write(5u, &regs.DR.WORD, UINT32_C(0x8001));
    check_write(6u, &regs.DDR.WORD, UINT32_C(0x7fff));

    CHECK_EQ_U32(C2_OK,
                 c2_gpio_get_shadow(&gpio, &input_mask, &output_bits));
    CHECK_EQ_U32(UINT32_C(0x7fff), input_mask);
    CHECK_EQ_U32(UINT32_C(0x8001), output_bits);
    CHECK_EQ_U32(0u, mock_gpio_dr_reads);

    CHECK_EQ_U32(C2_ERROR_INVALID_ARGUMENT,
                 c2_gpio_read_inputs(&gpio, UINT32_C(0x8000), &sampled));
    CHECK_EQ_U32(0u, mock_gpio_dr_reads);
    mock_push_read(&regs.DR.WORD, UINT32_C(0xbeef0001));
    CHECK_EQ_U32(C2_OK,
                 c2_gpio_read_inputs(&gpio, UINT32_C(0x0001), &sampled));
    CHECK_EQ_U32(1u, sampled);
    CHECK_EQ_U32(1u, mock_gpio_dr_reads);
    CHECK_EQ_U32(0u, mock_gpio_ddr_reads);
    CHECK_EQ_SIZE(mock_read_count, mock_read_index);
    CHECK_EQ_U32(0u, mock_failures);
}

static void test_gpio_psram_bit15_sequence(void)
{
    C2_GPIO_TypeDef regs = {0};
    c2_gpio_t gpio = {0};

    mock_gpio_dr = &regs.DR.WORD;
    mock_gpio_ddr = &regs.DDR.WORD;
    mock_reset();

    CHECK_EQ_U32(C2_OK,
                 c2_gpio_init(&gpio, &regs, C2_GPIO_PIN_MASK, 0u));
    CHECK_EQ_U32(C2_OK,
                 c2_gpio_clear_mask(&gpio, UINT32_C(0x8000)));
    CHECK_EQ_U32(C2_OK,
                 c2_gpio_set_direction_mask(&gpio, UINT32_C(0x8000),
                                            C2_GPIO_DIRECTION_OUTPUT));
    CHECK_EQ_U32(C2_OK,
                 c2_gpio_set_mask(&gpio, UINT32_C(0x8000)));
    CHECK_EQ_U32(0u, mock_gpio_dr_reads);
    CHECK_EQ_U32(0u, mock_gpio_ddr_reads);
    check_write(0u, &regs.DDR.WORD, UINT32_C(0xffff));
    check_write(1u, &regs.DR.WORD, 0u);
    check_write(2u, &regs.DDR.WORD, 0u);
    check_write(3u, &regs.DR.WORD, 0u);
    check_write(4u, &regs.DDR.WORD, 0u);
    check_write(5u, &regs.DR.WORD, UINT32_C(0x8000));
    CHECK_EQ_U32(0u, mock_failures);
}

static void test_sys_uart(void)
{
    C2_SYS_UART_TypeDef regs = {0};
    c2_sys_uart_t uart = {0};
    uint8_t byte = UINT8_C(0xee);
    uint8_t buffer[3] = {0};
    const uint8_t tx[3] = {UINT8_C(0x11), UINT8_C(0x22), UINT8_C(0x33)};
    size_t count = 99u;

    mock_sys_data = &regs.DATA.WORD;
    mock_reset();

    CHECK_EQ_U32(C2_ERROR_INVALID_ARGUMENT,
                 c2_sys_uart_init(NULL, &regs, UINT32_C(72000000),
                                  UINT32_C(115200)));
    CHECK_EQ_U32(C2_ERROR_INVALID_ARGUMENT,
                 c2_sys_uart_init(&uart, &regs, 0u, UINT32_C(115200)));
    CHECK_EQ_U32(C2_ERROR_INVALID_ARGUMENT,
                 c2_sys_uart_init(&uart, &regs, 1u, 2u));
    CHECK_EQ_U32(C2_OK,
                 c2_sys_uart_init(&uart, &regs, UINT32_C(72000000),
                                  UINT32_C(115200)));
    CHECK_EQ_U32(UINT32_C(625), regs.CLKDIV.WORD);
    check_write(0u, &regs.CLKDIV.WORD, UINT32_C(625));
    CHECK_EQ_U32(C2_OK,
                 c2_sys_uart_configure(&uart, UINT32_C(48000000),
                                       UINT32_C(9600)));
    CHECK_EQ_U32(UINT32_C(5000), regs.CLKDIV.WORD);

    CHECK_EQ_U32(C2_OK, c2_sys_uart_write_byte(&uart, UINT8_C(0xa5)));
    CHECK_EQ_U32(UINT32_C(0xa5), regs.DATA.WORD);
    CHECK_EQ_U32(C2_OK, c2_sys_uart_write(&uart, tx, 3u, &count));
    CHECK_EQ_SIZE(3u, count);
    CHECK_EQ_U32(0u, mock_sys_data_reads);

    mock_push_read(&regs.DATA.WORD, UINT32_C(0x0000015a));
    CHECK_EQ_U32(C2_ERROR_BUSY,
                 c2_sys_uart_read_byte_nonblocking(&uart, &byte));
    CHECK_EQ_U32(UINT8_C(0xee), byte);
    mock_push_read(&regs.DATA.WORD, UINT32_C(0x0000005a));
    CHECK_EQ_U32(C2_OK, c2_sys_uart_read_byte_nonblocking(&uart, &byte));
    CHECK_EQ_U32(UINT8_C(0x5a), byte);

    CHECK_EQ_U32(C2_ERROR_TIMEOUT, c2_sys_uart_read_byte(&uart, &byte, 0u));
    CHECK_EQ_U32(2u, mock_sys_data_reads);
    mock_push_read(&regs.DATA.WORD, UINT32_C(0xffffffff));
    mock_push_read(&regs.DATA.WORD, UINT32_C(0x00010000));
    mock_push_read(&regs.DATA.WORD, UINT32_C(0x0000007c));
    CHECK_EQ_U32(C2_OK, c2_sys_uart_read_byte(&uart, &byte, 3u));
    CHECK_EQ_U32(UINT8_C(0x7c), byte);

    mock_push_read(&regs.DATA.WORD, UINT32_C(0xffffffff));
    mock_push_read(&regs.DATA.WORD, UINT32_C(0x00000012));
    mock_push_read(&regs.DATA.WORD, UINT32_C(0x00000034));
    CHECK_EQ_U32(C2_OK,
                 c2_sys_uart_read(&uart, buffer, 2u, &count, 3u));
    CHECK_EQ_SIZE(2u, count);
    CHECK_EQ_U32(UINT8_C(0x12), buffer[0]);
    CHECK_EQ_U32(UINT8_C(0x34), buffer[1]);

    CHECK_EQ_U32(C2_OK, c2_sys_uart_disable(&uart));
    CHECK_EQ_U32(C2_ERROR_NOT_INITIALIZED,
                 c2_sys_uart_write_byte(&uart, 0u));
    CHECK_EQ_SIZE(mock_read_count, mock_read_index);
    CHECK_EQ_U32(0u, mock_failures);
}

static void test_hp_uart_configuration_irq_and_flush(void)
{
    C2_HP_UART_TypeDef regs = {0};
    c2_hp_uart_t uart = {0};
    c2_hp_uart_config_t config = hp_default_config();
    uint32_t status_word = 0u;

    mock_hp_lsr = &regs.LSR.WORD;
    mock_reset();

    config.data_bits = 4u;
    CHECK_EQ_U32(C2_ERROR_INVALID_ARGUMENT,
                 c2_hp_uart_init(&uart, &regs, UINT32_C(72000000), &config));
    CHECK_EQ_SIZE(0u, mock_write_count);
    config = hp_default_config();
    config.irq_enable_mask = UINT32_C(8);
    CHECK_EQ_U32(C2_ERROR_INVALID_ARGUMENT,
                 c2_hp_uart_init(&uart, &regs, UINT32_C(72000000), &config));
    config = hp_default_config();
    CHECK_EQ_U32(C2_ERROR_INVALID_ARGUMENT,
                 c2_hp_uart_init(&uart, &regs, UINT32_C(200000), &config));

    CHECK_EQ_U32(C2_OK,
                 c2_hp_uart_init(&uart, &regs, UINT32_C(72000000), &config));
    CHECK_EQ_SIZE(5u, mock_write_count);
    check_write(0u, &regs.LCR.WORD, 0u);
    check_write(1u, &regs.DIV.WORD, UINT32_C(624));
    check_write(2u, &regs.FCR.WORD, UINT32_C(0x0f));
    check_write(3u, &regs.FCR.WORD, UINT32_C(0x0c));
    check_write(4u, &regs.LCR.WORD, UINT32_C(0x18));

    config.data_bits = 4u;
    CHECK_EQ_U32(C2_ERROR_INVALID_ARGUMENT,
                 c2_hp_uart_init(&uart, &regs, UINT32_C(72000000), &config));
    CHECK_EQ_SIZE(5u, mock_write_count);
    CHECK_TRUE(uart.initialized);
    CHECK_TRUE(uart.regs == &regs);
    config = hp_default_config();

    CHECK_EQ_U32(C2_OK,
                 c2_hp_uart_set_irq_mask(&uart,
                                         C2_HP_UART_IRQ_RX |
                                         C2_HP_UART_IRQ_TX));
    check_write(5u, &regs.LCR.WORD, UINT32_C(0x1b));
    CHECK_EQ_U32(C2_ERROR_INVALID_ARGUMENT,
                 c2_hp_uart_set_irq_mask(&uart, UINT32_C(0x100)));

    CHECK_EQ_U32(C2_OK, c2_hp_uart_flush(&uart, C2_HP_UART_FLUSH_TX));
    check_write(6u, &regs.FCR.WORD, UINT32_C(0x0e));
    check_write(7u, &regs.FCR.WORD, UINT32_C(0x0c));
    CHECK_EQ_U32(C2_ERROR_INVALID_ARGUMENT,
                 c2_hp_uart_flush(&uart, (c2_hp_uart_flush_t)0));

    mock_push_read(&regs.LSR.WORD,
                   C2_HP_UART_STATUS_TX_FIFO_FULL |
                   C2_HP_UART_STATUS_RX_FIFO_EMPTY |
                   UINT32_C(0xfffffe00));
    CHECK_EQ_U32(C2_OK, c2_hp_uart_get_status(&uart, &status_word));
    CHECK_EQ_U32(C2_HP_UART_STATUS_TX_FIFO_FULL |
                 C2_HP_UART_STATUS_RX_FIFO_EMPTY,
                 status_word);

    config = hp_default_config();
    config.data_bits = 7u;
    config.stop_bits = C2_UART_STOP_BITS_2;
    config.parity = C2_UART_PARITY_EVEN;
    config.irq_enable_mask = C2_HP_UART_IRQ_PARITY;
    config.rx_trigger = C2_HP_UART_RX_TRIGGER_2_BYTES;
    config.baud_rate = UINT32_C(9600);
    CHECK_EQ_U32(C2_OK,
                 c2_hp_uart_configure(&uart, UINT32_C(48000000), &config));
    CHECK_EQ_U32(UINT32_C(4999), regs.DIV.WORD);
    CHECK_EQ_U32(UINT32_C(0xf4), regs.LCR.WORD);
    CHECK_EQ_U32(UINT32_C(0x04), regs.FCR.WORD);
    CHECK_EQ_SIZE(mock_read_count, mock_read_index);
    CHECK_EQ_U32(0u, mock_failures);
}

static void test_hp_uart_io_and_budgets(void)
{
    C2_HP_UART_TypeDef regs = {0};
    c2_hp_uart_t uart = {0};
    c2_hp_uart_config_t config = hp_default_config();
    const uint8_t tx[2] = {UINT8_C(0x41), UINT8_C(0x42)};
    uint8_t rx[2] = {0};
    uint8_t byte = 0u;
    size_t count = 0u;
    size_t writes_before;

    mock_hp_lsr = &regs.LSR.WORD;
    mock_reset();
    CHECK_EQ_U32(C2_OK,
                 c2_hp_uart_init(&uart, &regs, UINT32_C(72000000), &config));
    mock_write_count = 0u;

    mock_push_read(&regs.LSR.WORD, C2_HP_UART_STATUS_TX_FIFO_FULL);
    CHECK_EQ_U32(C2_ERROR_BUSY,
                 c2_hp_uart_write_byte_nonblocking(&uart, UINT8_C(0x55)));
    CHECK_EQ_SIZE(0u, mock_write_count);
    mock_push_read(&regs.LSR.WORD, 0u);
    CHECK_EQ_U32(C2_OK,
                 c2_hp_uart_write_byte_nonblocking(&uart, UINT8_C(0x55)));
    check_write(0u, &regs.TRX.WORD, UINT32_C(0x55));

    writes_before = mock_write_count;
    CHECK_EQ_U32(C2_ERROR_TIMEOUT,
                 c2_hp_uart_write_byte(&uart, UINT8_C(0x66), 0u));
    CHECK_EQ_SIZE(writes_before, mock_write_count);
    mock_push_read(&regs.LSR.WORD, C2_HP_UART_STATUS_TX_FIFO_FULL);
    mock_push_read(&regs.LSR.WORD, 0u);
    CHECK_EQ_U32(C2_OK,
                 c2_hp_uart_write_byte(&uart, UINT8_C(0x66), 2u));

    CHECK_EQ_U32(C2_ERROR_TIMEOUT,
                 c2_hp_uart_wait_tx_complete(&uart, 0u));
    mock_push_read(&regs.LSR.WORD, C2_HP_UART_STATUS_TX_FIFO_EMPTY);
    mock_push_read(&regs.LSR.WORD, C2_HP_UART_STATUS_TX_IDLE);
    CHECK_EQ_U32(C2_OK, c2_hp_uart_wait_tx_complete(&uart, 2u));
    mock_push_read(&regs.LSR.WORD, C2_HP_UART_STATUS_TX_FIFO_EMPTY);
    mock_push_read(&regs.LSR.WORD, C2_HP_UART_STATUS_TX_FIFO_EMPTY);
    CHECK_EQ_U32(C2_ERROR_TIMEOUT,
                 c2_hp_uart_wait_tx_complete(&uart, 2u));

    mock_push_read(&regs.LSR.WORD, C2_HP_UART_STATUS_TX_FIFO_FULL);
    mock_push_read(&regs.LSR.WORD, 0u);
    mock_push_read(&regs.LSR.WORD, 0u);
    CHECK_EQ_U32(C2_OK, c2_hp_uart_write(&uart, tx, 2u, &count, 3u));
    CHECK_EQ_SIZE(2u, count);

    mock_push_read(&regs.LSR.WORD, C2_HP_UART_STATUS_RX_FIFO_EMPTY);
    CHECK_EQ_U32(C2_ERROR_BUSY,
                 c2_hp_uart_read_byte_nonblocking(&uart, &byte));
    mock_push_read(&regs.LSR.WORD, 0u);
    mock_push_read(&regs.TRX.WORD, UINT32_C(0x5a));
    CHECK_EQ_U32(C2_OK, c2_hp_uart_read_byte_nonblocking(&uart, &byte));
    CHECK_EQ_U32(UINT8_C(0x5a), byte);

    mock_push_read(&regs.LSR.WORD, C2_HP_UART_STATUS_PARITY_ERROR);
    mock_push_read(&regs.TRX.WORD, UINT32_C(0x33));
    CHECK_EQ_U32(C2_ERROR_IO,
                 c2_hp_uart_read_byte_nonblocking(&uart, &byte));
    CHECK_EQ_U32(UINT8_C(0x33), byte);

    CHECK_EQ_U32(C2_ERROR_TIMEOUT,
                 c2_hp_uart_read_byte(&uart, &byte, 0u));
    mock_push_read(&regs.LSR.WORD, C2_HP_UART_STATUS_RX_FIFO_EMPTY);
    mock_push_read(&regs.LSR.WORD, 0u);
    mock_push_read(&regs.TRX.WORD, UINT32_C(0x77));
    CHECK_EQ_U32(C2_OK, c2_hp_uart_read_byte(&uart, &byte, 2u));
    CHECK_EQ_U32(UINT8_C(0x77), byte);

    mock_push_read(&regs.LSR.WORD, C2_HP_UART_STATUS_RX_FIFO_EMPTY);
    mock_push_read(&regs.LSR.WORD, 0u);
    mock_push_read(&regs.TRX.WORD, UINT32_C(0x10));
    mock_push_read(&regs.LSR.WORD, 0u);
    mock_push_read(&regs.TRX.WORD, UINT32_C(0x20));
    CHECK_EQ_U32(C2_OK, c2_hp_uart_read(&uart, rx, 2u, &count, 3u));
    CHECK_EQ_SIZE(2u, count);
    CHECK_EQ_U32(UINT8_C(0x10), rx[0]);
    CHECK_EQ_U32(UINT8_C(0x20), rx[1]);

    CHECK_EQ_U32(C2_OK, c2_hp_uart_disable(&uart));
    CHECK_EQ_U32(UINT32_C(0x18), regs.LCR.WORD);
    CHECK_EQ_U32(C2_ERROR_NOT_INITIALIZED,
                 c2_hp_uart_get_status(&uart, NULL));
    CHECK_EQ_SIZE(mock_read_count, mock_read_index);
    CHECK_EQ_U32(0u, mock_failures);
}

static void test_hp_uart_configuration_boundaries(void)
{
    static const struct {
        uint32_t clock_hz;
        uint32_t baud_rate;
        c2_status_t result;
        uint32_t divider;
    } dividers[] = {
        {0u, 1u, C2_ERROR_INVALID_ARGUMENT, 0u},
        {72000000u, 0u, C2_ERROR_INVALID_ARGUMENT, 0u},
        {2u, 1u, C2_ERROR_INVALID_ARGUMENT, 0u},
        {3u, 1u, C2_OK, 2u},
        {65536u, 1u, C2_OK, 65535u},
        {65537u, 1u, C2_ERROR_INVALID_ARGUMENT, 0u},
        {UINT32_MAX, 1u, C2_ERROR_INVALID_ARGUMENT, 0u},
        {72000001u, 115200u, C2_OK, 624u}
    };
    static const c2_uart_parity_t parities[] = {
        C2_UART_PARITY_NONE, C2_UART_PARITY_ODD, C2_UART_PARITY_EVEN
    };
    static const uint32_t parity_masks[] = {0u, 0x40u, 0xc0u};
    static const uint32_t word_masks[] = {0u, 0x08u, 0x10u, 0x18u};
    C2_HP_UART_TypeDef regs = {0};
    c2_hp_uart_t uart = {0};
    c2_hp_uart_config_t config = hp_default_config();

    mock_hp_lsr = &regs.LSR.WORD;
    mock_reset();
    CHECK_EQ_U32(C2_OK, c2_hp_uart_init(&uart, &regs, 72000000u, &config));

    for (size_t i = 0u; i < sizeof(dividers) / sizeof(dividers[0]); ++i) {
        uint32_t old_divider = regs.DIV.WORD;
        uint32_t old_lcr = uart.lcr_shadow;
        uint32_t old_fcr = uart.fcr_trigger_shadow;

        mock_reset();
        config.baud_rate = dividers[i].baud_rate;
        CHECK_EQ_U32(dividers[i].result,
                     c2_hp_uart_configure(&uart, dividers[i].clock_hz, &config));
        CHECK_TRUE(uart.initialized);
        CHECK_TRUE(uart.regs == &regs);
        CHECK_EQ_U32(0u, mock_hp_lsr_reads);
        if (dividers[i].result == C2_OK) {
            CHECK_EQ_SIZE(5u, mock_write_count);
            CHECK_EQ_U32(dividers[i].divider, regs.DIV.WORD);
        } else {
            CHECK_EQ_SIZE(0u, mock_write_count);
            CHECK_EQ_U32(old_divider, regs.DIV.WORD);
            CHECK_EQ_U32(old_lcr, uart.lcr_shadow);
            CHECK_EQ_U32(old_fcr, uart.fcr_trigger_shadow);
        }
        CHECK_EQ_U32(0u, mock_failures);
    }

    /* Exercise every supported line format without depending on header bits. */
    config = hp_default_config();
    for (size_t width = 0u; width < 4u; ++width) {
        for (size_t parity = 0u; parity < 3u; ++parity) {
            for (unsigned stop = 0u; stop < 2u; ++stop) {
                uint32_t frame = word_masks[width] | parity_masks[parity] |
                                 (stop == 0u ? 0u : 0x20u);
                uint32_t trigger = (uint32_t)width << 2;

                mock_reset();
                config.data_bits = (uint8_t)(width + 5u);
                config.parity = parities[parity];
                config.stop_bits = stop == 0u ? C2_UART_STOP_BITS_1 :
                                               C2_UART_STOP_BITS_2;
                config.rx_trigger = (c2_hp_uart_rx_trigger_t)width;
                CHECK_EQ_U32(C2_OK, c2_hp_uart_configure(&uart, 72000000u, &config));
                CHECK_EQ_SIZE(5u, mock_write_count);
                check_write(2u, &regs.FCR.WORD, trigger | 3u);
                check_write(3u, &regs.FCR.WORD, trigger);
                check_write(4u, &regs.LCR.WORD, frame);
                CHECK_EQ_U32(frame, uart.lcr_shadow);
                CHECK_EQ_U32(trigger, uart.fcr_trigger_shadow);
                CHECK_EQ_U32(0u, mock_hp_lsr_reads);
                CHECK_EQ_U32(0u, mock_failures);
            }
        }
    }

    for (unsigned kind = 1u; kind <= 3u; ++kind) {
        mock_reset();
        CHECK_EQ_U32(C2_OK, c2_hp_uart_flush(&uart, (c2_hp_uart_flush_t)kind));
        CHECK_EQ_SIZE(2u, mock_write_count);
        check_write(0u, &regs.FCR.WORD, 0x0cu | kind);
        check_write(1u, &regs.FCR.WORD, 0x0cu);
        CHECK_EQ_U32(0x0cu, uart.fcr_trigger_shadow);
        CHECK_EQ_U32(0u, mock_hp_lsr_reads);
        CHECK_EQ_U32(0u, mock_failures);
    }
}

static void test_hp_uart_buffer_boundaries(void)
{
    C2_HP_UART_TypeDef regs = {0};
    c2_hp_uart_t uart = {0};
    c2_hp_uart_config_t config = hp_default_config();
    const uint8_t tx[] = {0x00u, 0xffu, 0x5au};
    uint8_t rx[3] = {0xeeu, 0xeeu, 0xeeu};
    uint8_t byte = 0xeeu;
    size_t count = 99u;
    size_t remaining;

    mock_hp_lsr = &regs.LSR.WORD;
    mock_reset();
    CHECK_EQ_U32(C2_OK, c2_hp_uart_init(&uart, &regs, 72000000u, &config));
    mock_reset();

    CHECK_EQ_U32(C2_OK, c2_hp_uart_write(&uart, NULL, 0u, &count, 0u));
    CHECK_EQ_SIZE(0u, count);
    count = 99u;
    CHECK_EQ_U32(C2_OK, c2_hp_uart_read(&uart, NULL, 0u, &count, 0u));
    CHECK_EQ_SIZE(0u, count);
    count = 99u;
    CHECK_EQ_U32(C2_ERROR_TIMEOUT, c2_hp_uart_write(&uart, tx, 3u, &count, 0u));
    CHECK_EQ_SIZE(0u, count);
    count = 99u;
    CHECK_EQ_U32(C2_ERROR_TIMEOUT, c2_hp_uart_read(&uart, rx, 3u, &count, 0u));
    CHECK_EQ_SIZE(0u, count);
    CHECK_EQ_U32(0xeeu, rx[0]);
    CHECK_EQ_U32(C2_ERROR_TIMEOUT, c2_hp_uart_wait_tx_complete(&uart, 0u));

    count = 99u;
    CHECK_EQ_U32(C2_ERROR_INVALID_ARGUMENT,
                 c2_hp_uart_write(&uart, NULL, 1u, &count, 1u));
    CHECK_EQ_SIZE(99u, count);
    CHECK_EQ_U32(C2_ERROR_INVALID_ARGUMENT,
                 c2_hp_uart_read(&uart, NULL, 1u, &count, 1u));
    CHECK_EQ_SIZE(99u, count);
    CHECK_EQ_U32(C2_ERROR_INVALID_ARGUMENT,
                 c2_hp_uart_write(&uart, tx, 1u, NULL, 1u));
    CHECK_EQ_U32(C2_ERROR_INVALID_ARGUMENT,
                 c2_hp_uart_read(&uart, rx, 1u, NULL, 1u));
    CHECK_EQ_U32(0u, mock_hp_lsr_reads);
    CHECK_EQ_SIZE(0u, mock_write_count);

    mock_push_read(&regs.LSR.WORD, 0u);
    mock_push_read(&regs.LSR.WORD, C2_HP_UART_STATUS_TX_FIFO_FULL);
    mock_push_read(&regs.LSR.WORD, C2_HP_UART_STATUS_TX_FIFO_FULL);
    CHECK_EQ_U32(C2_ERROR_TIMEOUT, c2_hp_uart_write(&uart, tx, 3u, &count, 3u));
    CHECK_EQ_SIZE(1u, count);
    CHECK_EQ_SIZE(1u, mock_write_count);
    CHECK_EQ_U32(3u, mock_hp_lsr_reads);
    check_write(0u, &regs.TRX.WORD, 0x00u);
    CHECK_EQ_SIZE(mock_read_count, mock_read_index);

    /* Resume explicitly from the accepted prefix; no hidden retry or flush. */
    mock_push_read(&regs.LSR.WORD, 0u);
    mock_push_read(&regs.LSR.WORD, 0u);
    CHECK_EQ_U32(C2_OK,
                 c2_hp_uart_write(&uart, tx + count, 3u - count, &remaining, 2u));
    CHECK_EQ_SIZE(2u, remaining);
    CHECK_EQ_SIZE(3u, mock_write_count);
    check_write(1u, &regs.TRX.WORD, 0xffu);
    check_write(2u, &regs.TRX.WORD, 0x5au);
    CHECK_EQ_SIZE(mock_read_count, mock_read_index);
    CHECK_EQ_U32(0u, mock_failures);

    mock_reset();
    mock_push_read(&regs.LSR.WORD, 0u);
    mock_push_read(&regs.TRX.WORD, 0x00u);
    mock_push_read(&regs.LSR.WORD, C2_HP_UART_STATUS_RX_FIFO_EMPTY);
    mock_push_read(&regs.LSR.WORD, C2_HP_UART_STATUS_RX_FIFO_EMPTY);
    CHECK_EQ_U32(C2_ERROR_TIMEOUT, c2_hp_uart_read(&uart, rx, 3u, &count, 3u));
    CHECK_EQ_SIZE(1u, count);
    CHECK_EQ_U32(0x00u, rx[0]);
    CHECK_EQ_U32(0xeeu, rx[1]);
    CHECK_EQ_U32(0xeeu, rx[2]);
    CHECK_EQ_U32(3u, mock_hp_lsr_reads);
    CHECK_EQ_SIZE(mock_read_count, mock_read_index);
    CHECK_EQ_SIZE(0u, mock_write_count);
    CHECK_EQ_U32(0u, mock_failures);

    mock_reset();
    mock_push_read(&regs.LSR.WORD, 0u);
    mock_push_read(&regs.TRX.WORD, 0xffu);
    mock_push_read(&regs.LSR.WORD, C2_HP_UART_STATUS_PARITY_ERROR);
    mock_push_read(&regs.TRX.WORD, 0x5au);
    mock_push_read(&regs.LSR.WORD, 0u);
    mock_push_read(&regs.TRX.WORD, 0x55u);
    CHECK_EQ_U32(C2_ERROR_IO, c2_hp_uart_read(&uart, rx, 3u, &count, 3u));
    CHECK_EQ_SIZE(2u, count);
    CHECK_EQ_U32(0xffu, rx[0]);
    CHECK_EQ_U32(0x5au, rx[1]);
    CHECK_EQ_U32(0xeeu, rx[2]);
    CHECK_EQ_SIZE(4u, mock_read_index);
    CHECK_EQ_U32(2u, mock_hp_lsr_reads);
    CHECK_EQ_U32(C2_OK, c2_hp_uart_read_byte_nonblocking(&uart, &byte));
    CHECK_EQ_U32(0x55u, byte);
    CHECK_EQ_SIZE(mock_read_count, mock_read_index);
    CHECK_EQ_SIZE(0u, mock_write_count);
    CHECK_EQ_U32(0u, mock_failures);

    mock_reset();
    mock_push_read(&regs.LSR.WORD,
                   C2_HP_UART_STATUS_RX_FIFO_EMPTY | C2_HP_UART_STATUS_PARITY_ERROR);
    CHECK_EQ_U32(C2_ERROR_BUSY, c2_hp_uart_read_byte_nonblocking(&uart, &byte));
    CHECK_EQ_U32(0x55u, byte);
    CHECK_EQ_SIZE(mock_read_count, mock_read_index);
    CHECK_EQ_SIZE(0u, mock_write_count);
    CHECK_EQ_U32(0u, mock_failures);
}

int main(void)
{
    test_gpio_parameters_and_shadow();
    test_gpio_psram_bit15_sequence();
    test_sys_uart();
    test_hp_uart_configuration_irq_and_flush();
    test_hp_uart_io_and_budgets();
    test_hp_uart_configuration_boundaries();
    test_hp_uart_buffer_boundaries();

    if (test_failures != 0u) {
        fprintf(stderr, "gpio/uart tests: %u failure(s)\n", test_failures);
        return 1;
    }
    puts("gpio/uart tests: PASS");
    return 0;
}
