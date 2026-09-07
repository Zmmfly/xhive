#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "c2_psram.h"
#include "c2_qspi.h"

typedef enum {
    MOCK_READ,
    MOCK_WRITE
} mock_operation_t;

typedef struct {
    mock_operation_t operation;
    const volatile uint32_t *address;
    uint32_t value;
} mock_event_t;

#define MOCK_EVENT_CAPACITY 128U
#define DELAY_STAGE_CAPACITY 8U

static uint32_t qspi_words[11];
static uint32_t psram_words[2];
static uint32_t gpio_words[2];
static mock_event_t mock_events[MOCK_EVENT_CAPACITY];
static size_t mock_event_count;
static size_t mock_event_index;
static c2_psram_delay_stage_t delay_stages[DELAY_STAGE_CAPACITY];
static size_t delay_expected_event_indices[DELAY_STAGE_CAPACITY];
static size_t delay_expected_count;
static size_t delay_stage_count;
static int delay_failure_stage;
static unsigned test_failures;

static C2_QSPI_TypeDef *mock_qspi(void)
{
    return (C2_QSPI_TypeDef *)(void *)qspi_words;
}

static C2_PSRAM_TypeDef *mock_psram(void)
{
    return (C2_PSRAM_TypeDef *)(void *)psram_words;
}

static C2_GPIO_TypeDef *mock_gpio_regs(void)
{
    return (C2_GPIO_TypeDef *)(void *)gpio_words;
}

static void fail_at(const char *message, size_t index)
{
    (void)fprintf(stderr, "mock failure at event %zu: %s\n", index, message);
    abort();
}

static bool address_in_block(const volatile uint32_t *address,
                             const uint32_t *block,
                             size_t block_bytes)
{
    uintptr_t current = (uintptr_t)(const volatile void *)address;
    uintptr_t begin = (uintptr_t)(const void *)block;
    return current >= begin && current < begin + block_bytes &&
           ((current - begin) % sizeof(uint32_t)) == 0U;
}

static void validate_address(const volatile uint32_t *address)
{
    if (!address_in_block(address, qspi_words, sizeof(qspi_words)) &&
        !address_in_block(address, psram_words, sizeof(psram_words)) &&
        !address_in_block(address, gpio_words, sizeof(gpio_words))) {
        fail_at("MMIO address outside mock blocks", mock_event_index);
    }
}

uint32_t qspi_psram_mock_read32(const volatile uint32_t *address)
{
    const mock_event_t *event;

    validate_address(address);
    if (mock_event_index >= mock_event_count) {
        fail_at("unexpected read", mock_event_index);
    }
    event = &mock_events[mock_event_index];
    if (event->operation != MOCK_READ) {
        fail_at("read where write was expected", mock_event_index);
    }
    if (event->address != address) {
        fail_at("read from wrong register", mock_event_index);
    }
    ++mock_event_index;
    return event->value;
}

void qspi_psram_mock_write32(volatile uint32_t *address, uint32_t value)
{
    const mock_event_t *event;

    validate_address(address);
    if (mock_event_index >= mock_event_count) {
        fail_at("unexpected write", mock_event_index);
    }
    event = &mock_events[mock_event_index];
    if (event->operation != MOCK_WRITE) {
        fail_at("write where read was expected", mock_event_index);
    }
    if (event->address != address) {
        fail_at("write to wrong register", mock_event_index);
    }
    if (event->value != value) {
        fail_at("write value mismatch", mock_event_index);
    }
    ++mock_event_index;
}

static void mock_reset(void)
{
    size_t i;

    for (i = 0U; i < sizeof(qspi_words) / sizeof(qspi_words[0]); ++i) {
        qspi_words[i] = 0U;
    }
    for (i = 0U; i < sizeof(psram_words) / sizeof(psram_words[0]); ++i) {
        psram_words[i] = 0U;
    }
    for (i = 0U; i < sizeof(gpio_words) / sizeof(gpio_words[0]); ++i) {
        gpio_words[i] = 0U;
    }
    mock_event_count = 0U;
    mock_event_index = 0U;
    delay_expected_count = 0U;
    delay_stage_count = 0U;
    delay_failure_stage = -1;
}

static void expect_event(mock_operation_t operation,
                         const volatile uint32_t *address,
                         uint32_t value)
{
    if (mock_event_count >= MOCK_EVENT_CAPACITY) {
        fail_at("event script overflow", mock_event_count);
    }
    mock_events[mock_event_count].operation = operation;
    mock_events[mock_event_count].address = address;
    mock_events[mock_event_count].value = value;
    ++mock_event_count;
}

static void expect_read(const volatile uint32_t *address, uint32_t value)
{
    expect_event(MOCK_READ, address, value);
}

static void expect_write(volatile uint32_t *address, uint32_t value)
{
    expect_event(MOCK_WRITE, address, value);
}

static void verify_script_consumed(const char *name)
{
    if (mock_event_index != mock_event_count) {
        (void)fprintf(stderr, "%s: consumed %zu of %zu MMIO events\n",
                      name, mock_event_index, mock_event_count);
        ++test_failures;
    }
}

#define CHECK(test_name, expression)                                             \
    do {                                                                         \
        if (!(expression)) {                                                     \
            (void)fprintf(stderr, "%s:%d: %s failed\n",                       \
                          (test_name), __LINE__, #expression);                    \
            ++test_failures;                                                     \
        }                                                                        \
    } while (false)

static c2_status_t test_delay(void *context, c2_psram_delay_stage_t stage)
{
    (void)context;
    if (delay_stage_count >= DELAY_STAGE_CAPACITY) {
        return C2_ERROR_IO;
    }
    if (delay_expected_count != 0U &&
        (delay_stage_count >= delay_expected_count ||
         mock_event_index != delay_expected_event_indices[delay_stage_count])) {
        fail_at("delay callback at wrong MMIO sequence point", mock_event_index);
    }
    delay_stages[delay_stage_count++] = stage;
    if ((int)stage == delay_failure_stage) {
        return C2_ERROR_IO;
    }
    return C2_OK;
}

static c2_gpio_t make_gpio_context(uint32_t output_shadow,
                                   uint32_t input_mask_shadow)
{
    c2_gpio_t gpio;
    gpio.regs = mock_gpio_regs();
    gpio.output_shadow = output_shadow;
    gpio.input_mask_shadow = input_mask_shadow;
    gpio.initialized = true;
    return gpio;
}

static void expect_psram_prefix(uint32_t gpio_output, uint32_t gpio_inputs)
{
    C2_QSPI_TypeDef *qspi = mock_qspi();
    C2_PSRAM_TypeDef *psram = mock_psram();
    C2_GPIO_TypeDef *gpio = mock_gpio_regs();

    expect_write(&qspi->STATUS.WORD, C2_QSPI_CONTROL_RESET_ASSERT);
    expect_write(&qspi->STATUS.WORD, 0U);
    expect_write(&qspi->INTCFG.WORD, 0U);
    expect_write(&qspi->DUM.WORD, 0U);
    expect_write(&qspi->CLKDIV.WORD, 1U);
    expect_write(&gpio->DR.WORD,
                 gpio_output & ~C2_PSRAM_QPI_MODE_GPIO_MASK);
    expect_write(&gpio->DDR.WORD,
                 gpio_inputs & ~C2_PSRAM_QPI_MODE_GPIO_MASK);
    expect_write(&psram->WC.WORD, C2_PSRAM_INIT_WC);
    expect_write(&psram->CHD.WORD, C2_PSRAM_INIT_CHD);
}

static void expect_psram_command(uint32_t command, uint32_t completion)
{
    C2_QSPI_TypeDef *qspi = mock_qspi();
    expect_write(&qspi->LEN.WORD, C2_QSPI_RAW_PSRAM_BYTE_LENGTH);
    expect_write(&qspi->TXFIFO.WORD, command << 24);
    expect_write(&qspi->STATUS.WORD, C2_QSPI_RAW_PSRAM_TRIGGER);
    expect_read(&qspi->STATUS.WORD, completion);
}

static void test_qspi_configuration(void)
{
    static const char name[] = "qspi_configuration";
    C2_QSPI_TypeDef *qspi = mock_qspi();

    mock_reset();
    expect_write(&qspi->CLKDIV.WORD, 7U);
    expect_write(&qspi->CMD.WORD, 0U);
    expect_write(&qspi->ADR.WORD, 0U);
    expect_write(&qspi->LEN.WORD, 0U);
    CHECK(name, c2_qspi_init(qspi, 7U) == C2_OK);

    expect_write(&qspi->CLKDIV.WORD, UINT32_MAX);
    CHECK(name, c2_qspi_set_clkdiv(qspi, UINT32_MAX) == C2_OK);
    expect_write(&qspi->STATUS.WORD, C2_QSPI_CONTROL_RESET_ASSERT);
    expect_write(&qspi->STATUS.WORD, 0U);
    CHECK(name, c2_qspi_reset(qspi) == C2_OK);
    expect_read(&qspi->STATUS.WORD, 1U);
    expect_read(&qspi->STATUS.WORD, 0U);
    expect_write(&qspi->CMD.WORD, (uint32_t)C2_QSPI_CS2);
    CHECK(name, c2_qspi_select(qspi, C2_QSPI_CS2, 2U) == C2_OK);
    CHECK(name, c2_qspi_select(qspi, (c2_qspi_chip_select_t)0U, 1U) ==
                C2_ERROR_INVALID_ARGUMENT);
    verify_script_consumed(name);

    mock_reset();
    expect_read(&qspi->STATUS.WORD, 1U);
    CHECK(name, c2_qspi_select(qspi, C2_QSPI_CS0, 1U) ==
                C2_ERROR_TIMEOUT);
    CHECK(name, c2_qspi_select(qspi, C2_QSPI_CS0, 0U) ==
                C2_ERROR_TIMEOUT);
    verify_script_consumed(name);
}

static void test_qspi_sdk_fifo_modes(void)
{
    static const char name[] = "qspi_sdk_fifo_modes";
    C2_QSPI_TypeDef *qspi = mock_qspi();
    const uint8_t bytes[] = { UINT8_C(0x66), UINT8_C(0x99) };
    size_t submitted = 99U;

    mock_reset();
    /* In SDK mode 1 and 3 are busy, while 2 has bit0 clear. */
    expect_read(&qspi->STATUS.WORD, 1U);
    expect_read(&qspi->STATUS.WORD, 3U);
    expect_read(&qspi->STATUS.WORD, 2U);
    expect_write(&qspi->TXFIFO.WORD, UINT32_C(0x11223344));
    CHECK(name, c2_qspi_fifo_write_word(qspi, UINT32_C(0x11223344), 3U) ==
                C2_OK);

    expect_read(&qspi->STATUS.WORD, 0U);
    expect_write(&qspi->TXFIFO.WORD, UINT32_C(0x66));
    expect_read(&qspi->STATUS.WORD, 1U);
    expect_read(&qspi->STATUS.WORD, 0U);
    expect_write(&qspi->TXFIFO.WORD, UINT32_C(0x99));
    expect_read(&qspi->STATUS.WORD, 0U);
    CHECK(name, c2_qspi_write_bytes(qspi, bytes, sizeof(bytes), 4U,
                                    &submitted) == C2_OK);
    CHECK(name, submitted == sizeof(bytes));

    expect_read(&qspi->STATUS.WORD, 1U);
    expect_read(&qspi->STATUS.WORD, 1U);
    CHECK(name, c2_qspi_fifo_write_word(qspi, 0U, 2U) == C2_ERROR_TIMEOUT);
    verify_script_consumed(name);

    mock_reset();
    submitted = 5U;
    CHECK(name, c2_qspi_write_bytes(qspi, bytes, sizeof(bytes), 2U,
                                    &submitted) == C2_ERROR_INVALID_ARGUMENT);
    CHECK(name, submitted == 0U);
    CHECK(name, c2_qspi_wait_sdk_idle(qspi, 0U) == C2_ERROR_TIMEOUT);
    CHECK(name, c2_qspi_write_bytes(qspi, NULL, 0U, 0U, NULL) == C2_OK);
    verify_script_consumed(name);
}

static void test_qspi_raw_transaction(void)
{
    static const char name[] = "qspi_raw_transaction";
    C2_QSPI_TypeDef *qspi = mock_qspi();
    c2_qspi_raw_transaction_t transaction = {
        C2_QSPI_RAW_WRITE_LEN | C2_QSPI_RAW_WRITE_TXFIFO,
        0U,
        0U,
        C2_QSPI_RAW_PSRAM_BYTE_LENGTH,
        0U,
        UINT32_C(0x66000000),
        C2_QSPI_RAW_PSRAM_TRIGGER,
        UINT32_MAX,
        C2_QSPI_RAW_PSRAM_DONE
    };
    uint32_t last_status = UINT32_MAX;
    uint32_t rx = 0U;

    mock_reset();
    expect_write(&qspi->LEN.WORD, C2_QSPI_RAW_PSRAM_BYTE_LENGTH);
    expect_write(&qspi->TXFIFO.WORD, UINT32_C(0x66000000));
    expect_write(&qspi->STATUS.WORD, C2_QSPI_RAW_PSRAM_TRIGGER);
    /* Raw completion is exact STATUS==1: 0/2/3/0x101 do not complete. */
    expect_read(&qspi->STATUS.WORD, 0U);
    expect_read(&qspi->STATUS.WORD, 2U);
    expect_read(&qspi->STATUS.WORD, 3U);
    expect_read(&qspi->STATUS.WORD, UINT32_C(0x101));
    expect_read(&qspi->STATUS.WORD, 1U);
    CHECK(name, c2_qspi_transaction_raw(qspi, &transaction, 5U,
                                        &last_status) == C2_OK);
    CHECK(name, last_status == 1U);

    expect_read(&qspi->RXFIFO.WORD, UINT32_C(0xdeadbeef));
    CHECK(name, c2_qspi_fifo_read_word_raw(qspi, &rx) == C2_OK);
    CHECK(name, rx == UINT32_C(0xdeadbeef));
    verify_script_consumed(name);

    mock_reset();
    CHECK(name, c2_qspi_transaction_raw(qspi, &transaction, 0U, NULL) ==
                C2_ERROR_INVALID_ARGUMENT);
    transaction.completion_mask = 0U;
    CHECK(name, c2_qspi_transaction_raw(qspi, &transaction, 1U, NULL) ==
                C2_ERROR_INVALID_ARGUMENT);
    verify_script_consumed(name);
}

static void test_psram_full_sequence(void)
{
    static const char name[] = "psram_full_sequence";
    C2_QSPI_TypeDef *qspi = mock_qspi();
    C2_PSRAM_TypeDef *psram = mock_psram();
    C2_GPIO_TypeDef *gpio_regs = mock_gpio_regs();
    c2_gpio_t gpio = make_gpio_context(UINT32_C(0x25), C2_GPIO_PIN_MASK);

    mock_reset();
    delay_expected_count = 4U;
    delay_expected_event_indices[0] = 9U;
    delay_expected_event_indices[1] = 13U;
    delay_expected_event_indices[2] = 17U;
    delay_expected_event_indices[3] = 21U;
    expect_psram_prefix(gpio.output_shadow, gpio.input_mask_shadow);
    expect_psram_command(UINT32_C(0x66), 1U);
    expect_psram_command(UINT32_C(0x99), 1U);
    expect_psram_command(UINT32_C(0x35), 1U);
    expect_write(&gpio_regs->DR.WORD,
                 UINT32_C(0x25) | C2_PSRAM_QPI_MODE_GPIO_MASK);
    expect_write(&psram->WC.WORD, C2_PSRAM_RUN_WC);
    expect_write(&psram->CHD.WORD, C2_PSRAM_RUN_CHD);

    CHECK(name, c2_psram_init(psram, qspi, &gpio, 3U, test_delay, NULL) ==
                C2_OK);
    CHECK(name, delay_stage_count == 4U);
    CHECK(name, delay_stages[0] == C2_PSRAM_DELAY_BEFORE_RESET_ENABLE);
    CHECK(name, delay_stages[1] == C2_PSRAM_DELAY_AFTER_RESET_ENABLE);
    CHECK(name, delay_stages[2] == C2_PSRAM_DELAY_AFTER_DEVICE_RESET);
    CHECK(name, delay_stages[3] == C2_PSRAM_DELAY_AFTER_ENTER_QPI);
    CHECK(name, gpio.output_shadow ==
                (UINT32_C(0x25) | C2_PSRAM_QPI_MODE_GPIO_MASK));
    CHECK(name, (gpio.input_mask_shadow & C2_PSRAM_QPI_MODE_GPIO_MASK) == 0U);
    verify_script_consumed(name);
}

static void test_psram_timeout_stops_before_run_mode(void)
{
    static const char name[] = "psram_timeout_stops_before_run_mode";
    C2_QSPI_TypeDef *qspi = mock_qspi();
    C2_PSRAM_TypeDef *psram = mock_psram();
    c2_gpio_t gpio = make_gpio_context(0U, C2_GPIO_PIN_MASK);

    mock_reset();
    delay_expected_count = 2U;
    delay_expected_event_indices[0] = 9U;
    delay_expected_event_indices[1] = 13U;
    expect_psram_prefix(0U, C2_GPIO_PIN_MASK);
    expect_psram_command(UINT32_C(0x66), 1U);
    expect_write(&qspi->LEN.WORD, C2_QSPI_RAW_PSRAM_BYTE_LENGTH);
    expect_write(&qspi->TXFIFO.WORD, UINT32_C(0x99000000));
    expect_write(&qspi->STATUS.WORD, C2_QSPI_RAW_PSRAM_TRIGGER);
    expect_read(&qspi->STATUS.WORD, 0U);
    expect_read(&qspi->STATUS.WORD, 0U);
    expect_read(&qspi->STATUS.WORD, 0U);

    CHECK(name, c2_psram_init(psram, qspi, &gpio, 4U, test_delay, NULL) ==
                C2_ERROR_TIMEOUT);
    CHECK(name, delay_stage_count == 2U);
    CHECK(name, gpio.output_shadow == 0U);
    CHECK(name, (gpio.input_mask_shadow & C2_PSRAM_QPI_MODE_GPIO_MASK) == 0U);
    verify_script_consumed(name);
}

static void test_psram_delay_failure_stops_before_qpi(void)
{
    static const char name[] = "psram_delay_failure_stops_before_qpi";
    C2_QSPI_TypeDef *qspi = mock_qspi();
    C2_PSRAM_TypeDef *psram = mock_psram();
    c2_gpio_t gpio = make_gpio_context(0U, C2_GPIO_PIN_MASK);

    mock_reset();
    delay_expected_count = 3U;
    delay_expected_event_indices[0] = 9U;
    delay_expected_event_indices[1] = 13U;
    delay_expected_event_indices[2] = 17U;
    delay_failure_stage = (int)C2_PSRAM_DELAY_AFTER_DEVICE_RESET;
    expect_psram_prefix(0U, C2_GPIO_PIN_MASK);
    expect_psram_command(UINT32_C(0x66), 1U);
    expect_psram_command(UINT32_C(0x99), 1U);
    CHECK(name, c2_psram_init(psram, qspi, &gpio, 3U, test_delay, NULL) ==
                C2_ERROR_IO);
    CHECK(name, delay_stage_count == 3U);
    CHECK(name, gpio.output_shadow == 0U);
    verify_script_consumed(name);
}

static void test_psram_parameters_are_non_destructive(void)
{
    static const char name[] = "psram_parameters_are_non_destructive";
    C2_QSPI_TypeDef *qspi = mock_qspi();
    C2_PSRAM_TypeDef *psram = mock_psram();
    c2_gpio_t gpio = make_gpio_context(0U, C2_GPIO_PIN_MASK);

    mock_reset();
    CHECK(name, c2_psram_init(psram, qspi, &gpio, 2U, NULL, NULL) ==
                C2_ERROR_INVALID_ARGUMENT);
    gpio.output_shadow = C2_PSRAM_QPI_MODE_GPIO_MASK;
    CHECK(name, c2_psram_init(psram, qspi, &gpio, 3U, NULL, NULL) ==
                C2_ERROR_BUSY);
    gpio.output_shadow = 0U;
    gpio.initialized = false;
    CHECK(name, c2_psram_init(psram, qspi, &gpio, 3U, NULL, NULL) ==
                C2_ERROR_NOT_INITIALIZED);
    CHECK(name, c2_psram_init(NULL, qspi, &gpio, 3U, NULL, NULL) ==
                C2_ERROR_INVALID_ARGUMENT);
    verify_script_consumed(name);
}

int main(void)
{
    test_qspi_configuration();
    test_qspi_sdk_fifo_modes();
    test_qspi_raw_transaction();
    test_psram_full_sequence();
    test_psram_timeout_stops_before_run_mode();
    test_psram_delay_failure_stops_before_qpi();
    test_psram_parameters_are_non_destructive();

    if (test_failures != 0U) {
        (void)fprintf(stderr, "%u QSPI/PSRAM test checks failed\n", test_failures);
        return EXIT_FAILURE;
    }
    (void)puts("QSPI/PSRAM tests passed");
    return EXIT_SUCCESS;
}
