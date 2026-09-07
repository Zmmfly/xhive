#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "c2_i2c.h"

typedef enum {
    MOCK_READ,
    MOCK_WRITE
} mock_operation_t;

typedef struct {
    mock_operation_t operation;
    size_t offset;
    uint32_t value;
} mock_event_t;

#define MOCK_EVENT_CAPACITY 256U

static C2_I2C_TypeDef mock_registers;
static mock_event_t mock_events[MOCK_EVENT_CAPACITY];
static size_t mock_event_count;
static size_t mock_event_index;
static unsigned test_failures;

static C2_I2C_TypeDef *mock_regs(void)
{
    return &mock_registers;
}

static void fail_at(const char *message, size_t index)
{
    (void)fprintf(stderr, "mock failure at event %zu: %s\n", index, message);
    abort();
}

static size_t address_offset(const volatile uint32_t *address)
{
    uintptr_t base = (uintptr_t)(const volatile void *)&mock_registers;
    uintptr_t current = (uintptr_t)(const volatile void *)address;

    if ((current < base) || (current >= base + sizeof(mock_registers))) {
        fail_at("MMIO address outside mock register block", mock_event_index);
    }
    return (size_t)(current - base);
}

uint32_t i2c_mock_read32(const volatile uint32_t *address)
{
    const mock_event_t *event;

    if (mock_event_index >= mock_event_count) {
        fail_at("unexpected read", mock_event_index);
    }
    event = &mock_events[mock_event_index];
    if (event->operation != MOCK_READ) {
        fail_at("read where write was expected", mock_event_index);
    }
    if (event->offset != address_offset(address)) {
        fail_at("read from wrong register", mock_event_index);
    }
    ++mock_event_index;
    return event->value;
}

void i2c_mock_write32(volatile uint32_t *address, uint32_t value)
{
    const mock_event_t *event;

    if (mock_event_index >= mock_event_count) {
        fail_at("unexpected write", mock_event_index);
    }
    event = &mock_events[mock_event_index];
    if (event->operation != MOCK_WRITE) {
        fail_at("write where read was expected", mock_event_index);
    }
    if (event->offset != address_offset(address)) {
        fail_at("write to wrong register", mock_event_index);
    }
    if (event->value != value) {
        fail_at("write value mismatch", mock_event_index);
    }
    ++mock_event_index;
}

static void mock_reset(void)
{
    mock_event_count = 0U;
    mock_event_index = 0U;
}

static void expect_event(mock_operation_t operation, size_t offset, uint32_t value)
{
    if (mock_event_count >= MOCK_EVENT_CAPACITY) {
        fail_at("event script overflow", mock_event_count);
    }
    mock_events[mock_event_count].operation = operation;
    mock_events[mock_event_count].offset = offset;
    mock_events[mock_event_count].value = value;
    ++mock_event_count;
}

static void expect_read(size_t offset, uint32_t value)
{
    expect_event(MOCK_READ, offset, value);
}

static void expect_write(size_t offset, uint32_t value)
{
    expect_event(MOCK_WRITE, offset, value);
}

static void expect_command(uint32_t pre_status,
                           uint32_t command,
                           uint32_t completion_status)
{
    expect_read(offsetof(C2_I2C_TypeDef, SR), pre_status);
    if ((pre_status & C2_I2C_STATUS_IF) != 0U) {
        expect_write(offsetof(C2_I2C_TypeDef, CMD), C2_I2C_CMD_IACK);
        expect_read(offsetof(C2_I2C_TypeDef, SR),
                    pre_status & ~C2_I2C_STATUS_IF);
    }
    expect_write(offsetof(C2_I2C_TypeDef, CMD), command);
    expect_read(offsetof(C2_I2C_TypeDef, SR), completion_status);
}

static void expect_begin(void)
{
    expect_read(offsetof(C2_I2C_TypeDef, CTRL), C2_I2C_CTRL_EN);
    expect_read(offsetof(C2_I2C_TypeDef, SR), 0U);
}

static void expect_tx(uint8_t byte)
{
    expect_write(offsetof(C2_I2C_TypeDef, TXR), byte);
}

static void expect_successful_stop(uint32_t pre_status, uint32_t rxk)
{
    expect_command(pre_status, C2_I2C_CMD_STO, C2_I2C_STATUS_IF | rxk);
    expect_write(offsetof(C2_I2C_TypeDef, CMD), C2_I2C_CMD_IACK);
    expect_read(offsetof(C2_I2C_TypeDef, SR), rxk);
}

static void verify_script_consumed(const char *test_name)
{
    if (mock_event_index != mock_event_count) {
        (void)fprintf(stderr, "%s: consumed %zu of %zu MMIO events\n",
                      test_name, mock_event_index, mock_event_count);
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

static void test_configure_and_control(void)
{
    static const char name[] = "configure_and_control";
    uint16_t prescaler = 0U;
    c2_i2c_status_snapshot_t snapshot;

    mock_reset();
    expect_read(offsetof(C2_I2C_TypeDef, SR), 0U);
    expect_read(offsetof(C2_I2C_TypeDef, CTRL),
                C2_I2C_CTRL_EN | C2_I2C_CTRL_IEN);
    expect_write(offsetof(C2_I2C_TypeDef, CTRL), C2_I2C_CTRL_IEN);
    expect_write(offsetof(C2_I2C_TypeDef, PSCR), 35U);
    CHECK(name, c2_i2c_configure(mock_regs(), 72000000U, 400000U,
                                 &prescaler) == C2_OK);
    CHECK(name, prescaler == 35U);

    expect_read(offsetof(C2_I2C_TypeDef, CTRL), C2_I2C_CTRL_IEN);
    expect_write(offsetof(C2_I2C_TypeDef, CTRL),
                 C2_I2C_CTRL_IEN | C2_I2C_CTRL_EN);
    CHECK(name, c2_i2c_enable(mock_regs()) == C2_OK);

    expect_read(offsetof(C2_I2C_TypeDef, CTRL),
                C2_I2C_CTRL_IEN | C2_I2C_CTRL_EN);
    expect_write(offsetof(C2_I2C_TypeDef, CTRL), C2_I2C_CTRL_EN);
    CHECK(name, c2_i2c_set_interrupt_enabled(mock_regs(), false) == C2_OK);

    expect_read(offsetof(C2_I2C_TypeDef, SR),
                C2_I2C_STATUS_IF | C2_I2C_STATUS_TIP |
                C2_I2C_STATUS_AL | C2_I2C_STATUS_BSY | C2_I2C_STATUS_RXK);
    CHECK(name, c2_i2c_get_status(mock_regs(), &snapshot) == C2_OK);
    CHECK(name, snapshot.interrupt_pending);
    CHECK(name, snapshot.transfer_in_progress);
    CHECK(name, snapshot.arbitration_lost);
    CHECK(name, snapshot.bus_busy);
    CHECK(name, snapshot.nack_received);

    verify_script_consumed(name);
}

static void test_write_sequence(void)
{
    static const char name[] = "write_sequence";
    const uint8_t data[] = { UINT8_C(0x5A) };

    mock_reset();
    expect_begin();
    expect_tx(UINT8_C(0xA0));
    expect_command(0U, C2_I2C_CMD_STA | C2_I2C_CMD_WR,
                   C2_I2C_STATUS_IF | C2_I2C_STATUS_BSY);
    expect_tx(data[0]);
    expect_command(C2_I2C_STATUS_IF | C2_I2C_STATUS_BSY,
                   C2_I2C_CMD_WR,
                   C2_I2C_STATUS_IF | C2_I2C_STATUS_BSY);
    expect_successful_stop(C2_I2C_STATUS_IF | C2_I2C_STATUS_BSY, 0U);

    CHECK(name, c2_i2c_master_write(mock_regs(), UINT8_C(0x50), data,
                                    sizeof(data), 2U) == C2_OK);
    verify_script_consumed(name);
}

static void test_repeated_start_and_read_ack(void)
{
    static const char name[] = "repeated_start_and_read_ack";
    const uint8_t register_address[] = { UINT8_C(0x12) };
    uint8_t data[2] = { 0U, 0U };

    mock_reset();
    expect_begin();
    expect_tx(UINT8_C(0xA2));
    expect_command(0U, C2_I2C_CMD_STA | C2_I2C_CMD_WR,
                   C2_I2C_STATUS_IF | C2_I2C_STATUS_BSY);
    expect_tx(register_address[0]);
    expect_command(C2_I2C_STATUS_IF | C2_I2C_STATUS_BSY,
                   C2_I2C_CMD_WR,
                   C2_I2C_STATUS_IF | C2_I2C_STATUS_BSY);

    expect_tx(UINT8_C(0xA3));
    expect_command(C2_I2C_STATUS_IF | C2_I2C_STATUS_BSY,
                   C2_I2C_CMD_STA | C2_I2C_CMD_WR,
                   C2_I2C_STATUS_IF | C2_I2C_STATUS_BSY);

    /* C2 RTL polarity: ACK=1 continues, ACK=0 terminates with NACK. */
    expect_command(C2_I2C_STATUS_IF | C2_I2C_STATUS_BSY,
                   C2_I2C_CMD_RD | C2_I2C_CMD_ACK,
                   C2_I2C_STATUS_IF | C2_I2C_STATUS_BSY);
    expect_read(offsetof(C2_I2C_TypeDef, RXR), UINT8_C(0x34));
    expect_command(C2_I2C_STATUS_IF | C2_I2C_STATUS_BSY,
                   C2_I2C_CMD_RD,
                   C2_I2C_STATUS_IF | C2_I2C_STATUS_BSY);
    expect_read(offsetof(C2_I2C_TypeDef, RXR), UINT8_C(0x56));
    expect_successful_stop(C2_I2C_STATUS_IF | C2_I2C_STATUS_BSY, 0U);

    CHECK(name, c2_i2c_master_write_read(mock_regs(), UINT8_C(0x51),
                                         register_address,
                                         sizeof(register_address), data,
                                         sizeof(data), 2U) == C2_OK);
    CHECK(name, data[0] == UINT8_C(0x34));
    CHECK(name, data[1] == UINT8_C(0x56));
    verify_script_consumed(name);
}

static void test_nack_cleanup(void)
{
    static const char name[] = "nack_cleanup";
    const uint8_t data[] = { UINT8_C(0x11) };
    uint32_t nack = C2_I2C_STATUS_IF | C2_I2C_STATUS_BSY |
                    C2_I2C_STATUS_RXK;

    mock_reset();
    expect_begin();
    expect_tx(UINT8_C(0xA0));
    expect_command(0U, C2_I2C_CMD_STA | C2_I2C_CMD_WR, nack);
    expect_read(offsetof(C2_I2C_TypeDef, SR), nack);
    expect_successful_stop(nack, C2_I2C_STATUS_RXK);

    CHECK(name, c2_i2c_master_write(mock_regs(), UINT8_C(0x50), data,
                                    sizeof(data), 2U) == C2_ERROR_NACK);
    verify_script_consumed(name);
}

static void test_arbitration_does_not_stop(void)
{
    static const char name[] = "arbitration_does_not_stop";
    const uint8_t data[] = { UINT8_C(0x22) };
    uint32_t arbitration = C2_I2C_STATUS_IF | C2_I2C_STATUS_AL |
                           C2_I2C_STATUS_BSY;

    mock_reset();
    expect_begin();
    expect_tx(UINT8_C(0xA0));
    expect_command(0U, C2_I2C_CMD_STA | C2_I2C_CMD_WR, arbitration);
    expect_read(offsetof(C2_I2C_TypeDef, SR), arbitration);
    expect_write(offsetof(C2_I2C_TypeDef, CMD), C2_I2C_CMD_IACK);
    expect_read(offsetof(C2_I2C_TypeDef, SR),
                C2_I2C_STATUS_AL | C2_I2C_STATUS_BSY);

    CHECK(name, c2_i2c_master_write(mock_regs(), UINT8_C(0x50), data,
                                    sizeof(data), 2U) ==
                C2_ERROR_ARBITRATION);
    verify_script_consumed(name);
}

static void test_timeout_is_bounded_and_stops(void)
{
    static const char name[] = "timeout_is_bounded_and_stops";
    const uint8_t data[] = { UINT8_C(0x33) };
    uint32_t active = C2_I2C_STATUS_TIP | C2_I2C_STATUS_BSY;

    mock_reset();
    expect_begin();
    expect_tx(UINT8_C(0xA0));
    expect_read(offsetof(C2_I2C_TypeDef, SR), 0U);
    expect_write(offsetof(C2_I2C_TypeDef, CMD),
                 C2_I2C_CMD_STA | C2_I2C_CMD_WR);
    expect_read(offsetof(C2_I2C_TypeDef, SR), active);
    expect_read(offsetof(C2_I2C_TypeDef, SR), active);

    expect_read(offsetof(C2_I2C_TypeDef, SR), active);
    expect_read(offsetof(C2_I2C_TypeDef, SR), active);
    expect_read(offsetof(C2_I2C_TypeDef, SR),
                C2_I2C_STATUS_IF | C2_I2C_STATUS_BSY);
    expect_successful_stop(C2_I2C_STATUS_IF | C2_I2C_STATUS_BSY, 0U);

    CHECK(name, c2_i2c_master_write(mock_regs(), UINT8_C(0x50), data,
                                    sizeof(data), 2U) == C2_ERROR_TIMEOUT);
    verify_script_consumed(name);
}

static void test_interrupt_ack(void)
{
    static const char name[] = "interrupt_ack";

    mock_reset();
    expect_read(offsetof(C2_I2C_TypeDef, SR), C2_I2C_STATUS_IF);
    expect_write(offsetof(C2_I2C_TypeDef, CMD), C2_I2C_CMD_IACK);
    expect_read(offsetof(C2_I2C_TypeDef, SR), 0U);
    CHECK(name, c2_i2c_ack_interrupt(mock_regs(), 1U) == C2_OK);
    verify_script_consumed(name);
}

static void test_parameter_boundaries(void)
{
    static const char name[] = "parameter_boundaries";
    uint8_t byte = 0U;

    mock_reset();
    CHECK(name, c2_i2c_configure(NULL, 1U, 1U, NULL) ==
                C2_ERROR_INVALID_ARGUMENT);
    CHECK(name, c2_i2c_configure(mock_regs(), 0U, 100000U, NULL) ==
                C2_ERROR_INVALID_ARGUMENT);
    CHECK(name, c2_i2c_configure(mock_regs(), UINT32_MAX, 1U, NULL) ==
                C2_ERROR_INVALID_ARGUMENT);
    CHECK(name, c2_i2c_configure_prescaler(mock_regs(),
                                            C2_I2C_PRESCALER_MAX + 1U) ==
                C2_ERROR_INVALID_ARGUMENT);
    CHECK(name, c2_i2c_master_write(mock_regs(), UINT8_C(0x80), &byte, 1U,
                                    1U) == C2_ERROR_INVALID_ARGUMENT);
    CHECK(name, c2_i2c_master_write(mock_regs(), UINT8_C(0x20), NULL, 1U,
                                    1U) == C2_ERROR_INVALID_ARGUMENT);
    CHECK(name, c2_i2c_master_read(mock_regs(), UINT8_C(0x20), &byte, 0U,
                                   1U) == C2_ERROR_INVALID_ARGUMENT);
    CHECK(name, c2_i2c_master_read(mock_regs(), UINT8_C(0x20), &byte, 1U,
                                   0U) == C2_ERROR_INVALID_ARGUMENT);
    CHECK(name, c2_i2c_master_write_read(mock_regs(), UINT8_C(0x20), &byte,
                                         0U, &byte, 1U, 1U) ==
                C2_ERROR_INVALID_ARGUMENT);
    CHECK(name, c2_i2c_ack_interrupt(mock_regs(), 0U) ==
                C2_ERROR_INVALID_ARGUMENT);
    verify_script_consumed(name);
}

int main(void)
{
    test_configure_and_control();
    test_write_sequence();
    test_repeated_start_and_read_ack();
    test_nack_cleanup();
    test_arbitration_does_not_stop();
    test_timeout_is_bounded_and_stops();
    test_interrupt_ack();
    test_parameter_boundaries();

    if (test_failures != 0U) {
        (void)fprintf(stderr, "%u I2C test checks failed\n", test_failures);
        return EXIT_FAILURE;
    }
    (void)puts("I2C tests passed");
    return EXIT_SUCCESS;
}
