/* Deterministic host tests; never dereference a hardware address. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "c2_archinfo.h"
#include "c2_ps2.h"
#include "c2_rng.h"

static C2_RNG_TypeDef rng;
static C2_PS2_TypeDef ps2;
static C2_ARCHINFO_TypeDef arch;
static uint32_t rng_ctrl, rng_value, ps2_ctrl, ps2_stat;
static c2_archinfo_snapshot_t arch_value;
static uint32_t fifo[16];
static size_t fifo_size, fifo_pos;
static unsigned int reads, writes, status_reads, sample_reads;
static volatile uint32_t *write_addr[64];
static uint32_t write_value[64];

uint32_t c2_test_misc_read(volatile const uint32_t *address)
{
    ++reads;
    if (address == &rng.CTRL.WORD) {
        return rng_ctrl;
    }
    if (address == &rng.VAL.WORD) {
        ++sample_reads;
        return rng_value++;
    }
    if (address == &ps2.CTRL.WORD) {
        return ps2_ctrl;
    }
    if (address == &ps2.STAT.WORD) {
        uint32_t value = ps2_stat;
        ps2_stat = 0u;
        ++status_reads;
        return value;
    }
    if (address == &ps2.DATA.WORD) {
        return fifo_pos < fifo_size ? fifo[fifo_pos++] : 0u;
    }
    if (address == &arch.SYS.WORD) { return arch_value.sys; }
    if (address == &arch.IDL.WORD) { return arch_value.idl; }
    if (address == &arch.IDH.WORD) { return arch_value.idh; }
    assert(!"unexpected MMIO read (including WO SEED)");
    return 0u;
}

void c2_test_misc_write(volatile uint32_t *address, uint32_t value)
{
    assert(writes < 64u);
    write_addr[writes] = address;
    write_value[writes++] = value;
    if (address == &rng.CTRL.WORD) {
        assert(value <= 1u);
        rng_ctrl = value;
    } else if (address == &rng.SEED.WORD) {
        assert(rng_ctrl == 1u);
        rng_value = value;
    } else if (address == &ps2.CTRL.WORD) {
        assert(value <= 3u);
        ps2_ctrl = value;
    } else if (address == &arch.SYS.WORD) {
        arch_value.sys = value;
    } else if (address == &arch.IDL.WORD) {
        arch_value.idl = value;
    } else if (address == &arch.IDH.WORD) {
        arch_value.idh = value;
    } else {
        assert(!"unexpected MMIO write");
    }
}

static void reset(void)
{
    reads = writes = status_reads = sample_reads = 0u;
    fifo_pos = fifo_size = 0u;
}

static void test_rng(void)
{
    uint32_t value;
    uint8_t bytes[7] = {0};
    reset();
    assert(c2_rng_init(NULL, 1u) == C2_ERROR_INVALID_ARGUMENT);
    assert(c2_rng_init(&rng, 0u) == C2_ERROR_INVALID_ARGUMENT);
    assert(reads == 0u && writes == 0u);
    assert(c2_rng_init(&rng, 0x12345678u) == C2_OK);
    assert(writes == 3u && reads == 0u);
    assert(write_addr[0] == &rng.CTRL.WORD && write_value[0] == 1u);
    assert(write_addr[1] == &rng.SEED.WORD && write_value[1] == 0x12345678u);
    assert(write_addr[2] == &rng.CTRL.WORD && write_value[2] == 0u);
    assert(c2_rng_read(&rng, &value) == C2_OK && value == 0x12345678u);
    assert(c2_rng_seed_write_enable(&rng, true) == C2_OK);
    assert(c2_rng_seed(&rng, 42u) == C2_OK && rng_ctrl == 1u);
    assert(c2_rng_seed_write_enable(&rng, false) == C2_OK);
    assert(c2_rng_seed(&rng, 12u) == C2_OK && rng_ctrl == 0u);
    assert(c2_rng_stop(&rng) == C2_OK && rng_value == 0u && rng_ctrl == 0u);
    reset();
    assert(c2_rng_fill(&rng, NULL, 0u) == C2_OK);
    assert(c2_rng_fill(&rng, NULL, 1u) == C2_ERROR_INVALID_ARGUMENT);
    assert(c2_rng_read(&rng, NULL) == C2_ERROR_INVALID_ARGUMENT);
    assert(c2_rng_read(NULL, &value) == C2_ERROR_INVALID_ARGUMENT);
    assert(c2_rng_fill(NULL, bytes, sizeof(bytes)) == C2_ERROR_INVALID_ARGUMENT);
    assert(c2_rng_seed_write_enable(NULL, false) == C2_ERROR_INVALID_ARGUMENT);
    assert(c2_rng_seed(NULL, 1u) == C2_ERROR_INVALID_ARGUMENT);
    assert(c2_rng_stop(NULL) == C2_ERROR_INVALID_ARGUMENT);
    assert(reads == 0u && writes == 0u);
    rng_value = 0x44332211u;
    assert(c2_rng_fill(&rng, bytes, sizeof(bytes)) == C2_OK);
    assert(sample_reads == 2u && writes == 0u);
    assert(memcmp(bytes, "\x11\x22\x33\x44\x12\x22\x33", 7u) == 0);
}

static void test_ps2(void)
{
    bool pending = false;
    uint8_t data[3] = {0xaau, 0xaau, 0xaau};
    size_t received = 99u;
    reset();
    assert(c2_ps2_init(&ps2, false) == C2_OK && ps2_ctrl == 2u);
    assert(c2_ps2_irq_enable(&ps2, true) == C2_OK && ps2_ctrl == 3u);
    assert(c2_ps2_enable(&ps2, false) == C2_OK && ps2_ctrl == 1u);
    assert(c2_ps2_enable(&ps2, true) == C2_OK && ps2_ctrl == 3u);
    ps2_stat = 1u;
    assert(c2_ps2_status_ack(&ps2, &pending) == C2_OK && pending);
    assert(status_reads == 1u && ps2_stat == 0u);
    assert(c2_ps2_status_ack(&ps2, &pending) == C2_OK && !pending);
    assert(c2_ps2_try_read(&ps2, data) == C2_ERROR_BUSY && data[0] == 0xaau);
    assert(c2_ps2_read_raw(&ps2, data) == C2_OK && data[0] == 0u);
    reset();
    fifo[0] = 0u; fifo[1] = 0xf0u; fifo[2] = 0u; fifo[3] = 0x76u;
    fifo_size = 4u;
    assert(c2_ps2_receive(&ps2, data, 3u, &received, 4u) == C2_ERROR_TIMEOUT);
    assert(received == 2u && data[0] == 0xf0u && data[1] == 0x76u);
    assert(reads == 4u && status_reads == 0u && writes == 0u);
    reset();
    assert(c2_ps2_receive(&ps2, data, 1u, &received, 0u) == C2_ERROR_TIMEOUT);
    assert(received == 0u && reads == 0u);
    assert(c2_ps2_receive(&ps2, NULL, 0u, &received, 0u) == C2_OK);
    assert(c2_ps2_receive(&ps2, NULL, 1u, &received, 1u) == C2_ERROR_INVALID_ARGUMENT);
    assert(c2_ps2_receive(&ps2, data, 1u, NULL, 1u) == C2_ERROR_INVALID_ARGUMENT);
    assert(c2_ps2_status_ack(&ps2, NULL) == C2_ERROR_INVALID_ARGUMENT);
    assert(c2_ps2_status_ack(NULL, &pending) == C2_ERROR_INVALID_ARGUMENT);
    assert(c2_ps2_init(NULL, false) == C2_ERROR_INVALID_ARGUMENT);
    assert(c2_ps2_deinit(NULL) == C2_ERROR_INVALID_ARGUMENT);
    assert(c2_ps2_enable(NULL, true) == C2_ERROR_INVALID_ARGUMENT);
    assert(c2_ps2_irq_enable(NULL, true) == C2_ERROR_INVALID_ARGUMENT);
    assert(c2_ps2_read_raw(NULL, data) == C2_ERROR_INVALID_ARGUMENT);
    assert(c2_ps2_read_raw(&ps2, NULL) == C2_ERROR_INVALID_ARGUMENT);
    assert(c2_ps2_try_read(NULL, data) == C2_ERROR_INVALID_ARGUMENT);
    assert(c2_ps2_try_read(&ps2, NULL) == C2_ERROR_INVALID_ARGUMENT);
    assert(c2_ps2_receive(NULL, data, 1u, &received, 1u) == C2_ERROR_INVALID_ARGUMENT);
    assert(reads == 0u && writes == 0u);
    fifo[0] = 0x1cu; fifo_size = 1u;
    assert(c2_ps2_receive(&ps2, data, 1u, &received, 1u) == C2_OK);
    assert(received == 1u && data[0] == 0x1cu && reads == 1u);
    fifo_pos = 0u;
    assert(c2_ps2_try_read(&ps2, data) == C2_OK && data[0] == 0x1cu);
    assert(c2_ps2_irq_enable(&ps2, false) == C2_OK && ps2_ctrl == 2u);
    assert(c2_ps2_init(&ps2, true) == C2_OK && ps2_ctrl == 3u);
    assert(c2_ps2_deinit(&ps2) == C2_OK && ps2_ctrl == 0u);
}

static void test_archinfo(void)
{
    c2_archinfo_snapshot_t snapshot = {0x07280u, (2u << 30) | (0xabu << 22) |
                                                (0x0130u << 6) | 17u, 0x202611u};
    c2_archinfo_snapshot_t readback;
    c2_archinfo_info_t info = {0};
    reset();
    assert(c2_archinfo_write(&arch, &snapshot) == C2_OK && writes == 3u);
    assert(c2_archinfo_read(&arch, &readback) == C2_OK && reads == 3u);
    assert(readback.sys == snapshot.sys && readback.idl == snapshot.idl &&
           readback.idh == snapshot.idh);
    assert(c2_archinfo_decode(&readback, &info) == C2_OK);
    assert(info.clock_code == 72u && info.sram_kb == 128u);
    assert(info.process_nm == 130u && info.type == 2u && info.vendor == 0xabu);
    assert(info.custom == 17u && info.year == 2026u && info.month == 11u);
    snapshot.sys = 0x0a080u;
    assert(c2_archinfo_decode(&snapshot, &info) == C2_ERROR_IO);
    assert(info.clock_code == 72u && info.process_nm == 130u);
    snapshot.sys = 0x07280u; snapshot.idl = 0xau << 6;
    assert(c2_archinfo_decode(&snapshot, &info) == C2_ERROR_IO);
    snapshot.idl = 0u; snapshot.idh = 0x20260fu;
    assert(c2_archinfo_decode(&snapshot, &info) == C2_ERROR_IO);
    reset();
    snapshot.idh = 0xff000000u;
    assert(c2_archinfo_write(&arch, &snapshot) == C2_ERROR_INVALID_ARGUMENT);
    snapshot.idh = 0u; snapshot.sys = 0xfff00000u;
    assert(c2_archinfo_write(&arch, &snapshot) == C2_ERROR_INVALID_ARGUMENT);
    assert(c2_archinfo_write(NULL, &snapshot) == C2_ERROR_INVALID_ARGUMENT);
    assert(c2_archinfo_write(&arch, NULL) == C2_ERROR_INVALID_ARGUMENT);
    assert(c2_archinfo_read(NULL, &snapshot) == C2_ERROR_INVALID_ARGUMENT);
    assert(c2_archinfo_read(&arch, NULL) == C2_ERROR_INVALID_ARGUMENT);
    assert(c2_archinfo_decode(NULL, &info) == C2_ERROR_INVALID_ARGUMENT);
    assert(c2_archinfo_decode(&snapshot, NULL) == C2_ERROR_INVALID_ARGUMENT);
    assert(reads == 0u && writes == 0u);
}

int main(void)
{
    test_rng();
    test_ps2();
    test_archinfo();
    puts("ARCHINFO/RNG/PS2 mock tests passed");
    return 0;
}
