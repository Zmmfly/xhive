/* Correctness-only N32H474 probe. Reuse the reviewed benchmark startup/linker,
 * but no platform.c, clock changes, GPIO markers, UART or hardware timing.
 * The private production helper is included, not reimplemented here. */
#include "../../src/pf.c"

#if !defined(__GNUC__) || !defined(__arm__) || !defined(__ARM_EABI__) || \
    !defined(__SOFTFP__) || defined(__ARM_FP) || defined(__clang__) || \
    !UKF_IEEE_BINARY32 || !UKF_IEEE_BINARY64
#error "This probe must exercise the GNU ARM EABI no-FP IEEE integer raw path"
#endif

struct pf_raw_probe_result {
    uint32_t magic;           /* 0x50524632, PRF2 */
    uint32_t phase;           /* 1=running, 2=done, 4=fault */
    uint32_t cpacr;
    uint32_t q15_checked;
    uint32_t q31_checked;
    uint32_t failures;
    uint32_t first_format;    /* 15 or 31; zero when no mismatch */
    uint32_t reserved;
    uint64_t first_bits;
    int64_t first_expected;
    int64_t first_actual;
};
volatile struct pf_raw_probe_result g_pf_raw_probe;

static __attribute__((noinline)) int64_t native_q15(float value)
{
    volatile float input = value;
    float scaled = input * 32768.0f;
    return (int32_t)(scaled >= 0 ? scaled + 0.5f : scaled - 0.5f);
}

static __attribute__((noinline)) int64_t native_q31(double value)
{
    volatile double input = value;
    double scaled = input * 2147483648.0;
    return (int64_t)(scaled >= 0 ? scaled + 0.5 : scaled - 0.5);
}

static void mismatch(uint32_t format, uint64_t bits, int64_t expected, int64_t actual)
{
    if (g_pf_raw_probe.failures == 0) {
        g_pf_raw_probe.first_format = format;
        g_pf_raw_probe.first_bits = bits;
        g_pf_raw_probe.first_expected = expected;
        g_pf_raw_probe.first_actual = actual;
    }
    ++g_pf_raw_probe.failures;
}

static void check_q15_bits(uint32_t bits)
{
    float value;
    int64_t expected, actual;
    memcpy(&value, &bits, sizeof(value));
    expected = native_q15(value);
    actual = pf_q_q15_raw(value);
    ++g_pf_raw_probe.q15_checked;
    if (actual != expected) mismatch(15, bits, expected, actual);
}

static void check_q31_bits(uint64_t bits)
{
    double value;
    int64_t expected, actual;
    memcpy(&value, &bits, sizeof(value));
    expected = native_q31(value);
    actual = pf_q_q31_raw(value);
    ++g_pf_raw_probe.q31_checked;
    if (actual != expected) mismatch(31, bits, expected, actual);
}

static void q31_cell(int32_t cell)
{
    double half = ((double)cell + 0.5) * 0x1p-31;
    uint64_t bits;
    memcpy(&bits, &half, sizeof(bits));
    check_q31_bits(bits - 1);
    check_q31_bits(bits);
    check_q31_bits(bits + 1);
}

__attribute__((noreturn, noinline)) void pf_raw_probe_done(void)
{
    for (;;) __asm volatile ("nop");
}

__attribute__((noreturn)) void bench_fault(void)
{
    g_pf_raw_probe.phase = 4;
    for (;;) __asm volatile ("nop");
}

void _init(void) {}
void _fini(void) {}

int main(void)
{
    int32_t cell;
    uint32_t random = UINT32_C(0x196ca371), i;
    static const uint32_t f_edges[] = {0, UINT32_C(0x80000000), UINT32_C(0x3f800000),
        UINT32_C(0xbf800000), UINT32_C(0x377fffff), UINT32_C(0xb77fffff)};
    static const uint64_t d_edges[] = {0, UINT64_C(0x8000000000000000), UINT64_C(0x3ff0000000000000),
        UINT64_C(0xbff0000000000000), UINT64_C(0x3defffffffffffff), UINT64_C(0xbdefffffffffffff)};
    static const int32_t cells[] = {INT32_MIN, INT32_MIN + 1, -1, 0, 1, INT32_MAX - 1, INT32_MAX};
    g_pf_raw_probe.magic = UINT32_C(0x50524632);
    g_pf_raw_probe.cpacr = *(volatile const uint32_t *)UINT32_C(0xe000ed88);
    g_pf_raw_probe.phase = 1;
    for (cell = INT16_MIN; cell <= INT16_MAX; ++cell) {
        float half = ((float)cell + 0.5f) * 0x1p-15f;
        uint32_t bits;
        memcpy(&bits, &half, sizeof(bits));
        check_q15_bits(bits - 1);
        check_q15_bits(bits);
        check_q15_bits(bits + 1);
    }
    for (i = 0; i < sizeof(f_edges) / sizeof(f_edges[0]); ++i) check_q15_bits(f_edges[i]);
    for (i = 0; i < sizeof(cells) / sizeof(cells[0]); ++i) q31_cell(cells[i]);
    for (i = 0; i < 100000; ++i) {
        random ^= random << 13;
        random ^= random >> 17;
        random ^= random << 5;
        q31_cell((int32_t)((int64_t)random + INT32_MIN));
    }
    for (i = 0; i < sizeof(d_edges) / sizeof(d_edges[0]); ++i) check_q31_bits(d_edges[i]);
    g_pf_raw_probe.phase = 2;
    pf_raw_probe_done();
}
