#include "fieldctrl_arith.h"

/* Volder rotation/vectoring, 32 iterations. Angles use 2^62 counts/turn;
 * vector coordinates use 2^60 counts/unit. The largest vectoring coordinate
 * is < 2.34*2^60, so every signed addition fits int64_t. Division, not signed
 * right shift, specifies rounding toward zero on all C99 implementations. */
static const int64_t fc_atan_turns[32] = {
    INT64_C(576460752303423488), INT64_C(340304653033718298),
    INT64_C(179807632645220259), INT64_C(91273161881380487),
    INT64_C(45813697873323707), INT64_C(22929182573009054),
    INT64_C(11467389120678282), INT64_C(5734044481687724),
    INT64_C(2867065987018958), INT64_C(1433538461969102),
    INT64_C(716769914547871), INT64_C(358385042719534),
    INT64_C(179192532040472), INT64_C(89596267355325),
    INT64_C(44798133844548), INT64_C(22399066943135),
    INT64_C(11199533474175), INT64_C(5599766737413),
    INT64_C(2799883368747), INT64_C(1399941684379),
    INT64_C(699970842190), INT64_C(349985421095),
    INT64_C(174992710548), INT64_C(87496355274),
    INT64_C(43748177637), INT64_C(21874088818),
    INT64_C(10937044409), INT64_C(5468522205),
    INT64_C(2734261102), INT64_C(1367130551),
    INT64_C(683565276), INT64_C(341782638)
};

linears_fixed_t fcq_wrap(linears_fixed_t x, linears_status_t *status) {
    linears_fixed_t r = {0, 0};
    const linears_fixed_t one = {INT32_C(1073741824), 1};
    int shift;
    int64_t magnitude, fraction;
    if (*status != LINEARS_OK || x.mantissa == 0 || x.exponent >= 31) return r;
    if (x.exponent <= 0) {
        r = x;
    } else {
        shift = 31 - x.exponent;
        magnitude = x.mantissa < 0 ? -(int64_t)x.mantissa : x.mantissa;
        fraction = magnitude & ((INT64_C(1) << shift) - 1);
        if (x.mantissa < 0) fraction = -fraction;
        *status = linears_fixed_from_i64(fraction, shift, 0, &r);
    }
    if (*status == LINEARS_OK && r.mantissa < 0) *status = linears_fixed_add(one, r, &r);
    if (*status == LINEARS_OK && linears_fixed_compare(r, one) >= 0) r = fcq_zero();
    return r;
}

/* Canonical nonnegative input below one, rounded to the angular accumulator. */
static int64_t fc_turn_counts(linears_fixed_t x) {
    int shift = x.exponent + 31;
    uint64_t m = (uint32_t)x.mantissa;
    if (x.mantissa == 0) return 0;
    if (shift >= 0) return (int64_t)(m << shift);
    shift = -shift;
    if (shift >= 64) return 0;
    return (int64_t)((m >> shift) + ((m & ((UINT64_C(1) << shift) - 1)) >=
                                    (UINT64_C(1) << (shift - 1))));
}

void fcq_sincos(linears_fixed_t a, linears_fixed_t *sn, linears_fixed_t *cs,
               linears_status_t *status) {
    const int64_t turn = INT64_C(4611686018427387904);
    const int64_t quarter = INT64_C(1152921504606846976);
    int64_t x = INT64_C(700114967507363239), y = 0;
    int64_t z, dx, dy;
    int sign = 1;
    unsigned i;
    linears_fixed_t w = fcq_wrap(a, status), s, c;
    if (*status != LINEARS_OK) return;
    z = fc_turn_counts(w);
    if (z == 0 || z == quarter || z == 2 * quarter || z == 3 * quarter) {
        s = fcq_i(z == quarter ? 1 : (z == 3 * quarter ? -1 : 0), status);
        c = fcq_i(z == 0 ? 1 : (z == 2 * quarter ? -1 : 0), status);
    } else {
        if (z >= 2 * quarter) z -= turn;
        if (z > quarter) { z -= 2 * quarter; sign = -1; }
        else if (z < -quarter) { z += 2 * quarter; sign = -1; }
        for (i = 0; i < 32; ++i) {
            dx = x / (INT64_C(1) << i);
            dy = y / (INT64_C(1) << i);
            if (z >= 0) { x -= dy; y += dx; z -= fc_atan_turns[i]; }
            else { x += dy; y -= dx; z += fc_atan_turns[i]; }
        }
        *status = linears_fixed_from_i64(sign * y, 60, 0, &s);
        if (*status == LINEARS_OK) *status = linears_fixed_from_i64(sign * x, 60, 0, &c);
        /* Working rounding can place a component one LSB outside [-1,1]. */
        if (*status == LINEARS_OK) {
            const linears_fixed_t one = {INT32_C(1073741824), 1};
            const linears_fixed_t negone = {-INT32_C(1073741824), 1};
            if (linears_fixed_compare(s, one) > 0) s = one;
            if (linears_fixed_compare(s, negone) < 0) s = negone;
            if (linears_fixed_compare(c, one) > 0) c = one;
            if (linears_fixed_compare(c, negone) < 0) c = negone;
        }
    }
    if (*status == LINEARS_OK) { *sn = s; *cs = c; }
}

/* Scale both coordinates into one 60-fractional-bit integer block. */
static int64_t fc_vector_counts(linears_fixed_t x, int exponent) {
    int shift = x.exponent - exponent + 29;
    uint64_t m = x.mantissa < 0 ? (uint64_t)(-(int64_t)x.mantissa) : (uint64_t)x.mantissa;
    if (shift >= 0) m <<= shift;
    else if (-shift >= 64) m = 0;
    else m >>= -shift;
    return x.mantissa < 0 ? -(int64_t)m : (int64_t)m;
}

linears_fixed_t fcq_atan2(linears_fixed_t fy, linears_fixed_t fx,
                        linears_status_t *status) {
    linears_fixed_t a = {0, 0};
    int exponent;
    int64_t x, y, z = 0, dx, dy;
    unsigned i;
    const int64_t turn = INT64_C(4611686018427387904);
    if (*status != LINEARS_OK) return a;
    if (fx.mantissa == 0 && fy.mantissa == 0) { *status = LINEARS_ERROR_DOMAIN; return a; }
    if (fy.mantissa == 0) return fx.mantissa > 0 ? a : (linears_fixed_t){INT32_C(1073741824), 0};
    if (fx.mantissa == 0) return fy.mantissa > 0 ? (linears_fixed_t){INT32_C(1073741824), -1} :
                                                                (linears_fixed_t){INT32_C(1610612736), 0};
    exponent = fx.exponent > fy.exponent ? fx.exponent : fy.exponent;
    x = fc_vector_counts(fx, exponent);
    y = fc_vector_counts(fy, exponent);
    if (x < 0) { x = -x; y = -y; z = turn / 2; }
    for (i = 0; i < 32; ++i) {
        if (y == 0) break;
        dx = x / (INT64_C(1) << i);
        dy = y / (INT64_C(1) << i);
        if (y > 0) { x += dy; y -= dx; z += fc_atan_turns[i]; }
        else { x -= dy; y += dx; z -= fc_atan_turns[i]; }
    }
    if (z < 0) z += turn;
    if (z >= turn) z -= turn;
    *status = linears_fixed_from_i64(z, 62, 0, &a);
    return fcq_wrap(a, status);
}
