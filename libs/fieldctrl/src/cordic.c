#include "fieldctrl_arith.h"

/* Volder rotation/vectoring, 32 iterations. Angles use 2^62 counts/turn;
 * vector coordinates use 2^60 counts/unit. The largest vectoring coordinate
 * is < 2.34*2^60, so every signed addition fits int64_t. Unsigned-magnitude
 * shifts preserve division's toward-zero rule without a per-round divider. */
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

/* CORDIC coordinates have magnitude < 2^62 and bits is in [0,31]. Thus the
 * shifted magnitude fits int64_t before sign restoration. No implementation-
 * defined right shift of a negative signed value is used. */
static inline int64_t fc_coord_shift(int64_t value, unsigned bits) {
    uint64_t magnitude = value < 0 ? UINT64_C(0) - (uint64_t)value : (uint64_t)value;
    int64_t shifted = (int64_t)(magnitude >> bits);
    return value < 0 ? -shifted : shifted;
}

linears_fixed_t fcq_wrap(linears_fixed_t x, linears_status_t *status) {
    linears_fixed_t remainder;
    if (*status != LINEARS_OK) return 0;
    remainder = x % LINEARS_FIXED_ONE;
    return remainder < 0 ? remainder + LINEARS_FIXED_ONE : remainder;
}

void fcq_sincos(linears_fixed_t a, linears_fixed_t *sn, linears_fixed_t *cs,
               linears_status_t *status) {
    const int64_t turn = INT64_C(4611686018427387904);
    const int64_t quarter = INT64_C(1152921504606846976);
    int64_t x = INT64_C(700114967507363239), y = 0;
    int64_t z, dx, dy;
    int sign = 1;
    unsigned i;
    linears_fixed_t w = fcq_wrap(a, status), s = 0, c = 0;
    if (*status != LINEARS_OK) return;
    /* Invariant F31 turns to invariant F62 turns; the product is < 2^62. */
    z = w * LINEARS_FIXED_ONE;
    if (z == 0 || z == quarter || z == 2 * quarter || z == 3 * quarter) {
        s = z == quarter ? LINEARS_FIXED_ONE : (z == 3 * quarter ? -LINEARS_FIXED_ONE : 0);
        c = z == 0 ? LINEARS_FIXED_ONE : (z == 2 * quarter ? -LINEARS_FIXED_ONE : 0);
    } else {
        if (z >= 2 * quarter) z -= turn;
        if (z > quarter) { z -= 2 * quarter; sign = -1; }
        else if (z < -quarter) { z += 2 * quarter; sign = -1; }
        for (i = 0; i < 32; ++i) {
            dx = fc_coord_shift(x, i);
            dy = fc_coord_shift(y, i);
            if (z >= 0) { x -= dy; y += dx; z -= fc_atan_turns[i]; }
            else { x += dy; y -= dx; z += fc_atan_turns[i]; }
        }
        /* Core raw muldiv supplies nearest-away F60 -> F31 conversion. */
        *status = linears_fixed_muldiv(sign * y, 1, INT64_C(536870912), &s);
        if (*status == LINEARS_OK)
            *status = linears_fixed_muldiv(sign * x, 1, INT64_C(536870912), &c);
    }
    if (*status == LINEARS_OK) { *sn = s; *cs = c; }
}

linears_fixed_t fcq_atan2(linears_fixed_t fy, linears_fixed_t fx,
                        linears_status_t *status) {
    linears_fixed_t a = 0, ax, ay, largest, nx, ny;
    int64_t x, y, z = 0, dx, dy;
    unsigned i;
    const int64_t turn = INT64_C(4611686018427387904);
    if (*status != LINEARS_OK) return 0;
    if (fx == 0 && fy == 0) { *status = LINEARS_ERROR_DOMAIN; return 0; }
    if (fy == 0) return fx > 0 ? 0 : LINEARS_FIXED_ONE / 2;
    if (fx == 0) return fy > 0 ? LINEARS_FIXED_ONE / 4 : 3 * (LINEARS_FIXED_ONE / 4);
    ax = fcq_abs(fx, status);
    ay = fcq_abs(fy, status);
    largest = ax >= ay ? ax : ay;
    nx = fcq_div(fx, largest, status);
    ny = fcq_div(fy, largest, status);
    if (*status != LINEARS_OK) return 0;
    /* Unit ratios are F31. A constant 29-bit shift supplies F60 coordinates;
     * no input-dependent binary point or exponent is stored or propagated. */
    x = nx * INT64_C(536870912);
    y = ny * INT64_C(536870912);
    if (x < 0) { x = -x; y = -y; z = turn / 2; }
    for (i = 0; i < 32; ++i) {
        if (y == 0) break;
        dx = fc_coord_shift(x, i);
        dy = fc_coord_shift(y, i);
        if (y > 0) { x += dy; y -= dx; z += fc_atan_turns[i]; }
        else { x -= dy; y += dx; z -= fc_atan_turns[i]; }
    }
    if (z < 0) z += turn;
    if (z >= turn) z -= turn;
    *status = linears_fixed_muldiv(z, 1, LINEARS_FIXED_ONE, &a);
    return fcq_wrap(a, status);
}
