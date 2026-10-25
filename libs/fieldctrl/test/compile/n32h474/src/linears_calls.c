/* Compile/link fixture only; this is not a numerical or hardware test. */
#include "linears.h"
#include "linears_decomp.h"
#include "linears_numeric.h"

/* Q factorization inputs stay in range; inversion uses (3/4)*Hadamard(2),
 * whose inverse has representable +/-2/3 entries despite wider LU work values. */
#define PROBE_LINEARS(suffix, raw_type, work_type, first, last, rhs, inv_first, inv_off, inv_last) \
static int probe_##suffix(void) { \
    raw_type adata[4] = {first, 0, 0, last}; \
    raw_type bdata[4] = {rhs, 0, 0, rhs}; \
    raw_type inverse_data[4] = {inv_first, inv_off, inv_off, inv_last}; \
    raw_type output[3][4] = {{0}}; \
    linears_matrix_t a = {adata, 2, 2}; \
    linears_matrix_t b = {bdata, 2, 2}; \
    linears_matrix_t inverse_input = {inverse_data, 2, 2}; \
    linears_matrix_t x = {output[0], 2, 2}; \
    linears_matrix_t y = {output[1], 2, 2}; \
    linears_matrix_t z = {output[2], 2, 2}; \
    linears_workspace_unit_t basic[LINEARS_WORKSPACE_UNITS(LINEARS_BASIC_WORKSPACE_BYTES(2, 2))]; \
    work_type values[LINEARS_QR_WORKSPACE(2, 2)]; \
    linears_##suffix##_workspace_t workspace = {values, LINEARS_QR_WORKSPACE(2, 2)}; \
    size_t permutation[2]; \
    uint8_t blocks[2]; \
    int parity = 0; \
    int result = linears_mul_##suffix(&a, &b, &x, basic, sizeof(basic)); \
    result |= linears_lu_##suffix(&a, &x, &y, permutation, 2, &parity, workspace); \
    result |= linears_solve_##suffix(&a, &b, &x, workspace); \
    result |= linears_inverse_##suffix(&inverse_input, &x, workspace); \
    z.rows = 1; z.cols = 1; \
    result |= linears_determinant_##suffix(&a, &z, workspace); \
    result |= linears_cholesky_##suffix(&a, &x, workspace); \
    result |= linears_qr_##suffix(&a, &x, &y, workspace); \
    result |= linears_ldlt_##suffix(&a, &x, &y, permutation, 2, blocks, 2, workspace); \
    z.rows = 2; z.cols = 1; \
    result |= linears_svd_##suffix(&a, &x, &z, &y, 32, workspace); \
    return result; \
}

PROBE_LINEARS(f32, float, float, 2.0f, 3.0f, 1.0f, 2.0f, 0.0f, 3.0f)
PROBE_LINEARS(f64, double, double, 2.0, 3.0, 1.0, 2.0, 0.0, 3.0)
PROBE_LINEARS(q15, int16_t, linears_fixed_t, 8192, 12288, 4096, 24576, 24576, -24576)
PROBE_LINEARS(q31, int32_t, linears_fixed_t, 536870912, 805306368, 268435456,
              1610612736, 1610612736, -1610612736)
#undef PROBE_LINEARS

int fieldctrl_probe_linears(void) {
    int result = probe_f32();
    result |= probe_f64();
    result |= probe_q15();
    result |= probe_q31();
    return result;
}
