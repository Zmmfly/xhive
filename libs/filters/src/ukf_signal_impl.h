/* Signal-boundary helpers for the filter templates. Include after U_JOIN,
 * U_TRY, U_TAG, U_FIXED, U_WORK, U_SIGNAL and U_FINITE are defined; the
 * including template undefines U_CHECK_FINITE, U_IMPORT_VIEW, U_FITS and
 * U_STORE itself. Q filters widen signals into working scratch at the cursor
 * and narrow results back in two passes (range check, then store). */
#if U_FIXED
#define U_CHECK_FINITE(matrix) ((void)0)
#define U_IMPORT_VIEW(matrix) U_JOIN(ukf_import_, U_TAG)((matrix), &cursor)
#define U_FITS(matrix) \
    U_TRY(U_JOIN(ukf_fits_, U_TAG)((const U_WORK *)(matrix)->data, (matrix)->rows * (matrix)->cols))
#define U_STORE(source, target) \
    U_JOIN(ukf_store_, U_TAG)((const U_WORK *)(source)->data, (source)->rows * (source)->cols, \
                              (U_SIGNAL *)(target)->data)
#else
#define U_CHECK_FINITE(matrix) U_TRY(U_FINITE(matrix))
#define U_IMPORT_VIEW(matrix) (*(matrix))
#define U_FITS(matrix) ((void)0)
#define U_STORE(source, target) \
    memcpy((target)->data, (source)->data, (source)->rows * (source)->cols * sizeof(U_WORK))
#endif
