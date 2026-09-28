/*
 * divsufsort64.h for libdivsufsort64
 * Copyright (c) 2003-2008 Yuta Mori All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef DIVSUFSORT64_H
#define DIVSUFSORT64_H

#include "divsufsort_types.h"

#ifndef SAIDX64_T
#define SAIDX64_T
/** @brief Signed 64-bit byte offset, suffix index, or length. */
typedef int64_t saidx64_t;
#endif
#ifndef PRIdSAIDX64_T
/** @brief printf decimal format fragment for saidx64_t. */
#define PRIdSAIDX64_T PRId64
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Construct a suffix array with 64-bit indices.
 *
 * @param[in] T Input buffer containing n bytes. Must not be NULL, even for n=0.
 * @param[out] SA Output buffer for n indices. Must not be NULL, even for n=0.
 * @param[in] n Nonnegative input length in bytes.
 * @return saint_t Status; SA is valid on success.
 * @retval 0 Success, including n=0.
 * @retval -1 A required pointer is NULL or n is negative.
 * @retval -2 Internal allocation failed.
 *
 * @pre T and SA must not overlap. Buffer sizes must fit size_t.
 */
DIVSUFSORT_API saint_t
divsufsort64(const sauchar_t *T, saidx64_t *SA, saidx64_t n);

/**
 * @brief Compute a Burrows-Wheeler transform with 64-bit indices.
 *
 * @param[in] T Input buffer containing n bytes. Must not be NULL.
 * @param[out] U Output buffer for n bytes. Must not be NULL; may equal T.
 * @param[out] A Scratch buffer for at least n+1 indices, or NULL to allocate it.
 * @param[in] n Nonnegative input length in bytes.
 * @return saidx64_t Primary index: 0 for empty input, otherwise in [1,n].
 * @retval -1 T or U is NULL, or n is negative.
 * @retval -2 Internal allocation failed; no transform is available.
 *
 * @pre n+1 and all allocation sizes must be representable. A must not overlap T/U.
 * @note n=1 copies the input byte. Scratch contents are not preserved.
 */
DIVSUFSORT_API saidx64_t
divbwt64(const sauchar_t *T, sauchar_t *U, saidx64_t *A, saidx64_t n);

/**
 * @brief Get the version string of the 64-bit-index library.
 *
 * @return Non-NULL pointer to immutable static storage; do not free it.
 */
DIVSUFSORT_API const char *divsufsort64_version(void);

/**
 * @brief Compute a BWT using a 64-bit suffix array, or construct one internally.
 *
 * @param[in] T Input buffer containing n bytes. Must not be NULL.
 * @param[out] U Output buffer for n bytes. Must not be NULL; may equal T.
 * @param[in,out] SA Valid suffix array of n indices, or NULL to construct internally.
 *                   May be overwritten for an in-place transform.
 * @param[in] n Nonnegative input length in bytes.
 * @param[out] idx Required primary-index output; defined only on success.
 * @return saint_t Operation status.
 * @retval 0 Success; *idx is 0 for n=0, otherwise in [1,n].
 * @retval -1 A required pointer is NULL or n is negative.
 * @retval -2 Internal allocation failed when SA is NULL.
 *
 * @pre A supplied SA must be valid for T and must not overlap T/U or idx.
 *      When SA is NULL, divbwt64's allocation-size preconditions also apply.
 */
DIVSUFSORT_API saint_t
bw_transform64(const sauchar_t *T, sauchar_t *U, saidx64_t *SA,
               saidx64_t n, saidx64_t *idx);

/**
 * @brief Invert a Burrows-Wheeler transform using 64-bit indices.
 *
 * @param[in] T Transformed input containing n bytes. Must not be NULL.
 * @param[out] U Output buffer for n bytes. Must not be NULL.
 * @param[out] A Scratch buffer for n indices, or NULL to allocate it internally.
 * @param[in] n Nonnegative byte count; allocation sizes must fit size_t.
 * @param[in] idx Primary index: 0 for n=0, otherwise in [1,n].
 * @return saint_t Operation status.
 * @retval 0 Success; see the small-input behavior below.
 * @retval -1 T or U is NULL, n is negative, or idx is outside its allowed range.
 * @retval -2 Internal allocation failed.
 *
 * @pre A must not overlap T/U. T and idx must describe a valid BWT.
 * @note The existing implementation performs no output write for n<=1;
 *       callers must preserve/copy the sole byte themselves when n=1.
 */
DIVSUFSORT_API saint_t
inverse_bw_transform64(const sauchar_t *T, sauchar_t *U, saidx64_t *A,
                       saidx64_t n, saidx64_t idx);

/**
 * @brief Check a suffix array containing 64-bit indices.
 *
 * @param[in] T Input buffer containing n bytes. Must not be NULL.
 * @param[in] SA Array of n indices to check. Must not be NULL.
 * @param[in] n Nonnegative input length in bytes.
 * @param[in] verbose Zero for silence; nonzero enables stderr diagnostics.
 * @return saint_t Validation result.
 * @retval 0 Valid suffix array, including n=0.
 * @retval -1 A required pointer is NULL or n is negative.
 * @retval -2 An index is outside [0,n).
 * @retval -3 Suffix first bytes are not ordered.
 * @retval -4 A suffix is in the wrong position.
 */
DIVSUFSORT_API saint_t
sufcheck64(const sauchar_t *T, const saidx64_t *SA, saidx64_t n, saint_t verbose);

/**
 * @brief Search a byte pattern using a 64-bit suffix array.
 *
 * @param[in] T Non-NULL input buffer containing Tsize bytes.
 * @param[in] Tsize Nonnegative input length in bytes.
 * @param[in] P Non-NULL pattern buffer containing Psize bytes.
 * @param[in] Psize Nonnegative pattern length in bytes.
 * @param[in] SA Non-NULL sorted suffix array containing SAsize valid indices.
 * @param[in] SAsize Nonnegative number of suffix indices to search.
 * @param[out] left Optional first-match or insertion position in SA. Set to -1
 *                  for invalid arguments or an empty text/suffix array.
 * @return saidx64_t Match count; an empty pattern matches every supplied suffix
 *         when the text and suffix array are nonempty.
 * @retval -1 A required pointer is NULL or a size is negative.
 *
 * @pre SA must be valid and sorted for T. All buffer sizes must fit size_t.
 */
DIVSUFSORT_API saidx64_t
sa_search64(const sauchar_t *T, saidx64_t Tsize, const sauchar_t *P, saidx64_t Psize,
            const saidx64_t *SA, saidx64_t SAsize, saidx64_t *left);

/**
 * @brief Search a byte value using a 64-bit suffix array.
 *
 * @param[in] T Non-NULL input buffer containing Tsize bytes.
 * @param[in] Tsize Nonnegative input length in bytes.
 * @param[in] SA Non-NULL sorted suffix array containing SAsize valid indices.
 * @param[in] SAsize Nonnegative number of suffix indices to search.
 * @param[in] c Byte value to find, in [0,255].
 * @param[out] left Optional first-match or insertion position in SA. Set to -1
 *                  for invalid arguments or an empty text/suffix array.
 * @return saidx64_t Number of matching suffixes, or -1 for invalid arguments.
 * @retval -1 T or SA is NULL, or a size is negative.
 *
 * @pre SA must be valid and sorted for T. All buffer sizes must fit size_t.
 */
DIVSUFSORT_API saidx64_t
sa_simplesearch64(const sauchar_t *T, saidx64_t Tsize, const saidx64_t *SA,
                  saidx64_t SAsize, saint_t c, saidx64_t *left);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* DIVSUFSORT64_H */
