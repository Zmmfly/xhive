/*
 * Common public types for libdivsufsort.
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

#ifndef DIVSUFSORT_TYPES_H
#define DIVSUFSORT_TYPES_H

/* Some freestanding GCC/libc combinations need the libc type metadata before
 * inttypes.h. Use it when available, without inventing typedefs or formats. */
#if defined(__has_include)
# if __has_include(<sys/types.h>)
#  include <sys/types.h>
# endif
#endif
#include <stdint.h>
#include <inttypes.h>

/** @brief Optional declaration annotation; empty for the SDK object library. */
#ifndef DIVSUFSORT_API
#define DIVSUFSORT_API
#endif

#ifndef SAUCHAR_T
#define SAUCHAR_T
/** @brief Unsigned input/output byte, independent of the index width. */
typedef uint8_t sauchar_t;
#endif

#ifndef SAINT_T
#define SAINT_T
/** @brief Signed 32-bit status or character value in both API variants. */
typedef int32_t saint_t;
#endif

#ifndef PRIdSAINT_T
/** @brief printf decimal format fragment for saint_t (always 32-bit). */
#define PRIdSAINT_T PRId32
#endif

#endif /* DIVSUFSORT_TYPES_H */
