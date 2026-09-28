/*
 * lfs.h for libdivsufsort
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

#ifndef DIVSUFSORT_LFS_H
#define DIVSUFSORT_LFS_H

/* Example-only I/O policy. Include this before any system header. */
#if !defined(DIVSUFSORT_NO_LFS) && (defined(__unix__) || defined(__APPLE__))
# ifndef _POSIX_C_SOURCE
#  define _POSIX_C_SOURCE 200112L
# endif
# ifndef _FILE_OFFSET_BITS
#  define _FILE_OFFSET_BITS 64
# endif
#endif

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>

#define LFS_FOPEN fopen
#if !defined(DIVSUFSORT_NO_LFS) && defined(_WIN32)
# define LFS_OFF_T int64_t
# define LFS_FTELL _ftelli64
# define LFS_FSEEK _fseeki64
# define LFS_PRId PRId64
#elif !defined(DIVSUFSORT_NO_LFS) && (defined(__unix__) || defined(__APPLE__))
# include <sys/types.h>
/* Reject an incompatible off_t instead of silently truncating large offsets. */
typedef char divsufsort_lfs_requires_64_bit_off_t[(sizeof(off_t) >= 8) ? 1 : -1];
# define LFS_OFF_T int64_t
# define LFS_FTELL ftello
# define LFS_FSEEK fseeko
# define LFS_PRId PRId64
#else
/* ISO C fallback; file offsets are limited by the host's long type. */
# define LFS_OFF_T long
# define LFS_FTELL ftell
# define LFS_FSEEK fseek
# define LFS_PRId "ld"
#endif
#ifndef PRIdOFF_T
# define PRIdOFF_T LFS_PRId
#endif

#endif /* DIVSUFSORT_LFS_H */
