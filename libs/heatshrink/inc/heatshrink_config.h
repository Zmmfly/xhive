#ifndef HEATSHRINK_CONFIG_H
#define HEATSHRINK_CONFIG_H

#ifdef BUILD_IN_XHIVE
#include "heatshrink_kconfig.h"
#endif

/* Should functionality assuming dynamic allocation be used? */
#ifndef HEATSHRINK_DYNAMIC_ALLOC
#define HEATSHRINK_DYNAMIC_ALLOC 1
#endif

/* Should caller-provided buffers be used instead? This third mode is
 * mutually exclusive with dynamic allocation: set
 * HEATSHRINK_DYNAMIC_ALLOC to 0 and this to 1, then configure
 * instances with heatshrink_encoder_init()/heatshrink_decoder_init()
 * and caller-owned buffers. */
#ifndef HEATSHRINK_EXTERNAL_BUFFERS
#define HEATSHRINK_EXTERNAL_BUFFERS 0
#endif

#if HEATSHRINK_DYNAMIC_ALLOC
    /* Optional replacement of malloc/free */
    #define HEATSHRINK_MALLOC(SZ) malloc(SZ)
    #define HEATSHRINK_FREE(P, SZ) free(P)
#else
    /* Required parameters for static configuration */
    #ifndef HEATSHRINK_STATIC_INPUT_BUFFER_SIZE
    #define HEATSHRINK_STATIC_INPUT_BUFFER_SIZE 32
    #endif

    #ifndef HEATSHRINK_STATIC_WINDOW_BITS
    #define HEATSHRINK_STATIC_WINDOW_BITS 8
    #endif

    #ifndef HEATSHRINK_STATIC_LOOKAHEAD_BITS
    #define HEATSHRINK_STATIC_LOOKAHEAD_BITS 4
    #endif
#endif

/* Turn on logging for debugging. */
#ifndef HEATSHRINK_DEBUGGING_LOGS
#define HEATSHRINK_DEBUGGING_LOGS 0
#endif

/* Use indexing for faster compression. (This requires additional space.) */
#ifndef HEATSHRINK_USE_INDEX
#define HEATSHRINK_USE_INDEX 1
#endif

#endif
