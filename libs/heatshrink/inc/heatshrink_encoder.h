#ifndef HEATSHRINK_ENCODER_H
#define HEATSHRINK_ENCODER_H

#include <stdint.h>
#include <stddef.h>
#include "heatshrink_common.h"
#include "heatshrink_config.h"

typedef enum {
    HSER_SINK_OK,               /* data sunk into input buffer */
    HSER_SINK_ERROR_NULL=-1,    /* NULL argument */
    HSER_SINK_ERROR_MISUSE=-2,  /* API misuse */
} HSE_sink_res;

typedef enum {
    HSER_POLL_EMPTY,            /* input exhausted */
    HSER_POLL_MORE,             /* poll again for more output  */
    HSER_POLL_ERROR_NULL=-1,    /* NULL argument */
    HSER_POLL_ERROR_MISUSE=-2,  /* API misuse */
} HSE_poll_res;

typedef enum {
    HSER_FINISH_DONE,           /* encoding is complete */
    HSER_FINISH_MORE,           /* more output remaining; use poll */
    HSER_FINISH_ERROR_NULL=-1,  /* NULL argument */
} HSE_finish_res;

typedef enum {
    HSER_INIT_OK,               /* encoder configured and reset */
    HSER_INIT_ERROR_NULL=-1,    /* NULL argument */
    HSER_INIT_ERROR_MISUSE=-2,  /* invalid size or undersized buffer */
} HSE_init_res;

#if HEATSHRINK_DYNAMIC_ALLOC || HEATSHRINK_EXTERNAL_BUFFERS
#define HEATSHRINK_ENCODER_WINDOW_BITS(HSE) \
    ((HSE)->window_sz2)
#define HEATSHRINK_ENCODER_LOOKAHEAD_BITS(HSE) \
    ((HSE)->lookahead_sz2)
#define HEATSHRINK_ENCODER_INDEX(HSE) \
    ((HSE)->search_index)
struct hs_index {
    uint16_t size;
    int16_t index[];
};
#else
#define HEATSHRINK_ENCODER_WINDOW_BITS(_) \
    (HEATSHRINK_STATIC_WINDOW_BITS)
#define HEATSHRINK_ENCODER_LOOKAHEAD_BITS(_) \
    (HEATSHRINK_STATIC_LOOKAHEAD_BITS)
#define HEATSHRINK_ENCODER_INDEX(HSE) \
    (&(HSE)->search_index)
struct hs_index {
    uint16_t size;
    int16_t index[2 << HEATSHRINK_STATIC_WINDOW_BITS];
};
#endif

typedef struct {
    uint16_t input_size;        /* bytes in input buffer */
    uint16_t match_scan_index;
    uint16_t match_length;
    uint16_t match_pos;
    uint16_t outgoing_bits;     /* enqueued outgoing bits */
    uint8_t outgoing_bits_count;
    uint8_t flags;
    uint8_t state;              /* current state machine node */
    uint8_t current_byte;       /* current byte of output */
    uint8_t bit_index;          /* current bit index */
#if HEATSHRINK_DYNAMIC_ALLOC
    uint8_t window_sz2;         /* 2^n size of window */
    uint8_t lookahead_sz2;      /* 2^n size of lookahead */
#if HEATSHRINK_USE_INDEX
    struct hs_index *search_index;
#endif
    /* input buffer and / sliding window for expansion */
    uint8_t buffer[];
#elif HEATSHRINK_EXTERNAL_BUFFERS
    uint8_t window_sz2;         /* 2^n size of window */
    uint8_t lookahead_sz2;      /* 2^n size of lookahead */
#if HEATSHRINK_USE_INDEX
    struct hs_index *search_index;
#endif
    /* caller-provided input buffer and / sliding window,
     * bound via heatshrink_encoder_init() */
    uint8_t *buffer;
#else
    #if HEATSHRINK_USE_INDEX
        struct hs_index search_index;
    #endif
    /* input buffer and / sliding window for expansion */
    uint8_t buffer[2 << HEATSHRINK_ENCODER_WINDOW_BITS(_)];
#endif
} heatshrink_encoder;

#if HEATSHRINK_DYNAMIC_ALLOC
/* Allocate a new encoder struct and its buffers.
 * Returns NULL on error. */
heatshrink_encoder *heatshrink_encoder_alloc(uint8_t window_sz2,
    uint8_t lookahead_sz2);

/* Free an encoder. */
void heatshrink_encoder_free(heatshrink_encoder *hse);
#elif HEATSHRINK_EXTERNAL_BUFFERS
/* Window buffer capacity required by an encoder with the given window
 * size in bits: 2 * 2^WINDOW_SZ2 bytes (current input plus the previous
 * buffer scanned for backreferences). */
#define HEATSHRINK_ENCODER_WINDOW_BUFFER_SIZE(WINDOW_SZ2) \
    ((size_t)2U << (WINDOW_SZ2))

/* Search index buffer capacity required by an encoder with the given
 * window size in bits, when HEATSHRINK_USE_INDEX is enabled. */
#define HEATSHRINK_ENCODER_INDEX_BUFFER_SIZE(WINDOW_SZ2) \
    (sizeof(struct hs_index) + \
        (HEATSHRINK_ENCODER_WINDOW_BUFFER_SIZE(WINDOW_SZ2) * \
            sizeof(int16_t)))

/**
 * @brief Initialize an encoder with caller-provided buffers.
 *
 * Configures an encoder instance whose state is allocated by the
 * caller, binding it to externally owned buffers. The window and
 * lookahead sizes are runtime selections; the caller must provide
 * buffers large enough for the requested sizes.
 *
 * @param[in,out] hse Encoder instance to configure. Must not be NULL.
 * @param[in] window_sz2 Window size in bits; the window covers
 *                      2^window_sz2 bytes of input. In range
 *                      [HEATSHRINK_MIN_WINDOW_BITS,
 *                      HEATSHRINK_MAX_WINDOW_BITS].
 * @param[in] lookahead_sz2 Back-reference length field size in bits,
 *                          encoding match lengths up to 2^lookahead_sz2
 *                          bytes. In range [HEATSHRINK_MIN_LOOKAHEAD_BITS,
 *                          window_sz2), i.e. strictly less than
 *                          window_sz2.
 * @param[in] window_buffer Caller-provided window buffer with capacity
 *                         for at least
 *                         HEATSHRINK_ENCODER_WINDOW_BUFFER_SIZE(window_sz2)
 *                         bytes. Must not be NULL. Its lifetime must
 *                         cover all uses of the encoder until
 *                         re-initialization.
 * @param[in] window_buffer_size Capacity of window_buffer in bytes.
 * @param[in] index_buffer Caller-provided search index buffer with
 *                         capacity for at least
 *                         HEATSHRINK_ENCODER_INDEX_BUFFER_SIZE(window_sz2)
 *                         bytes, aligned to at least 2 bytes. Ignored
 *                         (may be NULL) when HEATSHRINK_USE_INDEX is 0.
 *                         Its lifetime must cover all uses of the
 *                         encoder until re-initialization.
 * @param[in] index_buffer_size Capacity of index_buffer in bytes.
 * @return HSE_init_res Initialization status.
 * @retval HSER_INIT_OK The encoder is configured and reset for use.
 * @retval HSER_INIT_ERROR_NULL hse is NULL.
 * @retval HSER_INIT_ERROR_MISUSE A size parameter is out of range, a
 *                                required buffer is NULL, or a buffer
 *                                capacity is too small.
 *
 * @pre The instance must not be in use; call this before any other
 *      encoder function on the instance.
 * @note The window and lookahead sizes must match the decoder used to
 *       decompress the resulting stream.
 * @note The buffer contents need not be initialized by the caller; the
 *       encoder clears the window buffer and rebuilds the index
 *       contents itself.
 */
HSE_init_res heatshrink_encoder_init(heatshrink_encoder *hse,
    uint8_t window_sz2, uint8_t lookahead_sz2,
    uint8_t *window_buffer, size_t window_buffer_size,
    void *index_buffer, size_t index_buffer_size);
#endif

/* Reset an encoder. */
void heatshrink_encoder_reset(heatshrink_encoder *hse);

/* Sink up to SIZE bytes from IN_BUF into the encoder.
 * INPUT_SIZE is set to the number of bytes actually sunk (in case a
 * buffer was filled.). */
HSE_sink_res heatshrink_encoder_sink(heatshrink_encoder *hse,
    uint8_t *in_buf, size_t size, size_t *input_size);

/* Poll for output from the encoder, copying at most OUT_BUF_SIZE bytes into
 * OUT_BUF (setting *OUTPUT_SIZE to the actual amount copied). */
HSE_poll_res heatshrink_encoder_poll(heatshrink_encoder *hse,
    uint8_t *out_buf, size_t out_buf_size, size_t *output_size);

/* Notify the encoder that the input stream is finished.
 * If the return value is HSER_FINISH_MORE, there is still more output, so
 * call heatshrink_encoder_poll and repeat. */
HSE_finish_res heatshrink_encoder_finish(heatshrink_encoder *hse);

#endif
