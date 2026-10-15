#ifndef HEATSHRINK_DECODER_H
#define HEATSHRINK_DECODER_H

#include <stdint.h>
#include <stddef.h>
#include "heatshrink_common.h"
#include "heatshrink_config.h"

typedef enum {
    HSDR_SINK_OK,               /* data sunk, ready to poll */
    HSDR_SINK_FULL,             /* out of space in internal buffer */
    HSDR_SINK_ERROR_NULL=-1,    /* NULL argument */
} HSD_sink_res;

typedef enum {
    HSDR_POLL_EMPTY,            /* input exhausted */
    HSDR_POLL_MORE,             /* more data remaining, call again w/ fresh output buffer */
    HSDR_POLL_ERROR_NULL=-1,    /* NULL arguments */
    HSDR_POLL_ERROR_UNKNOWN=-2,
} HSD_poll_res;

typedef enum {
    HSDR_FINISH_DONE,           /* output is done */
    HSDR_FINISH_MORE,           /* more output remains */
    HSDR_FINISH_ERROR_NULL=-1,  /* NULL arguments */
} HSD_finish_res;

typedef enum {
    HSDR_INIT_OK,               /* decoder configured and reset */
    HSDR_INIT_ERROR_NULL=-1,    /* NULL argument */
    HSDR_INIT_ERROR_MISUSE=-2,  /* invalid size or undersized buffer */
} HSD_init_res;

#if HEATSHRINK_DYNAMIC_ALLOC || HEATSHRINK_EXTERNAL_BUFFERS
#define HEATSHRINK_DECODER_INPUT_BUFFER_SIZE(BUF) \
    ((BUF)->input_buffer_size)
#define HEATSHRINK_DECODER_WINDOW_BITS(BUF) \
    ((BUF)->window_sz2)
#define HEATSHRINK_DECODER_LOOKAHEAD_BITS(BUF) \
    ((BUF)->lookahead_sz2)
#else
#define HEATSHRINK_DECODER_INPUT_BUFFER_SIZE(_) \
    HEATSHRINK_STATIC_INPUT_BUFFER_SIZE
#define HEATSHRINK_DECODER_WINDOW_BITS(_) \
    (HEATSHRINK_STATIC_WINDOW_BITS)
#define HEATSHRINK_DECODER_LOOKAHEAD_BITS(BUF) \
    (HEATSHRINK_STATIC_LOOKAHEAD_BITS)
#endif

/* Direct access to the input and expansion window buffers. Dynamic and
 * static builds share one buffer where the window follows the input;
 * external-buffer builds keep two caller-provided pointers. */
#if HEATSHRINK_EXTERNAL_BUFFERS
#define HEATSHRINK_DECODER_INPUT_BUFFER(HSD) \
    ((HSD)->input_buffer)
#define HEATSHRINK_DECODER_WINDOW_BUFFER(HSD) \
    ((HSD)->window_buffer)
#else
#define HEATSHRINK_DECODER_INPUT_BUFFER(HSD) \
    ((HSD)->buffers)
#define HEATSHRINK_DECODER_WINDOW_BUFFER(HSD) \
    (&((HSD)->buffers[HEATSHRINK_DECODER_INPUT_BUFFER_SIZE(HSD)]))
#endif

typedef struct {
    uint16_t input_size;        /* bytes in input buffer */
    uint16_t input_index;       /* offset to next unprocessed input byte */
    uint16_t output_count;      /* how many bytes to output */
    uint16_t output_index;      /* index for bytes to output */
    uint16_t head_index;        /* head of window buffer */
    uint8_t state;              /* current state machine node */
    uint8_t current_byte;       /* current byte of input */
    uint8_t bit_index;          /* current bit index */

#if HEATSHRINK_DYNAMIC_ALLOC
    /* Fields that are only used if dynamically allocated. */
    uint8_t window_sz2;         /* window buffer bits */
    uint8_t lookahead_sz2;      /* lookahead bits */
    uint16_t input_buffer_size; /* input buffer size */

    /* Input buffer, then expansion window buffer */
    uint8_t buffers[];
#elif HEATSHRINK_EXTERNAL_BUFFERS
    uint8_t window_sz2;         /* window buffer bits */
    uint8_t lookahead_sz2;      /* lookahead bits */
    uint16_t input_buffer_size; /* input buffer size */

    /* Caller-provided input buffer and expansion window buffer,
     * bound via heatshrink_decoder_init() */
    uint8_t *input_buffer;
    uint8_t *window_buffer;
#else
    /* Input buffer, then expansion window buffer */
    uint8_t buffers[(1 << HEATSHRINK_DECODER_WINDOW_BITS(_))
        + HEATSHRINK_DECODER_INPUT_BUFFER_SIZE(_)];
#endif
} heatshrink_decoder;

#if HEATSHRINK_DYNAMIC_ALLOC
/* Allocate a decoder with an input buffer of INPUT_BUFFER_SIZE bytes,
 * an expansion buffer size of 2^WINDOW_SZ2, and a lookahead
 * size of 2^lookahead_sz2. (The window buffer and lookahead sizes
 * must match the settings used when the data was compressed.)
 * Returns NULL on error. */
heatshrink_decoder *heatshrink_decoder_alloc(uint16_t input_buffer_size,
    uint8_t expansion_buffer_sz2, uint8_t lookahead_sz2);

/* Free a decoder. */
void heatshrink_decoder_free(heatshrink_decoder *hsd);
#elif HEATSHRINK_EXTERNAL_BUFFERS
/* Expansion window buffer capacity required by a decoder with the given
 * window size in bits: 2^WINDOW_SZ2 bytes. */
#define HEATSHRINK_DECODER_WINDOW_BUFFER_SIZE(WINDOW_SZ2) \
    ((size_t)1U << (WINDOW_SZ2))

/**
 * @brief Initialize a decoder with caller-provided buffers.
 *
 * The caller owns the instance and all buffers; the input buffer size
 * and the window and lookahead sizes are selected at runtime.
 *
 * @param[in,out] hsd Decoder instance to configure. Must not be NULL.
 * @param[in] input_buffer_size Input buffer capacity to use, in bytes.
 *                             Range [1, 65535].
 * @param[in] window_sz2 Window size in bits; the window covers
 *                      2^window_sz2 bytes. In range
 *                      [HEATSHRINK_MIN_WINDOW_BITS,
 *                      HEATSHRINK_MAX_WINDOW_BITS].
 * @param[in] lookahead_sz2 Back-reference length field size in bits,
 *                          decoding match lengths up to 2^lookahead_sz2
 *                          bytes. In range [HEATSHRINK_MIN_LOOKAHEAD_BITS,
 *                          window_sz2).
 * @param[in] input_buffer Input buffer with capacity for at least
 *                         input_buffer_size bytes. Must not be NULL and
 *                         must outlive the decoder until
 *                         re-initialization.
 * @param[in] window_buffer Expansion window buffer with capacity for at
 *                         least
 *                         HEATSHRINK_DECODER_WINDOW_BUFFER_SIZE(window_sz2)
 *                         bytes. Must not be NULL.
 * @param[in] window_buffer_size Capacity of window_buffer in bytes.
 * @return HSD_init_res Initialization status.
 * @retval HSDR_INIT_OK The decoder is configured and reset for use.
 * @retval HSDR_INIT_ERROR_NULL hsd is NULL.
 * @retval HSDR_INIT_ERROR_MISUSE A size parameter is out of range, a
 *                                required buffer is NULL, or
 *                                window_buffer_size is too small.
 *
 * @pre The instance must not be in use; call this before any other
 *      decoder function on the instance.
 * @note The window and lookahead sizes must match the settings used to
 *       compress the data.
 * @note The buffer contents need not be initialized by the caller.
 */
HSD_init_res heatshrink_decoder_init(heatshrink_decoder *hsd,
    uint16_t input_buffer_size, uint8_t window_sz2,
    uint8_t lookahead_sz2, uint8_t *input_buffer,
    uint8_t *window_buffer, size_t window_buffer_size);
#endif

/* Reset a decoder. */
void heatshrink_decoder_reset(heatshrink_decoder *hsd);

/* Sink at most SIZE bytes from IN_BUF into the decoder. *INPUT_SIZE is set to
 * indicate how many bytes were actually sunk (in case a buffer was filled). */
HSD_sink_res heatshrink_decoder_sink(heatshrink_decoder *hsd,
    uint8_t *in_buf, size_t size, size_t *input_size);

/* Poll for output from the decoder, copying at most OUT_BUF_SIZE bytes into
 * OUT_BUF (setting *OUTPUT_SIZE to the actual amount copied). */
HSD_poll_res heatshrink_decoder_poll(heatshrink_decoder *hsd,
    uint8_t *out_buf, size_t out_buf_size, size_t *output_size);

/* Notify the decoder that the input stream is finished.
 * If the return value is HSDR_FINISH_MORE, there is still more output, so
 * call heatshrink_decoder_poll and repeat. */
HSD_finish_res heatshrink_decoder_finish(heatshrink_decoder *hsd);

#endif
