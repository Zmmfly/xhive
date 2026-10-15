/* Checked-in C consumer for the external-buffer allocation mode built
 * with HEATSHRINK_USE_INDEX=0: performs one encode/decode roundtrip
 * through heatshrink_encoder_init()/heatshrink_decoder_init() and
 * returns non-zero on any mismatch. */

#include <stdio.h>
#include <string.h>

#include "heatshrink_decoder.h"
#include "heatshrink_encoder.h"

#define WINDOW_SZ2 8
#define LOOKAHEAD_SZ2 4
#define INPUT_BUFFER_SIZE 32

static heatshrink_encoder encoder;
static heatshrink_decoder decoder;
static uint8_t enc_window[HEATSHRINK_ENCODER_WINDOW_BUFFER_SIZE(WINDOW_SZ2)];
static uint8_t dec_input[INPUT_BUFFER_SIZE];
static uint8_t dec_window[HEATSHRINK_DECODER_WINDOW_BUFFER_SIZE(WINDOW_SZ2)];
static uint8_t compressed[2048];
static uint8_t decompressed[2048];

static const char input[] =
    "external buffer heatshrink roundtrip without index "
    "external buffer heatshrink roundtrip without index "
    "external buffer heatshrink roundtrip without index ";

/* Drain pending encoder output; fails if the output buffer is full. */
static int drain_encoder(size_t out_cap, size_t *out_len) {
    for (;;) {
        size_t polled = 0;
        HSE_poll_res pres;
        if (*out_len >= out_cap) { return -1; }
        pres = heatshrink_encoder_poll(&encoder, &compressed[*out_len],
            out_cap - *out_len, &polled);
        if (pres < 0) { return -1; }
        *out_len += polled;
        if (pres != HSER_POLL_MORE) { return 0; }
    }
}

/* Drain pending decoder output; fails if the output buffer is full. */
static int drain_decoder(size_t out_cap, size_t *out_len) {
    for (;;) {
        size_t polled = 0;
        HSD_poll_res pres;
        if (*out_len >= out_cap) { return -1; }
        pres = heatshrink_decoder_poll(&decoder, &decompressed[*out_len],
            out_cap - *out_len, &polled);
        if (pres < 0) { return -1; }
        *out_len += polled;
        if (pres != HSDR_POLL_MORE) { return 0; }
    }
}

static int encode_all(const char *in, size_t in_len, size_t out_cap,
        size_t *out_len) {
    size_t sunk = 0;
    *out_len = 0;
    while (sunk < in_len) {
        size_t sunk_now = 0;
        if (heatshrink_encoder_sink(&encoder, (uint8_t *)&in[sunk],
                in_len - sunk, &sunk_now) != HSER_SINK_OK) {
            return -1;
        }
        sunk += sunk_now;
        if (drain_encoder(out_cap, out_len) != 0) { return -1; }
    }
    for (;;) {
        HSE_finish_res fres = heatshrink_encoder_finish(&encoder);
        if (fres < 0) { return -1; }
        if (fres == HSER_FINISH_DONE) { return 0; }
        if (drain_encoder(out_cap, out_len) != 0) { return -1; }
    }
}

static int decode_all(const uint8_t *in, size_t in_len, size_t out_cap,
        size_t *out_len) {
    size_t sunk = 0;
    *out_len = 0;
    while (sunk < in_len) {
        size_t sunk_now = 0;
        HSD_sink_res sres = heatshrink_decoder_sink(&decoder,
            (uint8_t *)&in[sunk], in_len - sunk, &sunk_now);
        if (sres < 0) { return -1; }
        sunk += sunk_now;
        if (drain_decoder(out_cap, out_len) != 0) { return -1; }
    }
    for (;;) {
        HSD_finish_res fres = heatshrink_decoder_finish(&decoder);
        if (fres < 0) { return -1; }
        if (fres == HSDR_FINISH_DONE) { return 0; }
        if (drain_decoder(out_cap, out_len) != 0) { return -1; }
    }
}

int main(void) {
    const size_t in_len = sizeof(input) - 1;
    size_t clen = 0;
    size_t dlen = 0;

    if (heatshrink_encoder_init(&encoder, WINDOW_SZ2, LOOKAHEAD_SZ2,
            enc_window, sizeof(enc_window), NULL, 0) != HSER_INIT_OK) {
        fprintf(stderr, "encoder init failed\n");
        return 1;
    }
    if (heatshrink_decoder_init(&decoder, INPUT_BUFFER_SIZE, WINDOW_SZ2,
            LOOKAHEAD_SZ2, dec_input, dec_window,
            sizeof(dec_window)) != HSDR_INIT_OK) {
        fprintf(stderr, "decoder init failed\n");
        return 1;
    }
    if (encode_all(input, in_len, sizeof(compressed), &clen) != 0) {
        fprintf(stderr, "encode failed\n");
        return 2;
    }
    if (decode_all(compressed, clen, sizeof(decompressed), &dlen) != 0) {
        fprintf(stderr, "decode failed\n");
        return 3;
    }
    if (dlen != in_len || memcmp(decompressed, input, in_len) != 0) {
        fprintf(stderr, "roundtrip mismatch: %zu in, %zu compressed, %zu out\n",
            in_len, clen, dlen);
        return 4;
    }
    printf("noindex roundtrip ok: %zu -> %zu -> %zu bytes\n", in_len, clen, dlen);
    return 0;
}
