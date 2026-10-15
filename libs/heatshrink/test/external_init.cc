/* Host tests for the external-buffer allocation mode: runtime-selected
 * window and lookahead sizes bound to caller-provided buffers via
 * heatshrink_encoder_init() / heatshrink_decoder_init(). The
 * heatshrink_external target builds the library with
 * HEATSHRINK_DYNAMIC_ALLOC=0 and HEATSHRINK_EXTERNAL_BUFFERS=1. */

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

extern "C" {
#include "heatshrink_common.h"
#include "heatshrink_decoder.h"
#include "heatshrink_encoder.h"
}

namespace {

/* Largest window exercised by these tests. */
constexpr uint8_t kMaxWindowBits = 10;

/* Buffers owned by the test, sized for kMaxWindowBits so smaller runtime
 * configurations can reuse the same storage. The index buffer is a
 * uint16_t array to guarantee the required 2-byte alignment. */
struct ExternalStorage {
    uint8_t encoder_window[HEATSHRINK_ENCODER_WINDOW_BUFFER_SIZE(kMaxWindowBits)];
    uint16_t encoder_index[HEATSHRINK_ENCODER_INDEX_BUFFER_SIZE(kMaxWindowBits)
        / sizeof(uint16_t)];
    uint8_t decoder_input[64];
    uint8_t decoder_window[HEATSHRINK_DECODER_WINDOW_BUFFER_SIZE(kMaxWindowBits)];
};

/* Payload mixing repetitive runs with pseudo-random noise so every
 * configuration emits both back-references and literals. */
std::vector<uint8_t> MakePayload(size_t size) {
    static const char kPattern[] = "heatshrink external init roundtrip payload";
    const size_t pattern_len = sizeof(kPattern) - 1;
    std::vector<uint8_t> data;
    data.reserve(size);
    uint32_t x = 0x12345678u;
    while (data.size() < size) {
        for (size_t i = 0; i < 29 && data.size() < size; ++i) {
            data.push_back(static_cast<uint8_t>(kPattern[data.size() % pattern_len]));
        }
        for (size_t i = 0; i < 13 && data.size() < size; ++i) {
            x = x * 1664525u + 1013904223u;
            data.push_back(static_cast<uint8_t>(x >> 24));
        }
    }
    return data;
}

/* Fully repetitive payload for compression-ratio comparison. */
std::vector<uint8_t> MakeRepetitivePayload(size_t size) {
    static const char kPattern[] = "heatshrink external init roundtrip payload";
    const size_t pattern_len = sizeof(kPattern) - 1;
    std::vector<uint8_t> data;
    data.reserve(size);
    for (size_t i = 0; i < size; ++i) {
        data.push_back(static_cast<uint8_t>(kPattern[i % pattern_len]));
    }
    return data;
}

/* Sink the whole input in small chunks, polling in between, then finish
 * and drain the output. Small scratch buffers exercise the POLL_MORE
 * and backlog paths. */
bool EncodeAll(heatshrink_encoder *hse, const std::vector<uint8_t> &input,
               std::vector<uint8_t> *output) {
    output->clear();
    uint8_t scratch[23];
    size_t sunk = 0;
    while (sunk < input.size()) {
        size_t sunk_now = 0;
        const size_t chunk = std::min<size_t>(37, input.size() - sunk);
        if (heatshrink_encoder_sink(hse, const_cast<uint8_t *>(&input[sunk]),
                chunk, &sunk_now) != HSER_SINK_OK) {
            return false;
        }
        sunk += sunk_now;
        HSE_poll_res pres;
        do {
            size_t polled = 0;
            pres = heatshrink_encoder_poll(hse, scratch, sizeof(scratch), &polled);
            if (pres < 0) { return false; }
            output->insert(output->end(), scratch, scratch + polled);
        } while (pres == HSER_POLL_MORE);
    }
    for (;;) {
        const HSE_finish_res fres = heatshrink_encoder_finish(hse);
        if (fres < 0) { return false; }
        if (fres == HSER_FINISH_DONE) { return true; }
        HSE_poll_res pres;
        do {
            size_t polled = 0;
            pres = heatshrink_encoder_poll(hse, scratch, sizeof(scratch), &polled);
            if (pres < 0) { return false; }
            output->insert(output->end(), scratch, scratch + polled);
        } while (pres == HSER_POLL_MORE);
    }
}

bool DecodeAll(heatshrink_decoder *hsd, const std::vector<uint8_t> &input,
               std::vector<uint8_t> *output) {
    output->clear();
    uint8_t scratch[31];
    size_t sunk = 0;
    while (sunk < input.size()) {
        size_t sunk_now = 0;
        const size_t chunk = std::min<size_t>(29, input.size() - sunk);
        const HSD_sink_res sres = heatshrink_decoder_sink(hsd,
            const_cast<uint8_t *>(&input[sunk]), chunk, &sunk_now);
        if (sres < 0) { return false; }
        sunk += sunk_now;
        HSD_poll_res pres;
        do {
            size_t polled = 0;
            pres = heatshrink_decoder_poll(hsd, scratch, sizeof(scratch), &polled);
            if (pres < 0) { return false; }
            output->insert(output->end(), scratch, scratch + polled);
        } while (pres == HSDR_POLL_MORE);
    }
    for (;;) {
        const HSD_finish_res fres = heatshrink_decoder_finish(hsd);
        if (fres < 0) { return false; }
        if (fres == HSDR_FINISH_DONE) { return true; }
        HSD_poll_res pres;
        do {
            size_t polled = 0;
            pres = heatshrink_decoder_poll(hsd, scratch, sizeof(scratch), &polled);
            if (pres < 0) { return false; }
            output->insert(output->end(), scratch, scratch + polled);
        } while (pres == HSDR_POLL_MORE);
    }
}

}  // namespace

TEST(HeatshrinkExternalInit, RejectsInvalidParameters) {
    ExternalStorage st;
    heatshrink_encoder hse;
    heatshrink_decoder hsd;

    /* Valid baseline variants for the encoder. */
    auto init_enc = [&](uint8_t w, uint8_t l, uint8_t *wb, size_t wbs,
                        void *ib, size_t ibs) {
        return heatshrink_encoder_init(&hse, w, l, wb, wbs, ib, ibs);
    };
    const size_t enc_win_sz = sizeof(st.encoder_window);
    const size_t enc_idx_sz = sizeof(st.encoder_index);

    EXPECT_EQ(HSER_INIT_ERROR_NULL,
        heatshrink_encoder_init(nullptr, 8, 4, st.encoder_window, enc_win_sz,
            st.encoder_index, enc_idx_sz));
    EXPECT_EQ(HSER_INIT_ERROR_MISUSE,
        init_enc(HEATSHRINK_MIN_WINDOW_BITS - 1, 3, st.encoder_window,
            enc_win_sz, st.encoder_index, enc_idx_sz));
    EXPECT_EQ(HSER_INIT_ERROR_MISUSE,
        init_enc(HEATSHRINK_MAX_WINDOW_BITS + 1, 3, st.encoder_window,
            enc_win_sz, st.encoder_index, enc_idx_sz));
    EXPECT_EQ(HSER_INIT_ERROR_MISUSE,
        init_enc(8, HEATSHRINK_MIN_LOOKAHEAD_BITS - 1, st.encoder_window,
            enc_win_sz, st.encoder_index, enc_idx_sz));
    EXPECT_EQ(HSER_INIT_ERROR_MISUSE,
        init_enc(4, 4, st.encoder_window, enc_win_sz, st.encoder_index,
            enc_idx_sz));
    EXPECT_EQ(HSER_INIT_ERROR_MISUSE,
        init_enc(8, 4, nullptr, enc_win_sz, st.encoder_index, enc_idx_sz));
    EXPECT_EQ(HSER_INIT_ERROR_MISUSE,
        init_enc(8, 4, st.encoder_window,
            HEATSHRINK_ENCODER_WINDOW_BUFFER_SIZE(8) - 1, st.encoder_index,
            enc_idx_sz));
    EXPECT_EQ(HSER_INIT_ERROR_MISUSE,
        init_enc(8, 4, st.encoder_window, enc_win_sz, nullptr, enc_idx_sz));
    EXPECT_EQ(HSER_INIT_ERROR_MISUSE,
        init_enc(8, 4, st.encoder_window, enc_win_sz, st.encoder_index,
            HEATSHRINK_ENCODER_INDEX_BUFFER_SIZE(8) - 1));
    EXPECT_EQ(HSER_INIT_OK,
        init_enc(8, 4, st.encoder_window, enc_win_sz, st.encoder_index,
            enc_idx_sz));

    /* Valid baseline variants for the decoder. */
    auto init_dec = [&](uint16_t ibs, uint8_t w, uint8_t l, uint8_t *in,
                        uint8_t *win, size_t winsz) {
        return heatshrink_decoder_init(&hsd, ibs, w, l, in, win, winsz);
    };
    const size_t dec_win_sz = sizeof(st.decoder_window);

    EXPECT_EQ(HSDR_INIT_ERROR_NULL,
        heatshrink_decoder_init(nullptr, 32, 8, 4, st.decoder_input,
            st.decoder_window, dec_win_sz));
    EXPECT_EQ(HSDR_INIT_ERROR_MISUSE,
        init_dec(0, 8, 4, st.decoder_input, st.decoder_window, dec_win_sz));
    EXPECT_EQ(HSDR_INIT_ERROR_MISUSE,
        init_dec(32, HEATSHRINK_MIN_WINDOW_BITS - 1, 3, st.decoder_input,
            st.decoder_window, dec_win_sz));
    EXPECT_EQ(HSDR_INIT_ERROR_MISUSE,
        init_dec(32, HEATSHRINK_MAX_WINDOW_BITS + 1, 3, st.decoder_input,
            st.decoder_window, dec_win_sz));
    EXPECT_EQ(HSDR_INIT_ERROR_MISUSE,
        init_dec(32, 8, HEATSHRINK_MIN_LOOKAHEAD_BITS - 1, st.decoder_input,
            st.decoder_window, dec_win_sz));
    EXPECT_EQ(HSDR_INIT_ERROR_MISUSE,
        init_dec(32, 4, 4, st.decoder_input, st.decoder_window, dec_win_sz));
    EXPECT_EQ(HSDR_INIT_ERROR_MISUSE,
        init_dec(32, 8, 4, nullptr, st.decoder_window, dec_win_sz));
    EXPECT_EQ(HSDR_INIT_ERROR_MISUSE,
        init_dec(32, 8, 4, st.decoder_input, nullptr, dec_win_sz));
    EXPECT_EQ(HSDR_INIT_ERROR_MISUSE,
        init_dec(32, 8, 4, st.decoder_input, st.decoder_window,
            HEATSHRINK_DECODER_WINDOW_BUFFER_SIZE(8) - 1));
    EXPECT_EQ(HSDR_INIT_OK,
        init_dec(32, 8, 4, st.decoder_input, st.decoder_window, dec_win_sz));
}

TEST(HeatshrinkExternalInit, RoundtripsWithRuntimeSelectedSizes) {
    struct Config {
        uint8_t window;
        uint8_t lookahead;
        uint16_t ibs;
    };
    /* Includes windows > 8 to exercise the decoder backref-index MSB
     * path with a runtime-selected window size. */
    const Config kConfigs[] = {
        {4, 3, 1}, {5, 3, 32}, {8, 4, 64}, {9, 3, 32}, {10, 4, 64}, {10, 3, 7},
    };
    const std::vector<uint8_t> payload = MakePayload(2048);
    ExternalStorage st;
    heatshrink_encoder hse;
    heatshrink_decoder hsd;

    for (const auto &cfg : kConfigs) {
        ASSERT_EQ(HSER_INIT_OK,
            heatshrink_encoder_init(&hse, cfg.window, cfg.lookahead,
                st.encoder_window, sizeof(st.encoder_window),
                st.encoder_index, sizeof(st.encoder_index)))
            << "encoder window=" << int(cfg.window);
        std::vector<uint8_t> compressed;
        ASSERT_TRUE(EncodeAll(&hse, payload, &compressed));

        ASSERT_EQ(HSDR_INIT_OK,
            heatshrink_decoder_init(&hsd, cfg.ibs, cfg.window, cfg.lookahead,
                st.decoder_input, st.decoder_window,
                sizeof(st.decoder_window)))
            << "decoder window=" << int(cfg.window);
        std::vector<uint8_t> decompressed;
        ASSERT_TRUE(DecodeAll(&hsd, compressed, &decompressed));
        EXPECT_EQ(payload, decompressed)
            << "window=" << int(cfg.window)
            << " lookahead=" << int(cfg.lookahead)
            << " ibs=" << cfg.ibs;
    }
}

TEST(HeatshrinkExternalInit, ReinitializesInstanceAndBuffers) {
    const std::vector<uint8_t> payload = MakePayload(777);
    ExternalStorage st;
    heatshrink_encoder hse;
    heatshrink_decoder hsd;
    std::vector<uint8_t> compressed;
    std::vector<uint8_t> decompressed;

    /* First pass with a small window. */
    ASSERT_EQ(HSER_INIT_OK,
        heatshrink_encoder_init(&hse, 4, 3, st.encoder_window,
            sizeof(st.encoder_window), st.encoder_index,
            sizeof(st.encoder_index)));
    ASSERT_TRUE(EncodeAll(&hse, payload, &compressed));
    ASSERT_EQ(HSDR_INIT_OK,
        heatshrink_decoder_init(&hsd, 5, 4, 3, st.decoder_input,
            st.decoder_window, sizeof(st.decoder_window)));
    ASSERT_TRUE(DecodeAll(&hsd, compressed, &decompressed));
    EXPECT_EQ(payload, decompressed);

    /* Re-initialize the same instances and buffers with different
     * runtime sizes. */
    ASSERT_EQ(HSER_INIT_OK,
        heatshrink_encoder_init(&hse, 10, 4, st.encoder_window,
            sizeof(st.encoder_window), st.encoder_index,
            sizeof(st.encoder_index)));
    ASSERT_TRUE(EncodeAll(&hse, payload, &compressed));
    ASSERT_EQ(HSDR_INIT_OK,
        heatshrink_decoder_init(&hsd, 64, 10, 4, st.decoder_input,
            st.decoder_window, sizeof(st.decoder_window)));
    ASSERT_TRUE(DecodeAll(&hsd, compressed, &decompressed));
    EXPECT_EQ(payload, decompressed);
}

TEST(HeatshrinkExternalInit, WindowSizeAffectsCompression) {
    const std::vector<uint8_t> payload = MakeRepetitivePayload(2048);
    ExternalStorage st;
    heatshrink_encoder hse;
    std::vector<uint8_t> small;
    std::vector<uint8_t> large;

    ASSERT_EQ(HSER_INIT_OK,
        heatshrink_encoder_init(&hse, 4, 3, st.encoder_window,
            sizeof(st.encoder_window), st.encoder_index,
            sizeof(st.encoder_index)));
    ASSERT_TRUE(EncodeAll(&hse, payload, &small));

    ASSERT_EQ(HSER_INIT_OK,
        heatshrink_encoder_init(&hse, 10, 4, st.encoder_window,
            sizeof(st.encoder_window), st.encoder_index,
            sizeof(st.encoder_index)));
    ASSERT_TRUE(EncodeAll(&hse, payload, &large));

    /* A window larger than the repeating pattern compresses it much
     * better; this verifies the runtime window selection is effective. */
    EXPECT_LT(large.size(), small.size());
}
