/*
 * C++ side of the divsufsort consumer fixture.
 *
 * It includes both public headers twice in reverse order (64-bit first, the
 * opposite of consumer.c), statically checks the return-type ABI of the entry
 * points, and exports reverse_header_check() with C linkage so the C consumer
 * links against it. That mixed-language link is part of what the fixture
 * verifies; a successful link and a zero exit code are the pass condition.
 *
 * Build requirements: C++11, warnings-as-errors. No GoogleTest is used.
 */

#include <divsufsort64.h>
#include <divsufsort.h>
#include <divsufsort64.h>
#include <divsufsort.h>

#include <type_traits>

/* Functions are only referenced inside unevaluated decltype operands, so no
 * call is emitted and no runtime precondition applies. Both index widths must
 * report saint_t status, and the 64-bit transform must return a 64-bit index,
 * regardless of include order or repetition. */
static_assert(std::is_same<decltype(divsufsort(nullptr, nullptr, 0)),
                            saint_t>::value,
              "divsufsort must return saint_t");
static_assert(std::is_same<decltype(divsufsort64(nullptr, nullptr, 0)),
                            saint_t>::value,
              "divsufsort64 must return saint_t");
static_assert(std::is_same<decltype(divbwt64(nullptr, nullptr, nullptr, 0)),
                            saidx64_t>::value,
              "divbwt64 must return a 64-bit index");

/* Exported with C linkage so consumer.c can call it; returns nonzero when any
 * public type width differs from the documented contract. */
extern "C" int reverse_header_check(void) {
    return sizeof(saint_t) != 4 || sizeof(saidx_t) != 4 || sizeof(saidx64_t) != 8;
}
