#include "doctest.h"

#include <cmath>
#include <cstdint>
#include <limits>

#include "pw_adv_encode.h"

using namespace pw_adv;

// Golden PW-ADV-1 frame — the SAME bytes as the DovesDataLogger repo's
// sensoregg_protocol parser fixture (tests/sensoregg_protocol_test.cpp
// there). This is the wire contract: if either side changes the layout,
// BOTH golden tests must change with it — deliberately.
//
//   FF FF        company ID (SIG test/internal, inside the array)
//   50 57        magic 'P' 'W'
//   01           protocol version
//   00           flags
//   64 19        EGT 0x1964 = 6500 deci-degC = 650.0 C
//   4D 01        cold junction 0x014D = 333 deci-degC = 33.3 C
//   00           MCP9600 STATUS
//   FF           battery stub
//   2A 01        sequence 0x012A = 298
static const uint8_t kGoldenFrame[kPayloadLen] = {
    0xFF, 0xFF, 0x50, 0x57, 0x01, 0x00, 0x64, 0x19,
    0x4D, 0x01, 0x00, 0xFF, 0x2A, 0x01};

TEST_CASE("pw_adv - golden frame encodes byte-for-byte (logger wire contract)") {
    uint8_t out[kPayloadLen];
    buildPayload(out, 650.0f, 33.3f, 0x00, /*pairing=*/false, /*seq=*/298);
    for (size_t i = 0; i < kPayloadLen; i++) {
        CAPTURE(i);
        CHECK(out[i] == kGoldenFrame[i]);
    }
}

TEST_CASE("pw_adv - invalid readings emit the 0x8000 sentinel, never a cast") {
    CHECK(encodeDeciC(std::numeric_limits<float>::quiet_NaN()) == INT16_MIN);
    CHECK(encodeDeciC(-271.0f) == INT16_MIN);   // below absolute zero-ish floor
    CHECK(encodeDeciC(1401.0f) == INT16_MIN);   // above K-type ceiling
    // Boundaries themselves are valid.
    CHECK(encodeDeciC(-270.0f) == -2700);
    CHECK(encodeDeciC(1400.0f) == 14000);
}

TEST_CASE("pw_adv - deci-degC rounding and negatives") {
    CHECK(encodeDeciC(0.0f) == 0);
    CHECK(encodeDeciC(33.3f) == 333);
    CHECK(encodeDeciC(-12.5f) == -125);
    CHECK(encodeDeciC(650.04f) == 6500);   // rounds to nearest deci-degree
    CHECK(encodeDeciC(650.06f) == 6501);
}

TEST_CASE("pw_adv - negative temperature little-endian bytes") {
    uint8_t out[kPayloadLen];
    // EGT -12.5 C = -125 = 0xFF83 LE; matches the logger's negative fixture.
    buildPayload(out, -12.5f, -0.1f, 0x00, false, 0);
    CHECK(out[6] == 0x83);
    CHECK(out[7] == 0xFF);
    CHECK(out[8] == 0xFF);   // -1 deci-degC
    CHECK(out[9] == 0xFF);
}

TEST_CASE("pw_adv - flag bits: pairing and MCP input-range fault") {
    uint8_t out[kPayloadLen];
    buildPayload(out, 650.0f, 33.3f, 0x00, true, 0);
    CHECK(out[5] == kFlagPairing);

    buildPayload(out, 650.0f, 33.3f, kStatusInputRangeMask, false, 0);
    CHECK(out[5] == kFlagTcFault);
    CHECK(out[10] == kStatusInputRangeMask);   // raw STATUS passes through

    buildPayload(out, 650.0f, 33.3f, kStatusInputRangeMask, true, 0);
    CHECK(out[5] == (kFlagPairing | kFlagTcFault));
}

TEST_CASE("pw_adv - sequence little-endian, including the uint16 wrap values") {
    uint8_t out[kPayloadLen];
    buildPayload(out, 650.0f, 33.3f, 0x00, false, 0xFFFF);
    CHECK(out[12] == 0xFF);
    CHECK(out[13] == 0xFF);
    buildPayload(out, 650.0f, 33.3f, 0x00, false, 0);
    CHECK(out[12] == 0x00);
    CHECK(out[13] == 0x00);
}

TEST_CASE("pw_adv - c2f known points and NaN propagation") {
    CHECK(c2f(0.0f) == doctest::Approx(32.0f));
    CHECK(c2f(650.0f) == doctest::Approx(1202.0f));
    CHECK(c2f(-40.0f) == doctest::Approx(-40.0f));
    CHECK(std::isnan(c2f(std::numeric_limits<float>::quiet_NaN())));
}
