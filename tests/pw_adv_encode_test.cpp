#include "doctest.h"

#include <cmath>
#include <cstdint>
#include <limits>

#include "nan_bits.h"
#include "pw_adv_encode.h"

using namespace pw_adv;

// Golden PW-ADV-2 frame. The DovesDataLogger repo's sensoregg_protocol
// parser fixture must adopt these SAME bytes when its v2 round lands
// (as of this commit the logger still pins the 14-byte v1 frame and
// drops v2 — see the header note in pw_adv_encode.h). This is the wire
// contract: if either side changes the layout, BOTH golden tests must
// change with it — deliberately.
//
//   FF FF        company ID (SIG test/internal, inside the array)
//   50 57        magic 'P' 'W'
//   02           protocol version
//   00           flags
//   64 19        EGT 0x1964 = 6500 deci-degC = 650.0 C
//   4D 01        cold junction 0x014D = 333 deci-degC = 33.3 C
//   00           MCP9600 STATUS
//   57           battery 87%
//   2A 01        sequence 0x012A = 298
//   EA 00        thermistor 0x00EA = 234 deci-degC = 23.4 C
static const uint8_t kGoldenFrame[kPayloadLen] = {
    0xFF, 0xFF, 0x50, 0x57, 0x02, 0x00, 0x64, 0x19,
    0x4D, 0x01, 0x00, 0x57, 0x2A, 0x01, 0xEA, 0x00};

TEST_CASE("pw_adv - golden frame encodes byte-for-byte (logger wire contract)") {
    uint8_t out[kPayloadLen];
    buildPayload(out, 650.0f, 33.3f, 0x00, /*pairing=*/false, /*seq=*/298,
                 /*batteryPct=*/87, /*thermC=*/23.4f);
    for (size_t i = 0; i < kPayloadLen; i++) {
        CAPTURE(i);
        CHECK(out[i] == kGoldenFrame[i]);
    }
}

TEST_CASE("pw_adv - unknown battery and invalid thermistor wire bytes") {
    uint8_t out[kPayloadLen];
    buildPayload(out, 650.0f, 33.3f, 0x00, false, 0,
                 kBatteryUnknown, std::numeric_limits<float>::quiet_NaN());
    CHECK(out[11] == 0xFF);              // battery unknown
    CHECK(out[14] == 0x00);              // 0x8000 sentinel, little-endian
    CHECK(out[15] == 0x80);
}

TEST_CASE("nan_bits - bit-level NaN check (the -Ofast-proof one)") {
    CHECK(isNanF(std::numeric_limits<float>::quiet_NaN()));
    CHECK(isNanF(std::numeric_limits<float>::signaling_NaN()));
    CHECK(isNanF(-std::numeric_limits<float>::quiet_NaN()));
    CHECK(!isNanF(0.0f));
    CHECK(!isNanF(-0.0f));
    CHECK(!isNanF(650.0f));
    CHECK(!isNanF(std::numeric_limits<float>::infinity()));
    CHECK(!isNanF(-std::numeric_limits<float>::infinity()));
    CHECK(!isNanF(std::numeric_limits<float>::max()));
    CHECK(!isNanF(std::numeric_limits<float>::denorm_min()));
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
    buildPayload(out, -12.5f, -0.1f, 0x00, false, 0, 87, 23.4f);
    CHECK(out[6] == 0x83);
    CHECK(out[7] == 0xFF);
    CHECK(out[8] == 0xFF);   // -1 deci-degC
    CHECK(out[9] == 0xFF);
}

TEST_CASE("pw_adv - flag bits: pairing and MCP input-range fault") {
    uint8_t out[kPayloadLen];
    buildPayload(out, 650.0f, 33.3f, 0x00, true, 0, 87, 23.4f);
    CHECK(out[5] == kFlagPairing);

    buildPayload(out, 650.0f, 33.3f, kStatusInputRangeMask, false, 0, 87, 23.4f);
    CHECK(out[5] == kFlagTcFault);
    CHECK(out[10] == kStatusInputRangeMask);   // raw STATUS passes through

    buildPayload(out, 650.0f, 33.3f, kStatusInputRangeMask, true, 0, 87, 23.4f);
    CHECK(out[5] == (kFlagPairing | kFlagTcFault));
}

TEST_CASE("pw_adv - sequence little-endian, including the uint16 wrap values") {
    uint8_t out[kPayloadLen];
    buildPayload(out, 650.0f, 33.3f, 0x00, false, 0xFFFF, 87, 23.4f);
    CHECK(out[12] == 0xFF);
    CHECK(out[13] == 0xFF);
    buildPayload(out, 650.0f, 33.3f, 0x00, false, 0, 87, 23.4f);
    CHECK(out[12] == 0x00);
    CHECK(out[13] == 0x00);
}

TEST_CASE("pw_adv - c2f known points and NaN propagation") {
    CHECK(c2f(0.0f) == doctest::Approx(32.0f));
    CHECK(c2f(650.0f) == doctest::Approx(1202.0f));
    CHECK(c2f(-40.0f) == doctest::Approx(-40.0f));
    CHECK(std::isnan(c2f(std::numeric_limits<float>::quiet_NaN())));
}
