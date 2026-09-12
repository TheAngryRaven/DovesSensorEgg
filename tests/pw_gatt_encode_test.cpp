#include "doctest.h"

#include <cstdint>
#include <cstring>
#include <limits>

#include "pw_gatt_encode.h"

using namespace pw_gatt;

TEST_CASE("pw_gatt - centi-degC encoding for the ESS temperature mirror") {
    CHECK(encodeCentiC(23.40f) == 2340);
    CHECK(encodeCentiC(0.0f) == 0);
    CHECK(encodeCentiC(-10.55f) == -1055);
    CHECK(encodeCentiC(33.3f) == 3330);
}

TEST_CASE("pw_gatt - NaN becomes the ESS 'not known' value") {
    CHECK(encodeCentiC(std::numeric_limits<float>::quiet_NaN()) == kEssUnknown);
    CHECK((uint16_t)encodeCentiC(std::numeric_limits<float>::quiet_NaN()) ==
          0x8000);
}

TEST_CASE("pw_gatt - out-of-range reports 'not known', never a clamped lie") {
    // The reason EGT is never mirrored to ESS: 650 C does not fit sint16
    // centi-degC, and 327.67 C on a generic app's screen would be a lie.
    CHECK(encodeCentiC(650.0f) == kEssUnknown);
    CHECK(encodeCentiC(328.0f) == kEssUnknown);
    CHECK(encodeCentiC(-300.0f) == kEssUnknown);
    CHECK(encodeCentiC(std::numeric_limits<float>::infinity()) == kEssUnknown);
    CHECK(encodeCentiC(-std::numeric_limits<float>::infinity()) == kEssUnknown);
}

TEST_CASE("pw_gatt - representable boundaries encode exactly") {
    CHECK(encodeCentiC(327.67f) == 32767);
    CHECK(encodeCentiC(-273.15f) == -27315);
}

TEST_CASE("pw_gatt - rounds to nearest centi-degree") {
    CHECK(encodeCentiC(23.456f) == 2346);
    CHECK(encodeCentiC(23.454f) == 2345);
    CHECK(encodeCentiC(-0.004f) == 0);
}

// ---------------------------------------------------------------------------
// PerchWerks service builders (roadmap phases 2-3). These golden bytes are
// the wire contract with DovesDataLogger's sensoregg_gatt decoder — its
// fixtures are byte-identical. If either side changes a layout, both repos'
// vectors change with it, deliberately (the pw_adv_encode discipline).
// ---------------------------------------------------------------------------

// The EGT pod's 104-byte Descriptor value: 8-byte header (schema 1, device
// type 0x01, fw PW_FW_MAJOR.PW_FW_MINOR, 4 channels, record_len 24) + four
// 24-byte records per PW_CHANNEL_SCHEMA.md section 6.1.
// 250 = FA 00, 1000 = E8 03, 30000 = 30 75; 0.1f = CD CC CC 3D,
// 1.0f = 00 00 80 3F; names ASCII NUL-padded to 8.
static const uint8_t kGoldenDescriptor[104] = {
    // header
    0x01, 0x01, PW_FW_MAJOR, PW_FW_MINOR, 0x04, 0x18, 0x00, 0x00,
    // ch0 EGT: temperature, 250 ms, scale 0.1
    0x00, 0x01, 0xFA, 0x00, 0xCD, 0xCC, 0xCC, 0x3D, 0x00, 0x00, 0x00, 0x00,
    0x45, 0x47, 0x54, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // ch1 CJ: temperature, 250 ms, scale 0.1
    0x01, 0x01, 0xFA, 0x00, 0xCD, 0xCC, 0xCC, 0x3D, 0x00, 0x00, 0x00, 0x00,
    0x43, 0x4A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // ch2 IAT: temperature, 1000 ms, scale 0.1
    0x02, 0x01, 0xE8, 0x03, 0xCD, 0xCC, 0xCC, 0x3D, 0x00, 0x00, 0x00, 0x00,
    0x49, 0x41, 0x54, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // ch3 BATT: ratio, 30000 ms, scale 1.0
    0x03, 0x08, 0x30, 0x75, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
    0x42, 0x41, 0x54, 0x54, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

TEST_CASE("pw_gatt - EGT pod descriptor encodes byte-for-byte (wire contract)") {
    uint8_t out[kEgtPodDescriptorLen];
    const size_t len =
        buildDescriptor(out, sizeof(out), kDeviceTypeEgtPod, PW_FW_MAJOR,
                        PW_FW_MINOR, kEgtPodChannels, kEgtPodChannelCount);
    REQUIRE(len == sizeof(kGoldenDescriptor));
    for (size_t i = 0; i < len; i++) {
        CAPTURE(i);
        CHECK(out[i] == kGoldenDescriptor[i]);
    }
}

TEST_CASE("pw_gatt - descriptor guards refuse bad inputs") {
    uint8_t out[512];
    // One byte short of the EGT pod's 104.
    CHECK(buildDescriptor(out, kEgtPodDescriptorLen - 1, kDeviceTypeEgtPod, 1,
                          0, kEgtPodChannels, kEgtPodChannelCount) == 0);
    CHECK(buildDescriptor(out, sizeof(out), kDeviceTypeEgtPod, 1, 0,
                          kEgtPodChannels, 0) == 0);
    // 22 channels would exceed the 512-byte ATT attribute ceiling.
    CHECK(buildDescriptor(out, sizeof(out), kDeviceTypeEgtPod, 1, 0,
                          kEgtPodChannels, 22) == 0);
    CHECK(buildDescriptor(nullptr, sizeof(out), kDeviceTypeEgtPod, 1, 0,
                          kEgtPodChannels, 4) == 0);
    // The 21-channel maximum fills the attribute exactly: 8 + 21*24 = 512.
    ChannelDef many[kMaxChannels] = {};
    for (uint8_t i = 0; i < kMaxChannels; i++) {
        many[i] = kEgtPodChannels[0];
        many[i].channelId = i;
    }
    CHECK(buildDescriptor(out, sizeof(out), kDeviceTypeEgtPod, 1, 0, many,
                          kMaxChannels) == 512);
}

TEST_CASE("pw_gatt - clock value encodes boot_id + LE millis") {
    uint8_t out[kClockLen];
    buildClock(out, 0xA5, 0x01234567UL);
    const uint8_t golden[kClockLen] = {0xA5, 0x00, 0x67, 0x45, 0x23, 0x01};
    CHECK(memcmp(out, golden, sizeof(golden)) == 0);
    CHECK(out[1] == 0);  // reserved byte stays zero
}

TEST_CASE("pw_gatt - sample frame encodes byte-for-byte (wire contract)") {
    // ch 0, boot 0xA5, seq 0x2A, base 0xDEADBEEF, interval 250 ms,
    // samples {1234, -40, sentinel}.
    const int16_t samples[3] = {1234, -40, INT16_MIN};
    uint8_t out[kSampleHeaderLen + 6];
    const size_t len = buildSampleFrame(out, sizeof(out), 0, 0xA5, 0x2A,
                                        0xDEADBEEFUL, 250, samples, 3);
    REQUIRE(len == 16);
    const uint8_t golden[16] = {0x00, 0xA5, 0x2A, 0xEF, 0xBE, 0xAD, 0xDE,
                                0xFA, 0x00, 0x03, 0xD2, 0x04, 0xD8, 0xFF,
                                0x00, 0x80};
    for (size_t i = 0; i < len; i++) {
        CAPTURE(i);
        CHECK(out[i] == golden[i]);
    }
}

TEST_CASE("pw_gatt - sample frame guards and wrap bytes") {
    uint8_t out[64];
    const int16_t one[1] = {0};
    CHECK(buildSampleFrame(out, sizeof(out), 0, 0, 0, 0, 250, one, 0) == 0);
    CHECK(buildSampleFrame(out, 11, 0, 0, 0, 0, 250, one, 1) == 0);  // needs 12
    CHECK(buildSampleFrame(nullptr, 64, 0, 0, 0, 0, 250, one, 1) == 0);
    // u32 base wrap is bytes-only — no arithmetic in the builder.
    CHECK(buildSampleFrame(out, sizeof(out), 0, 0, 0, 0xFFFFFFFFUL, 250, one,
                           1) == 12);
    CHECK(out[3] == 0xFF);
    CHECK(out[6] == 0xFF);
}

TEST_CASE("pw_gatt - notify sizing matches the spec's worked examples") {
    CHECK(maxSamplesForMtu(23) == 5);     // default ATT_MTU
    CHECK(maxSamplesForMtu(247) == 117);  // negotiated max
    CHECK(maxSamplesForMtu(15) == 1);     // smallest that fits one sample
    CHECK(maxSamplesForMtu(14) == 0);
    CHECK(maxSamplesForMtu(13) == 0);
    CHECK(maxSamplesForMtu(0) == 0);
    CHECK(maxSamplesForMtu(65535) == 255);  // clamped to the u8 n field
}
