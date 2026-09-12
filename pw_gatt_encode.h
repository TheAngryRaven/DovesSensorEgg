#pragma once

///////////////////////////////////////////
// PW GATT ENCODERS
// Pure logic for the GATT-side wire formats (docs/PW_SENSOR_SERVICE.md,
// docs/PW_CHANNEL_SCHEMA.md), host-tested exactly like pw_adv_encode.
// No Arduino headers. The golden-byte tests pin every builder to the
// spec tables — and the DovesDataLogger repo's sensoregg_gatt decoder
// pins the SAME bytes from the other end (the cross-repo contract,
// exactly like pw_adv_encode <-> sensoregg_protocol).
//
// All multi-byte fields little-endian; f32 is IEEE-754 binary32 LE.
///////////////////////////////////////////

#include <stddef.h>
#include <stdint.h>

// Firmware version — single source of truth. The Descriptor carries the
// numerics; the DIS Firmware Revision string is derived so the two can
// never drift.
#define PW_FW_MAJOR 1
#define PW_FW_MINOR 1
#define PW_STR2(x) #x
#define PW_STR(x) PW_STR2(x)
#define PW_FW_REV PW_STR(PW_FW_MAJOR) "." PW_STR(PW_FW_MINOR)

namespace pw_gatt {

// Bluetooth SIG Temperature characteristic 0x2A6E: sint16 in
// centi-degC, 0x8000 = "value is not known". Its ceiling is 327.67 C —
// an EGT at 650 C cannot be represented, which is why EGT is never
// mirrored to ESS (docs/PW_SENSOR_SERVICE.md section 7): out-of-range
// maps to "not known", never to a clamped lie.
constexpr int16_t kEssUnknown = INT16_MIN;  // 0x8000 on the wire

int16_t encodeCentiC(float c);

// ---- PerchWerks Sensor Service builders (spec sections 4-6) -------------

// Channel schema quantities used by this pod (PW_CHANNEL_SCHEMA.md
// section 3; canonical units degC / percent — no unit byte exists).
constexpr uint8_t kQuantityTemperature = 0x01;
constexpr uint8_t kQuantityRatio = 0x08;

constexpr uint8_t kSchemaVersion = 1;
constexpr uint8_t kRecordLen = 24;          // schema v1 channel record
constexpr uint8_t kDeviceTypeEgtPod = 0x01; // PW_CHANNEL_SCHEMA.md section 4
constexpr size_t kDescHeaderLen = 8;
constexpr size_t kSampleHeaderLen = 10;
constexpr size_t kClockLen = 6;
constexpr uint8_t kMaxChannels = 21;        // 8 + 21*24 = 512 = ATT max

// Host-side form of a channel descriptor record (PW_CHANNEL_SCHEMA.md
// section 5). name is NUL-terminated here for convenience; it goes on
// the wire as char[8] NUL-PADDED (not necessarily terminated).
struct ChannelDef {
  uint8_t channelId;
  uint8_t quantity;
  uint16_t samplePeriodMs;
  float scale;
  float offset;
  char name[9];
};

// The EGT pod's channel table — the normative device-type 0x01 table
// from PW_CHANNEL_SCHEMA.md section 6.1, verbatim.
constexpr uint8_t kEgtPodChannelCount = 4;
constexpr size_t kEgtPodDescriptorLen =
    kDescHeaderLen + (size_t)kEgtPodChannelCount * kRecordLen;  // 104
extern const ChannelDef kEgtPodChannels[kEgtPodChannelCount];

// Descriptor characteristic value (spec section 4): 8-byte header +
// count packed records. Returns bytes written; 0 on null args, count
// outside 1..kMaxChannels, or outCap too small.
size_t buildDescriptor(uint8_t* out, size_t outCap, uint8_t deviceType,
                       uint8_t fwMajor, uint8_t fwMinor,
                       const ChannelDef* chans, uint8_t count);

// Clock characteristic value (spec section 6): boot_id, reserved 0,
// u32 LE pod millis.
void buildClock(uint8_t out[kClockLen], uint8_t bootId, uint32_t millisNow);

// Sample notify frame (spec section 5): 10-byte header + n s16 samples.
// Sample raws carry the 0x8000 invalid sentinel verbatim (produce them
// with pw_adv::encodeDeciC / the schema's own encoding — this builder
// never inspects values). Returns 10 + 2*n; 0 on null args, n == 0, or
// outCap too small.
size_t buildSampleFrame(uint8_t* out, size_t outCap, uint8_t channelId,
                        uint8_t bootId, uint8_t seq, uint32_t baseTimestampMs,
                        uint16_t intervalMs, const int16_t* samples, uint8_t n);

// Largest n that keeps a frame inside one notification: payload is
// ATT_MTU - 3, minus the 10-byte header, two bytes per sample —
// 23 -> 5, 247 -> 117 (the spec section 5 worked examples). Clamped to
// 255 (n is a u8); 0 when even one sample cannot fit.
uint8_t maxSamplesForMtu(uint16_t attMtu);

}  // namespace pw_gatt
