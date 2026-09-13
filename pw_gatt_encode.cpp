#include "pw_gatt_encode.h"

#include <math.h>
#include <string.h>

#include "nan_bits.h"

namespace pw_gatt {

int16_t encodeCentiC(float c) {
  // isNanF, not isnan: the device build is -Ofast, where isnan() folds
  // to false and a NaN would fall through into lroundf -> garbage on
  // the wire instead of the "not known" value. The NaN check must come
  // FIRST - the range comparisons are only meaningful once c is known
  // to be a real number. Range: sint16 centi-degC representable span.
  if (isNanF(c) || c < -273.15f || c > 327.67f) return kEssUnknown;
  return (int16_t)lroundf(c * 100.0f);
}

// PW_CHANNEL_SCHEMA.md section 6.1 — the normative EGT-pod table.
// Deci-degC via scale 0.1 keeps the beacon's encoding idiom; BATT is
// whole percent. The 104-byte golden test pins the packed form.
const ChannelDef kEgtPodChannels[kEgtPodChannelCount] = {
    {0, kQuantityTemperature, 250, 0.1f, 0.0f, "EGT"},
    {1, kQuantityTemperature, 250, 0.1f, 0.0f, "CJ"},
    {2, kQuantityTemperature, 1000, 0.1f, 0.0f, "IAT"},
    {3, kQuantityRatio, 30000, 1.0f, 0.0f, "BATT"},
};

namespace {

inline void putU16(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)(v >> 8);
}

inline void putU32(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
  p[2] = (uint8_t)((v >> 16) & 0xFF);
  p[3] = (uint8_t)((v >> 24) & 0xFF);
}

}  // namespace

size_t buildDescriptor(uint8_t* out, size_t outCap, uint8_t deviceType,
                       uint8_t fwMajor, uint8_t fwMinor,
                       const ChannelDef* chans, uint8_t count) {
  if (out == nullptr || chans == nullptr || count == 0 ||
      count > kMaxChannels) {
    return 0;
  }
  const size_t need = kDescHeaderLen + (size_t)count * kRecordLen;
  if (outCap < need) return 0;

  out[0] = kSchemaVersion;
  out[1] = deviceType;
  out[2] = fwMajor;
  out[3] = fwMinor;
  out[4] = count;
  out[5] = kRecordLen;
  out[6] = 0;  // reserved
  out[7] = 0;

  size_t off = kDescHeaderLen;
  for (uint8_t i = 0; i < count; i++) {
    const ChannelDef& c = chans[i];
    memset(&out[off], 0, kRecordLen);  // covers name padding + flags/reserved
    out[off + 0] = c.channelId;
    out[off + 1] = c.quantity;
    putU16(&out[off + 2], c.samplePeriodMs);
    // f32 LE via memcpy: both host (x86-64) and target (Cortex-M4) are
    // little-endian IEEE-754; no type punning, -Werror clean.
    memcpy(&out[off + 4], &c.scale, 4);
    memcpy(&out[off + 8], &c.offset, 4);
    for (size_t j = 0; j < 8 && c.name[j] != '\0'; j++) {
      out[off + 12 + j] = (uint8_t)c.name[j];
    }
    off += kRecordLen;
  }
  return off;
}

void buildClock(uint8_t out[kClockLen], uint8_t bootId, uint32_t millisNow) {
  out[0] = bootId;
  out[1] = 0;  // reserved
  putU32(&out[2], millisNow);
}

size_t buildSampleFrame(uint8_t* out, size_t outCap, uint8_t channelId,
                        uint8_t bootId, uint8_t seq, uint32_t baseTimestampMs,
                        uint16_t intervalMs, const int16_t* samples,
                        uint8_t n) {
  if (out == nullptr || samples == nullptr || n == 0) return 0;
  const size_t need = kSampleHeaderLen + (size_t)n * 2;
  if (outCap < need) return 0;

  out[0] = channelId;
  out[1] = bootId;
  out[2] = seq;
  putU32(&out[3], baseTimestampMs);
  putU16(&out[7], intervalMs);
  out[9] = n;
  for (uint8_t i = 0; i < n; i++) {
    putU16(&out[kSampleHeaderLen + (size_t)i * 2], (uint16_t)samples[i]);
  }
  return need;
}

uint8_t maxSamplesForMtu(uint16_t attMtu) {
  // Notify payload = ATT_MTU - 3; the 10-byte header must fit, then two
  // bytes per sample.
  if (attMtu < kSampleHeaderLen + 3 + 2) return 0;
  const uint16_t n = (uint16_t)(((attMtu - 3) - kSampleHeaderLen) / 2);
  return n > 255 ? (uint8_t)255 : (uint8_t)n;
}

}  // namespace pw_gatt
