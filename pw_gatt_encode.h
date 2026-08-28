#pragma once

///////////////////////////////////////////
// PW GATT ENCODERS
// Pure logic for the GATT-side wire formats (docs/PW_SENSOR_SERVICE.md),
// host-tested exactly like pw_adv_encode. No Arduino headers.
//
// Phase 1 slice (docs/ROADMAP.md): the ESS mirror codec. The
// PerchWerks descriptor / sample-frame / clock builders land here in
// Phases 2-3 with golden-byte tests pinned to the spec's tables.
///////////////////////////////////////////

#include <stdint.h>

namespace pw_gatt {

// Bluetooth SIG Temperature characteristic 0x2A6E: sint16 in
// centi-degC, 0x8000 = "value is not known". Its ceiling is 327.67 C —
// an EGT at 650 C cannot be represented, which is why EGT is never
// mirrored to ESS (docs/PW_SENSOR_SERVICE.md section 7): out-of-range
// maps to "not known", never to a clamped lie.
constexpr int16_t kEssUnknown = INT16_MIN;  // 0x8000 on the wire

int16_t encodeCentiC(float c);

}  // namespace pw_gatt
