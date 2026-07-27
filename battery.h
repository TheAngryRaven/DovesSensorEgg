#pragma once

///////////////////////////////////////////
// BATTERY CODEC (pure, host-tested)
//
// Converts a SAADC reading of the XIAO nRF52840's onboard battery
// divider to volts and a wire-format percent byte. Arduino-free; the
// ADC plumbing (and the VBAT_ENABLE gating) lives in the sketch.
//
// The scaling math is ported from DovesDataLogger (BirdsEye.ino,
// getBatteryVoltage / getBatteryPercent) — the calibration work was
// done there and the two devices share the same board:
//   - 12-bit read, AR_INTERNAL reference (3.6 V full scale). The 3.6
//     assumption is BAKED INTO the 3.024 calibration below — the two
//     constants move together or not at all.
//   - Nominal divider: (1000+510)/510 = 2.9608, but reads ~2% low due
//     to resistor/VREF tolerances (4.11 V observed at a true 4.20 V
//     full charge). Calibrated: 2.9608 * (4.20/4.11) = 3.024.
//   - Percent: linear LiPo window, 3.3 V (cutoff) = 0%, 4.2 V (full
//     charge) = 100%, clamped.
//
// One egg-side addition: below kNoBatteryVolts the divider is reading
// a missing pack (bench on USB power), and the wire byte becomes
// pw_adv::kBatteryUnknown (0xFF) instead of a lying 0%.
///////////////////////////////////////////

#include <stdint.h>

namespace battery {

constexpr float kVref        = 3.6f;     // AR_INTERNAL full scale
constexpr float kAdcMax      = 4096.0f;  // 12-bit (matches datalogger)
constexpr float kScale       = 3.024f;   // calibrated divider, see above
constexpr float kEmptyVolts  = 3.3f;     // LiPo cutoff  -> 0%
constexpr float kFullVolts   = 4.2f;     // full charge  -> 100%
constexpr float kNoBatteryVolts = 2.5f;  // below this: no pack fitted

constexpr uint8_t kUnknown = 0xFF;       // == pw_adv::kBatteryUnknown

// ADC counts -> battery volts (calibrated).
float countsToVolts(uint16_t counts);

// Volts -> wire percent byte: 0-100 linear over the LiPo window, or
// kUnknown when the voltage says there is no pack to measure.
uint8_t voltsToPercent(float volts);

}  // namespace battery
