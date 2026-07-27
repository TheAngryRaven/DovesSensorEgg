#pragma once

///////////////////////////////////////////
// THERMISTOR CODEC (pure, host-tested)
//
// Converts a ratiometric SAADC reading of the auxiliary NTC divider to
// Celsius. Arduino-free, following the repo's pw_adv / mcp9600_regs
// extraction pattern; the ADC plumbing lives in the sketch.
//
// Electrical contract (see the sketch + README):
//   - Divider: NTC from the power-gate GPIO (driven to VDD during the
//     read) to the sense node; R_FIXED from the sense node to GND.
//       Vout / Vdd = R_FIXED / (R_FIXED + R_NTC)
//   - ADC: 12-bit, AR_VDD4 reference -> full scale == VDD, the same
//     rail the GPIO drives the divider with. The supply cancels out of
//     the ratio EXACTLY, so there is no calibration constant here.
//   - Counts pegged at either rail mean an open or shorted divider
//     (thermistor unplugged, wiring fault): NAN, which the payload
//     encoder turns into the 0x8000 wire sentinel.
//   - A smoothing cap (10 nF default) sits on the sense node at the
//     board end — invisible to this math, but the sketch's
//     THERM_SETTLE_MS must cover its charge time (they are coupled;
//     see the define in the sketch).
//
// Temperature model: the B-parameter equation,
//   T = 1 / (1/T0 + ln(R/R0)/B) - 273.15
// Good to ~+/-1 C over -20..80 C for a garden-variety 100k/3950 part;
// tune the constants below once the real part is characterized.
///////////////////////////////////////////

#include <stdint.h>

namespace thermistor {

// Part constants — 100k NTC, B ~= 3950 (user's part), 100k fixed leg.
constexpr float kR0     = 100000.0f;  // NTC resistance at 25 C
constexpr float kB      = 3950.0f;    // B25/85 coefficient
constexpr float kRFixed = 100000.0f;  // divider bottom leg (to GND)
constexpr float kT0K    = 298.15f;    // 25 C in kelvin

constexpr uint16_t kAdcMax = 4096;    // 12-bit SAADC (core convention)

// Rail guards: outside these counts the divider is open/shorted, not a
// temperature. 16/4080 keep ~0.4% margin against noise on a live rail.
constexpr uint16_t kCountsFloor   = 16;
constexpr uint16_t kCountsCeiling = 4080;

// NTC resistance implied by a ratiometric reading. NAN when the counts
// peg a rail (see above) — resistance is then undefined, not huge.
float countsToResistance(uint16_t counts);

// Full conversion: counts -> Celsius. NAN on rail-pegged counts.
float countsToC(uint16_t counts);

}  // namespace thermistor
