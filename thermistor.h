#pragma once

///////////////////////////////////////////
// THERMISTOR CODEC (pure, host-tested)
//
// Converts a ratiometric SAADC reading of the auxiliary NTC divider to
// Celsius. Arduino-free, following the repo's pw_adv / mcp9600_regs
// extraction pattern; the ADC plumbing lives in the sketch.
//
// Electrical contract (see the sketch + README) — matches the
// as-built harness (2026-07-27): R_FIXED from the power-gate GPIO
// (driven to VDD during the read) down to the sense node; NTC from the
// sense node to GND.
//       Vout / Vdd = R_NTC / (R_FIXED + R_NTC)
//   - ADC: 12-bit, AR_VDD4 reference -> full scale == VDD, the same
//     rail the GPIO drives the divider with. The supply cancels out of
//     the ratio EXACTLY, so there is no calibration constant here.
//   - Counts pegged at either rail mean an open or shorted divider:
//     NAN, which the payload encoder turns into the 0x8000 wire
//     sentinel. In this topology, HIGH rail = the NTC leg is not
//     conducting (open joint, thermistor missing); LOW rail = no drive
//     from the power gate, or the sense line is not on the node.
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

// Part constants. Two probe classes have been identified on this bench
// (2026-07-27) by the serial line's measured-R readout — pick ONE:
//
//   Glass bead "Ender 3" spare: 100k @25C, B 3950 (reads ~109k at 23C)
//     -> the values below, with the 100k fixed leg. CURRENT CONFIG.
//   Sealed metal-tube probe: 10k class (reads ~10.9k at ambient)
//     -> kR0 = 10000, kB ~= 3950 (some are 3435 - calibrate against a
//        known temp), and ideally kRFixed = 10000 to re-center the
//        divider's sensitivity on the interesting range.
constexpr float kR0     = 100000.0f;  // NTC resistance at 25 C
constexpr float kB      = 3950.0f;    // B25/85 coefficient
constexpr float kRFixed = 100000.0f;  // divider top leg (power gate side)
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
