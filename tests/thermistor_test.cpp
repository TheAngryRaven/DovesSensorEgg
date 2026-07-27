#include "doctest.h"

#include <cmath>
#include <cstdint>

#include "thermistor.h"

using namespace thermistor;

// The divider is NTC-on-the-bottom (as built): counts/4096 =
// Rntc/(Rfix+Rntc). Current config: 10k/3950 probe against the
// physical 100k fixed leg (the deliberate "lazy" divider — see
// thermistor.h).

TEST_CASE("thermistor - resistance recovery is ratio-exact") {
    // Mid-scale means Rntc == kRFixed, whatever the probe class.
    CHECK(countsToResistance(2048) == doctest::Approx(kRFixed).epsilon(0.001));
    // R0 (10k) lands at counts = 4096 * 10/(110) ~= 372 -> 25 C.
    CHECK(countsToResistance(372) == doctest::Approx(kR0).epsilon(0.005));
    CHECK(countsToC(372) == doctest::Approx(25.0f).epsilon(0.01));
}

TEST_CASE("thermistor - monotonic: more counts (higher R) = colder") {
    float prev = countsToC(kCountsFloor);
    for (uint16_t c = kCountsFloor + 64; c <= kCountsCeiling; c += 64) {
        const float t = countsToC(c);
        CAPTURE(c);
        CHECK(t < prev);
        prev = t;
    }
}

TEST_CASE("thermistor - B-equation spot checks for a 10k/3950 part") {
    // R(T) = R0 * exp(B * (1/T - 1/T0)); counts = 4096*R/(Rfix+R).
    // 0 C  -> R ~= 33.62k -> counts ~= 1031 (Approx near zero is
    // relative, so bound the absolute error directly)
    // 50 C -> R ~= 3.588k -> counts ~= 142
    CHECK(std::fabs(countsToC(1031)) < 0.2f);
    CHECK(countsToC(142) == doctest::Approx(50.0f).epsilon(0.01));
}

TEST_CASE("thermistor - rail-pegged counts are a fault, not a temperature") {
    // Shorted NTC / sense wire to GND -> counts ~ 0.
    CHECK(std::isnan(countsToC(0)));
    CHECK(std::isnan(countsToC(kCountsFloor - 1)));
    // Open NTC leg: node pulled to the rail through Rfix -> full scale.
    CHECK(std::isnan(countsToC(kCountsCeiling + 1)));
    CHECK(std::isnan(countsToC(4095)));
    // Just inside the guards is still a number.
    CHECK(!std::isnan(countsToC(kCountsFloor)));
    CHECK(!std::isnan(countsToC(kCountsCeiling)));
}
