#include "doctest.h"

#include <cmath>
#include <cstdint>

#include "thermistor.h"

using namespace thermistor;

// The divider is fixed-leg-on-the-bottom: counts/4096 = Rfix/(Rfix+Rntc).
// With R0 == kRFixed (100k/100k), R = R0 lands at exactly mid-scale.

TEST_CASE("thermistor - R0 at mid-scale reads 25.0 C") {
    // counts = 4096 * Rfix/(Rfix+R0) = 2048 when Rfix == R0.
    CHECK(countsToResistance(2048) == doctest::Approx(kR0).epsilon(0.001));
    CHECK(countsToC(2048) == doctest::Approx(25.0f).epsilon(0.01));
}

TEST_CASE("thermistor - monotonic: more counts (lower R) = hotter") {
    float prev = countsToC(kCountsFloor);
    for (uint16_t c = kCountsFloor + 64; c <= kCountsCeiling; c += 64) {
        const float t = countsToC(c);
        CAPTURE(c);
        CHECK(t > prev);
        prev = t;
    }
}

TEST_CASE("thermistor - B-equation spot checks for a 100k/3950 part") {
    // R(T) = R0 * exp(B * (1/T - 1/T0)); counts = 4096*Rfix/(Rfix+R).
    // 0 C  -> R ~= 336.6k -> counts ~= 938 (Approx near zero is relative,
    // so bound the absolute error directly)
    // 50 C -> R ~= 35.85k -> counts ~= 3016
    CHECK(std::fabs(countsToC(938)) < 0.2f);
    CHECK(countsToC(3016) == doctest::Approx(50.0f).epsilon(0.01));
}

TEST_CASE("thermistor - rail-pegged counts are a fault, not a temperature") {
    // Open NTC: sense node pulled to GND through Rfix -> counts ~ 0.
    CHECK(std::isnan(countsToC(0)));
    CHECK(std::isnan(countsToC(kCountsFloor - 1)));
    // Shorted NTC: sense node at the rail -> counts ~ full scale.
    CHECK(std::isnan(countsToC(kCountsCeiling + 1)));
    CHECK(std::isnan(countsToC(4095)));
    // Just inside the guards is still a number.
    CHECK(!std::isnan(countsToC(kCountsFloor)));
    CHECK(!std::isnan(countsToC(kCountsCeiling)));
}
