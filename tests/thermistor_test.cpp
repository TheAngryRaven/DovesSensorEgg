#include "doctest.h"

#include <cmath>
#include <cstdint>

#include "thermistor.h"

using namespace thermistor;

// The divider is NTC-on-the-bottom (as built): counts/4096 =
// Rntc/(Rfix+Rntc). With R0 == kRFixed (100k/100k), R = R0 lands at
// exactly mid-scale.

TEST_CASE("thermistor - R0 at mid-scale reads 25.0 C") {
    // counts = 4096 * R0/(Rfix+R0) = 2048 when Rfix == R0.
    CHECK(countsToResistance(2048) == doctest::Approx(kR0).epsilon(0.001));
    CHECK(countsToC(2048) == doctest::Approx(25.0f).epsilon(0.01));
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

TEST_CASE("thermistor - B-equation spot checks for a 100k/3950 part") {
    // R(T) = R0 * exp(B * (1/T - 1/T0)); counts = 4096*R/(Rfix+R).
    // 0 C  -> R ~= 336.6k -> counts ~= 3157 (Approx near zero is
    // relative, so bound the absolute error directly)
    // 50 C -> R ~= 35.85k -> counts ~= 1081
    CHECK(std::fabs(countsToC(3157)) < 0.2f);
    CHECK(countsToC(1081) == doctest::Approx(50.0f).epsilon(0.01));
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
