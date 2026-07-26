#include "doctest.h"

#include <cstdint>

#include "battery.h"

using namespace battery;

// The counts->volts scaling is the DovesDataLogger calibration ported
// verbatim (see battery.h): volts = counts * 3.6 / 4096 * 3.024.

TEST_CASE("battery - counts to volts matches the datalogger calibration") {
    // Full charge 4.20 V -> counts = 4.20 / 3.024 / 3.6 * 4096 = 1580.4
    CHECK(countsToVolts(1580) == doctest::Approx(4.20f).epsilon(0.001));
    // Cutoff 3.30 V -> counts ~= 1241.7
    CHECK(countsToVolts(1242) == doctest::Approx(3.30f).epsilon(0.001));
    CHECK(countsToVolts(0) == doctest::Approx(0.0f));
}

TEST_CASE("battery - percent endpoints and clamping") {
    CHECK(voltsToPercent(4.20f) == 100);
    CHECK(voltsToPercent(4.30f) == 100);   // charger overshoot clamps
    CHECK(voltsToPercent(3.30f) == 0);
    CHECK(voltsToPercent(2.90f) == 0);     // sagging pack clamps, still a pack
}

TEST_CASE("battery - midpoint rounds to nearest percent") {
    // 3.75 V = halfway through the 3.3-4.2 window -> 50%.
    CHECK(voltsToPercent(3.75f) == 50);
    CHECK(voltsToPercent(4.11f) == 90);
}

TEST_CASE("battery - no pack fitted reads unknown, not 0%") {
    // USB-only bench: divider sees a floating/near-zero BAT pad.
    CHECK(voltsToPercent(0.0f) == kUnknown);
    CHECK(voltsToPercent(2.49f) == kUnknown);
    CHECK(voltsToPercent(2.51f) == 0);     // just above the gate: a real, dead pack
}
