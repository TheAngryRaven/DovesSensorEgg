#include "doctest.h"

#include <cstdint>
#include <limits>

#include "pw_gatt_encode.h"

using namespace pw_gatt;

TEST_CASE("pw_gatt - centi-degC encoding for the ESS temperature mirror") {
    CHECK(encodeCentiC(23.40f) == 2340);
    CHECK(encodeCentiC(0.0f) == 0);
    CHECK(encodeCentiC(-10.55f) == -1055);
    CHECK(encodeCentiC(33.3f) == 3330);
}

TEST_CASE("pw_gatt - NaN becomes the ESS 'not known' value") {
    CHECK(encodeCentiC(std::numeric_limits<float>::quiet_NaN()) == kEssUnknown);
    CHECK((uint16_t)encodeCentiC(std::numeric_limits<float>::quiet_NaN()) ==
          0x8000);
}

TEST_CASE("pw_gatt - out-of-range reports 'not known', never a clamped lie") {
    // The reason EGT is never mirrored to ESS: 650 C does not fit sint16
    // centi-degC, and 327.67 C on a generic app's screen would be a lie.
    CHECK(encodeCentiC(650.0f) == kEssUnknown);
    CHECK(encodeCentiC(328.0f) == kEssUnknown);
    CHECK(encodeCentiC(-300.0f) == kEssUnknown);
    CHECK(encodeCentiC(std::numeric_limits<float>::infinity()) == kEssUnknown);
    CHECK(encodeCentiC(-std::numeric_limits<float>::infinity()) == kEssUnknown);
}

TEST_CASE("pw_gatt - representable boundaries encode exactly") {
    CHECK(encodeCentiC(327.67f) == 32767);
    CHECK(encodeCentiC(-273.15f) == -27315);
}

TEST_CASE("pw_gatt - rounds to nearest centi-degree") {
    CHECK(encodeCentiC(23.456f) == 2346);
    CHECK(encodeCentiC(23.454f) == 2345);
    CHECK(encodeCentiC(-0.004f) == 0);
}
