#include "doctest.h"

#include <cstdint>

#include "mcp9600_regs.h"

using namespace mcp9600_regs;

TEST_CASE("mcp9600_regs - temperature decode: 0.0625 C per LSB") {
    // 25.0 C = 400 LSB = 0x0190 (big-endian register pair).
    CHECK(decodeTempC(0x01, 0x90) == doctest::Approx(25.0f));
    // 650.0 C = 10400 LSB = 0x28A0 — a working EGT reading.
    CHECK(decodeTempC(0x28, 0xA0) == doctest::Approx(650.0f));
    CHECK(decodeTempC(0x00, 0x00) == doctest::Approx(0.0f));
    CHECK(decodeTempC(0x00, 0x01) == doctest::Approx(0.0625f));
}

TEST_CASE("mcp9600_regs - temperature decode: negative sign extension") {
    // -1.0 C = -16 LSB = 0xFFF0.
    CHECK(decodeTempC(0xFF, 0xF0) == doctest::Approx(-1.0f));
    // -12.5 C = -200 LSB = 0xFF38.
    CHECK(decodeTempC(0xFF, 0x38) == doctest::Approx(-12.5f));
}

TEST_CASE("mcp9600_regs - device ID accepts MCP9600 and MCP9601, rejects others") {
    CHECK(isValidDeviceId(0x40));        // MCP9600
    CHECK(isValidDeviceId(0x41));        // MCP9601
    CHECK_FALSE(isValidDeviceId(0x00));  // dead bus reads
    CHECK_FALSE(isValidDeviceId(0xFF));  // floating bus reads
    CHECK_FALSE(isValidDeviceId(0x3C));  // an OLED answering by mistake
}

TEST_CASE("mcp9600_regs - config bytes") {
    // Sensor config: K-type (type bits 000) + filter coefficient 3.
    CHECK(sensorConfig(3) == 0x03);
    CHECK(sensorConfig(0) == 0x00);
    // Filter is a 3-bit field — out-of-range input must not leak upward
    // into the thermocouple-type bits.
    CHECK(sensorConfig(0xFF) == 0x07);

    // Device config: 16-bit resolution (01 << 5 = 0x20), Normal mode 00.
    CHECK(deviceConfigNormal() == 0x20);
    // Shutdown mode = 01 in the low bits, same resolution.
    CHECK(deviceConfigShutdown() == 0x21);
}

TEST_CASE("mcp9600_regs - register map spot checks (wire contract with the chip)") {
    CHECK(kRegHotJunction == 0x00);
    CHECK(kRegColdJunction == 0x02);
    CHECK(kRegStatus == 0x04);
    CHECK(kRegSensorConfig == 0x05);
    CHECK(kRegDeviceConfig == 0x06);
    CHECK(kRegDeviceId == 0x20);
    CHECK(kAddrFirst == 0x60);
    CHECK(kAddrLast == 0x67);
    CHECK(kStatusInputRange == 0x10);
}
