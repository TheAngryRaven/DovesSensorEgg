#pragma once

///////////////////////////////////////////
// MCP9600 REGISTER-LEVEL LOGIC (pure, host-tested)
//
// Arduino-free constants + codecs for the MCP9600 thermocouple amp,
// following the repo's pw_adv_encode pattern. The I2C plumbing lives in
// mcp9600.{h,cpp}; everything here is testable on the host.
//
// The chip is a register-pointer I2C slave: write a one-byte pointer,
// then read. Temperatures are int16, 0.0625 degC per LSB. It clock-
// stretches aggressively and may NACK its own address mid-conversion —
// which is why detection is a retried ID-register handshake, never a
// blind address probe.
///////////////////////////////////////////

#include <stdint.h>

namespace mcp9600_regs {

// Register pointers.
constexpr uint8_t kRegHotJunction  = 0x00;  // thermocouple temp (int16)
constexpr uint8_t kRegColdJunction = 0x02;  // cold junction temp (int16)
constexpr uint8_t kRegStatus       = 0x04;  // bit4 = input range fault
constexpr uint8_t kRegSensorConfig = 0x05;  // TC type + filter
constexpr uint8_t kRegDeviceConfig = 0x06;  // ADC resolution + mode
constexpr uint8_t kRegDeviceId     = 0x20;  // ID/revision (2 bytes)

// I2C address range (ADDR pin strapping).
constexpr uint8_t kAddrFirst = 0x60;
constexpr uint8_t kAddrLast  = 0x67;

// Device ID register high byte: 0x40 = MCP9600, 0x41 = MCP9601.
bool isValidDeviceId(uint8_t idHigh);

// Temperature codec: int16 big-endian register pair, 0.0625 degC/LSB.
float decodeTempC(uint8_t hi, uint8_t lo);

// Sensor config (reg 0x05): K-type thermocouple (bits 6:4 = 000) with
// filter coefficient n (bits 2:0). The sketch uses filter 3.
uint8_t sensorConfig(uint8_t filterCoeff);

// Device config (reg 0x06): ADC resolution (bits 6:5) + shutdown mode
// (bits 1:0). Resolution ladder — conversion time is the busy window
// during which the chip stretches/NACKs, so lower resolution shrinks the
// collision surface at the cost of coarser temps:
//   00 = 18-bit  0.0625 C  ~250 ms
//   01 = 16-bit  0.25 C    ~63 ms   <- used: matches the deci-degC wire format
//   10 = 14-bit  1 C       ~16 ms
//   11 = 12-bit  4 C       ~4 ms
// (There is no 8-bit mode.) Change kAdcResolutionBits to move the ladder.
constexpr uint8_t kAdcResolutionBits = 0x01;  // 16-bit

uint8_t deviceConfigNormal();    // selected resolution, Normal mode (00)
uint8_t deviceConfigShutdown();  // selected resolution, Shutdown mode (01)

// STATUS bit 4: input range fault (probe open / reversed). Mirrors
// pw_adv's kStatusInputRangeMask — kept here for register completeness.
constexpr uint8_t kStatusInputRange = 0x10;

}  // namespace mcp9600_regs
