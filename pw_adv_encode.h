#pragma once

///////////////////////////////////////////
// PW-ADV-1 PAYLOAD ENCODER
// The egg side of the wire contract with DovesDataLogger's
// sensoregg_protocol parser: this unit builds the exact 14-byte
// Manufacturer Specific Data payload the logger decodes. Pure logic —
// no Arduino headers — so the host test harness compiles it; the
// golden-byte test mirrors the logger repo's parser fixture, pinning
// encode == decode across the two repos.
//
// Layout (little-endian fields; Bluefruit's addManufacturerData()
// passes the buffer RAW, so bytes 0-1 ARE the company ID):
//   0-1  company ID FF FF     2-3  magic 'P' 'W'      4  proto version 01
//   5    flags (bit0 pairing, bit1 TC fault)
//   6-7  EGT int16 deci-degC  8-9  CJ int16 deci-degC
//   10   raw MCP9600 STATUS   11   battery stub FF    12-13 sequence
///////////////////////////////////////////

#include <stddef.h>
#include <stdint.h>

namespace pw_adv {

constexpr size_t  kPayloadLen = 14;
constexpr uint8_t kProtoVer   = 0x01;

// MCP9600 STATUS bit 4 = input range fault (probe open / reversed);
// mapped into payload flags bit1 so the logger can show *TC FAULT*.
constexpr uint8_t kStatusInputRangeMask = 0x10;
constexpr uint8_t kFlagPairing          = 0x01;
constexpr uint8_t kFlagTcFault          = 0x02;

// 0x8000 (INT16_MIN) = "no valid reading". Emitted instead of casting
// NaN / out-of-range floats to int16_t — that cast is undefined behavior
// and produces plausible-looking garbage.
int16_t encodeDeciC(float c);

// Celsius -> Fahrenheit for the debug screen. NaN propagates.
float c2f(float c);

// Build the full 14-byte payload. `status` is the raw MCP9600 STATUS
// register (fault flag derived here); `pairingActive` sets flags bit0.
void buildPayload(uint8_t out[kPayloadLen], float egtC, float cjC,
                  uint8_t status, bool pairingActive, uint16_t seq);

}  // namespace pw_adv
