#pragma once

///////////////////////////////////////////
// PW-ADV-2 PAYLOAD ENCODER
// The egg side of the wire contract with DovesDataLogger's
// sensoregg_protocol parser: this unit builds the exact 16-byte
// Manufacturer Specific Data payload the logger decodes. Pure logic —
// no Arduino headers — so the host test harness compiles it; the
// golden-byte test mirrors the logger repo's parser fixture, pinning
// encode == decode across the two repos.
//
// v2 (2026-07-26) appends to v1 — bytes 0-13 keep their exact v1
// offsets, so the logger's v1 field decoding carried over unchanged
// when its version gate learned 0x02. That logger round has LANDED
// (DovesDataLogger BETA: sensoregg_protocol accepts 0x01 and 0x02 and
// captures the full 16 bytes — the old RX path truncated at 14), so
// v2 parses end-to-end. nRF Connect stays the neutral bench check for
// the raw bytes.
//
// Layout (little-endian fields; Bluefruit's addManufacturerData()
// passes the buffer RAW, so bytes 0-1 ARE the company ID):
//   0-1  company ID FF FF     2-3  magic 'P' 'W'      4  proto version 02
//   5    flags (bit0 pairing, bit1 TC fault)
//   6-7  EGT int16 deci-degC  8-9  CJ int16 deci-degC
//   10   raw MCP9600 STATUS   11   battery percent (0-100, FF = unknown)
//   12-13 sequence            14-15 thermistor int16 deci-degC
///////////////////////////////////////////

#include <stddef.h>
#include <stdint.h>

namespace pw_adv {

constexpr size_t  kPayloadLen = 16;
constexpr uint8_t kProtoVer   = 0x02;

// Battery byte 11: percent 0-100, or this when no pack / not measured.
constexpr uint8_t kBatteryUnknown = 0xFF;

// MCP9600 STATUS bit 4 = input range fault (probe open / reversed);
// mapped into payload flags bit1 so the logger can show *TC FAULT*.
constexpr uint8_t kStatusInputRangeMask = 0x10;
constexpr uint8_t kFlagPairing          = 0x01;
constexpr uint8_t kFlagTcFault          = 0x02;

// 0x8000 (INT16_MIN) = "no valid reading". Emitted instead of casting
// NaN / out-of-range floats to int16_t — that cast is undefined behavior
// and produces plausible-looking garbage. Covers the thermistor too: an
// open/shorted divider arrives here as NaN and leaves as the sentinel,
// so no separate thermistor-fault flag bit is needed.
int16_t encodeDeciC(float c);

// Celsius -> Fahrenheit for the debug screen. NaN propagates.
float c2f(float c);

// Build the full 16-byte payload. `status` is the raw MCP9600 STATUS
// register (fault flag derived here); `pairingActive` sets flags bit0;
// `batteryPct` goes into byte 11 verbatim (pass kBatteryUnknown when
// unmeasured); `thermC` is the auxiliary thermistor in Celsius.
void buildPayload(uint8_t out[kPayloadLen], float egtC, float cjC,
                  uint8_t status, bool pairingActive, uint16_t seq,
                  uint8_t batteryPct, float thermC);

}  // namespace pw_adv
