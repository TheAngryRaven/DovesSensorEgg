#pragma once

///////////////////////////////////////////
// SOFT I2C MASTER (bit-banged, timeout-capable)
//
// The MCP9600's dedicated bus. The hardware TWIM's Wire wrapper blocks
// with NO timeout — the 2026-07-19 zombie incident was the loop parked
// forever inside an MCP read after ignition EMI wedged the bus. A
// bit-banged master fixes the category: every SCL release waits for the
// line to actually rise (clock-stretch tolerated) but BOUNDED — a wedged
// or stretching-forever slave returns kTimeout instead of hanging, so
// the app keeps looping, readings go to the wire sentinel, and runtime
// recovery gets a chance to run before the WDT ever fires.
//
// Also: I2C has no minimum clock (it's a static protocol, and so is the
// MCP9600), so this bus runs deliberately slow (~50 kHz half-period) for
// EMI margin. The OLED stays on hardware Wire, now free to run 400 kHz.
//
// Open-drain is emulated the classic way: INPUT_PULLUP = release high,
// OUTPUT+LOW = drive low. External 4.7 k pullups recommended on the bus;
// the nRF52's internal ~13 k pullups are workable at this speed.
///////////////////////////////////////////

#include <stdint.h>
#include <stddef.h>

enum class SoftI2CStatus : uint8_t {
  kOk = 0,
  kNackAddr,   // address byte not ACKed (absent, or MCP busy mid-conversion)
  kNackData,   // a data byte not ACKed
  kTimeout,    // SCL or SDA held low past the stretch timeout — bus wedged
};

struct SoftI2C {
  uint8_t  sdaPin;
  uint8_t  sclPin;
  uint16_t halfPeriodUs;      // 10 -> ~50 kHz
  uint32_t stretchTimeoutUs;  // max wait for SCL/SDA to rise (per edge)
};

// Configure pins (released/high) — call once at boot.
void softI2cBegin(SoftI2C& bus);

// Standard bus recovery: 9 clocks with SDA released, then a STOP. Run at
// boot and before every runtime recovery attempt (a slave reset
// mid-transaction can hold SDA low forever otherwise).
void softI2cBusClear(SoftI2C& bus);

// Write wlen bytes then (repeated-start) read rlen bytes. Either half may
// be empty (len 0). Returns the first failure encountered; on any
// non-kOk result the bus is left released with a best-effort STOP.
SoftI2CStatus softI2cWriteRead(SoftI2C& bus, uint8_t addr,
                               const uint8_t* wbuf, size_t wlen,
                               uint8_t* rbuf, size_t rlen);

// Convenience: pure write with STOP.
static inline SoftI2CStatus softI2cWrite(SoftI2C& bus, uint8_t addr,
                                         const uint8_t* buf, size_t len) {
  return softI2cWriteRead(bus, addr, buf, len, nullptr, 0);
}
