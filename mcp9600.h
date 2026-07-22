#pragma once

///////////////////////////////////////////
// MCP9600 DRIVER (over soft_i2c + mcp9600_regs)
//
// Replaces the Adafruit_MCP9600 library: that path went through the
// core's no-timeout Wire, and this chip's clock stretching + ignition
// EMI is exactly how the 2026-07-19 zombie hang happened. Every call
// here is bounded by the soft bus's stretch timeout and returns a
// definite answer.
//
// Detection is a proper ID-register handshake (pointer-write 0x20, read
// 2 bytes, validate 0x40/0x41) retried per address — the chip may NACK
// its own address mid-conversion, so one NACK never means "absent".
// Blind START/STOP probes of the MCP range are gone entirely; community
// experience says they can confuse its interface state machine.
//
// modeCycle() is the chip's nearest thing to a reset command: Shutdown
// -> Normal via the device-config register, then a full reconfigure.
// Used at begin and by the runtime recovery path.
///////////////////////////////////////////

#include <stdint.h>

#include "soft_i2c.h"

struct Mcp9600 {
  SoftI2C* bus = nullptr;
  uint8_t  addr = 0;   // 0 = not detected
};

// One well-formed ID handshake at addr. Returns true and fills idHigh /
// idLow on success (caller validates via mcp9600_regs::isValidDeviceId).
bool mcpIdHandshake(Mcp9600& m, uint8_t addr, uint8_t& idHigh, uint8_t& idLow);

// Scan 0x60..0x67 with retried handshakes (triesPerAddr, gapMs apart).
// On success sets m.addr and returns true.
bool mcpDetect(Mcp9600& m, uint8_t triesPerAddr, uint8_t gapMs);

// Write sensor + device config, then read sensor config back to verify.
bool mcpConfigure(Mcp9600& m);

// Shutdown -> Normal -> configure(). The "reset command" equivalent.
bool mcpModeCycle(Mcp9600& m);

// Reads. NaN / 0xFF on any bus fault (timeout, NACK) — never stale data.
float   mcpReadHotC(Mcp9600& m);
float   mcpReadColdC(Mcp9600& m);
uint8_t mcpReadStatus(Mcp9600& m);

// Put the chip in Shutdown mode (deep sleep; ~uA).
void mcpShutdown(Mcp9600& m);
