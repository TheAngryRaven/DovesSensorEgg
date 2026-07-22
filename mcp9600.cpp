#include "mcp9600.h"

#include <Arduino.h>
#include <math.h>

#include "mcp9600_regs.h"

using namespace mcp9600_regs;

static SoftI2CStatus regRead(Mcp9600& m, uint8_t addr, uint8_t reg,
                             uint8_t* buf, size_t len) {
  return softI2cWriteRead(*m.bus, addr, &reg, 1, buf, len);
}

bool mcpIdHandshake(Mcp9600& m, uint8_t addr, uint8_t& idHigh, uint8_t& idLow) {
  uint8_t id[2] = {0, 0};
  if (regRead(m, addr, kRegDeviceId, id, 2) != SoftI2CStatus::kOk) return false;
  idHigh = id[0];
  idLow  = id[1];
  return true;
}

bool mcpDetect(Mcp9600& m, uint8_t triesPerAddr, uint8_t gapMs) {
  for (uint8_t addr = kAddrFirst; addr <= kAddrLast; addr++) {
    for (uint8_t t = 0; t < triesPerAddr; t++) {
      uint8_t hi = 0, lo = 0;
      if (mcpIdHandshake(m, addr, hi, lo) && isValidDeviceId(hi)) {
        m.addr = addr;
        Serial.print("MCP9600 @0x"); Serial.print(addr, HEX);
        Serial.print(" ID 0x"); Serial.print(hi, HEX);
        Serial.print(" rev 0x"); Serial.println(lo, HEX);
        return true;
      }
      delay(gapMs);  // mid-conversion NACK is normal — give it a beat
    }
  }
  return false;
}

bool mcpConfigure(Mcp9600& m) {
  const uint8_t sensorCfg[2] = {kRegSensorConfig, sensorConfig(3)};
  const uint8_t deviceCfg[2] = {kRegDeviceConfig, deviceConfigNormal()};
  if (softI2cWrite(*m.bus, m.addr, sensorCfg, 2) != SoftI2CStatus::kOk)
    return false;
  if (softI2cWrite(*m.bus, m.addr, deviceCfg, 2) != SoftI2CStatus::kOk)
    return false;
  // Read-back verify: a chip that ACKed but didn't take the config (EMI
  // glitch mid-write) is caught here instead of silently mismeasuring.
  uint8_t check = 0xFF;
  if (regRead(m, m.addr, kRegSensorConfig, &check, 1) != SoftI2CStatus::kOk)
    return false;
  return check == sensorConfig(3);
}

bool mcpModeCycle(Mcp9600& m) {
  const uint8_t shutdownCfg[2] = {kRegDeviceConfig, deviceConfigShutdown()};
  if (softI2cWrite(*m.bus, m.addr, shutdownCfg, 2) != SoftI2CStatus::kOk)
    return false;
  delay(5);  // let the conversion engine actually stop
  return mcpConfigure(m);  // Normal mode is written by configure()
}

static float readTempReg(Mcp9600& m, uint8_t reg) {
  uint8_t raw[2];
  if (m.addr == 0) return NAN;
  if (regRead(m, m.addr, reg, raw, 2) != SoftI2CStatus::kOk) return NAN;
  return decodeTempC(raw[0], raw[1]);
}

float mcpReadHotC(Mcp9600& m)  { return readTempReg(m, kRegHotJunction); }
float mcpReadColdC(Mcp9600& m) { return readTempReg(m, kRegColdJunction); }

uint8_t mcpReadStatus(Mcp9600& m) {
  uint8_t st = 0xFF;
  if (m.addr == 0) return 0xFF;
  if (regRead(m, m.addr, kRegStatus, &st, 1) != SoftI2CStatus::kOk) return 0xFF;
  return st;
}

void mcpShutdown(Mcp9600& m) {
  if (m.addr == 0) return;
  const uint8_t shutdownCfg[2] = {kRegDeviceConfig, deviceConfigShutdown()};
  (void)softI2cWrite(*m.bus, m.addr, shutdownCfg, 2);
}
