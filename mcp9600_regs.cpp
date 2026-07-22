#include "mcp9600_regs.h"

namespace mcp9600_regs {

bool isValidDeviceId(uint8_t idHigh) {
  return idHigh == 0x40 || idHigh == 0x41;
}

float decodeTempC(uint8_t hi, uint8_t lo) {
  const int16_t raw = (int16_t)(((uint16_t)hi << 8) | lo);
  return (float)raw * 0.0625f;
}

uint8_t sensorConfig(uint8_t filterCoeff) {
  // Bits 6:4 = thermocouple type, 000 = K. Bits 2:0 = filter coefficient.
  return (uint8_t)(filterCoeff & 0x07);
}

uint8_t deviceConfigNormal() {
  return (uint8_t)(kAdcResolutionBits << 5);  // mode bits 1:0 = 00 (Normal)
}

uint8_t deviceConfigShutdown() {
  return (uint8_t)((kAdcResolutionBits << 5) | 0x01);  // mode 01 = Shutdown
}

}  // namespace mcp9600_regs
