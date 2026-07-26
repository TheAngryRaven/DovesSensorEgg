#include "battery.h"

namespace battery {

float countsToVolts(uint16_t counts) {
  return (float)counts * kVref / kAdcMax * kScale;
}

uint8_t voltsToPercent(float volts) {
  if (volts < kNoBatteryVolts) return kUnknown;
  float pct = (volts - kEmptyVolts) / (kFullVolts - kEmptyVolts) * 100.0f;
  if (pct < 0.0f)   pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  return (uint8_t)(pct + 0.5f);
}

}  // namespace battery
