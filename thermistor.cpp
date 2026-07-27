#include "thermistor.h"

#include <math.h>

namespace thermistor {

float countsToResistance(uint16_t counts) {
  if (counts < kCountsFloor || counts > kCountsCeiling) return NAN;
  // Vout/Vdd = counts/kAdcMax = kRFixed / (kRFixed + Rntc)
  //   -> Rntc = kRFixed * (kAdcMax - counts) / counts
  return kRFixed * ((float)(kAdcMax - counts)) / (float)counts;
}

float countsToC(uint16_t counts) {
  const float r = countsToResistance(counts);
  if (isnan(r)) return NAN;
  const float invT = 1.0f / kT0K + logf(r / kR0) / kB;
  return 1.0f / invT - 273.15f;
}

}  // namespace thermistor
