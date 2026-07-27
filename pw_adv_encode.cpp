#include "pw_adv_encode.h"

#include <math.h>

namespace pw_adv {

int16_t encodeDeciC(float c) {
  if (isnan(c) || c < -270.0f || c > 1400.0f) return INT16_MIN;
  return (int16_t)lroundf(c * 10.0f);
}

float c2f(float c) { return c * 9.0f / 5.0f + 32.0f; }

void buildPayload(uint8_t out[kPayloadLen], float egtC, float cjC,
                  uint8_t status, bool pairingActive, uint16_t seq,
                  uint8_t batteryPct, float thermC) {
  uint8_t flags = 0;
  if (pairingActive)                    flags |= kFlagPairing;
  if (status & kStatusInputRangeMask)   flags |= kFlagTcFault;

  const int16_t egt = encodeDeciC(egtC);
  const int16_t cj  = encodeDeciC(cjC);
  const int16_t th  = encodeDeciC(thermC);

  out[0]  = 0xFF; out[1] = 0xFF;             // company ID (SIG test/internal)
  out[2]  = 'P';  out[3] = 'W';              // magic
  out[4]  = kProtoVer;
  out[5]  = flags;
  out[6]  = (uint8_t)(egt & 0xFF);
  out[7]  = (uint8_t)((egt >> 8) & 0xFF);
  out[8]  = (uint8_t)(cj & 0xFF);
  out[9]  = (uint8_t)((cj >> 8) & 0xFF);
  out[10] = status;
  out[11] = batteryPct;
  out[12] = (uint8_t)(seq & 0xFF);
  out[13] = (uint8_t)((seq >> 8) & 0xFF);
  out[14] = (uint8_t)(th & 0xFF);
  out[15] = (uint8_t)((th >> 8) & 0xFF);
}

}  // namespace pw_adv
