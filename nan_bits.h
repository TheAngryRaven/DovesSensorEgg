#pragma once

///////////////////////////////////////////
// NaN DETECTION THAT SURVIVES -Ofast
//
// The Seeed nRF52 platform compiles sketches with -Ofast, which
// includes -ffinite-math-only: GCC constant-folds isnan()/isinf() to
// FALSE and assumes NaN never exists in comparisons. Field-confirmed
// 2026-07-27: every isnan()-gated branch in device code was silently
// compiled out (the thermistor EMA never seeded -> permanent nan; the
// nan serial forensics never printed in any build; the payload's
// NaN -> 0x8000 sentinel guard could cast garbage onto the wire).
// Host builds don't use -Ofast, so the host test suite could not see
// any of it.
//
// This helper inspects the IEEE-754 bit pattern via memcpy, which the
// optimizer cannot fold away: exponent all-ones + nonzero mantissa.
// RULE: device-compiled code (the .ino and every module it links) must
// use isNanF() and must never compare against a possibly-NaN value
// without checking it first. Plain isnan() is reserved for host-only
// code.
///////////////////////////////////////////

#include <stdint.h>
#include <string.h>

static inline bool isNanF(float f) {
  uint32_t u;
  memcpy(&u, &f, sizeof u);
  return (u & 0x7F800000u) == 0x7F800000u && (u & 0x007FFFFFu) != 0u;
}
