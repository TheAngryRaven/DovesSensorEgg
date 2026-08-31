#include "pw_gatt_encode.h"

#include <math.h>

#include "nan_bits.h"

namespace pw_gatt {

int16_t encodeCentiC(float c) {
  // isNanF, not isnan: the device build is -Ofast, where isnan() folds
  // to false and a NaN would fall through into lroundf -> garbage on
  // the wire instead of the "not known" value. The NaN check must come
  // FIRST - the range comparisons are only meaningful once c is known
  // to be a real number. Range: sint16 centi-degC representable span.
  if (isNanF(c) || c < -273.15f || c > 327.67f) return kEssUnknown;
  return (int16_t)lroundf(c * 100.0f);
}

}  // namespace pw_gatt
