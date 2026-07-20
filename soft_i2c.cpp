#include "soft_i2c.h"

#include <Arduino.h>

// Open-drain emulation. Never drive HIGH — release and let pullups work.
static inline void pinRelease(uint8_t pin) { pinMode(pin, INPUT_PULLUP); }
static inline void pinDriveLow(uint8_t pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
}
static inline bool pinRead(uint8_t pin) { return digitalRead(pin) == HIGH; }

// Release a line and wait (bounded) for it to actually rise. SCL: this is
// clock-stretch tolerance — the slave may legally hold SCL low; a healthy
// MCP9600 releases within its conversion bookkeeping, a wedged one never
// does, and the bound converts "hang forever" into an error return.
static bool releaseAndWaitHigh(uint8_t pin, uint32_t timeoutUs) {
  pinRelease(pin);
  const uint32_t t0 = micros();
  while (!pinRead(pin)) {
    if ((uint32_t)(micros() - t0) >= timeoutUs) return false;
  }
  return true;
}

static inline void halfDelay(const SoftI2C& bus) {
  delayMicroseconds(bus.halfPeriodUs);
}

void softI2cBegin(SoftI2C& bus) {
  pinRelease(bus.sdaPin);
  pinRelease(bus.sclPin);
}

void softI2cBusClear(SoftI2C& bus) {
  pinRelease(bus.sdaPin);
  for (int i = 0; i < 9; i++) {
    pinDriveLow(bus.sclPin);
    delayMicroseconds(bus.halfPeriodUs);
    pinRelease(bus.sclPin);
    delayMicroseconds(bus.halfPeriodUs);
  }
  // STOP: SDA low->high while SCL high.
  pinDriveLow(bus.sdaPin);
  delayMicroseconds(bus.halfPeriodUs);
  pinRelease(bus.sclPin);
  delayMicroseconds(bus.halfPeriodUs);
  pinRelease(bus.sdaPin);
  delayMicroseconds(bus.halfPeriodUs);
}

// START (or repeated START): SDA high->low while SCL high.
static bool doStart(SoftI2C& bus) {
  if (!releaseAndWaitHigh(bus.sdaPin, bus.stretchTimeoutUs)) return false;
  if (!releaseAndWaitHigh(bus.sclPin, bus.stretchTimeoutUs)) return false;
  halfDelay(bus);
  pinDriveLow(bus.sdaPin);
  halfDelay(bus);
  pinDriveLow(bus.sclPin);
  return true;
}

// STOP: SDA low->high while SCL high. Best-effort (also used on errors).
static void doStop(SoftI2C& bus) {
  pinDriveLow(bus.sdaPin);
  halfDelay(bus);
  (void)releaseAndWaitHigh(bus.sclPin, bus.stretchTimeoutUs);
  halfDelay(bus);
  pinRelease(bus.sdaPin);
  halfDelay(bus);
}

// Clock one bit out. False on SCL stretch timeout.
static bool writeBit(SoftI2C& bus, bool bit) {
  if (bit) {
    pinRelease(bus.sdaPin);
  } else {
    pinDriveLow(bus.sdaPin);
  }
  halfDelay(bus);
  if (!releaseAndWaitHigh(bus.sclPin, bus.stretchTimeoutUs)) return false;
  halfDelay(bus);
  pinDriveLow(bus.sclPin);
  return true;
}

// Clock one bit in. False on SCL stretch timeout.
static bool readBit(SoftI2C& bus, bool& bit) {
  pinRelease(bus.sdaPin);
  halfDelay(bus);
  if (!releaseAndWaitHigh(bus.sclPin, bus.stretchTimeoutUs)) return false;
  bit = pinRead(bus.sdaPin);
  halfDelay(bus);
  pinDriveLow(bus.sclPin);
  return true;
}

// Write a byte, return the slave's ACK. False return = timeout.
static bool writeByte(SoftI2C& bus, uint8_t b, bool& acked) {
  for (int i = 7; i >= 0; i--) {
    if (!writeBit(bus, (b >> i) & 1)) return false;
  }
  bool nack;
  if (!readBit(bus, nack)) return false;  // ACK bit: low = ACK
  acked = !nack;
  return true;
}

// Read a byte, sending ACK (more to come) or NACK (last byte).
static bool readByte(SoftI2C& bus, uint8_t& b, bool ack) {
  b = 0;
  for (int i = 7; i >= 0; i--) {
    bool bit;
    if (!readBit(bus, bit)) return false;
    if (bit) b |= (uint8_t)(1 << i);
  }
  return writeBit(bus, !ack);
}

SoftI2CStatus softI2cWriteRead(SoftI2C& bus, uint8_t addr,
                               const uint8_t* wbuf, size_t wlen,
                               uint8_t* rbuf, size_t rlen) {
  bool acked = false;

  if (wlen > 0 || rlen == 0) {
    if (!doStart(bus)) { doStop(bus); return SoftI2CStatus::kTimeout; }
    if (!writeByte(bus, (uint8_t)(addr << 1), acked)) {
      doStop(bus); return SoftI2CStatus::kTimeout;
    }
    if (!acked) { doStop(bus); return SoftI2CStatus::kNackAddr; }
    for (size_t i = 0; i < wlen; i++) {
      if (!writeByte(bus, wbuf[i], acked)) {
        doStop(bus); return SoftI2CStatus::kTimeout;
      }
      if (!acked) { doStop(bus); return SoftI2CStatus::kNackData; }
    }
  }

  if (rlen > 0) {
    // Repeated START (or first START for a pure read).
    if (!doStart(bus)) { doStop(bus); return SoftI2CStatus::kTimeout; }
    if (!writeByte(bus, (uint8_t)((addr << 1) | 1), acked)) {
      doStop(bus); return SoftI2CStatus::kTimeout;
    }
    if (!acked) { doStop(bus); return SoftI2CStatus::kNackAddr; }
    for (size_t i = 0; i < rlen; i++) {
      if (!readByte(bus, rbuf[i], i + 1 < rlen)) {
        doStop(bus); return SoftI2CStatus::kTimeout;
      }
    }
  }

  doStop(bus);
  return SoftI2CStatus::kOk;
}
