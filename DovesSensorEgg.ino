/* ==========================================================================
   DovesSensorEgg - wireless EGT pod (PW-ADV-1 broadcaster)
   XIAO nRF52840 + Adafruit MCP9600 + I2C OLED + button

   Boot -> bird splash -> scan I2C -> BLE up (MAC shown) -> live temp on
   screen + serial + BLE advertising.

   BLE: pure BROADCASTER. EGT + cold junction are packed into a 14-byte
   Manufacturer Specific Data AD structure ("PW-ADV-1") and advertised at
   ~10 Hz under the name "PWEGT". The egg never accepts a connection; the
   DovesDataLogger receives the broadcasts with a passive scan. nRF
   Connect on a phone doubles as a live hex debugger (mfg data starts
   FF FF 50 57, bytes 12-13 increment).

   Button on D1: short press toggles C / F on the debug screen; long
   press (>1 s) opens a 30 s pairing window (advertises flags bit0 -
   informational, the logger pairs by MAC/magic); held 10 s -> deep
   sleep (nRF52 System OFF, display + MCP + radio all down, ~uA - the
   enclosure has no power switch). Wake = hold the button 5 s: the
   press wakes the chip, but boot goes straight back to sleep unless
   the button stays held - a pocket bump never lights the screen.
   Deep sleep needs USB UNPLUGGED: the nRF52840 cannot hold System OFF
   with bus power present, it just resets.

   THE OLED BUS RUNS AT 100 kHz (OLED_I2C_HZ) and that number is load
   bearing, not a preference. This core's Wire spins on TWIM events with
   no timeout - see wireLinesIdleHigh() - so a transfer that goes wrong
   badly enough never returns, and the watchdog turns that into an exact
   8 s reboot cycle. Raising the bus to 400 kHz on the nRF52's internal
   ~13 k pullups is what caused the 2026-07-26 boot loop. Note the speed
   must be given to the DISPLAY DRIVER too: Adafruit_SSD1306/SH110X
   default to 400 kHz and re-assert it around every transfer.

   NOTHING IN BRING-UP REBOOTS THE POD (2026-07-26). A missing or
   unhealthy MCP9600 boots degraded - radio and screen up, readings on
   the wire's invalid sentinel, sensor retried by loop()'s recovery tick
   - because rebooting is the one recovery that cannot fix a failing
   boot. If the pod resets BOOT_SAFE_AFTER times anyway without a
   healthy run in between, it comes up in SAFE MODE: watchdog held off
   until loop() is running, boot screens and deep sleep disabled. See
   the boot-loop breaker note below.

   WIRING (changed 2026-07-20): the MCP9600 lives on its OWN bit-banged
   I2C bus - SDA=D2, SCL=D3, ~50 kHz, 4.7k pullups recommended - so a
   wedged sensor can't take the OLED down and every transaction has a
   timeout (see soft_i2c.h). The OLED stays on hardware Wire (D4/D5),
   now at 400 kHz. The Adafruit MCP9600 library is no longer used; the
   register driver is mcp9600.{h,cpp} over the soft bus.

   Libraries (Arduino Library Manager):
     Adafruit SH110X          (or Adafruit SSD1306 if you flip USE_SH1106 to 0)
     Adafruit GFX Library     <- must be current; SH110X needs Adafruit_GrayOLED
     Adafruit BusIO
     Bluefruit nRF52 (built into the Seeed XIAO nRF52840 board package)
   ========================================================================== */

#include <Wire.h>
#include <bluefruit.h>
#include <avr/dtostrf.h>     // dtostrf() needs its own header on this core

#include "images.h"          // boot splash (the bird, from DovesDataLogger)
#include "pw_adv_encode.h"   // PW-ADV-1 payload builder (host-tested)
#include "soft_i2c.h"        // timeout-capable bit-banged bus (MCP9600)
#include "mcp9600.h"         // register driver over the soft bus
#include "mcp9600_regs.h"    // host-tested register codecs

// ---- SCREEN DRIVER: flip this if the boot splash geometry looks wrong ------
#define USE_SH1106   0        // 1 = SH1106 (1.3")    0 = SSD1306 (0.96")
#define OLED_W     128
#define OLED_H      64

// OLED bus speed. 100 kHz, and passed to the display driver as well as to
// Wire — see the boot-loop note below. Do not raise this without fitting
// real pullups first: TwoWire::begin() on this core configures the nRF52's
// INTERNAL ~13 k pullups, and 13 k against a few tens of pF of flying lead
// gives a rise time near 1 us. That fits inside 100 kHz's 1000 ns budget
// and blows straight through 400 kHz's 300 ns.
//
// Both numbers matter, and the second one is easy to miss: Adafruit_SSD1306
// (and SH110X) default to clkDuring=400000 and re-assert it with
// wire->setClock() around EVERY transfer, so a display constructed without
// these arguments runs at 400 kHz no matter what the sketch asked Wire for.
#define OLED_I2C_HZ  100000UL
// ---------------------------------------------------------------------------

#if USE_SH1106
  #include <Adafruit_SH110X.h>
  Adafruit_SH1106G oled(OLED_W, OLED_H, &Wire, -1, OLED_I2C_HZ, OLED_I2C_HZ);
  #define PX_ON   SH110X_WHITE
  #define PX_OFF  SH110X_BLACK
  #define DRV_NAME "SH1106"
  #define DISPLAY_OFF() oled.oled_command(SH110X_DISPLAYOFF)
#else
  #include <Adafruit_SSD1306.h>
  Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire, -1, OLED_I2C_HZ, OLED_I2C_HZ);
  #define PX_ON   SSD1306_WHITE
  #define PX_OFF  SSD1306_BLACK
  #define DRV_NAME "SSD1306"
  #define DISPLAY_OFF() oled.ssd1306_command(SSD1306_DISPLAYOFF)
#endif

#define BTN_PIN     D1
// MCP9600's dedicated soft bus (see header note). D2/D3 are free pins:
// D1 = button, D4/D5 = hardware Wire (OLED).
#define MCP_SDA_PIN D2
#define MCP_SCL_PIN D3
#define READ_MS    100        // MCP @16-bit converts in ~63-80ms
#define ADV_MS     250        // payload rebuild tick (see ADV_INTERVAL_UNITS)
#define DRAW_MS    200
#define SERIAL_MS  250

// Advertising interval: 179 x 0.625 ms = 111.875 ms — DELIBERATELY not the
// spec's 100 ms. The logger scans on a 100 ms cycle with a 60 ms window;
// equal periods phase-lock, and when the egg's packets parked in the deaf
// 40 ms the logger lost it for seconds at a time (first bench soak). An
// off-100 interval sweeps the phase through the window continuously, so
// the worst-case blind stretch is bounded at ~1 s instead of unbounded.
// Rebuilding the payload every ADV_MS (not every READ_MS) lets this
// SoftDevice interval - not our loop tick - govern when packets air.
#define ADV_INTERVAL_UNITS 179

#define BTN_LONG_MS   1000    // long press -> pairing window
#define PAIR_WINDOW_MS 30000  // flags bit0 stays set this long

// ---- deep sleep (System OFF — no power switch on the enclosure) ----------
#define SLEEP_HOLD_MS 10000   // button held this long -> deep sleep
#define WAKE_HOLD_MS   5000   // wake press must be HELD this long to boot
#define SLEEP_HINT_MS  2000   // hold this long -> on-screen sleep countdown
#define BTN_STABLE_MS    16   // button level must hold this long to count

// ---- boot-loop breaker ----------------------------------------------------
// Boot-loop incident (2026-07-26): the pod rebooted forever after a flash
// and only DFU mode could stop it. Three separate reboot paths had been
// added with nothing to break the cycle they formed:
//   * fatal() hard-reset every 30 s, so an MCP9600 that missed its boot
//     handshake rebooted the pod for as long as it stayed missing;
//   * wakeHoldGate() sampled the button ONCE, undebounced, so a bouncing
//     wake press re-entered System OFF mid-press, re-triggered SENSE-LOW
//     and looped at boot speed with nothing ever drawn;
//   * System OFF with USB VBUS present, which the nRF52840 cannot hold —
//     it wakes and resets immediately.
// All three are fixed at the source below, but the pod also needs to be
// able to break ANY future cycle on its own, including one we haven't
// thought of. GPREGRET2 survives every warm reset (watchdog, soft reset,
// pin reset) and System OFF, and is cleared by a real power cycle; the
// Adafruit bootloader owns GPREGRET for the DFU magic and never touches
// GPREGRET2. Boot increments a counter there; a run that stays healthy
// for BOOT_HEALTHY_MS clears it. BOOT_SAFE_AFTER boots without a healthy
// run in between means something in bring-up is cycling, so the pod comes
// up in SAFE MODE: no watchdog until loop() is actually running, no boot
// delays, no deep-sleep gate, everything optional skipped. Safe mode is
// deliberately boring — the point is a pod that sits still, holds its USB
// enumeration and can be re-flashed without the DFU dance.
#define BOOT_TAG_MASK    0xF0
#define BOOT_TAG_VALUE   0xB0  // canary: our counter vs. power-on garbage
#define BOOT_COUNT_MASK  0x0F
#define BOOT_SAFE_AFTER  3     // boots with no healthy run -> safe mode
#define BOOT_HEALTHY_MS  15000 // loop() alive this long -> the boot "took"

// ---- PW-ADV-1 payload (14 bytes, little-endian fields) --------------------
// Bluefruit's addManufacturerData() passes the buffer through RAW - it does
// NOT prepend a company ID - so bytes 0-1 of this array ARE the company ID
// and the logger indexes the array identically. Layout:
//   0-1  company ID FF FF     2-3  magic 'P' 'W'      4  proto version 01
//   5    flags (bit0 pairing, bit1 TC fault)
//   6-7  EGT int16 deci-degC  8-9  CJ int16 deci-degC
//   10   raw MCP9600 STATUS   11   battery stub FF    12-13 sequence
SoftI2C mcpBus = {MCP_SDA_PIN, MCP_SCL_PIN,
                  /*halfPeriodUs=*/10,      // ~50 kHz: slow on purpose (EMI margin)
                  /*stretchTimeoutUs=*/5000};
Mcp9600 mcp;

uint8_t  oledAddr = 0;
bool     oledOK   = false;
bool     showF    = true;
uint32_t nRead    = 0;

bool     safeMode  = false;   // boot-loop breaker tripped (see BOOT_* above)
uint8_t  bootCount = 0;       // consecutive boots without a healthy run

uint16_t advSeq    = 0;
uint32_t pairUntil = 0;       // millis deadline; 0 = pairing window closed
bool     advOK     = false;   // last Advertising.start() result
uint32_t advFails  = 0;       // consecutive-rebuild failure count (debug)

// c2f() lives in pw_adv_encode (host-tested) — used by the debug screen.
using pw_adv::c2f;

// -------------------------------------------------- hardware watchdog
// Field incident (2026-07-19): ~3-4 h into a session the app hung — prime
// suspect a blocking MCP9600 I2C transaction wedged by ignition EMI (the
// probe lead is an antenna, the MCP9600 clock-stretches, and this core's
// Wire has no timeout) — while the SoftDevice kept rebroadcasting the
// last-set advertising payload forever. Result: a zombie egg beaconing a
// frozen value that the logger initially read as a live link.
//
// The nRF52840 hardware WDT reboots out of ANY such hang: it is fed once
// per loop() pass, so a wedge anywhere (I2C, OLED, BLE) trips it within
// WDT_TIMEOUT_S. Boot then runs i2cBusClear(), which is exactly the
// recovery a wedged bus needs, and the sequence counter restarting from 0
// is how the logger's zombie detection sees the egg come back to life.
// Once started the WDT cannot be stopped or re-configured.
//
// In SAFE MODE the watchdog is NOT armed until loop() is running. A pod
// that is already cycling must not be rebooted again by the very timer
// meant to protect a healthy run — an un-armed watchdog turns a bring-up
// hang into a device that sits still, keeps its USB enumeration and can
// be re-flashed, instead of one that loops out of reach.
#define WDT_TIMEOUT_S   8      // bring-up feeds through bootDelay(), see below

void wdtSetup() {
  NRF_WDT->CONFIG = WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos;
  NRF_WDT->CRV    = WDT_TIMEOUT_S * 32768;              // 32768 Hz LFCLK
  NRF_WDT->RREN   = WDT_RREN_RR0_Enabled << WDT_RREN_RR0_Pos;
  NRF_WDT->TASKS_START = 1;
}

// Harmless before TASKS_START — a stopped watchdog ignores the reload.
void wdtPet() { NRF_WDT->RR[0] = WDT_RR_RR_Reload; }

// Every deliberate wait in bring-up goes through here instead of delay():
// the feed comes with the wait, so no hand-placed wdtPet() can be missed
// or drift out of range when a boot screen grows. Bring-up is the one
// place the pod blocks for seconds at a time, and a missed feed there is
// a permanent boot loop, not a one-off reboot.
void bootDelay(uint32_t ms) {
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    wdtPet();
    delay(10);
  }
  wdtPet();
}

// Name each stage of bring-up on the way IN, flushed immediately. When the
// pod hangs, the last line on the wire is the step that hung — the 2026-07-26
// loop had to be pinned down by measuring the reboot period against
// WDT_TIMEOUT_S and reading the core's Wire source, because boot printed
// nothing between the banner and a step three calls later. Serial.flush()
// matters: without it the tail of the CDC FIFO dies with the hang.
void bootStep(const char* what) {
  Serial.print("[boot] "); Serial.println(what);
  Serial.flush();
}

// Read + clear the sticky reset-reason register. Callers test the bits:
// DOG = previous run hung and the watchdog rebooted it; OFF = a GPIO
// (the button) woke the chip out of System OFF deep sleep.
uint32_t captureResetReason() {
  uint32_t rr = NRF_POWER->RESETREAS;
  NRF_POWER->RESETREAS = rr;   // sticky until written — clear for next boot
  return rr;
}

// -------------------------------------------------- restricted registers
// NRF_POWER is restricted once the SoftDevice is enabled, so both helpers
// dispatch the same way systemOff() already does.
bool sdEnabled() {
  uint8_t on = 0;
  (void)sd_softdevice_is_enabled(&on);
  return on != 0;
}

// True when USB bus power is present. The nRF52840 CANNOT hold System OFF
// with VBUS up: the USB regulator wakes it straight back out and the pod
// resets instead of sleeping. Every sleep path checks this first.
bool vbusPresent() {
  uint32_t status = 0;
  if (sdEnabled()) {
    if (sd_power_usbregstatus_get(&status) != NRF_SUCCESS) return false;
  } else {
    status = NRF_POWER->USBREGSTATUS;
  }
  return (status & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
}

// -------------------------------------------------- boot-loop counter
uint8_t bootRegGet() {
  uint32_t v = 0;
  if (sdEnabled()) { (void)sd_power_gpregret_get(1, &v); }
  else             { v = NRF_POWER->GPREGRET2; }
  return (uint8_t)v;
}

void bootRegSet(uint8_t v) {
  if (sdEnabled()) {
    (void)sd_power_gpregret_clr(1, 0xFF);
    (void)sd_power_gpregret_set(1, v);
  } else {
    NRF_POWER->GPREGRET2 = v;
  }
}

// Count this boot. Returns the number of consecutive boots that have not
// yet been confirmed healthy (1 = this is the first).
uint8_t bootCountBump() {
  const uint8_t reg = bootRegGet();
  uint8_t n = ((reg & BOOT_TAG_MASK) == BOOT_TAG_VALUE) ? (reg & BOOT_COUNT_MASK) : 0;
  if (n < BOOT_COUNT_MASK) n++;
  bootRegSet((uint8_t)(BOOT_TAG_VALUE | n));
  return n;
}

// The run stuck: forget the streak so the next boot starts clean.
void bootCountClear() { bootRegSet(BOOT_TAG_VALUE); }

// -------------------------------------------------- debounced button level
// A single digitalRead() is not enough to decide anything that reboots the
// pod: the contact bounces for milliseconds and the internal ~13 k pull-up
// needs a beat to pull the line up after pinMode(). The undebounced read in
// the old wakeHoldGate() is exactly how a wake press turned into a System
// OFF <-> wake ping-pong. (btnEvent() below keeps its own edge debounce for
// the C/F and pairing presses — those decide nothing that costs a reboot.)
bool btnReleasedStable() {
  const uint32_t t0 = millis();
  while (millis() - t0 < BTN_STABLE_MS) {
    if (digitalRead(BTN_PIN) == LOW) return false;
    delay(1);
  }
  return true;
}

// Block until the button has been released and stayed released. Fed, so a
// button held (or stuck) forever is the watchdog's problem, not a hang.
void btnWaitRelease() {
  while (!btnReleasedStable()) { wdtPet(); delay(10); }
}

// -------------------------------------------------- deep sleep (System OFF)
// Mirrors the DovesDataLogger's shutdownSystemOff() on the same MCU. Wake
// is a full reset (RESETREAS.OFF records the cause); the WDT halts with
// every other clock and re-arms on the fresh boot. Does not return.
void systemOff() {
  wdtPet();
  // Wake source: SENSE-LOW with pull-up on the button (active low). Pull +
  // SENSE config is retained in System OFF; P-number via the pin map.
  nrf_gpio_cfg_sense_input(g_ADigitalPinMap[BTN_PIN],
                           NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
  // A set LATCH bit is a pending DETECT = instant re-wake; clear last.
  NRF_P0->LATCH = 0xFFFFFFFF;
  NRF_P1->LATCH = 0xFFFFFFFF;
  // Cortex-M4F: a pending FPU exception can inhibit low-power entry.
  __set_FPSCR(__get_FPSCR() & ~0x9F);
  NVIC_ClearPendingIRQ(FPU_IRQn);
  // NRF_POWER is restricted while the SoftDevice is enabled (it is, once
  // bleSetup() has run — but the early wake-hold gate sleeps before that).
  if (sdEnabled()) {
    sd_power_reset_reason_clr(0xFFFFFFFF);
    (void)sd_power_system_off();
  } else {
    NRF_POWER->RESETREAS = 0xFFFFFFFF;
    NRF_POWER->SYSTEMOFF = 1;
  }
  // Only reachable in emulated System OFF (debugger attached).
  while (true) { __WFE(); }
}

// Full teardown then System OFF: radio silent, MCP9600 in shutdown mode,
// display dark. Returns ONLY if USB bus power blocks System OFF.
bool enterDeepSleep() {
  // On USB, SYSTEMOFF is not sleep — the regulator wakes the chip straight
  // back out and the pod resets. Refuse and say so rather than handing the
  // user an unexplained reboot every time they hold the button on the bench.
  if (vbusPresent()) {
    Serial.println("USB power present - System OFF would reset instantly; staying awake");
    if (oledOK) {
      oled.clearDisplay();
      oled.setTextSize(1); oled.setTextColor(PX_ON, PX_OFF);
      oled.setCursor(0, 4);  oled.print("CAN'T SLEEP");
      oled.setCursor(0, 26); oled.print("USB power is on.");
      oled.setCursor(0, 40); oled.print("Unplug, then hold");
      oled.setCursor(0, 52); oled.print("the button again.");
      oled.display();
    }
    btnWaitRelease();
    bootDelay(2000);
    return false;
  }

  Serial.println("deep sleep - hold button 5 s to wake");
  Bluefruit.Advertising.stop();
  mcpShutdown(mcp);                        // MCP9600 shutdown mode (~uA)
  if (oledOK) {
    oled.clearDisplay();
    oled.display();
    DISPLAY_OFF();
  }
  // Held button = SENSE satisfied = instant re-wake; wait for a release
  // that has actually settled, or the press bounces us straight back up.
  btnWaitRelease();
  delay(50);   // contact settle
  systemOff();
  return true; // unreachable outside emulated System OFF
}

// System OFF button wake: require a deliberate WAKE_HOLD_MS hold before
// booting. Released early -> straight back to sleep. Nothing is drawn and
// no peripheral is touched, so a pocket bump never lights the screen.
//
// Every "released" decision here is debounced. The old version sampled the
// pin once: one stray HIGH in the bounce of the wake press dropped the pod
// back into System OFF while the button was still physically down, SENSE-LOW
// fired again immediately, and the pod cycled at boot speed — no screen, no
// USB, DFU the only way back in.
void wakeHoldGate() {
  // Woken while on USB: System OFF cannot hold anyway, so gating the boot
  // on a 5 s hold would just reset in a circle. Boot normally.
  if (vbusPresent()) return;

  delay(20);   // pull-up settle + first bounce, before anything is decided
  const uint32_t t0 = millis();
  while (millis() - t0 < WAKE_HOLD_MS) {
    wdtPet();
    if (btnReleasedStable()) {
      systemOff();   // genuinely let go — back to sleep
      return;        // unreachable outside emulated System OFF
    }
    delay(10);
  }
  // Confirmed hold. Swallow the rest of it so the press doesn't fall
  // through into btnEvent() as a C/F toggle or pairing long-press.
  btnWaitRelease();
}

// -------------------------------------------------- I2C bus clear
// A reset mid-transaction (e.g. reflashing while the MCP was being read)
// can leave a slave driving SDA low - the address scan still half-works
// but register reads fail. Standard recovery: clock SCL 9 times with SDA
// released, then issue a STOP. Must run BEFORE Wire.begin().
void i2cBusClear() {
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, OUTPUT);
  for (int i = 0; i < 9; i++) {
    digitalWrite(SCL, LOW);  delayMicroseconds(10);
    digitalWrite(SCL, HIGH); delayMicroseconds(10);
  }
  // STOP condition: SDA low->high while SCL high.
  pinMode(SDA, OUTPUT);
  digitalWrite(SDA, LOW);  delayMicroseconds(10);
  digitalWrite(SCL, HIGH); delayMicroseconds(10);
  digitalWrite(SDA, HIGH); delayMicroseconds(10);
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
}

// -------------------------------------------------- hardware Wire safety
// THE HARDWARE Wire ON THIS CORE CAN HANG FOREVER, AND THAT HANG IS A BOOT
// LOOP. Wire_nRF52.cpp spins on raw TWIM events with no timeout:
//
//     while(!_p_twim->EVENTS_TXSTARTED && !_p_twim->EVENTS_ERROR);
//     while(!_p_twim->EVENTS_LASTTX    && !_p_twim->EVENTS_ERROR);
//     while(!_p_twim->EVENTS_STOPPED);          <- no error escape at all
//
// If a transfer goes wrong badly enough that the TWIM never raises STOPPED,
// endTransmission() never returns. There is no recovery from inside the
// app: the watchdog fires 8 s later, boot runs into the same transfer, and
// the pod cycles on an exact 8 s period until someone holds it in DFU. That
// is the 2026-07-26 boot loop, and raising this bus to 400 kHz on internal
// pullups is what walked us into it — scanBus() is the FIRST Wire transfer
// of the boot and the only one the sketch's own setClock() governs (the
// display driver overrides the clock for its own transfers, see
// OLED_I2C_HZ), so it took the change head-on.
//
// So bring-up never enters a Wire call blind. Both lines must read released
// and high, with the pullups on and the TWIM out of the way, before the
// peripheral is allowed to touch them; if they do not, the display is
// dropped and the pod boots without it. A pod that broadcasts with no
// screen is worth infinitely more than one that loops.
bool wireLinesIdleHigh() {
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  delayMicroseconds(200);          // pullup + line capacitance settle
  return digitalRead(SDA) == HIGH && digitalRead(SCL) == HIGH;
}

// -------------------------------------------------- OLED probe (Wire)
// OLED only. The MCP9600 is NOT probed here anymore: blind START/STOP
// probes are exactly what community experience says can confuse its
// interface state machine, and it now lives on its own bus with a
// proper ID handshake (mcpDetect). A bare probe is fine for the SSD1306.
void scanBus() {
  Serial.println("\n--- OLED probe (Wire) ---");
  for (uint8_t a = 0x3C; a <= 0x3D; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() != 0) continue;
    Serial.print("  OLED at 0x"); Serial.println(a, HEX);
    if (!oledAddr) oledAddr = a;
  }
  if (!oledAddr) Serial.println("  no OLED -> serial only");
}

// -------------------------------------------------- debounced button
// Short press (release < BTN_LONG_MS) -> 1. Long press fires 2 ONCE while
// still held at the threshold; the eventual release is then swallowed.
uint8_t btnEvent() {
  static bool     prev      = true;   // INPUT_PULLUP: idle HIGH
  static uint32_t tLast     = 0;
  static uint32_t tDown     = 0;
  static bool     longFired = false;

  bool now = digitalRead(BTN_PIN);

  if (now == prev) {
    // Held down past the threshold -> long press, once.
    if (now == LOW && !longFired && millis() - tDown >= BTN_LONG_MS) {
      longFired = true;
      return 2;
    }
    return 0;
  }
  if (millis() - tLast < 40) return 0;   // debounce the edge
  tLast = millis();
  prev  = now;

  if (now == LOW) {                      // press edge
    tDown     = millis();
    longFired = false;
    return 0;
  }
  return longFired ? 0 : 1;              // release edge -> short press
}

// -------------------------------------------------- PW-ADV-1 encode
// Byte layout + sentinel rules live in pw_adv_encode.{h,cpp} (pure,
// host-tested against the logger's parser fixture — the wire contract).

// Rebuild the advertising data with fresh sensor values. stop -> clear ->
// rebuild -> start is ugly and correct: Bluefruit has no supported
// in-place advertising-data mutation path. Do not invent one.
// start()'s result is tracked: a failed restart leaves the radio silent
// until the next tick, and a RUN of failures is the one egg-side fault
// that looks exactly like a dead egg to the logger - so it's surfaced on
// the debug screen ("ADV!") and serial instead of being swallowed.
void updateAdvertising(float egtC, float cjC, uint8_t st) {
  uint8_t payload[pw_adv::kPayloadLen];
  advSeq++;                                  // one increment per adv update
  pw_adv::buildPayload(payload, egtC, cjC, st,
                       millis() < pairUntil, advSeq);

  Bluefruit.Advertising.stop();
  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addManufacturerData(payload, pw_adv::kPayloadLen);
  Bluefruit.Advertising.addName();           // "PWEGT" - keeps nRF Connect useful
  advOK = Bluefruit.Advertising.start(0);    // 0 = advertise forever
  if (!advOK) {
    advFails++;
    Serial.print("ADV restart failed, count "); Serial.println(advFails);
  }
}

// -------------------------------------------------- BLE bringup
void bleSetup() {
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("PWEGT");

  // Nothing ever connects to the egg. If this line fails to compile on
  // the installed core version, delete it and move on.
  Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED);

  Bluefruit.Advertising.setInterval(ADV_INTERVAL_UNITS, ADV_INTERVAL_UNITS);
  Bluefruit.Advertising.setFastTimeout(0);      // never drop to the slow interval

  // Print our MAC (human order, MSB first) so it can be copied into the
  // logger's SENSOREGG_MAC #define for strict pairing.
  uint8_t mac[6];
  Bluefruit.getAddr(mac);                       // returns LSB first
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
  Serial.print("BLE up, MAC "); Serial.println(macStr);

  if (oledOK && !safeMode) {
    oled.clearDisplay();
    oled.setTextSize(1); oled.setTextColor(PX_ON, PX_OFF);
    oled.setCursor(0, 0);  oled.print("BLE: PWEGT");
    oled.setCursor(0, 18); oled.print("MAC (for logger):");
    oled.setCursor(0, 32); oled.print(macStr);
    oled.display();
    bootDelay(3000);   // fed throughout — see bootDelay()
  }
}

// -------------------------------------------------- sensor bring-up failure
// NOT fatal, and deliberately NOT a reboot. The old fatal() hard-reset the
// pod every 30 s until the MCP9600 answered, which is a boot loop whenever
// the sensor is absent, mis-wired or just slow to come back — and rebooting
// is the one recovery that cannot possibly help, because boot is what keeps
// failing. It also made the pod near-impossible to re-flash: the USB CDC
// port vanished every 30 s.
//
// Everything the reboot was supposed to buy already exists further down:
// reads return the 0x8000 wire sentinel (logger shows "---", never a stale
// value), and loop()'s recovery path re-runs the bus clear, the detect and
// the mode-cycle every 2 s for as long as the fault lasts. So boot through
// it: radio up, screen up, sensor marked absent and retried in place.
void sensorFailed(const char* msg) {
  Serial.print("SENSOR FAULT: "); Serial.println(msg);
  Serial.println("booting degraded - EGT reports invalid, runtime recovery keeps retrying");
  mcp.addr = 0;                 // reads -> NaN / 0xFF -> wire sentinel
  if (oledOK) {
    oled.clearDisplay();
    oled.setTextSize(2); oled.setTextColor(PX_ON, PX_OFF);
    oled.setCursor(0, 0);  oled.print("NO SENSOR");
    oled.setTextSize(1);
    oled.setCursor(0, 24); oled.print(msg);
    oled.setCursor(0, 40); oled.print("check MCP9600 on");
    oled.setCursor(0, 52); oled.print("D2/D3 - retrying");
    oled.display();
  }
  bootDelay(3000);
}

// -------------------------------------------------- sleep countdown screen
void drawSleepCountdown(uint32_t secondsLeft) {
  if (!oledOK) return;
  oled.clearDisplay();
  oled.setTextColor(PX_ON, PX_OFF);
  oled.setTextSize(2);
  oled.setCursor(10, 8);
  oled.print("SLEEP in");
  oled.setTextSize(3);
  oled.setCursor(56, 32);
  oled.print(secondsLeft);
  oled.display();
}

// -------------------------------------------------- main screen
void drawScreen(float egtC, float cjC, uint8_t st) {
  if (!oledOK) return;
  float egt = showF ? c2f(egtC) : egtC;
  float cj  = showF ? c2f(cjC)  : cjC;

  oled.clearDisplay();
  oled.setTextColor(PX_ON, PX_OFF);

  // Bicolor panel: rows 0-15 are the yellow band. Header = the info:
  // address, data-ready (STATUS bit6), probe fault (STATUS bit4), and the
  // advertising packet counter (right-aligned). The scale lives on the CJ
  // line. 0xFF status = bus fault -> flags unknown, don't fake health.
  const bool stValid = (st != 0xFF);
  oled.setTextSize(1);
  oled.setCursor(0, 4);
  if (mcp.addr) { oled.print("0x"); oled.print(mcp.addr, HEX); }
  else            oled.print("--");
  oled.setCursor(36, 4);
  oled.print((stValid && (st & 0x40)) ? "RDY" : "--");
  oled.setCursor(66, 4);
  oled.print(!stValid ? "?" : ((st & 0x10) ? "OPEN" : "OK"));
  char seqStr[8];
  snprintf(seqStr, sizeof(seqStr), "#%u", (unsigned)advSeq);
  oled.setCursor(OLED_W - 6 * (int16_t)strlen(seqStr), 4);
  oled.print(seqStr);

  // Blue: just the temperatures, centered. Widths are computed from the
  // formatted strings (dtostrf — this core's printf lacks reliable %f).
  char numBuf[12];
  char lineBuf[20];
  if (isnan(egt)) {
    snprintf(lineBuf, sizeof(lineBuf), "---");
  } else {
    dtostrf(egt, 1, 1, numBuf);
    snprintf(lineBuf, sizeof(lineBuf), "%s", numBuf);
  }
  oled.setTextSize(3);  // 18 px per glyph
  oled.setCursor((OLED_W - 18 * (int16_t)strlen(lineBuf)) / 2, 20);
  oled.print(lineBuf);

  if (isnan(cj)) {
    snprintf(lineBuf, sizeof(lineBuf), "CJ ---");
  } else {
    dtostrf(cj, 1, 1, numBuf);
    snprintf(lineBuf, sizeof(lineBuf), "CJ %s %c", numBuf, showF ? 'F' : 'C');
  }
  oled.setTextSize(1);  // 6 px per glyph
  oled.setCursor((OLED_W - 6 * (int16_t)strlen(lineBuf)) / 2, 50);
  oled.print(lineBuf);

  // Transient states borrow the bottom-right corner (normally blank).
  // SAFE outranks the others: it says the pod broke a boot loop to get
  // here, which the user needs to see for longer than a pairing window.
  if (safeMode) {
    oled.setCursor(OLED_W - 24, 56); oled.print("SAFE");
  } else if (!advOK) {
    oled.setCursor(OLED_W - 24, 56); oled.print("ADV!");
  } else if (millis() < pairUntil) {
    oled.setCursor(OLED_W - 24, 56); oled.print("PAIR");
  }

  oled.display();
}

// -------------------------------------------------- setup
void setup() {
  // Reset-cause first (the register is sticky), then count the boot, THEN
  // decide whether to arm the WDT. Order matters: the counter has to be
  // bumped before anything that could hang, or a cycle never gets counted
  // and never trips safe mode.
  const uint32_t resetReas = captureResetReason();
  const bool wdtReboot = (resetReas & POWER_RESETREAS_DOG_Msk) != 0;
  const bool offWake   = (resetReas & POWER_RESETREAS_OFF_Msk) != 0;

  bootCount = bootCountBump();
  safeMode  = bootCount >= BOOT_SAFE_AFTER;

  // Healthy boot: arm the watchdog first thing, so a hang anywhere reboots
  // the pod instead of zombifying it. Safe mode defers it to loop() — see
  // the note by WDT_TIMEOUT_S.
  if (!safeMode) wdtSetup();

  pinMode(BTN_PIN, INPUT_PULLUP);
  delay(5);   // let the internal pull-up actually pull the line up

  // Woken out of deep sleep by the button: demand the 5 s hold before
  // anything else powers up. (A watchdog reboot skips this — after a hang
  // the pod must come back broadcasting without a human. So does safe
  // mode: the sleep path is itself a reboot path, and a pod that is
  // already cycling must not be handed another way to go dark.)
  if (offWake && !wdtReboot && !safeMode) wakeHoldGate();

  Serial.begin(115200);
  while (!Serial && millis() < 3000) { wdtPet(); }  // don't block on battery

  Serial.println("\nDovesSensorEgg - wireless EGT pod (PW-ADV-1)");
  Serial.print("boot #"); Serial.print(bootCount);
  Serial.print(" since last healthy run, RESETREAS 0x");
  Serial.println(resetReas, HEX);
  if (wdtReboot) {
    // The previous run hung (I2C wedge under ignition EMI is the known
    // suspect) and the watchdog pulled us out. The i2cBusClear() below is
    // exactly the recovery that wedge needs.
    Serial.println("!! WATCHDOG REBOOT - previous run hung");
  }
  if (safeMode) {
    Serial.println("!! SAFE MODE - repeated reboots without a healthy run.");
    Serial.println("   Watchdog held off until loop() runs; display NOT brought");
    Serial.println("   up (the hardware Wire is the one call that can hang with");
    Serial.println("   no way back); boot screens and deep sleep disabled.");
    Serial.println("   Power-cycle after a good run to clear.");
  }

  // ---- display bring-up, the one part of boot that can hang unrecoverably.
  // Safe mode skips it outright: the hardware Wire is the only call in this
  // sketch with no way back, so a pod that has been cycling gets brought up
  // without it. Serial and BLE do not need the display, and a pod on the air
  // with a dark screen can at least be diagnosed and re-flashed.
  bootStep("display bring-up");
  if (safeMode) {
    Serial.println("  safe mode - display skipped (hardware Wire not entered)");
  } else {
    i2cBusClear();             // recover a slave wedged by a mid-read reset
    bool linesOK = wireLinesIdleHigh();
    if (!linesOK) {
      Serial.println("  OLED bus not idle-high - re-running bus clear");
      i2cBusClear();
      linesOK = wireLinesIdleHigh();
    }
    if (!linesOK) {
      // Entering the TWIM now is the hang. Refuse, and boot without it.
      Serial.println("  !! OLED bus STILL held low (SDA/SCL) - skipping display.");
      Serial.println("     Check D4/D5 wiring and pullups; serial + BLE only.");
    } else {
      Wire.begin();
      Wire.setClock(OLED_I2C_HZ);
      delay(100);
      scanBus();

      if (oledAddr) {
      #if USE_SH1106
        oledOK = oled.begin(oledAddr, true);
      #else
        oledOK = oled.begin(SSD1306_SWITCHCAPVCC, oledAddr);
      #endif
        Serial.print("  OLED "); Serial.print(DRV_NAME);
        Serial.println(oledOK ? " init ok" : " init FAILED");
      } else {
        Serial.println("  no OLED on bus - serial only");
      }
    }
  }

  // MCP9600 on its own timeout-capable soft bus: proper ID handshake at
  // each candidate address (retried — a mid-conversion NACK is normal and
  // must never read as "absent"), never a blind probe.
  bootStep("MCP9600 detect (soft bus D2/D3)");
  softI2cBegin(mcpBus);
  softI2cBusClear(mcpBus);
  mcp.bus = &mcpBus;

  // Safe mode does one quick pass instead of three retried ones: bring-up
  // is what keeps failing, so it gets the shortest path to loop(), where
  // the recovery tick retries the sensor anyway.
  const int detectAttempts = safeMode ? 1 : 3;
  bool mcpOK = false;
  for (int attempt = 1; attempt <= detectAttempts && !mcpOK; attempt++) {
    mcpOK = mcpDetect(mcp, /*triesPerAddr=*/safeMode ? 1 : 3, /*gapMs=*/20);
    if (!mcpOK) {
      Serial.print("MCP detect attempt "); Serial.print(attempt);
      Serial.println(" failed (soft bus D2/D3)");
      softI2cBusClear(mcpBus);
      bootDelay(200);
    }
    wdtPet();
  }

  // The boot screens are ~7.5 s of blocking bring-up. Safe mode skips them
  // outright — they are the longest stretch where a wedged OLED bus can
  // park the pod, and nothing on them is worth another cycle.
  if (oledOK && !safeMode) {
    // Boot splash — the bird, same art as the datalogger.
    oled.clearDisplay();
    oled.drawBitmap(0, 0, image_data_bird1, 128, 64, PX_ON);
    oled.display();
    bootDelay(2500);

    oled.clearDisplay();
    oled.setTextSize(1); oled.setTextColor(PX_ON, PX_OFF);
    oled.setCursor(0, 0);  oled.print("I2C SCAN");
    oled.setCursor(0, 18); oled.print("OLED 0x"); oled.print(oledAddr, HEX);
    oled.setCursor(0, 32); oled.print("MCP  ");
    if (mcp.addr) { oled.print("0x"); oled.print(mcp.addr, HEX); }
    else            oled.print("NOT FOUND");
    if (wdtReboot) { oled.setCursor(0, 54); oled.print("WDT RESET"); }
    oled.display();
    bootDelay(2000);
  }
  // (No safe-mode screen: safe mode never brought the display up. It says
  // what it is on serial, which is the interface that still works.)

  // Mode-cycle (Shutdown -> Normal, the chip's nearest thing to a reset
  // command) + full config with read-back verify.
  bool cfgOK = false;
  if (mcpOK) {
    bootStep("MCP9600 configure");
    for (int attempt = 1; attempt <= 3 && !cfgOK; attempt++) {
      cfgOK = mcpModeCycle(mcp);
      if (!cfgOK) {
        Serial.print("MCP config attempt "); Serial.print(attempt);
        Serial.println(" failed");
        bootDelay(100);
      }
      wdtPet();
    }
  }

  // Neither of these reboots any more — see sensorFailed().
  if (!mcpOK)       sensorFailed("not on bus");
  else if (!cfgOK)  sensorFailed("config failed");
  else { Serial.print("MCP9600 up @0x"); Serial.println(mcp.addr, HEX); }

  bootStep("BLE bring-up");
  bleSetup();

  bootStep("setup complete - entering loop()");
  Serial.println("D1 short = C/F toggle, long = pairing window\n");
}

// -------------------------------------------------- loop
void loop() {
  wdtPet();  // sole feed point: a wedge ANYWHERE below reboots the pod

  // Safe mode deferred arming the watchdog until bring-up was survived —
  // now that we are actually looping, it is safe (and wanted) again.
  static bool wdtArmedLate = false;
  if (safeMode && !wdtArmedLate) {
    wdtArmedLate = true;
    wdtSetup();
    wdtPet();
    Serial.println("safe mode: bring-up survived, watchdog armed");
  }

  // The boot "took". Clear the streak so the next boot starts from zero and
  // the pod comes back out of safe mode on its own. Deliberately time-based
  // rather than "reached loop() once": a pod that reboots after 3 s of
  // runtime is still looping, and must still count as one.
  static bool bootConfirmed = false;
  if (!bootConfirmed && millis() >= BOOT_HEALTHY_MS) {
    bootConfirmed = true;
    bootCountClear();
    Serial.println("boot confirmed healthy - reboot streak cleared");
  }

  static uint32_t tRead = 0, tAdv = 0, tDraw = 0, tSer = 0;
  static float    egtC = NAN, cjC = NAN;
  static uint8_t  st = 0;

  uint8_t btn = btnEvent();
  if (btn == 1) {
    showF = !showF;
    Serial.print("unit -> "); Serial.println(showF ? "F" : "C");
    tDraw = 0;                                   // repaint immediately
  } else if (btn == 2) {
    pairUntil = millis() + PAIR_WINDOW_MS;
    Serial.println("pairing window open (30 s)");
    tDraw = 0;
  }

  // Deep-sleep hold: raw level, independent of btnEvent()'s edge logic.
  // (The pairing long-press fires at 1 s on the way to 10 s — harmless,
  // the pod sleeps right after and advertising stops anyway.) The
  // countdown screen below makes the 10 s hold legible from ~2 s in.
  // Safe mode disables it: System OFF is a reboot path, and a pod that has
  // been cycling must not be given another way to disappear before the user
  // has read the screen.
  static uint32_t sleepHoldSince = 0;
  const bool btnDown = digitalRead(BTN_PIN) == LOW;
  if (!btnDown || safeMode) {
    sleepHoldSince = 0;
  } else if (sleepHoldSince == 0) {
    sleepHoldSince = millis();
  } else if (millis() - sleepHoldSince >= SLEEP_HOLD_MS) {
    // Returns only when USB bus power blocks System OFF; it has already
    // waited out the release, so just re-arm the hold.
    (void)enterDeepSleep();
    sleepHoldSince = 0;
    tDraw = 0;
  }

  if (millis() - tRead >= READ_MS) {
    tRead = millis();
    egtC  = mcpReadHotC(mcp);    // NaN on any bus fault — never stale data
    cjC   = mcpReadColdC(mcp);
    st    = mcpReadStatus(mcp);  // 0xFF on fault (sets the TC-fault flag)
    nRead++;

    // Runtime recovery: the soft bus's timeouts turn a wedge into an error
    // we can SEE, so a sick sensor gets fixed in place instead of waiting
    // for the WDT. ~0.5 s of consecutive faults -> bus clear + mode-cycle
    // reconfigure (the chip's "reset"), throttled to one attempt per 2 s.
    // Meanwhile the payload keeps broadcasting the 0x8000 sentinel and the
    // sequence counter keeps advancing — the pod can never zombie again.
    //
    // This is also what carries a pod that booted WITHOUT the sensor (see
    // sensorFailed()): with no address yet, recovery re-runs detection
    // first, so a sensor that shows up late — or a lead reseated in the
    // field — is picked up without a reboot. One try per address and no
    // inter-try gap keeps that pass to a couple of ms, well inside the
    // advertising tick; the boot path is the one that can afford to be
    // patient about a mid-conversion NACK.
    static uint8_t  mcpFaultRun = 0;
    static uint32_t tRecover    = 0;
    const bool faulted = isnan(egtC) && isnan(cjC);
    if (!faulted) {
      mcpFaultRun = 0;
    } else if (mcpFaultRun < 255) {
      mcpFaultRun++;
    }
    if (mcpFaultRun >= 5 && millis() - tRecover >= 2000) {
      tRecover = millis();
      softI2cBusClear(mcpBus);
      if (mcp.addr == 0) {
        if (mcpDetect(mcp, /*triesPerAddr=*/1, /*gapMs=*/0)) {
          Serial.print("MCP9600 found at runtime @0x");
          Serial.println(mcp.addr, HEX);
        }
      } else {
        Serial.println("MCP fault run -> bus clear + mode cycle");
      }
      if (mcp.addr != 0 && mcpModeCycle(mcp)) {
        Serial.println("MCP recovered in place");
        mcpFaultRun = 0;
      }
    }
  }

  // Payload rebuild on its own slower tick, so between rebuilds the
  // SoftDevice advertises autonomously at ADV_INTERVAL_UNITS - the
  // de-aliasing interval, not our loop cadence, controls the airtime.
  // ~4 Hz payload refresh is still far inside the probe's thermal
  // bandwidth (time constant 0.5-3 s).
  if (millis() - tAdv >= ADV_MS) {
    tAdv = millis();
    updateAdvertising(egtC, cjC, st);
  }

  if (millis() - tDraw >= DRAW_MS) {
    tDraw = millis();
    if (btnDown && sleepHoldSince != 0 &&
        millis() - sleepHoldSince >= SLEEP_HINT_MS) {
      drawSleepCountdown(
          (SLEEP_HOLD_MS - (millis() - sleepHoldSince) + 999) / 1000);
    } else {
      drawScreen(egtC, cjC, st);
    }
  }

  if (millis() - tSer >= SERIAL_MS) {
    tSer = millis();
    Serial.print("EGT ");  Serial.print(egtC, 2);        Serial.print(" C / ");
    Serial.print(c2f(egtC), 2);                          Serial.print(" F    CJ ");
    Serial.print(cjC, 2);                                Serial.print(" C    ST 0x");
    if (st < 0x10) Serial.print("0");
    Serial.print(st, HEX);
    if (st & 0x10) Serial.print("   [INPUT RANGE - probe open/reversed?]");
    Serial.println();
  }
}
