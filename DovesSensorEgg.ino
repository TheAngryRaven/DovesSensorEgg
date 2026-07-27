/* ==========================================================================
   DovesSensorEgg - wireless EGT pod (PW-ADV-2 broadcaster)
   XIAO nRF52840 + Adafruit MCP9600 + I2C OLED + button

   Boot -> bird splash -> scan I2C -> BLE up (MAC shown) -> live temp on
   screen + serial + BLE advertising.

   BLE: pure BROADCASTER. EGT + cold junction + aux thermistor + battery
   are packed into a 16-byte Manufacturer Specific Data AD structure
   ("PW-ADV-2") and advertised at ~10 Hz under the name "PWEGT". The egg
   never accepts a connection; the DovesDataLogger receives the
   broadcasts with a passive scan. nRF Connect on a phone doubles as a
   live hex debugger (mfg data starts FF FF 50 57, bytes 12-13
   increment). COMPAT: the logger's parser still requires version 0x01
   and truncates at 14 bytes — until its v2 round lands, the logger
   DROPS these frames; bench with nRF Connect.

   Button on D1: short press toggles C / F on the debug screen; long
   press (>1 s) opens a 30 s pairing window (advertises flags bit0 -
   informational, the logger pairs by MAC/magic); held 10 s -> deep
   sleep (nRF52 System OFF, display + MCP + radio all down, ~uA - the
   enclosure has no power switch). Wake = hold the button 5 s: the
   press wakes the chip, but boot goes straight back to sleep unless
   the button stays held - a pocket bump never lights the screen.
   Deep sleep needs USB UNPLUGGED: the nRF52840 cannot hold System OFF
   with bus power present, it just resets.

   THE HARDWARE Wire ON THIS CORE CAN HANG FOREVER (no timeout on its
   TWIM event spins - see oledSoftProbe()'s comment), and a hang in boot
   is a watchdog loop. So the display is triple-gated: the OLED must ACK
   a probe on a TEMPORARY timeout-capable soft bus before hardware Wire
   is entered at all; if the previous run died inside display bring-up
   (noinit scratch - the display deadman), the display is skipped for a
   boot; and safe mode never brings it up. The bus runs at 400 kHz
   (OLED_I2C_HZ), passed to the DISPLAY DRIVER too - Adafruit_SSD1306 /
   SH110X re-assert their constructor speed around every transfer.

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
   timeout (see soft_i2c.h). The OLED stays on hardware Wire (D4/D5) at
   100 kHz. The Adafruit MCP9600 library is no longer used; the register
   driver is mcp9600.{h,cpp} over the soft bus.

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
#include "pw_adv_encode.h"   // PW-ADV-2 payload builder (host-tested)
#include "thermistor.h"      // aux NTC codec (host-tested)
#include "battery.h"         // XIAO battery divider codec (host-tested)
#include "soft_i2c.h"        // timeout-capable bit-banged bus (MCP9600)
#include "mcp9600.h"         // register driver over the soft bus
#include "mcp9600_regs.h"    // host-tested register codecs

// ---- SCREEN DRIVER: flip this if the boot splash geometry looks wrong ------
#define USE_SH1106   0        // 1 = SH1106 (1.3")    0 = SSD1306 (0.96")
#define OLED_W     128
#define OLED_H      64

// OLED bus speed: 400 kHz, the panel's rated fast-mode and what this pod
// ran from day one (see below). A full 128x64 frame is ~1 KB; at 400 kHz
// a redraw is ~25 ms of blocking I2C, at 100 kHz it is ~90 ms — at the
// 5 Hz draw tick that difference is nearly half the loop's time budget.
// The OLED breakout carries its own pullups, which is what makes 400 kHz
// legitimate; the nRF52's internal ~13 k pullups alone would be marginal.
//
// History: this was dropped to 100 kHz while chasing the 2026-07-26 boot
// loop, on a rise-time theory that turned out to be WRONG — the loop's
// root cause was a mis-soldered power harness (GND on 3V3, VCC on an IO
// pin), which wedged the first TWIM transfer of every boot. With the
// harness fixed, 400 kHz is back. If a wedge ever reappears, the guards
// hold: the soft probe refuses a dead bus before Wire is entered, and
// the display deadman skips the display one boot after any death inside
// its bring-up.
//
// The speed must ALSO be handed to the display driver — Adafruit_SSD1306
// and SH110X default to clkDuring=400000 and re-assert it with
// wire->setClock() around EVERY transfer, so the constructor args below,
// not Wire.setClock(), are what actually governs the panel's transfers.
#define OLED_I2C_HZ  400000UL
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
// Aux thermistor (as built): 100k FIXED from THERM_PWR_PIN down to the
// sense node, 100k NTC from the sense node to GND, smoothing cap on the
// node. Powered ONLY for the ~13 ms around each read: no idle drain, no
// self-heating, nothing to leak in System OFF. A0 is the XIAO's one
// remaining free analog pin; D6 is a free digital pin doing power-gate
// duty (divider draw ~16 uA max — microscopic against GPIO drive).
#define THERM_PWR_PIN D6
#define THERM_ADC_PIN A0
// Power-gate settle before sampling. Sized for the SMOOTHING CAP on the
// sense node (10 nF default — fit it: the leads run near ignition
// wiring, and it also makes the SAADC's 3 us acquisition legitimate at
// this source impedance, see readThermistorC). The node charges through
// the divider each pulse, worst case tau = R_FIXED x C at cold-NTC
// (Thevenin -> 100k); settle >= ~10 tau or cold readings come out low:
//   no cap -> 3 ms | 10 nF -> 12 | 22 nF -> 25 | 47 nF -> 50 | 100 nF -> 75
// (75 ms stalls one loop tick per second — benign, the SoftDevice keeps
// advertising autonomously between payload rebuilds.)
#define THERM_SETTLE_MS 12
#define READ_MS    100        // MCP @16-bit converts in ~63-80ms
#define THERM_MS  1000        // aux thermistor tick (gated ~4 ms pulse)
#define BATT_MS  30000        // battery tick (divider pulsed, not left on)
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
// and only DFU mode could stop it. Root cause, found three rounds in: a
// mis-soldered power harness (GND on 3V3, VCC on an IO) wedging the
// no-timeout hardware Wire — see the OLED probe note. But the hunt also
// exposed three real latent reboot paths that had been added with nothing
// to break the cycle they could form:
//   * fatal() hard-reset every 30 s, so an MCP9600 that missed its boot
//     handshake rebooted the pod for as long as it stayed missing;
//   * wakeHoldGate() sampled the button ONCE, undebounced, so a bouncing
//     wake press re-entered System OFF mid-press, re-triggered SENSE-LOW
//     and looped at boot speed with nothing ever drawn;
//   * System OFF with USB VBUS present, which the nRF52840 cannot hold —
//     it wakes and resets immediately.
// All three are fixed at the source below, but the pod also needs to be
// able to break ANY future cycle on its own, including one we haven't
// thought of. Boot increments a counter; a run that stays healthy for
// BOOT_HEALTHY_MS clears it. BOOT_SAFE_AFTER boots without a healthy run
// in between means something in bring-up is cycling, so the pod comes up
// in SAFE MODE: no watchdog until loop() is actually running, no boot
// delays, no deep-sleep gate, everything optional skipped. Safe mode is
// deliberately boring — the point is a pod that sits still, holds its USB
// enumeration and can be re-flashed without the DFU dance.
//
// STORAGE (revised 2026-07-26, round two): the counter lives in BOTH
// GPREGRET2 and a .noinit RAM block, and the RAM copy is the one trusted.
// The field log from the first fix showed "boot #1, RESETREAS 0x0" on
// every single reboot of an active watchdog loop — RESETREAS should have
// read DOG and the counter should have climbed, so something between the
// reset and setup() (the bootloader is the suspect; this core's startup
// is clean, we checked) scrubs the POWER registers, and the breaker built
// on them never fired. SRAM is retained through watchdog and soft resets
// on the nRF52840 and no bootloader scrubs the middle of the app's RAM,
// so a magic-tagged noinit block survives exactly the resets we need to
// count. Lost on a real power cycle — which is the correct reset for the
// streak anyway. GPREGRET2 is still written and reported as evidence.
//
// The RAM block also records WHICH bring-up stage the previous run died
// in. That powers a per-stage deadman: died in display bring-up last time
// -> this boot skips the display outright (see the display block), which
// converges in ONE reboot instead of BOOT_SAFE_AFTER.
#define BOOT_TAG_MASK    0xF0
#define BOOT_TAG_VALUE   0xB0  // canary: our counter vs. power-on garbage
#define BOOT_COUNT_MASK  0x0F
#define BOOT_SAFE_AFTER  3     // boots with no healthy run -> safe mode
#define BOOT_HEALTHY_MS  15000 // loop() alive this long -> the boot "took"

// Bring-up stages, recorded on the way IN to each. STAGE_LOOP means the
// previous run made it through setup() entirely.
enum BootStage : uint8_t {
  STAGE_NONE = 0,
  STAGE_EARLY,        // before the display block
  STAGE_DISPLAY,      // display bring-up (the one no-way-back stage)
  STAGE_MCP_DETECT,
  STAGE_MCP_CONFIG,
  STAGE_BLE,
  STAGE_LOOP,
};

// Magic-tagged, inverse-checked so power-on garbage can't fake validity.
// Uninitialised on purpose: the section is outside __bss_start__..__bss_end__
// so the startup code never zeroes it and it rides through warm resets.
struct BootScratch {
  uint32_t magic;
  uint8_t  count;      uint8_t countInv;
  uint8_t  stage;      uint8_t stageInv;
};
#define SCRATCH_MAGIC 0xB007E663UL
// The section-name suffix forces NOBITS ('@' comments out the assembler's
// own trailing flags on ARM): this core's linker scripts have no .noinit
// rule, and without the override GCC emits the orphan section as PROGBITS
// — i.e. loadable content at a RAM address in the .hex. Placement is
// link-order luck either way, so CI (and any local build) should confirm
// the section stays outside __bss_start__..__bss_end__ and below
// __HeapBase — it currently lands just before .bss.
__attribute__((section(".noinit,\"aw\",%nobits@")))
static BootScratch bootScratch;

static bool scratchValid() {
  return bootScratch.magic == SCRATCH_MAGIC &&
         bootScratch.count == (uint8_t)~bootScratch.countInv &&
         bootScratch.stage == (uint8_t)~bootScratch.stageInv;
}

static void scratchWrite(uint8_t count, uint8_t stage) {
  bootScratch.magic    = SCRATCH_MAGIC;
  bootScratch.count    = count;
  bootScratch.countInv = (uint8_t)~count;
  bootScratch.stage    = stage;
  bootScratch.stageInv = (uint8_t)~stage;
}

static const char* stageName(uint8_t s) {
  switch (s) {
    case STAGE_EARLY:      return "early boot";
    case STAGE_DISPLAY:    return "display bring-up";
    case STAGE_MCP_DETECT: return "MCP9600 detect";
    case STAGE_MCP_CONFIG: return "MCP9600 configure";
    case STAGE_BLE:        return "BLE bring-up";
    case STAGE_LOOP:       return "loop (ran)";
    default:               return "none";
  }
}

// ---- PW-ADV-2 payload (16 bytes, little-endian fields) --------------------
// Bluefruit's addManufacturerData() passes the buffer through RAW - it does
// NOT prepend a company ID - so bytes 0-1 of this array ARE the company ID
// and the logger indexes the array identically. v2 appends to v1: bytes
// 0-13 keep their exact v1 offsets. Layout:
//   0-1  company ID FF FF     2-3  magic 'P' 'W'      4  proto version 02
//   5    flags (bit0 pairing, bit1 TC fault)
//   6-7  EGT int16 deci-degC  8-9  CJ int16 deci-degC
//   10   raw MCP9600 STATUS   11   battery % (FF = unknown)
//   12-13 sequence            14-15 thermistor int16 deci-degC
SoftI2C mcpBus = {MCP_SDA_PIN, MCP_SCL_PIN,
                  /*halfPeriodUs=*/10,      // ~50 kHz: slow on purpose (EMI margin)
                  /*stretchTimeoutUs=*/5000};
Mcp9600 mcp;

uint8_t  oledAddr = 0;
bool     oledOK   = false;
bool     showF    = true;
uint32_t nRead    = 0;

// Latest aux readings, refreshed on their own ticks in loop(); the
// payload rebuild just reads the cache.
float    thermC     = NAN;
uint16_t thermRaw   = 0;      // last averaged ADC counts (fault forensics)
uint8_t  thermPwrPin = THERM_PWR_PIN;  // runtime: diag adopts a misplaced wire
uint8_t  batteryPct = pw_adv::kBatteryUnknown;

bool     safeMode  = false;   // boot-loop breaker tripped (see BOOT_* above)
uint8_t  bootCount = 0;       // consecutive boots without a healthy run
bool     displayDeadman = false;  // previous run died IN display bring-up
bool     wireBegun = false;   // hardware Wire owns D4/D5 - no soft probing them

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
// WDT_TIMEOUT_S. Boot then re-clears both sensor buses (the soft probes
// each start with a bus clear), and the sequence counter restarting from 0
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

// Name each stage of bring-up on the way IN — on serial, flushed, AND in
// the noinit scratch, so the next boot knows where this one died even if
// nobody was watching the terminal. Serial.flush() matters twice over:
// without it the tail of the CDC FIFO dies with the hang, and on this
// core CDC output is not flushed automatically — a println right before
// a hang is otherwise simply lost, which is why the first round of logs
// showed nothing between the banner and a step three calls later.
void bootStep(const char* what, uint8_t stage) {
  scratchWrite(scratchValid() ? bootScratch.count : bootCount, stage);
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
  digitalWrite(thermPwrPin, LOW);          // thermistor divider dead
  pinMode(VBAT_ENABLE, INPUT);             // battery divider off (boot state)
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

// -------------------------------------------------- OLED probe (soft bus)
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
// the pod cycles on an exact ~9 s period (8 s WDT + boot overhead) until
// someone holds it in DFU. Field-confirmed twice on 2026-07-26. ROOT
// CAUSE (found third round): a mis-soldered power harness — GND on the
// 3V3 pin, VCC on an IO — which wedged the first TWIM transfer of every
// boot. Not a firmware bug, but it proved the CLASS is real: any
// electrical fault on this bus turns the no-timeout Wire into an
// unrecoverable boot loop, which is why these guards stay after the fix.
//
// So the probing moved OFF the hardware Wire entirely. The OLED is probed
// on a TEMPORARY timeout-capable soft bus over the same two pins: bus
// clear, then a real addressed transaction (START, address, ACK check,
// STOP) at each candidate address, every edge bounded. Only a live,
// ACKing OLED earns hardware-Wire bring-up at all — an absent, unpowered
// or line-clamping module fails the probe cleanly and the pod boots
// without a display instead of entering a call that cannot return. The
// probe also doubles as the bus-clear that used to run here (a reset
// mid-transaction can leave a slave driving SDA low).
//
// A pod that broadcasts with no screen is worth infinitely more than one
// that loops. The residual risk — OLED ACKs the soft probe but the TWIM
// still wedges inside oled.begin() — is covered by the display deadman:
// the noinit scratch records that we died in STAGE_DISPLAY and the next
// boot skips the display outright.
uint8_t oledSoftProbe() {
  SoftI2C probe = {SDA, SCL,
                   /*halfPeriodUs=*/5,        // ~100 kHz
                   /*stretchTimeoutUs=*/2000};
  softI2cBegin(probe);
  softI2cBusClear(probe);
  uint8_t found = 0;
  for (uint8_t a = 0x3C; a <= 0x3D && !found; a++) {
    if (softI2cWriteRead(probe, a, nullptr, 0, nullptr, 0) ==
        SoftI2CStatus::kOk) {
      found = a;
    }
  }
  // Pins are left released (INPUT_PULLUP) for Wire to take over.
  return found;
}

// -------------------------------------------------- harness diagnostic
// Field report (2026-07-26, after the loop was broken): BOTH I2C devices
// silent at once — no OLED ACK on D4/D5, no MCP9600 on D2/D3. Two
// devices on two separate buses do not fail simultaneously by
// coincidence; that is a statement about the HARNESS (module power, a
// shared rail, or SDA/SCL orientation), and the pod can pin it down
// itself instead of asking for a multimeter. Everything here runs on the
// bounded soft bus — the hardware Wire is never touched.
//
// Two tools:
//  * lineReport(): per-pin electrical state. With the internal pullup a
//    healthy idle line reads HIGH — LOW means clamped (short, or an
//    unpowered device's protection diode). Then discharge the line and
//    FLOAT it: only an EXTERNAL pullup can bring it back up, and every
//    breakout in this build carries its own pullups — so "no external
//    pullup" on both pins of a bus means the module is absent or, far
//    more likely after a rewire, not powered.
//  * a full-address scan (0x08-0x77) over every pin pairing of the four
//    bus pins — canonical and swapped, both buses — with an MCP9600 ID
//    handshake at 0x60-0x67 hits and OLED recognition at 0x3C/0x3D. A
//    module wired with SDA/SCL swapped or plugged into the other bus is
//    FOUND and named. (Blind probes of the MCP range are normally
//    avoided — see mcp9600.h — but this only runs when the chip is
//    already unreachable, and a found chip is immediately mode-cycled.)
//
// The MCP9600 is adopted wherever it answers: its bus is soft, so the
// pins are just numbers. The OLED can only be REPORTED if found off its
// canonical orientation — the TWIM's pins are fixed by the variant — so
// the pod prints exactly which wires to swap instead.

struct BusCfg { uint8_t sda; uint8_t scl; };
static const BusCfg kBusCfgs[] = {
  {MCP_SDA_PIN, MCP_SCL_PIN},   // canonical sensor bus
  {MCP_SCL_PIN, MCP_SDA_PIN},   // D2/D3 swapped
  {SDA, SCL},                   // canonical display bus (D4/D5)
  {SCL, SDA},                   // D4/D5 swapped
};

static void printPinName(uint8_t pin) {
  Serial.print('D'); Serial.print(pin);  // Dn == n on this variant
}

static bool cfgUsesDisplayPins(uint8_t sdaPin) {
  return sdaPin == SDA || sdaPin == SCL;   // (scl is then the other one)
}

static void lineReport(uint8_t pin) {
  pinMode(pin, INPUT_PULLUP);
  delayMicroseconds(300);
  const bool highWithPull = digitalRead(pin) == HIGH;
  // Discharge, then float with NO pull: only an external pullup can
  // bring the line back up within the settle window.
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
  delayMicroseconds(50);
  pinMode(pin, INPUT);
  delayMicroseconds(300);
  const bool extPullup = digitalRead(pin) == HIGH;
  pinMode(pin, INPUT_PULLUP);

  Serial.print("  line "); printPinName(pin); Serial.print(": ");
  if (!highWithPull) {
    Serial.println("STUCK LOW - short, or clamped by an unpowered device");
  } else if (extPullup) {
    Serial.println("ok - external pullup present (module powered)");
  } else {
    Serial.println("no external pullup - module absent or UNPOWERED?");
  }
}

// Scan one pin pairing. Prints what ACKed; fills in a validated MCP9600
// address and/or an OLED address if seen. Returns total ACK count.
static uint8_t scanCfg(uint8_t sdaPin, uint8_t sclPin,
                       uint8_t& mcpAt, uint8_t& oledAt) {
  SoftI2C bus = {sdaPin, sclPin, /*halfPeriodUs=*/5, /*stretchTimeoutUs=*/2000};
  softI2cBegin(bus);
  softI2cBusClear(bus);
  mcpAt = 0;
  oledAt = 0;
  uint8_t nAck = 0;
  Serial.print("  scan SDA="); printPinName(sdaPin);
  Serial.print(" SCL=");       printPinName(sclPin);
  Serial.print(":");
  for (uint8_t a = 0x08; a <= 0x77; a++) {
    if (softI2cWriteRead(bus, a, nullptr, 0, nullptr, 0) !=
        SoftI2CStatus::kOk) {
      continue;
    }
    nAck++;
    if (nAck <= 8) { Serial.print(" 0x"); Serial.print(a, HEX); }
    if (a >= mcp9600_regs::kAddrFirst && a <= mcp9600_regs::kAddrLast) {
      Mcp9600 probeChip;
      probeChip.bus = &bus;
      uint8_t hi = 0, lo = 0;
      if (mcpIdHandshake(probeChip, a, hi, lo) &&
          mcp9600_regs::isValidDeviceId(hi)) {
        mcpAt = a;
      }
    } else if (a == 0x3C || a == 0x3D) {
      oledAt = a;
    }
  }
  if (nAck == 0) Serial.print(" no ACKs (0x08-0x77)");
  Serial.println();
  return nAck;
}

// Full harness sweep. Returns true if the MCP9600 was found and adopted
// (mcp.addr + mcpBus pins are then live; caller still mode-cycles it).
static bool harnessDiag() {
  Serial.println("[boot] harness diagnostic - I2C device(s) missing");
  lineReport(MCP_SDA_PIN);
  lineReport(MCP_SCL_PIN);
  if (!wireBegun) {
    lineReport(SDA);
    lineReport(SCL);
  }

  bool mcpAdopted = false;
  uint16_t totalAcks = 0;
  for (const BusCfg& cfg : kBusCfgs) {
    if (wireBegun && cfgUsesDisplayPins(cfg.sda)) continue;  // TWIM owns D4/D5
    uint8_t mcpAt = 0, oledAt = 0;
    totalAcks += scanCfg(cfg.sda, cfg.scl, mcpAt, oledAt);

    if (mcpAt && !mcpAdopted) {
      mcpBus.sdaPin = cfg.sda;
      mcpBus.sclPin = cfg.scl;
      softI2cBegin(mcpBus);
      softI2cBusClear(mcpBus);
      mcp.addr = mcpAt;
      mcpAdopted = true;
      Serial.print("  -> MCP9600 found at 0x"); Serial.print(mcpAt, HEX);
      Serial.print(", ADOPTING SDA="); printPinName(cfg.sda);
      Serial.print(" SCL=");           printPinName(cfg.scl);
      Serial.println();
      if (cfg.sda != MCP_SDA_PIN || cfg.scl != MCP_SCL_PIN) {
        Serial.println("     (non-canonical - rewire to SDA->D2 SCL->D3 when convenient)");
      }
    }
    if (oledAt) {
      Serial.print("  -> OLED found at 0x"); Serial.print(oledAt, HEX);
      Serial.print(" with SDA="); printPinName(cfg.sda);
      Serial.print(" SCL=");      printPinName(cfg.scl);
      Serial.println();
      if (cfg.sda != SDA || cfg.scl != SCL) {
        Serial.println("     the display bus is HARDWARE and cannot follow: rewire the");
        Serial.println("     OLED to SDA->D4 SCL->D5 to get the screen back");
      }
    }
  }

  if (totalAcks == 0) {
    Serial.println("  VERDICT: nothing ACKed on any pairing of D2-D5. Two dead");
    Serial.println("  buses at once is almost always module POWER - check 3V3 and");
    Serial.println("  GND to both breakouts first, then data wiring. Expected:");
    Serial.println("    MCP9600: SDA->D2  SCL->D3   |   OLED: SDA->D4  SCL->D5");
    Serial.println("    (XIAO left column, USB up: D0 D1 D2 D3 D4 D5 D6 top->bottom)");
  }
  Serial.flush();
  return mcpAdopted;
}

// Runtime flavor: quiet single-try sweep, used by loop()'s recovery tick
// while no sensor is adopted. A found chip prints; misses stay silent.
static bool mcpRuntimeLocate() {
  for (const BusCfg& cfg : kBusCfgs) {
    if (wireBegun && cfgUsesDisplayPins(cfg.sda)) continue;
    mcpBus.sdaPin = cfg.sda;
    mcpBus.sclPin = cfg.scl;
    softI2cBegin(mcpBus);
    softI2cBusClear(mcpBus);
    if (mcpDetect(mcp, /*triesPerAddr=*/1, /*gapMs=*/0)) {
      if (cfg.sda != MCP_SDA_PIN || cfg.scl != MCP_SCL_PIN) {
        Serial.print("MCP9600 answering on SDA=");
        printPinName(cfg.sda);
        Serial.print(" SCL=");
        printPinName(cfg.scl);
        Serial.println(" - wires not on D2(SDA)/D3(SCL); adopted anyway");
      }
      return true;
    }
  }
  mcpBus.sdaPin = MCP_SDA_PIN;   // canonical again for the next attempt
  mcpBus.sclPin = MCP_SCL_PIN;
  return false;
}

// -------------------------------------------------- aux ADC reads
// Both reads are PULSED: the source is energized, allowed to settle,
// sampled, and de-energized, so neither divider draws anything between
// ticks or in deep sleep. Both set analogReference() at the call site —
// it is global state in this core and the two reads need DIFFERENT
// references, so nothing here may assume it.

// Thermistor: ratiometric on purpose. D6 drives the divider at VDD and
// AR_VDD4 makes VDD the ADC's full scale, so the supply voltage cancels
// out of the ratio exactly — no calibration constant, no VDD-tolerance
// error. THERM_SETTLE_MS lets the sense node charge through the divider
// into its smoothing cap (the two are coupled — see the define). The
// cap is also what makes the ADC timing legitimate: this core's SAADC
// acquisition is a fixed 3 us, rated by Nordic for <= 10k source
// impedance, and the divider's Thevenin runs to ~100k cold — but a
// >= 10 nF reservoir is ~4000x the SAADC sample cap, so each sample
// draws locally with ~0.03% droop. First sample after the reference
// switch is discarded per SAADC habit, then 4 are averaged. Total
// ~13 ms blocking once per THERM_MS — fine against the 100 ms loop
// tick, not worth a state machine.
//
// TACQ trap: analogSampleTime() is GLOBAL core state, like
// analogReference. Do not "fix" the battery divider's out-of-spec
// source impedance (1M||510k ~ 338k at 3 us) by raising it — the
// datalogger's x3.024 battery calibration was measured WITH that
// acquisition droop baked in (very likely part of its observed 4.11 V
// at a true 4.20 V). Thermistor gets the cap + default TACQ; battery
// gets untouched TACQ + the verbatim calibration. They move together.
float readThermistorC() {
  digitalWrite(thermPwrPin, HIGH);
  delay(THERM_SETTLE_MS);
  analogReference(AR_VDD4);
  (void)analogRead(THERM_ADC_PIN);           // discard: reference settle
  uint32_t sum = 0;
  for (int i = 0; i < 4; i++) sum += (uint32_t)analogRead(THERM_ADC_PIN);
  digitalWrite(thermPwrPin, LOW);
  thermRaw = (uint16_t)(sum / 4);            // kept for fault forensics
  return thermistor::countsToC(thermRaw);
}

// -------------------------------------------------- thermistor harness diag
// Same philosophy as the I2C harness diagnostic: a persistent nan is a
// WIRING statement, and the pod can localize it without a multimeter.
// Runs from loop() after a few consecutive rail-pegged reads, then every
// 15 s while the fault lasts, so re-flowing a joint shows up live. All
// experiments are bounded GPIO/ADC pokes on pins that are free in the
// canonical wiring.
//
// Decision tree (as-built divider: fixed leg from the power pin down to
// the sense node, NTC from the node to GND):
//   drive HIGH -> valid counts        intermittent - normal path recovers
//   drive HIGH -> HIGH rail:
//     drive LOW also HIGH             node tied to a live rail (3V3?)
//     drive LOW goes LOW              drive works, no load -> NTC leg open
//   drive HIGH -> LOW rail (no response):
//     one of D7-D10 moves A0          power wire on the wrong pin -> ADOPT
//     a peripheral analog pin follows the drive
//                                     sense wire on the wrong pin -> report
//     A0 charge-injection holds       A0 floating: sense wire not on the
//                                     node, or the cap is IN SERIES with A0
//     A0 charge-injection decays      A0 loaded but nothing drives the node
//                                     -> fixed-resistor leg open
static uint16_t thermReadAt(uint8_t drivePin, bool high) {
  digitalWrite(drivePin, high ? HIGH : LOW);
  delay(THERM_SETTLE_MS);
  analogReference(AR_VDD4);
  (void)analogRead(THERM_ADC_PIN);
  return (uint16_t)analogRead(THERM_ADC_PIN);
}

void thermDiagnose() {
  Serial.print("[therm] diag: ");

  const uint16_t cHi = thermReadAt(thermPwrPin, true);
  const uint16_t cLo = thermReadAt(thermPwrPin, false);

  if (cHi >= thermistor::kCountsFloor && cHi <= thermistor::kCountsCeiling) {
    Serial.print("reads fine right now (");
    Serial.print(cHi);
    Serial.println(" counts) - intermittent joint?");
    Serial.flush();
    return;
  }

  if (cHi > thermistor::kCountsCeiling) {
    if (cLo > thermistor::kCountsCeiling) {
      Serial.println("node HIGH even with drive low - sense node tied to a live rail (3V3?)");
    } else {
      Serial.print("drive OK (high ");
      Serial.print(cHi);
      Serial.print(" / low ");
      Serial.print(cLo);
      Serial.println(") but no NTC load - NTC leg open, reflow its joints");
    }
    Serial.flush();
    return;
  }

  // LOW rail: A0 sees nothing from the drive pin. Hunt for the divider.
  Serial.print("no response on D");
  Serial.print(thermPwrPin);
  Serial.println(" (LOW rail); probing...");

  // Is the power wire on a different free pin? These are all unused in
  // the canonical wiring, so a brief high pulse is safe - and if the
  // divider answers, adopt that pin for the session.
  static const uint8_t kAltDrive[] = {D7, D8, D9, D10};
  for (uint8_t pin : kAltDrive) {
    pinMode(pin, OUTPUT);
    const uint16_t c = thermReadAt(pin, true);
    digitalWrite(pin, LOW);
    if (c > 200) {
      thermPwrPin = pin;                     // leave it OUTPUT LOW: adopted
      Serial.print("[therm]   divider answers on D");
      Serial.print(pin);
      Serial.print(" (");
      Serial.print(c);
      Serial.println(" counts) - power wire is on that pin, not D6.");
      Serial.println("[therm]   ADOPTED for this session; move the wire or THERM_PWR_PIN.");
      Serial.flush();
      return;
    }
    pinMode(pin, INPUT);                     // not it - release
  }

  // Is the sense wire on a different analog pin? Read-only sweep: a
  // divider node sits mid-scale and FOLLOWS the drive; the peripheral
  // pins' own pullups sit at the rail and do not.
  static const uint8_t kAltSense[] = {A1, A2, A3, A4, A5};
  static const char* const kAltRole[] = {
      "the button", "MCP9600 SDA", "MCP9600 SCL", "OLED SDA", "OLED SCL"};
  digitalWrite(thermPwrPin, HIGH);
  delay(THERM_SETTLE_MS);
  analogReference(AR_VDD4);
  uint16_t hi[5];
  for (int i = 0; i < 5; i++) hi[i] = (uint16_t)analogRead(kAltSense[i]);
  digitalWrite(thermPwrPin, LOW);
  delay(THERM_SETTLE_MS);
  for (int i = 0; i < 5; i++) {
    const uint16_t lo = (uint16_t)analogRead(kAltSense[i]);
    if (hi[i] > 200 && hi[i] < 3900 && hi[i] > lo + 800) {
      Serial.print("[therm]   a divider node is following the drive pin on D");
      Serial.print(kAltSense[i]);
      Serial.print(" - that pin belongs to ");
      Serial.print(kAltRole[i]);
      Serial.println("! Move the sense wire to A0/D0.");
      Serial.flush();
      return;
    }
  }

  // Is the node SHORTED to ground? Drive A0 itself high and read while
  // still driving: a normal node follows the pin driver (~full scale
  // against the divider's k-ohms); a dead short holds it near zero.
  // The classic cause on this bench: a THERMOCOUPLE plugged in where
  // the NTC belongs — a TC is a few OHMS end to end.
  pinMode(THERM_ADC_PIN, OUTPUT);
  digitalWrite(THERM_ADC_PIN, HIGH);
  delay(1);
  analogReference(AR_VDD4);
  const uint16_t driven = (uint16_t)analogRead(THERM_ADC_PIN);
  if (driven < 1000) {
    pinMode(THERM_ADC_PIN, INPUT);
    Serial.print("[therm]   node SHORTED to ground (");
    Serial.print(driven);
    Serial.println(" counts while driven high) - shorted leads, or is this");
    Serial.println("[therm]   probe actually a THERMOCOUPLE? A TC is a few ohms;");
    Serial.println("[therm]   this input needs an NTC thermistor.");
    Serial.flush();
    return;
  }

  // Not shorted: is A0 even connected to a node? The drive above just
  // charged it - release and watch the charge bleed. A real node
  // (divider legs and/or the cap to ground) drains the pin in a few
  // ms; a floating pin holds the charge on its own few pF more or
  // less indefinitely.
  pinMode(THERM_ADC_PIN, INPUT);             // no pull
  delay(5);
  const uint16_t held = (uint16_t)analogRead(THERM_ADC_PIN);
  if (held > 2500) {
    Serial.print("[therm]   A0 holds injected charge (");
    Serial.print(held);
    Serial.println(" counts) - A0 is FLOATING: sense wire not on the node,");
    Serial.println("[therm]   or the cap is IN SERIES between the node and A0.");
  } else {
    Serial.print("[therm]   A0 is loaded (injected charge fell to ");
    Serial.print(held);
    Serial.println(" counts) but nothing drives the node - fixed-resistor");
    Serial.println("[therm]   leg open, or the power wire is not on D6-D10.");
  }
  Serial.flush();
}

// Battery: the XIAO's onboard 1M/510k divider, gated by VBAT_ENABLE
// (P0.14, active LOW). The datalogger leaves it enabled forever; the
// egg pulses it — enable, settle (the 1M leg into the ADC input is the
// slow RC here, hence 5 ms), read, then back to Hi-Z INPUT, which is
// the variant's boot state and draws nothing in System OFF. Uses the
// default AR_INTERNAL 3.6 V full scale — battery::kScale is calibrated
// against exactly that (see battery.h).
uint8_t readBatteryPct() {
  pinMode(VBAT_ENABLE, OUTPUT);
  digitalWrite(VBAT_ENABLE, LOW);
  delay(5);
  analogReference(AR_INTERNAL);
  (void)analogRead(PIN_VBAT);                // discard: reference settle
  const uint16_t counts = (uint16_t)analogRead(PIN_VBAT);
  pinMode(VBAT_ENABLE, INPUT);               // divider off (boot state)
  return battery::voltsToPercent(battery::countsToVolts(counts));
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

// -------------------------------------------------- PW-ADV-2 encode
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
                       millis() < pairUntil, advSeq,
                       batteryPct, thermC);

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
  char lineBuf[32];
  if (isnan(egt)) {
    snprintf(lineBuf, sizeof(lineBuf), "---");
  } else {
    dtostrf(egt, 1, 1, numBuf);
    snprintf(lineBuf, sizeof(lineBuf), "%s", numBuf);
  }
  oled.setTextSize(3);  // 18 px per glyph
  oled.setCursor((OLED_W - 18 * (int16_t)strlen(lineBuf)) / 2, 20);
  oled.print(lineBuf);

  // Bottom status line: CJ, aux thermistor, battery. dtostrf per value
  // (this core's printf lacks reliable %f), then one centered line.
  // Worst case "CJ -12.3F -12.3 100%" = 20 glyphs of a 21-glyph budget
  // (the C/F toggle letter rides on the CJ number; the middle figure is
  // the aux thermistor in the same unit).
  char thBuf[8];
  if (isnan(cj)) snprintf(numBuf, sizeof(numBuf), "---");
  else           dtostrf(cj, 1, 1, numBuf);
  float th = showF ? c2f(thermC) : thermC;
  if (isnan(th)) snprintf(thBuf, sizeof(thBuf), "---");
  else           dtostrf(th, 1, 1, thBuf);
  char pctBuf[6];
  if (batteryPct == pw_adv::kBatteryUnknown) {
    snprintf(pctBuf, sizeof(pctBuf), "--%%");
  } else {
    snprintf(pctBuf, sizeof(pctBuf), "%u%%", (unsigned)batteryPct);
  }
  snprintf(lineBuf, sizeof(lineBuf), "CJ %s%c %s %s",
           numBuf, showF ? 'F' : 'C', thBuf, pctBuf);
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
  // NOTE: field logs show RESETREAS reads 0x0 even mid-watchdog-loop on
  // this board — something before the sketch (the bootloader, most
  // likely) scrubs it. These flags are therefore best-effort: fine for
  // cosmetics (the WDT RESET banner), never load-bearing. The noinit
  // scratch below is what actually has to survive.
  const bool wdtReboot = (resetReas & POWER_RESETREAS_DOG_Msk) != 0;
  const bool offWake   = (resetReas & POWER_RESETREAS_OFF_Msk) != 0;

  // Trusted store first: the noinit RAM scratch (survives warm resets,
  // dies with real power loss). GPREGRET2 is bumped too, but only as
  // evidence — the same field logs show it coming back 0 every boot.
  const bool    ramValid  = scratchValid();
  const uint8_t prevStage = ramValid ? bootScratch.stage
                                     : (uint8_t)STAGE_NONE;
  uint8_t ramCount = ramValid ? bootScratch.count : 0;
  if (ramCount < 255) ramCount++;
  const uint8_t regRaw   = bootRegGet();
  const uint8_t regCount = bootCountBump();
  bootCount = (ramCount > regCount) ? ramCount : regCount;
  scratchWrite(ramCount, STAGE_EARLY);

  safeMode       = bootCount >= BOOT_SAFE_AFTER;
  displayDeadman = ramValid && prevStage == STAGE_DISPLAY;

  // Healthy boot: arm the watchdog first thing, so a hang anywhere reboots
  // the pod instead of zombifying it. Safe mode defers it to loop() — see
  // the note by WDT_TIMEOUT_S.
  if (!safeMode) wdtSetup();

  pinMode(BTN_PIN, INPUT_PULLUP);
  delay(5);   // let the internal pull-up actually pull the line up

  // Aux ADC plumbing. The thermistor power gate idles LOW (divider
  // dead); VBAT_ENABLE stays at its boot state (Hi-Z, divider off) and
  // is pulsed only inside readBatteryPct(). analogReference is set at
  // each call site, never here — the two reads need different ones.
  pinMode(THERM_PWR_PIN, OUTPUT);
  digitalWrite(THERM_PWR_PIN, LOW);
  analogReadResolution(12);

  // Woken out of deep sleep by the button: demand the 5 s hold before
  // anything else powers up. (A watchdog reboot skips this — after a hang
  // the pod must come back broadcasting without a human. So does safe
  // mode: the sleep path is itself a reboot path, and a pod that is
  // already cycling must not be handed another way to go dark.)
  if (offWake && !wdtReboot && !safeMode) wakeHoldGate();

  Serial.begin(115200);
  while (!Serial && millis() < 3000) { wdtPet(); }  // don't block on battery

  Serial.println("\nDovesSensorEgg - wireless EGT pod (PW-ADV-1)");
  // One forensic line with every storage source, so a log of a looping
  // pod says which stores survive its resets and where the last run died.
  Serial.print("boot #"); Serial.print(bootCount);
  Serial.print(" since last healthy run (ram ");
  Serial.print(ramValid ? "ok" : "lost");
  Serial.print(", prev stage: "); Serial.print(stageName(prevStage));
  Serial.print(", reg 0x"); Serial.print(regRaw, HEX);
  Serial.print("), RESETREAS 0x"); Serial.println(resetReas, HEX);
  Serial.flush();
  if (wdtReboot) {
    // The previous run hung (I2C wedge under ignition EMI is the known
    // suspect) and the watchdog pulled us out. The bus clears in the soft
    // probes below are exactly the recovery that wedge needs. (This banner
    // is best-effort: RESETREAS is scrubbed before us on this board, so
    // its absence proves nothing - trust the "prev stage" field instead.)
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
  // Three gates before the hardware Wire is allowed near D4/D5:
  //   1. safe mode      - the pod has been cycling; don't even probe.
  //   2. display deadman - the PREVIOUS run died in exactly this stage
  //                        (noinit scratch says so); skip in one reboot
  //                        instead of waiting out the safe-mode count.
  //   3. soft probe     - the OLED must ACK a real addressed transaction
  //                        on the timeout-capable soft bus first. Only a
  //                        demonstrably live module earns the TWIM.
  // Serial and BLE do not need the display; a pod on the air with a dark
  // screen can be diagnosed and re-flashed.
  bootStep("display bring-up", STAGE_DISPLAY);
  if (safeMode) {
    Serial.println("  safe mode - display skipped (hardware Wire not entered)");
  } else if (displayDeadman) {
    Serial.println("  !! previous boot died in display bring-up - display DISABLED");
    Serial.println("     this boot. Serial + BLE only; power-cycle to retry the");
    Serial.println("     display. Check the OLED module and D4/D5 wiring.");
  } else {
    const uint8_t probeAddr = oledSoftProbe();
    if (!probeAddr) {
      Serial.println("  no OLED ACK on soft probe (D4/D5) - booting without display");
    } else {
      oledAddr = probeAddr;
      Serial.print("  OLED ACKs at 0x"); Serial.println(oledAddr, HEX);
      Serial.println("  entering hardware Wire (the no-way-back call)...");
      Serial.flush();   // if boot dies here, the log must already say where
      wireBegun = true;
      Wire.begin();
      Wire.setClock(OLED_I2C_HZ);
      delay(50);
    #if USE_SH1106
      oledOK = oled.begin(oledAddr, true);
    #else
      oledOK = oled.begin(SSD1306_SWITCHCAPVCC, oledAddr);
    #endif
      Serial.print("  OLED "); Serial.print(DRV_NAME);
      Serial.println(oledOK ? " init ok" : " init FAILED");
    }
  }

  // MCP9600 on its own timeout-capable soft bus: proper ID handshake at
  // each candidate address (retried — a mid-conversion NACK is normal and
  // must never read as "absent"), never a blind probe.
  bootStep("MCP9600 detect (soft bus D2/D3)", STAGE_MCP_DETECT);
  softI2cBegin(mcpBus);
  softI2cBusClear(mcpBus);
  mcp.bus = &mcpBus;

  // Full retried scan, safe mode included. Safe mode trims things that can
  // HANG or that cost seconds; this costs ~0.5 s and buys the one rule the
  // driver is built on — the chip NACKs its own address mid-conversion, so
  // a single NACK never means absent. Thinning it here would make the pod
  // most likely to report "no sensor" exactly when someone is standing over
  // it trying to work out what is wrong.
  bool mcpOK = false;
  for (int attempt = 1; attempt <= 3 && !mcpOK; attempt++) {
    mcpOK = mcpDetect(mcp, /*triesPerAddr=*/3, /*gapMs=*/20);
    if (!mcpOK) {
      Serial.print("MCP detect attempt "); Serial.print(attempt);
      Serial.println(" failed (soft bus D2/D3)");
      softI2cBusClear(mcpBus);
      bootDelay(200);
    }
    wdtPet();
  }

  // Anything missing -> full harness diagnostic: per-line electrical
  // state, then a scan of every pin pairing of D2-D5 for both devices.
  // Both buses silent at once is a harness statement (power, rail,
  // SDA/SCL orientation), not two coincidences — make the pod say which.
  // The sweep ADOPTS an MCP9600 found on non-canonical pins (its bus is
  // soft; the pins are just numbers) and names the exact rewire for an
  // OLED found off D4(SDA)/D5(SCL) — the TWIM's pins can't follow.
  if (!oledOK || !mcpOK) {
    if (harnessDiag() && !mcpOK) {
      Serial.println("  (sensor adopted by the diagnostic - continuing)");
      mcpOK = true;
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
    bootStep("MCP9600 configure", STAGE_MCP_CONFIG);
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

  bootStep("BLE bring-up", STAGE_BLE);
  bleSetup();

  bootStep("setup complete - entering loop()", STAGE_LOOP);
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
    scratchWrite(0, STAGE_LOOP);   // RAM copy too - it is the trusted one
    Serial.println("boot confirmed healthy - reboot streak cleared");
  }

  static uint32_t tRead = 0, tAdv = 0, tDraw = 0, tSer = 0;
  static uint32_t tTherm = 0, tBatt = 0;
  // Aux ticks. tBatt starts one interval in the past so the first
  // payload after boot carries a real battery byte, not 30 s of 0xFF.
  if (millis() - tTherm >= THERM_MS) {
    tTherm = millis();
    thermC = readThermistorC();
    // Persistent rail-pegged reads -> run the harness diagnostic, then
    // re-run every 15 s while the fault lasts (verdicts print each time,
    // so re-flowing a joint shows up live on serial).
    static uint8_t  thermNanRun = 0;
    static uint32_t tThermDiag  = 0;
    if (!isnan(thermC)) {
      thermNanRun = 0;
    } else if (thermNanRun < 255) {
      thermNanRun++;
    }
    if (thermNanRun >= 3 &&
        (tThermDiag == 0 || millis() - tThermDiag >= 15000)) {
      tThermDiag = millis();
      thermDiagnose();
    }
  }
  if (tBatt == 0 || millis() - tBatt >= BATT_MS) {
    tBatt = millis();
    batteryPct = readBatteryPct();
  }
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
        // Not adopted yet: sweep every pin pairing (quiet), so a sensor
        // wired to the wrong pins — or plugged in after boot — is picked
        // up in place. ~30 ms once per 2 s; the loop never notices.
        if (mcpRuntimeLocate()) {
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
    Serial.print("    TH ");
    Serial.print(thermC, 1);
    Serial.print(" C");
    if (!isnan(thermC)) {
      // Measured NTC resistance: the ground truth that identifies the
      // part. A "100k" input reading R=10k at ambient is a 10k probe;
      // R=1.9k means the fixed leg is not the value the math assumes.
      Serial.print(" (R=");
      Serial.print(thermistor::countsToResistance(thermRaw) / 1000.0f, 1);
      Serial.print("k)");
    }
    if (isnan(thermC)) {
      // Rail-pegged divider: say WHICH rail, because in this topology
      // (fixed leg on D6, NTC to GND) they mean different faults.
      Serial.print(" (raw ");
      Serial.print(thermRaw);
      if (thermRaw >= thermistor::kCountsCeiling) {
        Serial.print(" HIGH rail - NTC leg open?");
      } else {
        Serial.print(" LOW rail - no D6 drive, or A0 not on the node?");
      }
      Serial.print(")");
    }
    Serial.print("    BAT ");
    if (batteryPct == pw_adv::kBatteryUnknown) Serial.print("--");
    else                                       Serial.print(batteryPct);
    Serial.print("%");
    Serial.println();
  }
}
