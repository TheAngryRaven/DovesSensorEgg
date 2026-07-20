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

#include "images.h"          // boot splash (the bird, from DovesDataLogger)
#include "pw_adv_encode.h"   // PW-ADV-1 payload builder (host-tested)
#include "soft_i2c.h"        // timeout-capable bit-banged bus (MCP9600)
#include "mcp9600.h"         // register driver over the soft bus
#include "mcp9600_regs.h"    // host-tested register codecs

// ---- SCREEN DRIVER: flip this if the boot splash geometry looks wrong ------
#define USE_SH1106   0        // 1 = SH1106 (1.3")    0 = SSD1306 (0.96")
#define OLED_W     128
#define OLED_H      64
// ---------------------------------------------------------------------------

#if USE_SH1106
  #include <Adafruit_SH110X.h>
  Adafruit_SH1106G oled(OLED_W, OLED_H, &Wire, -1);
  #define PX_ON   SH110X_WHITE
  #define PX_OFF  SH110X_BLACK
  #define DRV_NAME "SH1106"
  #define DISPLAY_OFF() oled.oled_command(SH110X_DISPLAYOFF)
#else
  #include <Adafruit_SSD1306.h>
  Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire, -1);
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

uint8_t  mcpAddr  = 0;
uint8_t  oledAddr = 0;
bool     oledOK   = false;
bool     showF    = true;
uint32_t nRead    = 0;

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
#define WDT_TIMEOUT_S   8      // generous: setup()'s info screens take ~7 s
#define FATAL_REBOOT_MS 30000  // fatal() shows its screen this long, then retries

void wdtSetup() {
  NRF_WDT->CONFIG = WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos;
  NRF_WDT->CRV    = WDT_TIMEOUT_S * 32768;              // 32768 Hz LFCLK
  NRF_WDT->RREN   = WDT_RREN_RR0_Enabled << WDT_RREN_RR0_Pos;
  NRF_WDT->TASKS_START = 1;
}

void wdtPet() { NRF_WDT->RR[0] = WDT_RR_RR_Reload; }

// Read + clear the sticky reset-reason register. Callers test the bits:
// DOG = previous run hung and the watchdog rebooted it; OFF = a GPIO
// (the button) woke the chip out of System OFF deep sleep.
uint32_t captureResetReason() {
  uint32_t rr = NRF_POWER->RESETREAS;
  NRF_POWER->RESETREAS = rr;   // sticky until written — clear for next boot
  return rr;
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
  uint8_t sdEnabled = 0;
  (void)sd_softdevice_is_enabled(&sdEnabled);
  if (sdEnabled) {
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
// display dark. Does not return.
void enterDeepSleep() {
  Serial.println("deep sleep - hold button 5 s to wake");
  Bluefruit.Advertising.stop();
  mcpShutdown(mcp);                        // MCP9600 shutdown mode (~uA)
  if (oledOK) {
    oled.clearDisplay();
    oled.display();
    DISPLAY_OFF();
  }
  // Held button = SENSE satisfied = instant re-wake; wait for release.
  while (digitalRead(BTN_PIN) == LOW) { wdtPet(); delay(10); }
  delay(50);   // contact settle
  systemOff();
}

// System OFF button wake: require a deliberate WAKE_HOLD_MS hold before
// booting. Released early -> straight back to sleep. Nothing is drawn and
// no peripheral is touched, so a pocket bump never lights the screen.
void wakeHoldGate() {
  const uint32_t t0 = millis();
  while (digitalRead(BTN_PIN) == LOW) {
    wdtPet();
    if (millis() - t0 >= WAKE_HOLD_MS) {
      // Confirmed. Swallow the rest of the hold so the press doesn't fall
      // through into btnEvent() as a C/F toggle or pairing long-press.
      while (digitalRead(BTN_PIN) == LOW) { wdtPet(); delay(10); }
      return;
    }
    delay(10);
  }
  systemOff();  // released early — back to sleep
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

  if (oledOK) {
    oled.clearDisplay();
    oled.setTextSize(1); oled.setTextColor(PX_ON, PX_OFF);
    oled.setCursor(0, 0);  oled.print("BLE: PWEGT");
    oled.setCursor(0, 18); oled.print("MAC (for logger):");
    oled.setCursor(0, 32); oled.print(macStr);
    oled.display();
    delay(3000);
    wdtPet();  // 3 s hold — stay inside the watchdog window
  }
}

// -------------------------------------------------- fatal halt
// Shows the message long enough to read, then hard-resets and tries again:
// a TRANSIENT boot failure (EMI glitch during the MCP probe, marginal
// battery sag) self-heals instead of bricking the pod for the day, while a
// permanent fault (probe genuinely absent) still shows a readable screen
// 30 s out of every retry cycle. The WDT is fed while the screen is up —
// this halt is deliberate, not a hang.
void fatal(const char* msg) {
  Serial.print("FATAL: "); Serial.println(msg);
  const uint32_t t0 = millis();
  while (1) {
    wdtPet();
    if (oledOK) {
      oled.clearDisplay();
      oled.setTextSize(2); oled.setTextColor(PX_ON, PX_OFF);
      oled.setCursor(0, 0);  oled.print("FATAL");
      oled.setTextSize(1);
      oled.setCursor(0, 24); oled.print(msg);
      oled.setCursor(0, 54); oled.print("retrying in 30s");
      oled.display();
    }
    delay(1000);
    if (millis() - t0 >= FATAL_REBOOT_MS) NVIC_SystemReset();
  }
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
  oled.print("0x"); oled.print(mcpAddr, HEX);
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
  if (!advOK) {
    oled.setCursor(OLED_W - 24, 56); oled.print("ADV!");
  } else if (millis() < pairUntil) {
    oled.setCursor(OLED_W - 24, 56); oled.print("PAIR");
  }

  oled.display();
}

// -------------------------------------------------- setup
void setup() {
  // Reset-cause first (the register is sticky), THEN arm the WDT — from
  // here on a hang anywhere reboots the pod instead of zombifying it.
  const uint32_t resetReas = captureResetReason();
  const bool wdtReboot = (resetReas & POWER_RESETREAS_DOG_Msk) != 0;
  const bool offWake   = (resetReas & POWER_RESETREAS_OFF_Msk) != 0;
  wdtSetup();

  pinMode(BTN_PIN, INPUT_PULLUP);

  // Woken out of deep sleep by the button: demand the 5 s hold before
  // anything else powers up. (A watchdog reboot skips this — after a hang
  // the pod must come back broadcasting without a human.)
  if (offWake) wakeHoldGate();

  Serial.begin(115200);
  while (!Serial && millis() < 3000) { }     // don't block forever on battery

  Serial.println("\nDovesSensorEgg - wireless EGT pod (PW-ADV-1)");
  if (wdtReboot) {
    // The previous run hung (I2C wedge under ignition EMI is the known
    // suspect) and the watchdog pulled us out. The i2cBusClear() below is
    // exactly the recovery that wedge needs.
    Serial.println("!! WATCHDOG REBOOT - previous run hung");
  }

  i2cBusClear();               // recover a slave wedged by a mid-read reset
  Wire.begin();
  // 400 kHz: the OLED is alone on this bus now. (It was pinned to 100 kHz
  // only because the clock-stretching MCP9600 shared it — that constraint
  // moved to the dedicated soft bus along with the chip.)
  Wire.setClock(400000);
  delay(100);
  scanBus();

  if (oledAddr) {
  #if USE_SH1106
    oledOK = oled.begin(oledAddr, true);
  #else
    oledOK = oled.begin(SSD1306_SWITCHCAPVCC, oledAddr);
  #endif
    Serial.print("OLED "); Serial.print(DRV_NAME);
    Serial.println(oledOK ? " init ok" : " init FAILED");
  } else {
    Serial.println("no OLED on bus - serial only");
  }

  // MCP9600 on its own timeout-capable soft bus: proper ID handshake at
  // each candidate address (retried — a mid-conversion NACK is normal and
  // must never read as "absent"), never a blind probe.
  softI2cBegin(mcpBus);
  softI2cBusClear(mcpBus);
  mcp.bus = &mcpBus;

  bool mcpOK = false;
  for (int attempt = 1; attempt <= 3 && !mcpOK; attempt++) {
    mcpOK = mcpDetect(mcp, /*triesPerAddr=*/3, /*gapMs=*/20);
    if (!mcpOK) {
      Serial.print("MCP detect attempt "); Serial.print(attempt);
      Serial.println(" failed (soft bus D2/D3)");
      softI2cBusClear(mcpBus);
      delay(200);
    }
    wdtPet();
  }
  mcpAddr = mcp.addr;

  if (oledOK) {
    // Boot splash — the bird, same art as the datalogger.
    oled.clearDisplay();
    oled.drawBitmap(0, 0, image_data_bird1, 128, 64, PX_ON);
    oled.display();
    delay(2500);
    wdtPet();  // 2.5 s hold — stay inside the watchdog window

    oled.clearDisplay();
    oled.setTextSize(1); oled.setTextColor(PX_ON, PX_OFF);
    oled.setCursor(0, 0);  oled.print("I2C SCAN");
    oled.setCursor(0, 18); oled.print("OLED 0x"); oled.print(oledAddr, HEX);
    oled.setCursor(0, 32); oled.print("MCP  ");
    if (mcpAddr) { oled.print("0x"); oled.print(mcpAddr, HEX); }
    else           oled.print("NOT FOUND");
    if (wdtReboot) { oled.setCursor(0, 54); oled.print("WDT RESET"); }
    oled.display();
    delay(2000);
    wdtPet();
  }

  if (!mcpOK) fatal("MCP not on bus");

  // Mode-cycle (Shutdown -> Normal, the chip's nearest thing to a reset
  // command) + full config with read-back verify.
  bool cfgOK = false;
  for (int attempt = 1; attempt <= 3 && !cfgOK; attempt++) {
    cfgOK = mcpModeCycle(mcp);
    if (!cfgOK) {
      Serial.print("MCP config attempt "); Serial.print(attempt);
      Serial.println(" failed");
      delay(100);
    }
    wdtPet();
  }
  if (!cfgOK) fatal("MCP config fail");

  Serial.print("MCP9600 up @0x"); Serial.println(mcpAddr, HEX);

  bleSetup();

  Serial.println("D1 short = C/F toggle, long = pairing window\n");
}

// -------------------------------------------------- loop
void loop() {
  wdtPet();  // sole feed point: a wedge ANYWHERE below reboots the pod

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
  static uint32_t sleepHoldSince = 0;
  const bool btnDown = digitalRead(BTN_PIN) == LOW;
  if (!btnDown) {
    sleepHoldSince = 0;
  } else if (sleepHoldSince == 0) {
    sleepHoldSince = millis();
  } else if (millis() - sleepHoldSince >= SLEEP_HOLD_MS) {
    enterDeepSleep();                            // does not return
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
      Serial.println("MCP fault run -> bus clear + mode cycle");
      softI2cBusClear(mcpBus);
      if (mcpModeCycle(mcp)) {
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
