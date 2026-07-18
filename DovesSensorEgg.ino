/* ==========================================================================
   DovesSensorEgg - wireless EGT pod (PW-ADV-1 broadcaster)
   XIAO nRF52840 + Adafruit MCP9600 + I2C OLED + button

   Boot -> scan I2C -> border test -> BLE up (MAC shown) -> live temp on
   screen + serial + BLE advertising.

   BLE: pure BROADCASTER. EGT + cold junction are packed into a 14-byte
   Manufacturer Specific Data AD structure ("PW-ADV-1") and advertised at
   ~10 Hz under the name "PWEGT". The egg never accepts a connection; the
   DovesDataLogger receives the broadcasts with a passive scan. nRF
   Connect on a phone doubles as a live hex debugger (mfg data starts
   FF FF 50 57, bytes 12-13 increment).

   Button on D1: short press toggles C / F on the debug screen; long
   press (>1 s) opens a 30 s pairing window (advertises flags bit0 -
   informational, the logger pairs by MAC/magic).

   Libraries (Arduino Library Manager):
     Adafruit MCP9600
     Adafruit SH110X          (or Adafruit SSD1306 if you flip USE_SH1106 to 0)
     Adafruit GFX Library     <- must be current; SH110X needs Adafruit_GrayOLED
     Adafruit BusIO
     Bluefruit nRF52 (built into the Seeed XIAO nRF52840 board package)
   ========================================================================== */

#include <Wire.h>
#include <Adafruit_MCP9600.h>
#include <bluefruit.h>

// ---- SCREEN DRIVER: flip this if the boot border test looks wrong ----------
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
#else
  #include <Adafruit_SSD1306.h>
  Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire, -1);
  #define PX_ON   SSD1306_WHITE
  #define PX_OFF  SSD1306_BLACK
  #define DRV_NAME "SSD1306"
#endif

#define BTN_PIN     D1
#define READ_MS    100        // MCP @16-bit converts in 80ms
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

// ---- PW-ADV-1 payload (14 bytes, little-endian fields) --------------------
// Bluefruit's addManufacturerData() passes the buffer through RAW - it does
// NOT prepend a company ID - so bytes 0-1 of this array ARE the company ID
// and the logger indexes the array identically. Layout:
//   0-1  company ID FF FF     2-3  magic 'P' 'W'      4  proto version 01
//   5    flags (bit0 pairing, bit1 TC fault)
//   6-7  EGT int16 deci-degC  8-9  CJ int16 deci-degC
//   10   raw MCP9600 STATUS   11   battery stub FF    12-13 sequence
#define ADV_PAYLOAD_LEN 14
#define ADV_PROTO_VER   0x01

Adafruit_MCP9600 mcp;

uint8_t  mcpAddr  = 0;
uint8_t  oledAddr = 0;
bool     oledOK   = false;
bool     showF    = true;
uint32_t nRead    = 0;

uint16_t advSeq    = 0;
uint32_t pairUntil = 0;       // millis deadline; 0 = pairing window closed
bool     advOK     = false;   // last Advertising.start() result
uint32_t advFails  = 0;       // consecutive-rebuild failure count (debug)

float c2f(float c) { return c * 9.0f / 5.0f + 32.0f; }

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

// -------------------------------------------------- I2C bus scan
void scanBus() {
  Serial.println("\n--- I2C scan ---");
  uint8_t n = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() != 0) continue;
    n++;
    Serial.print("  0x"); Serial.print(a, HEX);
    if (a == 0x3C || a == 0x3D) {
      Serial.print("   <- OLED");
      if (!oledAddr) oledAddr = a;
    } else if (a >= 0x60 && a <= 0x67) {
      Serial.print("   <- MCP9600");
      if (!mcpAddr) mcpAddr = a;
    }
    Serial.println();
  }
  if (n == 0) Serial.println("  NOTHING FOUND -> check SDA=D4, SCL=D5, 3V3, GND");
  Serial.print("--- "); Serial.print(n); Serial.println(" device(s) ---\n");
}

// -------------------------------------------------- raw STATUS read (reg 0x04)
// Done by hand instead of mcp.getStatus() so it compiles on any lib version.
uint8_t mcpStatus() {
  if (!mcpAddr) return 0xFF;
  Wire.beginTransmission(mcpAddr);
  Wire.write(0x04);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  if (Wire.requestFrom(mcpAddr, (uint8_t)1) != 1) return 0xFF;
  return Wire.read();
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
// 0x8000 (INT16_MIN) = "no valid reading". Emit the sentinel instead of
// casting NaN / out-of-range floats to int16_t - that cast is undefined
// behavior and produces plausible-looking garbage.
int16_t encodeDeciC(float c) {
  if (isnan(c) || c < -270.0f || c > 1400.0f) return INT16_MIN;
  return (int16_t)lroundf(c * 10.0f);
}

void buildAdvPayload(uint8_t out[ADV_PAYLOAD_LEN], float egtC, float cjC,
                     uint8_t st) {
  uint8_t flags = 0;
  if (millis() < pairUntil) flags |= 0x01;   // pairing window active
  if (st & 0x10)            flags |= 0x02;   // MCP input-range = TC fault

  int16_t egt = encodeDeciC(egtC);
  int16_t cj  = encodeDeciC(cjC);

  out[0]  = 0xFF; out[1] = 0xFF;             // company ID (SIG test/internal)
  out[2]  = 'P';  out[3] = 'W';              // magic
  out[4]  = ADV_PROTO_VER;
  out[5]  = flags;
  out[6]  = (uint8_t)(egt & 0xFF);
  out[7]  = (uint8_t)((egt >> 8) & 0xFF);
  out[8]  = (uint8_t)(cj & 0xFF);
  out[9]  = (uint8_t)((cj >> 8) & 0xFF);
  out[10] = st;
  out[11] = 0xFF;                            // battery: stub
  out[12] = (uint8_t)(advSeq & 0xFF);
  out[13] = (uint8_t)((advSeq >> 8) & 0xFF);
}

// Rebuild the advertising data with fresh sensor values. stop -> clear ->
// rebuild -> start is ugly and correct: Bluefruit has no supported
// in-place advertising-data mutation path. Do not invent one.
// start()'s result is tracked: a failed restart leaves the radio silent
// until the next tick, and a RUN of failures is the one egg-side fault
// that looks exactly like a dead egg to the logger - so it's surfaced on
// the debug screen ("ADV!") and serial instead of being swallowed.
void updateAdvertising(float egtC, float cjC, uint8_t st) {
  uint8_t payload[ADV_PAYLOAD_LEN];
  advSeq++;                                  // one increment per adv update
  buildAdvPayload(payload, egtC, cjC, st);

  Bluefruit.Advertising.stop();
  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addManufacturerData(payload, ADV_PAYLOAD_LEN);
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
  }
}

// -------------------------------------------------- fatal halt
void fatal(const char* msg) {
  Serial.print("FATAL: "); Serial.println(msg);
  while (1) {
    if (oledOK) {
      oled.clearDisplay();
      oled.setTextSize(2); oled.setTextColor(PX_ON, PX_OFF);
      oled.setCursor(0, 0);  oled.print("FATAL");
      oled.setTextSize(1);
      oled.setCursor(0, 24); oled.print(msg);
      oled.display();
    }
    delay(1000);
  }
}

// -------------------------------------------------- boot border test
// Correct driver  -> border hugs all four edges, X hits all four corners.
// Wrong driver    -> 2px gap one side, clipped/noisy stripe the other.
void borderTest() {
  oled.clearDisplay();
  oled.drawRect(0, 0, OLED_W, OLED_H, PX_ON);
  oled.drawLine(0, 0, OLED_W - 1, OLED_H - 1, PX_ON);
  oled.drawLine(OLED_W - 1, 0, 0, OLED_H - 1, PX_ON);
  oled.setTextSize(1);
  oled.setTextColor(PX_ON, PX_OFF);
  oled.setCursor(40, 28);
  oled.print(DRV_NAME);
  oled.display();
  delay(2500);
}

// -------------------------------------------------- main screen
void drawScreen(float egtC, float cjC, uint8_t st) {
  if (!oledOK) return;
  float egt = showF ? c2f(egtC) : egtC;
  float cj  = showF ? c2f(cjC)  : cjC;

  oled.clearDisplay();
  oled.setTextColor(PX_ON, PX_OFF);

  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print("EGT @0x"); oled.print(mcpAddr, HEX);
  oled.setCursor(OLED_W - 6, 0);
  oled.print(showF ? "F" : "C");

  oled.setTextSize(3);
  oled.setCursor(0, 14);
  if (isnan(egt)) oled.print("---");
  else            oled.print(egt, 1);

  oled.setTextSize(1);
  oled.setCursor(0, 42);
  oled.print("CJ "); oled.print(cj, 1); oled.print(showF ? " F" : " C");

  oled.setCursor(0, 54);
  oled.print("ST ");
  if (st < 0x10) oled.print("0");
  oled.print(st, HEX);
  if (st & 0x10) oled.print(" OPEN?");
  oled.print(" #"); oled.print(advSeq);         // adv sequence counter
  if (!advOK)               oled.print(" ADV!"); // last adv restart failed
  else if (millis() < pairUntil) oled.print(" PAIR");

  oled.display();
}

// -------------------------------------------------- setup
void setup() {
  pinMode(BTN_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  while (!Serial && millis() < 3000) { }     // don't block forever on battery

  Serial.println("\nDovesSensorEgg - wireless EGT pod (PW-ADV-1)");

  i2cBusClear();               // recover a slave wedged by a mid-read reset
  Wire.begin();
  // 100 kHz, not 400: the MCP9600's I2C tops out at 100 kHz per datasheet
  // (it also clock-stretches). 400 kHz ACKs the address scan but the
  // multi-byte register reads - begin()'s device-ID check first - are
  // marginal and fail intermittently. The OLED doesn't care.
  Wire.setClock(100000);
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

  if (oledOK) {
    borderTest();
    oled.clearDisplay();
    oled.setTextSize(1); oled.setTextColor(PX_ON, PX_OFF);
    oled.setCursor(0, 0);  oled.print("I2C SCAN");
    oled.setCursor(0, 18); oled.print("OLED 0x"); oled.print(oledAddr, HEX);
    oled.setCursor(0, 32); oled.print("MCP  ");
    if (mcpAddr) { oled.print("0x"); oled.print(mcpAddr, HEX); }
    else           oled.print("NOT FOUND");
    oled.display();
    delay(2000);
  }

  if (!mcpAddr) fatal("MCP not on bus");

  // begin() = the device-ID read at reg 0x20 (expects 0x40/0x41 in the
  // high byte). Retry a few times - the MCP9600 is slow to wake and can
  // need a moment after a bus clear - and dump the raw ID on failure so
  // a bad read is distinguishable from a wrong/absent chip.
  bool mcpOK = false;
  for (int attempt = 1; attempt <= 5 && !mcpOK; attempt++) {
    mcpOK = mcp.begin(mcpAddr);
    if (!mcpOK) {
      Serial.print("MCP begin() attempt "); Serial.print(attempt);
      Serial.print(" failed, ID reg 0x20 = 0x");
      Wire.beginTransmission(mcpAddr);
      Wire.write(0x20);
      Wire.endTransmission(false);
      if (Wire.requestFrom(mcpAddr, (uint8_t)2) == 2) {
        Serial.print(Wire.read(), HEX); Serial.print(" 0x");
        Serial.println(Wire.read(), HEX);
      } else {
        Serial.println("<read failed>");
      }
      delay(200);
    }
  }
  if (!mcpOK) fatal("MCP begin() fail");

  mcp.setThermocoupleType(MCP9600_TYPE_K);
  mcp.setADCresolution(MCP9600_ADCRESOLUTION_16);   // 80ms. NOT the 18-bit default.
  mcp.setFilterCoefficient(3);
  mcp.enable(true);

  Serial.print("MCP9600 up @0x"); Serial.println(mcpAddr, HEX);

  bleSetup();

  Serial.println("D1 short = C/F toggle, long = pairing window\n");
}

// -------------------------------------------------- loop
void loop() {
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

  if (millis() - tRead >= READ_MS) {
    tRead = millis();
    egtC  = mcp.readThermocouple();
    cjC   = mcp.readAmbient();
    st    = mcpStatus();
    nRead++;
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
    drawScreen(egtC, cjC, st);
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
