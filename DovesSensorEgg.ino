/* ==========================================================================
   PerchWerks EGT Pod - BENCH BRINGUP  (no BLE)
   XIAO nRF52840 + Adafruit MCP9600 + I2C OLED + button

   Boot -> scan I2C -> border test -> live temp on screen + serial
   Button on D1 toggles C / F

   Libraries (Arduino Library Manager):
     Adafruit MCP9600
     Adafruit SH110X          (or Adafruit SSD1306 if you flip USE_SH1106 to 0)
     Adafruit GFX Library     <- must be current; SH110X needs Adafruit_GrayOLED
     Adafruit BusIO
   ========================================================================== */

#include <Wire.h>
#include <Adafruit_MCP9600.h>

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
#define DRAW_MS    200
#define SERIAL_MS  250

Adafruit_MCP9600 mcp;

uint8_t  mcpAddr  = 0;
uint8_t  oledAddr = 0;
bool     oledOK   = false;
bool     showF    = true;
uint32_t nRead    = 0;

float c2f(float c) { return c * 9.0f / 5.0f + 32.0f; }

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
bool btnPressed() {
  static bool     prev  = true;      // INPUT_PULLUP: idle HIGH
  static uint32_t tLast = 0;
  bool now = digitalRead(BTN_PIN);
  if (now == prev) return false;
  if (millis() - tLast < 40) return false;
  tLast = millis();
  prev  = now;
  return (now == LOW);               // fire on press
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
  else         { oled.print("  n"); oled.print(nRead); }

  oled.display();
}

// -------------------------------------------------- setup
void setup() {
  pinMode(BTN_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  while (!Serial && millis() < 3000) { }     // don't block forever on battery

  Serial.println("\nPerchWerks EGT pod - bench bringup");

  Wire.begin();
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

  if (!mcpAddr)          fatal("MCP not on bus");
  if (!mcp.begin(mcpAddr)) fatal("MCP begin() fail");

  mcp.setThermocoupleType(MCP9600_TYPE_K);
  mcp.setADCresolution(MCP9600_ADCRESOLUTION_16);   // 80ms. NOT the 18-bit default.
  mcp.setFilterCoefficient(3);
  mcp.enable(true);

  Serial.print("MCP9600 up @0x"); Serial.println(mcpAddr, HEX);
  Serial.println("D1 = C/F toggle\n");
}

// -------------------------------------------------- loop
void loop() {
  static uint32_t tRead = 0, tDraw = 0, tSer = 0;
  static float    egtC = NAN, cjC = NAN;
  static uint8_t  st = 0;

  if (btnPressed()) {
    showF = !showF;
    Serial.print("unit -> "); Serial.println(showF ? "F" : "C");
    tDraw = 0;                                   // repaint immediately
  }

  if (millis() - tRead >= READ_MS) {
    tRead = millis();
    egtC  = mcp.readThermocouple();
    cjC   = mcp.readAmbient();
    st    = mcpStatus();
    nRead++;
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
