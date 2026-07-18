# DovesSensorEgg

Wireless sensor backpack for
[DovesDataLogger](https://github.com/TheAngryRaven/DovesDataLogger).

A Seeed XIAO nRF52840 + Adafruit MCP9600 thermocouple amp reads a K-type
EGT probe and **broadcasts** the reading in BLE advertising packets —
protocol `PW-ADV-1`. The egg is a pure broadcaster: it never accepts a
connection, so the logger receives it with a passive scan that cannot
interfere with the logger's Insta360 camera link. A small SSD1306 OLED
shows live temps for bench debugging.

## Protocol — PW-ADV-1

Manufacturer Specific Data, 14 bytes, advertised at ~10 Hz under the
name `PWEGT` (multi-byte fields little-endian). The company ID is
*inside* the array — Bluefruit passes the buffer through raw on both
sides:

| Byte | Field |
|---|---|
| 0–1 | Company ID `FF FF` (SIG test/internal) |
| 2–3 | Magic `50 57` (`PW`) |
| 4 | Protocol version `01` |
| 5 | Flags: bit0 = pairing window, bit1 = TC fault |
| 6–7 | EGT, int16 deci-°C (`0x8000` = invalid) |
| 8–9 | Cold junction, int16 deci-°C (`0x8000` = invalid) |
| 10 | Raw MCP9600 STATUS (reg 0x04; bit 4 = open probe) |
| 11 | Battery % (stub, always `0xFF`) |
| 12–13 | Sequence counter (wraps) |

NaN / out-of-range readings are sent as the `0x8000` sentinel — never
cast to int16 (UB). The logger treats readings older than 1 s as gone
(logs `nan`) so a dropout is never a held flat line.

## Hardware

| Signal | XIAO pin |
|---|---|
| MCP9600 + OLED SCL | D5 |
| MCP9600 + OLED SDA | D4 |
| Button | D1 ↔ GND (`INPUT_PULLUP`) |

I2C addresses are auto-detected at boot (`0x3C/0x3D` OLED,
`0x60–0x67` MCP9600). K-type polarity: **red is negative** (US ANSI).

## Controls

- **Short press** — toggle °C/°F on the debug screen.
- **Long press (>1 s)** — open a 30 s pairing window (sets flags bit0;
  informational — the logger pairs by MAC or payload magic).

The egg prints its MAC on serial and the OLED at boot — copy it into the
logger's `SENSOREGG_MAC` define for strict pairing, or leave the logger's
default all-zeros to accept any egg.

## Libraries

Adafruit MCP9600, Adafruit SSD1306 (or SH110X — `USE_SH1106` flag),
Adafruit GFX, Adafruit BusIO; Bluefruit nRF52 comes with the Seeed XIAO
nRF52840 board package.
