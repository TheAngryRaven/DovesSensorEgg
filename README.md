# DovesSensorEgg

> **Status: experimental.** This is a proof-of-concept — the "wireless
> sensor pod" idea is promising as a unit, but it needs more work and
> conceptualization before being taken further. CI is deliberately basic
> (a compile check and a couple of unit tests, below); no release
> pipeline or auto-updating manifests until the concept firms up.

Wireless sensor backpack for
[DovesDataLogger](https://github.com/TheAngryRaven/DovesDataLogger).

A Seeed XIAO nRF52840 + Adafruit MCP9600 thermocouple amp reads a K-type
EGT probe and **broadcasts** the reading in BLE advertising packets —
protocol `PW-ADV-1`. The egg is a pure broadcaster: it never accepts a
connection, so the logger receives it with a passive scan that cannot
interfere with the logger's Insta360 camera link. A small SSD1306 OLED
shows live temps for bench debugging.

## Protocol — PW-ADV-1

Manufacturer Specific Data, 14 bytes, advertised under the name `PWEGT`
(multi-byte fields little-endian). Advertising interval is **111.875 ms
— deliberately not 100 ms**: the logger scans on a 100 ms cycle, and
equal periods phase-lock so the egg can park in the scanner's deaf zone
for seconds; an off-100 interval sweeps the phase instead. The payload
(and sequence counter) refresh every 250 ms — still far inside a K-type
probe's 0.5–3 s thermal time constant. The company ID is *inside* the
array — Bluefruit passes the buffer through raw on both sides:

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
- **Hold 10 s** — deep sleep (nRF52 System OFF, ~µA: radio silent, MCP9600
  in shutdown, display off — the enclosure has no power switch). A
  countdown appears on-screen from ~2 s into the hold.
- **Hold 5 s to wake** — the press wakes the chip, but boot drops straight
  back to sleep unless the button stays held for 5 s. Nothing is drawn
  during the gate, so a pocket bump never lights the screen or drains the
  battery. (A watchdog reboot skips the gate — after a hang the pod comes
  back broadcasting on its own.)

The egg prints its MAC on serial and the OLED at boot — copy it into the
logger's `SENSOREGG_MAC` define for strict pairing, or leave the logger's
default all-zeros to accept any egg.

## Reliability

The nRF52840 hardware **watchdog** (8 s) is armed first thing at boot and
fed once per `loop()` pass: a hang anywhere — the known case is a blocking
MCP9600 I2C transaction wedged by ignition EMI, which left the radio
beaconing a frozen payload for hours (a "zombie egg") — reboots the pod
within seconds. Boot's I2C bus-clear then recovers the wedged bus, and the
sequence counter restarting tells the logger's zombie detection the egg is
live again. A `FATAL` boot failure (probe absent) shows its screen for
30 s, then hard-resets and retries, so a transient boot glitch self-heals
in the field. A watchdog reboot is reported on serial
(`!! WATCHDOG REBOOT`) and as `WDT RESET` on the boot scan screen.

## Libraries

Adafruit MCP9600, Adafruit SSD1306 (or SH110X — `USE_SH1106` flag),
Adafruit GFX, Adafruit BusIO; Bluefruit nRF52 comes with the Seeed XIAO
nRF52840 board package.

## CI & tests

Two deliberately-basic GitHub Actions workflows (same shapes as the
DovesDataLogger repo's, minus everything release-related):

- **compile-sketch** — compiles the sketch for the Seeed XIAO nRF52840
  (the same board the datalogger uses) with the real libraries.
- **unit-tests** — host-built doctest suite over the extracted pure
  logic. `pw_adv_encode.{h,cpp}` builds the PW-ADV-1 payload, and its
  golden-byte test uses the **same fixture bytes** as the logger repo's
  `sensoregg_protocol` parser test — the two tests together pin the wire
  contract from both ends. Run locally:

  ```bash
  cmake -S tests -B tests/build
  cmake --build tests/build --parallel
  ctest --test-dir tests/build --output-on-failure
  ```
