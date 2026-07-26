# DovesSensorEgg

> **Status: experimental.** This is a proof-of-concept — the "wireless
> sensor pod" idea is promising as a unit, but it needs more work and
> conceptualization before being taken further. CI is deliberately basic
> (a compile check and a couple of unit tests, below); no release
> pipeline or auto-updating manifests until the concept firms up.

Wireless sensor backpack for
[DovesDataLogger](https://github.com/TheAngryRaven/DovesDataLogger).

A Seeed XIAO nRF52840 + Adafruit MCP9600 thermocouple amp reads a K-type
EGT probe (plus an aux 100k NTC thermistor and its own battery level) and
**broadcasts** the readings in BLE advertising packets — protocol
`PW-ADV-2`. The egg is a pure broadcaster: it never accepts a
connection, so the logger receives it with a passive scan that cannot
interfere with the logger's Insta360 camera link. A small SSD1306 OLED
shows live temps for bench debugging.

## Protocol — PW-ADV-2

> **Logger compatibility:** the DovesDataLogger's parser is still on v1 —
> it requires version `0x01` exactly and truncates received payloads at
> 14 bytes, so it silently **drops** v2 frames until its parser round
> lands. Bench-verify a v2 egg with nRF Connect (16-byte mfg data
> starting `FF FF 50 57 02`).

Manufacturer Specific Data, 16 bytes, advertised under the name `PWEGT`
(multi-byte fields little-endian). Advertising interval is **111.875 ms
— deliberately not 100 ms**: the logger scans on a 100 ms cycle, and
equal periods phase-lock so the egg can park in the scanner's deaf zone
for seconds; an off-100 interval sweeps the phase instead. The payload
(and sequence counter) refresh every 250 ms — still far inside a K-type
probe's 0.5–3 s thermal time constant. The company ID is *inside* the
array — Bluefruit passes the buffer through raw on both sides:

v2 appends to v1 — bytes 0–13 keep their exact v1 offsets:

| Byte | Field |
|---|---|
| 0–1 | Company ID `FF FF` (SIG test/internal) |
| 2–3 | Magic `50 57` (`PW`) |
| 4 | Protocol version `02` |
| 5 | Flags: bit0 = pairing window, bit1 = TC fault |
| 6–7 | EGT, int16 deci-°C (`0x8000` = invalid) |
| 8–9 | Cold junction, int16 deci-°C (`0x8000` = invalid) |
| 10 | Raw MCP9600 STATUS (reg 0x04; bit 4 = open probe) |
| 11 | Battery % 0–100 (`0xFF` = unknown / no pack) |
| 12–13 | Sequence counter (wraps) |
| 14–15 | Aux thermistor, int16 deci-°C (`0x8000` = invalid) |

An open/shorted thermistor divider is signalled by the `0x8000` sentinel
alone — no flag bit. Budget: 3 (flags AD) + 18 (mfg AD) + 7 (name) =
28 of the 31-byte legacy advertising payload.

NaN / out-of-range readings are sent as the `0x8000` sentinel — never
cast to int16 (UB). The logger treats readings older than 1 s as gone
(logs `nan`) so a dropout is never a held flat line.

## Hardware

> **Wiring changed 2026-07-20**: the MCP9600 moved to its own bus (D2/D3).
> Older builds had it sharing D4/D5 with the OLED.

| Signal | XIAO pin |
|---|---|
| OLED SCL (hardware `Wire`, 400 kHz) | D5 |
| OLED SDA | D4 |
| MCP9600 SCL (dedicated soft bus, ~50 kHz) | D3 |
| MCP9600 SDA | D2 |
| Button | D1 ↔ GND (`INPUT_PULLUP`) |
| Thermistor sense (100k NTC divider midpoint) | A0/D0 |
| Thermistor power gate | D6 |

> **The hardware `Wire` on this core can hang forever.** It spins on TWIM
> events with **no timeout** (`while (!_p_twim->EVENTS_STOPPED);` has no
> error escape at all), so a transfer that goes wrong badly enough never
> returns, and the watchdog turns that into an exact ~9 s reboot cycle
> (8 s WDT + boot overhead) — the 2026-07-26 boot loop. Root cause, found
> three rounds in: a **mis-soldered power harness** (GND on 3V3, VCC on an
> IO pin) wedging the first TWIM transfer of every boot. Not firmware —
> but it proved the class is real, so the guards stay: the sketch never
> enters `Wire` blind (the OLED must first ACK a real addressed probe on a
> temporary, timeout-capable **soft bus** over the same pins), and if a
> boot dies inside display bring-up anyway, the next boot skips the
> display outright (the **display deadman**, below). `OLED_I2C_HZ`
> (400 kHz — the panel's rated speed, viable because the breakout has its
> own pullups) must be given to the *display driver* as well as to `Wire`
> — Adafruit_SSD1306 and SH110X re-assert their constructor speed around
> every transfer, so a display constructed without those arguments ignores
> `Wire.setClock()`.

### Wiring truth table (XIAO, USB connector up)

```
left column, top to bottom          right column, top to bottom
  D0  <- thermistor sense (A0)        5V
  D1  <- button (to GND)              GND
  D2  <- MCP9600 SDA                  3V3  <- both modules' VCC
  D3  <- MCP9600 SCL                  D10
  D4  <- OLED SDA  (chip SDA pin)     D9
  D5  <- OLED SCL  (chip SCL pin)     D8
  D6  <- thermistor power gate        D7
```

**Thermistor divider** (aux temperature, payload bytes 14–15): 100k NTC
from **D6** to the sense node, 100k fixed from the sense node to **A0/D0
and GND**, and a **10 nF ceramic smoothing cap from the sense node to
GND** — placed at the XIAO end of the leads, not out at the thermistor,
with the NTC run as a twisted pair (the leads live near ignition wiring
and are antennas, same as the TC lead). The cap is load-bearing twice
over: it low-passes ignition spikes (~320 Hz cutoff at mid-scale), and
it is what makes the ADC timing legitimate — this core's SAADC uses a
fixed 3 µs acquisition, rated for ≤10 kΩ source impedance, while the
divider's Thevenin impedance reaches ~100 kΩ with a cold NTC; the cap is
a local charge reservoir ~4000× the SAADC's sample cap. Optional extra:
~1 kΩ in series from the node into A0 for RF/pin protection (no ratio
error — at DC the ADC draws from the cap, not through the resistor).

D6 is driven high only for the ~13 ms around each 1 Hz read — no idle
drain, no self-heating, nothing left energized in deep sleep. **The cap
value and `THERM_SETTLE_MS` are coupled**: the node charges through the
divider each pulse (worst-case τ = 100 kΩ × C, cold NTC), and the settle
must be ≥ ~10 τ or cold readings come out low. The define in the sketch
carries the table (10 nF → 12 ms, 100 nF → 75 ms, …) — change the cap,
change the number. The read is ratiometric (`AR_VDD4` reference = the
same rail D6 drives), so no calibration constant exists; tune
`THERM_R0/B/R_FIXED` in `thermistor.h` if the part deviates from
100k/3950.

**Battery** (payload byte 11): the XIAO's onboard 1M/510k divider on
`PIN_VBAT`, gated by `VBAT_ENABLE` and pulsed the same way (every 30 s).
The counts→volts calibration (`×3.024`) is ported from DovesDataLogger,
where it was empirically corrected against a true 4.20 V full charge;
percent is the same linear 3.3 V = 0% … 4.2 V = 100% window. Below
2.5 V the pod reports `0xFF` (unknown) — a USB-only bench with no pack
must not broadcast a lying 0%.

Both breakout modules also need **3V3 and GND**. Two buses going silent
at the same time is almost always the shared power feed, not the data
pins — the harness diagnostic below will say which.

### Harness diagnostic

If either device is missing at boot, the pod diagnoses its own wiring on
the bounded soft bus (never the hardware `Wire`):

- **Per-line electrical state** for each bus pin: `STUCK LOW` (short, or
  an unpowered device clamping the line), `no external pullup` (module
  absent or **unpowered** — every breakout in this build carries its own
  pullups, so their absence means the module isn't there electrically),
  or `ok`.
- **Full-address scan (0x08–0x77) over every pin pairing of D2–D5** —
  canonical and swapped, both buses — with an MCP9600 ID handshake and
  OLED recognition. A module wired with SDA/SCL swapped, or plugged into
  the other bus, is *found and named*. The MCP9600 is **adopted wherever
  it answers** (its bus is soft — the pins are just numbers) and the log
  tells you the canonical rewire; an OLED found off `SDA→D4 SCL→D5`
  can't be adopted (the TWIM's pins are fixed) so the log names the
  exact wires to swap. While no sensor is adopted, `loop()`'s recovery
  tick repeats a quiet version of the sweep every 2 s — plugging the
  sensor in late just works.

The MCP9600's bus is bit-banged with a per-transaction timeout and runs
deliberately slow for EMI margin — the chip clock-stretches aggressively
and its probe lead is an antenna. 4.7 kΩ pullups to 3V3 recommended on
D2/D3 (the internal ~13 kΩ pullups are workable at this speed). Addresses
auto-detect at boot: OLED `0x3C/0x3D` (probe), MCP9600 `0x60–0x67` via a
retried **ID-register handshake** — never a blind probe, which this chip
is known to dislike. K-type polarity: **red is negative** (US ANSI).

## Controls

- **Short press** — toggle °C/°F on the debug screen.
- **Long press (>1 s)** — open a 30 s pairing window (sets flags bit0;
  informational — the logger pairs by MAC or payload magic).
- **Hold 10 s** — deep sleep (nRF52 System OFF, ~µA: radio silent, MCP9600
  in shutdown, display off — the enclosure has no power switch). A
  countdown appears on-screen from ~2 s into the hold. **Unplug USB
  first**: the nRF52840 cannot hold System OFF with bus power present, so
  the pod says `CAN'T SLEEP` and stays awake rather than resetting.
- **Hold 5 s to wake** — the press wakes the chip, but boot drops straight
  back to sleep unless the button stays held for 5 s. Nothing is drawn
  during the gate, so a pocket bump never lights the screen or drains the
  battery. Every level decision in the gate is debounced. (A watchdog
  reboot skips the gate — after a hang the pod comes back broadcasting on
  its own. So does safe mode, below.)

The egg prints its MAC on serial and the OLED at boot — copy it into the
logger's `SENSOREGG_MAC` define for strict pairing, or leave the logger's
default all-zeros to accept any egg.

## Reliability

Layered, most-specific first:

1. **Timeout-capable sensor bus**: every MCP9600 transaction is bounded —
   a wedged or forever-stretching slave returns an error instead of
   parking the loop. Readings become the wire's `0x8000` invalid sentinel
   (logger shows `---`), and the sequence counter keeps advancing, so a
   sick sensor can never zombie the pod.
2. **Runtime recovery**: ~0.5 s of consecutive bus faults triggers a bus
   clear + a Shutdown→Normal **mode-cycle reconfigure** (the chip's
   nearest thing to a reset command), throttled to one attempt per 2 s —
   the sensor is fixed in place, no reboot.
3. **Watchdog**: the nRF52840 hardware **watchdog** (8 s) is armed first
   thing at boot and fed once per `loop()` pass: a hang anywhere reboots
   the pod within seconds (the original 2026-07-19 "zombie egg" incident
   — a wedged sensor read left the radio beaconing a frozen payload for
   hours). Boot's bus clears recover the wedged wires, and the sequence
   counter restarting tells the logger's zombie detection the egg is live
   again. A watchdog reboot is reported on serial (`!! WATCHDOG REBOOT`)
   and as `WDT RESET` on the boot scan screen.
4. **Bring-up never reboots**: a missing or unhealthy MCP9600 shows
   `NO SENSOR` and the pod boots anyway — radio and screen up, EGT on the
   invalid sentinel, sensor re-detected and reconfigured by the recovery
   tick above until it answers. Rebooting is the one recovery that cannot
   fix a failing boot, and the old 30 s `FATAL` hard-reset turned an
   absent sensor into an endless reboot cycle (and a pod that was
   near-impossible to re-flash, because the USB CDC port vanished every
   30 s).
5. **The display bus is never entered blind**: the hardware `Wire` is the
   only call in the sketch with no way back, so the OLED must ACK a real
   addressed probe on a temporary timeout-capable soft bus (bus clear +
   START/address/ACK/STOP, every edge bounded) before the TWIM is allowed
   near D4/D5. No ACK — absent, unpowered, or clamping the lines — and the
   display is dropped: the pod boots serial + BLE only. A pod broadcasting
   with a dark screen beats one that loops.
6. **Display deadman**: bring-up records which stage it is entering in a
   magic-tagged `.noinit` RAM block that survives warm resets. If the
   previous run died inside display bring-up — the only stage that *can*
   hang unrecoverably — the next boot skips the display outright and says
   so. Converges in one reboot; a power cycle retries the display.
7. **Boot-loop breaker**: consecutive boots without a healthy run are
   counted in the same `.noinit` block (RAM is the trusted store — field
   logs show `RESETREAS` and `GPREGRET2` come back scrubbed on this
   board's bootloader, so registers are written but only reported as
   evidence). A run that stays up 15 s clears the streak. Three boots
   without a healthy run means bring-up is cycling, so the pod comes up in
   **safe mode**: the watchdog is held off until `loop()` is actually
   running, the display is not brought up at all, boot screens and deep
   sleep are disabled. Safe mode is deliberately boring — the point is a
   pod that sits still, holds its USB enumeration and can be re-flashed
   without the DFU dance. Power-cycle after a good run to clear it.
8. **Boot forensics on serial**: every boot prints one line with the boot
   count, whether the RAM scratch survived, **which stage the previous run
   died in**, the raw `GPREGRET2` byte and `RESETREAS`; each stage then
   prints `[boot] <stage>`, flushed before entering it, so the last line
   on the wire always names the step that hung.

## Libraries

Adafruit SSD1306 (or SH110X — `USE_SH1106` flag), Adafruit GFX,
Adafruit BusIO; Bluefruit nRF52 comes with the Seeed XIAO nRF52840 board
package. The Adafruit MCP9600 library is no longer used — the sensor is
driven by the in-repo `mcp9600.{h,cpp}` register driver over the
timeout-capable `soft_i2c` bus (register codecs host-tested in
`mcp9600_regs`).

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
