# DovesSensorEgg

> **Status: reference implementation, evolving.** This repo is becoming
> the reference pod of the **PerchWerks Sensor Service** — a
> standardized, third-party-buildable wireless sensor pod system. The
> firmware currently shipping is the broadcast-only PW-ADV-2 stage; the
> system design lives in [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
> and the migration plan in [docs/ROADMAP.md](docs/ROADMAP.md). CI is
> deliberately basic (a compile check and unit tests, below); a release
> pipeline comes later in the roadmap.

Wireless sensor backpack for
[DovesDataLogger](https://github.com/TheAngryRaven/DovesDataLogger).

A Seeed XIAO nRF52840 + Adafruit MCP9600 thermocouple amp reads a K-type
EGT probe (plus an aux NTC thermistor for intake air and its own
battery level) and **broadcasts** the readings in BLE advertising
packets — protocol `PW-ADV-2` — which the logger receives with a
passive scan that cannot interfere with its Insta360 camera link. Since
phase 1 of the [PerchWerks Sensor Service](docs/ROADMAP.md) migration
the egg is also **connectable**: a connection currently serves the
standard GATT mirrors only — Device Information; Battery Service when a
pack is present at boot; Environmental Sensing temperature for the
intake-air thermistor and cold junction (never the EGT, which overflows
the standard characteristic — see
[docs/PW_SENSOR_SERVICE.md](docs/PW_SENSOR_SERVICE.md) §7). While a
link is up the beacon pauses; it resumes on disconnect, so an unclaimed
egg broadcasts exactly as before. A small SSD1306 OLED shows live temps
for bench debugging (a `LINK` badge marks a live connection).

## Documentation

| Doc | What it is |
|---|---|
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | System design + rationale, decision ledger |
| [docs/PW_SENSOR_SERVICE.md](docs/PW_SENSOR_SERVICE.md) | The GATT service spec (normative) |
| [docs/PW_CHANNEL_SCHEMA.md](docs/PW_CHANNEL_SCHEMA.md) | Transport-neutral channel schema — the universal protocol |
| [docs/PW_ADV_2.md](docs/PW_ADV_2.md) | The shipped broadcast beacon spec (normative) |
| [docs/ROADMAP.md](docs/ROADMAP.md) | Migration phases + parking lot |

## Protocol — PW-ADV-2

Full spec: **[docs/PW_ADV_2.md](docs/PW_ADV_2.md)** — byte table, radio
parameters, versioning and conformance anchors live there (single home;
moved out of this README).

Summary: 16-byte Manufacturer Specific Data under the name `PWEGT`,
advertised at 111.875 ms (deliberately off the logger's 100 ms scan
cycle so the beacon can't park in the scanner's deaf zone), payload and
sequence counter refreshed every 250 ms. Temperatures ride as int16
deci-°C with `0x8000` as the invalid sentinel (never a NaN cast — UB),
plus battery percent, raw MCP9600 STATUS, and flags for the pairing
window and TC fault. The logger's v2 parser round has **landed**
(DovesDataLogger BETA accepts v1 and v2); nRF Connect remains a handy
neutral bench check (16-byte mfg data starting `FF FF 50 57 02`).

Under the [PerchWerks Sensor Service](docs/PW_SENSOR_SERVICE.md)
migration this beacon survives byte-identical as the pre-pairing
advertisement; connections carry everything else.

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
| Thermistor sense (NTC divider midpoint) | A0/D0 |
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
  D0  <- thermistor sense node (A0)   5V
  D1  <- button (to GND)              GND
  D2  <- MCP9600 SDA                  3V3  <- both modules' VCC
  D3  <- MCP9600 SCL                  D10
  D4  <- OLED SDA  (chip SDA pin)     D9
  D5  <- OLED SCL  (chip SCL pin)     D8
  D6  <- thermistor power gate        D7
```

**Thermistor divider** (aux temperature, payload bytes 14–15): 100k
fixed resistor from **D6** to the sense node, the NTC from the sense
node to **GND** (currently the sealed 10k metal probe for intake-air
duty — profiles and the deliberate 10k-against-100k "lazy divider"
math live in `thermistor.h`), sense node to **A0/D0**, and a **10 nF ceramic
smoothing cap from the sense node to GND** (in parallel with the NTC —
NOT in series between the node and A0, which DC-blocks the pin and pegs
the reading at the LOW rail) — placed at the XIAO end of the leads, not
out at the thermistor,
with the NTC run as a twisted pair (the leads live near ignition wiring
and are antennas, same as the TC lead). The cap is load-bearing twice
over: it low-passes ignition spikes (~320 Hz cutoff at mid-scale), and
it is what makes the ADC timing legitimate — this core's SAADC uses a
fixed 3 µs acquisition, rated for ≤10 kΩ source impedance, while the
divider's Thevenin impedance reaches ~100 kΩ with a cold NTC; the cap is
a local charge reservoir ~4000× the SAADC's sample cap. Optional extra:
~1 kΩ in series from the node into A0 for RF/pin protection (no ratio
error — at DC the ADC draws from the cap, not through the resistor).

**Troubleshooting tell**: the serial line's measured `R=` is ground
truth, independent of the codec constants. If it reads a *suspiciously
exact standard resistor value* at ambient, a literal resistor is in the
harness where it shouldn't be — the 2026-07-27 hunt ended at `R=1.0k`,
which turned out to be a mispicked part soldered in as the smoothing
cap (real ceramics are marked `103`/`104` and have no color bands; they
read as a brief kick then open on a meter, never a steady ohms value).
`R=` at ambient also names any probe: ~110k = 100k/3950 class, ~11k =
10k class (adjust `kR0`/`kB` in `thermistor.h`).

When the divider reads `nan`, serial appends the raw ADC counts and
which rail they peg: HIGH rail = the NTC leg isn't conducting (open
joint / thermistor out of circuit), LOW rail = D6's drive isn't reaching
the divider or A0 isn't on the node. After three consecutive `nan`
reads the pod runs a **thermistor harness diagnostic** (repeating every
15 s while the fault lasts, so a re-flowed joint shows up live): it
distinguishes an open NTC leg from a node tied to a live rail, probes
D7–D10 in case the power wire landed on the wrong pin (**and adopts it**
for the session if found), checks whether a divider node is following
the drive on one of the peripheral analog pins (sense wire misplaced),
and finally charge-injects A0 to tell a floating sense wire (or a cap
soldered in series with A0) from an open fixed-resistor leg.

Thermistor reads are filtered twice: an 8-sample burst per read with
min and max discarded (one EMI spike cannot move the result), then a
cross-tick exponential average with a ~4 s time constant
(`THERM_EMA_ALPHA`) — physically free for intake air, which cannot
change faster than seconds. A fault resets the filter, so an unplug
still hits the wire sentinel within one tick. The smoothing cap is
still required hardware — the filter cleans up quantization and
sampling noise, the cap is what keeps ignition EMI out of the ADC's
acquisition window in the first place.

D6 is driven high only for the ~14 ms around each 1 Hz read — no idle
drain, no self-heating, nothing left energized in deep sleep. **The cap
value and `THERM_SETTLE_MS` are coupled**: the node charges through the
divider each pulse (worst-case τ = 100 kΩ × C, cold NTC), and the settle
must be ≥ ~10 τ or cold readings come out low. The define in the sketch
carries the table (10 nF → 12 ms, 100 nF → 75 ms, …) — change the cap,
change the number. The read is ratiometric (`AR_VDD4` reference = the
same rail D6 drives), so no calibration constant exists; tune
`kR0`/`kB`/`kRFixed` in `thermistor.h` if the part deviates from
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

## The -Ofast rule

The Seeed nRF52 platform compiles sketches with **`-Ofast`**, which
includes `-ffinite-math-only`: **`isnan()` is constant-folded to
`false`** and NaN comparisons are optimized on the assumption NaN can't
exist. Field-confirmed 2026-07-27 — every `isnan()`-gated branch in
device code was silently compiled out (the thermistor filter never
seeded and manufactured a permanent `nan`; the `nan` serial forensics
never printed in *any* build; the payload's NaN→`0x8000` sentinel guard
could have cast garbage onto the wire). Host builds don't use `-Ofast`,
so the test suite was blind to all of it.

Device-compiled code therefore uses `isNanF()` from `nan_bits.h` — a
bit-pattern check the optimizer cannot fold — and never compares a
possibly-NaN float before checking it. Plain `isnan()` is allowed in
host-only code.

## CI & tests

Two deliberately-basic GitHub Actions workflows (same shapes as the
DovesDataLogger repo's, minus everything release-related):

- **compile-sketch** — compiles the sketch for the Seeed XIAO nRF52840
  (the same board the datalogger uses) with the real libraries, and a
  second job packages a **flashable UF2** uploaded as a workflow
  artifact: download `DovesSensorEgg-uf2` from the run's Artifacts,
  double-tap reset on the pod to get the UF2 bootloader drive, and copy
  the `.uf2` over — no IDE or DFU utility needed. Locally the same
  build is `tools/build-uf2.sh` (see `tools/README.md`).
- **unit-tests** — host-built doctest suite over the extracted pure
  logic. `pw_adv_encode.{h,cpp}` builds the PW-ADV-2 payload, and its
  golden-byte test uses the **same fixture bytes** as the logger repo's
  `sensoregg_protocol` parser test — the two tests together pin the wire
  contract from both ends. Run locally:

  ```bash
  cmake -S tests -B tests/build
  cmake --build tests/build --parallel
  ctest --test-dir tests/build --output-on-failure
  ```
