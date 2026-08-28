# PerchWerks Wireless Pod Architecture

> **Status: rationale.** Nothing in this document is normative — it
> explains *why* the system is shaped the way it is. The contracts live
> in [PW_SENSOR_SERVICE.md](PW_SENSOR_SERVICE.md),
> [PW_CHANNEL_SCHEMA.md](PW_CHANNEL_SCHEMA.md) and
> [PW_ADV_2.md](PW_ADV_2.md); the migration plan lives in
> [ROADMAP.md](ROADMAP.md). Distilled from the 2026-08-27 design
> session. The Insta360 GPS link (the logger's "SmartyCam replacement")
> is untouched by everything here.

## 1. The BLE mental model (why the old framing was wrong)

BLE has two independent axes, and "server" on one says nothing about
the other:

- **Link-layer roles:** Advertiser / Scanner before a connection;
  **Central** (owns timing) / **Peripheral** after one.
- **GATT roles:** **Server** (hosts the attribute table) / **Client**
  (reads, writes, subscribes).

The Insta360 remote is a *peripheral* but a GATT *server* — the camera
connects to it and subscribes to its notify characteristic. The logger,
impersonating the remote, is Peripheral + GATT server toward the
camera. There is no such thing as making a pod an "Insta360 client" —
the camera neither knows nor cares what else the logger talks to.

What we actually want: the logger is *simultaneously* Central + GATT
client toward the pods. Fully supported:

- **nRF52840 / SoftDevice S140:** concurrent multirole, up to 20 links,
  any mix. Bluefruit: `Bluefruit.begin(periphCount, centralCount)` —
  e.g. `begin(2, 4)` = camera + phone on the peripheral side, four pods
  on the central side. The pod side needs only `begin(1, 0)`.
- **Zephyr + SDC (e.g. nRF54):** `CONFIG_BT_CENTRAL=y` +
  `CONFIG_BT_PERIPHERAL=y`, `CONFIG_BT_MAX_CONN=6`; bonus 2M PHY,
  extended advertising, Coded PHY — kept in mind so the spec stays
  stack-portable, but not the implementation target
  (see decision ledger §9).

**Radio budget sanity:** connection events are ~1–3 ms; at a 100 ms
interval a link uses ~2 % of radio time. Five links is trivial.
Collided events defer by priority — the loser misses one interval;
supervision timeouts are seconds. The camera link is not at risk.

**Why connections beat the current passive scan:** scanning is the
*lowest*-priority radio activity and gets starved first (e.g. during
the ~44 KB/s file pull to the phone). That starvation is what forced
the 111.875 ms phase-sweep hack in [PW_ADV_2.md](PW_ADV_2.md).
Connections have guaranteed slots; none of that pathology exists.

## 2. Verdict on the broadcast egg (PW-ADV-2)

Broadcast-only is *legitimate* for fire-and-forget, many-to-one,
loss-tolerant sensors (Bluetooth Mesh, ANT+, iBeacon all live there),
and the current beacon got real things right: the de-alias interval,
the `0x8000` sentinel discipline, the golden-byte contract pinned from
both repos. But it is the wrong default for a product other people
build:

- No ACK, no config path (rate, calibration, naming, DFU from logger).
- No time base beyond a wrapping sequence counter.
- No security — any egg at the track broadcasting `FF FF 50 57` lands
  in everyone's log.
- Hard 31-byte / ~100 ms ceiling.
- Forces a hand-rolled byte layout *per pod type* — the exact thing
  that doesn't scale to third parties.

So: broadcast is retired as the primary transport and retained as the
pre-pairing beacon ([PW_ADV_2.md](PW_ADV_2.md) §7).

## 3. The system in one page

Pods become GATT peripherals exposing one **PerchWerks Sensor Service**
([PW_SENSOR_SERVICE.md](PW_SENSOR_SERVICE.md)). Core principle:
**self-description** — the logger never needs pod-specific parsing.

- **Descriptor characteristic** (read once on connect): device type,
  firmware version, and the channel table — per channel: name,
  quantity (the data-type flag), scale/offset, rate — packed per the
  transport-neutral [channel schema](PW_CHANNEL_SCHEMA.md).
- **Sample characteristic** (notify): per-channel batched sample
  frames, acquisition-timestamped in pod-local time.
- **Clock characteristic** (read): pod-local clock + boot epoch, the
  anchor for the logger's clock fit.
- A third-party EGT pod, tire-temp array, or brake-pressure pod all
  look identical to the logger. Adding a sensor to a hub is one channel
  record — zero logger changes. **The schema is the universal
  protocol; the transports just carry it.**
- The **same channel schema serves the planned CAN backpack** — define
  once, carry over both transports. Wired CAN for pods on the frame;
  BLE for anything remote, hot, or spinning.
- Standard GATT mirrors (DIS, Battery, Environmental Sensing where the
  range fits) let nRF Connect read a pod with no PerchWerks software.

## 4. Discovery / pairing UX

Model: every HR strap and power meter ever made.

1. Pod advertises a short payload — type + latest headline value
   ([PW_ADV_2.md](PW_ADV_2.md)) — so it shows live in the logger's
   scan list before pairing.
2. User taps to add; logger bonds (LE Secure Connections) — and the
   pod only accepts a *new* bond while its physical pairing window is
   open (long-press). "That's *my* egg", physically enforced.
3. Pod auto-connects on power-up thereafter (logger filters to bonded
   pods).
4. Advertising stops on connect — a claimed pod stops polluting the
   air — and resumes on disconnect.

## 5. Data delivery: rate vs. interval (the 5 Hz non-problem)

Connection interval caps **delivery rate**, not **sample rate**. Each
connection event can carry multiple notifies, so a pod sampling at
50 Hz batches samples and ships every interval. Samples are timestamped
at acquisition — delivery jitter never touches data quality. The only
cost is latency (freshest sample ≤ one interval old): irrelevant for
logging, imperceptible on a dash.

- EGT reality check: K-type thermal time constant is 0.5–3 s; the
  broadcast already only refreshed at 250 ms. A 5 Hz link is *faster
  than the sensor physics*.
- Fast pods (suspension pot, brake pressure, the hub's 50 Hz channels):
  drop *that one link* to 30–50 ms and batch 2–3 frames per event.
  Intervals are per-connection; fat-fast and lazy-slow pods coexist.
- Batch framing (`[base, interval, n, samples...]`, sample *i* at
  `base + i×interval`) assumes hardware-timer acquisition — same
  discipline as the logger's RPM capture — so intra-batch spacing is
  crystal-accurate and only the base carries uncertainty. Specified in
  [PW_SENSOR_SERVICE.md](PW_SENSOR_SERVICE.md) §5.

## 6. Time model

No NTP-grade machinery. **Decided: the pod stays dumb** — it stamps in
local millis and reports its clock on request; the logger owns a
per-pod linear fit `pod_time → logger_time` (two constants), slewed as
drift is observed. Rationale over the pod-side alternative (logger
writes its clock into the pod): no clock state in pods, no re-sync
choreography, and a pod reboot can't silently corrupt the timebase.

- **Drift budget:** 32.768 kHz crystal at ±20 ppm ≈ 72 ms/hour worst
  case. Fine forever for temp pods; an opportunistic Clock re-read
  every few minutes erases it anyway.
- **Master clock for free:** the logger is GPS-disciplined at 10–25 Hz,
  so pod samples align with UTC, with each other, and with the Insta360
  video timeline — without any pod knowing GPS exists.
- **Epoch detection (must-have):** pod reboot (watchdog / brownout on a
  curb strike) resets millis. Made explicit as `boot_id` in every
  frame and Clock read ([PW_SENSOR_SERVICE.md](PW_SENSOR_SERVICE.md)
  §6); the backwards-base heuristic stays as a cross-check. Prevents
  silently interleaved garbage.

## 7. Scheduler hygiene / practical constraints (logger side)

- Don't put the camera link and a burst of pod links on the same
  connection interval with fat event lengths — the SoftDevice ends up
  perpetually arbitrating. Stagger: camera at whatever it dictates,
  pods at ~150 ms (or their per-pod rates), and it settles into a
  clean cadence.
- Keep central event lengths skinny (`configCentralConn`, a few slots)
  so pod links stay cheap.
- Central connections cost SoftDevice RAM (~1–2 KB each) — fine on the
  840, but budget it.

## 8. Decision ledger

| Decision | Status | Specified in |
|---|---|---|
| Primary transport: connected GATT; broadcast demoted to pre-pairing beacon | **Decided** | [PW_SENSOR_SERVICE.md](PW_SENSOR_SERVICE.md), [PW_ADV_2.md](PW_ADV_2.md) §7 |
| Implementation platform: Arduino/Bluefruit on XIAO nRF52840 (spec wording stack-portable) | **Decided** | [ROADMAP.md](ROADMAP.md) |
| Time sync: logger-side linear fit; pod time-dumb; no writable time characteristic in rev 1 | **Decided** | [PW_SENSOR_SERVICE.md](PW_SENSOR_SERVICE.md) §6 |
| Reboot/epoch detection: explicit `boot_id` + backwards-base cross-check | **Decided** | [PW_SENSOR_SERVICE.md](PW_SENSOR_SERVICE.md) §5–6 |
| PW-ADV-2 survives byte-identical as the pre-pairing beacon | **Decided** | [PW_ADV_2.md](PW_ADV_2.md) |
| Channel schema: transport-neutral, fixed 24-byte records, quantity enum with canonical units (no unit byte) | **Decided** | [PW_CHANNEL_SCHEMA.md](PW_CHANNEL_SCHEMA.md) |
| Descriptor packing: fixed-size records + declared stride, no TLV, single ≤512 B attribute, standard long read | **Decided** | [PW_SENSOR_SERVICE.md](PW_SENSOR_SERVICE.md) §4 |
| Sample delivery: per-channel batch frames, MTU-sized | **Decided** | [PW_SENSOR_SERVICE.md](PW_SENSOR_SERVICE.md) §5 |
| Security: LESC Just Works + bonding; new bonds gated on physical pairing window; whitelist logger-side | **Decided** | [PW_SENSOR_SERVICE.md](PW_SENSOR_SERVICE.md) §8 |
| ESS mirrors range-safe channels only — never EGT (0x2A6E caps at 327.67 °C) | **Decided** | [PW_SENSOR_SERVICE.md](PW_SENSOR_SERVICE.md) §7 |
| Universal hub (device type 0x02): 6 channels over the connection; beacon stays headline-only | **Decided** | [PW_CHANNEL_SCHEMA.md](PW_CHANNEL_SCHEMA.md) §6.2 |
| DFU-from-logger (Bluefruit `BLEDfu` exists; must be bond-gated; security review unfinished) | **Open** | [ROADMAP.md](ROADMAP.md) parking lot |
| PW-ADV-3 generic beacon | **Open** | [ROADMAP.md](ROADMAP.md) parking lot |
| Sub-ms sample intervals (>1 kHz channels) | **Open** | [ROADMAP.md](ROADMAP.md) parking lot |
| Encrypted-only PerchWerks service in a future revision | **Open** | [ROADMAP.md](ROADMAP.md) parking lot |
| CAN backpack transport spec (schema is ready for it) | **Open** | [ROADMAP.md](ROADMAP.md) parking lot |

## 9. Related documents

- [PW_SENSOR_SERVICE.md](PW_SENSOR_SERVICE.md) — the GATT contract
- [PW_CHANNEL_SCHEMA.md](PW_CHANNEL_SCHEMA.md) — the universal channel schema
- [PW_ADV_2.md](PW_ADV_2.md) — the shipped beacon / pre-pairing payload
- [ROADMAP.md](ROADMAP.md) — migration phases and parking lot
