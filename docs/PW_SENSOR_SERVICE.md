# PerchWerks Sensor Service — GATT Specification

> **Status: normative draft, service revision 1.** This is the page a
> pod firmware and a logger central are both built against. The words
> MUST / SHOULD / MAY are used in the RFC-2119 sense. Everything
> radio-stack-specific is confined to the non-normative Appendix A —
> the body of this spec is written against plain GATT and is portable
> to any stack (Bluefruit, Zephyr, whatever comes next).
>
> Design rationale lives in [ARCHITECTURE.md](ARCHITECTURE.md). Channel
> records and enums live in [PW_CHANNEL_SCHEMA.md](PW_CHANNEL_SCHEMA.md)
> and are **cited here, never redefined**. The pre-pairing beacon is
> specified in [PW_ADV_2.md](PW_ADV_2.md).

## 1. Overview

A PerchWerks pod is a BLE **Peripheral + GATT server** exposing one
PerchWerks Sensor Service. The consumer (the DovesDataLogger, a phone,
nRF Connect) is **Central + GATT client**. The service's core principle
is **self-description**: a client reads the Descriptor characteristic
once after connecting and from then on can parse every sample the pod
will ever send — there is no pod-type-specific parsing anywhere in a
client.

## 2. UUID allocation

All attributes share one vendor base UUID with a 16-bit index in the
alias slot (textual octets 2–3). The base was generated randomly once
and is frozen forever:

```
Base:    e1100000-4e72-4a75-96d7-c19c314a99ba
Pattern: e110XXXX-4e72-4a75-96d7-c19c314a99ba
```

The index space deliberately echoes the beacon magic `50 57` (`PW`):

| Index | Attribute | Properties |
|---|---|---|
| `0x5057` | PerchWerks Sensor Service | — |
| `0x5058` | Descriptor characteristic | Read |
| `0x5059` | Sample characteristic | Notify |
| `0x505A` | Clock characteristic | Read |
| `0x505B`–`0x50FF` | reserved (future: control/config, time-set, DFU gate) | — |

### Byte-order worked example

GATT APIs that take a 128-bit UUID as a byte array almost always want
it **little-endian** (least significant byte first) — the classic
screwup is passing the textual order. The service UUID
`e1105057-4e72-4a75-96d7-c19c314a99ba` as a little-endian array:

```c
// e1105057-4e72-4a75-96d7-c19c314a99ba, little-endian
const uint8_t kPwServiceUuid128[16] = {
    0xba, 0x99, 0x4a, 0x31, 0x9c, 0xc1, 0xd7, 0x96,
    0x75, 0x4a, 0x72, 0x4e, 0x57, 0x50, 0x10, 0xe1};
```

To derive any other attribute's array, replace bytes `[12]` and `[13]`
with the index low byte and high byte respectively (Descriptor
`0x5058` → `[12]=0x58, [13]=0x50`).

## 3. Roles and lifecycle

```
 unclaimed / disconnected            connected
┌─────────────────────────┐   ┌──────────────────────────────┐
│ advertise PW-ADV-2      │   │ client: discover service     │
│ (connectable, live      ├──►│ read Descriptor (once)       │
│  headline values in     │   │ read Clock (fit anchor)      │
│  the scan list)         │◄──┤ subscribe Sample             │
└─────────────────────────┘   │ pod: stream batched notifies │
        ▲    resume on        └──────────────────────────────┘
        └──  disconnect
```

- While disconnected, the pod MUST advertise the PW-ADV-2 beacon
  ([PW_ADV_2.md](PW_ADV_2.md)) as **connectable** advertising, so it
  shows live headline values in any scan list *before* pairing.
- Advertising stops while connected (inherent to a single-link
  peripheral) — a claimed pod stops polluting the air. On disconnect
  the pod MUST resume the beacon.
- The Descriptor value is **static from boot to boot-end**: a client
  reads it once per connection (or caches it keyed by device identity
  and `fw_major.fw_minor`).
- The pod never initiates writes to the client and never needs to know
  what the client does with the data — including whether GPS-derived
  wall time exists. See §6.

## 4. Descriptor characteristic (`0x5058`, Read)

One read tells the client everything needed to parse this pod.
Layout — an 8-byte header followed by packed channel records:

| Offset | Size | Field | Semantics |
|---|---|---|---|
| 0 | 1 | `schema_version` | `u8` — channel-schema revision (currently 1) |
| 1 | 1 | `device_type` | `u8` — [PW_CHANNEL_SCHEMA.md](PW_CHANNEL_SCHEMA.md) §4 |
| 2 | 1 | `fw_major` | `u8` |
| 3 | 1 | `fw_minor` | `u8` |
| 4 | 1 | `channel_count` | `u8`, 1–21 |
| 5 | 1 | `record_len` | `u8` — bytes per channel record (24 for schema_version 1) |
| 6 | 2 | reserved | producers write 0, consumers ignore |
| 8 | `channel_count × record_len` | records | channel descriptor records per [PW_CHANNEL_SCHEMA.md](PW_CHANNEL_SCHEMA.md) §5 |

- The value is a **single attribute**. Total size = 8 +
  `channel_count × record_len` ≤ 512 bytes (the ATT maximum attribute
  length), which is where the 21-channel cap comes from
  (8 + 21×24 = 512).
- Clients longer than one MTU away read it with the standard ATT
  **Read Blob** (long read) procedure — every stack's server serves
  this automatically. There is **no custom fragmentation protocol,
  ever**.
- The pod MUST populate the value once during initialization and MUST
  NOT change it while running (a config change that alters the channel
  table takes effect on reboot).
- Sizes of the reference devices: EGT pod 8 + 4×24 = **104 bytes**;
  6-channel universal hub 8 + 6×24 = **152 bytes**. Both are one
  long-read.
- Consumers MUST stride by `record_len` (not by 24) and MUST tolerate
  `record_len` > 24 — that is the schema's forward-compatibility
  mechanism.

## 5. Sample characteristic (`0x5059`, Notify)

Samples are shipped as **per-channel batch frames**: one notification
carries one channel's contiguous batch. Channels genuinely run at
different rates (a hub's throttle at 50 Hz next to a temperature at
4 Hz), and homogeneous per-channel streams are also what the logger's
per-pod clock fit wants to consume. Interleaved multi-channel frames
would force a rate matrix into every frame for no benefit.

Frame layout — 10-byte header + samples, all little-endian:

| Offset | Size | Field | Semantics |
|---|---|---|---|
| 0 | 1 | `channel_id` | matches a Descriptor record |
| 1 | 1 | `boot_id` | epoch counter, same value as Clock §6 |
| 2 | 1 | `seq` | per-channel frame counter, wraps at 255 |
| 3 | 4 | `base_timestamp` | `u32` — pod-local milliseconds at acquisition of sample 0 |
| 7 | 2 | `interval` | `u16` — ms between samples in this frame |
| 9 | 1 | `n` | sample count, ≥ 1 |
| 10 | `n × 2` | samples | `s16` LE; `0x8000` = invalid sentinel, verbatim per the schema |

Semantics:

- Sample *i* was acquired at pod time `base_timestamp + i × interval`.
  Timestamps are **acquisition** times — delivery jitter never touches
  data quality. Pods SHOULD acquire on a hardware-timer cadence so
  intra-batch spacing is crystal-accurate; only the base carries
  uncertainty.
- `base_timestamp` is **pod-local** milliseconds (wraps at ~49.7 days);
  mapping to real time is entirely the client's job (§6).
- `interval` SHOULD equal the channel's advertised `sample_period_ms`;
  the frame field is authoritative for the samples it carries.
- `seq` increments once per frame the pod emits for that channel. A gap
  observed by the client means the pod dropped frames (e.g. its notify
  queue was full during a radio-starved stretch) — the client logs a
  discontinuity rather than interpolating across it.
- **A frame MUST fit in a single notification**: payload ≤ ATT_MTU − 3.
  The pod sizes `n` to the live MTU. Worked examples:
  - ATT_MTU 23 (default) → 20-byte payload → `n ≤ 5`
  - ATT_MTU 247 → 244-byte payload → `n ≤ 117`
- Delivery cadence: the pod SHOULD flush each channel's pending batch
  at least once per connection interval when samples are pending.
  Multiple channels' frames in one connection event is normal and
  expected (that is what the link's event-length knob is for). Slow
  channels (e.g. battery at 1/30 Hz) simply send `n = 1` frames at
  their own rate.
- **Open:** `interval` in whole ms caps a channel at 1 kHz. Sub-ms
  channels (e.g. a future suspension pot at 2 kHz) need either a
  time-unit flag or a µs variant — deferred to a future service
  revision; tracked in the [ROADMAP.md](ROADMAP.md) parking lot.

## 6. Clock characteristic (`0x505A`, Read)

The pod is deliberately **time-dumb**: it stamps in local milliseconds
only and never consumes wall time. The client (the logger — which is
GPS-disciplined) owns a per-pod linear fit `pod_time → client_time`,
refined opportunistically. This was a considered decision (see
[ARCHITECTURE.md](ARCHITECTURE.md) §7): the pod-side alternative
(writing the logger's clock into the pod) puts clock state in every pod
and turns every pod reboot into a silent hazard. Consequently there is
**no writable time characteristic in revision 1** — the reserved index
range covers a future TimeSet if one is ever justified, which is
cleaner than a defined-but-ignorable write (a conformance trap).

Layout (6 bytes):

| Offset | Size | Field | Semantics |
|---|---|---|---|
| 0 | 1 | `boot_id` | epoch counter, see below |
| 1 | 1 | reserved | producers write 0 |
| 2 | 4 | `millis_now` | `u32` LE — pod-local ms, sampled when the read executes |

- A Clock read is a bounded request/response round trip, so
  `(client_receive_time, millis_now)` is the tightest point-pair the
  client's fit can get — tighter than notify-arrival timestamps. The
  client SHOULD read Clock on connect and MAY re-read every few minutes
  to slew its fit (a ±20 ppm pod crystal drifts ≤ ~72 ms/hour).
- **`boot_id` makes reboots explicit.** It changes on every pod boot
  (increment mod 256, or randomized on cold start — *inequality*, not
  ordering, is the signal) and is echoed in every Sample frame header.
  A changed `boot_id` means the pod's millis clock restarted: the
  client MUST discard its fit, re-anchor (re-read Clock), and log the
  discontinuity. The pre-`boot_id` heuristic — a batch base earlier
  than its predecessor by more than one interval — remains as a
  cross-check, but `boot_id` is unambiguous even across a u32 wrap or
  a long disconnection.

## 7. Standard service mirrors

Where standard GATT fits, the pod exposes it too, so nRF Connect and
other generic apps read a pod bare, with no PerchWerks software:

- **Device Information Service (`0x180A`) — MUST.** Manufacturer Name,
  Model Number (the device-type name), Firmware Revision (matching
  `fw_major.fw_minor`).
- **Battery Service (`0x180F`, Battery Level `0x2A19`) — conditional.**
  `0x2A19` is `u8` 0–100 with **no "unknown" encoding**, and GATT
  services cannot be removed once registered. The pod therefore
  instantiates BAS **only when a battery pack is detected at boot**
  (the existing below-2.5 V = no-pack rule) — the same honesty stance
  as the beacon's `0xFF` battery byte. A USB-powered bench pod simply
  has no Battery Service.
- **Environmental Sensing (`0x181A`, Temperature `0x2A6E`) — SHOULD,
  with a hard restriction.** `0x2A6E` is `s16` in **centi-°C**, so its
  ceiling is **327.67 °C — an EGT at 650 °C overflows it**. There is no
  standard high-temperature characteristic. Therefore: ESS Temperature
  MUST mirror only channels that physically fit the range (intake air,
  cold junction) and MUST NOT mirror EGT or other high-range channels.
  Note the conversion: PerchWerks temperatures are deci-°C raw — the
  ESS mirror is the engineering value re-encoded ×100 (i.e. raw ×10),
  and the schema's `0x8000` sentinel maps to ESS's "value is not known"
  (`0x8000` there too, conveniently).

Mirrors are conveniences, not the contract: the PerchWerks service is
the normative data path, and a client MUST NOT need the mirrors.

## 8. Security

Threat model: the paddock. The risk is **misattributed data** — someone
else's pod landing in your log (today any egg broadcasting
`FF FF 50 57` lands in everyone's log) — not interception of
temperature readings.

- The pod MUST support **LE Secure Connections pairing, Just Works**
  association, and **bonding** (persisted keys). MITM protection is not
  required — there is no display/keyboard on a potted pod, and the
  threat model doesn't demand it.
- **New bonds MUST be gated on the pairing window**: the existing
  long-press 30 s window (beacon flags bit 0) is promoted from
  informational to load-bearing. Outside the window the pod MUST reject
  pairing requests; **bonded reconnects MUST be accepted at any time**.
  This is the physical "that's *my* egg" guarantee: claiming a pod
  requires holding it.
- Bonds MUST survive power cycles. The pod SHOULD support at least 4
  bonds (primary logger, phone, spares).
- Accept-list/whitelist enforcement is the **client's** job (the logger
  filters to bonded pods); an unclaimed pod keeps open connectable
  advertising so it stays discoverable.
- The PerchWerks characteristics SHOULD require an encrypted link when
  the peer is bonded; the standard mirrors (§7) stay open-read.
  **Open:** whether a future revision locks the PerchWerks service to
  encrypted-only. Under Just Works, mandatory encryption buys little
  real access control, and multi-logger etiquette at a shared track is
  not yet understood — tracked in the [ROADMAP.md](ROADMAP.md) parking
  lot.

## 9. Connection parameters

- The pod MUST be fully functional at ATT_MTU 23 (that is where the
  `n ≤ 5` frame sizing case comes from) and SHOULD support MTU
  exchange up to 247. MTU only affects notify batch size, never
  correctness.
- Suggested connection intervals (client-chosen, per link): slow pods
  (temperatures) 100–200 ms; fast hubs (20 ms channels) 30–50 ms with
  2–3 frames per event. Fat-fast and lazy-slow pods coexist —
  intervals are per-connection.
- The pod SHOULD declare Peripheral Preferred Connection Parameters
  matching its fastest channel, and MUST tolerate whatever the client
  actually sets.

## 10. Third-party conformance checklist

A conformant pod:

1. Advertises a valid PW-ADV-2 beacon while disconnected, connectable,
   and resumes it on disconnect.
2. Exposes the PerchWerks Sensor Service under the exact UUIDs of §2.
3. Serves a Descriptor value that parses per §4 against
   [PW_CHANNEL_SCHEMA.md](PW_CHANNEL_SCHEMA.md), with truthful
   `sample_period_ms`, `scale`, `offset` per channel.
4. Streams Sample frames per §5: acquisition-time bases, sentinel for
   invalid readings, frames sized to the live MTU, `seq` honest about
   drops.
5. Serves Clock per §6 with a `boot_id` that changes every boot and
   matches the one in Sample frames.
6. Exposes DIS; exposes BAS only with a real pack; never mirrors an
   out-of-range channel to ESS.
7. Enforces §8: LESC + bonding, window-gated new bonds, any-time bonded
   reconnects.
8. Works at ATT_MTU 23.

Verify 1–8 with nRF Connect alone — no PerchWerks hardware needed.

## Appendix A — Bluefruit implementation notes (non-normative)

Target: Adafruit Bluefruit nRF52 API as shipped in the Seeed XIAO
nRF52840 board package (SoftDevice S140). Peripheral-side only — the
pod needs `Bluefruit.begin(1, 0)` as today.

- **UUIDs:** construct `BLEUuid` from the 16-byte little-endian arrays
  of §2. One shared base costs exactly one SoftDevice vendor-UUID slot
  regardless of characteristic count.
- **Descriptor:** `BLECharacteristic` with `CHR_PROPS_READ`,
  `setMaxLen(512)` (variable length), value written once at init with
  `write(buf, len)`. Long reads are served by the SoftDevice
  automatically. A 512-byte attribute may require growing the attribute
  table: `Bluefruit.configAttrTableSize(...)` **before** `begin()` —
  verify at Phase 2 bring-up ([ROADMAP.md](ROADMAP.md)).
- **Sample:** `CHR_PROPS_NOTIFY`, `notify(buf, len)` returns false when
  the HVN queue is full — that is the moment `seq` records a drop.
  Queue depth and MTU are `Bluefruit.configPrphConn(...)` knobs, set
  before `begin()`; sizing them is a Phase 3 task.
- **Clock:** easiest as a read callback (`setReadAuthorizeCallback` or
  a just-in-time `write()` refresh) so `millis_now` is sampled at read
  time.
- **Advertising:** removing
  `setType(BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED)`
  makes Bluefruit's default (connectable undirected) apply; the 31-byte
  AD budget is identical, so the beacon payload is untouched. The
  existing 250 ms stop/clearData/rebuild/start refresh cycle MUST be
  gated on `!Bluefruit.connected()` — with `periphCount = 1`,
  restarting advertising during a live connection fails (and is
  pointless). One new conditional, not a redesign.
- **Security:** recent Adafruit cores expose LESC + bonding via
  `BLESecurity` with bond storage on InternalFS, and per-characteristic
  permissions via `setPermission(SECMODE_*)`. **Verify the Seeed fork's
  vintage actually carries LESC before Phase 4** — if it only offers
  legacy pairing, bonding still works and the spec's window-gating
  still holds; note the deviation.
- **The `-Ofast` rule applies to all of this:** any new device code
  touching possibly-NaN floats uses `isNanF()` from `nan_bits.h`, never
  `isnan()` — see README §"The -Ofast rule".

## Appendix B — open items in this spec

Collected for visibility; each is tagged `**Open:**` at its section and
tracked in the [ROADMAP.md](ROADMAP.md) parking lot:

- §5: sub-ms sample intervals (>1 kHz channels).
- §8: encrypted-only PerchWerks service in a future revision.
