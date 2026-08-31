# PW-ADV-2 — Broadcast Beacon Specification

> **Status: normative; shipped.** This specifies the advertising payload
> the egg broadcasts today, promoted out of README prose and header
> comments into its single home. The byte table below is canonical —
> `pw_adv_encode.{h,cpp}` is the canonical *implementation*, and the
> golden-byte tests (this repo and the DovesDataLogger repo) are the
> conformance anchor. If the layout ever changes, this file and both
> golden tests change together, deliberately.
>
> **Role going forward:** with the move to the connectable
> [PerchWerks Sensor Service](PW_SENSOR_SERVICE.md), PW-ADV-2 is not
> retired — it becomes the **pre-pairing beacon**: the payload a pod
> advertises while disconnected, so it shows type + live headline
> values in a scan list before it is ever claimed. The bytes stay
> identical; only the advertising *type* changes from nonconnectable to
> connectable. Advertising stops while connected and resumes on
> disconnect.

## 1. Logger compatibility

The DovesDataLogger's v2 round has **landed** (BETA channel): its
`sensoregg_protocol` parser accepts versions `0x01` and `0x02` and
captures the full 16-byte payload (the old RX path truncated at 14
bytes and dropped v2). Both repos pin the same golden fixture bytes —
see §6. nRF Connect remains the neutral bench check (16-byte
manufacturer data starting `FF FF 50 57 02`).

## 2. Radio parameters

- Advertised name: **`PWEGT`** (Complete Local Name).
- Advertising type today: nonconnectable, nonscannable, undirected.
  (Becomes connectable undirected in the GATT phases — same 31-byte
  budget, byte-identical AD payload.)
- **Advertising interval: 111.875 ms — deliberately not 100 ms**
  (179 × 0.625 ms units, `ADV_INTERVAL_UNITS` in the sketch). The
  logger scans on a 100 ms cycle with a ~60 ms window; equal periods
  phase-lock, and the egg can park in the scanner's deaf zone for
  seconds. An off-100 interval sweeps the phase instead. No fast/slow
  fallback (`setFastTimeout(0)`).
- **Payload refresh: every 250 ms** — the payload (and sequence
  counter) is rebuilt on a 250 ms tick, independent of the airtime
  interval above. 250 ms is still far inside a K-type probe's 0.5–3 s
  thermal time constant.

## 3. AD structure and budget

Legacy advertising payload, 31 bytes:

| AD structure | Bytes |
|---|---|
| Flags | 3 |
| Manufacturer Specific Data (16-byte payload) | 18 |
| Complete Local Name `PWEGT` | 7 |
| **Total** | **28 of 31** |

The company ID is *inside* the manufacturer-data payload array —
Bluefruit passes the buffer through raw on both sides, so bytes 0–1
of the payload ARE the company ID on the air.

## 4. Payload — 16 bytes, multi-byte fields little-endian

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

Field semantics:

- **`0x8000` sentinel:** NaN / out-of-range readings are sent as
  `0x8000` (INT16_MIN) — never cast to int16 (undefined behavior,
  plausible-looking garbage). An open/shorted thermistor divider is
  signalled by the sentinel alone — no flag bit. This is the same
  sentinel rule as the
  [channel schema](PW_CHANNEL_SCHEMA.md)'s value model.
- **Flags bit0 (pairing window):** set while the long-press 30 s
  pairing window is open. Historically informational; under the GATT
  service it becomes load-bearing (new bonds are only accepted while
  it is set — [PW_SENSOR_SERVICE.md](PW_SENSOR_SERVICE.md) §8).
- **Flags bit1 (TC fault):** derived from MCP9600 STATUS bit 4
  (input-range / open-probe).
- **Battery `0xFF`:** below 2.5 V the pod reports unknown — a USB-only
  bench with no pack must not broadcast a lying 0%.
- **Sequence counter:** wraps; refreshes with the 250 ms payload tick.
  A receiver treats readings older than 1 s as gone (logs `nan`), so a
  dropout is never a held flat line; a counter that restarts near zero
  is the reboot tell.

## 5. Versioning rules

v2 appends to v1: bytes 0–13 keep their exact v1 offsets, so a v1
field decoder carries over unchanged once its version gate accepts
`0x02`. Any future version MUST follow the same rule — append, never
reorder — or take a new protocol version byte and a new spec.

Receivers MUST gate on the version byte and MUST NOT parse fields past
the length they know for that version.

## 6. Conformance and test anchors

- Encoder: `pw_adv_encode.{h,cpp}` (pure logic, host-tested; constants
  `kPayloadLen = 16`, `kProtoVer = 0x02`, `kFlagPairing = 0x01`,
  `kFlagTcFault = 0x02`, `kBatteryUnknown = 0xFF`,
  `kStatusInputRangeMask = 0x10`).
- Golden frame (`tests/pw_adv_encode_test.cpp`) — EGT 650.0 °C,
  CJ 33.3 °C, battery 87 %, seq 298, thermistor 23.4 °C:

  ```
  FF FF 50 57 02 00 64 19 4D 01 00 57 2A 01 EA 00
  ```

- The DovesDataLogger repo's `sensoregg_protocol` parser fixture adopts
  these same bytes when its v2 round lands — the two golden tests
  together pin the wire contract from both ends.

## 7. Relationship to the GATT service

In the [PerchWerks Sensor Service](PW_SENSOR_SERVICE.md) lifecycle this
beacon is the *discovery* payload: type + latest headline values in
the scan list, pairing-window bit for the claim flow, then the
connection carries everything else (full channel set, batching,
timestamps, security). Multi-channel devices like the universal hub do
**not** get their channels broadcast — headline values only; the
connection is the data path.

**Open:** whether a generic successor beacon (PW-ADV-3 — headline
values keyed by the channel schema's quantity enum instead of this
EGT-specific layout) ever replaces per-pod-type beacon layouts.
Budget is tight (six channels + type tags ≈ 24 bytes of manufacturer
data, which crowds out the name), and with the connection as the data
path the need is cosmetic. Parked in the
[ROADMAP.md](ROADMAP.md) parking lot.
