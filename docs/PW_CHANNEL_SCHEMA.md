# PerchWerks Channel Schema

> **Status: normative, schema_version 1.** This document is
> transport-neutral by design: it never mentions any radio, bus, or
> protocol. The BLE GATT service ([PW_SENSOR_SERVICE.md](PW_SENSOR_SERVICE.md))
> and the planned wired CAN backpack both carry these records verbatim —
> the schema is defined once, here, and cited (never redefined) by every
> transport spec.

## 1. Purpose

A PerchWerks sensor device is **self-describing**: it presents a table
of channel descriptor records, and a consumer (the logger, a phone app,
a third-party tool) parses *every* device with one code path. There is
no per-device-type parsing anywhere in a consumer.

This is the property that makes adding a sensor cheap: adding a throttle
channel to a hub is **one 24-byte record in the device's configuration**
— the `quantity` byte tells the consumer the data type, the
`scale`/`offset` pair tells it the engineering conversion, and the
consumer's code does not change.

## 2. Value model

- Samples are **`s16` little-endian** raw values.
- Engineering value: `real = raw × scale + offset`, with `scale` and
  `offset` from the channel's descriptor record (both `f32`).
- **`0x8000` (INT16_MIN) is the invalid sentinel** — "no valid reading".
  It is never a data value. Producers MUST emit the sentinel instead of
  casting NaN or out-of-range floats to `s16` (that cast is undefined
  behavior and produces plausible-looking garbage). Consumers MUST check
  for the sentinel *before* applying `scale`/`offset`.
- Every quantity has **one canonical unit** (table below). There is no
  unit field: a temperature is always °C on the wire, a pressure always
  kPa. Display conversion (°F, psi, mph) is strictly a consumer/UI
  concern. This deletes an entire permutation axis for third-party
  implementers.

## 3. Quantity enum (`u8`)

| Value | Quantity | Canonical unit | Typical raw encoding |
|---|---|---|---|
| `0x00` | reserved (invalid) | — | — |
| `0x01` | temperature | °C | deci-°C: scale 0.1 |
| `0x02` | pressure | kPa | scale 1 (or 0.1 for fine ranges) |
| `0x03` | voltage | V | milli-V: scale 0.001 |
| `0x04` | rotational speed | rpm | scale 1 |
| `0x05` | angle | degrees | deci-deg: scale 0.1 (steering: ±3276.7°) |
| `0x06` | linear position | mm | deci-mm: scale 0.1 |
| `0x07` | acceleration | m/s² | centi-m/s²: scale 0.01 |
| `0x08` | ratio | % | throttle/brake position, battery: scale 0.1 or 1 |
| `0x09`–`0xEF` | reserved | — | future PerchWerks assignment |
| `0xF0`–`0xFE` | private use | — | vendor/experimental; not portable |
| `0xFF` | reserved | — | — |

New quantities are assigned here, in this file, by pull request — the
table is the registry. Third parties needing a quantity before it is
assigned use the private-use range and propose the assignment.

## 4. Device-type enum (`u8`)

| Value | Device |
|---|---|
| `0x00` | reserved (invalid) |
| `0x01` | EGT pod ("egg"): EGT + cold junction + intake air + battery |
| `0x02` | universal hub: up to 6 general-purpose channels |
| `0x03`–`0xEF` | reserved for future PerchWerks assignment |
| `0xF0`–`0xFE` | private use (vendor/experimental) |
| `0xFF` | reserved |

The device type is informational (naming, icons, defaults). Consumers
MUST NOT branch parsing on it — the channel records alone define the
data. An unknown device type with valid channel records is fully usable.

## 5. Channel descriptor record — 24 bytes, packed, little-endian

| Offset | Size | Field | Semantics |
|---|---|---|---|
| 0 | 1 | `channel_id` | `u8`, unique within the device, stable across boots |
| 1 | 1 | `quantity` | `u8`, from §3 |
| 2 | 2 | `sample_period_ms` | `u16` LE, nominal period between samples (ms). 0 = aperiodic/on-change |
| 4 | 4 | `scale` | `f32` LE (IEEE-754 binary32) |
| 8 | 4 | `offset` | `f32` LE |
| 12 | 8 | `name` | `char[8]` ASCII, NUL-padded (not necessarily NUL-terminated at 8) |
| 20 | 1 | `flags` | `u8`, reserved — producers write 0, consumers ignore |
| 21 | 3 | reserved | producers write 0, consumers ignore |

Field order is chosen so every multi-byte field sits at its natural
alignment within the record; records themselves are packed back-to-back
with no inter-record padding. A record table is always
`channel_count × record_len` bytes, where `record_len` is declared by
the transport's container (24 in schema_version 1). Consumers MUST use
the declared `record_len` as the stride — a future schema_version may
append fields, and skipping unknown trailing bytes is the
forward-compatibility mechanism (this is why there is no TLV: fixed
records plus a declared stride give the same evolvability at a fraction
of the parser cost).

### Naming rules

`name` is a short human label for scan lists and log-channel headers:
uppercase ASCII, 1–8 chars, NUL-padded. Suggested conventions:
`EGT`, `CJ` (cold junction), `IAT` (intake air), `BATT`, `THR`
(throttle), `BRK` (brake), `STR` (steering), `SPD` (axle speed),
numbered variants `TEMP1`/`TEMP2` where a device carries several of one
quantity. Names are labels, not identity — `channel_id` + `quantity`
are what a consumer keys on.

## 6. Reference channel tables

### 6.1 Device type `0x01` — EGT pod

| `channel_id` | `quantity` | `sample_period_ms` | `scale` | `offset` | `name` |
|---|---|---|---|---|---|
| 0 | `0x01` temperature | 250 | 0.1 | 0 | `EGT` |
| 1 | `0x01` temperature | 250 | 0.1 | 0 | `CJ` |
| 2 | `0x01` temperature | 1000 | 0.1 | 0 | `IAT` |
| 3 | `0x08` ratio | 30000 | 1.0 | 0 | `BATT` |

Deci-°C with scale 0.1 is byte-compatible with the existing PW-ADV-2
temperature fields — the encoding idiom (and the `0x8000` sentinel)
carries over unchanged.

### 6.2 Device type `0x02` — universal hub (worked example)

The 6-channel configuration matching a "2 temps + 4-sensor hub"
(throttle, brake, steering, axle speed) setup:

| `channel_id` | `quantity` | `sample_period_ms` | `scale` | `offset` | `name` |
|---|---|---|---|---|---|
| 0 | `0x01` temperature | 250 | 0.1 | 0 | `TEMP1` |
| 1 | `0x01` temperature | 250 | 0.1 | 0 | `TEMP2` |
| 2 | `0x08` ratio | 20 | 0.1 | 0 | `THR` |
| 3 | `0x02` pressure | 20 | 1.0 | 0 | `BRK` |
| 4 | `0x05` angle | 20 | 0.1 | 0 | `STR` |
| 5 | `0x04` rotational speed | 50 | 1.0 | 0 | `SPD` |

Notes: a position-type brake sensor instead of a pressure sensor is the
*same record* with `quantity = 0x08` — nothing else changes. Wheel
speed in km/h is a consumer-side conversion from rpm using the
consumer's wheel-circumference setting; the wire stays rpm.

**The point of this example:** turning a 5-channel hub into this
6-channel one — or swapping what's plugged into a hub input — touches
only the device's own record table. Every consumer picks the change up
on the next read of the table, with zero code changes.

## 7. Versioning

`schema_version` (a `u8` carried by each transport's container next to
`record_len`) identifies this document's revision. Version 1 is this
document. Rules:

- A new version MAY append record fields (raising `record_len`) and MAY
  assign reserved enum values. It MUST NOT reorder, resize, or
  repurpose existing fields — offsets 0–23 are frozen.
- Consumers MUST accept records with a larger `record_len` than they
  know (skip the tail) and MUST reject a `record_len` smaller than 24.
- Enum additions are backwards-compatible by construction: an unknown
  `quantity` renders as raw values with the name as the label.
