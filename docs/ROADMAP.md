# Migration Roadmap — broadcast egg → PerchWerks GATT pod

> How the firmware gets from today's broadcast-only PW-ADV-2 beacon to
> a bonded, self-describing [PerchWerks Sensor Service](PW_SENSOR_SERVICE.md)
> pod, in shippable increments. Each phase is a branch/PR of its own,
> flashable on real hardware, and **bench-verifiable with nRF Connect
> alone** — the logger's central side doesn't exist until Phase 5.

## Standing invariants (every phase)

1. **The PW-ADV-2 beacon payload stays byte-identical.** The
   golden-byte test (`tests/pw_adv_encode_test.cpp`) is the regression
   keel; the logger's passive scan keeps working against an unclaimed
   pod through the whole migration.
2. **The `-Ofast` rule:** all new device code uses `isNanF()` from
   `nan_bits.h`, never `isnan()` (see README §"The -Ofast rule").
3. **Pure logic gets extracted and host-tested.** New wire formats go
   into a `pw_gatt_encode` module with golden-byte doctests added to
   `tests/CMakeLists.txt`, mirroring the `pw_adv_encode` pattern —
   fixture bytes checked against the tables in
   [PW_SENSOR_SERVICE.md](PW_SENSOR_SERVICE.md), and later adopted by
   the logger repo's tests to pin the contract from both ends.
4. All existing reliability layers (watchdog, safe mode, harness
   diagnostics, display deadman) survive untouched.

## Phase 0 — specs and docs (this branch) ✔

The five documents in `docs/`, README rework, merged-branch cleanup.
No firmware changes.

## Phase 1 — connectable shell + standard mirrors

Make the pod connectable without changing what it broadcasts.

- Remove the `setType(BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED)`
  line in `bleSetup()` (`DovesSensorEgg.ino`, the one line that forbids
  connections) — Bluefruit's default connectable undirected applies;
  update the sketch-header and README prose that state
  non-connectability as a design guarantee.
- Gate the 250 ms `updateAdvertising()` stop/rebuild/start cycle on
  `!Bluefruit.connected()`; add connect/disconnect callbacks
  (disconnect restarts advertising).
- Add the standard mirrors per spec §7: DIS always; BAS only when a
  pack is detected at boot; ESS Temperature for IAT/CJ only (never
  EGT).
- Start `pw_gatt_encode.{h,cpp}`: first slice is `encodeCentiC()`
  (deci→centi ×10 with range clamp and 0x8000 sentinel pass-through
  for the ESS mirror), with doctests.
- Risks to verify on hardware: advertising-restart-while-connected
  behavior, attribute-table RAM (`Bluefruit.configAttrTableSize`).

**Bench:** nRF Connect connects; DIS/BAS/ESS read correctly; on
disconnect the beacon resumes; a passive scanner sees an unchanged
PW-ADV-2 beacon whenever the pod is unclaimed.

## Phase 2 — PerchWerks service, read path

- Register service `0x5057` with Descriptor `0x5058` and Clock `0x505A`
  per spec §2/§4/§6.
- `pw_gatt_encode::buildDescriptor()` and `buildClock()` as pure
  functions with golden-byte doctests (the EGT pod's 104-byte
  descriptor from [PW_CHANNEL_SCHEMA.md](PW_CHANNEL_SCHEMA.md) §6.1 is
  the fixture).
- `boot_id`: one more byte in the existing magic-tagged `.noinit`
  boot scratch (the boot-loop-breaker block), incremented per boot;
  cold-start randomization is fine — inequality is the signal.

**Bench:** nRF Connect long-reads the Descriptor, bytes hand-checked
against the spec table; two Clock reads a few seconds apart show
`millis_now` advancing and a stable `boot_id`; power-cycle changes
`boot_id`.

## Phase 3 — Sample notify

- Per-channel batch buffers on the existing tick cadences (EGT 250 ms,
  IAT 1 s, BATT 30 s); hardware-timer acquisition tightening comes
  after correctness, not before.
- `pw_gatt_encode::buildSampleFrame()` with doctests: sentinel
  pass-through, `boot_id`/`seq` fields, u32 base wrap, MTU-sized `n`.
- `seq`-on-drop semantics wired to `notify()` returning false; HVN
  queue depth + MTU sizing via `Bluefruit.configPrphConn`.
- The advertising sequence counter and the per-channel frame `seq` are
  separate counters with separate owners (today `advSeq` is welded to
  the advertising rebuild tick).

**Bench:** subscribe in nRF Connect; frames arrive at the expected
cadence with monotonic bases; yank power mid-stream and reconnect —
`boot_id` changed, bases restarted.

## Phase 4 — security

- LESC + Just Works bonding, bonds persisted (InternalFS); verify the
  Seeed core fork actually carries LESC (Appendix A caveat in the
  spec) — if it only has legacy pairing, document the deviation, keep
  the rest.
- New-bond gating on the existing 30 s long-press pairing window
  (`PAIR_WINDOW_MS` / beacon flags bit0 — promoted to load-bearing);
  bonded reconnects accepted any time.
- Per-characteristic permissions per spec §8.

**Bench:** pair/bond/forget/re-pair from nRF Connect; pairing rejected
outside the window; bonded reconnect works with the window closed.

*Phases 3 and 4 are swappable; notify-first maximizes early value.*

## Phase 5 — logger central integration (cross-repo)

Out of this repo's code but in its contract:

- DovesDataLogger implements Central + GATT client against
  [PW_SENSOR_SERVICE.md](PW_SENSOR_SERVICE.md): scan list fed by the
  PW-ADV-2 beacon, bond-and-whitelist claim flow, Descriptor-driven
  channel setup, per-pod linear clock fit + `boot_id` epoch handling.
- The `pw_gatt_encode` golden fixtures get duplicated into the logger's
  test suite, exactly like `sensoregg_protocol` pins PW-ADV-2 today.
- Scheduler hygiene per [ARCHITECTURE.md](ARCHITECTURE.md) §7 (stagger
  pod links off the camera link; skinny central event lengths).

## Parking lot (open, not scheduled)

- **DFU-from-logger.** Bluefruit bundles an OTA DFU service (`BLEDfu`)
  so the cheap option exists, but it MUST be bond-gated and the
  security review is unfinished.
- **PW-ADV-3 generic beacon** — headline values keyed by the channel
  schema instead of the EGT-specific layout
  ([PW_ADV_2.md](PW_ADV_2.md) §7). Budget-tight; cosmetic while the
  connection is the data path.
- **Sub-ms sample intervals** (>1 kHz channels) — spec §5's u16 ms
  interval floor.
- **Encrypted-only PerchWerks service** in a future revision — spec §8;
  waits on multi-logger etiquette experience.
- **CAN backpack transport spec** — the channel schema is ready for
  it; the transport document doesn't exist yet.
