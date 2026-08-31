# tools/

## build-uf2.sh

Compiles the sketch and packages it as a **UF2** for drag-and-drop
flashing — no IDE, no DFU utility on the flashing machine:

1. Double-tap the XIAO nRF52840's reset button; it reboots into the UF2
   bootloader and shows up as a USB drive (`XIAO-SENSE`-style label).
2. Copy the `.uf2` from `build/` onto that drive.
3. The board flashes itself and reboots into the new firmware.

Requires: `arduino-cli` with the Seeed nRF52 core and the display
libraries installed, `python3`, and `adafruit-nrfutil` on PATH (the
Seeed BSP's post-build hook calls it) — the exact setup the
`compile-sketch` workflow uses. CI runs this script on every push/PR
and uploads the `.uf2` as a workflow artifact.

The conversion works from the exported Intel HEX, so the flash
addresses come from the linker — no hand-fed base address, and the
SoftDevice/bootloader regions are untouched (the UF2 carries only the
application, like any drag-drop app update on this bootloader).

## uf2conv.py / uf2families.json

Vendored verbatim from <https://github.com/microsoft/uf2>
(`utils/uf2conv.py`, `utils/uf2families.json`; MIT license), fetched
2026-08-29:

```
ad36ba2d61fb2ea371832262392088281eee474c609df4142adbee4ea3c20f26  uf2conv.py
9d8f561e050507c5909063005cd6f5b157e6fe6db1e8d25b5d9027d57064c194  uf2families.json
```

Vendored so the build never fetches a script off the network at CI
time. The nRF52840's UF2 family ID is `0xADA52840`.
