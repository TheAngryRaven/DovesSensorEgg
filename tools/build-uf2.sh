#!/usr/bin/env bash
# Compile the egg firmware and package it as a UF2 for drag-and-drop
# flashing (see tools/README.md for the double-tap-reset flow).
#
# Needs arduino-cli (Seeed nRF52 core + display libraries installed),
# python3, and adafruit-nrfutil on PATH — the same toolchain the
# compile-sketch workflow sets up.
set -euo pipefail

cd "$(dirname "$0")/.."   # repo root

FQBN="Seeeduino:nrf52:xiaonRF52840"
FQBN_DIR="${FQBN//:/.}"   # --export-binaries writes build/<fqbn-with-dots>/
SHA="$(git rev-parse --short HEAD 2>/dev/null || echo local)"
HEX="build/${FQBN_DIR}/DovesSensorEgg.ino.hex"
OUT="build/DovesSensorEgg-${SHA}.uf2"

arduino-cli compile --fqbn "$FQBN" --export-binaries .

# Convert from the Intel HEX so every block lands at its linked address:
# app region only, SoftDevice and bootloader untouched. 0xADA52840 is
# the nRF52840 UF2 family ID (tools/uf2families.json).
python3 tools/uf2conv.py -f 0xADA52840 -c -o "$OUT" "$HEX"

echo "UF2 ready: $OUT"
