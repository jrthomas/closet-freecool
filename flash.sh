#!/bin/bash
#
# flash.sh -- compile and OTA-push an ESP32-POE-ISO sketch
#
#   ./flash.sh                     compile + push to the default target
#   ./flash.sh 192.168.1.50        compile + push to a specific IP
#   ./flash.sh -c                  compile only, no upload
#
# The default target IP and the OTA password are read from
# <sketch>/secrets.h (OTA_DEFAULT_IP, OTA_PASSWORD), so they live in
# one git-ignored file. $OTA_PASS, if set in the environment, still
# overrides the file -- handy for pushing to a board that has not
# yet been reflashed with a new password.
#
# espota needs the BOARD to open a TCP connection back to this host
# on HOST_PORT, so that port must be open in ufw:
#   sudo ufw allow 40170/tcp
#
set -euo pipefail

SKETCH="poe_iso_eth_ota"
FQBN="esp32:esp32:esp32-poe-iso"
CORE_VER="3.3.11"
HOST_PORT=40170          # fixed so one ufw rule stays valid
BUILD_DIR="./build"

ESPOTA="$HOME/.arduino15/packages/esp32/hardware/esp32/${CORE_VER}/tools/espota.py"

cd "$(dirname "$0")"

# --- site config from secrets.h ---------------------------------
SECRETS="${SKETCH}/secrets.h"
[[ -f "$SECRETS" ]] || {
  echo "No $SECRETS -- copy ${SKETCH}/secrets.example.h to secrets.h and fill it in" >&2
  exit 1
}

# Pull a quoted string value out of a '#define NAME "value"' line.
secret() { sed -n "s/^[[:space:]]*#define[[:space:]]\+$1[[:space:]]\+\"\([^\"]*\)\".*/\1/p" "$SECRETS" | head -n1; }

DEFAULT_IP="$(secret OTA_DEFAULT_IP)"

# --- args -------------------------------------------------------
COMPILE_ONLY=0
IP="$DEFAULT_IP"
for arg in "$@"; do
  case "$arg" in
    -c|--compile-only) COMPILE_ONLY=1 ;;
    -h|--help) sed -n '2,17p' "$0"; exit 0 ;;
    *) IP="$arg" ;;
  esac
done

# --- sanity -----------------------------------------------------
[[ -d "$SKETCH" ]] || { echo "No sketch directory ./$SKETCH" >&2; exit 1; }

if [[ "$COMPILE_ONLY" -eq 0 ]]; then
  [[ -f "$ESPOTA" ]] || {
    echo "espota.py not found at $ESPOTA" >&2
    echo "Check CORE_VER -- installed cores:" >&2
    arduino-cli core list >&2
    exit 1
  }
  OTA_PASS="${OTA_PASS:-$(secret OTA_PASSWORD)}"
  [[ -n "$OTA_PASS" ]] || { echo "OTA_PASSWORD not found in $SECRETS" >&2; exit 1; }
  [[ -n "$IP" ]]       || { echo "No target IP -- set OTA_DEFAULT_IP in $SECRETS or pass one" >&2; exit 1; }
fi

# --- compile ----------------------------------------------------
# set -e aborts here on failure, so a stale binary is never pushed.
echo "==> Compiling $SKETCH"
arduino-cli compile --fqbn "$FQBN" --output-dir "$BUILD_DIR" "$SKETCH"

BIN="${BUILD_DIR}/${SKETCH}.ino.bin"
[[ -f "$BIN" ]] || { echo "Expected binary not found: $BIN" >&2; exit 1; }

if [[ "$COMPILE_ONLY" -eq 1 ]]; then
  echo "==> Compile only; not uploading."
  exit 0
fi

# --- upload -----------------------------------------------------
echo "==> Pushing to ${IP} (host port ${HOST_PORT})"
python3 "$ESPOTA" \
    -i "$IP" \
    -p 3232 \
    -P "$HOST_PORT" \
    -a "$OTA_PASS" \
    -f "$BIN" \
    -r -d

echo "==> Done. Board is rebooting; reconnect with: telnet ${IP}"
