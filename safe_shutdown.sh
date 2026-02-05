#!/bin/bash
# Safe shutdown helper for apps using the EtherCAT isolated process.
# - Sends SIGTERM to a matching process
# - Waits for graceful stop
# - Optionally attempts to release /dev/EtherCAT0 via fuser
#
# Usage examples:
#   ./safe_shutdown.sh --pattern "python.*your_app.py"
#   ./safe_shutdown.sh --pattern "python.*your_app.py" --timeout 20
#   ./safe_shutdown.sh --pattern "python.*your_app.py" --device /dev/EtherCAT0 --release-device

set -euo pipefail

PATTERN=""
TIMEOUT=15
DEVICE="/dev/EtherCAT0"
RELEASE_DEVICE=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --pattern)
      PATTERN="${2:-}"
      shift 2
      ;;
    --timeout)
      TIMEOUT="${2:-15}"
      shift 2
      ;;
    --device)
      DEVICE="${2:-/dev/EtherCAT0}"
      shift 2
      ;;
    --release-device)
      RELEASE_DEVICE=1
      shift 1
      ;;
    -h|--help)
      echo "Usage: $0 --pattern <pgrep_regex> [--timeout <sec>] [--device <path>] [--release-device]"
      exit 0
      ;;
    *)
      echo "Unknown arg: $1"
      echo "Run with --help for usage."
      exit 2
      ;;
  esac
done

if [[ -z "$PATTERN" ]]; then
  echo "ERROR: --pattern is required"
  echo "Example: $0 --pattern \"python.*your_app.py\""
  exit 2
fi

echo "=== EtherCAT Safe Shutdown ==="
echo "Pattern: $PATTERN"
echo "Timeout: ${TIMEOUT}s"
echo "Device:  $DEVICE"
echo

PIDS="$(pgrep -f "$PATTERN" || true)"
if [[ -z "$PIDS" ]]; then
  echo "No matching process found."
else
  echo "Found PIDs:"
  echo "$PIDS"
  echo
  echo "Sending SIGTERM..."
  # shellcheck disable=SC2086
  kill -TERM $PIDS 2>/dev/null || true

  for i in $(seq 1 "$TIMEOUT"); do
    # shellcheck disable=SC2086
    if ! kill -0 $PIDS 2>/dev/null; then
      echo "✓ Process terminated gracefully after ${i}s"
      break
    fi
    echo "  Waiting... (${i}/${TIMEOUT})"
    sleep 1
  done

  # shellcheck disable=SC2086
  if kill -0 $PIDS 2>/dev/null; then
    echo
    echo "! Process did not terminate within timeout - sending SIGKILL..."
    # shellcheck disable=SC2086
    kill -KILL $PIDS 2>/dev/null || true
    sleep 1
  fi
fi

if [[ "$RELEASE_DEVICE" -eq 1 ]]; then
  echo
  echo "=== Releasing EtherCAT device (best-effort) ==="
  if command -v fuser >/dev/null 2>&1 && [[ -e "$DEVICE" ]]; then
    fuser -k -TERM "$DEVICE" 2>/dev/null || true
    sleep 1
    fuser -k -KILL "$DEVICE" 2>/dev/null || true
    echo "✓ Release attempted via fuser"
  else
    echo "Skipping: fuser not available or device not present"
  fi
fi

echo
echo "=== Done ==="





