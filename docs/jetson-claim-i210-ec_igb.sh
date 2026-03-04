#!/usr/bin/env bash
set -euo pipefail

# Force-claim Intel i210 from built-in igb to ec_igb on Jetson kernels
# where CONFIG_IGB=y.

BDF="${BDF:-0001:01:00.0}"
IFACE="${IFACE:-enP1p1s0}"
PCI_DEV="/sys/bus/pci/devices/${BDF}"
IGB_UNBIND="/sys/bus/pci/drivers/igb/unbind"
EC_IGB_BIND="/sys/bus/pci/drivers/ec_igb/bind"
EC_IGB_NEW_ID="/sys/bus/pci/drivers/ec_igb/new_id"

if [[ ! -e "${PCI_DEV}" ]]; then
  echo "ERROR: PCI device ${BDF} not found."
  exit 1
fi

# Wait for ec_igb to be loaded by ethercatctl (with proper ec_master main_devices).
# Do NOT modprobe ec_igb here, as that would auto-load ec_master without parameters.
timeout=10
while [[ ! -e "${EC_IGB_BIND}" ]] && ((timeout > 0)); do
  sleep 0.5
  ((timeout--))
done
if [[ ! -e "${EC_IGB_BIND}" ]]; then
  echo "ERROR: ec_igb driver sysfs path not present. Did ethercatctl start run first?"
  exit 1
fi

# Already claimed: nothing to do.
if [[ -L "${PCI_DEV}/driver" ]]; then
  current_driver="$(basename "$(readlink -f "${PCI_DEV}/driver")")"
  if [[ "${current_driver}" == "ec_igb" ]]; then
    echo "OK: ${BDF} already claimed by ec_igb."
    exit 0
  fi
fi

# If native igb currently owns the device, unbind it.
if [[ -L "${PCI_DEV}/driver" ]]; then
  current_driver="$(basename "$(readlink -f "${PCI_DEV}/driver")")"
  if [[ "${current_driver}" == "igb" ]]; then
    ip link set "${IFACE}" down 2>/dev/null || true
    echo "${BDF}" > "${IGB_UNBIND}"
  fi
fi

# Pin this PCI function to ec_igb.
echo ec_igb > "${PCI_DEV}/driver_override"

# Bind to ec_igb (register i210 PCI ID as fallback if needed).
if ! echo "${BDF}" > "${EC_IGB_BIND}" 2>/dev/null; then
  # "File exists" on new_id is harmless if the ID was already registered.
  echo "8086 1533" > "${EC_IGB_NEW_ID}" 2>/dev/null || true
  echo "${BDF}" > "${EC_IGB_BIND}" 2>/dev/null || true
fi

if [[ -L "${PCI_DEV}/driver" ]] && [[ "$(basename "$(readlink -f "${PCI_DEV}/driver")")" == "ec_igb" ]]; then
  echo "OK: ${BDF} claimed by ec_igb."
  exit 0
fi

echo "ERROR: ${BDF} not claimed by ec_igb."
exit 1
