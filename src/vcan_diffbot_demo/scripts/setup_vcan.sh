#!/usr/bin/env bash
set -euo pipefail

interface="${1:-vcan0}"

sudo modprobe vcan 2>/dev/null || true
if ! ip link show "$interface" >/dev/null 2>&1; then
  sudo ip link add dev "$interface" type vcan
fi
sudo ip link set up "$interface"
ip -details link show "$interface"
