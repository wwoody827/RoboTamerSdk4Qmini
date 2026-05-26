#!/bin/bash
# ps4_pair.sh - Pair PS4 controller via Bluetooth

echo "Put PS controller into pairing mode (hold Share + PS button for 3 seconds)"
echo "Press Enter when controller is flashing..."
read

# Start scanning
echo "Scanning for PS controller..."
timeout 10s bluetoothctl scan on &
sleep 5

# Get MAC address of Wireless Controller
MAC=$(bluetoothctl devices | grep "Wireless Controller" | awk '{print $2}')

if [ -z "$MAC" ]; then
    echo "PS controller not found. Make sure it is in pairing mode."
    exit 1
fi

echo "Found PS controller: $MAC"

# Pair, trust, and connect
echo "Pairing..."
echo -e "pair $MAC\ntrust $MAC\nconnect $MAC\nexit" | bluetoothctl

echo "PS controller should now be paired and connected!"
