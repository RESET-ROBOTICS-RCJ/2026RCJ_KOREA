#!/bin/bash

set -e

FQBN="teensy:avr:teensy41"
BAUD="115200"

cd "$(dirname "$0")"

echo "Compiling Camera_UART_Test for Teensy 4.1..."
arduino-cli compile -b "$FQBN"
echo "Compilation successful!"

# Auto-detect Teensy port.
PORT=$(arduino-cli board list | awk '/[tT]eensy|usbmodem/ {print $1}' | head -n 1)

if [ -z "$PORT" ]; then
    echo "Error: No Teensy board found. Please plug it in."
    exit 1
fi

echo "Teensy found on port: $PORT"
echo "Uploading..."
arduino-cli upload -b "$FQBN" -p "$PORT"
echo "Upload complete!"

echo
echo "Opening serial monitor at $BAUD baud."
echo "Press Ctrl+C to quit."
echo
arduino-cli monitor -p "$PORT" -c baudrate="$BAUD"
