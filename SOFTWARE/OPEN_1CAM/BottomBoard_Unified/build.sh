#!/bin/bash
echo "Compiling for Teensy 4.1..."
arduino-cli compile -b teensy:avr:teensy41

if [ $? -eq 0 ]; then
    echo "Compilation successful!"
    
    # Auto-detect Teensy port
    PORT=$(arduino-cli board list | awk '/teensy/ {print $1}' | head -n 1)
    
    if [ -z "$PORT" ]; then
        echo "Error: No Teensy board found! Please plug it in."
        exit 1
    fi
    
    echo "Teensy found on port: $PORT"
    echo "Uploading..."
    arduino-cli upload -b teensy:avr:teensy41 -p "$PORT"
    
    if [ $? -eq 0 ]; then
        echo "Upload complete!"
    else
        echo "Upload failed!"
    fi
else
    echo "Compilation failed!"
fi
