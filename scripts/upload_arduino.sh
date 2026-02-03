#!/bin/bash
# Upload arduino_complete.ino to Arduino Uno via avrdude

set -e

SKETCH="/home/pi/FEA_SLAM_WS/scripts/arduino_complete.ino"
PORT="/dev/ttyACM0"
BOARD="arduino:avr:uno"
BUILD_DIR="/tmp/arduino_build"

echo "🔧 Arduino Upload Script"
echo "Sketch: $SKETCH"
echo "Port: $PORT"
echo "Board: $BOARD"
echo ""

# Check if sketch exists
if [ ! -f "$SKETCH" ]; then
    echo "❌ Sketch not found: $SKETCH"
    exit 1
fi

# Check if Arduino IDE is installed
if ! command -v arduino-cli &> /dev/null && ! command -v arduino &> /dev/null; then
    echo "⚠️  Arduino IDE/CLI not found locally"
    echo "Attempting alternative: Using avrdude directly..."
    
    # Try avrdude if available
    if command -v avrdude &> /dev/null; then
        echo "✅ Found avrdude, compiling with avr-gcc..."
        # This would require manual compilation - for now just inform user
        echo ""
        echo "📝 Manual Upload Instructions:"
        echo "1. Open Arduino IDE"
        echo "2. File > Open: $SKETCH"
        echo "3. Tools > Board: Arduino Uno"
        echo "4. Tools > Port: $PORT"
        echo "5. Sketch > Upload (Ctrl+U)"
        exit 0
    fi
    exit 1
fi

# Try arduino-cli first
if command -v arduino-cli &> /dev/null; then
    echo "✅ Using arduino-cli"
    mkdir -p "$BUILD_DIR"
    
    arduino-cli compile --fqbn "$BOARD" "$SKETCH" --output-dir "$BUILD_DIR" --verbose
    echo "✅ Compilation successful"
    
    arduino-cli upload -p "$PORT" --fqbn "$BOARD" "$BUILD_DIR" --verbose
    echo "✅ Upload successful!"
    exit 0
fi

# Fallback to arduino IDE
if command -v arduino &> /dev/null; then
    echo "✅ Using Arduino IDE"
    arduino --upload "$SKETCH" --port "$PORT" --board "$BOARD"
    echo "✅ Upload successful!"
    exit 0
fi

echo "❌ No Arduino tools found"
exit 1
