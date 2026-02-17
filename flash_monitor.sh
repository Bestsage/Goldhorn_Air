#!/bin/bash
set -e

# ================= Configuration =================
CRATE_NAME="flight-controller-rust" # Must match name in Cargo.toml
OUTPUT_BIN="firmware.bin"
DFU_ADDRESS="0x08000000:leave"
SERIAL_PORT="/dev/ttyACM0"
# =================================================

echo "[1/3] Building Project..."

# Check for cargo-binutils
if ! cargo --list | grep -q "objcopy"; then
    echo "  'cargo objcopy' not found. Installing cargo-binutils..."
    cargo install cargo-binutils
    rustup component add llvm-tools-preview
fi

# Build and create binary
# Note: cargo objcopy automatically handles the target defined in .cargo/config.toml
cargo objcopy --release -- -O binary $OUTPUT_BIN
echo "✅ Build successful: $OUTPUT_BIN created"

echo "🔥 [2/3] Flashing via DFU..."

# Check for dfu-util
if ! command -v dfu-util &> /dev/null; then
    echo "❌ Error: 'dfu-util' is not installed. Please install it (e.g. sudo apt install dfu-util)."
    exit 1
fi

# Flash
echo "👉 Please put the board in DFU Mode (Hold BOOT button, press RESET)"
# Try to flash. If it fails, we assume no device found.
if dfu-util -a 0 -s $DFU_ADDRESS -D $OUTPUT_BIN; then
    echo "✅ Flash successful!"
else
    echo "❌ Flash failed. Is the device connected in DFU mode?"
    exit 1
fi

echo "👀 [3/3] Opening Serial Monitor..."
echo "⏳ Waiting for device to re-enumerate on $SERIAL_PORT..."
sleep 3

if [ -e "$SERIAL_PORT" ]; then
    echo "✅ Port found: $SERIAL_PORT"
    
    if command -v tio &> /dev/null; then
        tio "$SERIAL_PORT"
    elif command -v minicom &> /dev/null; then
        minicom -D "$SERIAL_PORT"
    elif command -v picocom &> /dev/null; then
        picocom -b 115200 "$SERIAL_PORT"
    else
        echo "⚠️  No advanced serial tool found (tio, minicom, picocom)."
        echo "Using 'cat' to dump output. Press Ctrl+C to stop."
        # Configure basic serial parameters
        stty -F $SERIAL_PORT 115200 raw -echo -echoe -echok
        cat $SERIAL_PORT
    fi
else
    echo "❌ Error: Serial port $SERIAL_PORT not found after waiting."
    echo "   Check dmesg to see where it enumerated."
    exit 1
fi
