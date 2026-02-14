#!/bin/bash

# Configuration
EXE_NAME="FluxGeo"  # Change this to your actual binary name
BUILD_DIR="build-linux"
LIB_DIR="$BUILD_DIR/libs"

mkdir -p $LIB_DIR

echo "Copying dependencies..."

# Use ldd to find paths, grep to get absolute paths, and xargs to copy
ldd $BUILD_DIR/$EXE_NAME | grep "=> /" | awk '{print $3}' | xargs -I '{}' cp -v '{}' $LIB_DIR/

echo "Dependencies collected in $LIB_DIR"

# Patch the binary to look in the relative 'libs' folder
# This requires 'patchelf' (sudo apt install patchelf)
if command -v patchelf >/dev/null 2>&1; then
    echo "Patching RPATH..."
    patchelf --set-rpath '$ORIGIN/libs' $BUILD_DIR/$EXE_NAME
    echo "Done! You can now move the '$BUILD_DIR' folder to any Linux machine."
else
    echo "Error: patchelf not found. Run 'sudo apt install patchelf' to fix the binary path."
fi
