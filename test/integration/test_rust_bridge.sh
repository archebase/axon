#!/bin/bash
# Integration test for Rust bridge library

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRIDGE_DIR="$SCRIPT_DIR/../../src/bridge"
TEMP_DIR=$(mktemp -d)

echo "Testing Rust bridge library..."

cd "$BRIDGE_DIR"

# Build the library
echo "Building Rust library..."
cargo build --release

# Run Rust unit tests
echo "Running Rust unit tests..."
cargo test --release

# Test FFI interface (would require C test program)
echo "FFI interface tests would go here..."

# Cleanup
rm -rf "$TEMP_DIR"

echo "All tests passed!"

