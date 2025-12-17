#!/bin/bash
# Integration test for C FFI library (Rust)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
C_FFI_DIR="$SCRIPT_DIR/../../c"
TEMP_DIR=$(mktemp -d)

echo "Testing C FFI library..."

cd "$C_FFI_DIR"

# Build the library
echo "Building C FFI library (Rust)..."
cargo build --release

# Run Rust unit tests
echo "Running C FFI unit tests..."
cargo test --release

# Test FFI interface (would require C test program)
echo "FFI interface tests would go here..."

# Cleanup
rm -rf "$TEMP_DIR"

echo "All tests passed!"

