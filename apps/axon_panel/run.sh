#!/bin/bash
# Quick start script for Axon Webtool

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Check if node_modules exists
if [ ! -d "node_modules" ]; then
  echo "Installing dependencies..."
  npm install
fi

echo "Starting Axon Webtool development server..."
echo "Open http://localhost:3000 in your browser"
echo ""
echo "Make sure Axon Recorder is running on port 8080"
echo ""

npm run dev
