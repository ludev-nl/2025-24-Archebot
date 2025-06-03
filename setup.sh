#!/bin/bash

# Set script to exit on error
set -e

# Get the script's directory
SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"

# Build server
cd "$SCRIPT_DIR/src/archebot/src/server"
./setup.sh

# Navigate to script directory
cd "$SCRIPT_DIR/src/archebot/src/webapp"

# Build webapp
pnpm build
