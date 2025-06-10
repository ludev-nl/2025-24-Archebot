#!/bin/bash

# Set script to exit on error
set -e

# Get the script's directory
SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"

# Build rover
cd "$SCRIPT_DIR"
catkin build

# Build server
cd "$SCRIPT_DIR/src/archebot/src/server"
./setup.sh

# Build webapp
cd "$SCRIPT_DIR/src/archebot/src/webapp"
pnpm i
pnpm build
