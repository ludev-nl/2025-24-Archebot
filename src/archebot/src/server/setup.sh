#!/bin/bash

# Set script to exit on error
set -e

# Get the script's directory
SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"

# Navigate to script directory
cd "$SCRIPT_DIR"

# Check if virtual environment exists
if [ ! -d "$SCRIPT_DIR/venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv "$SCRIPT_DIR/venv" --no-wheel
fi

echo "Activating virtual environment..."
source "$SCRIPT_DIR/venv/bin/activate"
echo "Virtual environment activated."

python3 -m pip install --upgrade pip
python3 -m pip install setuptools --no-use-wheel --upgrade
python3 -m pip install wheel --no-cache

# Install dependencies
echo "Installing dependencies..."
python3 -m pip install -r "$SCRIPT_DIR/../../../../requirements.txt"
echo "Dependencies installed."

# Deactivate the venv
deactivate
