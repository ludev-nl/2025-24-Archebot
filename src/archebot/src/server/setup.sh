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
    virtualenv "$SCRIPT_DIR/venv" --no-wheel
fi

echo "Activating virtual environment..."
source "$SCRIPT_DIR/venv/bin/activate"
echo "Virtual environment activated."

pip install --upgrade pip
pip install setuptools --no-use-wheel --upgrade
pip install wheel --no-cache

# Install dependencies
echo "Installing dependencies..."
pip install -r "$SCRIPT_DIR/../../../../requirements.txt"
echo "Dependencies installed."

# Deactivate the venv
deactivate
