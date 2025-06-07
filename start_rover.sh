#!/bin/bash

# Set script to exit on error
set -e

# Get the script's directory
SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"

# Navigate to script directory
cd "$SCRIPT_DIR"

# Activate Catkin workspace
source "$SCRIPT_DIR/devel/setup.bash"

# Launch rover
roslaunch archebot archebot.launch

# Deactivate Catkin workspace
deactivate
