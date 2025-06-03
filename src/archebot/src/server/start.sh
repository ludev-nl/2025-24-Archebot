#!/bin/bash

# Set script to exit on error
set -e

# Get the script's directory
SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"

# Navigate to script directory
cd "$SCRIPT_DIR"

# Handle --drop_db flag
DROP_DB=false
for arg in "$@"; do
  if [ "$arg" == "--drop_db" ]; then
    DROP_DB=true
  fi
done

# Delete the database if requested
if [ "$DROP_DB" = true ]; then
    echo "Dropping existing database..."
    rm -f "$SCRIPT_DIR/db/database.db"
fi

echo "Activating virtual environment..."
source "$SCRIPT_DIR/venv/bin/activate"
echo "Virtual environment activated."

# Initialize the database
echo "Initializing the database..."
python3 "$SCRIPT_DIR/db/init_db.py"

# Start the Flask server
echo "Starting the Flask server..."
python3 "$SCRIPT_DIR/src/routes.py"

# Deactivate the venv
deactivate
