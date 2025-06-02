#!/bin/bash

# Get the first IP address
IP=$(hostname -I | awk '{print $1}')
PORT=5000
ENV_FILE=".env"
URL="http://$IP:$PORT"

# Create .env file if it doesn't exist
touch "$ENV_FILE"

# Update or insert the NEXT_PUBLIC_API_BASE_URL
if grep -q "^NEXT_PUBLIC_API_BASE_URL=" "$ENV_FILE"; then
  sed -i "s|^NEXT_PUBLIC_API_BASE_URL=.*|NEXT_PUBLIC_API_BASE_URL=$URL|" "$ENV_FILE"
else
  echo "NEXT_PUBLIC_API_BASE_URL=$URL" >> "$ENV_FILE"
fi

echo "NEXT_PUBLIC_API_BASE_URL set to $URL in $ENV_FILE"
