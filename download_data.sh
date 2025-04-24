#!/bin/bash

# ------------------------------
# CONFIGURATION
# ------------------------------
PROJECT_ID="hwajz"                        # OSF project ID
REMOTE_DIR="checkpoints"                  # Directory to download from OSF
LOCAL_DIR="ruka_data"                   # Local directory to save files
OSF_USERNAME="${OSF_USERNAME:-}"          # Set via env variable or prompt
OSF_TOKEN="${OSF_TOKEN:-}"                # Set via env variable or prompt
CONFIG_FILE="$HOME/.osfcli.config"        # Temporary config file path
# ------------------------------

# Prompt for credentials if not set
if [ -z "$OSF_USERNAME" ]; then
    read -p "Enter your OSF username (email): " OSF_USERNAME
fi

if [ -z "$OSF_TOKEN" ]; then
    read -s -p "Enter your OSF token: " OSF_TOKEN
    echo ""
fi

# Make sure osfclient is installed
if ! command -v osf &> /dev/null; then
    echo "❌ osfclient is not installed. Please install it: pip install osfclient"
    exit 1
fi

# Create local directory
mkdir -p "$LOCAL_DIR"

osf --project "$PROJECT_ID" clone "$LOCAL_DIR"
echo "✅ Download complete. Files saved in: $LOCAL_DIR/osfstorage/checkpoints"