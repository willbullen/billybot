#!/bin/bash
#
# OpenClaw (BillyBot) Jetson Configuration Deployment Script
# This script helps transfer the populated configuration files to your Jetson
#

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}🦞 OpenClaw Jetson Configuration Deployment${NC}"
echo "=============================================="
echo ""

# Check if JETSON_HOST is set
if [ -z "$JETSON_HOST" ]; then
    echo -e "${YELLOW}Please set JETSON_HOST environment variable${NC}"
    echo "Example: export JETSON_HOST=william@jetson.local"
    echo "Or run: JETSON_HOST=william@jetson.local ./jetson-deploy.sh"
    exit 1
fi

echo -e "${GREEN}Target Jetson:${NC} $JETSON_HOST"
echo ""

# Source and destination paths
SRC_CONFIG="/home/willbullen/.openclaw/openclaw.json"
SRC_ENV_DIR="/home/willbullen/.openclaw/env"
DEST_CONFIG="/home/william/.openclaw/openclaw.json"
DEST_ENV_DIR="/home/william/.openclaw/env"

# Function to deploy file
deploy_file() {
    local src=$1
    local dest=$2
    local desc=$3
    
    echo -e "${YELLOW}Deploying:${NC} $desc"
    echo "  From: $src"
    echo "  To: $JETSON_HOST:$dest"
    
    # Create remote directory
    ssh "$JETSON_HOST" "mkdir -p \$(dirname $dest)"
    
    # Copy file
    scp "$src" "$JETSON_HOST:$dest"
    
    # Set permissions
    ssh "$JETSON_HOST" "chmod 600 $dest"
    
    echo -e "${GREEN}✓${NC} Deployed successfully"
    echo ""
}

# Deploy main configuration
if [ -f "$SRC_CONFIG" ]; then
    deploy_file "$SRC_CONFIG" "$DEST_CONFIG" "Main configuration file"
else
    echo -e "${RED}✗ Configuration file not found: $SRC_CONFIG${NC}"
    exit 1
fi

# Deploy environment files
if [ -d "$SRC_ENV_DIR" ]; then
    echo -e "${YELLOW}Deploying environment files...${NC}"
    
    # Create remote directory
    ssh "$JETSON_HOST" "mkdir -p $DEST_ENV_DIR"
    
    # Copy all env files
    scp -r "$SRC_ENV_DIR"/* "$JETSON_HOST:$DEST_ENV_DIR/"
    
    # Set permissions
    ssh "$JETSON_HOST" "chmod 600 $DEST_ENV_DIR/*"
    
    echo -e "${GREEN}✓${NC} Environment files deployed"
    echo ""
else
    echo -e "${RED}✗ Environment directory not found: $SRC_ENV_DIR${NC}"
    exit 1
fi

# Verify deployment
echo -e "${YELLOW}Verifying deployment...${NC}"
ssh "$JETSON_HOST" "ls -lh /home/william/.openclaw/ && echo '---' && ls -lh /home/william/.openclaw/env/"

echo ""
echo -e "${GREEN}✅ Deployment complete!${NC}"
echo ""
echo "Next steps on your Jetson:"
echo "1. Verify configuration: cat ~/.openclaw/openclaw.json"
echo "2. Check environment files: ls -la ~/.openclaw/env/"
echo "3. Start OpenClaw gateway: openclaw gateway --port 18789 --verbose"
echo "4. Or run onboarding: openclaw onboard --install-daemon"
echo ""
echo -e "${YELLOW}⚠️  Security reminder:${NC}"
echo "- All API keys and credentials are now on the Jetson"
echo "- Files are protected with 600 permissions (owner read/write only)"
echo "- Never commit these files to git"
echo ""
echo "🦞 OpenClaw: Your own personal AI assistant. The lobster way."
echo "Source: https://github.com/openclaw/openclaw"
echo ""
