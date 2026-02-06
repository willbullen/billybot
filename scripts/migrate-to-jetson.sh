#!/bin/bash
# BillyBot Migration Script for Jetson Orin NX
# This script automates the migration from current system to Jetson

set -euo pipefail  # Exit on error, undefined vars, pipe failures

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
BACKUP_DIR="$PROJECT_ROOT/backups"
LOG_FILE="$PROJECT_ROOT/migration.log"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")

# Logging function
log() {
    echo -e "${BLUE}[$(date '+%Y-%m-%d %H:%M:%S')]${NC} $1" | tee -a "$LOG_FILE"
}

error() {
    echo -e "${RED}[ERROR]${NC} $1" | tee -a "$LOG_FILE"
    exit 1
}

success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1" | tee -a "$LOG_FILE"
}

warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1" | tee -a "$LOG_FILE"
}

# Check if running on Jetson
check_jetson() {
    log "Checking if this is a Jetson system..."
    
    if [[ -f /etc/nv_tegra_release ]]; then
        success "Detected Jetson system"
        log "JetPack version: $(cat /etc/nv_tegra_release)"
        return 0
    else
        warning "This doesn't appear to be a Jetson system"
        read -p "Continue anyway? (y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            error "Migration cancelled"
        fi
    fi
}

# Check system requirements
check_requirements() {
    log "Checking system requirements..."
    
    # Check available disk space
    AVAILABLE_SPACE=$(df -BG "$PROJECT_ROOT" | awk 'NR==2 {print $4}' | sed 's/G//')
    if [[ $AVAILABLE_SPACE -lt 10 ]]; then
        error "Insufficient disk space. Need at least 10GB, have ${AVAILABLE_SPACE}GB"
    fi
    
    # Check memory
    TOTAL_MEMORY=$(free -g | awk 'NR==2{print $2}')
    if [[ $TOTAL_MEMORY -lt 8 ]]; then
        warning "Low memory detected: ${TOTAL_MEMORY}GB. Recommend 8GB+ for optimal performance"
    fi
    
    success "System requirements check passed"
}

# Install system dependencies
install_dependencies() {
    log "Installing system dependencies..."
    
    # Update package lists
    sudo apt update
    
    # Install essential packages
    sudo apt install -y \
        curl \
        wget \
        git \
        build-essential \
        python3 \
        python3-pip \
        python3-venv \
        nodejs \
        npm \
        docker.io \
        docker-compose \
        jq \
        tree \
        htop \
        vim \
        unzip \
        ca-certificates \
        gnupg \
        lsb-release
    
    # Add user to docker group
    sudo usermod -aG docker $USER
    
    # Install Node.js (latest LTS)
    curl -fsSL https://deb.nodesource.com/setup_lts.x | sudo -E bash -
    sudo apt install -y nodejs
    
    # Install Python packages globally
    pip3 install --upgrade pip setuptools wheel
    
    success "System dependencies installed"
}

# Install Clawdbot
install_clawdbot() {
    log "Installing Clawdbot..."
    
    # Create Clawdbot directory
    sudo mkdir -p /opt/clawdbot
    sudo chown $USER:$USER /opt/clawdbot
    
    # Install globally via npm
    sudo npm install -g clawdbot
    
    # Create workspace directory
    mkdir -p ~/clawd
    
    # Initialize Clawdbot
    clawdbot wizard --mode local --workspace ~/clawd || true
    
    success "Clawdbot installed and initialized"
}

# Install Python dependencies
install_python_deps() {
    log "Installing Python dependencies..."
    
    # Create virtual environment
    python3 -m venv ~/clawd/venv
    source ~/clawd/venv/bin/activate
    
    # Install common packages
    pip install --upgrade pip
    pip install -r "$PROJECT_ROOT/dependencies/pip-requirements.txt"
    
    success "Python dependencies installed"
}

# Install Node.js dependencies
install_node_deps() {
    log "Installing Node.js dependencies..."
    
    # Create package.json if it doesn't exist
    if [[ ! -f ~/clawd/package.json ]]; then
        cd ~/clawd
        npm init -y
    fi
    
    # Install global packages
    npm install -g $(cat "$PROJECT_ROOT/dependencies/npm-packages.txt")
    
    success "Node.js dependencies installed"
}

# Configure system services
configure_services() {
    log "Configuring system services..."
    
    # Copy systemd service files
    sudo cp "$PROJECT_ROOT/config/systemd/clawdbot.service" /etc/systemd/system/
    sudo cp "$PROJECT_ROOT/config/systemd/clawdbot-updater.service" /etc/systemd/system/
    
    # Enable and start services
    sudo systemctl daemon-reload
    sudo systemctl enable clawdbot.service
    sudo systemctl enable clawdbot-updater.service
    
    success "System services configured"
}

# Restore configuration
restore_config() {
    log "Restoring configuration..."
    
    # Check if backup exists
    if [[ -f "$BACKUP_DIR/config-backup.tar.gz" ]]; then
        log "Restoring from backup..."
        tar -xzf "$BACKUP_DIR/config-backup.tar.gz" -C ~/
        success "Configuration restored from backup"
    else
        warning "No backup found, using template configuration"
        cp "$PROJECT_ROOT/config/clawdbot.json.template" ~/clawd/clawdbot.json
        success "Template configuration applied"
    fi
}

# Test installation
test_installation() {
    log "Testing installation..."
    
    # Test Clawdbot
    if command -v clawdbot &> /dev/null; then
        success "Clawdbot CLI is accessible"
        clawdbot --version
    else
        error "Clawdbot CLI not found"
    fi
    
    # Test Python environment
    if [[ -f ~/clawd/venv/bin/python ]]; then
        success "Python virtual environment exists"
        ~/clawd/venv/bin/python --version
    else
        error "Python virtual environment not found"
    fi
    
    # Test Docker
    if command -v docker &> /dev/null; then
        success "Docker is installed"
        docker --version
    else
        error "Docker not found"
    fi
    
    # Test services
    if systemctl is-active --quiet clawdbot.service; then
        success "Clawdbot service is running"
    else
        warning "Clawdbot service is not running (may need configuration)"
    fi
    
    success "Basic installation tests passed"
}

# Main migration function
main() {
    log "🚀 Starting BillyBot migration to Jetson Orin NX"
    log "Timestamp: $TIMESTAMP"
    log "Project root: $PROJECT_ROOT"
    
    # Pre-flight checks
    check_jetson
    check_requirements
    
    # Installation phase
    install_dependencies
    install_clawdbot
    install_python_deps
    install_node_deps
    configure_services
    restore_config
    
    # Testing phase
    test_installation
    
    # Final steps
    success "🎉 Migration completed successfully!"
    log "Next steps:"
    log "1. Review the configuration in ~/clawd/clawdbot.json"
    log "2. Start services: sudo systemctl start clawdbot.service"
    log "3. Test functionality: clawdbot status"
    log "4. Run full integration tests: $PROJECT_ROOT/scripts/test-integration.sh"
    log "5. Check logs: journalctl -u clawdbot -f"
    
    log "Migration log saved to: $LOG_FILE"
}

# Handle script arguments
case "${1:-}" in
    --help|-h)
        echo "BillyBot Migration Script for Jetson Orin NX"
        echo ""
        echo "Usage: $0 [OPTIONS]"
        echo ""
        echo "Options:"
        echo "  --help, -h     Show this help message"
        echo "  --dry-run      Show what would be installed (not implemented)"
        echo "  --backup-only  Only perform backup (not implemented)"
        echo ""
        echo "This script will:"
        echo "  1. Check system requirements"
        echo "  2. Install all dependencies"
        echo "  3. Configure Clawdbot"
        echo "  4. Test the installation"
        echo ""
        echo "Run without arguments to perform full migration."
        exit 0
        ;;
    --dry-run)
        warning "Dry run mode not implemented yet"
        exit 1
        ;;
    --backup-only)
        warning "Backup-only mode not implemented yet"
        exit 1
        ;;
    "")
        # Default: run full migration
        main
        ;;
    *)
        error "Unknown option: $1. Use --help for usage information."
        ;;
esac