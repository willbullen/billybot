#!/bin/bash
# BillyBot System Backup Script
# Creates comprehensive backup of current system before migration

set -euo pipefail

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
BACKUP_DIR="$PROJECT_ROOT/backups"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BACKUP_NAME="billybot-backup-$TIMESTAMP"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}🔄 BillyBot System Backup${NC}"
echo "Timestamp: $TIMESTAMP"
echo "Backup location: $BACKUP_DIR"
echo ""

# Create backup directory
mkdir -p "$BACKUP_DIR"

# Function to backup with progress
backup_item() {
    local source="$1"
    local dest="$2"
    local description="$3"
    
    echo -e "${BLUE}📦 Backing up: $description${NC}"
    
    if [[ -e "$source" ]]; then
        if [[ -d "$source" ]]; then
            # Directory backup with tar
            tar -czf "$dest" -C "$(dirname "$source")" "$(basename "$source")" 2>/dev/null || {
                echo -e "${YELLOW}⚠️  Warning: Could not backup directory $source${NC}"
                return 1
            }
        else
            # File backup
            cp "$source" "$dest" 2>/dev/null || {
                echo -e "${YELLOW}⚠️  Warning: Could not backup file $source${NC}"
                return 1
            }
        fi
        echo -e "${GREEN}✅ Backed up: $description${NC}"
        return 0
    else
        echo -e "${YELLOW}⚠️  Skipping (not found): $description${NC}"
        return 0
    fi
}

# Create manifest
echo "Creating backup manifest..."
cat > "$BACKUP_DIR/manifest-$TIMESTAMP.txt" << EOF
BillyBot System Backup
======================
Timestamp: $TIMESTAMP
Date: $(date)
Hostname: $(hostname)
User: $(whoami)

Backup Contents:
EOF

# Backup Clawdbot configuration and data
echo "Backing up Clawdbot..."
backup_item "$HOME/.clawdbot" "$BACKUP_DIR/clawdbot-config-$TIMESTAMP.tar.gz" "Clawdbot configuration"
backup_item "$HOME/clawd" "$BACKUP_DIR/clawd-data-$TIMESTAMP.tar.gz" "Clawdbot workspace"

# Backup environment files
echo "Backing up environment files..."
backup_item "$HOME/.bashrc" "$BACKUP_DIR/bashrc-$TIMESTAMP" "Bash configuration"
backup_item "$HOME/.profile" "$BACKUP_DIR/profile-$TIMESTAMP" "Profile configuration"

# Backup npm global packages
echo "Backing up npm global packages..."
npm list -g --depth=0 > "$BACKUP_DIR/npm-global-packages-$TIMESTAMP.txt" 2>/dev/null || echo "No npm packages found"

# Backup Python packages
echo "Backing up Python packages..."
pip3 list > "$BACKUP_DIR/pip-packages-$TIMESTAMP.txt" 2>/dev/null || echo "No pip packages found"

# Backup system packages
echo "Backing up system packages..."
dpkg -l > "$BACKUP_DIR/system-packages-$TIMESTAMP.txt" 2>/dev/null || echo "No dpkg packages found"

# Backup cron jobs
echo "Backing up cron jobs..."
crontab -l > "$BACKUP_DIR/crontab-$TIMESTAMP.txt" 2>/dev/null || echo "No crontab found"

# Backup systemd services
echo "Backing up systemd services..."
systemctl list-unit-files --state=enabled > "$BACKUP_DIR/systemd-services-$TIMESTAMP.txt" 2>/dev/null || echo "No systemd services found"

# Backup current working directory
echo "Backing up current projects..."
if [[ -n "${PWD:-}" ]]; then
    backup_item "$PWD" "$BACKUP_DIR/current-project-$TIMESTAMP.tar.gz" "Current project directory"
fi

# Backup SSH keys and git config
echo "Backing up SSH and Git configuration..."
backup_item "$HOME/.ssh" "$BACKUP_DIR/ssh-config-$TIMESTAMP.tar.gz" "SSH configuration"
backup_item "$HOME/.gitconfig" "$BACKUP_DIR/gitconfig-$TIMESTAMP" "Git configuration"

# Backup environment variables
echo "Backing up environment variables..."
env > "$BACKUP_DIR/environment-$TIMESTAMP.txt" 2>/dev/null || echo "Could not backup environment"

# Backup process information
echo "Backing up process information..."
ps aux > "$BACKUP_DIR/processes-$TIMESTAMP.txt" 2>/dev/null || echo "Could not backup processes"
df -h > "$BACKUP_DIR/disk-usage-$TIMESTAMP.txt" 2>/dev/null || echo "Could not backup disk usage"

# Create system info
echo "Collecting system information..."
cat > "$BACKUP_DIR/system-info-$TIMESTAMP.txt" << EOF
System Information
==================
OS: $(lsb_release -d -s 2>/dev/null || echo "Unknown")
Kernel: $(uname -r)
Architecture: $(uname -m)
Memory: $(free -h | grep Mem | awk '{print $2}') total, $(free -h | grep Mem | awk '{print $3}') used
Disk: $(df -h / | tail -1 | awk '{print $2}') total, $(df -h / | tail -1 | awk '{print $3}') used
CPU: $(nproc) cores
Load Average: $(uptime | awk -F'load average:' '{print $2}')
EOF

# Update manifest
cat >> "$BACKUP_DIR/manifest-$TIMESTAMP.txt" << EOF

Backup Files:
- clawdbot-config-$TIMESTAMP.tar.gz: Clawdbot configuration
- clawd-data-$TIMESTAMP.tar.gz: Clawdbot workspace data
- bashrc-$TIMESTAMP: Bash configuration
- profile-$TIMESTAMP: Profile configuration
- npm-global-packages-$TIMESTAMP.txt: NPM global packages
- pip-packages-$TIMESTAMP.txt: Python packages
- system-packages-$TIMESTAMP.txt: System packages
- crontab-$TIMESTAMP.txt: Cron jobs
- systemd-services-$TIMESTAMP.txt: Systemd services
- current-project-$TIMESTAMP.tar.gz: Current project files
- ssh-config-$TIMESTAMP.tar.gz: SSH configuration
- gitconfig-$TIMESTAMP: Git configuration
- environment-$TIMESTAMP.txt: Environment variables
- processes-$TIMESTAMP.txt: Running processes
- disk-usage-$TIMESTAMP.txt: Disk usage
- system-info-$TIMESTAMP.txt: System information
EOF

# Create summary
cat > "$BACKUP_DIR/summary-$TIMESTAMP.txt" << EOF
Backup Summary
==============
Date: $(date)
Hostname: $(hostname)
User: $(whoami)

Files Backed Up: $(ls "$BACKUP_DIR"/*-$TIMESTAMP* 2>/dev/null | wc -l)
Total Size: $(du -h "$BACKUP_DIR" | tail -1 | awk '{print $1}')
Backup Duration: $(($(date +%s) - $(date -d "$TIMESTAMP" +%s 2>/dev/null || echo 0))) seconds

Status: COMPLETED
EOF

# Calculate total size
TOTAL_SIZE=$(du -sh "$BACKUP_DIR" | awk '{print $1}')
FILE_COUNT=$(ls "$BACKUP_DIR"/*-$TIMESTAMP* 2>/dev/null | wc -l)

echo ""
echo -e "${GREEN}✅ Backup completed successfully!${NC}"
echo "Backup location: $BACKUP_DIR"
echo "Total size: $TOTAL_SIZE"
echo "Files backed up: $FILE_COUNT"
echo "Manifest: $BACKUP_DIR/manifest-$TIMESTAMP.txt"
echo "Summary: $BACKUP_DIR/summary-$TIMESTAMP.txt"
echo ""
echo -e "${BLUE}Next steps:${NC}"
echo "1. Copy backup files to safe location (external drive/cloud)"
echo "2. Verify backup integrity"
echo "3. Proceed with Jetson migration"
echo ""
echo -e "${YELLOW}⚠️  Important:${NC}"
echo "- Keep this backup safe until migration is complete"
echo "- Test restoration before proceeding with migration"
echo "- Backup contains sensitive data (API keys, etc.)"

# Create a single archive for easy transfer
echo "Creating single archive for transfer..."
cd "$BACKUP_DIR"
tar -czf "billybot-complete-backup-$TIMESTAMP.tar.gz" *-$TIMESTAMP*
echo -e "${GREEN}✅ Complete backup archive created: billybot-complete-backup-$TIMESTAMP.tar.gz${NC}"

echo ""
echo -e "${GREEN}🎉 System backup complete! Ready for migration.${NC}"