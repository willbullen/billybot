# BillyBot Migration Guide

Complete step-by-step guide for migrating BillyBot from your current system to NVIDIA Jetson Orin NX.

## 📋 Table of Contents

1. [Pre-Migration Preparation](#pre-migration-preparation)
2. [System Requirements](#system-requirements)
3. [Migration Steps](#migration-steps)
4. [Post-Migration Testing](#post-migration-testing)
5. [Troubleshooting](#troubleshooting)
6. [Rollback Procedure](#rollback-procedure)

## 🎯 Pre-Migration Preparation

### On Current System (DEV-UBUNTU-HOME)

#### Step 1: Run Full Backup
```bash
cd ~/billybot-migration
./scripts/backup-current-system.sh
```

This will create a timestamped backup containing:
- Clawdbot configuration (`~/.clawdbot/`)
- Workspace data (`~/clawd/`)
- Environment files (`.bashrc`, `.profile`)
- Installed packages (npm, pip, apt)
- SSH keys and Git config
- Cron jobs and systemd services
- System information

**Expected output:**
```
✅ Backup completed successfully!
Backup location: /home/william/billybot-migration/backups
Total size: ~500MB
Files backed up: 15+
```

#### Step 2: Verify Backup Integrity
```bash
# Check backup exists
ls -lh ~/billybot-migration/backups/

# Verify archives can be extracted
tar -tzf ~/billybot-migration/backups/clawd-data-*.tar.gz | head
tar -tzf ~/billybot-migration/backups/clawdbot-config-*.tar.gz | head
```

#### Step 3: Export Credentials Securely
```bash
# Create encrypted credentials archive
cd ~/clawd/env
tar -czf ~/credentials-backup.tar.gz *
gpg --symmetric --cipher-algo AES256 ~/credentials-backup.tar.gz
rm ~/credentials-backup.tar.gz

# Result: credentials-backup.tar.gz.gpg (encrypted)
```

**⚠️ Important:** Store this encrypted file separately (USB drive, password manager)

#### Step 4: Document Current State
```bash
# System info
clawdbot status > ~/billybot-current-status.txt
docker ps -a >> ~/billybot-current-status.txt
systemctl list-unit-files --state=enabled >> ~/billybot-current-status.txt

# Running services
ps aux | grep -E "clawdbot|node|python" >> ~/billybot-current-status.txt
```

### On Jetson Orin NX (Target System)

#### Step 1: Install JetPack OS
1. Download latest JetPack from NVIDIA: https://developer.nvidia.com/jetpack
2. Flash Jetson using SDK Manager or SD card image
3. Complete initial Ubuntu setup

**Recommended settings:**
- User: `william`
- Hostname: `jetson-billybot`
- Timezone: `Europe/Dublin`
- Enable SSH server

#### Step 2: Initial System Update
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y build-essential git curl wget
```

#### Step 3: Transfer Backup Files
```bash
# From current system, copy to Jetson:
scp ~/billybot-migration/backups/billybot-complete-backup-*.tar.gz william@jetson-billybot:~/
scp ~/credentials-backup.tar.gz.gpg william@jetson-billybot:~/

# Or use USB drive:
# 1. Copy files to USB on current system
# 2. Insert USB into Jetson
# 3. Mount and copy files
```

## 🔧 System Requirements

### Hardware (Jetson Orin NX)
- ✅ **CPU:** NVIDIA Jetson Orin NX (Cortex-A78AE)
- ✅ **RAM:** 8GB minimum (16GB recommended)
- ✅ **Storage:** 64GB minimum (128GB+ recommended)
- ✅ **Network:** Ethernet or WiFi with internet access
- ✅ **Power:** Official NVIDIA power supply

### Software
- ✅ **OS:** Ubuntu 20.04 or 22.04 (via JetPack)
- ✅ **JetPack:** Version 5.x or 6.x
- ✅ **Docker:** 20.10+
- ✅ **Node.js:** 18.x LTS or newer
- ✅ **Python:** 3.10+

### Network Requirements
- ✅ Internet access for package downloads
- ✅ Access to GitHub, npm registry, PyPI
- ✅ Telegram API accessible
- ✅ Your custom API endpoints accessible

## 🚀 Migration Steps

### Phase 1: Jetson Setup (30-45 minutes)

#### Step 1: Clone Migration Repository
```bash
git clone https://github.com/willbullen/billybot.git ~/billybot-migration
cd ~/billybot-migration
```

#### Step 2: Run Migration Script
```bash
chmod +x scripts/migrate-to-jetson.sh
./scripts/migrate-to-jetson.sh
```

The script will:
1. ✅ Check Jetson hardware
2. ✅ Verify system requirements
3. ✅ Install system dependencies (apt packages)
4. ✅ Install Node.js and npm
5. ✅ Install Python and pip
6. ✅ Install Docker and Docker Compose
7. ✅ Install Clawdbot globally
8. ✅ Create workspace directory
9. ✅ Configure systemd services
10. ✅ Run installation tests

**Expected duration:** 20-30 minutes (depending on internet speed)

#### Step 3: Restore Backup Data
```bash
# Extract complete backup
cd ~
tar -xzf billybot-complete-backup-*.tar.gz

# Restore Clawdbot configuration
tar -xzf clawdbot-config-*.tar.gz -C ~/

# Restore workspace data
tar -xzf clawd-data-*.tar.gz -C ~/

# Restore credentials (decrypt first)
gpg --decrypt credentials-backup.tar.gz.gpg > credentials-backup.tar.gz
tar -xzf credentials-backup.tar.gz -C ~/clawd/env/
rm credentials-backup.tar.gz
```

#### Step 4: Configure Clawdbot for Jetson
```bash
# Edit configuration
nano ~/.clawdbot/clawdbot.json

# Update settings:
# - Verify API keys are correct
# - Check workspace path: /home/william/clawd
# - Adjust memory limits for Jetson (6-8GB max)
# - Enable Jetson optimizations
```

**Key settings to verify:**
```json
{
  "agents": {
    "defaults": {
      "maxConcurrent": 2,  // Lower for Jetson
      "subagents": {
        "maxConcurrent": 4   // Lower for Jetson
      }
    }
  },
  "jetson": {
    "enabled": true,
    "limits": {
      "max_memory_gb": 6
    }
  }
}
```

### Phase 2: Service Configuration (10-15 minutes)

#### Step 1: Enable Systemd Services
```bash
# Enable Clawdbot service
sudo systemctl enable clawdbot.service

# Start service
sudo systemctl start clawdbot.service

# Check status
sudo systemctl status clawdbot.service
```

#### Step 2: Restore Cron Jobs
```bash
# Edit crontab
crontab -e

# Or restore from backup
crontab ~/crontab-*.txt

# Verify
crontab -l
```

#### Step 3: Configure Docker
```bash
# Add user to docker group (if not already done)
sudo usermod -aG docker $USER

# Restart docker service
sudo systemctl restart docker

# Test docker
docker run hello-world
```

### Phase 3: Testing & Validation (15-20 minutes)

#### Step 1: Test Clawdbot Core
```bash
# Check version
clawdbot --version

# Check status
clawdbot status

# Test gateway
curl http://localhost:18789/
```

#### Step 2: Test Integrations
```bash
# Test Telegram
# Send a message to BillyBot via Telegram
# Expected: Response within a few seconds

# Test API endpoints
curl -H "Authorization: Bearer baf3..." http://localhost:18789/api/status

# Test skills
clawdbot skills list
```

#### Step 3: Run Full Test Suite
```bash
cd ~/billybot-migration
./scripts/test-installation.sh
```

**Expected results:**
- ✅ Clawdbot CLI accessible
- ✅ Python environment exists
- ✅ Docker installed and working
- ✅ Services running
- ✅ Telegram bot responding
- ✅ Memory/CPU usage within limits

### Phase 4: Performance Optimization (10-15 minutes)

#### Step 1: Monitor Resource Usage
```bash
# Install Jetson stats
sudo -H pip install -U jetson-stats

# Monitor Jetson performance
jtop

# Check temperature
cat /sys/class/thermal/thermal_zone0/temp
```

#### Step 2: Optimize Settings
```bash
# Set power mode (max performance)
sudo nvpmodel -m 0
sudo jetson_clocks

# Or balanced mode for lower temps
sudo nvpmodel -m 2
```

#### Step 3: Test Under Load
```bash
# Send multiple requests to BillyBot
# Monitor CPU, memory, temperature
# Adjust limits if needed
```

## ✅ Post-Migration Testing

### Functional Tests

#### 1. Telegram Communication
```bash
# Test commands via Telegram:
- Send "hello" → Should get response
- Send "/status" → Should show system status
- Test file uploads
- Test voice messages (if configured)
```

#### 2. Automation Tasks
```bash
# Check cron jobs
systemctl list-timers

# Manually trigger a job
clawdbot cron run --id <job-id>

# Verify meter.ie scraper
cd ~/projects/meter-ie-scraper
./run.sh
```

#### 3. External APIs
```bash
# Test Met Éireann buoys
python ~/projects/scripts/met_buoys/buoy_monitor.py

# Test ClickUp
# (Fix API token first if needed)

# Test GitHub
gh auth status
gh repo list
```

### Performance Tests

#### 1. Response Time
```bash
# Measure response latency
time clawdbot status

# Expected: < 2 seconds
```

#### 2. Memory Usage
```bash
# Check memory after 1 hour of operation
free -h
ps aux | grep clawdbot

# Expected: < 4GB for main process
```

#### 3. Temperature
```bash
# Monitor temperature under load
watch -n 1 cat /sys/class/thermal/thermal_zone0/temp

# Expected: < 75°C normal, < 85°C under load
```

## 🔧 Troubleshooting

### Common Issues

#### Issue 1: High Memory Usage
**Symptoms:** System slow, OOM errors
**Solution:**
```bash
# Reduce concurrent sessions
nano ~/.clawdbot/clawdbot.json
# Set maxConcurrent: 1

# Clear cache
clawdbot cache clear

# Restart service
sudo systemctl restart clawdbot.service
```

#### Issue 2: Overheating
**Symptoms:** Thermal throttling, slow performance
**Solution:**
```bash
# Check temperature
jtop

# Add cooling
# - External fan recommended
# - Ensure good airflow

# Reduce power mode
sudo nvpmodel -m 2
```

#### Issue 3: Network Issues
**Symptoms:** API timeouts, connection errors
**Solution:**
```bash
# Check network
ping google.com
ping api.telegram.org

# Restart network
sudo systemctl restart NetworkManager

# Check firewall
sudo ufw status
```

#### Issue 4: Service Won't Start
**Symptoms:** systemctl fails, errors in journal
**Solution:**
```bash
# Check logs
sudo journalctl -u clawdbot -n 50

# Check configuration
clawdbot doctor

# Restart with verbose logging
sudo systemctl stop clawdbot
clawdbot gateway start --verbose
```

## 🔄 Rollback Procedure

If migration fails, restore to current system:

### Step 1: Stop Services on Jetson
```bash
sudo systemctl stop clawdbot.service
sudo systemctl disable clawdbot.service
```

### Step 2: Re-enable Current System
```bash
# On DEV-UBUNTU-HOME
sudo systemctl start clawdbot.service
clawdbot status
```

### Step 3: Verify Current System
```bash
# Test Telegram
# Send message → Should respond

# Check services
systemctl status clawdbot
crontab -l
```

### Step 4: Document Issues
```bash
# Save logs from Jetson
sudo journalctl -u clawdbot > ~/jetson-migration-logs.txt

# Document what failed
# Report issues to troubleshooting guide
```

## 📊 Success Criteria

Migration is successful when:

- ✅ Clawdbot responds to Telegram messages
- ✅ All cron jobs running on schedule
- ✅ Memory usage < 6GB sustained
- ✅ Temperature < 80°C sustained
- ✅ All integrations working (GitHub, ClickUp, etc.)
- ✅ Response time < 3 seconds
- ✅ No errors in systemd logs
- ✅ Backup can be restored if needed

## 🎉 Next Steps After Migration

1. **Monitor for 24 hours** - Watch performance, errors
2. **Optimize settings** - Tune based on actual usage
3. **Update documentation** - Record any changes made
4. **Backup Jetson** - Create new baseline backup
5. **Decommission old system** - After 1 week of stable operation

---

**Need help?** Check the troubleshooting guide or open an issue on GitHub.