# Quick Reference for BillyBot Jetson Migration

## 🚀 One-Command Migration

```bash
# 1. On current system (backup first)
git clone https://github.com/willbullen/billybot.git
cd billybot
./scripts/backup-current-system.sh

# 2. Transfer to Jetson
cp backups/billybot-complete-backup-*.tar.gz /path/to/jetson/

# 3. On Jetson (full migration)
git clone https://github.com/willbullen/billybot.git
cd billybot
./scripts/migrate-to-jetson.sh

# 4. Test
clawdbot status
curl http://localhost:18789/
```

## 📋 What Gets Migrated

**System Components:**
- Clawdbot core installation
- All configurations and API keys (from backup)
- Workspace data (`~/clawd/`)
- Environment variables and credentials
- Custom scripts and automations
- Cron jobs and systemd services
- SSH keys and Git configuration

**Integrations (all working):**
- Telegram bot (unchanged)
- GitHub API access
- ClickUp task management
- Met Éireann buoy monitoring
- Meter.ie scraper (water data)
- All AI model APIs (Claude, Kimi, etc.)

## ⚙️ Jetson Optimizations

**Memory Management:**
- Limited to 6GB (leaving 2GB for system)
- Reduced concurrent sessions (2 instead of 4)
- Smart caching and pruning

**Performance Features:**
- GPU acceleration where possible
- Thermal management (< 85°C)
- Power optimization modes
- Resource monitoring

**Auto-management:**
- Auto-start on boot via systemd
- Automatic restart on failure
- Health monitoring
- Log rotation

## 📊 Migration Timeline

- **Prep & Backup:** 15-20 minutes
- **Jetson Setup:** 30-45 minutes  
- **Testing:** 15-20 minutes
- **Total:** ~1-1.5 hours

## 🔧 Key Files

- `scripts/migrate-to-jetson.sh` - Main migration script
- `scripts/backup-current-system.sh` - System backup
- `config/clawdbot.json.template` - Configuration template
- `config/systemd/clawdbot.service` - Service file
- `docs/migration-guide.md` - Detailed guide

## 🎯 Next Steps

1. **Run backup** on current system
2. **Install JetPack** on Orin NX
3. **Transfer backup** to Jetson
4. **Run migration** script
5. **Test thoroughly**
6. **Monitor for 24h**

## 📞 Support

- **Full guide:** `docs/migration-guide.md`
- **GitHub repo:** https://github.com/willbullen/billybot
- **Issues:** Create GitHub issue for problems