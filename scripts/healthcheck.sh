#!/usr/bin/env bash
# BillyBot System Health Check
#
# Quick health check for all BillyBot services.
# Returns JSON-formatted status for each container.
#
# Usage:
#   ./scripts/healthcheck.sh          # Human-readable output
#   ./scripts/healthcheck.sh --json   # JSON output (for monitoring)

set -euo pipefail

JSON_MODE=false
[[ "${1:-}" == "--json" ]] && JSON_MODE=true

# Colors (only for non-JSON mode)
RED='\033[0;31m'
GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[0;33m'
NC='\033[0m'

CONTAINERS=("billybot-redis" "billybot-ros2" "billybot-nanobot" "billybot-dashboard")
ALL_OK=true
RESULTS=()

for container in "${CONTAINERS[@]}"; do
  state=$(docker inspect --format='{{.State.Status}}' "$container" 2>/dev/null || echo "not_found")
  health=$(docker inspect --format='{{.State.Health.Status}}' "$container" 2>/dev/null || echo "none")
  uptime=$(docker inspect --format='{{.State.StartedAt}}' "$container" 2>/dev/null || echo "")

  if $JSON_MODE; then
    RESULTS+=("{\"container\":\"${container}\",\"state\":\"${state}\",\"health\":\"${health}\"}")
  else
    if [[ "$state" == "running" ]]; then
      if [[ "$health" == "healthy" ]]; then
        echo -e "${GREEN}[OK]${NC}  ${container}: running (healthy)"
      elif [[ "$health" == "none" ]]; then
        echo -e "${YELLOW}[--]${NC}  ${container}: running (no healthcheck)"
      else
        echo -e "${YELLOW}[!!]${NC}  ${container}: running (${health})"
        ALL_OK=false
      fi
    else
      echo -e "${RED}[FAIL]${NC} ${container}: ${state}"
      ALL_OK=false
    fi
  fi
done

# Dashboard API check
if curl -sf http://localhost:8000/api/health/ >/dev/null 2>&1; then
  DASHBOARD_API="ok"
else
  DASHBOARD_API="unreachable"
  ALL_OK=false
fi

if $JSON_MODE; then
  echo "{\"containers\":[$(IFS=,; echo "${RESULTS[*]}")],\"dashboard_api\":\"${DASHBOARD_API}\",\"overall\":\"$(if $ALL_OK; then echo healthy; else echo degraded; fi)\"}"
else
  echo ""
  if $ALL_OK; then
    echo -e "${GREEN}Overall: HEALTHY${NC}"
  else
    echo -e "${YELLOW}Overall: DEGRADED${NC}"
  fi
fi

$ALL_OK
