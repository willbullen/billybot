#!/usr/bin/env bash
# BillyBot OTA Deployment Script
#
# Usage:
#   ./scripts/deploy.sh                    # Build + restart all
#   ./scripts/deploy.sh --service ros2-byc # Build + restart one service
#   ./scripts/deploy.sh --pull             # Pull pre-built images + restart
#   ./scripts/deploy.sh --rollback         # Rollback to previous images
#
# Environment:
#   BILLYBOT_REGISTRY  - Container registry (default: ghcr.io/willbullen/billybot)
#   BILLYBOT_TAG       - Image tag (default: latest)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
REGISTRY="${BILLYBOT_REGISTRY:-ghcr.io/willbullen/billybot}"
TAG="${BILLYBOT_TAG:-latest}"
COMPOSE="docker compose -f ${PROJECT_DIR}/docker-compose.yml"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
CYAN='\033[0;36m'
NC='\033[0m'

log()  { echo -e "${CYAN}[deploy]${NC} $*"; }
ok()   { echo -e "${GREEN}[deploy]${NC} $*"; }
fail() { echo -e "${RED}[deploy]${NC} $*" >&2; }

# Parse arguments
SERVICE=""
MODE="build"  # build | pull | rollback

while [[ $# -gt 0 ]]; do
  case $1 in
    --service)  SERVICE="$2"; shift 2 ;;
    --pull)     MODE="pull"; shift ;;
    --rollback) MODE="rollback"; shift ;;
    --tag)      TAG="$2"; shift 2 ;;
    *)          fail "Unknown option: $1"; exit 1 ;;
  esac
done

# Health check function
check_health() {
  local container=$1
  local max_wait=${2:-60}
  local waited=0
  log "Waiting for ${container} to become healthy (max ${max_wait}s)..."
  while [[ $waited -lt $max_wait ]]; do
    status=$(docker inspect --format='{{.State.Health.Status}}' "$container" 2>/dev/null || echo "unknown")
    if [[ "$status" == "healthy" ]]; then
      ok "${container} is healthy"
      return 0
    fi
    sleep 2
    waited=$((waited + 2))
  done
  fail "${container} did not become healthy within ${max_wait}s (status: ${status})"
  return 1
}

case "$MODE" in
  build)
    log "Building and deploying BillyBot..."

    # Tag current images for rollback
    log "Tagging current images for rollback..."
    for svc in ros2-byc nanobot dashboard; do
      img="billybot-${svc}"
      if docker image inspect "${img}:latest" &>/dev/null; then
        docker tag "${img}:latest" "${img}:rollback" 2>/dev/null || true
      fi
    done

    if [[ -n "$SERVICE" ]]; then
      log "Building ${SERVICE}..."
      $COMPOSE build "$SERVICE"
      log "Restarting ${SERVICE}..."
      $COMPOSE up -d "$SERVICE"
    else
      log "Building all services..."
      $COMPOSE build
      log "Restarting all services..."
      $COMPOSE up -d
    fi
    ;;

  pull)
    log "Pulling pre-built images (tag: ${TAG})..."
    for svc in ros2-byc nanobot dashboard; do
      img="${REGISTRY}/billybot-${svc}:${TAG}"
      log "Pulling ${img}..."
      docker pull "$img"
      docker tag "$img" "billybot-${svc}:latest"
    done
    log "Restarting services..."
    $COMPOSE up -d
    ;;

  rollback)
    log "Rolling back to previous images..."
    for svc in ros2-byc nanobot dashboard; do
      img="billybot-${svc}"
      if docker image inspect "${img}:rollback" &>/dev/null; then
        docker tag "${img}:rollback" "${img}:latest"
        ok "Rolled back ${img}"
      else
        fail "No rollback image for ${img}"
      fi
    done
    log "Restarting services..."
    $COMPOSE up -d
    ;;
esac

# Wait for health checks
log "Verifying deployment health..."
HEALTHY=true
for container in billybot-redis billybot-dashboard billybot-ros2 billybot-nanobot; do
  if [[ -n "$SERVICE" ]] && [[ "$container" != "billybot-${SERVICE}" ]] && [[ "$container" != "billybot-redis" ]]; then
    continue
  fi
  if ! check_health "$container" 60; then
    HEALTHY=false
  fi
done

if $HEALTHY; then
  ok "Deployment complete - all services healthy"
else
  fail "Deployment complete but some services unhealthy"
  fail "Run: ./scripts/deploy.sh --rollback"
  exit 1
fi
