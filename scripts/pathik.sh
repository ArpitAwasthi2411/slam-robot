#!/usr/bin/env bash
# Pathik on the Pi with Docker: install, update (with automatic rollback), rollback, status, logs.
#
#   bash scripts/pathik.sh install        once: Docker, ~/pathik folder, stop the old robot.service
#   bash ~/pathik/pathik.sh update        pull the newest image, restart, roll back by itself if it is unhealthy
#   bash ~/pathik/pathik.sh rollback      go back to the version that ran before the last update
#   bash ~/pathik/pathik.sh use sha-1a2b3c4   run one exact version (tags: latest, sha-xxxxxxx, v1.2.0)
#   bash ~/pathik/pathik.sh status | logs | stop | start
#   bash ~/pathik/pathik.sh users list    accounts (also: users reset-admin <name> '<password>')
set -euo pipefail
DIR="${PATHIK_DIR:-$HOME/pathik}"
NAME=pathik
here="$(cd "$(dirname "$0")" && pwd)"
compose() { docker compose --project-directory "$DIR" -f "$DIR/docker-compose.yml" "$@"; }
say() { echo "[pathik] $*"; }

current_version() { docker inspect -f '{{index .Config.Labels "org.opencontainers.image.version"}}' "$NAME" 2>/dev/null || true; }
set_tag() {
  touch "$DIR/.env"
  grep -v '^PATHIK_TAG=' "$DIR/.env" > "$DIR/.env.tmp" || true
  echo "PATHIK_TAG=$1" >> "$DIR/.env.tmp"
  mv "$DIR/.env.tmp" "$DIR/.env"
}
wait_healthy() {           # the image's HEALTHCHECK asks the app server /api/ping
  local t=0 s
  while [ $t -lt 150 ]; do
    s=$(docker inspect -f '{{.State.Health.Status}}' "$NAME" 2>/dev/null || echo missing)
    [ "$s" = healthy ] && return 0
    [ "$s" = unhealthy ] && return 1
    sleep 5; t=$((t + 5)); printf '.'
  done
  return 1
}

case "${1:-status}" in
  install)
    if ! command -v docker >/dev/null; then
      say "installing Docker (needs internet on the Pi)…"
      curl -fsSL https://get.docker.com | sudo sh
      sudo usermod -aG docker "$USER"
      say "log out and back in once (docker group), then run this again"; exit 0
    fi
    mkdir -p "$DIR" "$HOME/maps"
    cp "$here/../deploy/pi/docker-compose.yml" "$DIR/"
    cp "$here/pathik.sh" "$DIR/"
    [ -f "$DIR/robot.env" ] || cp "$here/../deploy/pi/robot.env.example" "$DIR/robot.env"
    [ -f "$DIR/.env" ] || echo "PATHIK_TAG=latest" > "$DIR/.env"
    sed -i "s#/home/arpit/maps#$HOME/maps#" "$DIR/docker-compose.yml"
    if systemctl is-enabled --quiet robot 2>/dev/null; then
      sudo systemctl disable --now robot && say "old robot.service stopped (Docker starts the robot now)"
    fi
    say "edit $DIR/robot.env (ports, GROQ key), then: bash $DIR/pathik.sh update"
    ;;
  update)
    prev=$(current_version)
    set_tag latest
    say "pulling the newest image…"
    compose pull
    compose up -d
    say "waiting for the robot to report healthy"
    if wait_healthy; then
      echo; say "running $(current_version) (was ${prev:-nothing})"
      [ -n "$prev" ] && echo "$prev" > "$DIR/.previous"
      docker image prune -f >/dev/null
    else
      echo; say "NOT healthy: rolling back to ${prev:-?}"
      compose logs --tail 40 || true
      if [ -n "$prev" ] && [ "$prev" != dev ]; then set_tag "$prev"; compose up -d; fi
      exit 1
    fi
    ;;
  rollback)
    prev=$(cat "$DIR/.previous" 2>/dev/null || true)
    [ -n "$prev" ] || { say "no previous version recorded"; exit 1; }
    set_tag "$prev"; compose up -d
    say "rolled back to $prev  (back to the newest: pathik.sh update)"
    ;;
  use)
    [ -n "${2:-}" ] || { say "usage: pathik.sh use <tag>"; exit 1; }
    set_tag "$2"; compose pull; compose up -d; say "running $2"
    ;;
  status)
    docker ps --filter "name=$NAME" --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}'
    say "version: $(current_version)"
    ;;
  logs) compose logs -f --tail 100 ;;
  stop) compose stop ;;
  start) compose up -d ;;
  users) shift; docker exec -it "$NAME" /entrypoint.sh users "$@" ;;
  *) sed -n '2,10p' "$0"; exit 1 ;;
esac
