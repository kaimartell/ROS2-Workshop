#!/usr/bin/env bash
set -euo pipefail

if ! command -v docker >/dev/null 2>&1; then
  echo "ERROR: docker CLI is not installed or not in PATH."
  exit 1
fi

if ! docker info >/dev/null 2>&1; then
  echo "ERROR: Docker Desktop is not running. Start Docker and try again."
  exit 1
fi

echo "Docker is running."

if command -v curl >/dev/null 2>&1; then
  if curl -fsS http://localhost:8000/health >/dev/null 2>&1; then
    echo "Host agent reachable at http://localhost:8000/health"
  else
    echo "WARNING: Host agent is not reachable at http://localhost:8000/health"
    echo "         Start it with: cd host_agent && python3 -m host_agent --port 8000 --backend mock"
  fi
else
  echo "WARNING: curl not found; skipping host agent health check."
fi
