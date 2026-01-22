#!/usr/bin/env bash
set -euo pipefail

# Run this from the repo root

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$SCRIPT_DIR"
PARENT_DIR="$(cd "$REPO_ROOT/.." && pwd)"
AUTOWARE_DIR="$PARENT_DIR/autoware"

echo "==> Team repo root: $REPO_ROOT"

echo "==> Creating local folders: maps/ testing/"
mkdir -p "$REPO_ROOT/maps" "$REPO_ROOT/testing"

cd "$PARENT_DIR"
echo "==> Parent dir: $PARENT_DIR"

if [[ -d "$AUTOWARE_DIR/.git" ]]; then
  echo "==> Autoware repo already exists: $AUTOWARE_DIR (skipping clone)"
else
  echo "==> Cloning Autoware into: $AUTOWARE_DIR"
  git clone https://github.com/autowarefoundation/autoware.git
fi


cd "$AUTOWARE_DIR"
echo "==> Running Autoware setup (no-nvidia, docker)"
./setup-dev-env.sh -y --no-nvidia docker

echo
echo "✅ Setup complete."
echo "Next: go back to your repo and run:"
echo "  ./dev_run.sh"
