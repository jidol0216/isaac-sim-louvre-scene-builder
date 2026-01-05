#!/bin/bash

# Louvre Extension Isaac Sim Launcher
# Quick launch script for Isaac Sim with Louvre Scene Builder extension

# 스크립트 위치 기준으로 경로 설정 (다른 PC에서도 동작)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXTENSION_PATH="$SCRIPT_DIR/exts"

# Isaac Sim 경로 (사용자 환경에 맞게 수정 필요)
ISAAC_SIM_PATH="${ISAAC_SIM_PATH:-$HOME/isaacsim}"

echo "🚀 Launching Isaac Sim with Louvre Scene Builder..."
echo "   Extension Path: $EXTENSION_PATH"
echo ""

cd "$ISAAC_SIM_PATH" && ./isaac-sim.sh --ext-folder "$EXTENSION_PATH"
