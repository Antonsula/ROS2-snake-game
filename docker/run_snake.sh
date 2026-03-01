#!/usr/bin/env bash
set -euo pipefail

declare -a PIDS=()

cleanup() {
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done

  for pid in "${PIDS[@]:-}"; do
    wait "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

ros2 run ros2_snake snake_game &
GAME_PID=$!

# Give the game node time to initialize topics before renderer starts.
sleep 1
ros2 run ros2_snake controller &
CONTROLLER_PID=$!

PIDS=("$GAME_PID" "$CONTROLLER_PID")
ros2 run ros2_snake renderer
