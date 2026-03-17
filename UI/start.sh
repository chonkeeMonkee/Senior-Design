#!/bin/bash
# ── ESP32 Submarine Dashboard Launcher ────────────────────────────────────────
# Usage: double-click in Finder, or run ./start.sh in terminal.
# Starts a local HTTP server and opens the dashboard automatically.

PORT=8080
DIR="$(cd "$(dirname "$0")" && pwd)"

echo "Starting ESP32 Dashboard on http://localhost:${PORT} ..."

# Kill any existing server on the same port
lsof -ti tcp:${PORT} | xargs kill -9 2>/dev/null

# Start the Python HTTP server in the background
python3 -m http.server ${PORT} --directory "${DIR}" &
SERVER_PID=$!

# Wait a moment for the server to start, then open the browser
sleep 0.8
open -a "Google Chrome" "http://localhost:${PORT}"

echo "Server PID: ${SERVER_PID}. Press Ctrl+C to stop."
wait ${SERVER_PID}
