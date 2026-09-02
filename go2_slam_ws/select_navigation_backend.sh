#!/bin/bash
# Fail-closed backend handover through the Web safety API.
set -euo pipefail

BACKEND="${1:-}"
WEB_URL="${GO2_WEB_URL:-http://127.0.0.1:8890}"
if [ "$BACKEND" != "scan" ] && [ "$BACKEND" != "nav2" ]; then
    echo "用法: $0 scan|nav2" >&2
    exit 2
fi

curl -fsS --max-time 5 -H 'Content-Type: application/json' \
    --data "{\"backend\":\"$BACKEND\"}" \
    "$WEB_URL/api/navigation/backend"
echo
