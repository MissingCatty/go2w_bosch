#!/bin/bash
set +e

UNIT="go2-remembr-vlm.service"
systemctl --user stop "$UNIT" >/dev/null 2>&1
systemctl --user reset-failed "$UNIT" >/dev/null 2>&1
echo "Qwen3.5 VLM 已停止"
