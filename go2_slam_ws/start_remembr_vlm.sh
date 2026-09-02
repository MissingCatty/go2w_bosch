#!/bin/bash
# Start the local Qwen3.5 VLM as an isolated, resource-bounded user service.
set -euo pipefail

WS="$(cd "$(dirname "$0")" && pwd)"
SERVER="${GO2_REMEMBR_VLM_SERVER:-$WS/third_party/llama.cpp/build-jetson/bin/llama-server}"
MODEL="${GO2_REMEMBR_VLM_MODEL:-$WS/models/Qwen3.5-2B-GGUF/Qwen3.5-2B-Q8_0.gguf}"
MMPROJ="${GO2_REMEMBR_MMPROJ:-$WS/models/Qwen3.5-2B-GGUF/mmproj-Q8_0.gguf}"
MODEL_ALIAS="${GO2_REMEMBR_VLM_ALIAS:-qwen3.5-2b-int8}"
UNIT="go2-remembr-vlm.service"
PORT="${GO2_REMEMBR_VLM_PORT:-8000}"
HEALTH_URL="http://127.0.0.1:${PORT}/health"
LOG="/tmp/go2_remembr_vlm_${UID:-$(id -u)}.log"
IMAGE_MAX_TOKENS="${GO2_REMEMBR_IMAGE_MAX_TOKENS:-128}"
FLASH_ATTN="${GO2_REMEMBR_FLASH_ATTN:-off}"
BATCH_SIZE="${GO2_REMEMBR_BATCH_SIZE:-128}"
UBATCH_SIZE="${GO2_REMEMBR_UBATCH_SIZE:-64}"

model_is_ready() {
    curl -fsS --max-time 1 "$HEALTH_URL" >/dev/null 2>&1 &&
        curl -fsS --max-time 1 "http://127.0.0.1:${PORT}/v1/models" 2>/dev/null |
            grep -Fq "\"id\":\"$MODEL_ALIAS\""
}

for path in "$SERVER" "$MODEL" "$MMPROJ"; do
    if [ ! -f "$path" ]; then
        echo "错误: 缺少 $path" >&2
        exit 1
    fi
done

if model_is_ready; then
    echo "Qwen3.5 VLM 已就绪: $HEALTH_URL ($MODEL_ALIAS)"
    exit 0
fi

systemctl --user stop "$UNIT" >/dev/null 2>&1 || true
systemctl --user reset-failed "$UNIT" >/dev/null 2>&1 || true
: >"$LOG"

echo "==> 启动 Qwen3.5-2B Q8_0（单并发 / 2K context）"
systemd-run --user --unit=go2-remembr-vlm --collect \
    --property=Restart=on-failure \
    --property=RestartSec=5 \
    --property=TimeoutStopSec=20 \
    --property=Nice=10 \
    --property=OOMScoreAdjust=500 \
    --property=CPUQuota=300% \
    --property=MemoryHigh=7G \
    --property=MemoryMax=9G \
    --property=IOSchedulingClass=idle \
    --property="StandardOutput=append:$LOG" \
    --property="StandardError=append:$LOG" \
    "$SERVER" \
        --model "$MODEL" \
        --mmproj "$MMPROJ" \
        --alias "$MODEL_ALIAS" \
        --host 127.0.0.1 \
        --port "$PORT" \
        --ctx-size 2048 \
        --parallel 1 \
        --gpu-layers all \
        --cache-type-k q8_0 \
        --cache-type-v f16 \
        --threads 4 \
        --threads-batch 4 \
        --batch-size "$BATCH_SIZE" \
        --ubatch-size "$UBATCH_SIZE" \
        --flash-attn "$FLASH_ATTN" \
        --image-max-tokens "$IMAGE_MAX_TOKENS" \
        --reasoning off >/dev/null

for _ in $(seq 1 180); do
    if model_is_ready; then
        echo "==> Qwen3.5 VLM 已就绪: $HEALTH_URL"
        echo "    日志: $LOG"
        exit 0
    fi
    if ! systemctl --user is-active --quiet "$UNIT"; then
        echo "错误: Qwen3.5 VLM 服务异常退出，日志末尾：" >&2
        tail -40 "$LOG" >&2 || true
        exit 1
    fi
    sleep 1
done

echo "错误: Qwen3.5 VLM 在 180 秒内未就绪，日志末尾：" >&2
tail -40 "$LOG" >&2 || true
exit 1
