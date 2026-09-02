#!/bin/bash
# Store the DeepSeek credential outside the workspace without echoing it.
set -euo pipefail

CONFIG_DIR="/home/unitree/.config/go2-remembr"
SECRET_FILE="$CONFIG_DIR/secrets.env"

read -r -s -p "DeepSeek API Key: " API_KEY
echo
if [ "${#API_KEY}" -lt 20 ]; then
    unset API_KEY
    echo "错误: API Key 长度异常，未保存" >&2
    exit 1
fi
if [[ "$API_KEY" == *$'\n'* || "$API_KEY" == *$'\r'* ]]; then
    unset API_KEY
    echo "错误: API Key 不能包含换行，未保存" >&2
    exit 1
fi

install -d -m 700 "$CONFIG_DIR"
umask 077
TEMP_FILE="$(mktemp "$CONFIG_DIR/.secrets.env.XXXXXX")"
trap 'rm -f "$TEMP_FILE"; unset API_KEY' EXIT
printf 'DEEPSEEK_API_KEY=%s\n' "$API_KEY" >"$TEMP_FILE"
chmod 600 "$TEMP_FILE"
mv -f "$TEMP_FILE" "$SECRET_FILE"
unset API_KEY
trap - EXIT

echo "已安全保存: $SECRET_FILE (0600)"
echo "密钥不在工作空间内，不会写入 ReMEmbR 数据库或日志。"
echo "重启完整导航链后 DeepSeek Flash reasoner 生效。"
