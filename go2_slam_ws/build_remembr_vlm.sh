#!/bin/bash
# Build a pinned llama.cpp CUDA server for Qwen3.5 on Jetson Orin (sm_87).
set -euo pipefail

WS="$(cd "$(dirname "$0")" && pwd)"
SOURCE="$WS/third_party/llama.cpp"
BUILD_DIR="$SOURCE/build-jetson"
CMAKE_BIN="/home/unitree/.local/bin/cmake"
PINNED_REV="d7a2074112d27649303fa107eb8c94db1ee435f3"

if [ ! -x "$CMAKE_BIN" ]; then
    echo "==> 安装用户级 CMake 3.31.6"
    python3 -m pip install --user 'cmake==3.31.6'
fi

if [ ! -d "$SOURCE/.git" ]; then
    echo "==> 获取 llama.cpp"
    git clone https://github.com/ggml-org/llama.cpp.git "$SOURCE"
    git -C "$SOURCE" checkout --detach "$PINNED_REV"
fi

actual_rev="$(git -C "$SOURCE" rev-parse HEAD)"
if [ "$actual_rev" != "$PINNED_REV" ]; then
    echo "警告: llama.cpp 当前为 $actual_rev，不是已验收提交 $PINNED_REV" >&2
fi

echo "==> 配置 Jetson CUDA 构建（sm_87，关闭 Flash Attention）"
"$CMAKE_BIN" -S "$SOURCE" -B "$BUILD_DIR" \
    -DCMAKE_BUILD_TYPE=Release \
    -DGGML_CUDA=ON \
    -DCMAKE_CUDA_ARCHITECTURES=87 \
    -DGGML_NATIVE=ON \
    -DGGML_CUDA_FA=OFF \
    -DGGML_CUDA_NCCL=OFF \
    -DGGML_CUDA_NO_VMM=ON \
    -DGGML_CUDA_GRAPHS=OFF \
    -DLLAMA_BUILD_TESTS=OFF \
    -DLLAMA_BUILD_EXAMPLES=OFF \
    -DLLAMA_BUILD_TOOLS=ON \
    -DLLAMA_BUILD_SERVER=ON \
    -DLLAMA_BUILD_APP=OFF \
    -DLLAMA_BUILD_UI=OFF \
    -DLLAMA_OPENSSL=OFF

echo "==> 编译 llama-server（低并发，避免影响在线导航）"
nice -n 10 "$CMAKE_BIN" --build "$BUILD_DIR" --config Release \
    --parallel 2 --target llama-server

"$BUILD_DIR/bin/llama-server" --version
echo "==> 完成: $BUILD_DIR/bin/llama-server"
