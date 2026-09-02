# GO2-W 端侧 ReMEmbR 架构适配

## 当前结论

这台 Orin NX 16GB 可以承载语义记忆闭环，但不适合原样叠加一套独立机器人、
Milvus 和导航进程。本仓库采用轻量适配：保留 ReMEmbR 的“连续视觉描述 →
时空记忆 → 检索/推理 → 地点候选”分层，把传感器、坐标和运动安全全部接回
现有稳定链路。

```text
Unitree 前置相机（已有 VideoClient，640×360 / 4 Hz）
  └─ 原子 JPEG 快照（只在内存中短暂缓冲，不落原图）
       └─ 本机 Qwen3.5-2B Q8_0（llama.cpp / loopback HTTP）
            └─ 片段中文描述 + 同时刻 map 位姿
                 └─ SQLite/WAL + embedding（NVMe）
                      └─ 文本 / 位置 / 时间检索
                           └─ 现有 5 cm 静态安全栅格 A* 预演
                                └─ 候选坐标（必须由人再次确认发送）

XT16 + LIO-SAM ── 已标定 map 位姿 ───────────────────┘
SCAN + 底盘安全门只接受原有 /api/navigation/goal ────┘
```

实现位于 `src/go2_remembr`，作为库嵌入现有 `web_server`，没有再启动一套 ROS
节点或 TF 消费者。这样语义帧使用的正是 Web 已验证的 map 位姿，也避免端侧
DDS、CPU 和内存重复开销。

## 硬件部署策略

当前硬件是 Jetson Orin NX 16GB、JetPack 5.1.1 / L4T R35.3.1、CUDA 11.4、
TensorRT 8.5.2，数据盘是约 477GB NVMe。建议保持三层隔离：

1. LIO-SAM、SCAN 和底盘安全门保持现状，永远拥有运动控制权。
2. `go2_remembr` 与 Web 同进程，仅做采样、数据库和候选验证；SQLite 位于
   `maps/remembr/remembr.sqlite3`，不需要 Milvus 服务。
3. VLM 作为独立的 systemd user service 监听 `127.0.0.1`，崩溃和内存不足都只会
   让记忆采集报错，不影响 SLAM、导航或底盘停车。

VLM 使用 Qwen3.5-2B INT8、单请求串行、短上下文，每段最多取 2 张
640×360 图像。不升级 JetPack，不改动现有 CUDA 链。模型服务无 ROS 控制
话题权限，只能通过 Web 传入的 JPEG 和 map 位姿生成文本记忆。

端侧资源控制已经固化在实际配置中：1.5 秒采样一次、3 秒形成一个片段、最多
2 帧、每次只进行一个 VLM 请求；推理繁忙时只保留最新的完整待处理片段，旧片段
直接覆盖，不形成无界队列。原始 JPEG 不写入数据库。embedding 是
256 维 Unicode feature hash，不占 GPU，便于在 VLM 未定时先验证完整数据链。
以后可独立换成本机 OpenAI-compatible embedding 服务。

记忆入库时在同一 SQLite 事务内做位姿去重：只在同一地图签名中比较，
平面距离不超过 0.35 m、高度差不超过 0.30 m、最短朝向差不超过 20°
时判定为相似视点。新记忆会替换所有匹配的旧记忆；反向视角、不同楼层
或不同地图不会被合并。删除与插入任一失败时整个事务回滚，旧记忆
保留。阈值位于 `maps/remembr/config.json` 的 `deduplication` 段。

## 构建与默认行为

```bash
cd /home/unitree/go2_slam_ws
colcon build --symlink-install --packages-up-to go2_slam_web
source install/setup.bash
```

也可以执行 `./start.sh --build`。源码默认 VLM backend 仍是 `disabled`，但本机的
`maps/remembr/config.json` 已覆盖为 Qwen3.5 本机服务。“自动记忆”仍默认
关闭：只有模型在线、地图标定有效且操作者在页面显式开启后才会采集。

如果构建时机器人原本就在运行，Python Web 进程不会热加载新后端。请把机器人
停稳并锁定底盘后，在方便重新做本次开机地图定位时执行完整重启：

```bash
./stop.sh
./start_navigation.sh
```

不要只在活动导航会话中单独重启 Web/host 服务，因为 Web/LIO 会话变化本来就会
让已有 map/odom 标定失效。

手工 API 示例：

```bash
curl -sS -X POST http://127.0.0.1:8890/api/remembr/memory \
  -H 'Content-Type: application/json' \
  -d '{"caption":"走廊尽头左侧有一个红色灭火器"}'

curl -sS -X POST http://127.0.0.1:8890/api/remembr/query \
  -H 'Content-Type: application/json' \
  -d '{"mode":"text","text":"红色灭火器在哪里","execute":false}'
```

手工写入和自动采集都要求：当前处于导航模式、地图基准签名存在、重启后的
map/odom 标定有效、位姿新鲜。切换地图后，检索只查看新地图签名对应的记录；
旧记录不删除，但不会串图。

## VLM 运行与更换

### 当前已集成的 Qwen3.5-2B INT8

当前端侧实现已经固定为以下运行组合：

- 主模型：`models/Qwen3.5-2B-GGUF/Qwen3.5-2B-Q8_0.gguf`，2.01 GB；
- 视觉投影：`models/Qwen3.5-2B-GGUF/mmproj-Q8_0.gguf`，365 MB；主模型和
  视觉投影都使用 Q8_0；
- 推理引擎：`third_party/llama.cpp` 的提交
  `d7a2074112d27649303fa107eb8c94db1ee435f3`；
- Jetson 构建：CUDA `sm_87`、单 GPU、关闭 Flash Attention/NCCL/VMM/CUDA
  Graph，避免 Qwen3.5 视觉路径在当前 JetPack 5.1.1 上使用未经验证的算子；
- 服务：`127.0.0.1:8000`，单并发、2K context、Q8 K-cache/F16 V-cache、每图
  最多 128 个视觉 token、72 token 输出上限、关闭 reasoning；
- systemd 资源边界：CPU 最多 300%、`MemoryHigh=7G`、`MemoryMax=9G`、
  `OOMScoreAdjust=500`。内存紧张时优先牺牲可选 VLM，而不是导航进程。

常用命令：

```bash
# 首次或源码更新后构建
./build_remembr_vlm.sh

# 仅启动/停止 VLM，不重启 ROS、LIO 或 SCAN
./start_remembr_vlm.sh
./stop_remembr_vlm.sh

# 完整导航启动会自动尝试启动 VLM；VLM 失败只降级语义记忆
./start_navigation.sh
```

实际覆盖配置位于 `maps/remembr/config.json`：每 3 秒形成一段记忆，按 1.5 秒
间隔取 2 帧，使用 `qwen3.5-2b-int8` OpenAI-compatible 服务。单次最多
72 个输出 token。自动采集仍默认关闭；完成地图定位后需要在导航页面
显式点击“开启自动记忆”。

在当前 Orin NX 16GB 与已运行的 LIO-SAM/SCAN/Web 并行实测中，同一组两张
640×360 画面均使用 331 个输入 token。2B INT8 冷请求的视觉/提示阶段约
2.53 秒，20 token 中文描述端到端约 3.42 秒，生成速度约 23.1 token/s；
切换前的 4B INT4 同图基线分别为 4.10 秒、6.94 秒和 11.9 token/s。两次
输出长度不同，因此端到端数字不是严格的等长输出比较；视觉阶段和 token/s
更适合横向判断。单张 640×360 冷请求实测约 1.94 秒。当前 llama.cpp
多模态路径没有使用 TensorRT；Flash Attention 在该构建中不受支持，开启会
回退 CPU，必须保持关闭。

### DeepSeek Flash 文本推理

高层 reasoner 已配置为官方 `deepseek-v4-flash` Chat Completions，关闭
思考并强制 JSON 输出。它只在用户发起语义查询时，从本地已检索的少量
候选记忆中选一条；JPEG 不会发到 DeepSeek。远程断网、超时或返回非法
ID 时自动退回本地检索第一条，查询不会中断，更不会产生底盘指令。

密钥不写入仓库或 JSON，请直接在机器终端执行：

```bash
cd /home/unitree/go2_slam_ws
./configure_remembr_api.sh
```

脚本不回显输入，密钥保存到工作空间外的
`/home/unitree/.config/go2-remembr/secrets.env`，目录权限 0700、文件权限
0600。`setup_env.sh` 只解析白名单变量 `DEEPSEEK_API_KEY`，不执行该
文件内容。密钥存入后需重启 host Web 进程才会生效。

`stop.sh` 会一并停止 VLM。VLM 服务仅绑定 loopback，局域网设备不能直接访问
8000 端口。

### 更换模型

复制默认配置后只填模型服务，不改代码：

```bash
mkdir -p /home/unitree/go2_slam_ws/maps/remembr
cp /home/unitree/go2_slam_ws/src/go2_remembr/go2_remembr/default_config.json \
  /home/unitree/go2_slam_ws/maps/remembr/config.json
```

编辑 `maps/remembr/config.json`：

```json
{
  "vlm": {
    "backend": "openai_compatible",
    "endpoint": "http://127.0.0.1:8000/v1/chat/completions",
    "model": "以后确定的本地模型名",
    "timeout_s": 90,
    "api_key_env": ""
  }
}
```

配置支持递归覆盖，所以实际文件也可以只保留上面这一段。`setup_env.sh` 会在
文件存在时自动设置 `GO2_REMEMBR_CONFIG`。支持的 VLM adapter：

- `disabled`：默认，不启动采集；
- `openai_compatible`：本机 OpenAI-compatible chat completions，多图 data URL；
- `simple_http`：向本机服务发送 `{model, prompt, images:[base64...]}`，接收
  `{caption}` 或 `{text}`。

embedding 支持 `hash` 和 `openai_compatible`。reasoner 支持本地确定性
`retrieval_only` 与 `openai_compatible`；当前用 DeepSeek Flash，但无论选择器输出
什么，它只能从已检索出的 memory ID 中选一个。

## 接口和安全边界

- `GET /api/remembr/status`：模型、数据库、当前地图记录数和最近错误；
- `POST /api/remembr/control`：开启/关闭自动采集；VLM 未配置时拒绝开启；
- `POST /api/remembr/memory`：在当前已对齐 map 位姿手工写入描述；
- `POST /api/remembr/query`：`text`、`position` 或 `time` 检索并返回候选。

`/api/remembr/query` 明确拒绝 `execute:true`。候选目标还会调用现有
`NavigationState.preview_plan()`，经过同一份膨胀静态地图检查；它只填入网页
目标框，既不发布 ROS Path，也不启用底盘。真实运动仍必须走现有
`/api/navigation/goal` 的人工确认、地图标定、SCAN、互斥锁和底盘安全门。

## 选定 VLM 后的验收

1. 导航正常运行 15 分钟，记录无 VLM 时的 `tegrastats`、LIO 频率和 Web 状态。
2. 只启用 VLM 服务、不启用自动记忆，确认基础内存和温度稳定。
3. 开启自动记忆，确认数据库每约 10 秒增加一条，JPEG 未落盘，map 坐标正确。
4. 覆盖文字标牌、物体、重复走廊、光照变化和人员经过场景，检查错误描述率。
5. 断开 VLM、杀死模型进程、让位姿过期和切换地图，确认采集停止而 SLAM/导航
   不受影响。
6. 查询记忆，确认页面只填入候选；未点击“规划并发送”时 ROS 全局路径和底盘
   均无动作。
