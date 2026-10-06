# 07 - 机器人本机 LLM 部署（Ollama + Qwen3.5-4B，.162）

> 2026-10-04 实机部署验证。在应用计算单元 `.162`（Jetson AGX Orin 64GB 开发套件，JetPack 6.2/L4T R36.4.3，
> CUDA 12.6，MAXN）上部署了离线 LLM 推理服务，中文实测 **35 tok/s、短回答端到端 0.8 秒**。

## 1. 选型

**Qwen3.5-4B**（Qwen 官方，2026-02-27 发布，Apache-2.0，图文多模态）：
- 2026 年小模型里 HF 下载量最高（**782 万**；对比同期 NVIDIA Nemotron-3-Nano-4B 为 293 万）；
- 中文原生；unsloth GGUF 社区量化齐全；Q4_K_M 单文件 2.6GB。

推理引擎：**Ollama v0.35.1**，注意它按平台分包：`ollama-linux-arm64.tar.zst` 是本体（bin + 常规 CUDA 库），
**`ollama-linux-arm64-jetpack6.tar.zst` 是 Jetson 专用 CUDA 库附件（只有 lib/ollama/cuda_jetpack6/，必须两个都装）**。

## 2. 部署结果（.162）

| 项 | 值 |
|---|---|
| 服务 | systemd `ollama.service`，`OLLAMA_HOST=0.0.0.0:11434`，`OLLAMA_MODELS=/home/ubuntu/ollama_models`，`OLLAMA_KEEP_ALIVE=30m` |
| GPU 识别 | `CUDA0 / Orin iGPU / compute 8.7 / libdirs=ollama,cuda_jetpack6 / 驱动 12.6`（jetpack6 库被 v0.35.1 原生感知） |
| 实测性能 | 生成 **34~35.4 tok/s**；prompt 处理 ~300 tok/s；短回答总耗时 **0.76s**（模型已热） |
| 模型名 | `qwen3.5-4b:latest`（2.7GB，Modelfile：num_ctx 4096） |

文件位置（.162）：`/usr/local/bin/ollama`、`/usr/local/lib/ollama/`（含 `cuda_jetpack6/`）、
`/etc/systemd/system/ollama.service`、模型与 Modelfile 备份在 `~/llm/`。
本机留档：`models/ollama_deploy/`（Modelfile + service + install.sh）与 `models/*.gguf|tar.zst` 源文件。

## 3. 用法

```bash
# .162 本机或经 Orin 端口转发后（本机）：
ssh -N -L 11434:192.168.0.162:11434 orin &     # 然后本机访问 http://127.0.0.1:11434
curl 127.0.0.1:11434/api/chat -d '{
  "model":"qwen3.5-4b", "stream":false, "think":false,
  "messages":[{"role":"system","content":"你是T800 Pro的语音助手，回答简短口语化。"},
              {"role":"user","content":"介绍一下你自己"}]}'
```

## 4. 踩坑记录

1. **Qwen3.5 默认输出思维链**：不关 thinking 会先输出几千 token 的推理过程（语音助手不可用）。
   API 加 **`"think": false`**（chat/generate 都支持）即直接回答，响应里无 thinking 块。
2. **Ollama 服务启动顺序坑**：先启了服务、后补的二进制时，GPU 发现会一直失败且 systemd 不会重载新文件——
   装完务必 `systemctl restart ollama`，日志出现 `inference compute ... library=CUDA ... description=Orin` 才算用上 GPU。
3. **jetpack6 包只是 CUDA 库附件**（仅 lib/ollama/cuda_jetpack6/），没有 bin/ollama 和 llama-server；
   必须再装本体 `ollama-linux-arm64.tar.zst`（v0.35.1 起旧式 runners 改为单一 `llama-server` 二进制）。
4. **`.162` 无外网**：模型与 Ollama 包全部本机下载后经 Orin 两跳 scp 推送（速度 ~3.4MB/s，受 WiFi 跳限制）。
   下载源实测：GGUF 走 **ModelScope 镜像**（hf-mirror 的 resolve 会 302 到 xethub CDN，本网络下不稳）；
   GitHub 大包走 **gh-proxy.com**（847KB/s，支持 Range 分段；ghfast.top 仅 ~40KB/s）。
   **gh-proxy 部分节点会忽略 Range 头**——多线程分段下载时加 `--max-filesize <段大小>` 防呆，
   否则某一段会变成整文件下载且难察觉。
5. 上下文 4k 时显存+内存占用无压力（61GB 统一内存），Qwen3.5-4B 为多模态底座——后续可接
   mmproj 投影文件让机器人头部相机画面直接进 LLM（视觉问答）。

## 5. 语音助手链路（已完成，2026-10-04）

```
AIUI 环麦(16kHz, Pulse default 源) → funasr ws:10095(出厂容器, SenseVoice)
  → final 文本 → t800_agent（Ollama qwen3.5-4b, think:false, XML 工具调用）
  → 面板「回答」pill + /sound_hub/tts（Piper 离线合成）播报
```

- **Agent**：`robot_control/bridge/t800_agent.py`（部署在 .162 `~/t800_mic/`）。提示词按 T800
  重新写（语音助手人设 + 工具说明 + 少样本），wtg 那套四足导航提示词与全部内置命令已删除；
  **工具只保留一条 `get_battery`（查电量）**：TCP 读 .163:9801 关节 bridge 的 `state.battery`
  （源头 `/hardware/power_info`，实测 54% / 64.4V，2s 缓存）→ 交给模型口语化回答。
  触发语实测："现在电量还有多少" / "电池要充电了吗" 均走工具；闲聊（"你好"）不调工具。
  两轮工具调用总耗时 ~1.3-1.6s（模型已热）。
- **集成**：`robot_control/bridge/mic_bridge.py`（9803）识别 final 后进 Agent，回复走
  `/sound_hub/tts` 播报并推面板；`--agent 0` 可退回纯聊天。
- **TTS 打通的两个修复**：① sound_hub 的 TtsServiceComponent 在
  `/app/applications/install/sound_hub/local/lib/python3.10/dist-packages/sound_hub/sound_hub.py`
  里出厂是注释掉的，解开 import + add_component；② 安装 /opt/piper（aarch64 二进制 +
  zh_CN-huayan-medium.onnx 语音 + espeak-ng 数据），sound_hub 日志确认
  "Streaming TTS with model: /opt/piper/zh_CN-huayan-medium.onnx" 后正常播放。
- **回声抑制**：播报后开 `tts_until = now + max(3s, 0.28×字数+2.5s)` 抑制窗，窗内 ASR 结果丢弃
  （状态机照常推进），解决"自己说话→自己识别→再回答"的 ~9s 自激循环。
- **重启大坑**：mic_bridge 重启**必须 `kill -9`**（`bridge/start_mic.sh` 已内置）。用普通
  kill/pkill（SIGTERM）时 rclpy 信号处理只把 context 置为 shutdown、**进程不退出**，之后所有
  TTS publish 报 `publisher's context is invalid`（表面进程健康、实则 TTS 静默失效）。
- 环麦直采（`plughw:CARD=AIUIUSBMC`）会被 PulseAudio 占设备而 rc=1 退出，桥内自动回退
  `default` 源（就是 AIUI 环麦，`pactl get-default-source` 可确认），分贝/识别不受影响。

## 6. 官方 TTS API 与音色（2026-10-04 补充）

**官方"让机器人说话"的 API = ROS2 话题 `/sound_hub/tts`**（`interface_protocol/msg/Tts`：
text / speaker / language / rate）。sound_hub 收到后调 Piper 合成并 `aplay` 播放；官方
`robot_manager` 里的 `TtsClientComponent.speak()` 就是同一话题的 Python 封装。一条命令即可直调：

```bash
# .162 上（source /opt/ros/humble + /app/applications/install/setup.bash，ROS_DOMAIN_ID=69）
ros2 topic pub --once /sound_hub/tts interface_protocol/msg/Tts \
  "{text: '你好，我是T800机器人', speaker: 'zh_CN-huayan-medium', language: 'zh', rate: 0}"
```

**音色机制**：`/opt/piper/<speaker>.onnx`；speaker 文件不存在时回退到
**原厂默认音色名 `en_GB-alan-low`**（`piper_engine.py` 写死；英文男声，开机播报那套英文文案就是配它的），
再找不到就用目录里任意 .onnx。出厂镜像里 **没有任何音色模型文件**（`/opt/piper` 整个目录都是后装的，
两台电脑 + docker 镜像 + 出厂软件包均无 .onnx），所以"原厂男声"只能按默认音色名 `en_GB-alan-low`
现装恢复。现装的三套音色：

| 音色文件 | 类型 | 状态 |
|---|---|---|
| `zh_CN-huayan-medium.onnx` | 中文女声 22050Hz | ✅ 正常，当前语音助手在用（`--tts-speaker` 默认值） |
| `en_GB-alan-low.onnx` | 英文男声 16kHz（原厂默认名） | ✅ 能合成；但 sound_hub 固定按 22050Hz 播放 16k 数据，**播放偏快**（引擎的小坑，改 `piper_engine.py` 的 aplay 采样率可修） |
| `zh_CN-chaowen-medium.onnx` | 中文男声 22050Hz（Piper 唯一中文男声） | ❌ 需 piper≥1.3（拼音音素表）；机器人自带 2023 版 1.2.0 报 `Phonemes must be one codepoint`。新版 wheel（aarch64）+ g2pW 注音模型可行但未部署（用户决定不折腾） |

另有 `robot_manager` 的状态播报（config `motion_state_monitor.yaml`，英文文案 + 可选 tts/alert，
当前为 alert 提示音），启用 tts 即用上述通道。
