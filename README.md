# 众擎 EngineAI（PM01 / T800）机器人知识库

> 构建时间：2026-10-03 ｜ 工作区：`/home/admin2/t800`（约 4.0 GB）
> 内容：10 个官方开源仓库完整快照（含 git-lfs 大文件，全部经哈希校验）、54 页官方文档离线镜像、18 篇中文笔记。
> 数据来源：GitHub [engineai-robotics](https://github.com/engineai-robotics) + 官方文档站 engineai.com.cn + 用户提供的资料链接（见 `docs/notes/00-资源链接汇总.md`）。

---

## 一、按任务找入口

| 我要做什么 | 去哪里 |
| --- | --- |
| 训练策略（行走 / 全身动作跟踪 / 舞蹈） | `repos/engineai_rl_lab/`（Isaac Lab，PM01/T800）+ 笔记 `docs/notes/repos/engineai_rl_lab.md` |
| 把策略部署到真机 | `repos/engineai_robotics_native_sdk/`（C++，MNN 策略）+ 笔记；操作手册看 `docs/t800-online/native-sdk/` 4 页 |
| 用 ROS2 做二次开发（发指令、读状态、接传感器） | `repos/engineai_ros2_workspace/` + 接口文档 `docs/t800-online/ros2-interface/` 10 页 + 笔记 `docs/notes/repos/engineai_ros2_workspace.md` |
| **控制真机（实机链路、手指/关节控制、Linux 面板）** | `docs/notes/04-Orin中转真机控制.md` + `robot_control/`（Electron 全关节控制面板） |
| **看激光雷达/深度相机（Odin2 点云 + 双目图像，本机查看）** | `docs/notes/06-T800雷达与深度相机Odin2.md` + `robot_control/`（自研查看器） |
| **机器人本机跑 LLM + 语音 Agent（Ollama + Qwen3.5-4B，35 tok/s，查电量/做动作工具）** | `docs/notes/07-机器人LLM部署.md` |
| **内置动作调研 + 语音控制动作（点头/挥手/握手/比心，覆盖控制实现）** | `docs/notes/08-T800内置动作与语音控制.md` |
| 在电脑上跑仿真（MuJoCo + 虚拟手柄，一条命令起三机） | `repos/engineai_robot_simulation_runtime/` + 笔记 |
| 人体动作重定向到机器人（BVH / SMPL-X / AMASS） | `repos/GMR/` + 笔记 `docs/notes/repos/GMR.md` |
| 拿机器人模型（URDF / mesh / USD） | `repos/engineai_robotics_description/`（PM01/T800/T800 Pro）+ 笔记 |
| 做导航（LiDAR → 端到端策略 → 速度指令） | `repos/engineai_navigation/` + 笔记 |
| **自建 T800 仿真 + 自研 WBC 训练（MuJoCo/MJX，站立/踏步/持载荷/短时行走）** | `sim/`（README + `docs/notes/03-T800仿真与WBC实践.md` + `docs/notes/05-T800仿真与WBC交付报告.md`） |
| 看官方文档原文（离线） | `docs/t800-online/`（32 页）、`docs/pm01-online/`（22 页） |
| 快速了解各仓库全貌 | `docs/notes/repos/` 下 10 篇仓库笔记（每篇 120-200 行） |
| PM01 / T800 硬件参数速查 | `docs/notes/02-PM01官方文档导读.md`、`docs/notes/01-T800官方文档导读.md` |
| 老款机器人（zqsa01 / 第一代技术栈） | `repos/engineai_legged_gym/`、`repos/engineai_humanoid/` + 对应笔记 |

## 二、目录结构

```
t800/
├── README.md                     ← 本文件（总索引）
├── repos/                        ← 10 个官方仓库完整快照（见下表）
├── docs/
│   ├── t800-online/              ← T800 官方文档 32 页（Markdown，含图解）
│   ├── pm01-online/              ← PM01 官方文档 22 页
│   ├── notes/
│   │   ├── 00-资源链接汇总.md     ← 官方/用户提供的所有链接 + 仓库快照表
│   │   ├── 01-T800官方文档导读.md ← 32 页逐页导读 + ROS2 接口清单
│   │   ├── 02-PM01官方文档导读.md ← PM01 硬件 5 页详解 + PM01/T800 差异
│   │   ├── 03-T800仿真与WBC实践.md ← 自建 MuJoCo 仿真与自研 WBC 的完整实践记录
│   │   ├── 04-Orin中转真机控制.md ← 真机控制实战：Orin 中转链路、ROS2 接口实测、手指/关节控制
│   │   ├── 06-T800雷达与深度相机Odin2.md ← Odin2 雷达相机接入：驱动选型（厂商 SDK vs 打包 SDK 的发现坑）、话题、查看通路
│   │   ├── 07-机器人LLM部署.md ← .162 上 Ollama + Qwen3.5-4B（2026-02，782万下载）：35 tok/s 实测、Jetson 专用包踩坑、语音 Agent（get_battery）
│   │   ├── 08-T800内置动作与语音控制.md ← 出厂动作调研（6 状态/开发版技能/动作库）、四个上肢动作实现与语音触发
│   │   └── repos/                ← 10 篇仓库笔记（定位/结构/用法/依赖/坑）
│   ├── assets/                   ← 文档图片（已从内嵌 base64 分离）
│   └── provenance/               ← 快照 commit、下载 sha256、校验记录、页面清单
└── scripts/                      ← 可重复运行的抓取与校验脚本（见第八节）
```

## 三、仓库快照清单

| 仓库 | 定位 | 分支/commit | 大小 |
| --- | --- | --- | --- |
| `engineai_rl_lab` | 新一代 RL 训练（Isaac Lab 2.3.2；whole-body tracking + AMP；PM01/T800） | main / c53f7ec6 | 363M |
| `engineai_ros2_workspace` | ROS2 Humble 部署与二次开发工作区（4 个包：接口、示例、MuJoCo 桥、音频） | community / c41043f6 | 164M |
| `engineai_robotics_native_sdk` | Native Control SDK：C++ 真机运动控制/策略部署（core 预编译 + Runner 插件层） | main / 335c60e8 | 811M |
| `engineai_robot_simulation_runtime` | Docker Compose 仿真运行时（MuJoCo + 虚拟手柄，镜像含 pm_v2/t800/t800pro） | main / 8c0ece74 | 76K |
| `engineai_robotics_description` | 机器人模型：PM01（24 DOF）/ T800 / T800 Pro（25/43 DOF）URDF+mesh+USD | master / 1d2d7601 | 1.5G |
| `GMR` | 动作重定向：BVH/SMPL-X/AMASS → PM01/T800，输出 pkl/CSV 供 tracking 训练 | main / 71e02c05 | 178M |
| `engineai_navigation` | PM01 端到端导航：Livox Mid-360 → 有序点云 → SRU 策略 → 速度指令 | master / 7ee29a3e | 849M |
| `engineai_amp` | 早期独立 AMP 仓库（包名 engineai_lab；仅 PM01；速度行走 + AMP） | main / 83ba64bb | 208M |
| `engineai_legged_gym` | 第一代 Isaac Gym 训练（仅 zqsa01，2024-09 后停更） | master / 1a659aa0 | 12M |
| `engineai_humanoid` | 第一代 C++ 实机控制器（LCM/ROS1/onnxruntime，2024-09 后停更） | master / 5c0cb92a | 11M |

技术代际：**老栈** `legged_gym`（训练）+ `humanoid`（部署，onnx）→ **新栈** `rl_lab`（Isaac Lab 训练）+ `native_sdk` / `ros2_workspace`（MNN/ROS2 部署）；`amp` 是中间过渡产物，`navigation` 和 `GMR` 是独立功能仓库。

## 四、官方文档离线镜像

| 文档树 | 页数 | 内容 |
| --- | --- | --- |
| `docs/t800-online/` | 32 | t800-developer（7 页硬件/电气）、t800-pro（8 页）、ros2-interface（10 页）、native-sdk（4 页）、support（3 页 FAQ） |
| `docs/pm01-online/` | 22 | pm01（5 页硬件）、ros2-interface（10 页）、native-sdk（4 页）、support（3 页） |

注意：**两个产品线的 ros2-interface（10 页）与 native-sdk（4 页）文档内容逐字相同**，support 页也基本一致；差异只在硬件/产品页（PM01 为 24 自由度等）。因此二次开发接口以任一套为准即可，硬件参数需按机型看对应目录。

图片已从页面内嵌 base64 分离为 `docs/assets/` 下的独立文件，Markdown 中的引用可直接打开。

## 五、笔记索引（18 篇）

- `docs/notes/00-资源链接汇总.md` — 所有外部链接、百度网盘入口、仓库快照表、上游依赖
- `docs/notes/01-T800官方文档导读.md` — T800 32 页逐页导读、ROS2 接口名称清单、SDK 源码解析要点
- `docs/notes/02-PM01官方文档导读.md` — PM01 硬件 5 页详解、PM01/T800 差异对比、开发入口映射
- `docs/notes/03-T800仿真与WBC实践.md` — 自建 MuJoCo 仿真、自研训练栈与 WBC 的实践与结论
- `docs/notes/04-Orin中转真机控制.md` — 真机控制实战：网络拓扑/凭据、SSH 中转、ROS2 接口实测、手指与关节控制、踩坑
- `docs/notes/05-T800仿真与WBC交付报告.md` — 自建仿真与 WBC 的交付报告
- `docs/notes/06-T800雷达与深度相机Odin2.md` — Odin2 激光雷达相机接入实战：传感网段拓扑、驱动选型与 SDK 发现坑、话题/帧率、本机查看通路
- `docs/notes/07-机器人LLM部署.md` — 机器人算力机本机 LLM 与语音 Agent：Qwen3.5-4B + Ollama（Jetson 包坑、think:false、35 tok/s 实测）、ASR→Agent→TTS 全链路与回声抑制
- `docs/notes/08-T800内置动作与语音控制.md` — 内置动作调研（Pro 6 状态、开发版技能库、覆盖控制 31 关节证据）+ 点头/挥手/握手/比心四个动作的关键帧、MuJoCo 离线校验与语音控制
- `docs/notes/repos/*.md` — 10 个仓库各一篇，模板统一：定位 / 目录结构 / 核心模块 / 使用方法 / 依赖 / 仓库间关系 / 已知问题

各笔记的"已知问题"一节的发现都基于快照源码逐文件核对，包含：任务注册损坏、文档与代码不一致、默认配置指向错误路径等，使用前建议先读对应仓库笔记的第 7 节。

## 六、数据来源与可信度

- **代码快照**：通过 `ghfast.top` 镜像下载 tarball（直连 GitHub 过慢，~60KB/s），随后用 GitHub git tree API 的 **blob SHA1 对每个文件逐一比对**：10/10 仓库 PASS（0 缺失 / 0 多余 / 0 哈希不符），确认镜像传输未被篡改。tarball 的 sha256 见 `docs/provenance/sha256.tsv`，校验原文见 `docs/provenance/verify.log`。
- **git-lfs 大文件**：`engineai_rl_lab`（23 个指针）与 `engineai_navigation`（5 个指针）中共 **23 个唯一 LFS 对象（约 252MB，含 122.7MB 的 PM01 USD 场景、T800 USD、AMP 策略权重与演示 gif）已全部拉取为真实文件**，每个对象经 **sha256 校验**、跨仓库共享对象自动去重（共替换 28 个指针文件）。
- **快照版本**：各仓库 commit 与日期见 `docs/provenance/snapshot.tsv`（也列于 `docs/notes/00-资源链接汇总.md` 第 3 节）。
- **文档**：抓取时间为 2026-10-03，页面清单见 `docs/provenance/manifest.tsv`（含每页来源 URL）。
- **未获取**：百度网盘中的"PM01 engineai lab 训练框架"打包资料（需要登录客户端，无法自动下载），见 `docs/notes/00-资源链接汇总.md` 第 2 节。

## 七、重要提示（汇总）

1. 用户在别处看到的 `engineai_rl_workspace` 仓库**已不存在（404）**，训练框架现为 `engineai_rl_lab`；`engineai_ros2_workspace` 仍在。
2. `engineai_rl_lab` 的部分训练任务注册存在缺陷（PM01 Low-Freq）、AMP 导出路径与 README 不符等，详见其笔记第 7 节。
3. 官方在线文档自身存在少量笔误与自相矛盾（如 Topic 名缺 `/motion` 前缀、PM01 单臂自由度 5/6 不一致），已在两篇导读笔记中逐条标注。
4. 第一代仓库（`legged_gym`/`humanoid`）已停更，面向的是老机型 zqsa01，不要与 PM01/T800 的新栈混用。
5. 仿真运行时（`engineai_robot_simulation_runtime`）以成品 Docker 镜像分发，仿真支持的机型与真机子集不同（如 ros2_workspace 自带仿真是 pm_v2）。

## 八、维护与更新

`scripts/` 中的脚本可直接重跑（幂等）：

| 脚本 | 作用 |
| --- | --- |
| `fetch_repos.sh` | 重新下载/更新全部 10 个仓库（镜像 + 直连回退，输出 sha256） |
| `repo_verify.py` | 用 GitHub API 逐文件校验本地快照（可指定单个仓库名） |
| `crawl_engineai_docs.py [t800\|pm01]` | 重新抓取官方文档为 Markdown（含图片本地化） |
| `fix_embedded_images.py` | 把页面内嵌 base64 图片分离为独立文件 |
| `fetch_lfs.py` / `fetch_lfs_ranged.py` / `lfs_chunked.py` | 拉取 git-lfs 对象（依次为多线程版、分段并发版、分块断点续传版——大文件推荐最后一个） |

更新流程建议：`fetch_repos.sh` → `repo_verify.py` → 需要时重跑文档抓取 → 对比新旧 commit 后更新笔记。
