# engineai_robot_simulation_runtime — Docker Compose 一键启动的众擎仿真运行环境（motion_control + MuJoCo + Virtual Gamepad）

> 来源：https://github.com/engineai-robotics/engineai_robot_simulation_runtime ｜ 分支：main ｜ commit：8c0ece7448a2（2026-09-18）｜ 快照日期：2026-10-03 ｜ 本地路径：`repos/engineai_robot_simulation_runtime/` ｜ 许可证：BSD 3-Clause（`LICENSE.txt`，Copyright (c) 2025- Shenzhen Zhongqing Robot Technology Co., Ltd. ("EngineAI")）

## 1. 定位与用途

- 用 Docker Compose 启动一套开箱即用的机器人仿真环境：**运动控制 `motion_control` + MuJoCo 物理仿真/可视化 `mujoco` + 虚拟手柄 `virtual_gamepad`** 三个服务。
- 应用以成品镜像分发，**一份镜像包含 `pm_v2`、`t800`、`t800pro` 三种机型**；切换机型只需 `./run.sh [robot]`，无需重新初始化。
- 主要用途：在无真机的情况下验证控制器/策略、学习 ROS2 接口（话题/服务/动作）、用虚拟手柄操作仿真机器人。教程默认从 `motion_control` 容器内开始，因为自定义接口包已安装在镜像内。
- 与源码级仿真（`engineai_ros2_workspace` 的 `mujoco_simulator`）互补：本仓库是容器化、多机型、带官方运动控制的运行时。

## 2. 目录结构

```text
engineai_robot_simulation_runtime/
├── README.md / README_CN.md        # 中英文完整教程（快速开始、脚本、ROS2、FAQ）
├── .env.example                    # 环境变量模板（init.sh 首次运行复制为 .env）
├── docker-compose.yml              # 三个服务定义（host network/ipc、X11、privileged）
├── docker-compose.override.yml     # 可选本地覆盖（示例，不入库；存在时自动合并）
├── init.sh                         # 拉取镜像、可选启用 GPU、写入 .env
├── run.sh                          # 启动仿真：./run.sh [pm_v2|t800|t800pro]
├── enter.sh                        # 进入运行中的容器并加载 ROS2
├── stop.sh                         # 停止并删除三个服务容器
├── clear.sh                        # 停止容器并删除本地镜像（不删 .env）
└── LICENSE.txt                     # BSD 3-Clause
```

## 3. 核心模块与关键文件

### 3.1 docker-compose 服务

| 服务 | 容器名 | 容器内入口 | 职责 |
|---|---|---|---|
| `motion_control` | `engineai_motion_control` | `./run.sh <robot>` | 机器人运动控制；默认监听 TCP `17667` |
| `mujoco` | `engineai_mujoco` | `./run_mujoco.sh <robot>` | MuJoCo 仿真与可视化 |
| `virtual_gamepad` | `engineai_virtual_gamepad` | `./run_virtual_gamepad.sh` | 虚拟手柄界面（TCP 发送控制数据） |

- 三个服务统一配置：`network_mode: host`、`ipc: host`、`privileged: true`、以宿主机 UID/GID 运行、挂载 `/tmp/.X11-unix`、`/dev`、`/dev/shm`（mujoco 额外挂载 `/tmp`），工作目录 `/apps/engineai_robotics`。
- 每个服务都注入 `ENGINEAI_ROBOTICS_DIR=/apps/engineai_robotics`、`ENGINEAI_ROBOTICS_RUNTIME_ENV=sim`、`ENGINEAI_ROBOTICS_PLATFORM=mujoco`、`ROS_DOMAIN_ID`、`ROS_LOCALHOST_ONLY`、`DISPLAY`、`QT_X11_NO_MITSHM=1`、`HOME=/tmp/engineai_home`。
- 注意：镜像内的 `run.sh`/`run_mujoco.sh`/`run_virtual_gamepad.sh` 与 `ros2_ws` 都不在本仓库中（需入镜像查看），本仓库只负责编排与启动。

### 3.2 脚本用法与选项

- `./init.sh [--repository <镜像仓库>] [--suffix abc] [-h]`
  - 首次运行从 `.env.example` 生成 `.env`；交互询问 `Use nvidia gpu? (y/n)`：选 y 会在 `docker-compose.yml` 的 `mujoco` 服务下**就地插入 `gpus: all`**，选 n 则清除该行（会改动被跟踪的 compose 文件）。
  - 默认拉取 `ghcr.io/engineai-robotics/engineai_robotics_sim_runtime:latest`；`--suffix abc` 拉取 `:abc-latest`（测试包，不会拉无前缀 latest）；`--repository` 覆盖 `.env` 中的镜像仓。
  - 拉取成功后把 `IMAGE_REPOSITORY`/`PACKAGE_SUFFIX`/`LOCAL_IMAGE`/`DEFAULT_ROBOT`/`ROS_DOMAIN_ID`/`ROS_LOCALHOST_ONLY` 写回 `.env`。
- `./run.sh [robot]`：机型优先级 `参数 > .env 的 DEFAULT_ROBOT > pm_v2`；先 `unset ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION CYCLONEDDS_URI`（避免继承宿主机 shell 的 ROS 配置），再导出 `ROBOT_NAME/HOST_UID/HOST_GID/ENGINEAI_VIRTUAL_GAMEPAD_TCP_PORT/DISPLAY`，执行 `xhost +local:`（若可用），存在 `docker-compose.override.yml` 时自动合并，最后 `docker compose up -d`。
- `./enter.sh [motion_control|mujoco|virtual_gamepad]`（默认 `motion_control`）：以宿主机 UID/GID 进入容器，容器内 `source /opt/ros/humble/setup.bash` 与 `/apps/engineai_robotics/ros2_ws/install/local_setup.bash`，工作目录 `/apps/engineai_robotics`，并注入宿主 `DISPLAY`/ROS 环境变量。
- `./stop.sh`：`docker compose down`（不删镜像、不删 `.env`）；同样支持自动合并 override。
- `./clear.sh [--images] [-y]`：先调用 `stop.sh`，再按 `IMAGE_REPOSITORY`/`IMAGE_BASE_NAME` 匹配删除本地镜像；`-y` 跳过确认；`--images` 为兼容保留参数，实际清理镜像本来就是唯一动作；不删除 `.env`。

### 3.3 环境变量（.env）

| 变量 | 默认/示例 | 作用 |
|---|---|---|
| `IMAGE_REPOSITORY` | `ghcr.io/engineai-robotics/engineai_robotics_sim_runtime` | 镜像仓库 |
| `PACKAGE_SUFFIX` | 空 或 `abc` | 测试 tag 前缀（`<suffix>-latest`） |
| `LOCAL_IMAGE` | `.../engineai_robotics_sim_runtime:latest` | compose 实际使用的镜像 |
| `DEFAULT_ROBOT` | `pm_v2` | 未传参时的默认机型 |
| `ROS_DOMAIN_ID` | `69` | ROS2 Domain ID |
| `ROS_LOCALHOST_ONLY` | `1` | 1=仅本机 DDS；跨主机需改 0 |
| `ENGINEAI_VIRTUAL_GAMEPAD_TCP_PORT` | `17667` | 虚拟手柄 TCP 端口（run.sh 注入，不在 .env.example 中） |

## 4. 使用方法

```bash
# 初始化（默认 GHCR；国内可换阿里云 ACR）
./init.sh
./init.sh --repository acr-engineai-robotics-open-registry.cn-wulanchabu.cr.aliyuncs.com/engineai-robotics-open/engineai_robotics_sim_runtime

# 启动（默认 pm_v2；同一镜像切机型无需重新 init）
./run.sh
./run.sh t800
./run.sh t800pro
docker compose ps

# 日志 / 停止 / 清理
docker compose logs -f
./stop.sh
./clear.sh -y

# 进入容器（默认 motion_control，已加载 ROS2 Humble 与自定义接口）
./enter.sh
./enter.sh mujoco
./enter.sh virtual_gamepad

# 可选 NVIDIA GPU：宿主机装好驱动 + Container Toolkit，在 compose 里给 mujoco 加 gpus: all
sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker
# ./init.sh 询问 GPU 时选 y 会自动加 gpus: all（需 Compose ≥ 2.30；旧版可用 runtime: nvidia 替代）

# Virtual Gamepad：目标填 127.0.0.1:17667，勾选 Enable 后按键操作；切换目标先取消 Enable

# 宿主机/其他设备用 ROS2 访问（三个容器为 host network）
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=69
export ROS_LOCALHOST_ONLY=1        # 跨主机时所有进程都设为 0，并把 .env 改为 0 后重启
ros2 node list
ros2 topic list -t
```

## 5. 依赖与环境

- **宿主机**：Linux x86_64（推荐 Ubuntu 22.04）、Docker Engine、Docker Compose v2（`docker compose`）或 v1（`docker-compose`，脚本自动识别）、可用图形桌面与 `DISPLAY`（MuJoCo 和虚拟手柄需要），另需 `curl python3 x11-xserver-utils ca-certificates`。
- **ROS2**：镜像内为 Humble；默认 `ROS_DOMAIN_ID=69`、`ROS_LOCALHOST_ONLY=1`；访问自定义接口可直接用 `./enter.sh motion_control`。
- **镜像来源与 tag**：默认 GHCR `ghcr.io/engineai-robotics/engineai_robotics_sim_runtime`，国内 ACR 镜像 `acr-engineai-robotics-open-registry.cn-wulanchabu.cr.aliyuncs.com/engineai-robotics-open/engineai_robotics_sim_runtime`；tag 为 `latest` 或 `<suffix>-latest`，仓库不提供版本号/摘要锁定。
- **虚拟机通信**：motion_control 默认监听 TCP `17667`，Virtual Gamepad 通过该端口发送控制数据；默认目标 `127.0.0.1`。
- **GPU（可选）**：NVIDIA 驱动 + NVIDIA Container Toolkit + Compose ≥ 2.30 的 `gpus: all`（或旧版 `runtime: nvidia` + `NVIDIA_VISIBLE_DEVICES=all`/`NVIDIA_DRIVER_CAPABILITIES=all`）。

## 6. 与其他仓库的关系

- **engineai_ros2_workspace**：接口风格一致（`/motion/*`、`/hardware/*` 自定义消息，domain 69）；本仓库把运动控制 + MuJoCo 打包成镜像并提供 `ros2_ws`，ros2_workspace 则是源码工作空间（含 `mujoco_simulator`、`interface_protocol`、示例）。仿真调试优先用本仓库，改接口/真机部署用 ros2_workspace。
- **native_sdk**：native_sdk 自带 MuJoCo 仿真、虚拟手柄（F5/Enable）与 ROS2 bridge；本仓库的 `motion_control` 可视为同类运行时能力的容器化交付，二者内部实现关系未在仓库中说明（未确认）。
- **rl_lab / 训练产物**：本仓库 README 未描述替换自定义策略/模型文件的入口，仿真镜像内的控制策略版本未公开（未确认）；RL 策略的 sim2sim/真机部署路线见 ros2_workspace 与 native_sdk。

## 7. 注意事项、已知问题、文档缺失点

- `init.sh` 会**直接修改 `docker-compose.yml`**（插入/删除 `gpus: all`），使工作区变“脏”；再次回答 n 可恢复。
- `clear.sh --images` 是兼容保留参数（helper 注释说明镜像清理本就是唯一动作），文档列出该选项但无额外行为。
- `.env`、`docker-compose.override.yml` 被 `.gitignore` 忽略，属本地配置；override 需自行创建，`run.sh`/`stop.sh` 自动合并。
- `run.sh`/`enter.sh` 会显式清掉宿主机 shell 的 ROS 发现变量（防止影响容器内发现）；宿主机直连时需自行保证 domain=69 且 `ROS_LOCALHOST_ONLY` 与 `.env` 一致，跨主机要改 `.env` 为 0 并重启。
- 容器为 `privileged` + host network + 挂载 `/dev`、`/dev/shm`，安全边界低，仅建议在可信开发机使用。
- 仓库无 Dockerfile/镜像构建脚本与版本信息：镜像内三机型、`ros2_ws`、各入口脚本的存在与行为只能以 README 描述为准，具体接口清单取决于交付版本（README 也建议“按交付版本的接口文档调用服务或动作”）。
- `run.sh` 不对机型参数做校验：传入非 `pm_v2/t800/t800pro` 的值时会原样传给容器内脚本，失败方式未确认。
- Compose 中所有服务都带 `NVIDIA_DRIVER_CAPABILITIES=all`，但默认没有 `gpus`；该变量本身不会启用 GPU。
- 本快照非 git 仓库（无 `.git`），commit 信息按任务给定记录，无法本地校验。
