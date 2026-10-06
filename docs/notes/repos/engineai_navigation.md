# engineai_navigation — PM01 基于 SRU 的 LiDAR 端到端导航（Isaac Lab 训练 + ROS2 真机/Gazebo 部署）

> 来源：https://github.com/engineai-robotics/engineai_navigation ｜ 分支：master ｜ commit：7ee29a3ede5e（2026-09-10）｜ 快照日期：2026-10-03 ｜ 本地路径：`repos/engineai_navigation/` ｜ 许可证：Apache-2.0（EngineAI 原创代码 `LICENSE`；内嵌第三方见 `NOTICE`：Super-LIO GPLv3、MNN Apache-2.0、sru-navigation-learning BSD-3、Livox SDK/驱动 MIT、导航任务源码 MIT、interface_protocol BSD-3）

## 1. 定位与用途

- 面向 EngineAI PM01 的自主导航项目（README 自述"基于 SRU 改进"）：用**有序 LiDAR 距离图像**（16×41 组织化点云）感知环境，**PPO 端到端**训练策略，输出速度指令交给底层行走策略执行。
- 两块内容：`training/` 用 Isaac Lab + 自研 SRU 版 rsl_rl 训练并导出 ONNX；`ros/` 是 ROS2 colcon 工作空间，分真机（deployment，PM01 + Livox Mid-360 已验证）与 Gazebo 仿真（simulation）两套源码。
- 仓库自带一次训练产物（`training/logs/.../model_5999.pt`、`export/policy.onnx`）与部署用 `817.onnx`，开箱可跑推理；真机行走依赖外部底层行走策略与 Super-LIO 里程计。
- 机型：仅 PM01（PM01 Edu v2 资产 + `PM01_CYLINDER_CFG`）；未见 T800 支持（未确认未来计划）。

## 2. 目录结构（主要 2-3 层）

```text
engineai_navigation/
├── training/                          # Isaac Lab 训练侧（Python）
│   ├── scripts/nav/                   # train.py / play.py（推理+导出 ONNX/JIT）/ convert_checkpoint.py
│   ├── source/engineai_rl_lab/        # Isaac Lab 扩展包（任务注册 Isaac-Navigation-PPO-PM01-v0）
│   │   └── engineai_rl_lab/
│   │       ├── tasks/navigation/      # PM01 导航环境、MDP（观测/奖励/终止/动作/地形课程）
│   │       │   ├── assets/data/       # 机器人 USD（LFS，本快照已拉取）、locomotion 策略 .pt/.onnx/.mnn
│   │       │   ├── config/pm01/       # PM01 环境与 PPO/SRU 智能体配置
│   │       │   └── mdp/navigation/    # SE2 动作、目标指令、观测、奖励、终止
│   │       ├── tasks/terrains/        # 迷宫/高台/坑洞地形生成（HfMazeTerrainCfg）
│   │       └── assets/pm01_edu_v2/    # PM01 URDF + mesh + texture
│   ├── sru-navigation-learning/       # SRU 版 rsl_rl（含 ActorCriticSRURangeImage、LSTM_SRU、PPO/MDPO）
│   └── logs/rsl_rl/pm01_navigation_range_image_ppo/<时间戳>/  # 随仓库的训练产物（model_5999.pt 53MB 等）
└── ros/
    ├── src/deployment/                # 真机树
    │   ├── livox/                     # livox_sdk2 + livox_ros_driver2（MIT）
    │   ├── protocol/interface_protocol/  # ROS2 消息包（含 BodyVelCmd）
    │   ├── rl_nav/mid360_preprocessor/   # Mid-360 点云重采样为 16×41 组织化点云
    │   ├── rl_nav/rl_nav_policy/         # ONNX 导航策略节点 + RViz 配置 + policy/817.onnx
    │   └── Super-LIO-ros2/            # 激光惯性里程计（GPLv3，独立节点）
    ├── src/simulation/                # 仿真树
    │   ├── simulator/                 # Gazebo Classic PM01 模型、ros2_control PD 控制器、Livox 桥
    │   ├── rl_nav/mid360_preprocessor/   # 同名不同实现（仿真版）
    │   ├── rl_nav/rl_nav_policy/         # 同名不同实现（仿真版，用 /joy）
    │   └── rl_walking/                # C++ MNN 行走策略节点（Gazebo 用）
    └── build|install/{deployment,simulation}/  # 两套独立 colcon 构建/安装目录
```

## 3. 核心模块与关键文件

### 3.1 训练侧（Isaac Lab）

- 任务注册：`training/source/engineai_rl_lab/.../config/pm01/__init__.py` 注册 `Isaac-Navigation-PPO-PM01-v0`；入口脚本 `training/scripts/nav/train.py`、`play.py`（`--export_onnx` / `--export_jit`）。
- 仿真频率：`navigation_env_cfg.py`（基础）与 `config/pm01/navigation_env_cfg.py`（PM01 专用）：sim dt 0.005s（200Hz），导航决策 5Hz（decimation 40），底层行走 decimation 2；episode 60s。
- 场景与感知：PM01 圆柱简化模型 + 迷宫地形（`HfMazeTerrainCfg`：maze 30%、non_maze 高台 50%、pits 动态障碍 20%）；RayCaster 模拟雷达：16 通道、垂直 FOV −60°~0°、水平 ±60°、水平分辨率 3° → 16×41=656 点，max_distance 5m；另有 critic 高度扫描与接触传感器。
- MDP：动作 `HumanoidNavigationSE2Action`（输出 3 维归一化速度，内部按 `policy_scaling=[0.7,0.25,0.7]` 缩放后驱动低层行走策略）；观测 actor = 线速度/角速度/重力投影/上一步动作/目标(4)/雷达点云(656×3)；critic 额外含 height scan 与无噪点云；奖励含到达目标 soft/tight、横移/转向/动作率惩罚；终止含碰撞、大俯仰角、到点、跌落；域随机化含摩擦、推力、动作缩放、观测延迟、低通系数、相机/雷达位姿噪声。
- 地形模块 `tasks/terrains/`：`MAZE_TERRAIN_CFG`/`HfMazeTerrainCfg` 生成迷宫、平台、坑洞，支持目标点采样与动态障碍。

### 3.2 策略网络（SRU）

- 配置 `config/pm01/agents/range_image_rsl_rl_cfg.py`：`class_name=ActorCriticSRURangeImage`，`rnn_type=lstm_sru`（SRU 增强循环单元）、隐藏 512、actor/critic 512-256-128 ELU、dropout 0.2；雷达输入 656×3 归一化 XYZ，range image 形状 16×41、CNN 特征 128；PPO：lr 1e-3 adaptive、γ=0.995、λ=0.95、16 步/环境、max_iterations 8000、seed 60，实验名 `pm01_navigation_range_image_ppo`。
- 实现位于 `training/sru-navigation-learning/rsl_rl/`：`modules/actor_critic_sru_range_image.py`、`networks/range_image.py`、`networks/sru_memory/lstm_sru.py`；该包是 rsl_rl 的 SRU 分支（BSD-3），**必须安装**（Isaac Lab 自带 rsl-rl-lib 不能训练本任务）。
- 注意：所谓"range image"在策略输入里实为组织化点云端点向量（656 点 × XYZ，按机体偏航系归一化），不是单通道距离图；ONNX 输入维 1984 = 3+3+3+3+4+656×3。

### 3.3 部署侧 ROS2（真机）

- `mid360_preprocessor`（C++）：订阅 Livox Mid-360（`/livox/lidar` CustomMsg 或 `/livox/lidar/pointcloud`）与 `/lio/robo/odom`、Super-LIO 世界点云 `/lio/cloud_world`；按训练视场重采样为 16×41 组织化 `PointCloud2` 发到 `/navigation/lidar/points`（另发局部地图 `/navigation/lidar/local_map`）；TF 目标帧 `LINK_TORSO_YAW`，无机器人 TF 时回退标定外参 `[0.1244, 0, 0.2129]` + RPY `[50,0,90]`。
- `rl_nav_policy`（Python，`node.py`/`policy.py`/`core.py`）：订阅点云、里程计与 `/goal_pose`（world 系目标），组装 1984 维观测，用 ONNX Runtime（CPU）推理循环策略（输入 `obs,h_in,c_in`，输出 `actions,h_out,c_out`，隐藏 512），tanh 后 EMA 滤波（`action_filter_alpha=0.8819`）。
- 输出接口（关键衔接）：① `/cmd_vel`（`geometry_msgs/Twist`）为**归一化速度**（裁剪到 ±1）；② `/motion/body_vel_cmd`（`interface_protocol/BodyVelCmd`）为**物理速度**，按 `body_vel_scale=[0.7,0.4,0.7]` 缩放，50Hz 定时保持发布（策略使能期间），2 秒无消息底层会自行停下。真机需先进入行走模式并按 `engineai_ros2_workspace` 的 Body Velocity Control 示例打开速度控制。
- 门控：默认策略不使能；真机 `BACK+X`（GamepadKeys BACK=6/X=4）交还本体速度，`BACK+Y`（Y=5）恢复 RL 导航；也可用服务 `/rl_nav_policy/set_policy_enabled`（`std_srvs/SetBool`）。到达目标 0.5m 内或输入超时（0.5s）自动停并复位循环状态。
- 辅助节点：`lidar_range_image` 发布 32FC1 距离图与 JET 伪彩图供 RViz 查看；`lio_robo_tf` 把 Super-LIO 里程计转成 world→机体 TF；launch `pm01_rl_nav_deploy.launch.py` 串联上述节点与 RViz。

### 3.4 仿真侧 ROS2（Gazebo Classic）

- `simulator`：Gazebo Classic + ros2_control（500Hz `pm01_sdk_pd_controller`）+ Livox Mid-360 点云桥；服务 `/pm01/reset_stand` 复位站立姿态。
- `rl_walking`：C++ 节点用 MNN 跑 PM01 行走策略（24 关节/22 动作/72 观测/15 历史），订阅 `/joint_states`、IMU、`/cmd_vel`（归一化），发布 `Float64MultiArray` 到 `/pm01_sdk_pd_controller/commands`；配置 `config/rl_walking.yaml` 与策略 `251009_221907_41000.mnn` 与 native_sdk 同源。
- 仿真版 `rl_nav_policy` 与真机版同名但实现/配置不同（用 `/joy`，MODE=6/X=2/Y=3；真机用 `/hardware/gamepad_keys`，BACK=6/X=4/Y=5）；README 明确警告两套 `rl_nav_policy`、`mid360_preprocessor` **不要混进同一个 install**。

## 4. 使用方法（可直接复制）

```bash
# ---- 训练（training/ 下；先准备 Isaac Lab 环境并克隆为 sru）----
conda create --name sru --clone env_isaaclab && conda activate sru
cd training
pip uninstall rsl-rl-lib -y
pip install -e source/engineai_rl_lab
pip install -e sru-navigation-learning

# 冒烟测试 / 正式训练 / 恢复训练
WANDB_MODE=disabled python scripts/nav/train.py --task Isaac-Navigation-PPO-PM01-v0 --num_envs 16 --headless --max_iterations 2
WANDB_MODE=disabled python scripts/nav/train.py --task Isaac-Navigation-PPO-PM01-v0 --num_envs 2048 --headless --max_iterations 6000
python scripts/nav/train.py --task Isaac-Navigation-PPO-PM01-v0 --resume --checkpoint <checkpoint.pt> --headless --num_envs 2048

# 推理 / 导出 ONNX（不指定 --checkpoint 时自动取 logs 下最新模型）
WANDB_MODE=disabled python scripts/nav/play.py --task Isaac-Navigation-PPO-PM01-v0 --num_envs 16 --export_onnx --checkpoint <checkpoint.pt>

# ---- 编译 ROS2（ros/ 下；真机树）----
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --base-paths src/deployment --build-base build/deployment --install-base install/deployment --symlink-install
source install/deployment/setup.bash

# ---- 编译 ROS2（仿真树，需把真机树的 Super-LIO/Livox 一并编入）----
PYTHONNOUSERSITE=1 colcon build --base-paths src/simulation src/deployment/Super-LIO-ros2 src/deployment/livox --build-base build/simulation --install-base install/simulation --symlink-install
source install/simulation/setup.bash

# ---- 仿真（ros/ 下，三个终端流程）----
ros2 launch rl_nav_policy pm01_rl_nav_sim.launch.py        # 1. Gazebo+行走+预处理+导航（迷宫加 world:=ground.world）
ros2 service call /pm01/reset_stand std_srvs/srv/Empty "{}" # 复位站立
ros2 launch super_lio Livox_mid360.py rviz:=true            # 2. 另开终端启动 Super-LIO
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 3.0, y: 0.0, z: 0.5}, orientation: {w: 1.0}}}"  # 3. 发目标（或用 RViz 2D Goal Pose）
# 手柄 MODE+X 手动 / MODE+Y RL 导航；无手柄可用服务：
ros2 service call /rl_nav_policy/set_policy_enabled std_srvs/srv/SetBool "{data: true}"

# ---- 真机（ros/ 下，三个终端；每个终端都 source Humble + deployment overlay）----
ros2 launch livox_ros_driver2 msg_MID360_launch.py          # 1. 雷达
ros2 launch super_lio Livox_mid360.py                       # 2. 里程计（save_map:=true 退出时存图）
ros2 launch rl_nav_policy pm01_rl_nav_deploy.launch.py      # 3. 导航（rviz:=false 可关）
# 真机需先进入行走模式并按 engineai_ros2_workspace 的 Body Velocity Control 示例打开速度控制；
# BACK+X 交还本体速度，BACK+Y 恢复 RL 导航。
```

## 5. 依赖与环境

- 训练：Ubuntu + Isaac Lab（克隆 `env_isaaclab` 环境）、Isaac Sim 4.5/5.0/5.1（`setup.py` classifiers）、Python ≥ 3.10、CUDA；`sru-navigation-learning` 必须以包名 `rsl_rl` 可导入。
- 部署：ROS2 Humble；glog、TBB、Eigen、PCL、yaml-cpp、NumPy、OpenCV、ONNX Runtime（`pip install --user onnxruntime`）；仿真额外需要 Gazebo Classic、ros2_control/ros2_controllers、xacro、joy、`ros-humble-gazebo-ros-pkgs` 等。
- 真机硬件：PM01 + Livox Mid-360；Super-LIO 提供 `/lio/robo/odom` 与 `/lio/cloud_world`；底层行走策略来自机器人体内运控（经 `/motion/body_vel_cmd` 接收速度）。
- 仓库自带大文件（非 LFS，直接提交）：`model_5999.pt`（53.8MB）、`vae_heightscan3.pth`（22.7MB）、`817.onnx`（8.5MB）、行走策略 `.mnn/.pt/.onnx`（约 4.2MB）等。

## 6. 与其他仓库的关系

- `engineai_ros2_workspace`：真机速度指令的落点。导航发 `/motion/body_vel_cmd`（BodyVelCmd），需按该仓库的 **Body Velocity Control** 示例进入行走模式并开启速度控制；运动状态机、QoS、2 秒超时语义均以该仓库接口为准。
- `engineai_robotics_native_sdk`：底层行走策略同源——`training/.../Policies/locomotion/pm01/251009_221907_41000.mnn` 与 native_sdk `assets/config/pm01_edu/rl_walking_example/policies/251009_221907_41000.mnn` 同名同大小（4,268,080 字节）；仿真 `rl_walking` 节点复刻了该策略的观测与 PD 控制逻辑。区别：native_sdk 直接接管关节控制，本仓库只发速度指令给运控。
- `engineai_rl_lab` / `engineai_amp`：训练框架层。本仓库 `training/source/engineai_rl_lab` 是独立的 Isaac Lab 扩展包（与 engineai_rl_lab 仓库同名但内容为导航专用任务），`sru-navigation-learning` 是 SRU 版 rsl_rl 分支；导航低层行走策略训练不在本仓库。
- `engineai_robotics_description`：PM01 模型来源之一；本仓库另带 `assets/pm01_edu_v2/`（URDF/mesh）与 USD 资产（LFS）。
- 外部：Super-LIO（GPLv3）提供里程计与静态地图；Livox SDK2/驱动（MIT）提供 Mid-360 数据。

## 7. 注意事项、已知问题、文档缺失点

- **LFS 大文件已在本知识库快照中拉取**：`training/source/engineai_rl_lab/.../assets/data/Robots/pm01/` 下 5 个 `.usd`（含 122.7MB 的 `serial_pm01_edu_base.usd`）均为 git-lfs 管理；本工作区已替换为真实文件并逐个 sha256 校验。若自行 clone，训练前必须 `git lfs pull`，否则 Isaac Lab 加载机器人资产失败。仓库 `.gitattributes` 只对 `*.usd` 启用 LFS，其余大文件（.pt/.pth/.onnx/.mnn）直接入库。
- 训练视场与部署预处理存在不一致：训练 RayCaster 垂直 FOV 为 −60°~0°，而部署 `mid360_preprocessor` 为 −45°~0°（两者注释都声称与训练一致），差异原因未在文档说明（未确认是否笔误或有意适配）。
- `training/logs/` 虽在 `.gitignore` 中，但快照包含一次完整训练产物（2026-08-14 的 model_5999.pt + export/policy.onnx + params），属随仓库分发的示例/发布产物。
- 真机/仿真两套 `rl_nav_policy`、`mid360_preprocessor` 同名不同实现，必须分别构建到 `build/{deployment,simulation}` 并 source 对应 overlay；混用会出问题（README 明确警告）。
- 仿真 launch 不启动 Super-LIO（代码中注释掉），需另开终端手动启动；导航策略需等行走控制器初始化（默认延迟 9~10s）。
- 仿真行走节点依赖 `ros/src/simulation/rl_walking/third_party/mnn/` 内的 MNN 头文件与 lib（`lib/*.so` 被 `.gitignore` 显式豁免，随仓库提供）。
- 机器人支持仅 PM01；T800/T800 Pro 未提及。真机验证配置为 PM01 + Livox Mid-360（README 原文）。
- `/cmd_vel` 是归一化量（±1），不能直接当 m/s 用；物理速度看 `/motion/body_vel_cmd`（×`body_vel_scale`）。默认策略不使能、无目标即停车，首次使用容易误判"节点没工作"。
- `convert_checkpoint.py` 用于旧 checkpoint（`position_embedding.penc.inv_freq`）到新位置编码（`pos_encoding`）的转换，加载旧模型报错时使用。
- 仓库快照无 .git 元数据；README 正文称项目目录为 `engineai_navigation_opensource`（与仓库名不一致，疑为文案遗留）。
