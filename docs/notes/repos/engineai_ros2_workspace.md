# engineai_ros2_workspace — 众擎 PM01/T800 系列真机 ROS2 接口与部署示例工作空间

> 来源：https://github.com/engineai-robotics/engineai_ros2_workspace ｜ 分支：community ｜ commit：c41043f64ca1（2026-08-14）｜ 快照日期：2026-10-03 ｜ 本地路径：`repos/engineai_ros2_workspace/` ｜ 许可证：BSD 3-Clause（`LICENSE`，Copyright (c) 2025- Shenzhen Zhongqing Robot Technology Co., Ltd. ("EngineAI")）

## 1. 定位与用途

- ROS2 Humble 工作空间，包含 **EngineAI 机器人对外 ROS2 接口协议（`interface_protocol`）**、示例代码（`interface_example`）、主机端 MuJoCo 仿真（`simulation/mujoco`）、预编译第三方库（`third_party`）和音频示例（`audio_example`）。
- 核心用途：把训练好的行走策略部署到真机（典型流程 pd_stand → joint_bridge → `rl_basic_example`），以及为上层应用/导航提供关节、速度、上肢、LED、状态机等标准接口。
- 支持机型（以配置目录和示例统计）：**PM01**（24 关节 J00–J23）、**T800**（25 关节）、**T800 Pro**（43 关节 J00–J42）；仿真资产只有 `pm_v2`（PM01 v2），且仿真仅支持关节桥接（joint_bridge）模式。
- 官方文档指针：README 指向飞书「EngineAI 开源知识库」；仓库内的接口说明在 `src/interface_protocol/README.md` 与 `new_msg_dsc.md`，对应官网 ros2-interface 栏目（见第 6 节）。

## 2. 目录结构

```text
engineai_ros2_workspace/
├── README.md / README_CN.md        # 中英文指南（构建/示例/板载部署；两版内容有差异）
├── scripts/
│   ├── build_nodes.sh              # colcon 构建入口：msg/example/app/sim/web/all
│   ├── sync_src.sh                 # rsync src/scripts 到 nezha 或 orin 板子
│   ├── install.sh                  # 把 install/ 部署到板子的 apps 目录
│   └── check_format.sh             # clang-format 格式检查
├── src/
│   ├── interface_protocol/         # ROS2 消息/服务定义包（接口协议核心）
│   │   ├── msg/                    # 19 个 .msg
│   │   ├── srv/                    # EnableMotor.srv（唯一服务）
│   │   ├── README.md               # 11 个 topic 的接口表（英文/代码引用为准）
│   │   └── new_msg_dsc.md          # 中文接口表（与 README 基本一致）
│   ├── interface_example/          # 示例包：C++ 可执行 + Python 脚本 + YAML 配置 + launch
│   │   ├── src/                    # rl_basic_example.cc / joint_test_example.cc / hold_joint_status.cc
│   │   ├── scripts/                # 关节桥接、速度、LED、运动规划、上肢覆盖、状态切换示例
│   │   ├── scripts/t800pro/        # T800 Pro 灵巧手（loose_fist）示例
│   │   ├── config/{pm01,t800,t800pro}/  # 各机型 YAML 与 pm01 的 mnn 策略
│   │   └── launch/rl_basic_example.launch.py
│   ├── simulation/mujoco/          # 主机端 MuJoCo 仿真包（mujoco_simulator）
│   │   ├── launch/                 # mujoco_simulator.launch.py
│   │   ├── assets/config/pm_v2.yaml、assets/resource/  # pm_v2 模型（urdf/xml/meshes）
│   │   ├── src/ include/           # ROS 接口、YAML 配置加载、仿真管理
│   │   └── simulate/               # MuJoCo 官方 simulate 可视化前端源码
│   ├── third_party/                # 预编译库 tar 包 + install.sh/make.sh（MNN/MuJoCo/Eigen/yaml-cpp）
│   └── audio_example/              # 非 ROS2 音频示例（pyaudio/pygame/pactl）
├── docs/images/                    # pm01.jpg、sim2sim.gif 等展示图
└── LICENSE                         # BSD 3-Clause
```

## 3. 核心模块与关键文件

### 3.1 interface_protocol（接口协议包）

- 定义 **19 个 msg + 1 个 srv**；`package.xml` 依赖 std_msgs/std_srvs/geometry_msgs，`rosidl_interface_packages` 组。
- 官方 README 表（`src/interface_protocol/README.md`）文档化的 11 个 topic：

| Topic | 消息 | 方向 | 典型示例 |
|---|---|---|---|
| `/motion/joint_override_command` | JointOverrideCommand | 发布 | upper_joint_override_example.py |
| `/motion/set_motion_state` | MotionStateRequest | 发布 | switch_to_terrain_walking_gait_example.py |
| `/motion/motion_state` | MotionState | 订阅 | 多处（当前状态 + 可用切换） |
| `/hardware/led_control` | LedControl | 发布 | led_control_example.py |
| `/motion/joint_motion_plan/request` | JointMotionPlanRequest | 发布 | joint_mutiple_motion_plan_example.py |
| `/motion/joint_motion_plan/state` | JointMotionPlanState | 订阅 | joint_mutiple_motion_plan_example.py |
| `/hardware/joint_state` | JointState | 订阅 | joint_bridge_example.py |
| `/hardware/joint_command` | JointCommand | 发布 | joint_bridge_example.py、joint_test_example.cc |
| `/motion/body_vel_cmd` | BodyVelCmd | 发布 | body_velocity_control_example.py |
| `/hardware/gamepad_keys` | GamepadKeys | 订阅 | rl_basic_example.cc |
| `/hardware/imu_info` | ImuInfo | 订阅 | rl_basic_example.cc |

- 关键消息字段：
  - `JointCommand`：`position/velocity/feed_forward_torque/torque/stiffness/damping` + `parallel_parser_type`（0=CLASSIC_PARSER，1=RL_PARSER）；真机推荐 100–500 Hz、Best Effort。
  - `MotionState`：`current_motion_task` + `available_transition_motions`；`MotionStateRequest` 只需 `target_motion_name`。
  - `JointMotionPlanRequest`：常量 `REQUEST_PLAN_EXECUTE=0 / REQUEST_CANCEL=1 / REQUEST_RESET=2`，含 `request_id`、`joint_indices`、`target_positions/velocities`、`execution_time`、`stiffness/damping`、`use_gravity_compensation`；`JointMotionPlanState` 状态 `DISABLED=0/IDLE=1/EXECUTING=2/EXITING=3` 与 `progress`。
  - `LedControl`：0x1–0xb 共 11 种灯效；`BodyVelCmd`：`linear_velocity`（x,y）+ `yaw_velocity`。
- 其余消息：`Alert`、`Heartbeat`、`MotorCommand`、`MotorState`、`MotorDebug`、`PowerInfo`、`Tts`、`ParallelParserType`。本快照中仅 `ParallelParserType` 被 `rl_basic_example.cc` 使用；其余无发布/订阅方，topic 归属未确认（疑为固件/内部预留）。
- 服务 `EnableMotor.srv`（`bool enable` → `bool success`/`string message`）在示例中无调用，属预留。
- 示例体现的 QoS 约定：状态订阅用 BEST_EFFORT/VOLATILE，切换/规划请求用 RELIABLE/VOLATILE。

### 3.2 interface_example（示例包）

- CMake 安装 3 个可执行：`hold_joint_status`（读取并保持当前关节位，500 Hz 发布）、`joint_test_example`（C++ 版关节测试，参数为配置目录，读取 `joint_test.yaml`）、`rl_basic_example`（MNN 策略推理，链接 `/opt/engineai_robotics_third_party/lib/libMNN.so`、yaml-cpp、Eigen）。
- Python 脚本（`<repo>/src/interface_example/scripts/`）：

| 脚本 | 接口 | 默认配置/参数 |
|---|---|---|
| `joint_bridge_example.py` | 订阅 /hardware/joint_state、/motion/motion_state；发布 /hardware/joint_command | `-f` 默认 `config/pm01/joint_test.yaml`，500 Hz 线性插值 |
| `body_velocity_control_example.py` | 发布 /motion/body_vel_cmd | 100 Hz，示例 vx=0.5 m/s |
| `led_control_example.py` | 发布 /hardware/led_control | 每 1 s 轮换灯效 |
| `switch_to_target_motion_example.py` | /motion/set_motion_state + /motion/motion_state | `-m/--target-motion` |
| `switch_to_terrain_walking_gait_example.py` | 同上 | `-m` 默认 `rl_terrain` |
| `joint_mutiple_motion_plan_example.py` | /motion/joint_motion_plan/request + /state（注释误写 Service） | `-f` 默认 `config/pm01/motion_plan_wave_hands.yaml` |
| `upper_joint_override_example.py` | /motion/joint_override_command + /motion/motion_state | `-f` 默认 `config/pm01/upper_joint_override.yaml`，五次多项式插值 |
| `t800pro/loose_fist_example.py` | /hardware/joint_command + /hardware/joint_state + /motion/motion_state | `--pose extend\|loose_fist\|both --hand left\|right --exec-time --hold-time --rate`，43 关节，手部索引 20–26/34–40 |

- `rl_basic_example`：读取 `config/<product>/rl_basic/basic/rl_basic_param.yaml`（42 维观测、15 帧历史、control_dt=0.01、默认策略 `policies/pm01_v2_rough_ppo_42obs.mnn`、active_joint_idx 0–11 为腿部），等待 `motion_state == "joint_bridge"` 后以手柄/IMU/关节状态驱动策略；launch 用环境变量 `PRODUCT` 选产品，默认 `pm01`。
- 机型配置目录：`pm01`（joint_test、pd_joint_test、motor、motion_plan_reset/shake_hand/wave_hands、upper_joint_override、rl_basic+策略）；`t800`（joint_test、pd_joint_test、motion_plan_*、upper_joint_override，25 关节）；`t800pro`（joint_test、pd_joint_test、motion_plan_*、upper_joint_override，43 关节含灵巧手）。

### 3.3 simulation/mujoco（主机端仿真包）

- 节点 `mujoco_simulator`（`use_sim_time=true`）：发布 `/hardware/joint_state`、`/hardware/imu_info`、`/motion/motion_state`；订阅 `/hardware/joint_command`；以 `mjcb_control` 回调做 PD+前馈力矩控制：`tau = feed_forward + kp*(q_cmd-q) + kd*(qd_cmd-qd)`。
- 仿真内 `/motion/motion_state` 恒为 `"joint_bridge"`，`available_transition_motions` 为空 → 仅能测试关节桥接类接口。
- 配置 `assets/config/pm_v2.yaml`：`urdf: serial_pm_v2.urdf`、`xml: pm_v2.xml`、`num_total_joints: 24`；topic 名也写在 YAML（sensor/actuator 段）。launch 设置 `PRODUCT=pm_v2`、`MUJOCO_ASSETS_PATH`、`LD_LIBRARY_PATH`。

### 3.4 scripts 与 third_party

- `build_nodes.sh` 目标：`msg`（interface_protocol）、`example`（interface_protocol+rdk_framework+interface_example）、`sim`（mujoco_simulator+interface_protocol+interface_example）、`app`/`web`/`all`（引用大量本快照不存在的应用包，见第 7 节）；colcon 参数 `--allow-overriding interface_protocol`。
- `sync_src.sh`：内置两台板子——`nezha` = `user@192.168.0.163`（密码 1，路径 `/home/user/source/engineai_workspace`）；`orin` = `ubuntu@192.168.0.162`（ubuntu/ubuntu，路径 `/home/ubuntu/source/engineai_workspace`）。`install.sh` 把 `install/` 推到板子 `/home/{user,ubuntu}/apps/install`。
- `third_party/install.sh`：解压预编译库到 `/opt/engineai_robotics_third_party`（含 MuJoCo、MNN、Eigen、yaml-cpp 的 include/lib）；`make.sh` 支持 build/install/pack/clean，源包 `engineai_robotics_third_party_src.tar.xz` 用于许可证合规。

## 4. 使用方法

```bash
# 0) 主机依赖（Ubuntu 22.04 + ROS2 Humble）
sudo apt update
sudo apt install rsync sshpass openssh-client libglfw3-dev libxinerama-dev libxcursor-dev
sudo apt install ros-dev-tools ros-humble-rmw-cyclonedds-cpp ros-humble-ros-base

# 1) 环境变量（写入 ~/.bashrc）
export ROS_DOMAIN_ID=69
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 2) 预编译第三方库（解压到 /opt）
./src/third_party/install.sh

# 3) 构建并 source
./scripts/build_nodes.sh example      # 协议 + 示例
./scripts/build_nodes.sh sim          # 仿真 + 协议 + 示例
source install/setup.bash

# 4) 主机仿真（仅 joint_bridge 接口；务必与真机隔离）
export ROS_LOCALHOST_ONLY=1           # 仿真时禁止误连真机
ros2 launch mujoco_simulator mujoco_simulator.launch.py

# 5) 真机连接检查（机器人内网 192.168.0.0/24，用主板自带网口）
ping 192.168.0.163

# 6) 常用示例（真机需先进入相应模式）
python src/interface_example/scripts/body_velocity_control_example.py          # 行走模式
python src/interface_example/scripts/joint_bridge_example.py -f src/interface_example/config/pm01/joint_test.yaml
python src/interface_example/scripts/led_control_example.py
python src/interface_example/scripts/switch_to_target_motion_example.py -m lower_body_balance
python src/interface_example/scripts/joint_mutiple_motion_plan_example.py -f src/interface_example/config/pm01/motion_plan_wave_hands.yaml
python src/interface_example/scripts/upper_joint_override_example.py -f src/interface_example/config/pm01/upper_joint_override.yaml
python src/interface_example/scripts/t800pro/loose_fist_example.py --hand left --pose loose_fist

# 7) RL 行走策略部署（真机：pd_stand → joint_bridge，确认后再启动）
ros2 topic echo /motion/motion_state   # 确认 current_motion_task == joint_bridge
ros2 launch interface_example rl_basic_example.launch.py
export PRODUCT=pm01                    # launch 通过 PRODUCT 选配置，默认 pm01

# 8) 同步到板子并在板载编译
./scripts/sync_src.sh orin
# 登录 Orin 后：
cd ~/source/engineai_workspace && ./scripts/build_nodes.sh
```

## 5. 依赖与环境

- **OS/ROS**：Ubuntu 22.04、ROS2 Humble Desktop、GCC ≥ 11、CMake ≥ 3.22、Python ≥ 3.10；中间件 CycloneDDS（`rmw_cyclonedds_cpp`）。
- **第三方库**（预编译包解压到 `/opt/engineai_robotics_third_party`）：MuJoCo、MNN（策略推理）、Eigen 3.4.0、yaml-cpp；CMake 通过 `CMAKE_PREFIX_PATH` 引用。
- **计算单元**：运控单元 Nezha `192.168.0.163`（user/1）；应用单元 Orin `192.168.0.162`（ubuntu/ubuntu）；机器人内网 192.168.0.0/24。
- **机型自由度**：PM01 24、T800 25、T800 Pro 43（来自各机型 YAML/示例）；仿真模型只有 pm_v2（24）。
- Python 脚本依赖：rclpy、numpy、pyyaml；音频示例依赖 numpy、pyaudio、pygame + `pactl`（非 ROS2）。
- README 提到的 RL 训练框架：`engineai-robotics/engineai_gym`（开源）；对应本知识库中的 `engineai_rl_lab`。

## 6. 与其他仓库的关系

- **engineai_gym / rl_lab（训练）→ 本仓库（部署）**：训练导出 `.mnn` 策略后，放入 `src/interface_example/config/<product>/rl_basic/basic/policies/`，按 `rl_basic_param.yaml` 对齐观测项、关节顺序、kp/kd、action_scale，再经 `rl_basic_example` 部署；sim2sim 可用 `simulation/mujoco`（仅 pm_v2、joint_bridge）。
- **native_sdk（原生 SDK/内部调度）**：本仓库是对外 ROS2 接口层；官方 native-sdk 文档记载其 ROS2 bridge 还涉及 `/hardware/motor_state`、`/hardware/motor_command`、`/hardware/joint_command_feedback` 等话题，与本仓库 `interface_protocol` 有交叠但不等价；本快照不含 native_sdk 源码。
- **simulation_runtime（容器化仿真）**：Docker Compose 一键启动 motion_control + MuJoCo + Virtual Gamepad，一个镜像含 pm_v2/t800/t800pro；与本仓库 `src/simulation/mujoco`（源码编译、单机型、单模式）互为补充。两者 ROS2 接口风格一致（/motion、/hardware 前缀，domain 69）。
- **官方 ros2-interface 栏目对应关系**（对照本知识库 `docs/t800-online/ros2-interface/` 10 章，快照未含网页原件，链接为本地存档）：
  - 01 概述与开发准备 → 本仓库 README（环境变量/构建/SSH）
  - 02 运动状态机 → `/motion/motion_state`、`/motion/set_motion_state` + switch_* 示例
  - 03 关节命令与全身控制 → `/hardware/joint_command|joint_state`、joint_bridge/joint_test/rl_basic
  - 04 机体速度控制 → `/motion/body_vel_cmd` + body_velocity_control_example.py
  - 05 上肢关节运动规划 → `/motion/joint_motion_plan/request|state` + joint_mutiple_motion_plan_example.py
  - 06 上肢关节覆盖控制 → `/motion/joint_override_command` + upper_joint_override_example.py
  - 07 LED 控制 → `/hardware/led_control` + led_control_example.py
  - 08 视觉传感器 → 本仓库无驱动（RealSense/Odin 由官方文档与独立仓库提供）
  - 09 音频录制与播放 → `src/audio_example/`（非 ROS2）
  - 10 附录 → `interface_protocol/README.md`、`new_msg_dsc.md` 的消息定义

## 7. 注意事项、已知问题、文档缺失点

- **README 与代码不一致**：
  - 中英文 README 都给出 `python src/interface_example/scripts/joint_test_example.py`，但该 Python 文件不存在；对应功能是 C++ 可执行 `joint_test_example` 和 `joint_bridge_example.py`。
  - 英文 README 的 "RL Walking Policy Deployment Example" 小节写成了 joint_test_example 流程（与中文版不一致）。
  - `build_nodes.sh` 的 `app`/`web`/`all` 目标引用的 `src/applications/*`、`rdk_framework`、`robot_manager`、`sound_hub`、`system_monitor`、`data_recorder`、`dashboard`、`foxglove_studio` 均不在本快照，只有 `msg`/`example`/`sim` 可构建。
  - `t800pro/loose_fist_example.py` 头部注释要求 `source scripts/setup_host_ros_env.bash` 和 `build/x86_64/_install/.../local_setup.sh`，两文件在本仓库不存在（疑似内部构建产物路径）。
  - `joint_mutiple_motion_plan_example.py` 文档字符串与注释称 "Service"，实际走 topic 实现（与 README 表一致）。
  - `config/pm01/motor.yaml`、`pd_joint_test.yaml` 在仓库中没有任何脚本引用，用途未确认。
- **机型覆盖不均**：只有 `config/pm01` 带 `rl_basic` 配置与 `.mnn` 策略；t800/t800pro 无 RL 示例。仿真资产只有 pm_v2，且仿真只提供 `joint_bridge` 状态（`/motion/motion_state` 恒定值、无可用切换列表）。
- **安全**：仿真与真机默认同一 `ROS_DOMAIN_ID=69`，README 明确要求仿真时设 `ROS_LOCALHOST_ONLY=1` 或断开真机；不要使用 USB 网卡；RL 示例有安全距离与急停提醒。
- **脚本内置明文密码/固定 IP**（user/1、ubuntu/ubuntu），仅适用于官方机器人内网，勿在公网复用。
- `third_party` 直接随仓库分发预编译与源码 tar 包（体积大，注意 LFS/克隆耗时）；组件许可证在 src 包内逐项查看。
- 无 `.action` 定义：全部为 topic + 唯一 srv（EnableMotor，未被示例使用）。
- 本快照非 git 仓库（无 `.git`），commit 信息按任务给定记录，无法本地校验。
