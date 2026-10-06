# engineai_robotics_native_sdk — EngineAI 人形机器人原生控制 SDK（预编译调度内核 + 可扩展 Runner，真机部署 RL 行走策略）

> 来源：https://github.com/engineai-robotics/engineai_robotics_native_sdk ｜ 分支：main ｜ commit：335c60e88772（2026-08-04）｜ 快照日期：2026-10-03 ｜ 本地路径：`repos/engineai_robotics_native_sdk/` ｜ 许可证：BSD 3-Clause（`LICENSE.txt`）；`src/protocol/interface_protocol/` 同为 BSD-3；`tools/virtual_gamepad/` 单独为 GPL-3.0-or-later

## 1. 定位与用途

- EngineAI 官方 C++ 原生控制 SDK，官方定位"机器人内部神经系统"，面向算法/控制/RL 工程师；用于人形机器人应用开发与系统集成、真机部署行走/舞蹈策略、编写自定义全身控制器。
- 核心能力：预编译实时调度内核（`core/`）+ 配置驱动的任务编排（Task/Runner）+ 模块化插件（`src/runner/*`）+ MuJoCo 仿真 + 真机部署脚本 + ROS2 数据桥 + PlotJuggler 可视化。
- 对应官方文档 `docs/t800-online/native-sdk/` 四页：01 介绍、02 开发环境与快速开始、3 源码解析、4 自定义 Runner；本仓库即该文档所述源码。
- 支持机型（按 `assets/config/` 目录）：`pm01_edu`、`t800`、`t800pro`（含腕部/灵巧手关节）；RL 策略以 MNN 格式随仓库分发。

## 2. 目录结构（主要 2-3 层）

```text
engineai_robotics_native_sdk/
├── assets/
│   ├── config/<robot>/            # 各机型配置（pm01_edu / t800 / t800pro）
│   │   ├── mode.yaml              # 配置入口：active_mode + tag→scope 映射
│   │   ├── task_resident/*.yaml   # 常驻任务（imu/motor/估计器/ros2_bridge…，sim/robot 两套）
│   │   ├── task_motion/*.yaml     # 运动状态机（idle/passive/pd_stand/walk/dance…）
│   │   ├── <runner名>/default.yaml# Runner 参数（策略路径、PD 增益、观测缩放等）
│   │   └── model/motor/parallel_ankle/parallel_palms/lcm/ros2_bridge/global_options/…
│   └── resource/
│       ├── robot/<robot>/{urdf,xml,meshes,texture}  # 与真机一致的 URDF / MuJoCo XML / 网格
│       ├── environment/{ground,chair}.xml           # 仿真环境
│       └── <robot>.xml                              # MuJoCo 顶层场景
├── core/                          # 预编译核心（只读，见 §3.1）
│   ├── include/                   # 框架头文件（basic/data/math/model/parameter/tool/app）
│   ├── lib_1.0.0_{humble_amd64,humble_arm64,jazzy_arm64}/  # 三套预编译 .so
│   ├── registry/auto_register_runners.cc   # 内核 Runner 注册清单
│   └── cmake/EngineAICore*.cmake           # find_package(EngineAICore) 导出配置
├── src/                           # 业务源码（可改）
│   ├── executor/main.cc           # 程序入口（Crashpad + App::RunMain）
│   ├── runner/                    # Runner 插件：idle/passive/pd_stand/input_command_arbiter/
│   │                              #   rl_walking_example/rl_dance_example/rl_lab/
│   │                              #   rl_mimic_trajectory_example/ros2_bridge
│   ├── data/_param/               # 各 Runner 的 YAML 参数类（LOAD_PARAM 宏）
│   ├── hardware/{gamepad,rc02}/   # 手柄/遥控器驱动示例
│   ├── protocol/interface_protocol/  # ROS2 msg 包（19 msg + 1 srv）
│   └── ros2_node/                 # ROS2 节点（hardware_interface_node / manager_node 等）
├── simulation/mujoco/             # MuJoCo 仿真模块（simulate 前端 + lcm_interface）
├── scripts/                       # build_mujoco/run_mujoco/run_robot/cross_compile_env/plotjuggler
├── docker/generate.sh             # 生成容器快捷入口 engineai_robotics_env
├── tools/virtual_gamepad/         # PyQt6 虚拟手柄（GPL-3.0）
├── build.sh / run.sh / install.sh / env.sh / clear.sh
└── cmake/AutoRegisterRunners.cmake# 扫描 REGISTER_RUNNER 宏自动生成注册代码
```

## 3. 核心模块与关键文件

### 3.1 src/ 与 core/ 的分工（哪些可改、哪些预编译）

- `core/` 是**预编译框架内核**：只有头文件 + 三套 .so + CMake 导出文件，**没有 core 源码，不应直接修改**。`core/registry/auto_register_runners.cc` 声明内核已注册的 7 个 Runner：imu、joint_motor_transform、motor、param_server、sim_publish、sim_subscribe、whole_body_estimate（对应 `libsrc_runner_*.so`，另有估计器/数学/模型/数据/任务库）。
- `src/` 是**可编译修改的业务层**：executor 入口、全部示例 Runner、参数类、硬件驱动示例、ROS2 msg/节点；`src/CMakeLists.txt` 逐目录 add_subdirectory，每个 Runner 编成 `src_runner_*` 共享库。
- 构建时 `CMakeLists.txt` 检测到 `core/cmake/EngineAICoreConfig.cmake` 即进入 `SDK_FRAMEWORK_MODE`，`find_package(EngineAICore)` 链接预编译库并把 core 的 .so 装进 `_install/lib`；`build.sh` 按 `ros_distro:arch` 选择 `core/lib_*` 目录并创建 `core/lib` 软链（快照中无 `core/lib`，为构建产物）。
- 注意：`libsrc_data.so`、`libsrc_task.so`、`libsrc_app.so` 也来自 core，即任务调度与数据总线本体是预编译的；业务扩展点主要是 Runner 与配置。
- 扩展机制：Runner 头文件里的 `REGISTER_RUNNER(类名, "runner名", kMotion|kResident)` 宏导出 `extern "C"` 工厂函数，`cmake/AutoRegisterRunners.cmake` 扫描源码自动生成注册文件并链接对应库；环境变量 `ENGINEAI_ROBOTICS_USED_RUNNERS=xxx_runner,...` 支持选择性编译/裁剪 Runner。

### 3.2 Runner 机制与生命周期

- Task 是调度单元（CPU、优先级、周期、`runner[]` 列表）：`task_resident/*.yaml` 常驻并行（input_command_arbiter、imu、whole_body_estimate、motor、ros2_bridge）；`task_motion/*.yaml` 同一时刻只激活一个运动状态，按键手动迁移或 `auto_transition` 自动迁移。
- Runner 生命周期（`core/include/basic/basic_runner.h`）：`Initialize()`（构造时一次）→ `Enter()`（进入任务，加载参数/模型）→ `Run()`（周期主逻辑）→ `TryExit()`（可跨多周期退场，返回 `kTrying/kCompleted/kFailed`）→ `Exit()` → `End()`。
- `MotionRunner`（`core/include/basic/motion_runner.h`）额外有 `SetupContext()/TeardownContext()`（首尾包裹），并用 `GetMutableOutput().SetCommand(q_des, qd_des, kp, kd, tau_ff)` 写本周期输出缓存；Task Manager 汇总所有激活 Runner 的输出，做插值/奇异点/扭矩限制后统一下发电机。
- 状态枚举：`RunnerState`（kRunning/kTryExit/kFault）、`TransitionState`（kCompleted/kTrying/kFailed/kRestoreRunning）。
- 输入仲裁：`src/runner/input_command_arbiter` 统一管理输入源，已注册 gamepad、virtual_gamepad、rc02 适配器，按注册顺序低→高优先级仲裁，结果写入 `data_store_->gamepad_info`。

### 3.3 配置系统（assets/config 结构、mode/task/runner 参数）

- 入口 `assets/config/<robot>/mode.yaml`：`active_mode: sim|robot` 选择整套 scope；`mode.<sim|robot>` 是 `tag → scope` 映射列表（如 `tag: pd_stand → scope: pd_stand/robot`）。快照中三机型 `active_mode` 均为 `sim`，真机部署前需改 `robot`（`install.sh` 会 ssh sed 自动改远端值）。
- 加载链路（对应官方 native-sdk/3）：`BasicParam(tag)` 按 tag 找到 scope → 参数类用 `LOAD_PARAM(字段)` 从 YAML 读取；`ParamManager::create<T>(tag)` 动态创建/重载参数实例。
- Task 参数：`task_motion/default.yaml` 中每个 motion 定义 `period`（walk 0.01s、dance 0.02s、站立类 0.002s）、`runner[].name`（须与 REGISTER_RUNNER 名字一致）、`param_tag`、`manual_transition`、`auto_transition`、`key`；t800pro 另有 `transition_rules`（可 `skip_transitions: [passive]`）。
- Runner 参数（以 `rl_walking_example/default.yaml` 为例）：`policy_file`（相对 config 目录）、`num_observations`、`active_joint_names`、`num_include_obs_steps`、观测缩放/裁剪、`default_joint_q`/`joint_kp`/`joint_kd`/`action_scale`（按肢体分组的二维数组，运行时展平）、`control_dt`、手柄指令缩放 `command_scale_pos/neg`、指令低通 `enable_remote_command_lpf`、`imu_install_bias`；t800/t800pro 行走配置另有启动过渡 `transition_time` 与 `disable_transition_joints`。
- 其他模块配置：`model/default.yaml`（URDF/MuJoCo XML、floating base、肢体关节分组、末端帧、接触点）、`motor/default.yaml`（电机类型/使能/符号/偏置/最小刚度阻尼）、`ros2_bridge/default.yaml`（激活节点、周期、topic 映射）、`parallel_ankle`/`parallel_palms`（并联踝/掌解析）、`lcm`（多播 239.255.76.67:7667）、`global_options`（`strict_motion_check`）。

### 3.4 RL 行走策略部署流程

- 策略文件位置：`assets/config/<robot>/rl_walking_example/policies/*.mnn`（pm01_edu、t800）或 `policy/*.mnn`（t800pro，目录名不一致）。仓库自带可用策略（均真实二进制，非 LFS 指针）：pm01_edu `policies/251009_221907_41000.mnn`（4.27MB）、t800 `policy/t800_260618_165257_30000.mnn`、t800pro `policy/t800_260318_150533_60000.mnn`。
- 切换策略：① 覆盖同名 .mnn；② 修改 `policy_file` 字段；③ 利用 `param_tag` 热切换（`Enter()` 检测 tag 变化后用 `ParamManager::create<T>(tag)` 重载，对应 mode.yaml 的不同 tag→scope）。
- 部署链路：`RlWalkingExamplePm01Runner`（继承 MotionRunner）→ `SetupContext()` 置 `parallel_by_classic_parser=false`（策略接管全部主动关节）→ `Enter()` 用 `math::MNNModel` 加载 .mnn、建立策略关节→全身边界索引映射、记录进入时关节位置与 PD 增益 → `Run()` 每周期：读关节位置/速度 → 手柄映射速度指令（左右摇杆 + 正负缩放 + 可选一阶低通）→ 组装观测 → MNN 前向 → 动作裁剪 → `q_des = default_q + action*action_scale` → `GetMutableOutput().SetCommand(...)`。
- 观测契约（必须与训练侧一致）：单步 72 维 = 关节位置相对 default 的偏差(22) + 关节速度(22) + 上一步动作(22) + 机体系角速度(3) + 重力投影(3)；15 步历史展平 + 3 维速度指令拼在尾部，策略输入 1083 维（72×15+3）。动作输出 22 个主动关节（不含腰/头；t800pro 右臂编号 J27–J31）。
- 控制方式：纯 PD 位置控制（qd_des=0、tau_ff=0），启动阶段按 `transition_time` 线性混合进入时的 PD 指令；`disable_transition_joints` 中的踝关节直接用策略目标。`TryExit()` 在 joint_override 仍使能时持续 `Run()` 并返回 kTrying，保证退场不失控。
- 其他策略示例：`rl_dance_example`（dance.mnn + dance.npz 轨迹）、`rl_lab`（部署 engineai_amp 训练产物，pm01_edu 78 维观测）、`rl_mimic_trajectory_example`（t800 起身/躺倒，19.8MB mnn + npy/csv 参考轨迹 + residual 控制）。

### 3.5 支持机型

| 机型 | 配置目录 | 状态机运动任务 | 备注 |
|---|---|---|---|
| PM01（EDU） | `assets/config/pm01_edu/` | idle/passive/pd_stand/walk/rl_lab/dance | 24 关节；并联踝；rl_lab 对应 engineai_amp |
| T800 | `assets/config/t800/` | 上述 + supine_to_stance/stance_to_supine | 25 关节（腰+头）；含起身/躺倒与舞蹈 |
| T800 Pro | `assets/config/t800pro/` | idle/passive/pd_stand/walk | 43 关节（双臂 7 含腕、双手各 7）；额外 parallel_palms 与 idle 配置 |

## 4. 使用方法（可直接复制）

```bash
# 容器开发环境（生成后新终端执行 engineai_robotics_env 进入）
cd repos/engineai_robotics_native_sdk
./docker/generate.sh

# 编译（默认 x86_64 + ROS2 Humble；产物 build/x86_64/_install）
engineai_robotics_env
./build.sh                                # 全量；-t release/debug、-j 核数
./build.sh -m runner_rl_walking_example   # 单模块增量（需已有 CMakeCache）
./build.sh -a aarch64                     # 交叉编译（宿主机执行，自动拉 cross_compile_env 镜像）
./build.sh -a aarch64 -r jazzy            # Jazzy 交叉编译
./clear.sh                                # 清空 build/

# 运行（入口 src_executor；<robot> 与 assets/config 目录名对应）
./run.sh pm01_edu

# MuJoCo 仿真（先确认 assets/config/<robot>/mode.yaml 的 active_mode: sim）
./scripts/build_mujoco.sh                 # GitHub 不通时用 ./scripts/build_mujoco.sh -m 走 Gitee 镜像
./scripts/run_mujoco.sh pm01_edu

# 真机部署（先编辑 install.sh 的 remote_user/remote_host/remote_dir，默认 user@192.168.0.163:~/projects/engineai_robotics）
./install.sh pm01_edu robot               # rsync 产物+assets 到运控单元，并自动把远端 active_mode 改为 robot
ssh user@192.168.0.163
sudo systemctl stop robotics.service
cd ~/projects/engineai_robotics && sudo ./run_robot.sh pm01_edu
# 后台运行：nohup sudo ./run_robot.sh pm01_edu > nohup.out 2>&1 & tail -f nohup.out

# 数据监测（ROS2 + PlotJuggler）
source build/ros2_env/install/setup.bash
./scripts/run_plotjuggler.sh              # 加 remote 参数可多机调试
# layout 文件：scripts/plotjuggler/common_data_display.xml

# 虚拟手柄（无实体手柄时；F5/Enable 后屏蔽实体手柄）
python3 tools/virtual_gamepad/virtual_gamepad.py
```

- 按键（Logitech F710 Xbox 模式，PM01/T800 通用）：idle→passive `LB+RB`；passive→pd_stand `LB+A`；pd_stand→walk `LB+B`；pd_stand→dance `LB+CROSS_X_DOWN`；全局 `LB+RB` 强制回 passive（软急停）；T800 另有 `START+CROSS_X_UP/DOWN` 起身/躺倒。
- 真机安全：建议吊架下进入 pd_stand 后再落地、再切行走；运行前用急停遥控器使能电机；`run_robot.sh` 会检查 `motor_service_server`/`src_executor` 占用、`chmod 666 /dev/ttyACM0`、source ROS 并设 `ROS_DOMAIN_ID=69`。

## 5. 依赖与环境

- 官方容器镜像 `ghcr.io/engineai-robotics/engineai_robotics_env:latest`（Ubuntu + ROS2 Humble 桌面版）；交叉编译镜像分 humble/jazzy 两个 tag。`docker/generate.sh` 默认不开 GPU 直通，仿真提帧需改 `NVIDIA_GPU_AVAILABLE=y` 重新生成。
- 构建：CMake ≥ 3.16、C++20；依赖 GTest、glog、yaml-cpp、fmt、Eigen3、lcm、pinocchio、qpOASES；ROS2 组件需要 fastcdr；MNN、Crashpad 等第三方库位于容器内 `/opt/engineai_robotics_third_party`，硬件抽象库在 `/opt/engineai_robotics_hardware`（环境变量见 `env.sh`/`run_robot.sh`）。
- 支持 ROS2 Humble（x86_64/arm64）与 Jazzy（arm64）三种 lib 组合；`run.sh` 只跑本机 `build/x86_64/_install`，aarch64 产物用 `install.sh <robot> robot aarch64` 下发。
- 机器人机型：pm01_edu、t800、t800pro；控制器：Logitech F710（Xbox 模式）、虚拟手柄、rc02（`src/hardware/rc02`，是否在默认仲裁链中启用未确认）。

## 6. 与其他仓库的关系

- `engineai_ros2_workspace`：对外 ROS2 接口层（运动状态机、joint_command、body_vel_cmd 等）。本仓库 `src/protocol/interface_protocol` 与其同源；native_sdk 的 ros2_bridge 默认发布 `/hardware/{imu_info,gamepad_keys,motor_debug,power_info,motor_state,motor_command,joint_state,joint_command_feedback}`、订阅 `/hardware/led_control` 与 `/motion/node_control`，与对外文档的 13 个 topic 重叠但不等价。注意本仓库 ros2_bridge 配置**未订阅** `/motion/body_vel_cmd`，行走示例的速度来源是手柄而非 ROS2 速度话题。
- `engineai_rl_lab` / `engineai_legged_gym` / `engineai_amp`：训练侧。`rl_lab` Runner 即部署 engineai_amp 训练产物（pm01_edu）；`rl_walking_example` 接收 MNN 策略（训练仓库一般导出 ONNX→MNN）；训练侧观测/关节顺序必须与部署配置 `active_joint_names`、`num_observations`、缩放参数完全一致。
- `engineai_robotics_description`：机器人模型仓库；本仓库 `assets/resource/robot/<robot>/urdf/serial_*.urdf` 与 MuJoCo XML/mesh 是 SDK 自带版本，用于仿真与模型加载，真机资产随 `install.sh` 同步。
- `engineai_navigation`：导航仓库复用同一份 PM01 行走策略（其 `Policies/locomotion/pm01/251009_221907_41000.mnn` 与本仓库 pm01_edu 策略文件同名同大小 4,268,080 字节）；其仿真 `rl_walking` 节点移植了本仓库行走策略的观测/PD 逻辑（24 关节/22 动作/72 观测/15 历史）。

## 7. 注意事项、已知问题、文档缺失点

- core 只提供二进制 .so 与头文件，核心调度/估计器/模型库行为不可见；三套 lib 必须与 ROS 发行版+架构匹配，`build.sh` 会创建 `core/lib` 软链（快照中不存在，属构建产物）。
- 三个 `mode.yaml` 快照默认 `active_mode: sim`；直接 `./run.sh` 不是真机模式，真机需 `robot`（`install.sh` 自动 sed，手工 rsync 时要自己改）。
- 策略目录命名不一致：pm01_edu/t800 用 `policies/`，t800pro 用 `policy/`；`rl_dance_example/policies/copy_policy_here.txt` 为空占位文件（策略实际存在，占位用途未说明）。
- t800pro 的 `idle/default.yaml` 注释写"使用 pd_stand_runner，但禁用严格运动检查"，实际运行的是 `idle_runner`（注释与实现不符）；pm01_edu/t800 无 idle scope 时 IdleRunner 走 param_ 为空的兜底分支（输出 Reset），属源码可见行为。
- 不同机型状态机差异较大（t800pro walk 有 `auto_transition: passive`，pm01/t800 dance 有 `auto_transition: walk`），改配置前先核对本机型文件。
- 官方文档 native-sdk/3 写 ros2_bridge 配置路径为 `assets/<robot>/ros2_bridge/default.yaml`，实际仓库为 `assets/config/<robot>/ros2_bridge/default.yaml`（文档路径少了 `config/`）。
- README_CN 1.7.2 的 Topic 表只列 6 个（"订阅/发布"按机器人视角描述），与 `ros2_bridge/default.yaml` 实际发布的 8 个 topic 不完全一致；官方文档 native-sdk/02 的 7.2 节 Topic 列表仅有截图。
- 仓库快照无 .git 元数据；未提供 core 源码、第三方库内容、`/opt/engineai_robotics_hardware` 硬件层与各机型实物验证说明。
- 策略兼容性：`num_observations`（72）与 `active_joint_names` 顺序是训练/部署契约，换策略必须同步；t800pro `command_scale_pos[0]=1.2`、t800 `command_scale_neg[0]=0.6` 为机型定制值，不要跨机型复制。
