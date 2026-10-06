# engineai_humanoid — 基于 MIT Cheetah Software 的 ZQSA01 双足 C++ 实机运动控制与 RL 部署程序

> 来源：https://github.com/engineai-robotics/engineai_humanoid ｜ 分支：master ｜ commit：5c0cb92a9981（2024-09-14）｜ 快照日期：2026-10-03 ｜ 本地路径：`repos/engineai_humanoid/` ｜ 许可证：未提供（根目录无 LICENSE；仅 third-party 各组件自带许可证，如 JCQP/qpOASES/osqp/ParamHandler/inih/Goldfarb 等）

## 1. 定位与用途

- EngineAI 第一代双足实机运动控制/部署程序，README 明确 "based on the MIT locomotion control algorithm [Cheetah Software]"，即 MIT Cheetah-Software 的 C++ 控制框架移植（FSM + 状态估计 + 腿控制器 + 腿式动力学模型），扩展出人形版本。
- 与 `engineai_legged_gym` 配对：legged_gym 训练并导出 `zqsa01_policy.onnx`，本仓库用 **onnxruntime** 在真机（主控板 NeZha/NaZha）上推理，实现 RL 行走部署。
- 支持的控制状态：被动（PASSIVE）、关节 PD 直腿站立（STAND_UP）、屈膝平衡站立（BALANCE_STAND，基于 `squat_stand_jpos`）、锁定关节安全阻尼（LOCK_JOINT）、RL 行走（RL_LOCOMOTION）、急停（ESTOP/OFF）。
- 通信：LCM（组播 `udpm://239.255.76.67:7667?ttl=255`）发布状态/指令与订阅手柄、控制参数；电机走 SPI（`/dev/spidev1.0,1.1`，motor-mcu 用户态驱动）；IMU 为 Yesense（代码内数据结构沿用 VectorNav 命名）。
- 机型：**ZQSA01 / ZQSA01-P**（`RobotType::ZQ_Biped_SA01`、`ZQ_Biped_SA01P`），12 个下肢关节（含并联踝），无上肢。
- README 声明框架设计为可组合模型法（MPC/WBC）与学习法，"model based part will be opened in the future"（当前快照未包含 MPC/WBC 实现，仅留 QP 求解器等第三方依赖）。

## 2. 目录结构

```
engineai_humanoid/
├── dep-pkgs/
│   ├── motor-mcu_1.0.8_amd64.deb        # 电机 MCU 用户态库（motor_init/set_motor_cmd/motor_zero 等 SPI API）
│   └── README.md                        # SPI 设备、12 电机索引（0–5 左腿 / 6–11 右腿）、零位标定说明
├── EngineAI_Controller/
│   ├── EngineAI_Controller.cpp          # main()：创建 EngineAI_Humanoid_Controller 并 main_helper()
│   ├── CMakeLists.txt                   # 顶层构建；安装前缀 build/EngineAI_Humanoid/install
│   ├── config.h.cmake                   # 生成 Configuration.h（THIS_COM）
│   ├── config/
│   │   ├── zqsa01-robot-default-param.yaml  # 机器人参数：controller_dt=0.002、站立位/刚度阻尼等
│   │   └── simulator-defaults.yaml          # Webots/仿真参数（旧 Cheetah 遗留，本仓库文档未提供用法）
│   ├── common/                          # 通用库 common_module：动力学(FloatingBaseModel/RobotConstructor)、
│   │                                    #   状态估计、LegController、GaitScheduler、DesiredStateCommand、
│   │                                    #   闭链映射(close_chain_mapping)、数学/滤波/参数/标定等
│   ├── robot/                           # 运行框架 robot 库：HardwareBridge、SimulationBridge、RobotRunner、
│   │                                    #   RobotController 接口、main_helper、rt/（SBUS 遥控、串口）
│   ├── user/EngineAI_Humanoid_Controller/   # 用户层：FSM 状态机 + RL 控制器
│   │   ├── EngineAI_Humanoid_Controller.{h,cpp}   # 初始化 GaitScheduler/ControlFSM，runController()
│   │   ├── EngineAI_UserParameters.h              # 继承 ControlParameters 的用户参数声明
│   │   ├── FSM_States/                            # Estop/Passive/JointPD/BalanceStand/LockJoint/RL_Locomotion
│   │   │                                          #   + ControlFSM、SafetyChecker、TransitionData
│   │   └── Controllers/RL_Controller/
│   │       ├── include/rl_controllers/*.h         # LearningBasedController、Lowpassfilter、RotationTools、Types
│   │       ├── src/LearningBasedController.cpp    # ONNX 推理 + 观测构造 + 关节指令/踝解耦
│   │       ├── config/zqsa01_rl.yaml              # RL 部署配置（安装为 install/config/zqsa01_rl.yaml）
│   │       └── policy/zqsa01/zqsa01_policy.onnx   # 默认策略（安装为 install/policy/zqsa01/）
│   ├── lcm-types/                       # *.lcm 定义 + 生成的 cpp/python/java（gamepad、leg_control_*、
│   │                                    #   state_estimator、control_parameter、spi_command/data 等）
│   ├── scripts/                         # run_biped.sh、autostart.sh、config_network_lcm.sh、
│   │                                    #   network_config.py、setup_network_mc.py、lcm-log2smat、旧 mini-cheetah 脚本
│   └── third-party/                     # LogitechGamepad、yesense/vectornav IMU、SOEM、QP 求解器、
│                                        #   ParamHandler(yaml-cpp衍生)、inih 等
└── README.md                            # 依赖安装、编译、真机部署与手柄操作说明
```

## 3. 核心模块与关键文件

### 3.1 程序入口与运行框架

- `EngineAI_Controller/EngineAI_Controller.cpp`：`main()` 创建 `EngineAI_Humanoid_Controller`，调用 `main_helper()`。
- `robot/src/main_helper.cpp`：定义 `USE_REAL_ROBOT`，真机路径 new `ZqSA01HardwareBridge(ctrl, …)` 并 `run()`（未定义时走 SimulationBridge/Webots）。
- `robot/src/HardwareBridge.cpp`：
  - `initCommon()`：实时调度（`SCHED_FIFO`）、订阅 LCM 通道 `interface`（gamepad_lcmt）与 `interface_request`（控制参数读写），发布 `interface_response`。
  - `ZqSA01HardwareBridge::run()`：SPI 任务周期 **2ms**（500Hz，`runSpi` 读写电机并发布 `joint_data`/`joint_cmd`）、手柄任务 100ms（`run_sbus`）、RobotRunner 控制线程周期 `controller_dt=0.002`（500Hz）。
  - `initHardware()`：`motor_init()`（SPI 电机库）、`YesenseDriver` IMU 初始化、`init_sbus(false)`。
  - 电机数据/指令做方向与零位换算（`joint_zero_offset`/`joint_direction`），关节顺序 12 个。
- `robot/src/RobotRunner.cpp`：`buildEngineAIRobot<float>()` 建模型（`EngineAIBiped.h` 硬编码质量/惯量/连杆长度：hip 0.429m、knee 0.37m、maxLegLength 0.76m、motorTauMax 20 等）；状态估计（Contact + VectorNav 姿态 + 线性 KF 位置速度）；每周期发布 LCM `leg_control_command`/`leg_control_data`/`state_estimator`；前 10 个周期禁使能。
- `robot/src/rt/rt_rc_interface.cpp`：手柄 → `rc_control_settings.mode`（`RC_mode`）；同时定义了 AT9s/Taranis 航模遥控与 Logitech 手柄两套逻辑；`rt_sbus.h` 同时定义 `RC_AT9s` 和 `K_USE_LOGITECH_GAMEPAD`，真机实际走 Logitech 分支。

### 3.2 ControlFSM 状态与手柄按键逻辑（关键文件：`user/.../FSM_States/ControlFSM.cpp`、`robot/src/rt/rt_rc_interface.cpp`）

| 手柄操作 | 配置的 rc_mode | FSM 状态 / 行为 | 代码位置 |
|---|---|---|---|
| LB + START | PASSIVE (=1) | 被动（无力矩） | `sbus_packet_complete_logitech` |
| LB + BACK | OFF (=0) | 急停/无控制 | 同上 |
| LB + RB | LOCK_JOINT (=8) | 锁定关节（kp=0、kd 取 `safety_damper_kd`，仅加阻尼） | 同上 |
| LB + A | STAND_UP (=2) | 直腿站立（`FSM_State_JointPD`，`straight_stand_jpos=[-0.1,0.2,-0.2]`） | 同上 |
| LB + B | BALANCE_STAND (=7) | 屈膝平衡站立（`FSM_State_BalanceStand`，`squat_stand_jpos=[-0.24,0.48,-0.25]`，3s 过渡） | 同上 |
| LB + X | LOCOMOTION (=11) | RL 行走（仅允许从 STAND_UP 进入） | 同上 |
| RL 模式下 A | — | 在"原地站立 / 行走"间切换（`gait_type` 0↔1 边沿触发） | 同上 |
| RL 模式下 B | — | 开关欧拉角 bias 标定模式 | 同上 |
| RL 模式下 RB | — | 开关线速度 bias 标定模式 | 同上 |
| CROSS 十字键 | — | 标定模式下 ±0.001 调整 pitch/roll 或 vx/vy | 同上 |
| RL 模式下 Y / X | — | 保存标定值到 yaml / 清零当前标定值 | 同上 |

- `ControlFSM::runFSM()` 按 `rcCommand->mode` 设置 `control_mode`（K_ESTOP/K_PASSIVE/K_JOINT_PD/K_LOCK_JOINT/K_RL_LOCOMOTION/K_BALANCE_STAND），有安全预检（姿态）与后检；初始状态为 ESTOP。
- `FSM_State_RL_Locomotion` 进入时 `new LearningBasedController` 并 `init()`；`locomotionSafe()` 目前直接 `return true`（下方角度/腿位移检查为死代码，安全限幅未启用）。
- 注意按键优先级：代码按 START/BACK/RB/A/B 顺序 `if-else`，同一帧按下一个组合生效；LB+X 之前必须处于 STAND_UP。

### 3.3 RL 控制器（关键文件：`user/.../Controllers/RL_Controller/src/LearningBasedController.cpp`）

- 加载策略：`loadPolicyModel()` 固定相对路径 **`../policy/zqsa01/zqsa01_policy.onnx`**（相对进程工作目录，即 `install/bin`）；onnxruntime CPU、单线程。
- 加载配置：`loadRLConfig()` 固定路径 **`../config/zqsa01_rl.yaml`**；读取控制频率 500Hz、`decimation=5`（→策略 100Hz）、`action_scale=0.5`、`gait_period=0.8`、`still_ratio=0.1`、`enable_filter/cutoff_frequency=30`、`enable_ankle_tau_mapping`、摇杆比例（vX 1.0 / vY 0.7 / ωz 0.8）、默认关节角、各关节 Kp/Kd、obs 缩放、裁剪值、尺寸（actions 12、单帧 obs 47、帧栈 15）。
- 观测（47 维，`computeObservation`）：命令 5 维（相位 sin/cos + vx,vy,ωz，含 bias 与摇杆比例）+ 关节位置偏差 12 + 关节速度 12 + 上一步动作 12 + 躯干角速度 3 + 欧拉角 3（叠加 roll/pitch bias）；15 帧历史拼成 705 维输入；原地站立（`gait_type==0` 且相位落入 `still_ratio`）时命令相位冻结为 [0,1,0,0,0]。
- 动作→关节：`q_des = action*action_scale + default_joint_pos`，输出 Kp/Kd 来自 yaml；`enable_ankle_tau_mapping=true` 时踝关节（索引 4/5/10/11）先算 PD 力矩再经闭链解耦 `Decouple::getDecoupleQVT` 映射为电机力矩（对应关节命令 Kp/Kd 置 0），其余关节位置闭环；另有关节位置映射分支。
- 滤波：`LowPassFilter`（100Hz 采样、30Hz 截止）在 `gait_type==0 && still` 时平滑动作。
- bias 标定：`updateBias()` 支持 vx/vy/roll/pitch bias 的清零、实时生效与"保存回 `../config/zqsa01_rl.yaml`"（标定模式下 press Y）。
- `loadRLConfig()` 里 `observations_size=47`、`observations_frame_size=15`，与策略输入 705 一致；策略文件与 legged_gym 样例同名同大小（2,109,894 字节），但 MD5 不同。

### 3.4 策略与配置文件路径

- 仓库内：`EngineAI_Controller/user/EngineAI_Humanoid_Controller/Controllers/RL_Controller/policy/zqsa01/zqsa01_policy.onnx`；`.../config/zqsa01_rl.yaml`。
- 安装后：`build/EngineAI_Humanoid/install/bin/EngineAI_Controller`、`install/config/zqsa01_rl.yaml`、`install/policy/zqsa01/zqsa01_policy.onnx`（分别由 CMake `install(FILES …)` 规则安装；`run_biped.sh` 装到 `build/EngineAI_Humanoid/` 一层）。
- 机器人参数：仓库 `config/zqsa01-robot-default-param.yaml` → `install/config/`，含 `controller_dt: 0.002`、`control_mode: 0`、`use_rc: 1`、`straight_stand_jpos`、`squat_stand_jpos`、`stand_joint_kp/kd`、`safety_damper_kd`、`sin_wave_period` 等。
- LCM 类型定义在 `lcm-types/*.lcm`（`gamepad_lcmt`、`leg_control_command_lcmt`、`leg_control_data_lcmt`、`state_estimator_lcmt`、`control_parameter_*`、旧四足 `spi_command_t/spi_data_t`）。

## 4. 使用方法

```bash
# 1) 依赖（README 建议版本）：Ubuntu 20.04、CMake（文档要求 >3.26，顶层 CMakeLists 仍写 3.5）、Eigen 3.3.7、
#    yaml-cpp 0.6.3、LCM 1.4/1.5、ROS Noetic、protobuf、onnxruntime 1.19.0；libmotor 由 deb 安装：
sudo dpkg -i dep-pkgs/motor-mcu_1.0.8_amd64.deb

# 2) 编译安装
cd engineai_humanoid/EngineAI_Controller
mkdir build && cd build
cmake .. && make -j
make install          # 产物：build/EngineAI_Humanoid/install/{bin,config,policy}

# 3) 真机运行（在主控板 NeZha 上）
#    本机静态 IP 192.168.0.100 / 255.255.255.0 / 网关 192.168.0.1；
sudo ifconfig <iface> multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev <iface>
ssh user@192.168.0.163        # 密码 1
sudo pkill EngineAI_Controller          # 停止默认自启进程
#    拷贝 EngineAI_Humanoid 到 /home/user/，替换 install/policy/zqsa01/ 下的 onnx、
#    按需修改 install/config/zqsa01_rl.yaml 的 Kp/Kd 等参数
cd ~/EngineAI_Humanoid/install/bin
sudo ./EngineAI_Controller    # 或运行同目录上一层的 ./run_biped.sh
#    本地可用 lcm-spy 观察 joint_data/joint_cmd/state_estimator

# 4) 手柄操作顺序（README）：
#    上电 → LB+back（下电/关闭）→ LB+start（使能）→ LB+B 或 LB+A（屈膝/直腿站立）→
#    站稳后 LB+X 进入 RL → A 切换站立/行走；紧急情况只用 LB+RB、LB+start、LB+back
```

## 5. 依赖与环境

- **OS**：Ubuntu 20.04；**C++14**；编译选项 `-O3 -march=core2`。
- **构建**：CMake ≥3.26（README，原因写在 onnx 更新算法）、Eigen 3.3.7、yaml-cpp 0.6.3、LCM 1.4/1.5（需 `LCM_ENABLE_JAVA` 生成 Java 类型）、ROS Noetic（README 说明用于 onnx/imu 库，代码本身未见 ROS 节点）、protobuf、`motor-mcu_1.0.8`。
- **推理**：onnxruntime **1.19.0**（`pkg_check_modules(ONNXRUNTIME libonnxruntime)`，找不到则 `find_package(onnxruntime)`）。
- **硬件/驱动**：SPI 电机 MCU（`/dev/spidev1.0`、`/dev/spidev1.1`，12 关节，左右各 6：thigh roll/yaw/pitch、knee、内踝、外踝）、Yesense IMU（串口驱动，代码中同时带 vectornav 第三方库但硬件初始化走 Yesense）、Logitech 手柄（USB/SBUS 输入）。
- **运行时**：LCM 组播 `udpm://239.255.76.67:7667?ttl=255`；控制环 500Hz（`TIME_STEP=2ms`），RL 策略 100Hz（decimation 5）。
- 主控板出厂已配好环境，README 强烈建议不要改动板载开发环境。

## 6. 与其他仓库的关系与代际关系

- **engineai_legged_gym（同代训练配对）**：策略由 legged_gym 的 `sim2real_deploy/export_onnx_policy.py` 导出（jit→onnx, opset 11, 输入 1×705），本仓库 `install/policy/zqsa01/zqsa01_policy.onnx` 使用；配置项与 legged_gym 的 `zqsa01_config.py` 成对（obs 缩放、action_scale、Kp/Kd、gait period 0.8、decimation）。二者共同构成 EngineAI **第一代 Isaac Gym + C++ 控制器** 技术栈。
- **engineai_rl_lab（新一代训练）**：Isaac Lab + PM01/T800，本仓库不支持这些机型，也无迁移工具；`zqsa01` 在知识库中只属于这一代。
- **engineai_ros2_workspace（新一代 ROS2 部署）**：面向新机型的 ROS2 控制与部署，与本仓库的 LCM/ROS1/SPI 链路无兼容关系，属替代关系。
- **engineai_robotics_native_sdk（新一代原生部署）**：用 MNN 推理 + MuJoCo sim2sim 部署 PM01/T800；本仓库的 onnxruntime 方案是其前代。
- **上游**：MIT Cheetah-Software（FSM/leg controller/状态估计框架与第三方 QP 求解器生态），README 明确致谢；`scripts/run_mc*.sh`、`setup_network_mc.py`、`send_to_mini_cheetah.sh` 等仍保留 mini-cheetah 命名，说明是从四足框架平移而来。
- 与 `engineai_robotics_description`（URDF/描述仓库）未见直接引用；本仓库的机器人模型为 C++ 硬编码而非 URDF。

## 7. 注意事项、已知问题、文档缺失点

1. **维护状态**：快照 2024-09-14，与 legged_gym 同为旧代；之后 EngineAI 转向 Isaac Lab（PM01/T800）+ ROS2/native_sdk，本仓库无新机型、无更新迹象。README 承诺的模型法（MPC/WBC）模块未在本快照开放。
2. **许可证**：根目录无 LICENSE，仅第三方组件各自带许可（MIT Cheetah 派生代码的授权在 README 中提到上游但未附全文）；知识库按"未提供"处理。
3. **无 git-lfs 指针**：`motor-mcu_1.0.8_amd64.deb`、`zqsa01_policy.onnx`、LCM jar、Java class 等均为真实文件。
4. **策略一致性存疑**：本仓库内置 onnx 与 legged_gym 样例同名同大小但 MD5 不同；且 `zqsa01_rl.yaml` 的默认关节角（l3=-0.06/l4=0.12/l5=-0.06）与踝阻尼（0.2）和 legged_gym 训练配置（-0.24/0.48/-0.24、阻尼 2）不一致，替换策略时需逐项核对（未确认训练时实际使用哪套默认位）。
5. **安全逻辑被短路**：`FSM_State_RL_Locomotion::locomotionSafe()` 直接 `return true`，姿态/腿位移保护不生效；实际安全依赖 `SafetyChecker` 的姿态预检（且 LOCK_JOINT 模式跳过）。
6. **机型不一致**：`EngineAIBiped.h` 的 `buildEngineAIRobot()` 硬编码 `_robotType = ZQ_Biped_SA01P`，而 `HardwareBridge.cpp` 把 `_robotRunner->robotType` 设为 `ZQ_Biped_SA01`；类型相关的限幅分支（`FSM_State.cpp`、`SafetyChecker.cpp`）取值不同，需注意。
7. **遗留/无关文件**：顶层 `scripts/run_mc.sh`、`run_mc_2.sh`、`autostart.sh`（指向 `/home/user/robot-software/build`）、`setup_network_mc.py`、`send_to_mini_cheetah.sh`、`spi_command_t/spi_data_t` LCM 类型均为旧四足框架遗留；`config/simulator-defaults.yaml` 与 `SimulationBridge` Webots 路径同样属旧仿真链路（本仓库文档未提供 sim2sim 用法，README 指向 legged_gym）。
8. **硬编码路径**：策略与配置用相对路径写死（`../policy/...`、`../config/...`），必须从 `install/bin` 目录运行；bias 保存会就地重写 `../config/zqsa01_rl.yaml`，README 提醒先备份默认配置。
9. **按键与代码细节**：`gait_type` 的 A 键切换依赖 `data.A > last_gait_type` 边沿比较（值为 0/1）；标定模式互斥（欧拉角与线速度标定不能同时开）；README 的按键列表与代码一致，但未说明标定步长（代码 ±0.001）。
10. **文档缺失**：无单元测试/CI、无接口文档；制动距离/通信带宽等未说明；`dep-pkgs/README.md` 的零位标定索引前后不一致（先写 21~23，后写 21~26）；主控板名称"NeZha/NaZha"拼写不统一。
11. **ROS1 依赖口径**：README 要求安装 ROS Noetic，但源码中未发现 roscpp 节点/话题，可能只用于 IMU/onnx 的预编译依赖（未确认）。
