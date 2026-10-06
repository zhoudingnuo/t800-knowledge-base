# engineai_legged_gym — EngineAI 早期基于 Isaac Gym 的 ZQSA01 双足 RL 训练与 sim2real 导出仓库

> 来源：https://github.com/engineai-robotics/engineai_legged_gym ｜ 分支：master ｜ commit：1a659aa0aa77（2024-09-11）｜ 快照日期：2026-10-03 ｜ 本地路径：`repos/engineai_legged_gym/` ｜ 许可证：未提供（根目录无 LICENSE；`setup.py` 声明 BSD-3-Clause；源码头沿用 NVIDIA/ETH Zurich 的 BSD-3-Clause，`rsl_rl/LICENSE` 为 BSD-3-Clause）

## 1. 定位与用途

- EngineAI 第一代（Isaac Gym 时代）双足机器人强化学习仓库，README 自称 "the full version of our repository which offers the basic operation of RL, including training, play(generate .pt model), sim2sim, and sim2real(generate .onnx model for real deployment)"，并声明"will be maintained and update continuously"（2024.09.10）。
- 从 ETH Zurich `legged_gym` + `rsl_rl` 派生并改造：保留 `LeggedRobot` 基类与 task_registry 结构，新增人形专用环境、对称性损失（symmetry loss）、观测/动作延迟域随机化等。
- **只注册了一个任务/机型**：`zqsa01`（ZQSA01 双足人形，12 个下肢关节），没有 PM01/T800 等新机型配置。
- 训练产物是 TorchScript（`policy_1.pt`）与 ONNX（`zqsa01_policy.onnx`），配套同代 C++ 部署仓库 `engineai_humanoid` 真机运行；sim2sim 用 MuJoCo。
- 仓库内自带真实样件 `sim2real_deploy/zqsa01_policy.onnx`（2,109,894 字节，ONNX producer 记录为 pytorch 1.13.1），可直接参考部署链路。

## 2. 目录结构

```
engineai_legged_gym/
├── legged_gym/
│   ├── envs/
│   │   ├── __init__.py                  # 任务注册：register("zqsa01", ZqSA01, ZqSA01Cfg, ZqSA01CfgPPO)
│   │   ├── base/                        # 上游基类：legged_robot.py / legged_robot_config.py / base_config.py / base_task.py
│   │   └── zqsa01/                      # 本仓库核心：zqsa01.py（环境）+ zqsa01_config.py（配置）
│   ├── scripts/
│   │   ├── train.py                     # 训练入口，训练前把配置拷贝到日志目录（record_config.py）
│   │   ├── play.py                      # 加载策略回放，默认导出 TorchScript，pygame 手柄控制
│   │   ├── sim2sim_zqsa01.py            # MuJoCo sim2sim（读取 zq_sa01.xml + jit 模型）
│   │   ├── record_config.py             # 把 zqsa01_config.py / legged_robot_config.py 快照进日志
│   │   └── test_euler.py                # 欧拉角工具小脚本
│   ├── utils/                           # helpers（get_args / export_policy_as_jit）、task_registry、logger、terrain、math
│   └── tests/test_env.py                # 简易环境测试脚本（与 play 类似）
├── resources/
│   ├── robots/zq_humanoid/
│   │   ├── urdf/zq_sa01.urdf            # 训练用 URDF（SolidWorks 导出，base_link + leg_l/r1..6_link）
│   │   ├── urdf/zq_sa01.xml             # MuJoCo MJCF（sim2sim 用，12 个 motor）
│   │   └── meshes/*.STL                 # 13 个网格（真实二进制文件，非 LFS 指针）
│   └── actuator_nets/anydrive_v3_lstm.pt # 上游 anymal 执行器网络遗留文件，代码中无引用
├── rsl_rl/                              # 内置 ETH rsl_rl 1.0.2（PPO），打补丁加入 sym_loss
├── sim2real_deploy/
│   ├── export_onnx_policy.py            # jit -> onnx 导出脚本（输入 1x705）
│   └── zqsa01_policy.onnx               # 已导出的样例策略（真实文件）
├── setup.py                             # 包名 engineai_training 1.0.0
├── README.md
└── .gitignore                           # 忽略 logs/、videos/、*.egg 等（快照中无 logs/）
```

## 3. 核心模块与关键文件

### 3.1 任务注册与配置（相对仓库根）

| 任务名 | 类 | 文件 |
|---|---|---|
| `zqsa01` | `ZqSA01` + `ZqSA01Cfg` + `ZqSA01CfgPPO` | `legged_gym/envs/__init__.py`、`legged_gym/envs/zqsa01/zqsa01.py`、`zqsa01_config.py` |

- `ZqSA01Cfg.env`：`frame_stack=15`，单帧观测 47 维 → `num_observations=705`；特权观测单帧 73 维、critic 帧栈 3；`num_actions=12`；`num_envs=4096`；episode 24s；`joint_num=12`。
- `ZqSA01Cfg.asset`：URDF `resources/robots/zq_humanoid/urdf/zq_sa01.urdf`；`foot_name="6_link"`、`knee_name="4_link"`；`terminate_after_contacts_on=['base_link','4_link']`；启用自碰撞（`self_collisions=0`）。
- `ZqSA01Cfg.control`：位置控制 `control_type='P'`；刚度/阻尼按关节后缀 `1_joint`~`6_joint` 配置（腿 roll/yaw 50/5、hip pitch 70/7、knee 70/7、踝 20/2，左右共用）；`action_scale=0.5`；`decimation=10`，`sim.dt=0.001` → 策略 100Hz。
- `ZqSA01Cfg.init_state`：初始高度 z=1.1；默认关节角 l3=-0.24、l4=0.48、l5=-0.24（左右相同）。
- `ZqSA01Cfg.terrain`：trimesh，8m×8m，20×20 地形块，比例 `[0.2,0.2,0.4,0.1,0.1,0,0]`（平地/障碍/均匀/上坡/下坡）。
- `ZqSA01Cfg.domain_rand`：摩擦 0.2–1.3、推力扰动（8s 间隔、±0.4m/s、±0.6rad/s）、基座质量 ±4kg、质心 ±0.06m、PD 增益 ±20%、力矩/连杆质量乘子、电机零位 ±0.035rad、关节摩擦/阻尼/电枢随机化，以及**动作指令延迟与观测（电机/IMU）延迟 1–10 个仿真步**。
- `ZqSA01Cfg.commands`：速度指令课程（`max_curriculum=1.7`），x∈[-0.5,1.5]、y∈[-0.5,0.5]、yaw∈[-1.5,1.5]，8s 重采样，非 heading 模式。
- `ZqSA01CfgPPO`：`ActorCritic` + `PPO`，actor [512,256,128]、critic [768,256,128]，`num_steps_per_env=60`、`max_iterations=300000`、`save_interval=100`，`learning_rate=1e-5`、`entropy_coef=0.001`、`gamma=0.994`、`lam=0.9`，实验名 `zqsa01_ppo`；**开启 `sym_loss=True`**，用 `obs_permutation`/`act_permutation`（左右腿镜像置换）作为额外损失（`sym_coef=1.0`）。

### 3.2 环境实现（关键文件：`legged_gym/envs/zqsa01/zqsa01.py`）

- 观测（单帧 47 维，见 `compute_observations`）：命令相位 `sin/cos`(2) + 速度指令(3) + 关节位置偏差(12) + 关节速度(12) + 上一步动作(12) + 角速度(3) + 欧拉角(3)；帧栈 15 维拼接为 705；噪音按 `noise_scales` 逐段缩放。
- 特权观测（单帧 73 维）：额外含参考关节角差、线速度、随机推力/力矩、摩擦、质量、步态相位/接触掩码。
- 域随机化实现含观测延迟缓冲（`obs_motor_latency_buffer`、`obs_imu_latency_buffer`）与动作延迟（`cmd_action_latency_buffer`）。
- 自定义奖励函数约 20 个（`_reward_joint_pos`、`feet_clearance`、`feet_contact_number`、`feet_air_time`、`foot_slip`、`feet_distance`、`knee_distance`、`orientation`、`base_height`、`base_acc`、`vel_mismatch_exp`、`track_vel_hard`、`tracking_lin_vel/ang_vel`、`low_speed`、`default_joint_pos`、`action_smoothness`、`torques/dof_vel/dof_acc`、`collision` 等），步态为固定周期 0.8s 的左右交替支撑相位（`sin>0` 左腿支撑、`sin<0` 右腿支撑）。
- 关键奖励参数：`base_height_target=0.827`、`cycle_time=0.8`、`tracking_sigma=5`；`only_positive_rewards=True`。

### 3.3 脚本与导出链路

- `legged_gym/scripts/train.py`：创建 env/runner 后调用 `record_config()` 把 `zqsa01_config.py` 与 `legged_robot_config.py` 原文拷入日志目录，再 `ppo_runner.learn()`。日志默认在 `logs/<experiment_name>/exported_data/<日期>_<run_name>/model_<iteration>.pt`。
- `legged_gym/scripts/play.py`：`__main__` 中 `EXPORT_POLICY=True`，加载 checkpoint 后调用 `export_policy_as_jit()` 导出到 `logs/zqsa01_ppo/0_exported/policies/policy_1.pt`；支持 pygame 手柄（左摇杆前后/左右、右摇杆 yaw）与 `FIX_COMMAND` 固定指令；末尾 `while True` 阻塞，需手工结束进程。
- `legged_gym/scripts/sim2sim_zqsa01.py`：加载 MJCF `resources/robots/zq_humanoid/urdf/zq_sa01.xml`，`dt=0.001`、`decimation=10`（100Hz 策略），PD 力矩控制（kp/kd 与训练一致，tau 限幅 200），按同样 47 维 × 15 帧构造观测；命令行 `--load_model` 指定策略。
- `sim2real_deploy/export_onnx_policy.py`：`torch.jit.load('logs/zqsa01_ppo/0_exported/policies/policy_1.pt')` → `torch.onnx.export`，`opset_version=11`，输入名 `input`（形状 1×705）、输出名 `output`，输出到 `sim2real_deploy/zqsa01_policy.onnx`。
- `legged_gym/utils/helpers.py`：`get_args()`（`--task/--resume/--experiment_name/--run_name/--load_run/--checkpoint/--num_envs/--seed/--max_iterations/--headless/--rl_device/--sim_device`）、`export_policy_as_jit()`（普通 MLP 用 `torch.jit.script`，LSTM 用自定义 exporter）。
- `rsl_rl/rsl_rl/algorithms/ppo.py`：vendored PPO 增加了 `sym_loss`/`obs_permutation`/`act_permutation`/`sym_coef` 参数，在 loss 中加 `sym_coef * (mu - m_mirror_act)^2`。

## 4. 使用方法

```bash
# 1) 安装（README 指定 python 3.8 + Isaac Gym Preview 4）
#   pip install torch==1.13.1+cu117 torchvision==0.14.1+cu117 torchaudio==0.13.1 \
#     --extra-index-url https://download.pytorch.org/whl/cu117
#   cd isaacgym/python && pip install -e .        # 需自行从 developer.nvidia.com 下载 Isaac Gym Preview 4
cd engineai_legged_gym
pip install -e rsl_rl                           # 内置 rsl_rl 1.0.2
pip install -e .                                # 包名 engineai_training
# 若报 libpython3.8m.so.1.0：sudo apt install libpython3.8 或设置 LD_LIBRARY_PATH

# 2) 训练
python legged_gym/scripts/train.py --task=zqsa01 --headless
#   可选 --num_envs / --max_iterations / --resume / --load_run / --checkpoint 等覆盖配置
#   结果：logs/zqsa01_ppo/.../model_<iteration>.pt（日志含配置原文）

# 3) 回放/导出 jit（play 默认 EXPORT_POLICY=True，导出到 logs/zqsa01_ppo/0_exported/policies/policy_1.pt）
python legged_gym/scripts/play.py --task=zqsa01

# 4) sim2sim（MuJoCo）
python legged_gym/scripts/sim2sim_zqsa01.py --load_model logs/zqsa01_ppo/0_exported/policies/policy_1.pt

# 5) sim2real 导出 onnx
python sim2real_deploy/export_onnx_policy.py    # 生成 sim2real_deploy/zqsa01_policy.onnx
# 之后按 engineai_humanoid README：把 onnx 拷贝到安装目录 install/policy/zqsa01/ 并运行 ./run_biped.sh
```

## 5. 依赖与环境

- **OS/Python**：Linux + NVIDIA GPU；Python 3.8（README 推荐）；`setup.py` 未声明 `python_requires`。
- **仿真**：NVIDIA Isaac Gym **Preview 4**（需从 NVIDIA 官网下载，不在仓库内）；`isaacgym` 为其 Python 包。
- **PyTorch**：README 命令安装 **1.13.1+cu117**（正文文字误写 "pytorch 1.10"，以命令为准）；样例 onnx 的 producer 记录为 pytorch 1.13.1。
- **RL**：内置 **rsl_rl 1.0.2**（ETH 原版 + sym_loss 补丁）；不依赖新版 rsl_rl ≥5 的 API。
- **Python 包**（`setup.py`）：`tensorboard`、`setuptools==59.5.0`、`matplotlib`、`numpy==1.23`、`opencv-python`、`tqdm`、`pygame`、`mujoco`、`mujoco-python-viewer`。
- **sim2sim**：MuJoCo + `mujoco_viewer`（`mujoco-python-viewer`），模型文件为仓库内 MJCF。
- CUDA/驱动最低版本、显存需求未说明；训练默认 `num_envs=4096`。

## 6. 与其他仓库的关系与代际关系

- **engineai_humanoid（同代部署配对）**：两个 README 相互引用。本仓库负责训练与导出 ONNX；`play.py` 产出的 jit 经 `sim2real_deploy/export_onnx_policy.py` 转成 `zqsa01_policy.onnx` 后，拷贝到 humanoid 的 `install/policy/zqsa01/` 由 C++ 控制器加载，配置文件对应 humanoid 的 `install/config/zqsa01_rl.yaml`。二者是 EngineAI **第一代（Isaac Gym + 自研 C++ 控制器）** 组合。
- **engineai_rl_lab（新一代训练）**：基于 Isaac Lab 2.3.2 + rsl_rl ≥5 的新仓库，只面向 PM01/T800，与本仓库无代码依赖、无迁移脚本；本仓库属于被其替代的旧代。旧代机型命名 `zqsa01` 在新仓库中不再出现。
- **engineai_ros2_workspace / engineai_robotics_native_sdk（新一代部署）**：新代训练产物以 MNN 部署、走 native_sdk/ROS2；本仓库的 ONNX + LCM/ROS1 C++ 部署链路与之并行、互不兼容。
- **上游**：ETH Zurich `legged_gym`、`rsl_rl`（均 BSD-3-Clause）；sim2sim 思路沿用 legged_gym 社区惯例。
- 与 `engineai_robotics_description`、`GMR`、`engineai_navigation` 等仓库未发现引用关系（当前快照中无互相引用）。

## 7. 注意事项、已知问题、文档缺失点

1. **维护状态**：最后快照 2024-09，仓库声明"持续维护"，但相较于 2026 年的 Isaac Lab 新仓库已明显是旧代；仅 zqsa01 一个机型，无新机型扩展。
2. **许可证口径不一**：根目录无 LICENSE；`setup.py` 写 `license="BSD-3-Clause"`；源码头为 NVIDIA/ETH 的 BSD-3-Clause。按知识库模板记"未提供"，引用时以源码头部为准。
3. **无 LFS 指针**：快照中 STL/URDF/MJCF、`anydrive_v3_lstm.pt`、`zqsa01_policy.onnx` 均为真实二进制文件，可直接使用；执行器网络 `.pt` 在代码中找不到引用（疑似上游遗留）。
4. **样例策略与 humanoid 内置策略不逐字节相同**：两边 `zqsa01_policy.onnx` 大小同为 2,109,894 字节，但 MD5 不同（本仓库 `b31a01ac…`，humanoid `e58abd0f…`），说明是不同时间/配置导出的两份文件。
5. **README 小错误**：正文写 "pytorch 1.10"，实际命令装 1.13.1；maintainer 拼写 "engienai"。
6. `play.py` 运行时默认导出 jit 且末尾死循环；`EXPORT_POLICY`/`RENDER`/`FIX_COMMAND` 是脚本内硬编码常量而非命令行参数。
7. `sim2sim_zqsa01.py` 的 `default_dof_pos`（-0.24/0.48）与训练配置一致，但 humanoid 部署配置 `zqsa01_rl.yaml` 的默认关节角（-0.06/0.12）和踝关节阻尼（0.2，训练为 2）不同；换策略时需自行核对一致性（未确认是否影响）。
8. 观测噪声/延迟随机化只存在于训练侧；部署侧 C++ 是否复现（噪声不加、延迟固定）需看 humanoid 仓库实现。
9. 无 CI、无单元测试（仅 `tests/test_env.py` 手动脚本）、无中文文档；仓库无 `logs/`（被 .gitignore 排除），没有随仓库提供的训练权重，只有 ONNX 样例。
10. `train.py` 顶部 `from record_config import record_config` 依赖脚本目录在 `sys.path`，从其他目录以模块方式运行会失败（未在文档说明）。
11. 地形接触力 GPU 不可靠等 Known Issues 沿用了上游 legged_gym README 的说明（建议改用 force sensor）。
