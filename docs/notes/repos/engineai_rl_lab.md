# engineai_rl_lab — 众擎基于 Isaac Lab 的 PM01/T800 强化学习训练框架（whole-body tracking + AMP）

> 来源：https://github.com/engineai-robotics/engineai_rl_lab ｜ 分支：main ｜ commit：c53f7ec614e3（2026-09-29）｜ 快照日期：2026-10-03 ｜ 本地路径：`repos/engineai_rl_lab/` ｜ 许可证：BSD 3-Clause（`LICENSE.txt`，Copyright (c) 2026, engineai-robotics）

## 1. 定位与用途

- EngineAI 官方新一代 Isaac Lab 强化学习训练仓库，提供 **PM01、T800** 两类人形机器人的训练环境。
- 目前覆盖两组任务：
  - **whole body tracking（全身动作跟踪）**：用 CSV/NPZ 参考动作驱动全身（含头部/手臂），奖励跟踪锚点与各刚体位姿、速度；对应 BeyondMimic 路线，用于跳舞等编排动作。
  - **AMP（Adversarial Motion Priors）**：用少量行走参考动作约束步态风格 + 速度指令，训练稳定行走策略。
- 配套：训练产物导出 ONNX→MNN，交给 `engineai_robotics_native_sdk` 做 MuJoCo Sim2Sim 与真机 Sim2Real。
- 仓库**不包含 T800 Pro**（README、源码均只出现 PM01/T800；T800 Pro 配置只存在于 native_sdk / ros2_workspace）。
- 架构：Isaac Lab 2.3.2 + Isaac Sim 5.1.0 + rsl_rl（≥5.0，PPO/自定义 AMP 算法）；AMP 实现参考 legged_lab + MimicKit，tracking 参考 whole_body_tracking/BeyondMimic。

## 2. 目录结构

```
engineai_rl_lab/
├── datasets/
│   ├── amp/{pm01,t800}/{config/dataset.yaml, data/*.npz}   # AMP 参考动作：GMR 紧凑格式（fps/root_pos/root_rot/dof_pos）
│   └── tracking/{pm01,t800}/dance*.csv、*.npz              # 全身跟踪动作，csv 经脚本转 npz
├── docs/                                                   # 演示 gif、手柄截图
├── logs/rsl_rl/                                            # 随仓库提供的示例训练输出（含已导出的 onnx/mnn）
│   ├── tracking_pm01/…、tracking_t800/…                    #   含 dance.pt、exported/policy.{onnx,mnn}、deploy_config.yaml
│   └── amp_velocity_flat_pm01/…、amp_velocity_flat_t800/…  #   含 model_19999.pt、export/model_19999.{onnx,mnn}
├── scripts/
│   ├── csv_to_npz.py、replay_npz.py                        # tracking 动作转换 / Isaac Lab 回放
│   ├── tracking/{train.py, play.py, cli_args.py}           # tracking 训练/回放+导出
│   └── amp/{train.py, play.py, replay_npz.py, cli_args.py} # AMP 训练/回放+导出/动作预览
└── source/engineai_rl_lab/
    ├── config/extension.toml                               # Isaac Lab 扩展元数据（模板未改名）
    └── engineai_rl_lab/
        ├── assets/{pm01,t800}/*.usd                        # PM01(pm01_edu)、T800 USD（git-lfs）
        ├── tasks/tracking/                                 # 全身跟踪：tracking_env_cfg.py + mdp/ + robots/ + config/{pm01,t800}/
        ├── tasks/locomotion/amp/                           # AMP：amp_env_cfg.py + algorithms/ + agents/ + utils/ + robots/ + config/{pm01,t800}/
        └── utils/{exporter.py, my_on_policy_runner.py}     # ONNX/MNN 导出与元数据、wandb 自动导出 Runner
```

## 3. 核心模块与关键文件

### 3.1 任务注册与任务名（gym.register）

| 任务名 | 配置文件 | 说明 |
|---|---|---|
| `Tracking-Flat-PM01-v0` | `tasks/tracking/config/pm01/__init__.py` → `flat_env_cfg.PM01FlatEnvCfg` | PM01 全身跟踪基础版（用本体感觉状态估计） |
| `Tracking-Flat-PM01-Wo-State-Estimation-v0` | `PM01FlatWoStateEstimationEnvCfg` | 去掉 `motion_anchor_pos_b`、`base_lin_vel`，对应真机可部署观测（仓库示例日志即此任务） |
| `Tracking-Flat-PM01-Low-Freq-v0` | `PM01FlatLowFreqEnvCfg` + `PM01FlatLowFreqPPORunnerCfg` | 低频控制变体（decimation 4→8，即 50Hz→25Hz；**该 Runner 类不存在，注册已损坏，见第 7 节**） |
| `Tracking-Flat-T800-v0` / `-Wo-State-Estimation-v0` / `-Low-Freq-v0` | `tasks/tracking/config/t800/__init__.py` | 同上，T800 三档；T800 的 LowFreq Runner 已实现 |
| `AMP-Flat-PM01-v1` | `tasks/locomotion/amp/config/pm01/__init__.py` → `flat_amp_env_cfg.PM01AMPFlatEnvCfg` | PM01 AMP 行走（实验名 `amp_velocity_flat_pm01`） |
| `AMP-Flat-T800-v1` | `tasks/locomotion/amp/config/t800/__init__.py` → `flat_amp_env_cfg.T800AMPFlatEnvCfg` | T800 AMP 行走（实验名 `amp_velocity_flat_t800`，带对称性镜像损失） |

### 3.2 Tracking 关键实现

- `tasks/tracking/tracking_env_cfg.py`：场景基类（平面地形、接触传感器）；`CommandsCfg.motion = MotionCommandCfg`（锚点 `LINK_BASE`，body 列表按机器人覆盖）；观测组 `policy`（含噪声）与 `critic`（特权，含 body_pos/ori）；奖励含 `motion_global_anchor_pos/ori`、`motion_body_pos/ori`、`motion_body_lin_vel/ang_vel`；终止条件 `anchor_pos`、`anchor_ori`、`ee_body_pos`。`decimation=4`、`sim.dt=0.005`（控制 50Hz）、episode 10s。
- `tasks/tracking/robots/pm01.py`：`PM01_CYLINDER_CFG`（`assets/pm01/serial_pm01_edu.usd`，24 关节 J00–J23，含头 `J23_HEAD_YAW`），`joint_sdk_names` 与真机关节顺序一致；`PM01_ACTION_SCALE` 由 `0.25*effort_limit/stiffness` 自动生成（髋/膝/踝 pitch ≈0.49，其余 ≈0.23）。
- `tasks/tracking/robots/t800.py`：`T800_CYLINDER_CFG`（`assets/t800/serial_t800.usd`，25 关节 J00–J24，含 `J23_HEAD_PITCH/J24_HEAD_YAW`）；分腿/踝/腰/臂/头 6 组 `DelayedImplicitActuatorCfg`（随机延迟 1–3 物理步，模拟真机通信延迟）；`T800_ACTION_SCALE` 逐关节显式给定（0.05–0.5）。
- `tasks/tracking/config/<robot>/flat_env_cfg.py`：覆盖锚点、跟踪刚体列表（PM01 13 个、T800 15 个）、不允许接触的刚体正则、末端终止刚体（PM01 踝+肘；T800 踝+腕）。
- PPO 配置 `agents/rsl_rl_ppo_cfg.py`：`num_steps_per_env=24`，`max_iterations` 20000（PM01）/30000（T800），`save_interval=2000`，MLP [512,256,128]+ELU，自适应学习率 1e-3、entropy 0.005；tracking 用 `empirical_normalization=True`，actor 不做 obs 归一化（归一化由 Runner 完成）。

### 3.3 AMP 关键实现

- `tasks/locomotion/amp/amp_env_cfg.py`：±2cm 随机起伏平地；action 为关节位置（DFS 顺序，`preserve_order=True`）；`OBS_HISTORY_LENGTH=15`、`DISC_HISTORY_LENGTH=10`；速度指令 `base_velocity`（默认 x 0.6–1.2 m/s，机器人配置改为 0.5–0.8、wz ±0.6）；`decimation=5`、`sim.dt=0.002`（控制 100Hz）、episode 20s；含域随机化（摩擦/质量/PD 增益/质心/推力）。
- 判别器观测对齐（关键坑）：`PM01_DISC_TERM_ORDER = (base_ang_vel, joint_pos, joint_vel, base_lin_vel, projected_gravity)`，用 `pin_observation_term_order` 固定拼接顺序；PM01 `frame_dim=55`（3+23+23+3+3），T800 `frame_dim=52`（`include_base_lin_vel=False`）。
- `tasks/locomotion/amp/algorithms/amp.py`：`class AMP(PPO)`，`AMPLossType` 支持 GAN/LSGAN/WGAN；风格奖励在 `process_env_step` 中以 `style_reward_scale` 与任务奖励 `task_style_lerp` 混合注入；判别器更新与 PPO mini-batch 解耦（`disc_update_interval`，PM01=4、disc_epochs=1；T800 用默认 2/2）；支持 `condition_dim=3` 的条件 AMP、对称性镜像损失（T800）、判别器经验复用缓冲区。
- `tasks/locomotion/amp/agents/amp_runner_cfg.py`：`RslRlAmpCfg` 集中所有 AMP 超参（dataset_path、frame_length/dim、损失类型、grad penalty、判别器学习率、replay buffer 容量等）；`algorithm.class_name` 指向 `…algorithms.amp:AMP`。
- `tasks/locomotion/amp/utils/AMP_data_loader.py`：读取 `dataset.yaml`（4 个 walk npz，含权重）；紧凑动作格式要求键 `fps/root_pos/root_rot/dof_pos`，按 YAML 的 `dof_indices` 重排、`zero_dof_indices` 置零（腰/躯干 yaw）；提供按动作权重采样的 mini-batch。
- `tasks/locomotion/amp/utils/AMP_discriminator.py`、`disc_replay_buffer.py`：判别器网络与经验回放。
- `tasks/locomotion/amp/robots/{pm01,t800}.py`：AMP 专用配置，关节数为 **23**（去掉头部），与动作数据一致；T800 使用延迟执行器。
- `datasets/amp/<robot>/config/dataset.yaml`：4 条 `walk_*_Skeleton9.npz` 等权，`dof_indices` 重排到机器人关节顺序，`zero_dof_indices:[2]`。

### 3.4 导出与 Runner

- `utils/exporter.py`：
  - `export_motion_policy_as_onnx(...)`：优先 actor 的 `as_onnx()`（opset 18），否则回退 `_OnnxMotionPolicyExporter`（opset 11，输入 `obs`、输出 `actions`）。
  - `attach_onnx_metadata(...)`：生成 `deploy_config.yaml`（`default_joint_pos`、`joint_names`、`joint_stiffness/damping`、`action_scale`、`observation_names`、`observation_history_lengths`），并把同一份元数据写入 ONNX `metadata_props`。
- `utils/my_on_policy_runner.py`：`MotionOnPolicyRunner(OnPolicyRunner)`，当 logger=wandb 时在 `save()` 里同步导出 ONNX+元数据并上传 wandb artifact。
- `scripts/tracking/play.py`：加载 checkpoint 后自动导出 `exported/policy.onnx`，若安装了 `mnn` 包则调用 `python -m MNN.tools.mnnconvert` 生成 `exported/policy.mnn`，再写 `exported/deploy_config.yaml`。
- `scripts/amp/play.py`：导出 `export/<checkpoint名>.onnx/.mnn`（如 `model_19999.onnx`），**不生成 deploy_config.yaml**。

## 4. 使用方法

```bash
# 1) 安装（需先按 Isaac Lab 官方文档装好 Isaac Lab 2.3.2 / Isaac Sim 5.1 环境）
git clone https://github.com/engineai-robotics/engineai_rl_lab.git
cd engineai_rl_lab
git lfs install && git lfs pull          # USD/pt 等大文件走 LFS，必须拉取
pip install -e source/engineai_rl_lab
pip install mnn                          # ONNX→MNN 转换所需（import 名为 MNN）

# 2) whole body tracking：CSV→NPZ、回放检查、训练、验证导出
python scripts/csv_to_npz.py --robot pm01 --input_fps 30 -f datasets/tracking/pm01/dance.csv
python scripts/replay_npz.py --robot pm01 --input_file datasets/tracking/pm01/dance.npz
python scripts/tracking/train.py --task Tracking-Flat-PM01-Wo-State-Estimation-v0 --headless \
  --num_envs 4096 --motion_file datasets/tracking/pm01/dance.npz
python scripts/tracking/play.py --task Tracking-Flat-PM01-Wo-State-Estimation-v0 --num_envs 1 \
  --motion_file datasets/tracking/pm01/dance.npz --load_run 2026-06-23_09-58-43 --checkpoint dance.pt
python -m tensorboard.main --logdir logs

# T800：把 --robot/task/motion_file 换成 t800 对应值（motion_file datasets/tracking/t800/dance_t800.npz）

# 3) AMP：先预览参考动作，再训练/回放导出（动作从 dataset.yaml 读取，无需 --motion_file）
python scripts/amp/replay_npz.py --robot pm01 --input_folder datasets/amp/pm01/data
python scripts/amp/train.py --task AMP-Flat-PM01-v1 --headless --num_envs 4096
python scripts/amp/play.py --task AMP-Flat-PM01-v1 --num_envs 1 --load_run RUN_DIRECTORY --checkpoint model_19999.pt
python -m tensorboard.main --logdir logs/rsl_rl

# 4) Sim2Sim/Sim2Real（在 engineai_robotics_native_sdk 中操作；详见 README）
# 把 <run>/exported/policy.mnn 复制到 native_sdk 的 assets/config/<robot>/rl_dance_example(rl_lab)/policies，
# npz 动作复制到 trajectories/，修改 default.yaml 的 policy_file/trajectory_file_npz；
# 容器内 ./scripts/run_mujoco.sh pm01_edu + ./run.sh pm01_edu，手柄 LB+X 进 AMP 行走、RB+X 进 tracking。
# 真机：./install.sh pm01_edu robot → ssh 机器人 → sudo systemctl stop robotics.service → sudo ./run_robot.sh pm01_edu
```

## 5. 依赖与环境

- **Python**：`python_requires>=3.10`（pyright 配置为 3.11；Isaac Lab 2.3.2 官方建议 3.11）。
- **Isaac Lab**：2.3.2，配套 commit `c22775241e28f465fe345fa1a482ad6d29d712b0`（README 明确不同版本代码可能不通用）；**Isaac Sim 5.1.0**（README 徽章）。
- **rsl_rl**：随 Isaac Lab 安装，需 ≥5.0 的新 API（`MLPModel`、`TensorDict`、`construct_algorithm`）；训练脚本内有 `sanitize_rsl_rl_cfg` 兼容旧版模型键。
- **Python 包**：`psutil`（setup.py）；导出需要 `onnx`、`pyyaml`；MNN 转换需 `mnn`（import `MNN`）；wandb 可选（logger/自动导出）。
- **CUDA/驱动**：仓库未规定，取决于 Isaac Sim 5.1 官方要求（未确认具体版本）。
- 训练默认 `--num_envs 4096`、headless；GPU 显存要求仓库未给出。

## 6. 与其他仓库的关系

- **engineai_robotics_native_sdk（部署，官方唯一文档化链路）**：Sim2Sim 用 MuJoCo（`run_mujoco.sh`）+ native_sdk 控制程序；Sim2Real 用 `install.sh pm01_edu robot` 推送并 `run_robot.sh` 运行，策略以 MNN 形式加载；追踪导出的 `deploy_config.yaml` 提供关节顺序/刚度/阻尼/action_scale/观测布局（native_sdk 侧读取逻辑未在本仓库，未确认）。
- **engineai_amp**：同组织的早期 Isaac Lab 训练仓库（仅 PM01/velocity+AMP），与本仓库无代码依赖；rl_lab 是其扩展/后继。注意 native_sdk README 中 `rl_lab` 状态说明的链接指向 engineai_amp，文档口径不一致（待确认）。
- **engineai_ros2_workspace**：ROS2 Humble 控制接口仓库，rl_lab 未提及与它的直接对接，策略部署走 native_sdk。
- **engineai_legged_gym**：Isaac Gym 旧代训练仓库，本仓库无引用。
- **上游致谢**：Isaac Lab、rsl_rl、BeyondMimic/whole_body_tracking、MNN、legged_lab、MimicKit、AMP_for_hardware。

## 7. 注意事项、已知问题、文档缺失点

1. **LFS 大文件已在本知识库快照中拉取**：上游用 git-lfs 管理 `assets/**/*.usd`、多数 `logs/**/*.pt` 与演示 gif（GitHub tarball 里只有 129B 指针）。本工作区已把 23 个 LFS 对象（约 252MB）替换为真实文件并逐个 sha256 校验（`scripts/fetch_lfs.py`、`scripts/lfs_chunked.py`），可直接训练/回放。若你从 GitHub 重新 clone，请执行 `git lfs install && git lfs pull`。示例导出件 `exported/policy.onnx|mnn`、`export/model_19999.onnx|mnn`（约 2.9MB）与 `tracking_pm01/.../dance.pt`（约 6.1MB）本就是真实文件。
2. **PM01 Low-Freq 任务注册损坏**：`tasks/tracking/config/pm01/__init__.py` 引用 `rsl_rl_ppo_cfg:PM01FlatLowFreqPPORunnerCfg`，但该文件中并未定义此类（T800 有定义）；使用 `Tracking-Flat-PM01-Low-Freq-v0` 会在解析 agent 配置时报错。
3. **AMP 导出路径与 README 不一致**：README 写 `exported/policy.mnn`，实际 AMP 的 `play.py` 导出到 `export/<checkpoint名>.mnn` 且无 `deploy_config.yaml`，复制部署时要按实际路径。
4. `datasets/amp/pm01/config/dataset-.yaml` 是遗留备份，引用不存在的 `Skeleton7` 文件，勿使用。
5. `scripts/tracking/cli_args.py` 的 `--resume` 用 `type=bool`（任何非空字符串都为 True）；AMP 版已修正为 `_str_to_bool`。
6. tracking 训练强依赖 `--motion_file`（或 `--registry_name` 从 wandb 拉取），否则抛 `ValueError`；play 也建议显式传 `--motion_file`。
7. `config/extension.toml` 未改模板（title "Extension Template"、repository 指向 isaac-sim/IsaacLab）；`setup.py` 的 `license="Apache-2.0"` 与 `LICENSE.txt` 的 BSD-3-Clause 不一致（以 LICENSE.txt 为准）。
8. AMP 训练脚本内 `sys.path` 手工插入本地 `source/engineai_rl_lab`，注释提示 editable install 可能解析到其他 checkout，多份克隆时注意。
9. 机器人覆盖：只有 PM01 与 T800（含 pm01_edu USD）；AMP 动作仅 23 自由度（无头），tracking 动作 24/25 自由度（含头），两者不可混用。
10. AMP 参考动作（`*_Skeleton9.npz`）的来源/生成工具链仓库未说明；tracking 的 CSV 要求首末帧与 pd stand 关节位置基本一致，否则模式切换不流畅。
11. 未提供单元测试、CI、AMP 判别器调参说明；`docs/` 只有 gif，没有详细文档。
