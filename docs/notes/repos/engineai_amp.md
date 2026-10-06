# engineai_amp — 众擎早期 PM01 专用 Isaac Lab 训练包（速度行走 + AMP 行走，README 自称 "EngineAI-Lab"）

> 来源：https://github.com/engineai-robotics/engineai_amp ｜ 分支：main ｜ commit：83ba64bbb58a（2026-05-06）｜ 快照日期：2026-10-03 ｜ 本地路径：`repos/engineai_amp/` ｜ 许可证：BSD 3-Clause（`LICENSE`，Copyright (c) 2025- Shenzhen Zhongqing Robot Technology Co., Ltd. ("EngineAI")）

## 1. 定位与用途

- 本仓库实际是 **EngineAI-Lab**（Python 包名 `engineai_lab`），一个基于 Isaac Lab/Isaac Sim 的早期训练包，README 原文：“a python package for training and deploying policies for EngineAI Robots”。
- 只支持 **PM01** 一款机器人、只做**平地速度行走**方向的两个任务：
  - `Flat-PM01-v0`：平地/崎岖地形速度跟踪（PPO）。
  - `Flat-AMP-PM01-v0`：在同一环境上叠加 AMP（对抗运动先验）风格奖励的行走。
- 与 `engineai_rl_lab`（PM01+T800、tracking+AMP、2026-09）相比，这是更早、能力更弱的一代：单机器人、无全身 tracking、AMP 实现较简单（无 LSGAN/WGAN、无条件 AMP、无多动作重排等）。
- 可**独立运行**：自带 `source/engineai_lab` 包、PM01 URDF/mesh、dataset 与脚本；只依赖 Isaac Lab + Isaac Sim + rsl_rl（≥5.0），不 import `engineai_rl_lab`，也不需要 `engineai_legged_gym`。
- 部署路线：训练 → `play.py` 导出 ONNX → 手动用 MNN-Converter 转 MNN → 真机/native_sdk。

## 2. 目录结构

```
engineai_amp/
├── config/extension.toml                    # Isaac Lab 扩展模板（未改名，repository 仍指向 isaac-sim/IsaacLab）
├── dataset/
│   ├── config/dataset.yaml                  # AMP 动作清单：motions -> ../data/locomotion.npz
│   └── data/locomotion.npz                  # AMP 参考动作（全刚体格式，约 3.3MB；dataset/ 被 .gitignore 忽略）
├── docs/                                    # training/sim2sim/deploy 演示 gif
├── scripts/
│   ├── train.py                             # 训练入口（Hydra + rsl_rl）
│   ├── play.py                              # 回放 checkpoint，并导出 ONNX 到 exported/
│   ├── replay_npz.py                        # 在 Isaac Lab 中回放全刚体 npz 动作（仅 PM01）
│   └── cli_args.py                          # rsl_rl 通用 CLI（experiment/run/load/logger/wandb）
├── setup.py                                 # pip install -e .；install_requires: psutil/onnxscript/wandb>=0.19
└── source/engineai_lab/
    ├── __init__.py                          # 导入 tasks，触发 gym 注册
    ├── robots/{pm01.py, actuator.py}        # PM01_CFG(URDF) + 延迟 PD 执行器（源自 whole_body_tracking）
    ├── algorithms/amp_ppo.py                # AMPPPO（继承 rsl_rl PPO，加判别器/风格奖励）
    ├── utils/{AMP_data_loader.py, AMP_discriminator.py}
    ├── assets/pm01/{urdf/serial_pm01.urdf, meshes/*.dae}
    └── tasks/velocity/
        ├── config/pm01/{__init__.py, flat_env_cfg.py, flat_amp_env_cfg.py, agents/}
        └── mdp/{rewards.py, observations.py, events.py}
```

## 3. 核心模块与关键文件

### 3.1 任务注册与配置

- `source/engineai_lab/tasks/velocity/config/pm01/__init__.py` 注册两个环境：
  - `Flat-PM01-v0` → `flat_env_cfg.PM01FlatEnvCfg` + `agents.rsl_rl_ppo_cfg:PM01FlatPPORunnerCfg`。
  - `Flat-AMP-PM01-v0` → `flat_amp_env_cfg.PM01AMPFlatEnvCfg` + `agents.amp_ppo_cfg:PM01FlatAMPPPORunnerCfg`。
- `robots/pm01.py`：`PM01_CFG` 用 **URDF**（`assets/pm01/urdf/serial_pm01.urdf` + `.dae` 网格）生成，`replace_cylinders_with_capsules=True`、自碰撞开启；有效动作关节 `PM_WAIST_DFS_JOINT_NAMES` 为 **23 个**（J00–J22，含 `J12_WAIST_YAW`，**不含头部**）；刚度/阻尼按关节组手写（髋 pitch 110、踝 30、臂 50 等），力矩限制 Q90/Q25 混配。
- `robots/actuator.py`：`DelayedImplicitActuator`（环形缓冲随机延迟 PD），显式注明代码来自 BeyondMimic/whole_body_tracking。
- `flat_env_cfg.py`：`PM01FlatEnvCfg`，含金字塔坡/上坡/下坡/离散障碍/随机起伏 5 类子地形（curriculum=True、`max_init_terrain_level=5`）；`decimation=5`、`sim.dt=0.002`（控制 100Hz）、episode 20s；`PM01Commands`（`lin_vel_x=(-1.0,1.5)`、`lin_vel_y=±0.5`、`ang_vel_z=±1.0`、heading 控制、重采样 7.5s、10% 站立环境）；奖励大量使用课程缩放的自定义项（`action_smoothness_with_curriculum`、`action_rate_with_curriculum`、`feet_air_time_similarity`、`track_lin_vel_xy_yaw_frame_exp` 等）；终止条件为 base/膝/肩/肘接触；执行器经 `_build_delayed_actuators()` 加 2–8 物理步随机延迟。
- `flat_amp_env_cfg.py`：`PM01AMPFlatEnvCfg(PM01FlatEnvCfg)`，只替换观测组（环境/地形/奖励与 `Flat-PM01-v0` 相同）：
  - `policy`、`critic` 组：joint_pos/joint_vel（23 维，DFS 顺序）、actions、base_ang_vel、projected_gravity 均 `history_length=15`，velocity_commands；critic 与 policy 用**同一组观测**（`obs_groups={"actor":["policy"],"critic":["policy"]}`）。
  - `amp` 组：`history = ObsTerm(func=dummy_history_term)`，`history_length=5`；`dummy_history_term = concat(joint_pos(23)*9, base_lin_vel_b*3*7)`，并显式把 `WAIST_YAW` 的位置/速度置零（对齐 AMP 专家帧）。
- `agents/rsl_rl_ppo_cfg.py`：`PM01BasePPORunnerCfg`（experiment_name `velocity_flat_terrain`，max_iterations 3000、save_interval 50、MLP[512,256,128] ELU、自适应 lr 1e-3）；`PM01FlatPPORunnerCfg`（1500 iters，并在 `__post_init__` 中写 `self.policy.actor_hidden_dims=[128,128,128]`，但该类只有 `actor/critic` 字段，疑似无效，见第 7 节）。
- `agents/amp_ppo_cfg.py`：`PM01FlatAMPPPORunnerCfg`（experiment_name `velocity_flat_terrain_amp`，max_iterations 80000、save_interval 500，`style_reward_weight=2.0`、`frame_length=5`、`frame_dim=26`、`frame_normalization=True`、判别器 [512,256,128]，`dataset_path="dataset/config/dataset.yaml"`；`algorithm.class_name="engineai_lab.algorithms.amp_ppo:AMPPPO"`）。

### 3.2 AMP 算法与数据

- `algorithms/amp_ppo.py`：`AMPPPO(PPO)`，`construct_algorithm` 自建 `Discriminator` + `AMPDataLoader` 注入算法；`process_env_step` 中 `amp_reward = 0.01 * style_reward_weight * discriminator.get_amp_reward(obs["amp"])`，与任务奖励直接相加（非 lerp）；`update()` 中**每 4 次 PPO 更新**做一轮判别器更新（硬编码 `ppo_update_counter % 4 == 0`），损失为 MSE（专家→+1，策略→−1）加梯度惩罚（`compute_grad_pen`，λ=10，仅对专家样本），Adam lr=1e-4；日志含 `discriminator_loss/amp_grad_penalty/amp_expert_score/amp_policy_score`。
- `utils/AMP_discriminator.py`：MLP（ReLU，[512,256,128]）+ 逐帧 `EmpiricalNormalization`；奖励函数 `clamp(1 - 0.25*(d-1)^2, min=0)`。
- `utils/AMP_data_loader.py`：解析 `dataset.yaml` 的 `motions`（file 或 folder，支持权重）；要求 npz 键 `fps, joint_pos, joint_vel, body_pos_w, body_quat_w, body_lin_vel_w, body_ang_vel_w`；取“锚点刚体（body 0）”计算各刚体 base 系位姿/速度；`mini_batch_generator` 以 old→new 的 5 帧历史拼接特征，每帧为 `joint_pos*9 + base_lin_vel_b*7 = 26` 维（与 `frame_dim` 对应）。
- `dataset/config/dataset.yaml`：只挂 1 个文件 `../data/locomotion.npz`（无权重/无重排字段）。

### 3.3 脚本

- `scripts/train.py`：标准 Hydra + rsl_rl OnPolicyRunner 流程，日志写 `logs/rsl_rl/<experiment_name>/<时间戳>_<run_name>/`，保存 `params/env.yaml`、`params/agent.yaml`。
- `scripts/play.py`：加载 checkpoint（`load_run`/`checkpoint`）后调用 `ppo_runner.export_policy_to_onnx()` 导出 `exported/<checkpoint名>.onnx`；**不转 MNN、不写部署元数据**。
- `scripts/replay_npz.py`：`--motion_file <npz>`，用 `PM01_CFG` 在 Isaac Lab 回放全刚体动作（硬编码 PM01，无 `--robot` 参数）。
- `scripts/cli_args.py`：rsl_rl 通用参数（`--experiment_name/--run_name/--resume/--load_run/--checkpoint/--logger/--wandb_path`）。

## 4. 使用方法

```bash
# 1) 环境（README 步骤）
conda create -n engineai_lab python=3.11
conda activate engineai_lab
# 先按 Isaac Lab pip 安装文档装好 Isaac Sim 依赖；再克隆 Isaac Lab 并切换 main 分支
git clone https://github.com/isaac-sim/IsaacLab.git && cd IsaacLab
git checkout 4df6560e          # README 推荐：支持 rsl-rl-lib >= 5.0 与 Isaac Sim >= 5.0
./isaaclab -i rsl_rl
# 安装本包
cd /path/to/engineai_amp
pip install -e .

# 2) 训练（普通速度行走 / AMP 行走）
python scripts/train.py --task=Flat-PM01-v0     --num_envs 4096 --headless --run_name <name>
python scripts/train.py --task=Flat-AMP-PM01-v0 --num_envs 4096 --headless --run_name <name>

# 3) 回放训练效果（同时在 exported/ 导出 onnx）
python scripts/play.py --task=Flat-PM01-v0     --num_envs 128 --load_run <name>
python scripts/play.py --task=Flat-AMP-PM01-v0 --num_envs 128 --load_run <name>
# 运行名在 logs/rsl_rl/velocity_flat_terrain[_amp]/ 下查找

# 4) 回放参考动作
python scripts/replay_npz.py --motion_file dataset/data/locomotion.npz

# 5) ONNX → MNN（手动，README 提供步骤）
git clone https://github.com/alibaba/mnn && cd mnn && mkdir build && cd build
cmake .. -DMNN_BUILD_CONVERTER=ON && make -j8
./MNNConvert -f ONNX --modelFile path_to_your_policy.onnx --MNNModel your_policy.mnn --bizCode MNN
# 之后按 engineai_robotics_native_sdk 文档在真机加载 MNN
```

## 5. 依赖与环境

- **Python 3.11**（README 明确）；`setup.py` 声明 `python_requires>=3.10`。
- **Isaac Lab main 分支 commit `4df6560e`**（README 推荐），要求 **rsl-rl ≥ 5.0、Isaac Sim ≥ 5.0**；代码使用 rsl_rl 5.x 新 API（`rsl_rl.models.MLPModel`、`TensorDict`、`PPO.construct_algorithm`），旧版 Isaac Lab/rsl_rl 不兼容。
- **Python 依赖**：`psutil`、`onnxscript`、`wandb>=0.19`（setup.py）；URDF/mesh/动作数据已随仓库提供，无需额外下载（但见第 7 节 dataset 的 gitignore 问题）。
- **CUDA/驱动**：仓库未规定，随 Isaac Sim 5.x 官方要求（未确认具体版本）。
- 训练默认 4096 环境、AMP 训练 8 万迭代；硬件要求仓库未说明。

## 6. 与其他仓库的关系

- **与 engineai_rl_lab**：同组织、不同代码库和包名（`engineai_lab` vs `engineai_rl_lab`），无 import/依赖关系；rl_lab 是新一代框架（T800、tracking、更完整 AMP），本仓库是偏早期的 PM01 验证包，二者任务名不冲突（`Flat-*` vs `Tracking-*`/`AMP-Flat-*`）。
- **与 engineai_robotics_native_sdk**：native_sdk README 中 pm01_edu 的 `rl_lab` 状态描述写的是“为 engineai_amp 配套的部署代码”（链接指向本仓库）；但 rl_lab 仓库 README 同样对接 native_sdk。两处口径不一致，实际部署时以 native_sdk `assets/config/pm01_edu/...` 下的配置为准（待确认）。
- **与 engineai_legged_gym**：无代码引用/依赖。legged_gym 是 Isaac Gym（Python 3.8/Torch 1.13）旧代训练库；本仓库是 Isaac Lab 新一代，AMP 结构参考 `AMP_for_hardware`（README 致谢），延迟执行器代码来自 `whole_body_tracking`。
- **与 engineai_ros2_workspace**：无文档化对接；策略部署链路是 ONNX→MNN→native_sdk，而非 ROS2。

## 7. 注意事项、已知问题、文档缺失点

1. **`Flat-PM01-v0` 的 Runner 配置疑似有 bug**：`agents/rsl_rl_ppo_cfg.py` 的 `PM01FlatPPORunnerCfg.__post_init__` 设置 `self.policy.actor_hidden_dims / critic_hidden_dims`，但基类只有 `actor`/`critic` 字段；该赋值很可能报属性错误或静默无效（意图把策略缩小为 [128,128,128] 不生效）。AMP 任务不受影响。（未在真实环境验证）
2. **`dataset/` 被 `.gitignore` 忽略**，但快照中存在 `dataset/data/locomotion.npz`（3.3MB）。fresh clone 后该文件是否随 git 分发**未确认**；README 未说明动作数据来源与生成方式（文件名暗示为行走动作）。
3. 仓库内**没有 logs/checkpoint、没有导出好的 MNN**，README 练习需自行训练；模型跨 checkpoint 复用性未说明。
4. `config/extension.toml` 未更新（title "Extension Template"、repository 指向 isaac-sim/IsaacLab，dependencies 列了 isaaclab_mimic 等模板依赖）；`setup.py` 写 `license="MIT"`，与 `LICENSE`/README 的 BSD-3 不一致（以 LICENSE 文件为准）。
5. 判别器更新周期 4 次 PPO update、判别器优化器 lr=1e-4、梯度惩罚 λ=10 均**硬编码**在 `amp_ppo.py`，不可通过配置调整；`self.policy_update_counter = 0` 为笔误（实际计数器是 `ppo_update_counter`，不影响逻辑）。
6. AMP 观测维度强耦合：`frame_dim=26`（23 关节 + 3 线速度）、关节顺序为 PM01 DFS 顺序、`WAIST_YAW` 置零；更换机器人或改关节数需同步改数据加载/观测/配置。
7. `scripts/replay_npz.py` 硬编码 `PM01_CFG`，无机器人选择参数；`play.py` 只导出 ONNX，MNN 转换靠 README 的手动步骤，无 `deploy_config.yaml` 之类的部署元数据（与 rl_lab 的追踪导出不同）。
8. 无 T800/T800 Pro 支持（native_sdk 的 t800/t800pro 配置与本仓库无关）；无单元测试与 CI；README 的 “Evaluating a Policy” 一节实际写的是 AMP 训练命令，文档章节标注有误。
9. 环境为 URDF 转 USD，自碰撞开启、随机延迟执行器 2–8 物理步，训练开销高于固定执行器方案，README 未给出性能基准。
