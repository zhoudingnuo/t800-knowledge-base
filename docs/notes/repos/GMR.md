# GMR — 把人体的 BVH / SMPL-X / AMASS 动作重定向到 EngineAI PM01 与 T800 的官方适配仓库

> 来源：https://github.com/engineai-robotics/GMR ，分支 main，快照 commit 71e02c0598f6（2026-08-05，提交信息"修改文档"）
> 快照日期：2026-10-03 ｜ 本地路径：repos/GMR/ ｜ 许可证：MIT（LICENSE 文件，Copyright 2025 Yanjieze；上游为 YanjieZe/GMR）

## 1. 定位与用途

本仓库是 EngineAI 对上游 [YanjieZe/GMR](https://github.com/YanjieZe/GMR)（General Motion Retargeting，arXiv:2510.02252）的机器人适配版本，在保留通用重定向框架的基础上，新增 PM01 与 T800 的 MuJoCo 模型及对应 IK 配置。

核心用途：把人体动捕数据（LAFAN1 BVH、SMPL-X / AMASS / OMOMO、GVHMR 预测结果）通过基于逆运动学的两阶段 IK 求解，转成 PM01 / T800 的全身关节轨迹（qpos），并支持 MuJoCo 实时可视化与导出 `.pkl` / `.csv` 数据集，供下游 RL 训练（如 engineai_rl_lab 的 whole-body tracking）使用。

上游 GMR 支持更多机器人（G1、H1 等），本适配仓库的 `--robot` 只注册了 `engineai_pm01`、`engineai_t800` 两种。

## 2. 目录结构（主要 2-3 层）

```text
GMR/
├── README.md / README_EN.md        # 中文/英文说明（数据准备、命令示例）
├── DOC.md                          # IK 配置字段（ik_match_table）注释说明
├── setup.py                        # pip 包 general_motion_retargeting 0.2.0 及依赖
├── LICENSE                         # MIT
├── assets/                         # 机器人模型资源（已内置，无需下载）
│   ├── engineai_pm01/              # PM01 的 MuJoCo 模型
│   │   ├── pm_v2.xml               # 入口：include xml/serial_pm_v2.xml + ground.xml
│   │   ├── ground.xml              # 地面
│   │   ├── meshes/                 # 25 个二进制 .STL（LINK_BASE + 24 个关节/连杆）
│   │   └── xml/                    # serial_pm_v2.xml、serial_links.xml、serial_actuators.xml、
│   │                               #   serial_sensors.xml、assets.xml
│   └── engineai_t800/              # T800 的 MuJoCo 模型
│       ├── t800_mocap.xml          # 入口：include xml/serial_t800.xml + ground.xml
│       ├── ground.xml
│       ├── meshes/                 # 26 个 .obj（MuJoCo 实际引用）+ 26 个 .dae
│       ├── texture/                # 16 张 .png 贴图
│       └── xml/                    # serial_t800.xml、serial_links.xml、serial_actuators.xml、
│                                   #   serial_sensors.xml、assets.xml
├── general_motion_retargeting/     # 核心 Python 包
│   ├── motion_retarget.py          # GeneralMotionRetargeting：两阶段 Mink IK 求解
│   ├── params.py                   # 机器人 XML 与输入源→IK 配置注册表
│   ├── ik_configs/                 # 4 个 JSON：smplx_to_pm01/t800、bvh_lafan1_to_pm01/t800
│   ├── robot_motion_viewer.py      # MuJoCo 可视化/录像、可视化人体对比
│   ├── data_loader.py              # load_robot_motion：读 .pkl 并把四元数 xyzw→wxyz
│   ├── kinematics_model.py         # 解析 MuJoCo XML 做正运动学（FK）
│   ├── neck_retarget.py            # SMPL-X 头部→机器人 neck yaw/pitch
│   ├── xrobot_utils.py             # XRoboToolkit 实时流/录制（可选依赖，缺失时跳过）
│   ├── rot_utils.py / torch_utils.py
│   └── utils/
│       ├── lafan1.py               # load_bvh_file：LAFAN1 BVH 读取（含 nokov 分支）
│       ├── smpl.py                 # load_smplx_file / load_gvhmr_pred_file、fps 对齐
│       └── lafan_vendor/           # 上游附带的 BVH 解析代码（extract.py、utils.py）
├── scripts/                        # 8 个命令行脚本（见第 3 节）
└── third_party/poselib/            # 上游 GMR 附带的 pose/skeleton 工具库
```

## 3. 核心模块与关键文件

| 文件（相对路径） | 作用 |
| --- | --- |
| `general_motion_retargeting/params.py` | `ROBOT_XML_DICT`：pm01→`assets/engineai_pm01/pm_v2.xml`，t800→`assets/engineai_t800/t800_mocap.xml`；`IK_CONFIG_DICT[src_human][robot]` 注册 smplx / bvh_lafan1 两套配置 |
| `general_motion_retargeting/motion_retarget.py` | IK 主类；默认 `solver="daqp"`、`damping=5e-1`、`max_iter=10`；先解 `ik_match_table1` 再解 `ik_match_table2`，返回 MuJoCo `qpos` |
| `general_motion_retargeting/ik_configs/*.json` | `robot_root_name/human_root_name`、`human_height_assumption`、`human_scale_table`、两张 `ik_match_table`（机器人 body 名、人体骨骼名、位置/旋转权重、偏移） |
| `general_motion_retargeting/data_loader.py` | 读取本仓库输出的运动 pkl（字段：fps/root_pos/root_rot/dof_pos/local_body_pos/link_body_list） |
| `scripts/bvh_to_robot.py` | LAFAN1 BVH → 机器人；`--format` 仅 `lafan1`；`--save_path` 导出 pkl |
| `scripts/smplx_to_robot.py` | SMPL-X/AMASS npz 或 pkl → 机器人；固定对齐到 30 fps |
| `scripts/gvhmr_to_robot.py` | GVHMR 预测文件（`hmr4d_results.pt`）→ 机器人 |
| `scripts/convert_omomo_to_smplx.py` | OMOMO 原始 `.p`（joints24）→ 本仓库可读的 SMPL-X 格式 |
| `scripts/smpl_to_smplx.py` | SMPL `.npz` → SMPL-X `.npz`（老数据兼容） |
| `scripts/vis_robot_motion.py` / `vis_robot_motion_dataset.py` | 可视化单个 pkl / 整个 pkl 目录（空格暂停、`[` `]` 切动作），可录 mp4 |
| `scripts/batch_gmr_pkl_to_csv.py` | pkl 目录批量转 CSV（列 = root_pos 3 + root_rot xyzw 4 + dof_pos N；fps>30 时降采样到 30） |
| `assets/engineai_pm01/xml/serial_links.xml` | PM01 运动链：24 个 hinge 关节 `J00_HIP_PITCH_L`…`J23_HEAD_YAW`，根 body `LINK_BASE` 带 `<freejoint/>` |
| `assets/engineai_t800/xml/serial_links.xml` | T800 运动链：25 个 hinge 关节 `J00`…`J24_HEAD_YAW`（比 PM01 多 `J23_HEAD_PITCH`），根 body `LINK_BASE` 带 `<freejoint/>` |

机器人模型要点（与 description 仓库同名关节对应）：
- PM01：24 自由度（12 腿 + `J12_WAIST_YAW` + 左右各 5 臂 + `J23_HEAD_YAW`），24 个 motor；qpos 维数 7+24=31；mesh 为二进制 STL，无贴图。
- T800：25 自由度（12 腿 + `J12_TORSO_YAW` + 左右各 5 臂 + 头 pitch/yaw），25 个 motor；qpos 维数 7+25=32；`assets.xml` 引用 `.obj`+`.png`，`.dae` 仅作同目录副本。
- IK 配置中的 body 名（如 t800 的 `LINK_WAIST_YAW`、`LINK_WRIST_END_L`）均能在对应 XML 中找到（已核对，`LINK_BASE` 定义在入口 XML 中）。

## 4. 使用方法

```bash
# 安装（Ubuntu 20.04/22.04 + Python 3.10；注意 smplx 依赖需访问 GitHub）
conda create -n gmr python=3.10 -y && conda activate gmr
cd repos/GMR && pip install -e .
# 若 MuJoCo 渲染报 libstdc++ 错误：conda install -c conda-forge libstdcxx-ng -y
```

输入数据与存放建议：
- LAFAN1 BVH：只需 `.bvh` 文件，骨骼名需符合 LAFAN1（如 `LeftToe`，脚本内部映射 `LeftFootMod`）；其他命名/单位不能保证直接可用。
- SMPL-X / AMASS：需自行从 SMPL-X 官网下载 `SMPLX_NEUTRAL.pkl`、`SMPLX_FEMALE.pkl`、`SMPLX_MALE.pkl` 放到 `assets/body_models/smplx/`（目录被 .gitignore 忽略，仓库内不存在）。AMASS 必须用 SMPL-X 格式（`*_stageii.npz`），不要 SMPL+H。
- OMOMO：先 `convert_omomo_to_smplx.py` 转为 SMPL-X；GVHMR：直接传 `hmr4d_results.pt`。

```bash
# LAFAN1 BVH → T800
python scripts/bvh_to_robot.py --bvh_file path/to/motion.bvh --format lafan1 \
  --robot engineai_t800 --save_path outputs/t800_motion.pkl --rate_limit

# AMASS/SMPL-X → T800（--rate_limit 按帧率播放，去掉则全速）
python scripts/smplx_to_robot.py --smplx_file /path/to/AMASS/ACCAD/.../General_A1_-_Stand_stageii.npz \
  --robot engineai_t800 --save_path outputs/amass/t800_stand.pkl --rate_limit

# 可视化 / 录像
python scripts/vis_robot_motion.py --robot engineai_t800 \
  --robot_motion_path outputs/t800_motion.pkl --record_video --video_path videos/t800_motion.mp4

# pkl 目录批量转 CSV（供下游训练）
python scripts/batch_gmr_pkl_to_csv.py --folder outputs/t800_motions
```

输出格式（`--save_path` 生成的 pickle，全仓库脚本一致）：

| 字段 | 说明 |
| --- | --- |
| `fps` | 帧率，SMPL-X 流程固定对齐 30，BVH 由 `--motion_fps`（默认 30）给出 |
| `root_pos` | (T, 3) float，根位置 |
| `root_rot` | (T, 4) float，根四元数，**xyzw**（脚本内部 qpos 为 wxyz，保存时转为 xyzw） |
| `dof_pos` | (T, N)，N=24（PM01）/ 25（T800），顺序同 MuJoCo 关节顺序 |
| `local_body_pos` / `link_body_list` | 当前脚本均写 `None`（保留字段，未使用） |

CSV 格式：每行 `root_pos(3) + root_rot xyzw(4) + dof_pos(N)`，共 31/32 列，逗号分隔；>30 fps 自动降采样到 30。

## 5. 依赖与环境

- Python >= 3.10；Ubuntu 20.04 / 22.04（README 声明）。
- `setup.py` 声明：`loop_rate_limiters`、`mink`、`mujoco`、`numpy`、`scipy`、`qpsolvers[proxqp]`、`rich`、`tqdm`、`opencv-python`、`natsort`、`psutil`、`smplx`（git+https://github.com/vchoutas/smplx）、`protobuf`、`redis[hiredis]`、`imageio[ffmpeg]`、`joblib`。
- IK 求解链路：Mink（SE3/FrameTask + QP）→ qpsolvers → MuJoCo 模型；可视化/录制用 MuJoCo Renderer + imageio。
- 可选：`xrobotoolkit_sdk`（`xrobot_utils.py` 实时流/录制，缺失时该功能跳过，不影响主流程）。
- 注意：`motion_retarget.py` 默认 `solver="daqp"`，而 `setup.py` 只显式声明 `qpsolvers[proxqp]`；daqp 是否随 mink 依赖链安装未确认，若运行报求解器缺失需自行安装。

## 6. 与其他仓库的关系

- 与 `repos/engineai_robotics_description/`：README 明确说明 PM01/T800 模型"参考自"该官方描述仓库；GMR 中的模型是重新组织的 MuJoCo 版本（关节名 `J00_…`、body 名 `LINK_*` 与 URDF 一致），mesh 格式（PM01 用 STL、T800 用 OBJ）、XML 拆分方式均为 GMR 侧自建，不是 description 内容的直接拷贝（description 当前不含任何 mjcf/XML）。
- 与 `repos/engineai_rl_lab/`（whole-body tracking / AMP）：前者产出的 pkl 经 `batch_gmr_pkl_to_csv.py` 转为 CSV（31/32 列），rl_lab 的 `scripts/csv_to_npz.py` 读取同样的列定义（`[:, :3]` 根位置、`[:, 3:7]` 根旋转 xyzw、`[:, 7:]` 关节角）生成训练用 npz（含 `fps/joint_pos/joint_vel/body_pos_w/body_quat_w/...`），再由 `scripts/tracking/train.py --motion_file xxx.npz` 训练。rl_lab 的 AMP 数据加载器（`AMP_data_loader.py`）直接读取"compact GMR-style" npz（键 `fps/root_pos/root_rot/dof_pos`，注记"quaternions as xyzw"），与本仓库 pkl 字段一一对应。注意 rl_lab README 未写明 CSV 来源，衔接关系是从两边代码格式核对得到。
- 与 `repos/engineai_robotics_native_sdk/`、`repos/engineai_ros2_workspace/`：无直接引用；SDK/ROS2 各自内置模型资源（如 SDK `assets/resource/robot/`、ROS2 仿真 `assets/resource/robot/pm_v2/`）。
- `third_party/poselib` 为上游 GMR 附带（FBX/骨骼工具），本适配流程未直接使用其 FBX 功能。

## 7. 注意事项、已知问题、文档缺失点

- 许可证：代码 MIT；但机器人模型、SMPL-X 模型、AMASS/OMOMO/LAFAN1 数据各自有独立许可，需分别遵守。
- LFS：本快照已实际检查，全仓库未发现 git-lfs 指针文件（`git-lfs.github.com`、`oid sha256`、`version https://` 三种特征均 0 命中）；`assets/` 内均为真实二进制/文本模型文件，可直接使用。这是快照 tarball 的内容，上游 Git 仓库是否启用 LFS 未确认。
- `assets/body_models/`（SMPL-X 人体模型）被 .gitignore 忽略且快照中不存在，SMPL-X/AMASS/GVHMR 流程必须先手动下载模型，否则脚本会加载失败。
- `README` 声明"提供 LAFAN1 BVH 转换"，但 `bvh_to_robot.py` 的 `--format` 只有 `lafan1` 一个选项；`utils/lafan1.py` 里还有未暴露的 `nokov` 分支，非 LAFAN1 命名 BVH 需自行转换。
- 无单元测试、无 CI 配置；`DOC.md` 只解释了 IK 配置字段，未给出字段取值建议和向新机器人扩展的完整指南。
- 输出 pkl 的 `local_body_pos/link_body_list` 恒为 `None`，下游若需要全身刚体位置需自行用 FK 计算。
- `smplx` 包版本差异可能导致模型文件扩展名（.pkl/.npz）匹配问题，README 提示必要时修改 `smplx/body_models.py` 的 `ext`。
- T800 的 `t800_mocap.xml` 中 keyframe 被注释（PM01 的 `floating_base_homing` keyframe 有效）；README 未说明该差异。
- 快照为 GitHub 源码包（无 .git 元数据），commit/日期来自工作区维护的 snapshot.tsv，无法在本地用 git 复核。
