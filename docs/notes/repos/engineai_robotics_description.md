# engineai_robotics_description — PM01 / T800 / T800 Pro 的官方机器人模型描述仓库（URDF + mesh + USD）

> 来源：https://github.com/engineai-robotics/engineai_robotics_description ，分支 master，快照 commit 1d2d7601f520（2026-08-14，提交信息"add pm01_lidar"）
> 快照日期：2026-10-03 ｜ 本地路径：repos/engineai_robotics_description/ ｜ 许可证：未提供（快照根目录只有 README.md、README_EN.md 与各机型目录，无 LICENSE / COPYING 文件）

## 1. 定位与用途

EngineAI 官方机器人模型描述仓库，按机型/版本文档化维护机器人的几何与运动学资源，供仿真（MuJoCo / Isaac）、SDK、ROS2 及第三方（如 GMR）引用。仓库自述内容包括 URDF、MuJoCo XML、mesh、贴图等；每个机器人一个独立目录。

当前快照只包含 2 个 PM01 版本、1 个 T800 版本和 2 个 T800 Pro 版本，共 520 个文件、约 1.5 GB。README 顶部有重要说明：**pm01_edu、t800、t800pro 尚未同步更新到 engineai_rl_lab 与 engineai_robotics_native_sdk**（即下游仓库内是各自维护的副本）。

## 2. 目录结构（主要 2-3 层）

```text
engineai_robotics_description/
├── README.md / README_EN.md            # 机型列表、目录约定、版本对照表（无 LICENSE）
├── pm01_edu/                           # PM01 Edu（v2）
│   ├── urdf/serial_pm01_edu.urdf       # 机器人名 engineai_pm01
│   ├── meshes/                         # 25 × .dae + 25 × .obj（成对，覆盖 LINK_BASE 及各连杆）
│   └── texture/                        # 23 × .png（按 LINK_*.png 命名）
├── pm01_lidar/                         # PM01 导航版（v1）
│   ├── urdf/serial_pm01_edu.urdf       # 文件名与 pm01_edu 相同；多一个 LIVOX_FRAME 固定连杆
│   ├── meshes/                         # 25 × .dae + 25 × .obj
│   └── texture/                        # 23 × .png
├── t800/                               # T800（v1.1）
│   ├── urdf/serial_t800.urdf           # 机器人名 t800
│   ├── meshes/                         # 26 × .dae + 26 × .obj + 5 × .STL（O2S 传感器件）
│   └── texture/                        # 16 × .png
└── t800pro/
    ├── t800pro_25dof/                  # T800 Pro（v1.1，25 自由度）
    │   ├── urdf/serial_t800pro.urdf
    │   ├── meshes/                     # 44 × .dae + 44 × .obj + 20 × .mtl（手指件）
    │   ├── texture/                    # 34 × .png
    │   └── usd/                        # Isaac Lab 用：入口 serial_t800pro.usd（二进制 USDC）
    │                                   #   + configuration/{robot,physics,sensor,base}.usd
    └── t800pro_43dof/                  # T800 Pro（v1.2，43 自由度）
        ├── urdf/serial_t800pro.urdf
        ├── meshes/                     # 44 × .dae + 44 × .obj + 20 × .mtl + 5 × .STL（O2S）
        └── texture/                    # 34 × .png
```

说明：README 的"每目录通常包含 urdf/xml/meshes/texture/usd"只是通用约定；**本快照没有任何 `xml/`（MuJoCo）或 xacro/mjcf 文件**（各版本对照表中 XML 列全部为 ✗），USD 也只在 t800pro_25dof 中存在。快照无 `.git` 元数据（tarball 解包），commit 信息来自工作区 snapshot.tsv。

## 3. 核心模块与关键文件

各机型 URDF（相对路径）与自由度定义：

| 目录 | URDF 文件 | 机器人名 | 连杆数 | 关节总数 | 可动关节（DOF） |
| --- | --- | --- | --- | --- | --- |
| pm01_edu | `pm01_edu/urdf/serial_pm01_edu.urdf` | engineai_pm01 | 31 | 30（24 revolute + 6 fixed） | 24 |
| pm01_lidar | `pm01_lidar/urdf/serial_pm01_edu.urdf` | engineai_pm01 | 32 | 31（24 revolute + 7 fixed） | 24 |
| t800 | `t800/urdf/serial_t800.urdf` | t800 | 35 | 34（25 revolute + 9 fixed） | 25 |
| t800pro_25dof | `t800pro/t800pro_25dof/urdf/serial_t800pro.urdf` | t800pro | 46 | 45（25 revolute + 20 fixed） | 25 |
| t800pro_43dof | `t800pro/t800pro_43dof/urdf/serial_t800pro.urdf` | t800pro | 51 | 50（43 revolute + 7 fixed） | 43 |

DOF 命名规律与分组（关节名 `J<两位序号>_<部位>`，序号在左右侧/手部有跳号）：
- 腿（所有机型一致，12）：`J00_HIP_PITCH_L`、`J01_HIP_ROLL_L`、`J02_HIP_YAW_L`、`J03_KNEE_PITCH_L`、`J04_ANKLE_PITCH_L`、`J05_ANKLE_ROLL_L`，右侧 `J06`–`J11` 同序。
- 腰/躯干（1）：PM01 为 `J12_WAIST_YAW`，T800 系列为 `J12_TORSO_YAW`。
- 臂（每侧 5）：`SHOULDER_PITCH/ROLL/YAW`、`ELBOW_PITCH/YAW`；PM01/T800 为 `J13`–`J22`，T800 Pro 左臂 `J13`–`J17`、右臂 `J27`–`J31`。
- 头：PM01 只有 `J23_HEAD_YAW`；T800 为 `J23_HEAD_PITCH` + `J24_HEAD_YAW`；T800 Pro 为 `J41_HEAD_PITCH` + `J42_HEAD_YAW`。
- T800 Pro 43 DOF 的手腕与手指（每侧 7）：`J18/J19` 腕 pitch/roll、`J20`–`J22` 拇指 3 关节、`J23/J24` 食指 2 关节、`J25/J26` 中指 2 关节，右侧为 `J32`–`J40`。
- 固定关节：脚底 `J_FIXED_FOOT_L/R`、末端 `J_FIXED_ELBOW_END_*`/`J_FIXED_WAIST_*`；传感器件 `LINK_O2S_IMU/LINK_O2S_LEFT_LIDAR/LINK_O2S_RIGHT_LIDAR/LINK_O2S_LEFT_RGB/LINK_O2S_RIGHT_RGB` 以 fixed joint 挂载（t800、t800pro_43dof）。
- pm01_lidar 比 pm01_edu 多 `J_FIXED_LIVOX_FRAME → LIVOX_FRAME`；pm01_edu/pm01_lidar 另有相机固定件 `camera_link_head`、`camera_link_torso`。
- t800pro_25dof 将腕部与手指关节全部设为 fixed（J18–J26、J32–J40 不在可动列表中），使 25 个可动关节与 T800 对应；无 camera/lidar 连杆。

文件命名与 mesh 规律：
- URDF 文件名固定为 `serial_<model>.urdf`；pm01 两个目录都用 `serial_pm01_edu.urdf`，t800pro 两个目录都用 `serial_t800pro.urdf`。
- mesh 以 `LINK_<部件>.dae` / `.obj` 成对出现（如 `LINK_HIP_PITCH_L.dae/.obj`）；手指/腕部件另有同名 `.mtl`（每个 t800pro 目录 20 个）。
- O2S 传感器件只有 `.STL`：`LINK_O2S_IMU.STL`、`LINK_O2S_LEFT_LIDAR.STL`、`LINK_O2S_LEFT_RGB.STL`、`LINK_O2S_RIGHT_LIDAR.STL`、`LINK_O2S_RIGHT_RGB.STL`。
- 贴图按部件命名（`LINK_*.png`）：PM01 各 23 张、T800 16 张、T800 Pro 各 34 张。
- mesh 引用路径两类：pm01/t800 为 `package://resource/robot/<model>/meshes/...`（ROS package 形式）；t800pro 为相对路径 `meshes/...`（需从 urdf/ 目录同级解析）。

mesh 规模（各目录合计约）：pm01_edu 373 MB、pm01_lidar 356 MB、t800 115 MB、t800pro_25dof 332 MB、t800pro_43dof 262 MB。单个最大：pm01_edu 的 `LINK_TORSO_YAW.dae` 约 27.8 MB；t800pro 的 `LINK_INDEX_TIP_R.dae`/`LINK_MIDDLE_TIP_L.dae` 约 10.7 MB；t800pro_25dof USD 中 `configuration/serial_t800pro_base.usd` 约 72.6 MB。全仓库 dae/obj 各 164 个、png 130 张、STL 10 个（t800 与 43dof 各 5）。

t800pro_25dof 的 USD：入口 `serial_t800pro.usd` 是二进制 USDC，内部 sublayer/reference 指向 `configuration/serial_t800pro_robot.usd`、`..._physics.usd`、`..._sensor.usd`、`..._base.usd`，为 Isaac Lab 的机器人装配结构；43dof 目录没有 USD。

## 4. 使用方法

本仓库是纯资源仓库，无安装脚本、无 build/launch 文件。使用方式是把对应机型目录作为模型资源接入下游：

```bash
# 1) MuJoCo：本仓库无 mjcf；需使用 GMR 等仓库中转换好的 XML
#    （GMR/assets/engineai_pm01/pm_v2.xml、GMR/assets/engineai_t800/t800_mocap.xml）

# 2) 普通 URDF 工具（示例：check_urdf / 转 xacro）
check_urdf repos/engineai_robotics_description/t800/urdf/serial_t800.urdf
#    注意：mesh 使用 package://resource/robot/... 前缀，直接加载会提示找不到
#    meshes，需要把目录安装/软链为名为 resource 的 ROS package，
#    或批量替换为相对路径；t800pro 的 URDF 用相对路径，从 urdf/ 同级解析即可。

# 3) Isaac Lab：仅 T800 Pro 25dof 提供 USD（把 usd/ 目录作为资产根）
#    usd_path: <repo>/t800pro/t800pro_25dof/usd/serial_t800pro.usd
```

输入/输出说明：URDF 输入是描述文件本身，输出（在仿真器中加载后）为对应机型的刚体/关节模型；关节顺序以 URDF 中的出现顺序为准（J00→J42 顺序基本单调，但 T800 Pro 左右臂/头部序号有跳号，接入控制器时需按名字而不是序号索引）。无独立的配置/参数文件。

## 5. 依赖与环境

- 无代码、无 requirements、无 package.xml（`package://resource/...` 引用的 ROS package 需下游自行提供）。
- URDF 需 URDF 解析器（ROS `urdf`、MuJoCo、Isaac 等）；mesh 为 DAE/OBJ/STL，纹理为 PNG。
- USD 需 OpenUSD / Isaac Sim 或 Isaac Lab 读取（二进制 USDC，无法用文本工具审阅）。
- 文件体量：仓库约 1.5 GB，使用/分发时注意完整克隆成本。

## 6. 与其他仓库的关系

- README 明确声明 pm01_edu、t800、t800pro **尚未同步**到 `repos/engineai_rl_lab/` 与 `repos/engineai_robotics_native_sdk/`；这两个仓库各带一份独立资源副本：
  - native_sdk：`assets/resource/robot/{pm01_edu,t800,t800pro}/` 含 urdf/xml/meshes/texture，配置在 `assets/config/<model>/model/default.yaml`（引用 `serial_t800.urdf` + `t800.xml`、按 `J00_...` 列 limbs）。
  - rl_lab：`assets/{pm01,t800}/serial_*.usd` + `configuration/*.usd`（Isaac Lab 用），另有独立 `config.yaml`/`.asset_hash`。
  - ROS2 仿真（`engineai_ros2_workspace/src/simulation/mujoco/assets/`）自带 `pm_v2.xml` + urdf/meshes，配置 `config/pm_v2.yaml` 声明 `num_total_joints: 24`。
- GMR（`repos/GMR/`）：README 声明 PM01/T800 模型"参考自"本仓库，但 GMR 内是自建 MuJoCo 模型（关节/连杆命名与本仓库一致），且 meshes 格式经转换（PM01 使用 STL、T800 使用 OBJ），不是本仓库目录的直接引用。
- engineai_amp 等仓库引用的是各自路径下的 `serial_pm01*.urdf` 副本（如 `engineai_amp/source/engineai_lab/robots/pm01.py` 指向其自身 ASSET_DIR 下的 URDF）。

## 7. 注意事项、已知问题、文档缺失点

- **许可证未提供**：仓库根无 LICENSE/COPYING，README 也未写许可条款；商用/再分发前必须向 EngineAI 确认。
- **README 与实际内容不一致**：README 称本仓库包含 MuJoCo XML，但快照内没有任何 `xml/`、mjcf、xacro 文件（对照表中 XML 均为 ✗），MuJoCo 资源目前实际由 GMR 提供；"camera_link/lidar_link" 列中 t800pro_25dof 标 ✗（其 URDF 确无传感器连杆），t800/43dof 有 `LINK_O2S_*` 传感器件（命名与表头表述不完全一致）。
- **pm01_lidar 的 mesh 引用指向 pm01_edu 路径**（`package://resource/robot/pm01_edu/meshes/...`），而不是自身目录；两目录 mesh 文件是否逐字节相同未确认，若作为独立 package 安装可能解析错误。
- **URDF 文件名不体现版本**：pm01_edu 与 pm01_lidar 同名 `serial_pm01_edu.urdf`，t800pro 两个版本同名 `serial_t800pro.urdf`，只能靠目录区分，脚本/工具批量处理时容易混淆。
- t800pro_25dof 与 43dof 的收敛差异只在关节 fixed/可动与传感器件；USD 仅 25dof 有，README 未说明生成工具链（Isaac URDF importer 推断，未确认）。
- 无 changelog、无版本标签说明、无 mesh 来源/CAD 转换说明；快照为 tarball（无 git 历史），无法核对 commit 内容。
- 未发现 git-lfs 指针（资源均为真实文件）；但若上游 Git 启用 LFS，克隆方仍需安装 git-lfs，本快照不受影响。
