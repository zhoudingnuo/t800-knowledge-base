# 04 - 用 Orin 中转控制 T800 Pro 真机（SSH + ROS2 实测）

> 2026-10-04 实机验证。本机（desk，192.168.1.204）的 USB 无线网卡是 **2.4GHz 单频**（AIC8800DC），
> 连不上机器人 5GHz 热点 `Robot_Wifi_T800_29C_0313`；而 Orin（192.168.1.50，Jetson AGX Orin 开发套件）
> 已通过 WiFi 接入机器人网络（IP `192.168.0.219`）。因此把 **Orin 当中转跳板**，本机所有控制都经它转发到机器人。
> 本文所有命令与结论均为实机验证结果。

## 1. 网络拓扑

```
本机 desk (192.168.1.204, Ubuntu 22.04, 用户 admin2)
   │  局域网 / 直连网线（SSH 免密）
   ▼
Orin (192.168.1.50, Jetson AGX Orin, 用户 orin-001, 有 GDM 桌面)
   │  WiFi: Robot_Wifi_T800_29C_0313 (5GHz 热点, 密码 e12345678) → 192.168.0.219
   ▼
机器人内网 192.168.0.0/24（机载路由器 192.168.0.1）
   ├─ 应用计算单元  Jetson Orin NX   192.168.0.162   ubuntu / ubuntu
   └─ 运控计算单元  Nezha N97        192.168.0.163   user / 1（控制接口都在这里）
```

凭据一览（官方默认值）：

| 机器 | IP | 用户 | 密码 |
|---|---|---|---|
| Orin 中转机 | 192.168.1.50 | orin-001 | 123 |
| 机器人应用单元 | 192.168.0.162 | ubuntu | ubuntu |
| 机器人运控单元 | 192.168.0.163 | user | 1 |

## 2. 登录链路

### 2.1 本机 → Orin（免密）

```bash
# 一次性配置（已配好）
ssh-keygen -t ed25519 -N "" -f ~/.ssh/id_ed25519
ssh-copy-id orin-001@192.168.1.50        # 密码 123
cat >> ~/.ssh/config << 'EOF'
Host orin
    HostName 192.168.1.50
    User orin-001
    IdentityFile ~/.ssh/id_ed25519
EOF
ssh orin hostname                        # 之后直接 ssh orin 即可
```

### 2.2 Orin → 机器人（经中转，不可直连时）

```bash
# 应用计算单元 .162
sshpass -p ubuntu ssh -o StrictHostKeyChecking=no ubuntu@192.168.0.162

# 运控计算单元 .163（ROS2 控制接口所在，控制命令都在这台上执行）
sshpass -p 1 ssh -o StrictHostKeyChecking=no user@192.168.0.163
```

一次性从本机穿透两跳执行命令（stdin 透传，便于下发脚本）：

```bash
ssh orin 'sshpass -p 1 ssh -o StrictHostKeyChecking=no user@192.168.0.163 bash -s' << 'EOF'
hostname; ip -br addr | head -5
EOF
```

## 3. 机器人本体现状（2026-10-04 实测）

- 运控单元 .163 上 `robotics.service`（自启动）运行的是 **Native SDK**：
  `/apps/engineai_robotics/run_robot.sh t800pro` → `src_executor t800pro`（这是官方交付的运控栈）。
- 该栈**内置了完整的 ROS2 对外接口**（`ros2_bridge_runner` 在 .163 上起了多个节点），
  文档 `docs/t800-online/ros2-interface/` 中列出的接口全部可用，无需另装 ros2_workspace。
- **重要**：控制命令必须在 **.163 本机**执行。从 .162 或 Orin 上做 ROS2 发现只能看到部分话题
  （跨主机 DDS 发现不对称，实测 .162 看不到 `joint_state`/`set_motion_state` 等），
  典型症状是 `ros2 topic list` 少一半、`topic echo` 报 "message type invalid"。
- **电机使能是硬前提**：`/hardware/power_info.enable` 必须为 `true`（用急停遥控器使能电机系统）。
  未使能时日志每 2 秒刷 `Motor bus not ready (self-check)`，且**状态切换会被拒绝**
  （日志：`Transition from [ idle ] to [ passive ] rejected by runner`）。

## 4. 每条 SSH 会话的环境准备（.163）

```bash
source /opt/ros/humble/setup.bash
source /apps/engineai_robotics/setup_board_ros_env.bash   # 内含 ROS_DOMAIN_ID=69 与 msg PYTHONPATH
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=0
```

验证环境正常：

```bash
ros2 topic list | head            # 应看到 /hardware/*、/motion/* 全套
ros2 node list                    # hardware_interface_node、locomotion_interface_node 等
```

## 5. 常用操作（均已实测通过）

### 5.1 读状态

```bash
ros2 topic echo /motion/motion_state --once --qos-reliability best_effort
ros2 topic echo /hardware/power_info  --once --qos-reliability best_effort   # enable/电量/电压
ros2 topic echo /hardware/joint_state --once --qos-reliability best_effort   # 43 关节 位置/速度/力矩
ros2 topic echo /hardware/gamepad_keys --once --qos-reliability best_effort  # 手柄/遥控器输入
```

### 5.2 状态机切换

```bash
ros2 topic pub --once /motion/set_motion_state interface_protocol/msg/MotionStateRequest \
  "{target_motion_name: lower_body_balance}"
```

规则：目标必须在 `available_transition_motions` 里；**每次只发一条**；3 秒超时；
实际转换可能被 runner 拒绝（如电机未使能）。

2026-10-04 实测状态路径：`idle →(急停遥控器使能，运控自动进入)→ rl_basic`；
`rl_basic` 可直接切 `lower_body_balance`（上肢可控制）、`pd_stand`、`passive`。
`joint_bridge`（全关节透传）需从 `pd_stand` 进入。

### 5.3 手指控制（上肢覆盖控制，lower_body_balance 模式）

接口：`/motion/joint_override_command`（`JointOverrideCommand`，100Hz，官方示例同款）。
手指关节索引：**左手 J20–J26、右手 J34–J40**（各 7 个）。官方"虚握拳"角度（已在实机验证，跟踪误差 ≈0.01 rad）：

```python
IDX   = list(range(20,27)) + list(range(34,41))
FIST  = [-0.85, 0.62, 0.48, -0.80, -0.58, -0.80, -0.58,   # 左
          0.85, 0.62, 0.48,  0.80,  0.58,  0.80,  0.58]   # 右
OPEN  = [0.0]*14
# 发布的 JointOverrideCommand 字段：
#   weight=1.0, joint_indices=IDX, position=目标角, velocity/feed_forward_torque/torque=0
#   stiffness=20.0, damping=0.2（低刚度，手指专用，官方安全值）
```

完整脚本见本知识库 `robot_control/examples/finger_demo.py`（本机与机器人上各一份）。
手腕（J18/J19/J32/J33）不在该接口允许列表内，需 `joint_bridge` + `/hardware/joint_command`。

### 5.4 LED

```bash
ros2 topic pub --once /hardware/led_control interface_protocol/msg/LedControl "{color: 1}"   # 闪红
ros2 topic pub --once /hardware/led_control interface_protocol/msg/LedControl "{color: 0}"   # 还原
```

### 5.5 全关节控制（joint_bridge + joint_command）

前置：进入 `joint_bridge`；数组长度必须 = **43**（T800 Pro）；500Hz、Best Effort；
`tau = kp*(q_cmd-q) + kd*(qd_cmd-qd) + feed_forward_torque`。
官方示例：`engineai_ros2_workspace/src/interface_example/scripts/joint_bridge_example.py`
（YAML 目标 + 线性插值）。组别增益参考（官方 `config/t800pro/joint_test.yaml`）：

| 组 | 关节 | kp | kd |
|---|---|---|---|
| 左/右腿 | J00–J11 | 1080/480/880/1000/800/100 | 25/25/25/25/2/2 |
| 腰 | J12 | 200 | 1 |
| 手臂(含腕) | J13–J19 / J27–J33 | 120×5 + 30×2 | 1.8/1.5/1.5/1.8/1.2/0.8/0.8 |
| 手 | J20–J26 / J34–J40 | 20 | 0.04（实测 0.2 更稳） |
| 头 | J41–J42 | 100 | 1 |

## 6. 关键坑（实机踩过）

1. **跨主机 DDS 不对称**：控制命令（发布）必须在 .163 本机执行；Orin/.162 上只做不了完整控制。
   后续如需从 Orin 直接控制，需在其上补装 `rmw_cyclonedds_cpp`（arm64）与 `interface_protocol`（arm64）。
2. **电机未使能**：状态切换会被 runner 拒绝，必须先让 `/hardware/power_info.enable == true`。
3. **本机连不上机器人热点**：5GHz 单频网卡问题，只能走 Orin 中转；换双频网卡可直连。
4. **.162 上没有编译工作区**：`~/source/engineai_workspace` 只有 src/scripts，没有 install，
   所以 .162 上 `ros2 topic echo /motion/motion_state` 会报 `message type invalid`（缺 msg 类型）。
5. **灯光接口在系统告警态可能被覆盖**（如电机未使能时的内部告警）。
6. **LocalDesk**（图形远控）与 SSH 各管一路：图形看桌面用 LocalDesk（`http://127.0.0.1:8899/?auto=1`），
   跑命令/传文件用 SSH（快、可脚本化）。

## 7. 可视化控制面板（已实现，2026-10-04 实机验证）

为了在**本机**开发图形化控制面板（Electron），搭了一条从本机到 .163 的**持续 TCP 通道**
（SSH 每次命令有 1–2 秒冷启动，做不了滑杆实时控制）。工程在 `robot_control/`：

```
desk 本机                         Orin 192.168.1.50               机器人 .163
┌────────────────────┐   TCP    ┌──────────────────┐   TCP(WiFi) ┌──────────────────┐
│ app/  Electron 面板 │ ───────▶ │ relay/relay.py   │ ──────────▶ │ bridge/bridge.py │
│  (43 滑杆/状态机)   │ :9801    │  :9801 → .163    │  :9801      │  ROS2 ↔ TCP      │
└────────────────────┘          └──────────────────┘             └────────┬─────────┘
                                                       /hardware/joint_command（200Hz）
                                                       /motion/set_motion_state
```

启动（三步）：

```bash
# 1) 机器人侧 bridge（.163，已部署在 ~/t800_bridge/）
sshpass -p 1 ssh -J orin user@192.168.0.163 \
  'cd ~/t800_bridge && setsid nohup ./start_bridge.sh > bridge.log 2>&1 < /dev/null &'

# 2) Orin 中转（relay.py 已部署到 ~/t800_relay.py）
ssh orin 'setsid nohup python3 ~/t800_relay.py --listen 192.168.1.50 --port 9801 \
  --target 192.168.0.163 --target-port 9801 > ~/t800_relay.log 2>&1 < /dev/null &'

# 3) 本机面板（Electron 已装好）
cd robot_control/app && npm start
```

**实机验证记录**：面板 43 个滑杆 + 实时状态显示正常；经面板链路完成
`lower_body_balance → rl_basic → pd_stand → joint_bridge` 模式切换，
并在 joint_bridge 下控制头部航向 J42：`0.000 → 0.200 rad`（精确跟踪）→ 复位 `0.000`，
最终保持在 joint_bridge、200Hz 持续下发保持位姿。

**调试中发现的坑（已修复，写进 robot_control/README.md）**：

1. `rclpy` 的 Node 内部已有 `_clients` 属性（service client 列表），bridge 里同名属性覆写会让执行器崩溃 → 改名 `_tcp_clients`。
2. `JointCommand.stiffness/damping`（float64[]）的 setter 断言每个元素必须是 `float`，整数增益（1080 等）会 AssertionError 崩溃 → 统一 `float()`。
3. relay 上游 socket 继承 `create_connection(timeout=5)` 的读超时，空闲 5 秒连接被关 → 连上后 `settimeout(None)`。
4. 面板初版：未触碰关节的目标初值为 0，单拖一个滑杆会把其余关节目标拉到 0 → 改为未触碰关节实时跟随当前角度。

替代方案（等效）：在 Orin 上开 `net.ipv4.ip_forward=1` + iptables MASQUERADE，
本机加 `ip route add 192.168.0.0/24 via 192.168.1.50`，即可直连 .163（重启后需重设）。

## 8. 相关文件

- 控制面板工程：`robot_control/`（Electron 应用 + bridge + relay + 示例脚本）
- 机器人原生 ROS2 接口文档：`docs/t800-online/ros2-interface/`（01–10 页）
- Native SDK 笔记：`docs/notes/repos/engineai_robotics_native_sdk.md`
- ROS2 工作区笔记：`docs/notes/repos/engineai_ros2_workspace.md`
