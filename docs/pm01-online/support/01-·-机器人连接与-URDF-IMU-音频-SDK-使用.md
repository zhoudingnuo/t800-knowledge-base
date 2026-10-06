<!-- 来源: https://engineai.com.cn/open/docs/pm01/support/01?product=pm01 -->

# 01 · 机器人连接与 URDF / IMU / 音频 / SDK 使用

## 1 开发环境

### 1.1 机器人上有两套计算单元，我该连哪一个？有什么区别？

PM01-EDU 机载标配**双计算平台**，实现运控与 AI 应用的分离：

 | 对比项 | 应用计算单元 | 运控计算模组
 | **定位** | AI 推理、视觉处理等高算力任务 | 实时运动控制和底层通信
 | **型号** | Jetson Orin NX | Nezha（Intel N97，Alder Lake-N）
 | **IP 地址** | `192.168.0.162` | `192.168.0.163`
 | **SSH 用户名 / 密码** | `ubuntu` / `ubuntu` | `user` / `1`

AI 推理、视觉处理及应用部署通常在应用计算单元进行；运动控制相关开发应遵循对应 SDK 的部署说明。上述硬件与连接信息针对文中 PM01-EDU 配置，不应直接套用到所有 T800 或其他硬件批次。

**SSH 登录方式**：

 | Bash
```
# 应用计算单元
ssh ubuntu@192.168.0.162     # 密码 ubuntu

# 运控计算单元
ssh user@192.168.0.163       # 密码 1
```

---

## 2 模型与资料

### 2.1 哪里能拿到机器人的 URDF 模型？做可视化 / 运动规划 / 仿真要用。

模型可用于可视化、运动学计算和仿真验证。使用前应核对机器人型号、关节定义、网格与惯性参数，以及仓库版本；仿真结果仍需在安全条件下进行真机验证。

**获取渠道**：

 | 来源 | 地址 | 说明
 | Native SDK 仓库 | [https://github.com/engineai-robotics/engineai_robotics_native_sdk](https://github.com/engineai-robotics/engineai_robotics_native_sdk) | 使用前核对机型与模型版本
 | ROS2 开发仓库 | [https://github.com/engineai-robotics/engineai_ros2_workspace](https://github.com/engineai-robotics/engineai_ros2_workspace) | ROS2 二次开发主仓库

Native SDK 还提供完整的 MuJoCo 仿真环境与真机部署脚本，支持从仿真验证到真实机器人系统的快速迁移与部署。

---

## 3 传感器开发

### 3.1 怎么获取机器人的 IMU 数据？

**方式 A：通过 ROS2 话题获取（推荐）**

先确认 ROS2 环境、工作空间和对应数据发布节点已就绪。以下为文中环境使用的通信配置，不等同于 IMU 已成功发布；以机器人实际配置为准。

 | Bash
```
export ROS_DOMAIN_ID=69
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

先在需要运行 ROS2 命令的终端临时设置上述变量并验证通信，再决定是否写入 ~/.bashrc。下方持久化命令只需执行一次；重复追加会产生重复配置。

 | Bash
```
echo -e '\nexport ROS_DOMAIN_ID=69\nexport ROS_LOCALHOST_ONLY=0\nexport RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc && source ~/.bashrc
```

**开发前确认事项**：

- - ROS2 Humble 环境可用，`ros2 topic list` 能正常列出系统话题
- - engineai_ros2_workspace 已编译，`interface_protocol` 和 `interface_example` 包可被找到
- - 机器人硬件接口节点已启动（可通过 `ros2 topic echo /hardware/joint_state` 验证）

**方式 B：T800 Odin2 雷达内置 IMU**

T800 Odin2 雷达内置 IMU 与机体 IMU 是不同数据源。雷达话题后缀为 imu，完整名称需包含实际设备前缀；机体 IMU 的名称、类型与坐标系以当前接口文档和运行结果为准，不能用关节状态话题代替验证。

在已配置 ROS2 环境的终端列出话题及类型，找到与目标设备对应的 IMU 话题后，填写其完整名称：

 | Bash
```
ros2 topic list -t
# 将下列值替换为实际 IMU 话题全名
IMU_TOPIC=/实际的IMU话题
ros2 topic info "$IMU_TOPIC" -v
ros2 topic echo "$IMU_TOPIC"
```

应确认存在发布者、持续收到数据，并核对 frame_id、时间戳与传感器来源；无数据时按 Q10.2、Q11.1、Q11.2 排查。

---

## 4 音频开发

### 4.1 怎么使用机器人的麦克风录音和扬声器播放？

机器人内置麦克风和扬声器，可以通过 Linux 命令行工具快速使用：

 | Bash
```
# 录制音频
arecord -f cd test.wav

# 回放录制的音频
aplay test.wav
```

**进阶：PyAudio 异步录音**

官方提供基于 PyAudio 的异步麦克风录音示例，录制结束自动生成 WAV 音频文件。核心配置参数：

 | 配置项 | 默认值 | 说明
 | `recorder_device_index` | None | 麦克风设备编号，None 使用系统默认
 | `recorder_audio_rate` | 16000 | 采样率 16000 Hz（语音识别常用）
 | `recorder_chunk_size` | 1024 | 单次采集帧数
 | `recorder_channels` | 1 | 硬件总声道数
 | `recorder_pickup_channels` | [0] | 选取的声道索引列表

使用示例：

 | Python
```
recorder_config = {
    "recorder_device_index": None,   # 使用系统默认麦克风
    "recorder_audio_rate": 16000,    # 采样率 16000 Hz
    "recorder_chunk_size": 1024,     # 单次采集帧数
    "recorder_channels": 1,          # 单声道麦克风
    "recorder_pickup_channels": [0]  # 选取第 0 声道
}

recorder = Recorder(recorder_config)
# ... 循环采集音频 ...
recorder.close()
save_wav(audio_frames, sample_rate=16000, channels=1, save_path="./my_record.wav")
```

以上仅为配置与调用片段，不能直接保存运行：还需要 Recorder 类、save_wav 函数、音频采集循环及 audio_frames 数据。只安装 pyaudio 不能补齐这些内容。请按 [D3] 的完整实现运行；若仅验证设备录放音，可先使用本节开头的 arecord / aplay 命令，录制后按 Ctrl+C 停止，再回放。

---

## 5 SDK 调用排查

### 5.1 SDK 调用异常有哪些常见原因？

**Q：为什么消息发送后没有效果？**

- - 检查 QoS 配置是否匹配（Reliable vs Best Effort）
- - 确认 topic 名称是否正确
- - 检查机器人是否处于正确的运动状态（如 joint_bridge 模式下关节命令才生效）

**Q：如何确定关节索引？**

- - 参考机器人配置文档
- - `使用 ros2 topic echo /hardware/joint_state 查看关节数量`这个没必要
- 以当前机型接口定义中的关节名称与索引映射为准；关节数量不能确定各索引对应的关节。

**Q：控制不稳定怎么办？**

- 先停止当前运动，按官方安全调试流程确保机器人稳定、周围无人，并具备可用的急停措施；不要在失稳时直接在线试调。
- 核对控制模式、关节索引与单位、目标位置及限位，确认指令周期和状态反馈是否稳定。
- 刚度、阻尼和控制频率需按对应机型、控制器及任务的官方建议设置；不应把降低刚度、增加阻尼或提高频率作为通用修复方法。
- - 检查目标位置是否在合理范围内

**Q：request_id 错误如何处理？没懂什么问题**

- - 订阅状态 topic 获取当前 request_id
- 仅在当前接口明确采用递增请求编号协议时，按其规定设置 request_id；不要将“当前 ID + 1”推广到所有 SDK 接口。
- - 如果状态不同步，等待下一个状态消息

---

### 5.2 开发环境搭好了，但程序连不上机器人 / 调接口没反应，怎么系统排查？

**官方推荐的开发建议**：

1. 先使用示例程序测试接口是否正常

2. 逐步修改参数观察效果

3. 使用 `ros2 topic echo` 监控消息

4. 使用 `ros2 topic hz` 检查发布频率

5. 实现安全保护机制（速度限制、超时检测等）

**网络与环境检查**：

- - 确认 ROS2 通信环境变量已正确配置（`ROS_DOMAIN_ID=69`、`ROS_LOCALHOST_ONLY=0`、`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`），否则无法与机器人通信
- - 确认机器人硬件接口节点已启动：`ros2 topic echo /hardware/joint_state` 有数据流出即正常
- - 通过机器人背后的网口连接机器人，即可通过 ROS2 多机和机器人通信
- 如果调试电脑同时连接外网和机器人，请确保外网IP网段与机器人网络的192.168.0.0/24不同，避免路由冲突

**手柄遥控提示**：PM01 机身有两个 USB 插槽，手柄的 USB 接收器需插入**下方**接口才能正常遥控，插上方接口无效。

**其他常见问题**：

- **接收不到话题消息 / 话题列表缺失**：先确认 `ros2 topic echo <topic_name>` 与 `ros2 topic list` 的结果，再按 Q10.2、Q11.1 的步骤排查网络与 QoS 配置。
- **现有 SDK / API 无法满足需求**：如已有 SDK / API 中间层无法实现所需功能，需要开放新的接口或源码，或反馈机器人、传感器的功能与性能不足问题，请通过官方支持渠道提交需求，说明使用场景与必要性，我们会尽快评估并回复。
