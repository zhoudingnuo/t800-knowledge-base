<!-- 来源: https://engineai.com.cn/open/docs/t800/ros2-interface/07?product=pm01 -->

# 7. LED 控制

## 1 接口概述

LED 控制接口用于控制机器人身上的 LED 灯效，支持多种颜色和灯效模式（闪烁、常亮、呼吸、流水等）。开发者通过发布 `LedControl` 消息来切换 LED 模式，命令会立即生效。

## 2 消息字段与模式枚举

**通信接口**：

 | 话题名称 | 消息类型 | 方向 | 说明
 | `/hardware/led_control` | `LedControl.msg` | 客户端 → 灯光控制服务 | 发送灯光控制请求

完整消息定义见 [GitHub 消息目录](https://github.com/engineai-robotics/engineai_ros2_workspace/tree/community/src/interface_protocol/msg)。

**支持的 LED 模式**：

 | 模式常量 | 说明 | 效果
 | `0` | 还原 | 还原内置灯光
 | `BLINK_RED` | 红色闪烁 | LED 红色快速闪烁
 | `BLINK_GREEN` | 绿色闪烁 | LED 绿色快速闪烁
 | `BLINK_BLUE` | 蓝色闪烁 | LED 蓝色快速闪烁
 | `BLINK_WHITE` | 白色闪烁 | LED 白色快速闪烁
 | `CONSTANT_ON_WHITE` | 白色常亮 | LED 白色持续点亮
 | `CONSTANT_ON_GREEN` | 绿色常亮 | LED 绿色持续点亮
 | `BREATHE_WHITE` | 白色呼吸 | LED 白色呼吸效果
 | `WATER_WHITE` | 白色流水 | LED 白色流水效果
 | `BREATHE_RED` | 红色呼吸 | LED 红色呼吸效果
 | `BLINK_ORANGE` | 橙色闪烁 | LED 橙色快速闪烁
 | `CONSTANT_ON_ORANGE` | 橙色常亮 | LED 橙色持续点亮

## 3 示例：LED 模式循环切换

**文件名**：[`led_control_example.py`](https://github.com/engineai-robotics/engineai_ros2_workspace/blob/community/src/interface_example/scripts/led_control_example.py)

本示例每隔 1 秒自动切换一次 LED 模式，循环展示所有可用的灯光效果。

**运行命令**：

 | Bash
```
python3 src/interface_example/scripts/led_control_example.py
```

**自定义 LED 控制**：

 | Python
```
import rclpy
from rclpy.node import Node
from interface_protocol.msg import LedControl

class LedControlExample(Node):
    def __init__(self):
        super().__init__("led_control_example")
        self.pub = self.create_publisher(LedControl, "/hardware/led_control", 10)
        # 每 1 秒切换一次 LED 模式
        self.modes = [LedControl.BLINK_RED, LedControl.BREATHE_WHITE,
                      LedControl.CONSTANT_ON_GREEN, LedControl.WATER_WHITE]
        self.idx = 0
        self.create_timer(1.0, self._tick)

    def _tick(self):
        msg = LedControl()
        msg.color = self.modes[self.idx]
        self.pub.publish(msg)
        self.get_logger().info(f"LED mode: {self.modes[self.idx]}")
        self.idx = (self.idx + 1) % len(self.modes)

rclpy.init()
rclpy.spin(LedControlExample())
```

## 4 注意事项

- LED 控制命令会立即生效
- 建议控制发布频率，避免过于频繁切换
- 某些模式组合可能不被硬件支持
- 如果发生内置系统错误或警告，ROS2 灯光设计接口将无效

📎 本章涉及的开源仓库源码文件（点击文件名跳转 GitHub）：[LedControl.msg](https://github.com/engineai-robotics/engineai_ros2_workspace/blob/community/src/interface_protocol/msg/LedControl.msg) · [led_control_example.py](https://github.com/engineai-robotics/engineai_ros2_workspace/blob/community/src/interface_example/scripts/led_control_example.py)
