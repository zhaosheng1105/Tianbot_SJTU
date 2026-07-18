# tianbot_drift_test

`tianbot_drift_test` 是独立于底盘驱动的 ROS 2 Python 测试包，用于 Tianbot
DYN 无转向圆周漂移的低速联调。底盘通信、消息和模式切换服务仍由
`tianbot_core` 提供；本包只负责测试状态机、闭环控制、参数配置和数据记录。

## 包结构

```text
tianbot_drift_test/
├── config/                         # 测试参数
├── launch/                         # ROS 2 启动文件
├── resource/                       # ament 包索引
├── tianbot_drift_test/             # Python 节点
├── package.xml
├── setup.cfg
└── setup.py
```

## 构建

```bash
cd ~/tianbot_ws
colcon build --packages-up-to tianbot_drift_test
source install/setup.bash
```

## 运行

先启动 DYN 底盘驱动：

```bash
ros2 launch tianbot_core tianbot_core.launch.py
```

启动测试节点：

```bash
ros2 launch tianbot_drift_test no_steer_circle_drift_test.launch.py
```

节点会自动请求 `/tianbot/set_control_mode` 切换到 MIT 模式，并等待底盘、
四个电机、IMU、里程计和陀螺零偏采样全部就绪。检查通过后默认倒计时 3 秒，
随后只自动执行一次测试，无需再调用 `start` 服务。

随时中止测试：

```bash
ros2 service call /no_steer_circle_drift_test/abort std_srvs/srv/Trigger "{}"
```

## 配置

默认配置位于 `config/no_steer_circle_drift_test.yaml`。可通过 launch 参数加载
另一份配置，或临时启用里程计半径反馈：

```bash
ros2 launch tianbot_drift_test no_steer_circle_drift_test.launch.py \
  config:=/absolute/path/to/drift_test.yaml \
  use_odom_radius:=true
```

需要恢复手动启动模式时，可在 launch 命令中关闭自动启动：

```bash
ros2 launch tianbot_drift_test no_steer_circle_drift_test.launch.py \
  auto_start:=false activate_mit:=false
```

DYN MIT 接口已经统一电机正方向，所以默认的 `motor_torque_signs` 是
`[1.0, 1.0, 1.0, 1.0]`。只有在其他硬件配置确实需要单轮反向时才应修改它。

## 安全说明

- 首次测试前应架空车轮确认四轮顺序和正方向。
- 保持足够大的封闭测试区域，并准备调用 `abort` 服务。
- 节点要求 IMU、里程计、MIT 模式和四个电机状态均正常。
- 默认配置包含速度、横摆角速度、轮端扭矩和扭矩变化率限制。
- CSV 日志默认写入 `~/.ros/drift_test_logs`。
