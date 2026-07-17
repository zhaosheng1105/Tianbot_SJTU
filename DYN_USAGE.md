# DYN 使用说明

## 启动驱动

```bash
source /opt/ros/humble/setup.bash
source ~/tianbot_ws/install/setup.bash
ros2 launch tianbot_core tianbot_core.launch.py
```

当前默认参数：
- `serial_port=/dev/tianbot_base`
- `type=dyn`
- `type_verify=false`

## 常用检查

```bash
ros2 node list
ros2 topic list
ros2 topic echo /tianbot/motion_mode_status --once
ros2 topic echo /tianbot/motor_feedback --once
ros2 topic echo /tianbot/voltage --once
ros2 topic echo /tianbot/odom --once
```

## 模式切换

切速度模式：

```bash
ros2 service call /tianbot/set_control_mode tianbot_core/srv/SetControlMode "{mode: 'speed', save_to_flash: false}"
```

切 MIT 模式：

```bash
ros2 service call /tianbot/set_control_mode tianbot_core/srv/SetControlMode "{mode: 'mit', save_to_flash: false}"
```

## 关键话题

- `/tianbot/cmd_vel`：整车速度控制
- `/tianbot/wheel_speed_cmd`：四轮独立速度控制
- `/tianbot/wheel_mit_cmd`：四轮 MIT 控制
- `/tianbot/motion_mode_status`：当前模式状态
- `/tianbot/motor_feedback`：四轮电机反馈
- `/tianbot/voltage`：电池电压
- `/tianbot/odom`：里程计
- `/tianbot/imu`：IMU 数据

## 自旋测试脚本

脚本位置：

```bash
~/tianbot_ws/spin_in_place.py
```

这个脚本会先调用 `/tianbot/set_control_mode` 切模式，等待 `/tianbot/motion_mode_status` 确认成功后，再用常驻 ROS2 节点持续发布控制命令。

### 速度模式原地自旋

```bash
python3 ~/tianbot_ws/spin_in_place.py speed
```

### MIT 模式原地自旋

```bash
python3 ~/tianbot_ws/spin_in_place.py mit
```

## 脚本参数

```bash
python3 ~/tianbot_ws/spin_in_place.py speed --wz 1.5 --forward-sec 3 --pause-sec 1 --reverse-sec 3
python3 ~/tianbot_ws/spin_in_place.py mit --tff 0.10 --rate 100 --forward-sec 4 --pause-sec 2 --reverse-sec 4
```

参数说明：
- `mode`：选择 `speed` 或 `mit`
- `--wz`：速度模式下的角速度，单位 `rad/s`
- `--tff`：MIT 模式下的扭矩前馈，单位 `N*m`
- `--rate`：发布频率，单位 `Hz`，默认 `100`
- `--warmup-zero-sec`：正式动作前发零命令的时间，用于预热链路
- `--forward-sec`：正向旋转持续时间
- `--pause-sec`：中间停顿时间
- `--reverse-sec`：反向旋转持续时间
- `--stop-zero-sec`：结束后发零命令的时间，用于收尾停稳
- `--wait-timeout`：等待 `set_control_mode` 和模式状态确认的超时时间，单位秒

## 脚本工作方式

- `speed` 模式：
  - 自动切到 `speed`
  - 等待 `pc_speed + speed + ready=true`
  - 持续向 `/tianbot/cmd_vel` 发布原地旋转命令
- `mit` 模式：
  - 自动切到 `mit`
  - 等待 `pc_mit + mit + ready=true`
  - 持续向 `/tianbot/wheel_mit_cmd` 发布四轮反向扭矩命令

## 建议

- 第一次联调建议先用较小参数：
  - `speed`：`--wz 0.5`
  - `mit`：`--tff 0.05`
- 如果方向反了：
  - `speed` 模式把 `--wz` 改成负值
  - `mit` 模式把 `--tff` 改成负值
