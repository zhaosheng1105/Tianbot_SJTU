# DYN 车上使用说明

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

## 自旋测试脚本

脚本位置：

```bash
~/tianbot_ws/spin_in_place.sh
```

速度模式示例：

```bash
~/tianbot_ws/spin_in_place.sh speed 3 2 3
```

含义：
- 先按正方向原地转 `3` 秒
- 停 `2` 秒
- 再反方向原地转 `3` 秒

MIT 模式示例：

```bash
~/tianbot_ws/spin_in_place.sh mit 2 2 2
```

默认参数：
- `speed` 模式使用 `angular.z=0.6 rad/s`
- `mit` 模式使用 `t_ff=0.10 N*m`
- 发布频率 `20 Hz`

可选环境变量：
- `SPIN_SPEED_WZ`：速度模式角速度，默认 `0.6`
- `SPIN_MIT_TFF`：MIT 模式扭矩，默认 `0.10`
- `SPIN_PUB_RATE`：发布频率，默认 `20`

示例：

```bash
SPIN_SPEED_WZ=0.8 ~/tianbot_ws/spin_in_place.sh speed 2 1 2
SPIN_MIT_TFF=0.15 ~/tianbot_ws/spin_in_place.sh mit 2 1 2
```

说明：
- `speed` 模式通过 `/tianbot/cmd_vel` 控制原地旋转
- `mit` 模式通过 `/tianbot/wheel_mit_cmd` 控制四轮反向扭矩实现原地旋转
- 如果旋转方向和预期相反，改小/改负参数即可
