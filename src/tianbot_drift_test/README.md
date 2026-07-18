# tianbot_drift_test

`tianbot_drift_test` 是 Tianbot DYN 无转向四轮独立驱动漂移测试包。包内包含
`drift_imu_only_pid.m` 和 `drift_imu_odom_pid.m` 的 ROS 2 硬件控制版本，并保留
原来的低速 `no_steer_circle_drift_test` 联调节点。

## 两套 MATLAB 控制器

| ROS 2 launch | MATLAB 来源 | 闭环反馈 |
| --- | --- | --- |
| `drift_imu_only_pid.launch.py` | `drift_imu_only_pid.m` | 仅 IMU 横摆角速度 PD；纵向扭矩按时间调度 |
| `drift_imu_odom_pid.launch.py` | `drift_imu_odom_pid.m` | IMU 横摆 PD、里程计纵向速度 P、里程计半径 PD 外环 |

两套控制器均保留 MATLAB 中的目标值、阶段时长、启动脉冲、平衡点前馈、
反馈增益、扭矩上限和扭矩变化率。控制状态依次为：静止 IMU 标定、加速、
起漂脉冲、闭环保持、减扭停车。

IMU-only 节点不订阅里程计，里程计不会参与其就绪条件、控制或阶段切换。
IMU+odom 节点使用 `Odometry.twist.twist.linear.x` 作为 MATLAB 中的纵向速度，
并在保持阶段根据里程计位姿锁定圆心和计算半径。

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

然后二选一启动控制器。不要同时启动两个控制 launch。

仅 IMU：

```bash
ros2 launch tianbot_drift_test drift_imu_only_pid.launch.py
```

IMU + 里程计：

```bash
ros2 launch tianbot_drift_test drift_imu_odom_pid.launch.py
```

每个 launch 默认都会自动完成以下操作：

1. 请求 `/tianbot/set_control_mode` 切换到 MIT 模式。
2. 等待 `pc_mit + mit + ready=true`、四个电机在线以及所需传感器数据。
3. 安全倒计时 3 秒。
4. 静止采集 IMU 零偏，随后自动执行一次测试。
5. 达到 3 圈后按 MATLAB 的 0.45 秒减扭曲线停车，并持续发布零命令。

## IMU 与车轮方向

- IMU 使用已确认的右手坐标系：逆时针 `angular_velocity.z > 0`，顺时针为负，
  所以两个配置中的 `imu_yaw_sign` 均为 `1.0`。
- MIT 消息接口、四轮数组顺序和模式确认流程沿用已成功测试的
  `spin_in_place.py`：控制器顺序为 `[FL, FR, RL, RR]`。
- 默认 `motor_torque_signs` 为 `[1.0, 1.0, 1.0, 1.0]`，即把 MATLAB 的四轮
  逻辑扭矩直接交给当前 DYN 接口。只有实际硬件映射发生变化时才修改。
- 起漂或保持阶段若 IMU 连续检测到与目标相反的明显横摆，节点会立即中止并
  发布四轮零扭矩。

## 随时中止

仅 IMU：

```bash
ros2 service call /drift_imu_only_pid/abort std_srvs/srv/Trigger "{}"
```

IMU + 里程计：

```bash
ros2 service call /drift_imu_odom_pid/abort std_srvs/srv/Trigger "{}"
```

若希望手动发起，可关闭自动启动和自动切换 MIT：

```bash
ros2 service call /tianbot/set_control_mode tianbot_core/srv/SetControlMode \
  "{mode: 'mit', save_to_flash: false}"
ros2 launch tianbot_drift_test drift_imu_only_pid.launch.py \
  auto_start:=false activate_mit:=false
ros2 service call /drift_imu_only_pid/start std_srvs/srv/Trigger "{}"
```

## 配置与日志

- `config/drift_imu_only_pid.yaml`：IMU-only MATLAB 参数。
- `config/drift_imu_odom_pid.yaml`：IMU+odom MATLAB 参数。
- CSV 默认写入 `~/.ros/drift_test_logs`，包含原始 IMU、零偏、滤波横摆角速度、
  参考值、积分圈数、里程计量、逻辑四轮扭矩和最终发布的四轮扭矩。
- 诊断话题分别为 `/drift_imu_only_pid/diagnostics` 和
  `/drift_imu_odom_pid/diagnostics`。

## 安全说明

MATLAB 文件是仿真控制器，默认配置会使用最高 `0.48 Nm/轮` 的起漂扭矩，尚不
等于真实车辆已经验证。首次落地测试应使用封闭场地、准备好上述 `abort` 命令，
并根据实际轮胎、地面和车辆质量逐步降低或调整配置。不要同时运行其他会向
`/tianbot/wheel_mit_cmd` 发布命令的节点。

原低速联调节点仍可通过以下命令运行：

```bash
ros2 launch tianbot_drift_test no_steer_circle_drift_test.launch.py
```
