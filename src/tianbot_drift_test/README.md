# tianbot_drift_test

`tianbot_drift_test` 是 Tianbot DYN 无转向四轮独立驱动漂移测试包。包内包含
`drift_imu_only_pid.m`、`drift_imu_odom_pid.m` 和四轮独立分配
`drift_4wid_independent_speed_yaw_pid.m` 的 ROS 2 硬件控制版本，并保留原来的
低速 `no_steer_circle_drift_test` 联调节点。

## 三套 MATLAB 控制器

| ROS 2 launch | MATLAB 来源 | 闭环反馈 |
| --- | --- | --- |
| `drift_imu_only_pid.launch.py` | `drift_imu_only_pid.m` | 仅 IMU 横摆角速度 PD；纵向扭矩按时间调度 |
| `drift_imu_odom_pid.launch.py` | `drift_imu_odom_pid.m` | IMU 横摆 PD、里程计纵向速度 P、里程计半径 PD 外环 |
| `drift_4wid_speed_yaw_pid.launch.py` | `drift_4wid_independent_speed_yaw_pid.m` | 原始 IMU yaw rate PID、原始 odom 速度 PID、四轮独立扭矩分配 |

三套控制器均保留 MATLAB 中的目标值、阶段时长、启动脉冲、平衡点前馈、
反馈增益、扭矩上限和扭矩变化率。控制状态依次为：静止 IMU 标定、加速、
起漂脉冲、闭环保持、减扭停车。

IMU-only 节点不订阅里程计，里程计不会参与其就绪条件、控制或阶段切换。
IMU+odom 节点使用 `Odometry.twist.twist.linear.x` 作为 MATLAB 中的纵向速度，
并在保持阶段根据里程计位姿锁定圆心和计算半径。

四轮独立节点的目标为 `R=1 m、V=2 m/s、CCW、3 圈`，使用 MATLAB 在
`mu=0.75` 下离线求解的四轮独立平衡扭矩、速度分配权重和 yaw 分配向量。
按实车测试要求，它不使用 IMU、yaw 导数、odom 速度或速度导数低通滤波：
IMU 只减去启动时的静止零偏，两个 D 项均由相邻原始传感器样本直接差分。
`command_slew_nmps` 是 MATLAB 中的扭矩命令变化率约束，不是传感器滤波。

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

然后选择一个控制器启动。不要同时启动多个控制 launch。

仅 IMU：

```bash
ros2 launch tianbot_drift_test drift_imu_only_pid.launch.py
```

IMU + 里程计：

```bash
ros2 launch tianbot_drift_test drift_imu_odom_pid.launch.py
```

四轮独立速度/yaw PID：

```bash
ros2 launch tianbot_drift_test drift_4wid_speed_yaw_pid.launch.py
```

每个 launch 默认都会自动完成以下操作：

1. 请求 `/tianbot/set_control_mode` 切换到 MIT 模式。
2. 等待 `pc_mit + mit + ready=true`、四个电机在线以及所需传感器数据。
3. 安全倒计时 3 秒。
4. 静止采集 IMU 零偏，随后自动执行一次测试。
5. 达到 3 圈后按各 MATLAB 配置的减扭曲线停车，并持续发布零命令。

## IMU 与车轮方向

- IMU 使用已确认的右手坐标系：逆时针 `angular_velocity.z > 0`，顺时针为负，
  所以三个配置中的 `imu_yaw_sign` 均为 `1.0`。
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

四轮独立速度/yaw PID：

```bash
ros2 service call /drift_4wid_speed_yaw_pid/abort std_srvs/srv/Trigger "{}"
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
- `config/drift_4wid_speed_yaw_pid.yaml`：四轮独立速度/yaw PID 参数和离线平衡点。
- CSV 默认写入 `~/.ros/drift_test_logs`。原有两个节点记录其滤波状态，四轮独立
  节点记录未经低通滤波的反馈、参考值、积分量和四轮扭矩。
- 诊断话题分别为 `/drift_imu_only_pid/diagnostics` 和
  `/drift_imu_odom_pid/diagnostics`。

四轮独立节点的日志和 `/drift_4wid_speed_yaw_pid/diagnostics` 明确使用
`unfiltered`/`raw` 字段名，另外记录速度/yaw 的误差、积分量、修正量及四轮
限幅前、限幅后和实际发布的扭矩。

## 安全说明

MATLAB 文件是仿真控制器。四轮独立控制的默认起漂脉冲最高为 `0.65 Nm/轮`，
尚不等于真实车辆已经验证。首次落地测试应使用封闭场地、准备好上述 `abort`
命令，并根据实际轮胎、地面和车辆质量逐步降低或调整配置。不要同时运行其他
会向 `/tianbot/wheel_mit_cmd` 发布命令的节点。

原低速联调节点仍可通过以下命令运行：

```bash
ros2 launch tianbot_drift_test no_steer_circle_drift_test.launch.py
```

## Rosbag 分析工具

包内提供了不依赖 `rosbag2_py` 的 SQLite3/CDR 分析工具，可直接解析本测试记录的
里程计、IMU、诊断量、MIT 四轮命令和四轮电机反馈：

```bash
ros2 run tianbot_drift_test analyze_drift_rosbag ~/tianbot_ws/drift_test_bag \
  --baseline-yaml ~/tianbot_ws/src/tianbot_drift_test/config/drift_imu_only_pid.yaml
```

也可以直接运行源码：

```bash
python3 src/tianbot_drift_test/tianbot_drift_test/analyze_drift_rosbag.py \
  drift_test_bag \
  --baseline-yaml src/tianbot_drift_test/config/drift_imu_only_pid.yaml
```

默认输出到 rosbag 目录下的 `analysis/`，包含参数差异、统计摘要、可浏览的
`report.html`、各话题 CSV，以及 odom、yaw rate、速度、四轮命令/反馈扭矩和
四轮转速 SVG 图。
