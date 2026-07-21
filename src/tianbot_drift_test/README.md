# tianbot_drift_test

`tianbot_drift_test` 是 Tianbot DYN 无转向四轮独立驱动漂移测试包，当前只保留
`drift_4wid_independent_speed_yaw_pid.m` 对应的 ROS 2 硬件控制器和 rosbag
离线分析工具。

控制器使用滤波后的 IMU 横摆角速度和里程计速度进行 PID 控制，并通过横纵向
加速度估计侧偏角，修正横摆角速度参考值。MATLAB 离线求解的平衡扭矩、速度
分配权重和 yaw 分配向量保存在配置文件中。

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

然后启动四轮独立速度/yaw PID：

```bash
ros2 launch tianbot_drift_test drift_4wid_speed_yaw_pid.launch.py
```

launch 默认会执行以下流程：

1. 请求 `/tianbot/set_control_mode` 切换到 MIT 模式。
2. 等待 `pc_mit + mit + ready=true`、四个电机在线以及 IMU/里程计数据。
3. 安全倒计时 3 秒。
4. 静止标定陀螺仪及横纵向加速度计零偏。
5. 执行加速、起漂脉冲、闭环保持和减扭停车。

手动中止：

```bash
ros2 service call /drift_4wid_speed_yaw_pid/abort std_srvs/srv/Trigger "{}"
```

若希望手动发起：

```bash
ros2 launch tianbot_drift_test drift_4wid_speed_yaw_pid.launch.py \
  auto_start:=false activate_mit:=false
ros2 service call /drift_4wid_speed_yaw_pid/start std_srvs/srv/Trigger "{}"
```

## 配置与日志

- `config/drift_4wid_speed_yaw_pid.yaml`：目标、滤波、PID、侧偏角估计、四轮扭矩
  分配及安全参数。
- CSV 默认写入 `~/.ros/drift_test_logs`。
- 诊断话题为 `/drift_4wid_speed_yaw_pid/diagnostics`。
- 四轮顺序固定为 `[FL, FR, RL, RR]`。

日志和诊断数据区分 `raw`/`filtered` 字段，并记录速度/yaw 误差、积分量、修正量、
加速度标定值、侧偏角估计以及四轮限幅前、限幅后和实际发布的扭矩。

## Rosbag 分析

分析工具位于工作区顶层 `tools/`，只依赖 Python 标准库，可以直接使用普通
Python 3：

```bash
python3 tools/analyze_drift_rosbag.py \
  drift_test_bag \
  --baseline-yaml src/tianbot_drift_test/config/drift_4wid_speed_yaw_pid.yaml
```

默认输出到 rosbag 目录下的 `analysis/`，包括 CSV、SVG、`summary.md`、
`summary.json` 和 `report.html`。

## 安全说明

MATLAB 文件是仿真控制器。新版平衡前馈最高约为 `0.615 Nm/轮`，加速阶段命令
最高可达到 `0.68 Nm/轮`，尚不代表已经通过真实车辆验证。首次落地测试应使用
封闭场地，提前准备好 abort 命令，并确认
IMU/加速度计方向、四轮顺序和电机扭矩符号。不要同时运行其他向
`/tianbot/wheel_mit_cmd` 发布命令的节点。
