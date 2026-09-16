# agv2_pkg

> 实车部署入口见 [DEPLOYMENT.md](DEPLOYMENT.md)。部署脚本强制使用显式外置
> 参数文件，避免账户路径变化或安装空间中的旧配置影响实车行为。

双舵轮底盘单进程驱动。它把原 `agv_pkg` 指令路径和 `steering_adapter`
中间进程合到一个 ROS 2 节点里，支持连续 `cmd_vel`、逐轮直控
`wheel_command` 和遥控器 Teleop。

## TODO

- CAN 协议层已独立封装在 `can_codec`，转向、轮毂、SDO 初始化、SOC 和
  Ctrl+C 终止帧都有单元测试覆盖。
- 运动学为真正的双舵轮 IK：`cmd_vel` 使用 `linear.x`、`linear.y`、
  `angular.z`，每个舵轮按自身位置计算目标舵角和轮速。
- 轮速有软件斜坡限制；舵角不做软件斜坡，直接下发目标角，实际转向速度由
  转向电机 `0xa4` 帧里的速度字段控制。
- 当前最大风险不是 CAN codec，而是实车几何参数和上游话题迁移：
  旧的 `steering_commands` 不再被订阅，上游需要改发 `/cmd_vel` 或
  `/wheel_command`，或者补桥接节点。
- `config/chassis.yaml` 里的 `wheel_spd_up_button` 和
  `wheel_spd_down_button` 是遗留配置；当前节点没有读取它们，Y/A 加减速档未实现。

## 构建与运行

```bash
colcon build --packages-select agv2_pkg
source install/setup.bash
ros2 launch agv2_pkg agv2_control.launch.py
```

Launch 默认会 `respawn=True`，节点崩溃后 1 秒自动重启。部署时有用，调参时如果
参数错误会反复重启。

### 安全停止

启动联锁释放后，停止启动脚本或节点前应先调用：

```bash
ros2 service call /agv2/prepare_shutdown std_srvs/srv/Trigger '{}'
```

服务立即将模式切换到 Idle/Locked、把内部轮速状态清零，并在 ROS context 和
CAN sender 仍有效时依次发布两路零轮速帧与 `0x001` 终止帧。发布后控制定时器
停止产生新的 CAN 指令，随后才可以向控制节点发送 SIGINT/SIGTERM。直接向节点
发送信号仍会尝试发送终止帧，但部署脚本应优先使用上述显式握手，并以物理 CAN
捕获确认最终帧确实到达总线。

Launch 默认还会启动被动故障日志。它只订阅 ROS 话题并维护内存环形缓存，
不会锁车、不会发布 `/to_can_bus`，也不会改变底盘控制。日志默认写到
`~/agv/agv_ws/data/chassis_faults/chassis_fault_<启动时间>/`。

发现底盘异常后，手动保存最近的日志：

```bash
ros2 service call /agv2/save_fault_log std_srvs/srv/Trigger "{}"
```

保存成功后 recorder 会继续记录下一次故障。不要通过杀死驱动节点、关闭 CAN
接口或关闭 tmux 来保存日志；snapshot 尚未触发时，内存中的历史不会落盘。

离线生成电机定位报告：

```bash
ros2 run agv2_pkg analyze_chassis_bag \
  ~/agv/agv_ws/data/chassis_faults/chassis_fault_YYYYMMDD_HHMMSS
```

报告生成在 bag 目录下的 `analysis/`，包含 `summary.md`、`telemetry.csv` 和
`motor_timeline.png`。如需关闭故障记录或扩大环形缓存：

```bash
ros2 launch agv2_pkg agv2_control.launch.py enable_fault_logging:=false
ros2 launch agv2_pkg agv2_control.launch.py fault_cache_size:=268435456
```

底盘1使用默认参数文件 `config/chassis.yaml`。底盘2如果出现“直行、纯转正常，
但 `/cmd_vel` 弧线和左摇杆斜向左右反”的现象，优先使用底盘2参数文件：

```bash
ros2 launch agv2_pkg agv2_control.launch.py \
  params_file:="$PWD/config/chassis_robot2.yaml"
```

部署启动脚本要求显式传入配置；底盘2可以这样启动：

```bash
sudo ./deploy/start_agv.sh "$PWD/config/chassis_robot2.yaml"
```

## ROS 接口

**订阅**

| 话题 | 类型 | 说明 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Auto 主路径，使用 `linear.x/y` 和 `angular.z` |
| `/wheel_command` | `agv2_pkg/WheelCommand` | Auto 逐轮直控，绕过 IK |
| `/joy` | `sensor_msgs/Joy` | 遥控器输入 |
| `/from_can_bus` | `can_msgs/Frame` | 底盘 CAN 反馈 |

**发布**

| 话题 | 类型 | 说明 |
|---|---|---|
| `/to_can_bus` | `can_msgs/Frame` | 转向、轮毂、初始化和终止 CAN 帧 |
| `/Battery_SOC_STATE` | `std_msgs/Float64` | 电池 SOC，来自 CAN ID `0x191` 的 byte 6 |
| `/agv2/chassis_telemetry` | `agv2_pkg/ChassisTelemetry` | 四电机命令、反馈、CAN ID 和被动诊断标志 |
| `/agv2/log_event` | `std_msgs/String` | 人工故障快照时间标记 |

**服务**

| 名称 | 类型 | 说明 |
|---|---|---|
| `/agv2/lock` | `std_srvs/Trigger` | 强制进入 Idle / Locked |
| `/agv2/save_fault_log` | `std_srvs/Trigger` | 仅保存环形日志，不影响底盘控制 |

**自定义消息** [`msg/WheelCommand.msg`](msg/WheelCommand.msg)

```text
float64 front_steer_angle   # rad, front steering wheel target angle
float64 rear_steer_angle    # rad, rear steering wheel target angle
float64 front_wheel_speed   # m/s, front wheel linear speed (+ = body forward)
float64 rear_wheel_speed    # m/s, rear wheel linear speed (+ = body forward)
```

## 控制流

```text
joy ─────────┐
cmd_vel ─────┤── arbitrator ── kinematics ── rate_limiter ── fsm ── can_codec ── /to_can_bus
wheel_command┘
```

模式由遥控器按键切换：

| 模式 | 进入方式 | 生效输入 | 行为 |
|---|---|---|---|
| Idle | 默认、急停、心跳超时、`/agv2/lock` | 无 | 轮速归零并锁车 |
| Teleop | `enable_button`，默认 button 8 | `/joy` | 手柄控制底盘 |
| Auto | `ad_enable_button`，默认 button 9 | `/cmd_vel` 或 `/wheel_command` | 自动控制 |

启动联锁默认保持有效：节点启动后只发布转向 `0x94` 位置查询，不发送 `0x88`
使能、`0xA4` 位置目标、轮毂 SDO/NMT、控制字或速度帧。必须先收到一帧按键
基线，再收到 Teleop/Auto 按键上升沿，并取得前后转向位置反馈、确认真实
`socket_can_sender` 已连接，才会开始硬件 bring-up。按键在节点启动前已经按住
不会解锁；需先释放再重新按下。VR Auto 桥会在 Auto 上升沿前自动发送一帧中性
`/joy` 基线。

Auto 模式下 `/cmd_vel` 和 `/wheel_command` 是会话式互斥：先到的源持有控制权，
直到它静默超过 `arbitrator.input_timeout_ms` 后，另一路才可接管。

输入新鲜度分两层：

- `fsm.cmd_watchdog_ms` 默认 200 ms：当前有效输入超过这个时间没更新，轮速开始斜坡降到 0。
- `arbitrator.input_timeout_ms` 默认 1000 ms：控制源超过这个时间没更新，才释放会话控制权。

## 重要参数

参数文件在 [`config/chassis.yaml`](config/chassis.yaml)，坐标系遵循 ROS
`base_link`：`x` 向前，`y` 向左，`z` 向上。

| 参数 | 当前值 | 单位 | 作用 |
|---|---:|---|---|
| `chassis.front_wheel_x` | `0.155` | m | 前舵轮相对 `base_link` 的 x 坐标 |
| `chassis.front_wheel_y` | `-0.169` | m | 前舵轮 y 坐标，负数表示在车体右侧 |
| `chassis.rear_wheel_x` | `-0.155` | m | 后舵轮 x 坐标 |
| `chassis.rear_wheel_y` | `0.169` | m | 后舵轮 y 坐标，正数表示在车体左侧 |
| `chassis.wheel_radius` | `0.07` | m | 轮速 `m/s` 与 CAN `RPM` 换算，必须实测 |
| `chassis.front_motor_direction` | `-1` | - | 前轮毂电机安装方向，前进反向时用 `-1` |
| `chassis.rear_motor_direction` | `1` | - | 后轮毂电机安装方向 |
| `chassis.front_steer_direction` | `-1` | - | 前转向电机角度符号修正 |
| `chassis.rear_steer_direction` | `-1` | - | 后转向电机角度符号修正 |
| `can.steer_front_id` | `0x142` | - | 物理前转向电机 CAN ID |
| `can.steer_rear_id` | `0x141` | - | 物理后转向电机 CAN ID |
| `can.wheel_front_id` | `0x201` | - | 物理前轮毂速度命令 CAN ID |
| `can.wheel_rear_id` | `0x202` | - | 物理后轮毂速度命令 CAN ID |
| `can.wheel_front_status_id` | `0x181` | - | 物理前轮毂状态反馈 CAN ID |
| `can.wheel_rear_status_id` | `0x182` | - | 物理后轮毂状态反馈 CAN ID |
| `can.sdo_front_id` / `can.front_node_id` | `0x601` / `1` | - | 物理前轮毂 SDO 初始化 |
| `can.sdo_rear_id` / `can.rear_node_id` | `0x602` / `2` | - | 物理后轮毂 SDO 初始化 |
| `wheel_bringup.wait_for_can_sender` | `true` | - | 等 `/to_can_bus` 有 sender 订阅后再发一次性启动帧 |
| `wheel_bringup.can_sender_node_name` | `socket_can_sender` | - | 只把真实 SocketCAN sender 视为就绪，忽略 rosbag 订阅 |
| `wheel_bringup.sdo_retry_period_ms` | `1000` | ms | 未收到轮毂状态前，SDO/NMT 初始化重试周期 |
| `wheel_bringup.status_stale_ms` | `500` | ms | 状态反馈多久未更新视为 stale，用于日志诊断 |
| `wheel_bringup.warn_period_ms` | `2000` | ms | 轮毂 bring-up 未完成时的日志节流周期 |
| `limits.max_linear_speed` | `0.5` | m/s | 限制 `cmd_vel`、`wheel_command` 和最终逐轮速度 |
| `limits.max_angular_speed` | `0.5` | rad/s | 限制 `cmd_vel.angular.z` |
| `limits.max_steer_angle` | `1.5708` | rad | 舵角硬限制，默认 +/- 90 deg |
| `limits.max_wheel_rpm` | `200.0` | RPM | CAN 编码前的轮毂转速上限 |
| `limits.singularity_speed` | `0.001` | m/s | 低于该轮速时保持上次舵角，避免原地抖角 |
| `rate_limit.loop_hz` | `20.0` | Hz | 主控制循环频率 |
| `rate_limit.max_d_speed_per_step` | `0.05` | m/s/帧 | 轮速每帧最大变化，当前约等于 1.0 m/s² |
| `fsm.cmd_watchdog_ms` | `200` | ms | 输入静默多久进入 RampDown |
| `fsm.lock_settle_ms` | `100` | ms | 轮速接近 0 后保持多久进入 Locked |
| `fsm.stop_speed_eps` | `0.005` | m/s | 判定轮子已经停下的速度阈值 |
| `arbitrator.input_timeout_ms` | `1000` | ms | Auto 输入源释放控制权的静默时间 |

### 遥控器参数

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `joystick.enable_button` | `8` | 进入 Teleop，要求 axes 0-3 全部回中 |
| `joystick.ad_enable_button` | `9` | 进入 Auto |
| `joystick.emergency_button` | `7` | 急停，进入 Idle / Locked |
| `joystick.left_linear_axis` | `1` | 左摇杆上下，映射为 `vx` |
| `joystick.left_angular_axis` | `0` | 左摇杆左右，映射为 `omega` |
| `joystick.left_linear_sign` | `1.0` | 左摇杆前后方向反了就改为 `-1.0` |
| `joystick.left_angular_sign` | `1.0` | 左摇杆转向方向反了就改为 `-1.0` |
| `joystick.speed_axis` | `7` | 十字键上/下，备用前进/后退 |
| `joystick.angular_axis` | `2` | 右摇杆左右，备用原地自转 |
| `joystick.angular_left_button` | `3` | X 键，横移左 |
| `joystick.angular_right_button` | `1` | B 键，横移右 |
| `joystick.heartbeat_timeout_s` | `0.5` | Teleop/Auto 中手柄心跳丢失多久锁车 |
| `joystick.teleop_max_speed` | `0.3` | m/s，Teleop 平移速度上限 |
| `joystick.teleop_rotate_omega` | `0.87` | rad/s，Teleop 自转角速度 |
| `joystick.teleop_steer_lock` | `1.5708` | rad，横移时锁到 +/- 90 deg |
| `joystick.teleop_deadband` | `0.1` | 摇杆死区 |

## 遥控器按键和轴

| 操作 | 默认映射 | 代码行为 |
|---|---|---|
| 进入 Teleop | button 8 | 只有 axes 0-3 都回中时才接受 |
| 进入 Auto | button 9 | `/cmd_vel` 和 `/wheel_command` 开始生效 |
| 急停/锁车 | button 7 | 立即进入 Idle / Locked |
| 左摇杆上下 | axis 1 | `vx = axis * left_linear_sign * teleop_max_speed` |
| 左摇杆左右 | axis 0 | `omega = axis * left_angular_sign * teleop_rotate_omega` |
| X 键 | button 3 | 逻辑舵角 `+teleop_steer_lock`，车体向左横移 |
| B 键 | button 1 | 逻辑舵角 `-teleop_steer_lock`，车体向右横移 |
| 右摇杆左右 | axis 2 | 左摇杆回中时备用原地自转，走双舵轮 IK |
| 十字键上/下 | axis 7 | 左摇杆和右摇杆都回中时备用前进/后退 |
| Y 键 | button 4 | 当前未实现加速档，配置项无效 |
| A 键 | button 0 | 当前未实现减速档，配置项无效 |

Teleop 优先级从高到低：

1. X/B 横移按钮。
2. 左摇杆连续控制，`vx + omega` 一起进入 IK。
3. 右摇杆左右备用原地自转。
4. 十字键上/下备用直行。
5. 全部回中则目标轮速为 0。

## CAN 约定

| 用途 | CAN ID |
|---|---|
| 后转向电机命令/反馈 | `0x141` |
| 前转向电机命令/反馈 | `0x142` |
| 前轮毂速度命令 | `0x201` |
| 后轮毂速度命令 | `0x202` |
| 前轮毂状态反馈 | `0x181` |
| 后轮毂状态反馈 | `0x182` |
| 电池 SOC | `0x191` |
| 前轮毂 SDO 初始化 | `0x601` |
| 后轮毂 SDO 初始化 | `0x602` |
| NMT broadcast | `0x000` |
| Ctrl+C 终止帧 | `0x001` |

启动时先保持上述联锁；联锁解除且前后 `0x94` 反馈有效后，才会发送转向
`0x88` 使能，并在下一控制周期开始发布 `0xA4`。实车确认 `0x94` 是会在
0°/360° 回绕的单圈反馈：反馈先归一化到 `[-180°, 180°]` 用于诊断，`0xA4`
目标始终保持在底盘标定零点附近的 `±90°` 本地工作圈内，不能根据 `0x94`
在 0° 与 360° 多圈目标之间切换。
`can.steer_speed_field` 直接配置 `0xA4` 的第 2-3 字节。2026-09-16 完成
Teleop 和直接 `/cmd_vel` 地面验证后恢复为旧 `agv2_pkg` 的值 `300`；在没有厂商
单位定义确认前，只按线上的相对值解释，不把它标注为物理角速度。
轮毂采用可重试 CANopen bring-up：未收到对应 `0x181` / `0x182` 状态反馈前，会按
`wheel_bringup.sdo_retry_period_ms` 重发 SDO/NMT 初始化；收到状态后停止重发 SDO，
并按状态位推进 `0x06 -> 0x07 -> 0x0f`。只有两个轮毂都反馈 enabled 后才发送速度帧。

如果某台底盘“纯直行、纯左右转正常，但弧线左右反”，通常不是遥控器轴号问题。
左摇杆斜向和 Auto `/cmd_vel` 弧线都走同一套 IK，弧线时前/后轮命令不同，因此会暴露
物理前后模块与 CAN ID 对应关系不一致的问题。优先用 `chassis_robot2.yaml` 验证；
若只互换了舵机或只互换了轮毂，只调整对应的 `can.*` 参数即可。
