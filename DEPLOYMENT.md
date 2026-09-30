# AGV2 驱动部署与实车启动

> develop 的故障处理、回退及测试要求见 [DEVELOPMENT.md](DEVELOPMENT.md)。
> 每台机器人分配独立 ROS_DOMAIN_ID，所有本车进程与调试终端使用相同值。

本仓库保存双舵轮底盘核心驱动及 2026-09-16 实车验证参数。支持 Ubuntu 22.04、
ROS 2 Humble、1 Mbps SocketCAN 和标准 Linux joystick。

## 新账户部署

```bash
mkdir -p ~/agv/agv_ws/src
git clone https://github.com/Franklin080864/agv_one_drive_program.git \
  ~/agv/agv_ws/src/agv2_pkg

cd ~/agv/agv_ws/src/agv2_pkg
bash deploy/install_dependencies_ubuntu22_humble.sh
bash deploy/build_humble.sh
```

仓库必须位于 `<workspace>/src/agv2_pkg`；构建结果写入工作区的
`install_release_humble`，不会提交到 Git。构建脚本随后运行运动学、CAN 编解码、
限速、遥控、轮毂 bring-up、控制安全和 FSM 共 7 组功能 gtest，并执行停机客户端测试；上游遗留的纯格式 lint 不作为
实车部署阻断条件。

## 启动

启动前确认：车辆处于空旷区域、物理急停按下、控制器上电、CAN 适配器和手柄
已连接、手柄全部回中。

本车必须显式使用仓库内的已验证外置配置：

```bash
cd ~/agv/agv_ws/src/agv2_pkg
sudo ./deploy/start_agv.sh --domain-id 11 "$PWD/deploy/chassis_full_manual_test.yaml"
```

查看运行状态：

```bash
tmux attach -t agv_control
```

启动联锁在收到手柄基线和后续模式按键上升沿之前保持被动。button 8 进入
Teleop，button 9 进入 Auto，button 7 软件锁车。手柄心跳超过 0.5 秒未更新会
进入 Idle/Locked。软件锁车不能替代物理急停。

当前主要映射：左摇杆 axis 1 前后、axis 0 偏航；X/button 3 左横移；
B/button 1 右横移；右摇杆 axis 2 备用自转；十字键 axis 7 备用直行。

## 直接 cmd_vel 验证

手柄保持连接并回中，短按 button 9 进入 Auto。确认没有其他 `/cmd_vel` 或
`/wheel_command` 发布者后，可执行有界直行：

```bash
source deploy/setup_agv_env.bash
export ROS_DOMAIN_ID=11  # 必须与本车启动参数一致
ros2 topic info /cmd_vel -v
ros2 topic info /wheel_command -v

ros2 topic pub -r 10 -t 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.20, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
ros2 topic pub -r 10 -t 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## 停机

先按下物理急停，再执行：

```bash
cd ~/agv/agv_ws/src/agv2_pkg
sudo ./deploy/stop_agv.sh --domain-id 11
```

停机脚本读取本实例的 Domain/CAN，调用 `/agv2/prepare_shutdown` 并检查响应
`success`，只有明确成功后才关闭 tmux 并将 CAN 接口置为 DOWN。拒绝、超时或
通信异常时保留现场。DDS 确认不等于实际制动确认；最终仍须确认车辆实际停止，
并看到 `state DOWN` 和 `can state STOPPED`。

## 更新

```bash
cd ~/agv/agv_ws/src/agv2_pkg
git pull --ff-only
bash deploy/build_humble.sh
```

更新后仍使用同一条显式外置配置启动命令。
