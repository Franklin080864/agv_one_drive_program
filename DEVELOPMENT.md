# develop 稳定性改进与实车验收

基线：`main@0f562453dcce7f6631b02d89f0316eced2f4fd44`。本分支用于下一轮实车验证，不能把软件回归测试视为硬件停车验证。

## 设计范围

保留原有双舵轮 IK、CAN 有效命令字节格式、前后轮映射、几何/轮径、速度限制、20 Hz 周期及已有消息定义。每车采用独立 `ROS_DOMAIN_ID`；本分支不将原始 CAN 话题桥接到车队网络。

默认启用的改进针对异常路径：

- 获选输入来源独立看门狗：200 ms 运动有效期与 1000 ms 控制权有效期分开，备用来源不能给旧指令续期。输入队列保留最近一条。
- 软件急停按下电平始终优先；按住期间的使能边沿被消耗，释放后需要重新按使能。
- 拒绝非有限输入；非法数据不进入速度积分。输入故障锁存后，显式复位会丢弃该输入会话；持续非法输入会再次被拒绝。
- 首次硬件就绪后监测两轮、两舵反馈及 CAN sender；超时、失能、驱动 fault/error_code 触发锁存停止。正常轮会继续收到明确的零速。
- 无有效输出许可时清零内部速度斜坡，初次就绪或人工恢复后从零重新爬升。
- 新增标准 `diagnostic_msgs/DiagnosticArray` 错误反馈和人工复位接口。
- 停机客户端解析 `Trigger.success`；DDS 确认失败不拆掉 CAN/进程。
- 初次 bring-up 期间可以对无反馈/陈旧反馈限频重试；运行中的故障默认先锁存，恢复需要人工复位与新的使能。不会自动发送电机 fault-reset。

以下运动行为默认不改，逐项实车验证后再开启：

| 参数 | 默认 | 作用 |
|---|---:|---|
| `safety.steer_alignment_enabled` | false | 目标与实际舵角偏差超过阈值时暂时停止行走，仍发送舵角目标 |
| `safety.steer_alignment_max_error_deg` | 15.0 | 上述轮速许可阈值，单位为角度 |
| `safety.steer_error_stop_enabled` | false | 舵角偏差持续达到 `diagnostics.steer_mismatch_deg` 后锁存故障 |
| `safety.steer_error_stop_ms` | 1000 | 舵角持续偏差的确认时间 |
| `safety.hold_steering_on_teleop_stop` | false | 手柄回中时保持当前命令舵角，避免带速回正 |
| `safety.feedback_stop_enabled` | true | 运行中反馈超时锁存；即使设为 false，陈旧反馈也不能放行轮速，但恢复语义变为非锁存，因此实车建议保持 true |

不在这一轮擅自改动：±90° IK 分支选择、逐轮速度比例限制、实际时间步长限速、完整 SDO 事务管理、硬件协议的未知单位。上述内容需要独立标定或实车比较后继续开发。

## 故障状态与恢复

服务与话题均随控制节点的 namespace 变化，默认名称如下：

- `/agv2/chassis_telemetry`：原消息格式保持不变。
- `/agv2/diagnostics`：新增控制状态、当前故障、锁存故障、反馈年龄、重试次数及 Domain ID。
- `/agv2/lock`：软件零速锁车，保持 Idle。
- `/agv2/reset_faults`：只清主机故障锁存与旧输入会话；不替代电机故障复位。
- `/agv2/reinitialize_transport`：人工请求通信恢复；只发送零目标 shutdown 控制字和限频 SDO/NMT，不发送使能序列。
- `/agv2/prepare_shutdown`：发送最终停车帧并等待 DDS 层确认。

`active_faults` 与 `latched_faults` 是十进制位掩码，`DiagnosticStatus.message` 同时提供可读名称：

| 位值 | 名称 |
|---:|---|
| 1 | can_sender_missing |
| 2 / 4 | front_wheel_timeout / rear_wheel_timeout |
| 8 / 16 | front_steer_timeout / rear_steer_timeout |
| 32 / 64 | front_wheel_fault / rear_wheel_fault |
| 128 | steering_mismatch |
| 256 | invalid_input |
| 512 / 1024 | front_wheel_disabled / rear_wheel_disabled |
| 2048 | transport_recovery |

`stop_reason` 表示当前停止或等待原因。`diagnostics.publish_period_ms` 默认 200 ms；`diagnostics.hardware_id` 默认 `agv`，可以填本车资产编号。

恢复顺序：

1. 停车并保持物理急停，读取诊断和电机故障；修复通信/驱动/输入问题。
2. 确认 CAN sender 在线、四路反馈新鲜、无驱动错误，软件急停按键已释放；非法命令来源已停止或修正。非 enabled 状态允许复位，以便下一次使能重新 bring-up。
3. 在本车 Domain 下调用 `/agv2/reset_faults`，检查响应 `success=true`。
4. 此时仍为 Idle，启动联锁重新生效；持续收到旧速度流也不会启动。
5. 按新的 Teleop/Auto 使能边沿，重新完成启动检查及速度爬升。

```bash
export ROS_DOMAIN_ID=11  # 示例；必须等于本车启动时的值
source deploy/setup_agv_env.bash
ros2 topic echo /agv2/diagnostics
# 另一个终端，在确认上述恢复条件后执行：
ros2 service call /agv2/reset_faults std_srvs/srv/Trigger '{}'
```

若轮毂重启后必须重新配置 PDO 才恢复反馈，可在锁车且 sender 已恢复的条件下，人工调用 `/agv2/reinitialize_transport`。该请求最多尝试 5 秒，诊断 `transport_recovery` 显示结果；通信恢复后仍须调用 reset_faults 并重新使能，超时保持锁存。它不清除电机自身故障。

刚启动尚未收到反馈时保持等待，不直接将正常 bring-up 等待判为运行故障。首次两轮就绪后才启用运行监测。`Locked` 与内部轮速为零均不证明实际轮子停止；当前没有硬件制动状态确认。

## 部署与回退

先按原部署流程安装依赖；新增 `diagnostic_msgs` 由 rosdep 安装。原来的已验证参数文件可以继续使用；缺少新增参数时使用以上默认值。不要把 robot1 的硬件映射覆盖到 robot2。

```bash
cd ~/agv/agv_ws/src/agv2_pkg
git fetch origin
git switch develop
git pull --ff-only
bash deploy/install_dependencies_ubuntu22_humble.sh
bash deploy/build_humble.sh

# 物理急停保持按下；11 为示例 Domain，第二台使用另一值。
sudo ./deploy/start_agv.sh --domain-id 11 "$PWD/deploy/chassis_full_manual_test.yaml"

# 停机前先按物理急停；脚本校验 Domain 和实例记录。
sudo ./deploy/stop_agv.sh --domain-id 11
```

二车继续使用本车已验证的配置，例如 `config/chassis_robot2.yaml`。同一车的导航、VR/Auto bridge、调试工具也必须使用该 Domain。`sudo` 可能丢弃调用终端环境，因此推荐显式 `--domain-id`，不要只依赖终端 `export`。

每台主机使用同一个部署账户，升级前停止旧栈，禁止绕过脚本重复启动。实例冲突检查仅覆盖该账户下带元数据的 tmux session，不能证明跨用户或手工启动进程的接口独占；不同机器人的 Domain 唯一性由部署人员维护。

默认 session 为 `agv_control`，接口为 `can0`；可用 `--session`、`--interface` 指定。启动会记录 Domain/CAN/参数文件。重复启动同一实例或发现已管理实例冲突时拒绝操作，不再杀掉旧实例。旧版本创建且没有元数据的 session 不能由新停机脚本猜测归属；请先使用原版本停机流程完成迁移。

ROS 图就绪检查仅表示进程和接口出现，不表示电机可运动。启动失败保留 session/CAN 供检查；停机拒绝或超时同样保留。`AGV_STARTUP_TIMEOUT_S` 默认 30，`AGV_SHUTDOWN_TIMEOUT_S` 默认 15。

回退前先用当前版本完成停机并确认车辆实际停止，再切回 `main`、重新构建，使用原分支脚本及原验证参数。不要在运行中的控制进程上切换版本。

## 回归与实车测试顺序

GitHub Actions 使用 Humble/Jammy 编译，执行七组核心 gtest、停机客户端测试和模拟 CAN 的 ROS 节点测试。模拟器不会打开 SocketCAN 接口；实车结论必须另行记录。

| 阶段 | 测试 | 验收 |
|---|---|---|
| 上电静止 | 默认参数启动、无使能、按住使能启动 | 不产生运动目标；诊断能解释等待原因 |
| 正常回归 | 原验证过的直行、转向、横移、Auto | 与 main 的几何/方向/速度行为一致 |
| 输入失联 | 当前来源断流，备用来源持续发零 | 当前运动指令约 200 ms 后开始减速；备用源不能延续旧指令 |
| 软件急停 | 按住急停再按模式键 | 保持 Idle/零速，释放后仍需新使能 |
| 通信异常 | 安全台架上模拟单轮/舵反馈丢失 | 明确故障原因、锁存停止、健康轮有零速指令 |
| 恢复 | 恢复反馈但不复位，随后复位但不使能 | 两个阶段都不自动运动；新使能后从零爬升 |
| 停机 | 正常响应、sender 不在、确认超时 | 后两种情况不得自动关闭 CAN/tmux |
| 可选运动策略 | 单独开启舵角门控/回中保持 | 比较方向切换轨迹、响应时间和可接受误差后再决定启用 |

先在安全台架/低速条件验证故障用例，再做地面与多车测试。记录软件提交、配置、Domain、故障时间、诊断、CAN 捕获与实际停止时间。物理急停、驱动器通信 watchdog、终止帧的制动语义仍需独立确认。
