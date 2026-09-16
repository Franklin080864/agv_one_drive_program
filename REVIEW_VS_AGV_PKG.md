# agv2_pkg vs agv_pkg —— 重构差异 Review

对照基线：`agv_pkg/src/agv_control.cpp`（原车上稳定运行版本）
对照对象：`agv2_pkg/`（重构后版本）
关注三件事：CAN 通信是否吻合、遥控逻辑是否变了、能否一次跑通。

---

## 1. CAN 通信层 —— 字节级基本一致

逐帧对比过的关键报文如下。

| 项目 | agv_pkg 原值 | agv2_pkg 实现位置 | 一致？ |
|---|---|---|---|
| 转向电机 ID（后/前） | `0x141` / `0x142` | [can_codec.hpp:10-11](include/agv2_pkg/can_codec.hpp#L10-L11) | ✅ |
| 转向使能 | `data={0x88, 0…}` | [can_codec.cpp:35-39](src/can_codec.cpp#L35-L39) | ✅ |
| 转向位置查询 | `data={0x94, 0…}` | [can_codec.cpp:41-45](src/can_codec.cpp#L41-L45) | ✅ |
| 多圈位置命令 | `0xa4, 0x00, motor_spped=300 LE, angle*100 LE int32` | [can_codec.cpp:47-59](src/can_codec.cpp#L47-L59)（`kSteerSpeedDps=300`） | ✅ |
| 轮毂速度（0x201/0x202）| `0x0f, 0x00, 0x03, val[0..3], 0x00` | [can_codec.cpp:61-77](src/can_codec.cpp#L61-L77) | ✅ |
| 轮毂控制字 0x06 / 0x07 / 0x0f | `data[0]=步进, data[2]=0x03` | [can_codec.cpp:79-89](src/can_codec.cpp#L79-L89) | ✅ |
| SDO 初始化 12 帧 + NMT | 0x601 / 0x602 序列 + 0x000 `{0x01, node_id}` | [can_codec.cpp:91-121](src/can_codec.cpp#L91-L121) | ✅ 字节级一致 |
| 终止帧（Ctrl+C） | `0x001 + {0xFF×7, 0xFD}` | [can_codec.cpp:123-127](src/can_codec.cpp#L123-L127) | ✅ |
| 反馈：0x94 解码 | `int32` 小端 ×0.01 度 | [can_codec.cpp:141-156](src/can_codec.cpp#L141-L156) | ✅ |
| 反馈：0x181 / 0x182 状态位 | bit0/1/2 → ready / switched_on / enabled | [can_codec.cpp:129-139](src/can_codec.cpp#L129-L139) | ✅ |
| 电池 SOC | 0x191 byte[6] | [can_codec.cpp:158-165](src/can_codec.cpp#L158-L165) | ✅ |

### 1.1 唯一会改变上行 0.1 RPM 数值的地方

- **原代码**（`agv_pkg/src/agv_control.cpp:1441-1459`）用**固定标量**：
  - 手柄默认档：`speed_flg × (100+150) = ×250`（≈ 25 RPM）
  - 手柄 Y 加速：`speed_flg × (100+300) = ×400`（≈ 40 RPM）
  - AD 模式：`speed_flg × 2000`（≈ 200 RPM）
- **新代码** [can_codec.cpp:61-77](src/can_codec.cpp#L61-L77) 用**物理转换**：
  `target_0.1rpm = (v_mps / wheel_radius) × 60 / 2π × 10`
- 用 yaml 当前 `wheel_radius=0.07`：
  - 手柄 `teleop_max_speed=0.1 m/s` → ≈ 14 RPM（比原默认档更保守）
  - AD `max_linear_speed=0.5 m/s` → ≈ 68 RPM（**仍低于原 200 RPM**）
- `wheel_radius` 在 [chassis.yaml:9](config/chassis.yaml#L9) 自标 *first-pass estimate*，量不准就速度系统性偏。

### 1.2 方向系数

`front_motor_direction=-1, rear_motor_direction=+1`（[chassis.yaml:12-13](config/chassis.yaml#L12-L13)）和
原代码"前进时 `speed_flg_1=-, speed_flg_2=+`"语义对得上。✅

---

## 2. 遥控控制逻辑 —— **同一组按钮，不同的运动模型**

按钮/轴号继续沿用新手柄（[on_joy](src/agv2_control_node.cpp#L161-L238)），但**底层产生的舵角和轮速变了**。

| 操作 | agv_pkg 原行为 | agv2_pkg 新行为 | 影响 |
|---|---|---|---|
| 上/下键（axis 7） | 舵角 0°，`speed_flg_{1,2}=∓ramp / ±ramp`（逐帧 +0.05 加速） | 舵角=0，`v_front=v_rear=±teleop_max_speed` 一次性给目标，由 limiter 斜率到达 | 行为相近 |
| 左摇杆（axis 1/0）连续控制 | 未定义 | `axis 1 -> vx`、`axis 0 -> omega`，走 `solve_ik(vx,0,omega)` | ✅ 新增连续模型控制 |
| 右摇杆横向（axis 2）自转 | 舵角=**−42.53°**，左右轮同号 ramp（双舵轮零半径旋转） | 走 `solve_ik(0,0,omega)`，默认几何下逻辑舵角≈**+42.5°**，经 `steer_direction=-1` 后硬件≈**−42.5°** | ✅ 已从差速硬拧改回双舵轮几何自转 |
| X 键（btn 3）横移 | 舵角=**−90°**，`dir=1` | 舵角=**+90°** | **舵角符号反**，需在车上确认机械正方向 |
| B 键（btn 1）横移 | 舵角=**+90°** | 舵角=**−90°** | 同上，符号反了 |
| Y 键（btn 4）加速 | 速度档位 `(wheel_spd+150) → (wheel_spd+300)` | **未实现**（按了不加速） | **加速档丢失** |
| A 键（btn 0）减速 | 默认档位 | **未实现** | **减速档丢失** |
| 上电使能（btn 8） | 仅在 axes[0..3] 全在中位时使能 | 同 [agv2_control_node.cpp:181-191](src/agv2_control_node.cpp#L181-L191) | ✅ |
| AD 使能（btn 9） | edge-trigger | 同 [agv2_control_node.cpp:192-195](src/agv2_control_node.cpp#L192-L195) | ✅ |
| 急停（btn 7） | 置位 `emergency_flg` | edge-trigger 进 Idle + lock | ✅ 行为等价 |
| 心跳丢失 0.5s 急停 | `agv_control.cpp:797-811` | [agv2_control_node.cpp:269-282](src/agv2_control_node.cpp#L269-L282) | ✅ |

> **Teleop 已连续化**：左摇杆现在是 `vx + omega` 主控，右摇杆横向仍作为自转备用，两者都复用 `solve_ik()`。横移 X/B 的舵角符号仍建议上车低速确认。

---

## 3. 一次开机就跑通的可能性 —— **大概率跑不通**

### 3.1 阻塞项（必须改）

1. **`chassis.yaml` 的几何全是占位值**（[chassis.yaml:5-9](config/chassis.yaml#L5-L9) 自己也写了 *recalibrate on real robot*）。
   按 `framework.md` 真车是**右前 + 左后对角双舵轮**，所以
   - `front_y` / `rear_y` 不应都为 0
   - `front_x` 与 `rear_x` 也不应对称
   - `wheel_radius=0.07` 仍需按实车复核
   不改直接跑 → IK 会算错向、错速。
2. **上游 topic 已断链**：
   - 原驱动订阅 `steering_commands`（`agv_msg/SteeringAngles`）
   - 新驱动改订阅 `cmd_vel` (Twist) 和 `wheel_command` (`agv2_pkg/WheelCommand`)
   - `apriltag_docking.cpp`、`apriltag_track_controller.py`、`apriltag_docking.py` 都还在发 `steering_commands`，新驱动收不到 → **AD 模式哑火**。
3. **`wheel_radius` 必须实测**，否则所有 m/s ↔ RPM 转换全偏。

### 3.2 时序/行为差异（不致命但与原行为不同）

4. **第一帧紧跟着发 0xa4 位置命令**：
   - 原 `ifFirst` 那帧只发 `0x88` 然后 return；
   - 新 [tick()](src/agv2_control_node.cpp#L260-L315) 第一帧 `bring_up_chassis()` 之后**继续**调 `emit_can_frames()`，会同帧把 `0xa4(0,0)` 发出去。
   - 多数情况无害（命令角度=0），但与原行为不同。
5. **CIA-402 推进**：[progress_wheel_state](src/agv2_control_node.cpp#L394-L402) 每个 tick 都发；原代码只在状态位未到位时发。功能等价，CAN 总线可承受。
6. **`respawn=True`** ([launch](launch/agv2_control.launch.py#L24-L25))：节点崩溃会自动拉起，部署有用；调试时如果是参数错误会无限重启，可能想关掉。

### 3.3 可接受/正向的改动

- 去掉 `gpiod` 依赖 ✅ 部署机不再需要 `libgpiodcxx`。
- 终止帧、SOC、SDO 初始化保持字节级一致 ✅
- 急停按钮 + 心跳超时行为保留 ✅
- 加了 `agv2/lock` 服务、yaml 参数化、单元测试入口（`test_kinematics`、`test_can_codec`） ✅
- rate_limiter 只保留轮速斜坡；舵角直接下发最终目标角，由转向电机自身速度字段控速 ✅
- IK 奇异点保持上次角度（[kinematics.cpp:22-30](src/kinematics.cpp#L22-L30)）而不是把舵角归零，更符合舵轮特性 ✅

---

## 4. 上车前最低改动清单

1. **量并写真实几何**到 `chassis.yaml`：双舵轮真实 `(x, y)` 和真实 `wheel_radius`。
2. **写一个 `steering_commands → wheel_command` 桥**，或改 apriltag 节点直接发 `wheel_command`/`cmd_vel`，否则 AD 模式不动。
3. **遥控自转已改回几何模式**：右摇杆横向走 `solve_ik(0,0,omega)`，默认几何能得到约 42.5° 舵角；横移 X/B 的 ±90° 符号仍需上车低速确认。
4.（可选）补回 Y / A 加减速档位，或在 README 显式声明已废弃。
5.（可选）让 `tick()` 第一帧只做 bring-up，不发 `0xa4`，与原行为对齐。

---

## 5. 一句话结论

CAN 协议层我没看到字节级差异，**底盘"听不懂"的风险主要来自 yaml 几何参数和上游 topic 断链，不是 codec**。
遥控的按钮映射没变，**自转已回到双舵轮几何模型**；横移符号仍需要上车确认。
