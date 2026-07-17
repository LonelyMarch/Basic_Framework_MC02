# DJI 电机驱动

`DJImotor` 是 C610、C620 和 GM6020 的 CAN 驱动层。它负责两件事：按照 DJI 官方协议解析反馈报文，以及把上层给出的直接指令或
PID 计算结果打包成控制帧发送出去。

模块不为每个物理电机创建 RTOS 任务。所有 DJI 实例由 `MotorControlTask()` 统一管理，`application/robot_task.c` 中的
`motor_task` 以约 1 kHz 周期调用该入口，并进一步调用一次 `DJIMotorControl()`。CAN 接收则由 BSP 的 `CANProcessTask`
在任务上下文中分发给本模块的反馈回调。

## 支持的设备与协议映射

初始化函数已经固定了电机型号和底层协议模式，注册后不能再切换：

| 初始化函数                         | 电机/控制器       | 协议指令 | 逻辑 ID | 控制帧 ID                        | 反馈帧 ID          |
|-------------------------------|--------------|------|-------|-------------------------------|-----------------|
| `DJIMotorInitM2006()`         | M2006 + C610 | 转矩电流 | 1~8   | ID 1~4：`0x200`；ID 5~8：`0x1FF` | `0x201`~`0x208` |
| `DJIMotorInitM3508()`         | M3508 + C620 | 转矩电流 | 1~8   | ID 1~4：`0x200`；ID 5~8：`0x1FF` | `0x201`~`0x208` |
| `DJIMotorInitGM6020Voltage()` | GM6020       | 转矩电压 | 1~7   | ID 1~4：`0x1FF`；ID 5~7：`0x2FF` | `0x205`~`0x20B` |
| `DJIMotorInitGM6020Current()` | GM6020       | 转矩电流 | 1~7   | ID 1~4：`0x1FE`；ID 5~7：`0x2FE` | `0x205`~`0x20B` |

这里的“逻辑 ID”就是 `DJIMotor_Init_Config_s.can_init_config.tx_id`。调用者只填写 CAN 句柄和电机编号，不要把 `0x200`、
`0x1FF` 等控制帧 ID 填入该字段；驱动会根据型号和编号自动计算发送 ID、反馈 ID 以及控制帧中的两字节槽位。

GM6020 电流模式要求在 RoboMaster Assistant 中预先启用电流环。C610、C620 和 GM6020 的反馈帧均为 8 字节标准 CAN 数据帧。

## 注册配置

```c
DJIMotor_Init_Config_s config = {
    .can_init_config = {
        .can_handle = &hfdcan1,
        .tx_id = 1,                 // DJI 电机逻辑 ID，不是最终 CAN 控制帧 ID
    },
    .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
    .control_mode = DJI_CONTROL_SPEED,
    .feedback_source = DJI_FEEDBACK_MOTOR,
    .speed_pid = {
        .Kp = 20.0f,
        .Ki = 1.0f,
        .Kd = 0.0f,
        .IntegralLimit = 10000.0f,
        .Improve = PID_Integral_Limit,
        .MaxOut = 15000.0f,
    },
};

DJIMotorInstance *motor = DJIMotorInitM3508(&config);
```

`DJIMotor_Init_Config_s` 的字段含义如下：

- `can_init_config.can_handle`：使用的 FDCAN 外设句柄。
- `can_init_config.tx_id`：电机逻辑 ID。驱动会检查 ID 范围、反馈 ID 冲突和控制帧槽位冲突。
- `motor_reverse_flag`：是否反转电机方向。
- `control_mode`：固定的上层控制方式，见下一节。
- `feedback_source`：闭环控制使用电机自身反馈，还是使用外部提供的角度/速度变量。
- `position_feedback_ptr`、`speed_feedback_ptr`：使用外部反馈时的只读指针，单位分别为 degree 和
  degree/s。速度模式必须提供速度指针；位置模式必须同时提供位置和速度指针。
- `position_pid`、`speed_pid`：位置环和速度环的 `PID_Init_Config_s`。直接模式不使用这些参数，速度模式只使用速度
  PID，位置模式使用位置串级速度 PID。

注册失败时函数返回 `NULL` 并记录日志。每个 CAN 总线上的反馈 ID 不能重复；同一个控制帧中的四个两字节槽位也不能被两个实例重复占用。模块静态池最多保存
12 个 DJI 实例和 12 个控制帧发送分组。

## 控制模式

`control_mode` 在注册时确定，运行期间不能切换。

### 直接模式：`DJI_CONTROL_DIRECT`

直接模式不运行 PID，由上层直接提交协议原始量：

```c
DJIMotorSetCurrentRaw(motor, current_raw);
DJIMotorSetVoltageRaw(motor, voltage_raw);
```

`DJIMotorSetCurrentRaw()` 适用于 C610、C620 和 GM6020 电流模式；GM6020 电压模式必须使用 `DJIMotorSetVoltageRaw()`
。如果控制模式或协议模式不匹配，函数只记录错误并忽略指令。

驱动会自动限幅：

| 协议        |             原始指令范围 |             反馈转矩电流换算 |
|-----------|-------------------:|---------------------:|
| C610 电流   | `-10000` ~ `10000` | `raw × 10 / 10000 A` |
| C620 电流   | `-16384` ~ `16384` | `raw × 20 / 16384 A` |
| GM6020 电流 | `-16384` ~ `16384` |  `raw × 3 / 16384 A` |
| GM6020 电压 | `-25000` ~ `25000` |          直接使用协议原始电压量 |

### 速度模式：`DJI_CONTROL_SPEED`

速度模式调用速度 PID，将目标速度转换为协议电流或电压指令：

```c
DJIMotorSetSpeed(motor, speed_degree_per_second);
```

目标值和反馈值单位均为 degree/s。电机反馈来自 `measure.speed_aps`；外部反馈模式则读取 `speed_feedback_ptr`。PID
输出最终会按底层协议的原始量程再次限幅。

### 位置模式：`DJI_CONTROL_POSITION`

位置模式是位置环串级速度环：位置 PID 先生成速度目标，速度 PID 再生成最终协议指令。

```c
DJIMotorSetPosition(motor, position_degree);
```

位置目标和反馈单位均为 degree。使用电机反馈时，位置反馈是从第一次有效反馈开始累计的多圈角度 `measure.total_angle`
；使用外部反馈时读取 `position_feedback_ptr`，速度环读取 `speed_feedback_ptr`。

## 方向与安全状态

- `DJIMotorInitInternal()` 注册后默认将实例置为停止状态，然后尝试使能。
- `DJIMotorEnable()` 只有在 daemon 判定电机在线时才会进入 `MOTOR_ENABLED`；否则保持停止。
- `DJIMotorStop()` 立即把待发送指令清零，并在下一次控制周期重置 PID 积分、微分等内部状态。
- 电机反馈丢失时，daemon 回调会清零指令、置停止标志并请求重置控制器；恢复反馈后需要上层再次调用 `DJIMotorEnable()`。
- 电机方向为 `MOTOR_DIRECTION_REVERSE` 时，电机自身的速度和累计角度反馈在闭环计算前取反，最终发送指令也取反。外部反馈指针不会在驱动内取反，使用外部反馈时应由上层提供与目标坐标系一致的符号。

daemon 的重载计数为 2，daemon 任务周期为 10 ms；反馈连续两个 daemon 周期没有到达后，会在下一次检查中触发掉线处理，实际约为
20~30 ms。即使选择外部反馈，DJI 总线反馈仍会被注册并用于在线检测。

## 反馈数据

每个实例通过 `DJIMotorInstance.measure` 暴露以下数据：

| 字段                     | 含义                                 |
|------------------------|------------------------------------|
| `ecd`                  | 当前机械角度原始值，范围 0~8191                |
| `last_ecd`             | 上一帧机械角度原始值                         |
| `speed_rpm`            | 协议原始转速，单位 rpm                      |
| `torque_current_raw`   | 协议原始转矩电流                           |
| `temperature`          | C620/GM6020 的温度；M2006/C610 中无效     |
| `temperature_valid`    | 温度字段是否有效                           |
| `angle_single_round`   | 当前单圈角度，单位 degree，范围约 0~360         |
| `total_ecd`            | 从第一次有效反馈开始累计的编码器增量                 |
| `total_angle`          | `total_ecd × 360 / 8192`，单位 degree |
| `speed_aps`            | 由 rpm 换算的角速度，单位 degree/s           |
| `torque_current_a`     | 按型号量程换算的转矩电流，单位 A                  |
| `feedback_initialized` | 是否已经收到第一帧有效反馈                      |

第一帧反馈只建立累计角度的基准，`total_angle` 从 0 开始；后续帧按半圈范围 `±4096` 处理编码器回绕，因此可以连续累计多圈角度。

## 周期控制与 CAN 收发路径

```text
电机反馈帧
    ↓ FDCAN 接收中断
BSP FIFO / 接收事件队列
    ↓
CANProcessTask()
    ↓ DJIDecodeFeedback()
更新 DJIMotorInstance.measure，并重载 daemon

应用任务设置目标或直接指令
    ↓
MotorControlTask()（约 1 kHz）
    ↓ DJIMotorControl()
计算 PID、写入控制帧槽位、按发送分组调用 CANTransmit()
    ↓
FDCAN TX FIFO
    ↓
FDCAN 硬件异步完成总线发送
```

`DJIMotorControl()` 先遍历所有实例，向其所属控制帧的两字节槽位写入指令，再遍历发送分组。相同 CAN 句柄和控制帧 ID
的电机会共用一帧，每个实例占用其中一个 16 位槽位；一个控制帧最多装载 4 个电机指令。发送调用使用
`CANTransmit(&sender, 1.0f)`，超时或提交失败由 BSP CAN 统一记录日志并返回失败。

## 常用调用示例

### M3508 速度闭环

```c
DJIMotor_Init_Config_s cfg = {
    .can_init_config = { .can_handle = &hfdcan1, .tx_id = 1 },
    .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
    .control_mode = DJI_CONTROL_SPEED,
    .feedback_source = DJI_FEEDBACK_MOTOR,
    .speed_pid = { .Kp = 10.0f, .MaxOut = 12000.0f },
};

DJIMotorInstance *motor = DJIMotorInitM3508(&cfg);
DJIMotorSetSpeed(motor, 3000.0f);       // degree/s
DJIMotorEnable(motor);
```

### GM6020 外部位置闭环

```c
DJIMotor_Init_Config_s cfg = {
    .can_init_config = { .can_handle = &hfdcan3, .tx_id = 1 },
    .control_mode = DJI_CONTROL_POSITION,
    .feedback_source = DJI_FEEDBACK_EXTERNAL,
    .position_feedback_ptr = &yaw_angle_degree,
    .speed_feedback_ptr = &yaw_speed_degree_per_second,
    .position_pid = { .Kp = 8.0f, .MaxOut = 500.0f },
    .speed_pid = { .Kp = 50.0f, .Ki = 200.0f, .MaxOut = 20000.0f },
};

DJIMotorInstance *motor = DJIMotorInitGM6020Voltage(&cfg);
DJIMotorSetPosition(motor, target_angle_degree);
DJIMotorEnable(motor);
```

应用层只负责更新目标、选择停止或使能状态；不要自行调用 `DJIMotorControl()`，也不要直接修改 `sender_can_instance` 或发送缓存。
