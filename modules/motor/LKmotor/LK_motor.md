# LK Motor 驱动

`LKmotor` 是瓴控 M 系列电机 CAN 协议驱动。模块负责实例注册、协议打包、反馈解析、在线检测以及统一任务发送，不在主控侧重复实现电机内部已有的位置、速度和转矩控制器。

## 任务与通信路径

LK 电机不为每个物理电机创建任务。`application/robot_task.c` 中的 `motor_task` 约每 1 ms 调用一次 `MotorControlTask()`，后者统一调用 `LKMotorControl()`。

```text
应用任务设置目标或提交查询命令
    ↓
LKMotorInstance 内部状态 / 固定命令队列
    ↓
MotorControlTask()，约 1 kHz
    ↓
LKMotorControl()
    ├─ 单电机模式：取出一条待发送命令，发送到 0x140 + ID
    └─ 多电机模式：组合一帧 0x280 转矩控制帧
    ↓
CANTransmit()
    ↓
FDCAN TX FIFO，由硬件异步完成总线发送

电机反馈 0x140 + ID
    ↓
FDCAN 接收中断
    ↓
BSP CAN 接收事件队列
    ↓
高优先级 CANProcessTask
    ↓
LKMotorDecode()
    ├─ 更新反馈数据
    └─ DaemonReload()
```

## 注册时固定控制模式

控制模式必须在 `LKMotorInit()` 注册实例时配置，注册后不可更改。驱动不再提供运行期模式切换接口。

```c
typedef enum
{
    LK_MOTOR_MODE_MULTI_TORQUE,
    LK_MOTOR_MODE_SINGLE_OPEN_TORQUE,
    LK_MOTOR_MODE_SINGLE_TORQUE,
    LK_MOTOR_MODE_SINGLE_SPEED,
    LK_MOTOR_MODE_SINGLE_POSITION,
} LKMotor_Work_Mode_e;
```

| 注册模式 | CAN协议 | 可用控制接口 |
|---|---|---|
| `LK_MOTOR_MODE_MULTI_TORQUE` | `0x280` 多电机转矩帧 | `LKMotorSetMultiTorque()` |
| `LK_MOTOR_MODE_SINGLE_OPEN_TORQUE` | `0x140 + ID`，命令 `0xA0` | `LKMotorSetOpenTorque()` |
| `LK_MOTOR_MODE_SINGLE_TORQUE` | `0x140 + ID`，命令 `0xA1` | `LKMotorSetTorque()` |
| `LK_MOTOR_MODE_SINGLE_SPEED` | `0x140 + ID`，命令 `0xA2` | `LKMotorSetSpeed()` |
| `LK_MOTOR_MODE_SINGLE_POSITION` | `0x140 + ID`，命令 `0xA3~0xA8` | 所有位置控制接口 |

调用与注册模式不匹配的控制接口会返回 `HAL_ERROR`。实例处于停止状态时，单电机控制接口返回 `HAL_BUSY`，不会绕过停止状态发送新的控制目标。

> `0x280` 模式需要提前在 LK Motor Tool 中启用。官方协议说明多电机模式和单电机模式不能对同一台电机同时使用，软件中的模式枚举不能改变电机工具里的实际配置。

## 初始化配置

```c
typedef struct
{
    CAN_Init_Config_s can_init_config;
    LKMotor_Work_Mode_e work_mode;
} LKMotor_Init_Config_s;
```

`can_init_config.tx_id` 在注册时填写逻辑电机 ID，而不是最终发送帧 ID。驱动会自动配置实际收发 ID。

### 多电机模式示例

```c
LKMotor_Init_Config_s config = {
    .can_init_config = {
        .can_handle = &hfdcan1,
        .tx_id = 1,
    },
    .work_mode = LK_MOTOR_MODE_MULTI_TORQUE,
};

LKMotorInstance *motor = LKMotorInit(&config);
LKMotorSetMultiTorque(motor, 500);
```

多电机模式限制：

- ID 只能为 1～4；
- 同一 ID 不能在同一条 FDCAN 上重复注册；
- 当前模块维护一个 `0x280` 多电机组，该组内电机必须使用同一条 FDCAN；
- 4 个槽位分别对应 DATA[0:1]、DATA[2:3]、DATA[4:5]、DATA[6:7]；
- 控制量为小端 `int16_t`，范围 `-2000~2000`。

### 单电机速度模式示例

```c
LKMotor_Init_Config_s config = {
    .can_init_config = {
        .can_handle = &hfdcan2,
        .tx_id = 12,
    },
    .work_mode = LK_MOTOR_MODE_SINGLE_SPEED,
};

LKMotorInstance *motor = LKMotorInit(&config);
LKMotorSetSpeed(motor, 30000); // 300.00 degree/s
```

单电机模式支持 ID 1～32，实际收发 ID 为 `0x140 + ID`。不同单电机实例可以位于不同的 FDCAN 总线上。

## 单电机控制命令

| 接口 | 命令 | 单位/范围 |
|---|---|---|
| `LKMotorSetOpenTorque()` | `0xA0` | 开环功率 `-1000~1000` |
| `LKMotorSetTorque()` | `0xA1` | 转矩电流 `-2000~2000` |
| `LKMotorSetSpeed()` | `0xA2` | `0.01 degree/s` |
| `LKMotorSetMultiTurnPosition()` | `0xA3` | 多圈位置，`0.01 degree` |
| `LKMotorSetMultiTurnPositionWithSpeed()` | `0xA4` | 多圈位置 + 最大速度 |
| `LKMotorSetSingleTurnPosition()` | `0xA5` | 单圈位置 `0~35999` |
| `LKMotorSetSingleTurnPositionWithSpeed()` | `0xA6` | 单圈位置 + 最大速度 |
| `LKMotorSetIncrementPosition()` | `0xA7` | 增量位置，`0.01 degree` |
| `LKMotorSetIncrementPositionWithSpeed()` | `0xA8` | 增量位置 + 最大速度 |

官方协议对不同系列的命令能力有限制：`0xA0` 仅适用于支持开环功率控制的 MS 系列，`0xA1` 仅适用于支持转矩闭环的 MF/MG 系列。注册模式必须与实际电机能力一致。

## 单电机命令队列

单电机控制、查询和维护命令不会在调用者任务中直接操作 CAN，而是放入实例内部的固定队列，由 `LKMotorControl()` 统一发送。

- 每个实例队列深度为 8；
- 队列使用短临界区保护，避免应用任务与电机任务并发读写造成数据撕裂；
- 队列已满时接口返回 `HAL_BUSY`，不会静默覆盖旧命令；
- 一次 `LKMotorControl()` 最多为每个单电机实例发送一条命令；
- CAN提交失败时，当前命令保留并在下一个控制周期重试；
- 停止和关闭命令具有最高优先级，会清除普通待发送命令。

接口返回 `HAL_OK` 表示命令已进入模块发送流程，不代表物理 CAN 总线已经完成发送。

## 停止、关闭与恢复

```c
LKMotorStop(motor);
LKMotorEnable(motor);
LKMotorOff(motor);
```

行为随注册模式确定：

| 接口 | 多电机模式 | 单电机模式 |
|---|---|---|
| `LKMotorStop()` | 将对应 `0x280` 槽位置零 | 高优先级发送 `0x81` |
| `LKMotorEnable()` | 恢复周期控制，目标仍为零 | 若此前是 `0x81` 停止则发送 `0x88` |
| `LKMotorOff()` | 等同于安全置零 | 高优先级发送 `0x80`，清除电机内部旧控制状态 |

调用 `LKMotorStop()` 或 `LKMotorOff()` 会清零模块侧参考值。停止后需要先调用 `LKMotorEnable()`，再设置新的控制目标。

如果停止或关闭命令还没有成功提交到 CAN，`LKMotorEnable()` 会返回 `HAL_BUSY`，避免后发的恢复命令覆盖尚未发送的安全命令。

如果实例因掉线自动执行了 `0x80` 关闭，反馈恢复后不会自动使能。此时 `LKMotorEnable()` 只解除模块侧停止锁，之后必须发送新的控制目标；不会用 `0x88` 恢复掉线前的旧目标。

## 掉线保护

每个实例注册 daemon，连续约 50～60 ms 没有反馈后判定离线。

离线时驱动会：

1. 只记录一次离线日志；
2. 清零参考值；
3. 置为停止状态；
4. 多电机模式继续发送零转矩槽位；
5. 单电机模式清除普通命令并优先尝试发送 `0x80`；
6. 禁止反馈恢复后自动恢复旧控制目标。

恢复反馈时只记录一次恢复日志并恢复在线状态，仍需上层显式调用 `LKMotorEnable()`。

## 反馈数据

控制类回复和 `0x9C` 状态回复包含：

- 温度：`int8_t`，`1 ℃/LSB`；
- 转矩电流：`int16_t`；
- 速度：`int16_t`，`1 degree/s/LSB`；
- 编码器：14 bit，范围 `0~16383`。

`0xA0` 的 DATA[2:3] 是开环功率而不是转矩电流，因此分别保存为：

- `open_power` / `open_power_valid`；
- `real_current` / `real_current_valid`。

第一次收到编码器反馈时只初始化角度状态，不进行跨圈判断。从第二帧开始，以半量程 8192 判断跨零点：

```text
delta > 8192   -> 反向跨圈，total_round--
delta < -8192  -> 正向跨圈，total_round++
```

写入新零点或清除角度后会重新初始化软件多圈累计，避免零点突变被误判为跨圈。

`LKMotorGetMeasure()` 在任务态短暂锁住调度器后复制反馈快照，避免与高优先级 `CANProcessTask` 的解析更新互相打断。

## 查询与维护命令

以下接口只适用于单电机协议实例；对 `LK_MOTOR_MODE_MULTI_TORQUE` 实例调用会返回 `HAL_ERROR`。

| 接口 | 命令 | 结果 |
|---|---|---|
| `LKMotorReadPID()` | `0x30` | `measure.pid_param` |
| `LKMotorWritePIDToRAM()` | `0x31` | 掉电失效 |
| `LKMotorWritePIDToROM()` | `0x32` | 掉电保存，避免频繁写入 |
| `LKMotorReadAcceleration()` | `0x33` | `measure.accel_dps2` |
| `LKMotorWriteAccelerationToRAM()` | `0x34` | 写入 RAM |
| `LKMotorReadEncoder()` | `0x90` | `measure.encoder` |
| `LKMotorWriteEncoderZeroToROM()` | `0x91` | 写入编码器零偏 |
| `LKMotorSetCurrentPositionAsZero()` | `0x19` | 写当前位置为 ROM 零点 |
| `LKMotorReadMultiTurnAngle()` | `0x92` | `measure.multi_turn_angle_0p01deg` |
| `LKMotorReadSingleTurnAngle()` | `0x94` | `measure.single_turn_angle_0p01deg` |
| `LKMotorClearAngle()` | `0x95` | 清除驱动角度信息 |
| `LKMotorReadStatus1AndError()` | `0x9A` | 温度、电压和错误状态 |
| `LKMotorClearError()` | `0x9B` | 清除错误状态 |
| `LKMotorReadStatus2()` | `0x9C` | 通用运动反馈 |
| `LKMotorReadStatus3()` | `0x9D` | 三相电流 |

读取类命令是异步的。接口返回成功后，应等待电机回复，再通过 `LKMotorGetMeasure()` 获取最新快照。

## 文件

| 文件 | 说明 |
|---|---|
| `LK9025.h` | 注册配置、固定模式、公开控制与查询接口 |
| `LK9025.c` | CAN编解码、任务发送、队列、掉线保护和反馈解析 |
| `LK-TECH电机CAN协议说明V2_3.pdf` | 官方 M 系列 CAN 协议 |
| `报文格式.png` | `0x280` 多电机控制帧截图 |
| `反馈报文.png` | 转矩控制反馈帧截图 |
