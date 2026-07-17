# DM 电机模块

## 1. 模块定位

`modules/motor/DMmotor`负责达妙电机的 CAN 通信、协议打包、反馈解析和在线监测，不在主控侧增加位置环或速度环 PID。

当前支持：

- 传统 MIT 模式；
- 传统位置速度模式；
- 传统速度模式；
- E-MIT（EMIT）力位混控模式；
- 一拖四固件的广播电流模式。

所有实例由统一的 `MotorControlTask()` 管理，不为每个物理电机创建任务。`application/robot_task.c` 中的 `motor_task` 约每 1
ms 调用一次 `MotorControlTask()`，随后调用 `DMMotorControl()` 完成全部 DM 实例的周期发送。

CAN 接收中断只把报文复制到 BSP 队列；`CANProcessTask()`在任务上下文中调用本模块的反馈解析函数。

## 2. 控制路径

```text
应用层注册实例并固定模式
        │
        ├─ DMMotorInitMIT()
        ├─ DMMotorInitPosVel()
        ├─ DMMotorInitVel()
        ├─ DMMotorInitEmit()
        └─ DMMotorInit1To4()
        │
        ▼
注册 CANInstance 和 DaemonInstance
        │
        ▼
应用层调用对应 SetCommand() 更新目标
        │
        ▼
MotorControlTask() → DMMotorControl()
        │
        ▼
按注册时固定的模式打包并调用 CANTransmit()
        │
        ▼
FDCAN 硬件 TX FIFO 异步发送
```

注册成功后实例保持停止态。模块不会自动使能，也不会自动保存零点。应用层应先设置安全目标，再显式调用`DMMotorEnable()`。

## 3. 传统控制模式

| 注册接口                  |            发送 ID | DLC | 控制内容                            |
|-----------------------|-----------------:|----:|---------------------------------|
| `DMMotorInitMIT()`    |         `CAN_ID` |   8 | 位置16位、速度12位、Kp12位、Kd12位、前馈扭矩12位 |
| `DMMotorInitPosVel()` | `0x100 + CAN_ID` |   8 | 小端float位置、小端float速度上限           |
| `DMMotorInitVel()`    | `0x200 + CAN_ID` |   4 | 小端float速度                       |
| `DMMotorInitEmit()`   | `0x300 + CAN_ID` |   8 | 小端float位置、速度限幅、电流限幅标幺值          |

模式在注册时由不同初始化接口确定，运行期不提供模式切换接口。

### 3.1 映射范围

传统模式注册时必须显式配置`p_min/p_max`、`v_min/v_max`和`t_min/t_max`，并与电机调试助手中的 PMAX、VMAX、TMAX 完全一致。

模块会在注册时检查：

- 上下限必须是有限浮点数；
- 每组上限必须严格大于下限；
- 基础 CAN_ID 不得大于`0x0F`，因为传统反馈 D0 低4位只能表示4位电机 ID。

范围不合法时注册失败并返回`NULL`。

### 3.2 E-MIT最大电流

`DMMotorEmitArgs.imax`必须显式配置为电机上电打印或调试助手中显示的最大电流，且必须大于0。模块不提供默认 Imax，因为不同 DM
型号的最大电流并不相同。

E-MIT发送的电流字段计算方式为：

```text
i_des = cur_limit / imax × 10000
```

结果限制在`0～10000`。

## 4. 一拖四模式

一拖四模式只实现官方 PDF 定义的广播电流通信，不发送传统模式的`FF FF ... FC/FD/FE/FB`特殊命令。

固定映射如下：

| 广播发送 ID | 槽位 |   反馈 ID |
|--------:|---:|--------:|
| `0x3FE` |  0 | `0x301` |
| `0x3FE` |  1 | `0x302` |
| `0x3FE` |  2 | `0x303` |
| `0x3FE` |  3 | `0x304` |
| `0x4FE` |  0 | `0x305` |
| `0x4FE` |  1 | `0x306` |
| `0x4FE` |  2 | `0x307` |
| `0x4FE` |  3 | `0x308` |

注册时会校验广播 ID、槽位和反馈 ID 的对应关系，并禁止同一路 FDCAN、同一个广播 ID 下重复注册同一槽位。

### 4.1 广播发送

每台电机占广播帧中的2字节。根据《一拖四版本说明.pdf》，每个控制电流都按大端排列：

```text
D[2 × slot]     = 控制电流高8位
D[2 × slot + 1] = 控制电流低8位
```

协议值`±16384`对应电机最大电流。`current_max`必须在注册时显式配置并大于0；`current_to_out_cfg`为0时自动按下式计算：

```text
current_to_out = 16384 / current_max
```

同一路 FDCAN 上相同广播 ID 的实例共用一个发送缓存。不同 FDCAN 上即使广播 ID 相同，也会使用各自独立的发送实例，不会跨总线复用。

### 4.2 一拖四启停语义

- `DMMotorEnable()`：只解除模块侧停止标志，不发送传统`0xFC`命令；
- `DMMotorStop()`：清零该实例的目标电流，后续广播帧对应槽位保持0；
- `DMMotorSetZero()`：一拖四协议未定义该命令，调用时记录警告并返回；
- `DMMotorClearError()`：一拖四协议未定义该命令，调用时记录警告并返回。

## 5. 反馈解析

### 5.1 传统模式反馈

传统 MIT、位置速度、速度和 E-MIT 使用相同的8字节反馈格式：

- D0高4位：状态/错误码；
- D0低4位：基础 CAN_ID；
- D1～D2：16位位置；
- D3～D4：12位速度；
- D4～D5：12位扭矩；
- D6：MOS温度；
- D7：转子/线圈温度。

解析前会检查 DLC 必须等于8，并检查 D0低4位与当前实例基础 CAN_ID 一致。校验失败的帧不会更新反馈，也不会给 daemon 喂狗。

### 5.2 一拖四反馈

一拖四反馈同样固定为8字节：

- D0～D1：大端单圈编码器；
- D2～D3：大端有符号速度，单位`rpm × 100`；
- D4～D5：大端有符号电流，单位mA；
- D6：线圈/转子温度；
- D7：PCB/MOS温度。

首帧反馈只建立编码器基准，不进行跨圈判断，避免电机初始位置位于后半圈时产生错误的负圈计数。离线恢复后的第一帧也会重新建立跨圈基准。

## 6. 在线与离线处理

每个实例注册一个 daemon。收到合法反馈后调用`DaemonReload()`；超时后执行离线回调：

1. 将实例置为停止态；
2. 清除尚未发送的特殊命令；
3. 清除旧控制目标；
4. 将反馈错误标记为`DM_ERROR_OFFLINE`；
5. 清除首帧标记，使恢复后的第一帧重新建立编码器基准。

模块不会自动重新使能，也不会自动恢复失联前目标。恢复控制时，应用层必须重新设置目标并显式调用`DMMotorEnable()`。

## 7. 公开接口

```c
DMMotorInstance* DMMotorInitMIT(Motor_Init_Config_s* config,
                                const DMMotorMitArgs* mit_args);
DMMotorInstance* DMMotorInitPosVel(Motor_Init_Config_s* config,
                                   const DMMotorPosVelArgs* posvel_args);
DMMotorInstance* DMMotorInitVel(Motor_Init_Config_s* config,
                                const DMMotorVelArgs* vel_args);
DMMotorInstance* DMMotorInitEmit(Motor_Init_Config_s* config,
                                 const DMMotorEmitArgs* emit_args);
DMMotorInstance* DMMotorInit1To4(Motor_Init_Config_s* config,
                                 const DMMotor1To4Args* args_1to4);

void DMMotorMITSetCommand(DMMotorInstance* motor,
                          float position, float velocity,
                          float kp, float kd, float torque_ff);
void DMMotorPosVelSetCommand(DMMotorInstance* motor,
                             float position, float velocity_limit);
void DMMotorVelSetCommand(DMMotorInstance* motor, float velocity);
void DMMotorEmitSetCommand(DMMotorInstance* motor,
                           float position, float vel_limit, float cur_limit);
void DMMotor1To4SetCurrent(DMMotorInstance* motor, float current);

void DMMotorStop(DMMotorInstance* motor);
void DMMotorEnable(DMMotorInstance* motor);
void DMMotorSetZero(DMMotorInstance* motor);
void DMMotorClearError(DMMotorInstance* motor);

DMMotorFeedback* DMMotorGetNormalFeedback(DMMotorInstance* motor);
DMMotorFeedback1To4* DMMotorGet1To4Feedback(DMMotorInstance* motor);
DMMotorError DMMotorGetError(DMMotorInstance* motor);
bool DMMotorIsOnline(DMMotorInstance* motor);
void DMMotorControl(void);
```

按照接口约定，各模式专用`SetCommand`只应传入对应初始化接口返回的实例。当前实现不在每次调用时重复验证实例模式，以避免周期控制路径中的额外分支。

## 8. 初始化示例

### 8.1 MIT模式

```c
DMMotorMitArgs mit_args = {
    .normal.range = {
        .p_min = -12.5f,
        .p_max = 12.5f,
        .v_min = -45.0f,
        .v_max = 45.0f,
        .t_min = -7.0f,
        .t_max = 7.0f,
    },
    .position = 0.0f,
    .velocity = 0.0f,
    .kp = 0.0f,
    .kd = 1.0f,
    .torque_ff = 0.0f,
};

DMMotorInstance* motor = DMMotorInitMIT(&config, &mit_args);
if (motor != NULL)
{
    DMMotorMITSetCommand(motor, 0.0f, 0.0f, 10.0f, 1.0f, 0.0f);
    DMMotorEnable(motor); // 注册不会自动使能，确认目标安全后显式使能
}
```

### 8.2 一拖四模式

```c
DMMotor1To4Args args_1to4 = {
    .can_rx_id = 0x301,
    .slot_index = 0,
    .tx_id_1to4 = DM_1TO4_ID_LO,
    .encoder_per_round_cfg = DM_1TO4_ENCODER_PER_ROUND_DEFAULT,
    .current_max = 10.26f,       // 必须填写该电机固件实际最大电流
    .current_to_out_cfg = 0.0f,  // 由模块按 16384/current_max 计算
    .target = 0.0f,
};

DMMotorInstance* motor = DMMotorInit1To4(&config, &args_1to4);
if (motor != NULL)
{
    DMMotor1To4SetCurrent(motor, 0.0f);
    DMMotorEnable(motor); // 只解除模块侧停止标志，不发送传统 FC 命令
}
```

## 9. 依赖

- `bsp/can`：CAN 注册、异步发送和任务上下文接收分发；
- `modules/daemon`：在线监测；
- `modules/motor/motor_def.h`：公共启停状态；
- `modules/motor/motor_task.c`：统一电机控制任务入口。
