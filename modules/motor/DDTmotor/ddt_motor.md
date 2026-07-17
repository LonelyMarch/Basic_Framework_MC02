# 本末 M0601C 电机驱动

## 1. 资料依据

本驱动对应本末科技 M0601C 电机，协议依据：

- `doc/DDT/M0601C+电机使用参考_20240613.pdf`；
- `doc/DDT/README.md`；
- `doc/DDT/数据换算公式.xlsx`；
- `doc/DDT/usart.c`；
- 旧工程`modules/motor/M0601C`仅用于核对接口意图，不作为协议优先依据。

旧工程存在空接收回调、全局单实例发送缓存和手动 GPIO 控制 RS485 方向等未完成设计。本实现使用当前工程已有的`bsp/usart`
，USART2/USART3已经由 STM32 HAL 的`HAL_RS485Ex_Init()`配置硬件自动 DE，不额外增加 BSP 层或手动切换方向引脚。

## 2. 任务与总线架构

M0601C 不为每个物理电机创建任务。一条 RS485 总线注册为一个`DDTMotorBus`，同一总线上的多个电机按 ID 注册为
`DDTMotorInstance`。

```text
应用层设置目标
        │
        ▼
DDTMotorSetCurrent / SetSpeed / SetPosition
        │
        ▼
仅更新实例目标并置待发送标志
        │
        ▼
MotorControlTask()，约 1 kHz
        │
        ▼
DDTMotorControl()
        │
        ▼
每条 RS485 总线一次最多提交一帧
        │
        ▼
bsp/usart → HAL RS485 硬件自动 DE
```

同一总线使用轮询调度，防止多个物理电机同时写 USART。官方`usart.c`显示普通驱动指令后电机会返回10字节反馈，因此每条总线严格保持一个在途请求：发送普通控制或
`0x74`查询后，必须收到回复或等待超时，才会发送下一台电机的帧。官方发送函数每帧后延时10
ms，本驱动使用非阻塞时间间隔实现同样约束，不在统一电机任务中调用阻塞延时。IT/DMA发送尚未完成时，驱动等待下一控制周期，不覆盖
BSP 内部发送缓冲。

## 3. 串口要求

官方文档使用 USB 转 RS485 通信。当前工程建议使用 USART2 或 USART3：

- RS485硬件自动DE；
- 当前代码配置为115200 baud；
- 8数据位；
- 无校验；
- 1停止位。

如果实物固件使用不同波特率，应在 CubeMX 中修改对应 USART 配置，而不是在电机模块中动态修改外设参数。

## 4. 固定控制模式

电机控制模式应提前使用官方上位机写入。`DDTMotorInit()`只声明该实例预期使用的模式，后续不可更改，也不会向电机发送`0xA0`
模式切换帧：

| 模式  | 注册枚举                      | 官方模式编号 | 控制范围                  |
|-----|---------------------------|-------:|-----------------------|
| 电流环 | `DDT_MOTOR_MODE_CURRENT`  | `0x01` | -8～8 A映射到-32767～32767 |
| 速度环 | `DDT_MOTOR_MODE_SPEED`    | `0x02` | -330～330 rpm          |
| 位置环 | `DDT_MOTOR_MODE_POSITION` | `0x03` | 0～360°映射到0～32767      |

方向反转配置只作用于有符号的电流和速度目标。官方没有定义位置坐标反向映射，因此位置模式注册为反向时会直接失败，位置坐标变换应由应用层明确完成。

应用层必须调用与注册模式对应的接口：电流实例使用`DDTMotorSetCurrent()`，速度实例使用`DDTMotorSetSpeed()`，位置实例使用
`DDTMotorSetPosition()`。模式专用接口会检查实例注册模式。

反馈DATA[1]会与注册模式比较。如果上位机实际模式与实例配置不一致，驱动设置`mode_mismatch`并立即锁定后续输出，必须修正上位机配置或实例注册配置后重新初始化。

## 5. 普通驱动帧

普通控制帧固定10字节：

|      字节 | 内容                       |
|--------:|--------------------------|
| DATA[0] | 电机ID                     |
| DATA[1] | `0x64`                   |
| DATA[2] | 目标值高8位                   |
| DATA[3] | 目标值低8位                   |
| DATA[4] | `0x00`                   |
| DATA[5] | `0x00`                   |
| DATA[6] | 加速时间                     |
| DATA[7] | 速度模式刹车：`0xFF`；不刹车：`0x00` |
| DATA[8] | `0x00`                   |
| DATA[9] | 前9字节的CRC-8/MAXIM         |

驱动内部实现了独立的CRC-8/MAXIM。当前工程公共`crc_8()`使用另一种CRC表，不能用于M0601C。实现已经用官方示例验证：

```text
01 64 00 00 00 00 00 00 00 → CRC 50
01 64 00 1E 00 00 00 00 00 → CRC 18
C8 64 00 00 00 00 00 00 00 → CRC DE
```

## 6. 启停语义

M0601C官方协议没有统一的使能/失能命令：

- `DDTMotorEnable()`要求实例已经设置过安全目标，然后按注册模式发送该目标；
- 电流模式停止时发送0电流；
- 速度模式停止时发送0 rpm，并把刹车字段写为`0xFF`；
- 位置模式没有官方通用停止或刹车指令，`DDTMotorStop()`返回`HAL_ERROR`，不会伪造停止语义。

位置模式的急停和失能必须由系统电源、驱动器硬件使能端或后续取得的官方指令实现。

## 7. 发送策略

`refresh_period_ms`在注册时配置：

- `0`：只有目标改变、使能或停止时发送；
- 大于`0`：使能状态下按指定周期重新发送当前目标。

RS485 115200 baud发送一帧10字节需要接近1 ms。多个电机共用总线时，不应为每台电机盲目配置1 ms刷新，否则总线无法承载。四台电机通常可从10～20
ms刷新周期开始评估。

## 8. 接收与反馈解析

### 8.1 普通驱动反馈

README与官方`usart.c`明确普通`0x64`驱动指令发送后返回10字节：

|        字节 | 解析内容                          |
|----------:|-------------------------------|
|   DATA[0] | 设备标识，驱动在无在途请求时尝试按该字节匹配实例      |
|   DATA[1] | 当前模式                          |
| DATA[2:3] | 电流原始值，大端有符号16位                |
| DATA[4:5] | 速度原始值，大端有符号16位，单位rpm          |
| DATA[6:7] | 位置原始值，大端无符号16位                |
|   DATA[8] | 错误码                           |
|   DATA[9] | 保留在原始反馈中，并计算前9字节的MAXIM校验结果供诊断 |

换算表给出的公式为：

```text
current_A = signed_current_raw × 8 / 32767
speed_rpm = signed_speed_raw
position_deg = unsigned_position_raw × 360 / 32768
```

换算表中电流公式使用`raw < 32767`判断符号，这会把合法正满量程`0x7FFF`误判为负数。驱动按照PDF中`-32767～32767`对应`-8～8 A`
的定义，采用标准`int16_t`解释，正确保留`0x7FFF = +8 A`。

### 8.2 `0x74`状态反馈

`0x74`查询回复与普通反馈的主要区别是温度字段：

|        字节 | 解析内容     |
|----------:|----------|
|   DATA[1] | 当前模式     |
| DATA[2:3] | 电流原始值    |
| DATA[4:5] | 速度原始值    |
|   DATA[6] | 电机温度，单位℃ |
|   DATA[7] | 单字节位置原始值 |
|   DATA[8] | 错误码      |

官方换算表只给出了16位位置换算公式，没有给出状态反馈DATA[7]的单字节位置公式。因此驱动把它保存到`status_position_raw`
，但不会用它覆盖普通反馈得到的`position_raw/position_deg`。

### 8.3 接收接口

- `DDTMotorGetMeasure()`：读取解析后的电流、速度、位置、温度、模式和错误码；
- `DDTMotorGetRawFeedback()`：读取最近一次完整原始回复和CRC诊断结果；
- `DDTMotorIsOnline()`：根据最近一次有效长度反馈的时间判断在线状态；
- 上电反馈`AA 55 FF`由总线识别并统计，不作为某个已注册实例的控制反馈。

## 9. ID设置与查询

### 9.1 设置ID

```c
DDTMotorBusSetDeviceId(bus, 1U);
```

驱动会由统一控制任务连续发送5次：

```text
AA 55 53 01 00 00 00 00 00 00
```

安全限制：

- 调用时总线上不能已经注册电机实例；
- 实物总线上只能连接一台电机；
- 官方规定每次上电只允许设置一次。

### 9.2 查询ID

```c
DDTMotorBusRequestDeviceId(bus);
```

发送：

```text
C8 64 00 00 00 00 00 00 00 DE
```

查询的原始回复通过`DDTMotorBusGetRawFeedback()`读取。

## 10. 使用示例

### 10.1 初始化总线

```c
DDTMotorBusInitConfig_t bus_config = {
    .usart_handle = &huart3,
    .transfer_mode = USART_TRANSFER_DMA,
    .response_timeout_ms = 20U,
    .inter_frame_interval_ms = 10U,
};

DDTMotorBus *ddt_bus = DDTMotorBusInit(&bus_config);
```

### 10.2 注册速度模式电机

```c
DDTMotorInitConfig_t motor_config = {
    .bus = ddt_bus,
    .device_id = 1U,
    .control_mode = DDT_MOTOR_MODE_SPEED,
    .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
    .acceleration_time = 0U,
    .refresh_period_ms = 20U,
};

DDTMotorInstance *motor = DDTMotorInit(&motor_config);
if (motor != NULL)
{
    DDTMotorSetSpeed(motor, 30.0f, 0U);
    DDTMotorEnable(motor);
}
```

### 10.3 请求状态反馈

```c
if (DDTMotorRequestStatus(motor) == HAL_OK)
{
    /* 回复异步到达，由 USARTProcess() 调用驱动接收回调保存。 */
}

const DDTMotorRawFeedback_t *feedback = DDTMotorGetRawFeedback(motor);
const DDTMotorMeasure_t *measure = DDTMotorGetMeasure(motor);
```

## 11. 文件与集成

- `ddt_motor.c`：协议、总线调度和收发实现；
- `ddt_motor.h`：公开类型与接口；
- `motor_task.c`：统一调用`DDTMotorControl()`；
- `CMakeLists.txt`：加入源码和头文件搜索路径。
