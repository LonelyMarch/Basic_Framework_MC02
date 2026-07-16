# HTM35xx/HTM43xx RS485 电机驱动

## 适用范围

本驱动对应 `doc/HT/RS485通信协议_V2.x.pdf`，适用于使用 `0x3E/0x3C + 设备地址 + 命令码 + CRC16_MODBUS` 协议的 HTM35xx/HTM43xx。

它与HT04的MIT/Cheetah协议完全独立。当前新增实现使用RS485；HTM系列的CAN命令码协议可在后续作为另一种传输实现加入，但不应与HT04共用报文解析。

## BSP依赖

本驱动直接使用现有 `bsp_usart`：

- USART硬件自动控制RS485 DE；
- ReceiveToIdle DMA/IT接收；
- 阻塞、IT或DMA发送；
- BSP任务上下文回调。

BSP只负责字节收发。协议头、长度、CRC、地址分发和请求-应答状态机全部位于本模块。

USART2和USART3虽然已配置RS485 DE，但工程生成代码当前波特率为10 Mbps。使用前必须在CubeMX中改为电机支持的波特率，默认是115200、8N1。

## 总线架构

同一条RS485总线只注册一个 `USARTInstance`，多台电机通过设备地址挂在总线管理对象下：

```text
HTMRS485Bus
    ├─ address 1 -> HTMMotorInstance
    ├─ address 2 -> HTMMotorInstance
    └─ address N -> HTMMotorInstance
```

每条总线同一时间只允许一个在途请求：

```text
选择电机
  └─ 发送请求
       └─ 等待相同包序号、地址和命令码的回复
            ├─ 收到合法回复：解析、释放总线
            └─ 超时：释放总线，等待下一轮
```

接收端使用流式缓存，不假设一次USART IDLE回调就是一帧，因此可以处理拆包、粘包和前导错误字节。

## 初始化总线

```c
HTMRS485Bus_Init_Config_s bus_config = {
    .usart_handle = &huart2,
    .transfer_mode = USART_TRANSFER_DMA,
    .response_timeout_ms = 20,
};

HTMRS485Bus *bus = HTMRS485BusInit(&bus_config);
```

当前最多支持2条HTM RS485总线，每条总线最多16个已注册实例。协议地址范围为1～32，同一总线上不允许重复地址。

## 注册电机与固定模式

控制模式在注册时确定，后续不可更改：

```c
HTMMotor_Init_Config_s motor_config = {
    .bus = bus,
    .device_address = 1,
    .control_mode = HTM_CONTROL_SPEED,
    .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
};

HTMMotorInstance *motor = HTMMotorInit(&motor_config);
```

支持的固定模式及对应协议命令：

| 注册模式 | 控制接口 | 命令码 |
|---|---|---:|
| `HTM_CONTROL_OPEN_LOOP` | `HTMMotorSetOpenLoop()` | `0x53` |
| `HTM_CONTROL_SPEED` | `HTMMotorSetSpeed()` | `0x54` |
| `HTM_CONTROL_ABSOLUTE_POSITION` | `HTMMotorSetAbsolutePosition()` | `0x55` |
| `HTM_CONTROL_RELATIVE_POSITION` | `HTMMotorSetRelativePosition()` | `0x56` |

调用与注册模式不匹配的控制接口会返回 `HAL_ERROR`。

## 控制管理

`MotorControlTask()`约每1 ms调用 `HTMRS485Control()`。它不会按1 kHz向每台电机盲目发送控制帧，而是：

1. 处理每条总线的回复等待和超时；
2. 优先提交实例排队的控制/停止命令；
3. 没有控制命令时轮询 `0x2F` 实时位置和速度；
4. 收到回复后才调度下一台电机。

这种方式符合半双工RS485的请求-应答约束。实际每台电机的反馈频率取决于波特率、电机数量和回复时延，不等于任务调用频率。

## 控制接口

```c
HAL_StatusTypeDef HTMMotorEnable(HTMMotorInstance *motor);
HAL_StatusTypeDef HTMMotorStop(HTMMotorInstance *motor);
HAL_StatusTypeDef HTMMotorSetOpenLoop(HTMMotorInstance *motor, int16_t power);
HAL_StatusTypeDef HTMMotorSetSpeed(HTMMotorInstance *motor, float speed_rpm);
HAL_StatusTypeDef HTMMotorSetAbsolutePosition(HTMMotorInstance *motor, float position_deg);
HAL_StatusTypeDef HTMMotorSetRelativePosition(HTMMotorInstance *motor, float delta_deg);
HAL_StatusTypeDef HTMMotorSetCurrentPositionAsZero(HTMMotorInstance *motor);
HAL_StatusTypeDef HTMMotorClearFault(HTMMotorInstance *motor);
```

- `HTMMotorEnable()`只解除模块的软件输出锁；真正进入何种运行状态由下一条固定模式控制命令决定。
- `HTMMotorStop()`会清除尚未提交的普通命令并优先排队 `0x50`关闭命令。
- 每个实例只保存一个待发命令；已有命令尚未提交时，普通新命令返回 `HAL_BUSY`。
- 设置当前位置为原点后，协议规定电机会进入关闭状态，因此模块同时清除软件使能。

绝对位置命令的协议数据类型是 `uint32_t`。当前接口只接受非负角度，并且绝对位置模式不支持通过 `motor_reverse_flag`生成负物理目标；需要反向绝对坐标时应在上层建立明确的零点和正方向映射。

## 反馈数据

`0x2F`以及控制命令回复用于更新：

- 单圈编码器Count；
- 多圈Count；
- 机械速度，单位0.1 RPM；
- 换算后的角度和RPM。

`0x40/0x41`回复格式已支持解析电压、电流、温度、故障码和运行状态；当前周期管理默认轮询 `0x2F`，状态读取可按应用需求继续增加公开请求接口。

合法回复通过CRC16_MODBUS校验，并核对包序号、设备地址和命令码后才会结束在途请求。

## 离线行为

每个HTM实例使用daemon监测合法回复。离线后：

- 只记录一次日志；
- 清除软件使能；
- 丢弃尚未提交的控制命令；
- 不自动恢复旧目标。

反馈恢复后仍保持禁止输出，必须由上层重新调用 `HTMMotorEnable()`并下发新目标。
