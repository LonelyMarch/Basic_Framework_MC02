# DMimu

## 1. 模块定位

`modules/DMimu` 是外部达妙科技 DM-IMU-L1 六轴 IMU 的通信驱动。它与板载 `modules/BMI088` 完全独立：

- `modules/BMI088` 直接通过 SPI 读取板载 BMI088，并在 STM32 内完成温控和四元数 EKF。
- `modules/DMimu` 通过 CAN 或 RS485 接收外部 DM-IMU-L1 已经输出的加速度、角速度、欧拉角和四元数，不重复运行 EKF。

协议依据：

- `doc/dm-imu-master/说明书/达妙科技DM-IMU-L1六轴IMU模块使用说明书V1.2.pdf`
- `doc/dm-imu-master/示例/MC02_CAN收发例程`
- `doc/dm-imu-master/示例/MC02_485接收例程`

当 PDF 与例程存在差异时，以 PDF V1.2 为准。

## 2. 架构

```text
application
    │
    ├── DMIMURegister()：注册时固定CAN或RS485
    ├── DMIMURequestData()：请求模式按需请求
    ├── DMIMUExecuteCommand()：校准、置零、保存等
    └── DMIMUGetMeasure()：读取公共测量快照
            ▲
            │
    modules/DMimu
       ├── CAN 8字节映射值解析
       └── RS485主动浮点帧/应答帧流解析
            ▲
            │
       bsp/can 或 bsp/usart
```

模块不创建任务：

- CAN中断只把报文放入BSP队列，`CANProcessTask()`在任务上下文调用DM-IMU回调。
- USART DMA/IDLE中断只保存接收片段，BSP服务任务中的`USARTProcess()`调用RS485解析回调。
- 主动模式下外部IMU自行周期发送，不需要DMimu控制任务。
- 请求模式由application按自己的调度周期调用一次`DMIMURequestData()`，驱动不擅自轮询。

## 3. 注册时固定通信接口

每个实例必须在注册配置中选择一种接口：

```c
typedef enum
{
    DM_IMU_TRANSPORT_CAN = 0,
    DM_IMU_TRANSPORT_RS485,
} DMIMUTransport_e;
```

实例注册成功后，通信接口以及对应的CAN ID或RS485从机ID不会改变。驱动刻意不提供以下运行期接口：

- 修改通信类型。
- 修改CAN_ID或MST_ID。
- 修改CAN/RS485波特率。

这些参数若在运行期修改，会导致已经注册到BSP的CAN过滤器、接收ID或UART配置失效。需要修改时，应使用官方上位机设置并重启系统，再按新参数注册实例。

## 4. CAN实例注册

```c
DMIMU_Init_Config_s config = {
    .transport = DM_IMU_TRANSPORT_CAN,
    .online_timeout_ms = 100U,
    .communication.can = {
        .can_handle = &hfdcan3,
        .can_id = 0x01U,
        .mst_id = 0x11U,
        .transmit_timeout_ms = 1.0F,
    },
};

DMIMUInstance *external_imu = DMIMURegister(&config);
```

CAN协议使用标准帧、DLC 8：

- STM32请求ID：注册配置中的`can_id`。
- IMU反馈ID：注册配置中的`mst_id`。
- 默认波特率：官方文档为1 Mbps，实际外设参数由CubeMX配置。

PDF附录三仍保留了一个旧版`StdId = 0x6FF、DLC = 4`请求片段，它与V1.2正文第8~9页更新后的`CAN_ID、DLC = 8、CC RID 读/写 DD 数据`
协议冲突。本驱动按V1.2正文和新版官方`DM-IMU/dm_imu.c`实现，不采用附录三旧请求格式。

## 5. RS485实例注册

```c
DMIMU_Init_Config_s config = {
    .transport = DM_IMU_TRANSPORT_RS485,
    .online_timeout_ms = 100U,
    .communication.rs485 = {
        .uart_handle = &huart2,
        .slave_id = 0x01U,
        .transfer_mode = USART_TRANSFER_DMA,
    },
};

DMIMUInstance *external_imu = DMIMURegister(&config);
```

RS485方向控制由CubeMX配置的UART硬件DE完成，DMimu不会直接操作DE GPIO。串口波特率、数据位和停止位同样由CubeMX配置。

驱动最多支持两个RS485 DM-IMU实例，这是因为当前`bsp/usart`模块回调不携带上下文参数，驱动使用两个固定回调入口把接收片段映射到实例。

## 6. 公共测量数据

CAN和RS485最终都更新同一个`DMIMUMeasure_s`：

```c
DMIMUMeasure_s measure;

if (DMIMUGetMeasure(external_imu, &measure) != 0U)
{
    float ax = measure.accel_mps2[0];
    float wz = measure.gyro_rad_s[2];
    float yaw = measure.yaw_deg;
    float qw = measure.quaternion[0];
}
```

统一单位：

| 字段                           | 单位/含义          |
|------------------------------|----------------|
| `accel_mps2[3]`              | m/s²           |
| `gyro_rad_s[3]`              | rad/s          |
| `roll_deg/pitch_deg/yaw_deg` | degree         |
| `quaternion[4]`              | w、x、y、z        |
| `temperature_c`              | 摄氏度，仅CAN加速度帧携带 |

`valid_mask`表示哪些类型至少收到过一次，四类数据还分别带有更新时间和更新计数。读取测量数据不会清除任何全局“updated”标志，因此多个上层消费者不会互相影响。

## 7. CAN数据解析

### 7.1 加速度

PDF第10页规定：

```text
DATA[0] = 0x01
DATA[1] = 温度
DATA[2:3] = Acc_X，小端16位映射值
DATA[4:5] = Acc_Y
DATA[6:7] = Acc_Z
```

映射范围来自PDF附录二：`-235.2 ~ 235.2 m/s²`。

官方CAN例程头文件给出温度范围`0 ~ 60°C`，驱动同时保留`temperature_raw`，并按8位线性映射生成`temperature_c`。

### 7.2 角速度

`DATA[0] = 0x02`，三轴均为16位小端映射值，范围`-34.88 ~ 34.88 rad/s`，对应约`±2000°/s`。

### 7.3 欧拉角

CAN帧顺序是：

```text
Pitch、Yaw、Roll
```

映射范围：

- Pitch：`-90 ~ 90°`
- Yaw：`-180 ~ 180°`
- Roll：`-180 ~ 180°`

驱动解析后统一写入命名明确的`roll_deg/pitch_deg/yaw_deg`字段。

### 7.4 四元数

四元数由四个14位映射值连续打包在`DATA[1:7]`，范围均为`-1 ~ 1`。

PDF位域表规定`DATA[2]`的bit7~2是`W[5:0]`。官方例程使用`0xF8`掩码会遗漏bit2，因此本驱动按PDF使用`0xFC`提取完整六位。

## 8. RS485解析

### 8.1 主动模式

主动模式沿用USB浮点帧：

```text
55 AA | ID | type | float数据 | CRC16 | 0A
```

- 加速度、角速度、欧拉角：19字节。
- 四元数：23字节。
- float为IEEE754小端格式。
- 欧拉角顺序为Roll、Pitch、Yaw，与CAN顺序不同。

RS485接收采用流缓存扫描，支持拆包、粘包、任意数据类型组合和噪声重同步，不要求设备必须一次发送默认80字节完整组合包。

PDF附录四给出的CRC表使用多项式`0x1021`，但更新表达式采用`crc << 1`，不同于常见CCITT查表代码的`crc << 8`
。通过PDF内DM-Upper上位机截图中的真实十六进制帧核算，可以确认设备使用附录原样算法，并以CRC低字节在前发送。官方RS485例程自身没有执行CRC校验。驱动因此：

1. 首选PDF附录原样`crc << 1`算法、小端CRC字节序。
2. 兼容PDF算法结果的大端字节序。
3. 兼容标准CRC16-CCITT-FALSE的两种字节序。
4. 使用兼容路径时增加`crc_compatibility_count`，便于连接实物后确认固件实际格式。

### 8.2 应答模式

应答帧固定24字节：

```text
A5 | 类型 | ID | 寄存器 | 读/写 | 数据0~3（各4字节） | 应答 | 保留 | 5A
```

- 类型`0x0C`：指令域。
- 类型`0x0D`：数据域。
- 数据域寄存器`0~3`分别为加速度、角速度、欧拉角、四元数。
- 应答码`0~3`分别为成功、寄存器不存在、数据无效、操作失败。

合法数据应答会同时更新公共测量快照和最近寄存器应答。

## 9. 请求数据

设备处于请求模式时，上层按需要请求一种数据：

```c
DMIMURequestData(external_imu, DM_IMU_DATA_ACCEL);
DMIMURequestData(external_imu, DM_IMU_DATA_GYRO);
DMIMURequestData(external_imu, DM_IMU_DATA_EULER);
DMIMURequestData(external_imu, DM_IMU_DATA_QUATERNION);
```

CAN会发送官方8字节`CC RID 读/写 DD 数据`请求。RS485会发送官方24字节`A5...5A`数据域请求。

驱动不自动连续发送四种请求。特别是RS485半双工链路应保持一问一答，上层应等待本次应答或超时后再请求下一项。

## 10. 公共命令

```c
DMIMUExecuteCommand(external_imu, DM_IMU_COMMAND_ZERO_EULER, 0U);
DMIMUExecuteCommand(external_imu, DM_IMU_COMMAND_GYRO_CALIBRATION, 0U);
DMIMUExecuteCommand(external_imu, DM_IMU_COMMAND_ACCEL_CALIBRATION, 0U);
DMIMUExecuteCommand(external_imu, DM_IMU_COMMAND_SET_ACTIVE_MODE, 1U);
DMIMUExecuteCommand(external_imu, DM_IMU_COMMAND_SET_OUTPUT_INTERVAL, interval_raw);
DMIMUExecuteCommand(external_imu, DM_IMU_COMMAND_SAVE_PARAMETERS, 0U);
```

驱动会根据实例通信接口映射不同寄存器编号。例如陀螺仪校准在CAN寄存器中是`0x07`，在RS485指令域中是`0x03`。

注意事项：

- `SET_ACTIVE_MODE`参数只能是0或1。
- 输出间隔单位在PDF寄存器章节中没有明确说明，接口直接传递设备原始值。
- 重启等部分写命令可能没有反馈，符合PDF说明。
- 参数修改后需要执行`SAVE_PARAMETERS`才可保证掉电保存。

## 11. 在线与新鲜度

```c
if (DMIMUIsOnline(external_imu) != 0U)
{
    /* 最近收到过合法传感器帧或寄存器应答。 */
}

if (DMIMUIsDataFresh(external_imu, DM_IMU_DATA_EULER, 50U) != 0U)
{
    /* 欧拉角最近50ms内更新过。 */
}
```

在线状态和四类数据新鲜度分开判断。只收到加速度不能证明四元数仍然新鲜。

## 12. 统计与诊断

`DMIMUGetStatistics()`提供：

- 合法接收帧、传感器帧、寄存器应答计数。
- 非法长度、帧头、ID、浮点值和未知类型计数。
- CRC错误与CRC兼容路径计数。
- RS485流缓存溢出计数。
- 发送提交失败计数。

接收高频路径不直接打印日志，避免1kHz主动输出时影响实时性。
