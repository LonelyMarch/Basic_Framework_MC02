# Servo Motor 模块

舵机控制模块，支持两种舵机类型：串行总线舵机 (Bus_Servo) 和传统 PWM 舵机 (PWM_Servo)。

## 整体架构

    ServoInit()
      |
      +-- Bus_Servo  --> USARTRegister() --> 注册总线舵机,挂载 DecodeServo 回调
      |
      +-- PWM_Servo  --> PWMRegister()   --> 注册 PWM 舵机

    ServoSetAngle()
      |
      +-- Bus_Servo  --> 栈上构建协议帧 --> USARTSend() DMA 发送
      |
      +-- PWM_Servo  --> 角度 -> 占空比换算 --> PWMSetDutyRatio()

    USART 收到数据 --> DecodeServo() --> 从帧内 ID 定位实例 --> 更新 recv_angle

## 关键限制

- 实例使用静态池分配，不依赖 malloc()。
- Bus_Servo 的所有实例共用一条 UART，一帧数据通过帧内 ID 字段路由到正确的舵机。
- 总线舵机协议帧在栈上临时构建，避免全局缓冲区 FreeRTOS 多任务冲突。
- 本模块暂未与 Daemon 集成，离线检测由上层负责。

## 常量定义

| 宏                   | 值      | 说明                                 |
|---------------------|--------|------------------------------------|
| SERVO_MOTOR_CNT     | 7      | 最大舵机实例数                            |
| Servo_Frame_First   | 0x55   | 总线舵机帧头字节 1                         |
| Servo_Frame_Second  | 0x55   | 总线舵机帧头字节 2                         |
| Servo_MAX_BUFF      | 10     | 总线舵机最大接收帧长度                        |
| SERVO_MOVE_CMD      | 0x03   | 移动命令字                              |
| SERVO_UNLOAD_CMD    | 0x14   | 卸载 / 释放扭矩命令字                       |
| SERVO_POS_READ_CMD  | 0x15   | 读取位置命令字                            |
| PWM_SERVO_ANGLE_MIN | 0.0f   | PWM 舵机最小角度 (度)                     |
| PWM_SERVO_ANGLE_MAX | 180.0f | PWM 舵机最大角度 (度)                     |
| PWM_SERVO_DUTY_MIN  | 0.025f | PWM 舵机最小占空比 (2.5% = 0.5ms @ 50Hz)  |
| PWM_SERVO_DUTY_MAX  | 0.125f | PWM 舵机最大占空比 (12.5% = 2.5ms @ 50Hz) |

## 类型定义

### ServoType_e

```c
typedef enum {
    Servo_None_Type = 0,
    Bus_Servo = 1,     // 串行总线舵机
    PWM_Servo = 2,     // 传统 PWM 舵机
} ServoType_e;
```

### Servo_Init_Config_s -- 初始化配置

| 字段              | 类型                   | 说明                     |
|-----------------|----------------------|------------------------|
| pwm_init_config | PWM_Init_Config_s    | PWM 舵机需要,PWM BSP 初始化参数 |
| servo_type      | ServoType_e          | 舵机类型                   |
| _handle         | UART_HandleTypeDef * | 总线舵机需要,对应的 UART HAL 句柄 |
| servo_id        | uint8_t              | 总线舵机 ID (索引到实例数组)      |

### ServoInstance -- 舵机实例

| 字段             | 类型              | 说明                 |
|----------------|-----------------|--------------------|
| servo_id       | uint8_t         | 总线舵机 ID            |
| angle          | float           | 当前目标角度             |
| recv_angle     | uint16_t        | 最近一次读取的角度 (总线舵机)   |
| pwm_instance   | PWMInstance *   | PWM 舵机的 BSP PWM 实例 |
| usart_instance | USARTInstance * | 总线舵机的 BSP USART 实例 |
| servo_type     | ServoType_e     | 舵机类型               |

## 接口清单

| 接口                               | 说明                                                   |
|----------------------------------|------------------------------------------------------|
| ServoInstance *ServoInit(config) | 注册舵机实例。返回实例指针,失败返回 NULL                              |
| void ServoSetAngle(servo, angle) | 设置舵机目标角度。Bus_Servo 构建协议帧并 DMA 发送,PWM_Servo 换算为占空比后设置 |

## 典型用法

### 总线舵机

```c
Servo_Init_Config_s config = {
    .servo_type = Bus_Servo,
    ._handle = &huart1,
    .servo_id = 1,
};
ServoInstance *servo = ServoInit(&config);
ServoSetAngle(servo, 90.0f);  // 转到 90 度
```

### PWM 舵机

```c
PWM_Init_Config_s pwm_cfg = {
    .htim = &htim5,
    .channel = TIM_CHANNEL_1,
    .period = 0.02f,       // 50Hz
    .dutyratio = 0.075f,   // 初始 90 度
};
Servo_Init_Config_s config = {
    .servo_type = PWM_Servo,
    .pwm_init_config = pwm_cfg,
};
ServoInstance *servo = ServoInit(&config);
ServoSetAngle(servo, 45.0f);  // 线性换算: duty = 0.025 + 45/180*(0.125-0.025) = 0.05
```

## PWM 角度换算

PWM 舵机的角度到占空比换算公式：

    duty = DUTY_MIN + (angle - ANGLE_MIN) / (ANGLE_MAX - ANGLE_MIN) * (DUTY_MAX - DUTY_MIN)

默认参数适配 SG90 等 50Hz / 0.5~2.5ms 脉宽的舵机。若使用其他型号，修改 PWM_SERVO_ANGLE_* 和 PWM_SERVO_DUTY_* 四个宏即可。

## 总线舵机协议帧格式

### 写入角度 (CMD=0x03)

| 字节 | 0    | 1    | 2   | 3    | 4  | 5       | 6       | 7      | 8      | 9     |
|----|------|------|-----|------|----|---------|---------|--------|--------|-------|
| 值  | 0x55 | 0x55 | LEN | 0x03 | ID | ANGLE_L | ANGLE_H | TIME_L | TIME_H | CKSUM |

- ANGLE: uint16_t 小端序
- TIME: 默认 0x0320 (800ms)
- CKSUM: ~(LEN + CMD + ID + ANGLE_L + ANGLE_H + TIME_L + TIME_H) & 0xFF

### 读取角度回复 (CMD=0x15)

| 字节 | 0    | 1    | 2   | 3    | 4  | 5        | 6       | 7       |
|----|------|------|-----|------|----|----------|---------|---------|
| 值  | 0x55 | 0x55 | LEN | 0x15 | ID | RESERVED | ANGLE_L | ANGLE_H |

DecodeServo() 从 DATA[4] 提取 ID 定位实例,从 DATA[6:7] 提取小端 uint16 角度。

## 文件清单

| 文件             | 说明                       |
|----------------|--------------------------|
| servo_motor.h  | 全量接口、类型定义                |
| servo_motor.c  | 实现:协议组帧、角度换算、反馈解码、BSP 注册 |
| servo_motor.md | 本文档                      |