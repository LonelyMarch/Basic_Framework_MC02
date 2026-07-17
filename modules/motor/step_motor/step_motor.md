# step_motor — 步进电机控制模块

## 概述

基于定时器 PWM 脉冲 + DIR 方向引脚的步进电机控制模块，适用于常规 **42 步进电机** + 外部驱动器（A4988 / TMC2209 / DM542 等）。

- 脉冲频率控制转速，50% 固定占空比保证脉宽稳定。
- 方向引脚控制转动方向。
- 可选 EN 引脚使能/失能驱动器。
- 支持**速度模式**和**位置模式**（梯形加减速 + 预计算减速阈值）。
- 所有速度逼近使用渐进式 `dv = accel × dt`，不会瞬时跳变脉冲频率。

## 硬件资源

| 资源          | 说明                    |
|-------------|-----------------------|
| TIM（PWM 通道） | 产生步进脉冲，频率可动态调整        |
| GPIO（DIR）   | 方向控制，低电平 CW / 高电平 CCW |
| GPIO（EN，可选） | 驱动器使能，低电平使能 / 高电平失能   |

## 典型参数（42 步进电机）

以 1.8° 步距角电机 + 16 微步驱动器为例：

| 参数              | 推荐值  | 说明                 |
|-----------------|------|--------------------|
| `steps_per_rev` | 3200 | 200 (1.8°) × 16 微步 |
| `max_speed_rps` | 5.0  | 最高转速 300 RPM       |
| `accel_rps2`    | 25.0 | 0 → 300 RPM 约 0.2s |

常用微步对应每转步数：

| 微步    | 每转步数 |
|-------|------|
| 1（整步） | 200  |
| 2     | 400  |
| 4     | 800  |
| 8     | 1600 |
| 16    | 3200 |
| 32    | 6400 |

## 工作模式

### 速度模式（SPEED）

调用 `StepMotorSetSpeed(motor, speed_rps)` 设置目标转速。电机按 `accel_rps2` 梯形加速到目标速度后匀速运行。转速为 0 且减速到位后
**自动切回 STOP**。

### 位置模式（POSITION）

调用 `StepMotorMoveRel()` 或 `StepMotorMoveAbs()` 设置目标位置。

`StepMotorMoveAbs()` 在设定目标时**预计算减速阈值**：

```
decel_threshold = (v_max² / (2·a)) × steps_per_rev
```

`StepMotorUpdate()` 每周期比较 `|remaining|` 与 `decel_threshold`：

- **加速/匀速段**：`remaining > decel_threshold`，向 `target_speed_rps` 梯形加速。
- **减速段**：`remaining ≤ decel_threshold`，目标速度按 `remaining / decel_threshold` 线性缩小，渐进式逼近。

减速段使用渐进式逼近而非直接赋值，防止脉冲频率突变导致步进电机失步。

### 停止（STOP）、使能/失能

- `StepMotorStop()`：立即关断脉冲。**`current_step` 不归零**，下次位置模式可从此位置继续。
- `StepMotorDisable()`：关断脉冲 + EN 拉高释放力矩。
- `StepMotorEnable()`：EN 拉低恢复力矩，但**不自动恢复之前的控制模式**，需调用者重新设定。

## 调用约束

- **同一实例的所有接口应在同一任务/中断上下文中调用**，模块不做任务间互斥保护。
- `StepMotorUpdate()` 需在高频任务中周期调用（推荐 1kHz），频率越高速度曲线越平滑。
- 初始化阶段（调度器启动前）即可调用 `StepMotorInit()`，底层 PWM/GPIO 注册不依赖 RTOS。

## API 参考

### StepMotorInit()

```c
StepMotorInstance *StepMotorInit(StepMotor_Init_Config_s *config);
```

注册步进电机实例，从静态池分配。`en_gpio_config.GPIO_Pin` 设为 `STEP_MOTOR_NO_EN_PIN` 可跳过 EN 引脚注册。失败返回 NULL 并打印
LOGERROR。

### StepMotorSetSpeed()

```c
void StepMotorSetSpeed(StepMotorInstance *motor, float speed_rps);
```

设置速度模式。正值 CW，负值 CCW。受 `max_speed_rps` 限幅。

### StepMotorMoveRel()

```c
void StepMotorMoveRel(StepMotorInstance *motor, int32_t steps, float speed_rps);
```

位置模式，相对移动 steps 步。内部委托给 `StepMotorMoveAbs()`。

### StepMotorMoveAbs()

```c
void StepMotorMoveAbs(StepMotorInstance *motor, int32_t step, float speed_rps);
```

位置模式，移动到绝对位置 step 步。内部预计算 `decel_threshold`。

### StepMotorUpdate()

```c
void StepMotorUpdate(StepMotorInstance *motor, float dt_s);
```

周期更新，必须在高频任务中调用。dt_s 为距上次调用的秒数。

### 直读/控制接口

| 函数                       | 说明               |
|--------------------------|------------------|
| `StepMotorStop()`        | 急停，脉冲关断，位置不归零    |
| `StepMotorDisable()`     | 失能 + 停止          |
| `StepMotorEnable()`      | 使能（不恢复控制模式）      |
| `StepMotorGetPosition()` | 读 current_step   |
| `StepMotorGetState()`    | 读 IDLE / RUNNING |

## 内部实现要点

### 减速阈值预计算

`StepMotorMoveAbs()` 调用时一次性计算从 `target_speed_rps` 减速到 0 需要的步数，存入 `decel_threshold`。
`StepMotorUpdate()` 每周期只做整数比较 `remaining_abs ≤ decel_threshold`，不做浮点运算。

### 渐进式速度逼近

所有速度变化通过 `StepMotorApproachSpeed()` 实现，每周期速度增量上限为 `accel × dt`，保证脉冲频率平滑过渡，不会瞬时跳变导致步进电机失步。

### 硬件交互

所有硬件操作均通过 BSP 层接口（`PWMRegister`、`PWMSetPeriod`、`PWMSetDutyRatio`、`GPIORegister`、`GPIOSet`、`GPIOReset`），无直接
HAL 调用或寄存器操作。

## 目录结构

```
step_motor/
    step_motor.h      // 类型定义与公共接口
    step_motor.c      // 实现
    step_motor.md     // 本说明
```
