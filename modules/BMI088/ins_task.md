# ins_task

`modules/BMI088/ins_task.c` 是板载 BMI088 的姿态解算与温控部分。它直接使用同目录下的 BMI088 芯片驱动获取传感器数据，并使用 `modules/algorithm` 中的四元数
EKF 进行姿态融合，并通过 `bsp/pwm` 控制 BMI088 加热 PWM。

本模块不直接操作 BMI088 寄存器，也不直接管理 SPI 片选。BMI088 芯片级读写由 `modules/BMI088` 完成，SPI、GPIO、PWM 等硬件访问由
BSP 层统一封装。芯片驱动、板载传感器温控和姿态解算现在共同归属于 `modules/BMI088`，不再额外设置一层仅服务于 BMI088 的通用 `imu` 模块。

## 对外数据

`INS_Init()` 返回 `attitude_t *`，指向模块内部的姿态视图：

```c
typedef struct
{
    float Gyro[3];
    float Accel[3];
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
} attitude_t;
```

字段含义：

- `Gyro[3]`：修正后的三轴角速度，来自 BMI088，单位为 `rad/s`。
- `Accel[3]`：修正后的三轴加速度，来自 BMI088，单位为 `m/s^2`。
- `Roll/Pitch/Yaw`：四元数 EKF 输出的欧拉角。
- `YawTotalAngle`：yaw 多圈累计角，便于云台多圈控制。

application 层持有该指针，用作云台或底盘控制的姿态反馈来源。

## 初始化流程

`INS_Init()` 通常在 application 层 `RobotInit()` 阶段调用，当前流程为：

1. 注册 `TIM3 CH4` PWM，作为 BMI088 恒温加热输出。
2. 调用 `INS_BMI088_Init()` 注册并初始化板载 BMI088。
3. BMI088 初始化阶段执行在线标定，失败时自动加载离线参数。
4. 设置 IMU 安装误差和三轴比例修正的默认参数。
5. 临时使用周期读取分支采集多帧加速度，估计初始 roll/pitch 四元数。
6. 初始化 `IMU_QuaternionEKF`。
7. 初始化温控 PID。
8. 初始化 DWT 时间戳，用于后续计算 EKF 的 `dt`。

`INS_Init()` 带防重复初始化保护。若已经初始化过，再次调用会直接返回已有的 `attitude_t` 指针。

初始化阶段有两个临时切换：

- BMI088 初始化完成后，会临时把 `ins_bmi088->work_mode` 改为 `BMI088_BLOCK_PERIODIC_MODE` 主动采一帧。
- 初始四元数估计前，也会临时使用周期读取分支。

这两个切换只影响 `BMI088Acquire()` 的软件读取分支，不会重新配置 BMI088 的 DRDY 寄存器。真正的硬件 DRDY 配置由
`BMI088Register()` 初始化时的 `INS_BMI088_WORK_MODE` 决定。

## BMI088 工作模式

当前默认：

```c
#define INS_BMI088_WORK_MODE BMI088_BLOCK_PERIODIC_MODE
```

### 周期模式

周期模式下，`INS_Task()` 以约 `1kHz` 周期主动调用 `BMI088Acquire()`。BMI088 不输出 DRDY 中断，采样节奏由 INS 任务决定。

这是当前稳定主路径，适合不希望依赖 EXTI 的配置。

### 触发模式

若要启用触发模式，可把 `INS_BMI088_WORK_MODE` 改为：

```c
#define INS_BMI088_WORK_MODE BMI088_BLOCK_TRIGGER_MODE
```

触发模式下，BMI088 会配置 `ACC_INT/GYRO_INT` 的 DRDY 输出。EXTI 回调只记录 DRDY 事件，实际 SPI 读取仍在 `INS_Task()` 中进行。

触发模式需要 CubeMX 开启 `EXTI line[15:10] interrupts`，当前建议优先级为 `5,0`。如果关闭 `EXTI15_10_IRQn`，触发模式下
`INS_Task()` 会长期等不到完整帧，最终进入 BMI088 离线保护。

触发模式并不是硬件级严格同步采样。当前逻辑要求 acc 和 gyro 都有未处理 DRDY 后才读取，并在读取前后检查 DRDY
计数，尽量避免发布跨采样窗口的数据。

## 运行流程

`INS_Task()` 应以约 `1kHz` 运行。当前每次调用的主流程为：

1. 判断触发模式下是否已有完整 acc+gyro DRDY 帧。
2. 调用 `BMI088Acquire()` 获取一帧 IMU 数据。
3. 若读取失败：
    - 周期模式下记录失败并跳过本周期 EKF。
    - 触发模式下若只是完整帧未到齐，等待下一周期；连续等待超过阈值后进入离线保护。
4. 若读取成功：
    - 清零 BMI088 失败计数和等待帧计数。
    - 使用 `DWT_GetDeltaT()` 计算 `dt`，并限幅到 `0.0001s ~ 0.005s`。
    - 更新 `INS.attitude.Accel[]` 和 `INS.attitude.Gyro[]`。
    - 执行安装误差和三轴比例修正。
    - 调用 `IMU_QuaternionEKF_Update()` 融合姿态。
    - 更新机体系/导航系基向量和运动加速度。
    - 更新 `Yaw/Pitch/Roll/YawTotalAngle`，由 application 层按需要读取姿态快照。
5. 每 `INS_TEMP_CTRL_PERIOD_COUNT` 次调用执行一次温控，当前约 `10Hz`。

若 BMI088 采样从离线状态恢复，模块会清零失败计数并恢复温控输出。

## 温度控制

BMI088 温控由 `IMU_Temperature_Ctrl()` 完成：

- 目标温度 `RefTemp = 40`。
- 温控 PID 输出原本按 `CCR` 计数值语义设计。
- `IMUPWMSet()` 会把 PID 输出换算成 `bsp_pwm` 接口使用的 `0.0f ~ 1.0f` 占空比。
- PWM 使用 `TIM3 CH4`，周期为 `0.001s`。

温控运行频率为约 `10Hz`。BMI088 温度寄存器由 `modules/BMI088` 约 `1Hz` 降频读取，其余温控周期使用最近一次有效温度。

如果 BMI088 连续读取失败达到 `INS_BMI088_OFFLINE_FAIL_COUNT`，模块会：

1. 关闭加热 PWM。
2. 清空 PID 输出、积分和历史项。
3. 限频打印离线日志。

这样可以避免传感器长时间离线时继续使用旧温度加热。

## 初始四元数

`InitQuaternion()` 使用加速度计估计初始 roll/pitch，yaw 初值置为 `0`。当前策略为：

1. 最多尝试 `INS_INIT_ACC_MAX_ATTEMPT` 次。
2. 累计 `INS_INIT_ACC_SAMPLE_COUNT` 次成功加速度采样。
3. 对加速度均值归一化。
4. 计算当前重力方向到导航系重力方向的旋转轴和旋转角。
5. 生成初始四元数。

若初始化阶段始终无法获得有效加速度，模块会使用单位四元数作为兜底值，并打印错误日志。后续 BMI088 恢复采样后，EKF 仍可继续运行。

## 安装误差与比例修正

`IMU_Param_Correction()` 用于修正 IMU 安装误差和三轴比例误差。默认参数不改变数据：

- `gyro_scale[3] = {1, 1, 1}`
- `accel_scale[3] = {1, 1, 1}`
- `Yaw/Pitch/Roll = 0`

运行期修正顺序为：

1. 陀螺仪使用 `gyro_scale[]` 做三轴比例修正。
2. 加速度计使用 `accel_scale[]` 做三轴比例修正。
3. 两者共用同一套安装角旋转矩阵，修正到机体系。

安装角单位为 `deg`。当 `Yaw/Pitch/Roll` 或 `flag` 改变时，模块会重新计算旋转矩阵；否则复用上一次矩阵，减少 1kHz 路径中的三角函数开销。

后续如果完成六面标定或整机安装标定，可以把标定结果写入 `IMU_Param_t`。

## 坐标变换

本模块提供两个三维向量坐标变换函数：

```c
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);
```

`INS_Task()` 使用它们完成：

- 机体系基向量到导航系的转换。
- 导航系重力向量到机体系的转换。
- 运动加速度从机体系到导航系的转换。

其中运动加速度会经过一阶低通滤波，滤波系数为 `INS.AccelLPF`。

## FreeRTOS 约定

- `INS_Init()` 在任务启动前调用，初始化和标定过程可以阻塞。
- `INS_Task()` 是高频任务，不应加入长阻塞、动态内存分配或大量日志输出。
- BMI088 SPI 读取在任务上下文中进行，不在 EXTI 中断中访问 SPI。
- 触发模式下 `BMI088Acquire()` 返回 `0` 可能只是完整帧尚未到齐，上层通过等待计数区分正常等待和离线异常。
- EKF 的 `dt` 来自 DWT，并进行上下限保护，避免调度抖动或异常暂停造成滤波步长过大。
- 温控使用最近一次有效温度，BMI088 长时间离线时会主动关闭加热。

## 相关模块

- `modules/BMI088`：BMI088 芯片驱动、数据解码、标定。
- `modules/algorithm/QuaternionEKF`：姿态 EKF。
- `modules/algorithm/controller`：温控 PID。
- `bsp/spi`：BMI088 SPI 总线和片选控制。
- `bsp/pwm`：BMI088 加热 PWM。
- `bsp/dwt`：初始化延时和 EKF 时间间隔。
