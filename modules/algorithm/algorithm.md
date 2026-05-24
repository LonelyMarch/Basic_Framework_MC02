# algorithm

`modules/algorithm`是module层中的纯算法集合,供application层和其他module复用。该目录不直接操作BSP外设,不创建FreeRTOS任务,也不在中断中主动调度系统资源。调用者需要自己决定算法实例的生命周期、调用周期和并发保护方式。

## 模块边界

algorithm层只负责计算:

- 输入来自调用者传入的结构体、数组或标量。
- 输出通过返回值、输出参数或算法实例中的状态给出。
- 不直接读写CAN、SPI、IIC、USART、USB、Flash等外设。
- 不直接依赖application层状态。
- 不在算法内部打印日志。

需要注意的是,`kalman_filter`和`user_lib`中的矩阵工具会在初始化阶段使用`user_malloc`分配矩阵缓冲区。在FreeRTOS头文件已包含时,`user_malloc`映射到`pvPortMalloc`;否则映射到标准库`malloc`。这类对象应在初始化阶段创建,不要在高频控制循环中反复创建和释放。

## 文件组成

| 文件 | 作用 |
| --- | --- |
| `controller.h/.c` | PID控制器,支持积分限幅、变速积分、微分先行、输出滤波、微分滤波等改进项。 |
| `user_lib.h/.c` | 通用数学工具,包括限幅、角度格式化、三维向量运算、平均滤波、CMSIS-DSP矩阵封装和零初始化内存分配。 |
| `kalman_filter.h/.c` | 通用卡尔曼滤波框架,基于CMSIS-DSP矩阵接口实现预测、量测、增益计算和状态更新。 |
| `QuaternionEKF.h/.c` | 面向IMU姿态解算的四元数EKF,在通用卡尔曼滤波框架上实现陀螺仪/加速度计融合。 |
| `crc8.h/.c` | CRC8查表计算。 |
| `crc16.h/.c` | CRC16和Modbus CRC查表计算。 |
| `chassis_kinematics.h/.c` | 麦克纳姆轮和通用全向轮底盘运动学解算。 |
| `algo_filter.h/.c` | 通用标量滤波器,包括指数平均、一阶低通和窗口平均。 |

## 运行与实时性原则

高频任务中应优先调用已经完成初始化或预计算的接口。当前algorithm层的设计原则是:

1. 固定参数在初始化阶段处理。
2. 控制循环中尽量只做乘加、查表和少量分支。
3. 同一个算法实例默认不保证跨任务并发安全。
4. 如果多个任务或任务/中断共享同一个实例,由上层加互斥锁、临界区或改成单任务所有权。

STM32H723当前工程已经在CMake中定义`ARM_MATH_CM7`,并链接`arm_cortexM7lfdp_math`。矩阵、三角函数和开方等可以继续使用CMSIS-DSP与硬件FPU。CubeMX中应保持FreeRTOS FPU支持开启;I-Cache可提升取指效率,但D-Cache涉及DMA缓存一致性,需要结合BSP层DMA缓冲区策略统一评估。

## PID控制器

`controller`提供`PIDInstance`和`PID_Init_Config_s`。典型用法是:

1. 静态或全局保存`PIDInstance`。
2. 初始化阶段调用`PIDInit()`。
3. 控制周期中调用`PIDCalculate()`。

`Improve`位域用于选择改进项,包括积分限幅、变速积分、微分先行、输出滤波、微分滤波等。该模块适合电机速度环、电流环、角度环等周期控制场景。

## 通用数学工具

`user_lib`包含以下常用工具:

- `VAL_LIMIT`、`VAL_MIN`、`VAL_MAX`等宏。
- `float_constrain()`、`int16_constrain()`、`abs_limit()`等限幅函数。
- `loop_float_constrain()`、`theta_format()`、`rad_format()`等角度/弧度范围整理函数。
- `Norm3d()`、`NormOf3d()`、`Cross3d()`、`Dot3d()`等三维向量运算。
- `AverageFilter()`简单平均滤波。
- `MatInit()`和`mat`类型,对CMSIS-DSP矩阵实例进行封装。

`MatInit()`在矩阵数据指针为空时会分配缓冲区,因此应在初始化阶段使用。高频循环中建议复用已经初始化好的矩阵实例。

## 卡尔曼滤波

`kalman_filter`提供通用`KalmanFilter_t`。核心流程由`Kalman_Filter_Update()`串联:

1. 量测更新钩子。
2. 状态预测。
3. 协方差预测。
4. 卡尔曼增益计算。
5. 状态修正。
6. 协方差修正。

调用者可以通过结构体中的函数指针覆盖各阶段行为,从而实现线性KF、EKF或带自定义量测逻辑的滤波器。矩阵运算基于CMSIS-DSP,初始化阶段会根据状态维度、控制维度和量测维度分配矩阵缓冲区。

如果矩阵求逆失败,当前实现会保留预测结果并记录错误状态,避免在运行期直接卡死。

## 四元数EKF

`QuaternionEKF`面向IMU姿态融合。对外接口为:

- `IMU_QuaternionEKF_Init()`
- `IMU_QuaternionEKF_Update()`

状态量包括四元数和陀螺仪零偏。更新时使用陀螺仪进行姿态预测,使用加速度计观测重力方向进行修正。当加速度输入无效时,算法只进行陀螺仪预测,避免无效量测污染姿态。

该模块内部使用通用卡尔曼滤波器和CMSIS-DSP矩阵接口,适合在IMU任务中固定周期调用。调用者应保证`dt`合理,并避免多个任务同时更新同一个姿态实例。

## CRC

`crc8`和`crc16`使用静态`const`查表方式实现:

- `crc_8()`
- `update_crc_8()`
- `crc_16()`
- `crc_modbus()`
- `update_crc_16()`

查表方式避免运行期动态建表,也避免FreeRTOS下首次调用初始化表的竞态。该实现适合串口、CAN分包、裁判系统、视觉协议等短帧校验场景。

## 底盘运动学

`chassis_kinematics`支持标准X型麦克纳姆轮和通用全向轮。

坐标约定:

- `vx`: 底盘前进方向速度,x轴向前为正。
- `vy`: 底盘横移方向速度,y轴向右为正。
- `wz`: 底盘角速度,从上往下看逆时针为正。

轮速单位约定:

- `wheel_radius > 0`时,轮速为轮角速度。
- `wheel_radius <= 0`时,轮速为轮接地点线速度。

底盘运动学只保留最快路径,必须先预计算再运行:

1. 麦轮初始化阶段调用`MecanumKinematicsPrepareConfig()`。
2. 麦轮控制循环调用`MecanumKinematicsCalculateWheelSpeed()`或`MecanumKinematicsEstimateChassisSpeed()`。
3. 全向轮初始化阶段调用`OmniKinematicsPrepareConfig()`。
4. 全向轮控制循环调用`OmniKinematicsCalculateWheelSpeed()`或`OmniKinematicsEstimateChassisSpeed()`。

预计算会提前完成轮子方向归一化、旋转项系数计算、麦轮几何系数计算和全向轮正解矩阵求逆。控制循环中不再提供每次重新归一化或重新求逆的慢接口。

## 通用标量滤波器

`algo_filter`提供三类标量滤波器。

指数平均:

```c
y = y + alpha * (x - y)
```

对应接口:

- `ExpAverageFilterInit()`
- `ExpAverageFilterUpdate()`
- `ExpAverageFilterReset()`

一阶低通:

```c
alpha = dt / (rc + dt)
y = y + alpha * (x - y)
```

为了保证运行期速度,`LowPassFilter_t`只保留固定周期接口。`LowPassFilterInit()`在初始化阶段根据`rc`和`dt`预计算`alpha`;`LowPassFilterUpdate()`运行期只做乘加。若调用者已经有滤波系数,可使用`LowPassFilterInitByAlpha()`。

窗口平均:

- `WindowAverageFilterInit()`
- `WindowAverageFilterUpdate()`
- `WindowAverageFilterReset()`

窗口平均的缓存由调用者提供,算法内部不使用动态内存。该实现通过维护`sum`和环形写入位置,每次更新只替换一个样本,避免每次遍历整个窗口。

## 使用建议

- 控制环、姿态解算、底盘运动学等高频场景,应在初始化阶段完成所有配置、矩阵分配和预计算。
- 算法实例建议由所属module或application静态持有。
- 中断中只建议调用无状态、耗时短的函数;带实例状态更新的滤波器、PID、EKF更适合在任务中调用。
- 跨任务共享实例时,上层必须负责互斥或改为单任务访问。
- 新增算法应优先保持纯计算属性,不要在algorithm层引入外设、日志、任务或消息队列依赖。
