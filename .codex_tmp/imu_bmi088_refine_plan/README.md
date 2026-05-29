# BMI088 / IMU 精修计划

本文档是本轮 Codex 对话中的临时计划文件,用于指导后续 `modules/BMI088` 与 `modules/imu` 的收敛重构。当前只做静态分析和计划整理,未编译,未修改工程源码。

## 参考材料

- 当前工程根目录: `E:\NUEDC\code\gimbal`
- 当前 BMI088 新模块: `modules/BMI088`
- 当前 IMU/INS 模块: `modules/imu`
- 达妙官方 IMU 例程: `C:\Users\LonelyMarch\Desktop\dm-mc02-master\demo\CtrBoard-H7_IMU`
- 达妙官方 IMU 温控例程: `C:\Users\LonelyMarch\Desktop\dm-mc02-master\demo\CtrBoard-H7_IMU_TempCtrl`
- BMI088 数据手册: `C:\Users\LonelyMarch\Desktop\dm-mc02-master\数据手册\bst-bmi088-ds001.pdf`

## 当前结论

当前工程同时保留两套 BMI088 驱动:

- `modules/BMI088/bmi088.c`: 新版注册式 BMI088 驱动,已经接入 `bsp/spi`、`bsp/gpio`、`bsp/pwm`,但还没有成为 INS 主流程使用的驱动。
- `modules/imu/BMI088driver.c`: 旧版 BMI088 驱动,现在已经被改成通过 `bsp/spi` 访问 SPI2,但接口和全局状态仍是旧结构。

实际姿态任务仍由 `modules/imu/ins_task.c` 调用旧接口:

- `BMI088Init(&hspi2, 1)`
- `BMI088_Read(&BMI088)`

因此精修目标应是: 保留 `modules/BMI088` 作为唯一 BMI088 芯片驱动,让 `modules/imu` 只负责 INS、姿态解算、温控和对外姿态数据。

## 硬件与 CubeMX 现状

当前工程与达妙官方例程一致的关键硬件连接:

- SPI2: BMI088 通信总线
- PC0: 加速度计片选,当前工程名 `CS2_ACCEL`,官方例程名 `ACC_CS`
- PC3: 陀螺仪片选,当前工程名 `CS2_GYRO`,官方例程名 `GYRO_CS`
- PE10: 加速度计 DRDY 中断,`ACC_INT`
- PE12: 陀螺仪 DRDY 中断,`GYRO_INT`
- PB1 / TIM3_CH4: IMU 加热 PWM

当前工程 SPI2 配置:

- CPOL high
- CPHA second edge
- 8 bit
- master
- 当前 `.ioc` 计算速率约 3.75 Mbit/s

官方例程 SPI2 也是 Mode 3,但速率约 7.5 Mbit/s。当前 3.75 Mbit/s 更保守,短期不用急改。

需要注意: 当前 `.ioc` 中 PE10/PE12 已是 EXTI 信号,但静态搜索未看到 `EXTI15_10_IRQHandler()`、`HAL_GPIO_EXTI_IRQHandler()` 或 `NVIC.EXTI15_10_IRQn`。所以如果后续启用中断触发读取,还需要在 CubeMX 中开启 `EXTI line[15:10] interrupt`,优先级建议按当前工程风格设为 5。

## 目标架构

### `modules/BMI088`

只负责 BMI088 芯片级能力:

- SPI 片选与寄存器读写封装
- 加速度计/陀螺仪初始化
- 数据读取与单位换算
- 温度读取与符号扩展
- 在线/离线标定参数管理
- 可选 DRDY 事件记录

不建议继续负责:

- 温控 PID
- 加热 PWM
- 姿态解算
- `VisionSetAltitude()`
- INS 任务调度

### `modules/imu`

负责板级 IMU/INS 逻辑:

- 创建并保存 `BMI088Instance`
- 注册 IMU 加热 PWM
- 初始化温控 PID
- 初始化四元数 EKF
- 读取 BMI088 数据
- 安装误差/坐标修正
- 更新 `attitude_t`
- 向视觉/云台电机提供姿态和角速度

## 阶段 1: 先把 `modules/BMI088` 打磨成可靠唯一驱动

建议先只支持 `BMI088_BLOCK_PERIODIC_MODE`,即 INS 任务 1 kHz 周期性同步读取。中断触发模式放到阶段 4。

具体修改项:

1. 清理职责边界
   - 从 `BMI088Instance` 和 `BMI088_Init_Config_s` 中移除或暂时弃用 `heat_pid`、`heat_pwm`、`heat_pid_config`、`heat_pwm_config`。
   - BMI088 驱动不再注册 PWM、不再初始化温控 PID。
   - 删除未使用的 `bmi088_daemon_instance`,除非后续明确要做模块离线监测。

2. 固定实例策略
   - 当前板子只有一个 BMI088,建议改为模块内静态实例,避免 `zmalloc()`。
   - `BMI088Register()` 可以返回静态实例指针。
   - 重复注册时直接返回已有实例,避免多次注册 SPI 片选实例。

3. 对齐官方初始化流程
   - accel 初始化: 上电后先读 chip id 触发 SPI 模式,延时,再读一次。
   - accel soft reset 后等待,再读两次 chip id。
   - gyro 初始化: reset 前后也读两次 chip id。
   - 写配置后读回校验。
   - 所有 HAL/SPI 失败路径返回明确错误码并限频打印日志。

4. 统一量程与灵敏度
   - 当前旧主流程使用 `BMI088_ACC_RANGE_6G` 和 `BMI088_ACCEL_6G_SEN`。
   - 官方例程默认是 `BMI088_ACC_RANGE_3G` 和 `BMI088_ACCEL_3G_SEN`。
   - 建议短期保持当前工程的 6G,因为云台运动和安装振动留量更大;但必须把寄存器配置、灵敏度、文档三者保持一致。

5. 修复温度读取
   - 温度原始值是 11 bit 有符号数。
   - 读取后需要:
     - `raw = (buf[0] << 3) | (buf[1] >> 5)`
     - 如果 `raw > 1023`,则 `raw -= 2048`
   - 新驱动当前周期读取里缺少这个符号扩展,需要修。

6. 统一数据输出
   - `BMI088Acquire()` 成功后同时更新:
     - `data_store->acc`
     - `data_store->gyro`
     - `data_store->temperature`
     - `bmi088->acc`
     - `bmi088->gyro`
     - `bmi088->temperature`
   - 加速度输出应乘 `acc_coef`。
   - 陀螺仪输出应减 `gyro_offset`。

7. 整理标定参数
   - 当前新驱动的离线宏名为 `BMI088_PRE_CALI_ACC_X_OFFSET`,但实际写入 `gyro_offset`,命名错误。
   - 建议改为:
     - `BMI088_PRE_CALI_GYRO_X_OFFSET`
     - `BMI088_PRE_CALI_GYRO_Y_OFFSET`
     - `BMI088_PRE_CALI_GYRO_Z_OFFSET`
     - `BMI088_PRE_CALI_G_NORM`
   - 标定失败时加载离线参数,并保证 `acc_coef = raw_sen * 9.81f / gNorm`。

8. 明确返回语义
   - 短期保留 `uint8_t BMI088Acquire(...)`: `1` 表示本次读取成功,`0` 表示失败或没有新数据。
   - 若后续要更精细诊断,再引入 `BMI088_Status_e`,不建议第一步过度扩大接口。

## 阶段 2: 迁移 `modules/imu/ins_task`

目标: `ins_task` 不再包含旧 `BMI088driver.h`,不再使用全局 `BMI088`。

具体修改项:

1. 头文件替换
   - `ins_task.h` 从 `#include "BMI088driver.h"` 改为 `#include "bmi088.h"` 或只在 `.c` 中包含。
   - 对外仍保持 `attitude_t *INS_Init(void)` 和 `void INS_Task(void)`。

2. 新增模块内状态
   - `static BMI088Instance *ins_bmi088 = NULL;`
   - `static BMI088_Data_t ins_bmi088_data = {0};`
   - 温控使用 `ins_bmi088_data.temperature`。

3. 在 `INS_Init()` 中注册 BMI088
   - SPI accel:
     - `spi_handle = &hspi2`
     - `GPIOx = CS2_ACCEL_GPIO_Port`
     - `cs_pin = CS2_ACCEL_Pin`
     - `spi_work_mode = SPI_BLOCK_MODE`
   - SPI gyro:
     - `spi_handle = &hspi2`
     - `GPIOx = CS2_GYRO_GPIO_Port`
     - `cs_pin = CS2_GYRO_Pin`
     - `spi_work_mode = SPI_BLOCK_MODE`
   - `work_mode = BMI088_BLOCK_PERIODIC_MODE`
   - `cali_mode` 初期可以继续使用在线标定,也可以后续切离线参数。

4. 温控继续留在 `modules/imu`
   - 注册 `TIM3_CH4` 的 `PWMInstance`。
   - 初始化 `TempCtrl`。
   - `IMU_Temperature_Ctrl()` 只根据 `ins_bmi088_data.temperature` 计算输出。
   - 保持当前 `bsp_pwm` 的 0~1 占空比语义。

5. 初始化四元数
   - `InitQuaternion()` 改为调用 `BMI088Acquire(ins_bmi088, &ins_bmi088_data)`。
   - 采样失败时跳过本次样本或重试,不要直接使用旧数据。
   - 保留 100 次静态平均的逻辑。

6. 主任务读取
   - `INS_Task()` 每 1 ms 调用 `BMI088Acquire()`。
   - 读取成功后更新 `INS.Accel`、`INS.Gyro`、EKF、视觉姿态。
   - 读取失败时保留上一帧姿态,并限频打印或交给 daemon 记录。

7. `attitude_t` 返回指针整理
   - 当前 `return (attitude_t *)&INS.Gyro` 是不稳妥的结构体布局偷懒。
   - 建议给 `INS_t` 中增加一个真实 `attitude_t attitude;`,或把 `attitude_t` 与 `INS_t` 的公共字段通过显式函数同步。
   - 第一轮迁移若想降风险,可以暂时保留原返回方式;第二轮再改。

## 阶段 3: 移除旧 BMI088 驱动链路

当 `ins_task` 已经完全改用 `modules/BMI088` 后:

1. 从 `CMakeLists.txt` 移除:
   - `modules/imu/BMI088driver.c`
   - `modules/imu/BMI088Middleware.c`

2. 从 include 路径和代码引用中清理:
   - `BMI088driver.h`
   - `BMI088Middleware.h`
   - `BMI088reg.h`

3. 文件处理策略:
   - 如果用户希望保留历史参考,先不删除文件,只不参与构建。
   - 如果用户希望彻底清理,再删除旧文件。

4. 静态检查:
   - `rg "BMI088Init|BMI088_Read|BMI088driver|BMI088Middleware|IMU_Data_t BMI088"`
   - 确认没有主流程引用旧符号。

## 阶段 4: 再考虑中断触发模式

短期不建议第一步就上中断触发,原因:

- 当前 `.ioc` 里未看到 EXTI15_10 NVIC 启用。
- 现有 `bsp/spi` 对上层表现为同步事务,周期阻塞读已经能满足 1 kHz IMU。
- 中断模式会同时牵涉 EXTI、SPI DMA、任务唤醒、数据新鲜度判断,改动面更大。

后续如果启用:

1. CubeMX 中开启 `EXTI line[15:10] interrupt`
   - 当前工程建议优先级 5,与 SPI2/DMA 和其他 RTOS 可调用中断保持一致。

2. 使用 GYRO_INT 作为主节拍
   - 陀螺仪 1 kHz 更适合作为姿态预测节奏。
   - ACC_INT 用于标记加速度新量测。

3. 中断/回调只收集事件
   - EXTI 回调不做 SPI 读写。
   - 回调只递增计数或设置标志。
   - 真正 `BMI088Acquire()` 放在 INS 任务上下文。

4. 任务唤醒方式
   - 可使用 `osThreadFlagsSet(insTaskHandle, ...)`。
   - 但这会让 `modules/imu` 依赖 application 层任务句柄,需要谨慎处理。
   - 更稳妥是 `INS_Task()` 周期轮询标志,后续再单独设计事件唤醒接口。

## 阶段 5: 文档与验证

文档更新:

- 重写 `modules/BMI088/bmi088.md`
- 重写 `modules/imu/ins_task.md`
- 如 `modules/module.md` 或总览文档有 module 层说明,同步更新

静态验证:

- 不编译。
- 使用 `rg` 检查旧符号是否清干净。
- 使用 `git diff --check` 检查空白问题。
- 重点检查:
  - `CMakeLists.txt` 是否只编译一套 BMI088 驱动。
  - `ins_task.c` 是否只通过 `BMI088Register/BMI088Acquire` 获取 IMU 数据。
  - 温度控制是否只依赖新数据结构。
  - SPI2 片选 GPIO 是否注册为 PC0/PC3。
  - 如果启用中断模式,`.ioc` 是否启用 EXTI15_10_IRQn。

## 推荐执行顺序

1. 先修 `modules/BMI088` 的纯驱动问题。
2. 再把 `modules/imu/ins_task` 切到新驱动。
3. 静态确认旧接口无引用。
4. 从 `CMakeLists.txt` 移除旧 `BMI088driver.c` / `BMI088Middleware.c`。
5. 更新文档。
6. 后续单独评估中断触发模式。

这个顺序的好处是每一步都有清晰边界: 先保证新驱动本身可靠,再切主流程,最后清理旧代码。
