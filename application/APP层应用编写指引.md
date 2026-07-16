# APP 层应用编写指引

## 基本边界

APP 层负责机器人业务状态机、目标生成、运动学和模块组合，不应重新实现外设驱动或通信协议。

- HAL 和 MCU 外设细节放在 `bsp`。
- 外部设备、通信协议和通用算法放在 `modules`。
- 机器人结构、控制策略和状态机放在 `application`。
- APP 之间通过 `message_center` 传递数据，不直接访问另一个 APP 的私有变量。

## 初始化与周期入口

每个 APP 保持统一结构：

```c
/**
 * @brief 初始化 APP 使用的模块实例和消息端点。
 */
void ExampleInit(void);

/**
 * @brief 执行一次非阻塞的状态机和目标更新。
 */
void ExampleTask(void);
```

`Init()` 应完成实例注册、消息发布者/订阅者注册和初始安全状态设置。`Task()` 只处理一次控制更新，不允许包含永久循环；永久循环由
`robot_task.c` 中的 RTOS 任务统一提供。

## 数据通信

跨 APP 数据优先使用 `message_center`：

1. 在 `modules/message_center/message_center.h` 增加 topic 枚举和对应数据长度。
2. 在 `robot_def.h` 定义跨 APP 消息结构。
3. 发布者和订阅者在各自 `Init()` 中注册。
4. 周期入口读取最新快照或发布新快照。

不要为了同一块 MCU 内部通信而把协议结构强制设为 1 字节对齐。只有数据确实需要直接映射到线上的字节协议时，才应在协议层明确处理打包、字节序和长度。

## 任务组织

- DJI、LK、HT、DM、DDT 等通信型电机由 `motor_task` 统一管理，不为每个物理电机创建任务。
- 同一驱动或相似驱动优先共享管理任务。
- `daemon_task` 只负责在线计数和短小离线回调；耗时恢复动作放回模块自己的管理任务。
- `app_task` 默认按 `cmd -> gimbal -> chassis -> shoot` 顺序运行。
- 需要严格采样周期的传感器或控制器，可以增加独立任务，但必须说明周期、优先级、最坏执行时间和数据交换方式。

## 安全约束

- 初始化阶段默认禁止执行器输出，收到有效控制源后再切换到使能状态。
- 遥控器、上位机或关键传感器离线时，命令层必须生成明确的安全目标。
- APP 周期函数不得长时间阻塞，不得在循环中高频打印日志。
- 电机模式、反馈源和 PID 参数应在实例注册时固定，运行期只更新目标值。
- APP 不直接调用 CAN/USART HAL 发送接口，应通过对应 module 或 BSP 实例接口完成。

## 新增 APP

新增例如 `lift` 应用时：

1. 创建 `application/lift/lift.c` 和 `lift.h`。
2. 提供 `LiftInit()` 与 `LiftTask()`。
3. 在 `RobotInit()` 中调用 `LiftInit()`。
4. 在 `RobotTask()` 中按数据依赖调用 `LiftTask()`，或在 `robot_task.c` 创建专用管理任务。
5. 在 `CMakeLists.txt` 增加源文件和 include 路径。
6. 在迁入文档中记录新增任务的周期、优先级和依赖。
