# referee

`modules/referee` 是 RoboMaster 裁判系统模块，负责裁判系统串口通信、协议解析、裁判数据缓存、客户端 UI 绘制，以及机器人间自定义交互数据收发。

## 模块组成

- `rm_referee.c/.h`：裁判系统通信核心。注册 USART BSP，解析裁判系统下发帧，维护 `referee_info_t` 数据快照，并提供交互帧发送队列。
- `referee_UI.c/.h`：客户端 UI 构帧工具。负责删除图层、绘制图形、绘制字符串，并把 UI 帧投递给 `RefereeSend()`。
- `referee_task.c/.h`：UI 任务逻辑。负责首次绘制 UI、检测 application 层传入的状态变化，并按需刷新动态 UI。
- `referee_protocol.h`：裁判系统协议命令码、数据长度、协议结构体和 UI 枚举定义。
- `crc_ref.c/.h`：裁判系统协议使用的 CRC8/CRC16 校验实现。

## 硬件与 BSP 依赖

当前工程中裁判系统使用 USART1。CubeMX 与 BSP 侧需要满足：

- USART1 RX DMA 已开启。
- USART1 TX DMA 已开启。
- USART1 global interrupt 已开启。
- `bsp/usart` 已接入 `BSPServiceTask()`，接收回调在任务上下文中执行。

裁判系统接收依赖 `HAL_UARTEx_ReceiveToIdle_DMA()` 的 IDLE 帧结束机制，因此 UART 全局中断必须开启，否则变长帧接收不可靠。

## 初始化流程

Application 层在初始化阶段调用 `UITaskInit()`：

```c
static Referee_Interactive_info_t ui_data;

if (UITaskInit(&huart1, &ui_data) == NULL)
{
    LOGWARNING("[chassis] referee init failed, UI and referee feedback are disabled");
}
```

`UITaskInit()` 内部会调用 `RefereeInit()` 完成 USART BSP 注册，并保存 application 层传入的 UI 状态结构体指针。

RTOS 启动后，`StartUITASK()` 会先调用 `MyUIInit()`：

- 返回 `1U`：已经收到裁判系统 `robot_id`，并且初始 UI 帧全部成功进入发送队列。
- 返回 `0U`：裁判系统暂未上线，或者初始 UI 帧暂时无法全部入队。后续由 `UITask()` 低频重试。

`StartUITASK()` 进入循环后每 `1ms` 调用一次 `UITask()`。`UITask()` 不会阻塞等待 UI 发送完成，而是每轮先泵一次发送队列，再根据状态决定是否继续初始化或刷新 UI。

## 接收流程

裁判系统 USART 收到数据后，底层流程如下：

1. USART DMA/IDLE 中断只记录接收长度并唤醒 BSP 服务任务。
2. `BSPServiceTask()` 调用 `USARTProcess()`。
3. `USARTProcess()` 在任务上下文执行 referee 的接收回调。
4. `JudgeReadData()` 在接收 buffer 中扫描裁判系统帧。
5. 找到 `0xA5` 帧头后校验帧头 CRC8。
6. 根据帧头中的数据长度计算完整帧长度。
7. 校验整帧 CRC16。
8. 根据 `cmd_id` 更新 `referee_info_t` 中对应的数据字段。

`0x0301` 学生机器人交互帧同时承载客户端 UI 和机器人间通信。本模块只把 `data_cmd_id == Communicate_Data_ID` 的数据保存到 `ReceiveData`，避免把本机发送的 UI 绘图帧误当作自定义通信数据。

## 数据读取

裁判系统内部数据由 USART 解析任务更新。其他任务读取裁判系统数据时，建议使用 `RefereeGet()` 复制快照：

```c
referee_info_t referee_snapshot;

if (RefereeGet(&referee_snapshot) != 0U)
{
    // 使用 referee_snapshot 中的功率、热量、血量、机器人 ID 等数据
}
```

`RefereeGet()` 会在短临界区中复制内部结构体，避免其他任务直接读取内部指针时碰到半更新数据。

`RefereeInit()` 仍返回内部 `referee_info_t *`，主要用于旧代码兼容和 UI 模块内部状态判断。新代码不建议长期保存并跨任务直接读取该指针。

## UI 数据源

`Referee_Interactive_info_t` 是 application 层传给 referee 模块的 UI 状态源，主要包含：

- `chassis_mode`
- `gimbal_mode`
- `shoot_mode`
- `friction_mode`
- `lid_mode`
- `Chassis_Power_Data`

referee 模块只负责检测这些字段是否变化，并根据变化刷新客户端 UI。真实机器人状态需要 application 层主动写入该结构体，否则动态 UI 不会反映实际状态。

## UI 任务逻辑

`UITask()` 的执行顺序如下：

1. 调用 `RefereeTxProcess()`，按裁判系统频率限制发送队列中的下一帧。
2. 如果 referee 未初始化完成，则直接返回。
3. 如果初始 UI 尚未绘制完成，则等待 `robot_id`，并按低频重试机制投递初始化 UI 帧。
4. 初始 UI 完成后，调用 `UIChangeCheck()` 检查 UI 数据源是否发生变化。
5. 调用 `MyUIRefresh()` 刷新发生变化的动态 UI。

动态 UI 只有在对应刷新帧成功进入发送队列后，才会清除对应 flag。若发送队列暂时满，flag 会保留，后续任务循环继续尝试。

当前 UI 测试逻辑默认关闭。如需让 UI 自动轮换模式用于调试，可在编译时定义：

```c
#define REFEREE_UI_TEST_ENABLE 1U
```

## UI 绘制接口

绘制接口分为两类：

- `UILineDraw()`、`UIRectangleDraw()`、`UICircleDraw()`、`UIOvalDraw()`、`UIArcDraw()`、`UIFloatDraw()`、`UIIntDraw()`、`UICharDraw()`：只填充图形或字符串结构体，不发送。
- `UIDelete()`、`UIGraphRefresh()`、`UICharRefresh()`：构造完整裁判系统交互帧，并投递到 referee 发送队列。

绘制一条线：

```c
Graph_Data_t line;

UILineDraw(&line, "ln0", UI_Graph_ADD, 7, UI_Color_White, 2, 100, 100, 300, 100);
(void)UIGraphRefresh(&referee_id, 1, line);
```

绘制字符串：

```c
String_Data_t text;

UICharDraw(&text, "tx0", UI_Graph_ADD, 7, UI_Color_Green, 18, 2, 100, 200, "Power:");
(void)UICharRefresh(&referee_id, text);
```

`UIDelete()`、`UIGraphRefresh()`、`UICharRefresh()` 返回 `HAL_StatusTypeDef`：

- `HAL_OK`：UI 帧已成功进入发送队列。
- `HAL_BUSY`：发送队列已满，本帧未入队。
- `HAL_ERROR`：参数错误或构帧过程失败。

`UIGraphRefresh()` 只支持一次发送 `1`、`2`、`5`、`7` 个图形，这是裁判系统 UI 协议本身的限制。

## 发送队列

裁判系统 `0x0301` 交互数据存在上行频率限制。为了避免 UI 绘制阻塞调用任务，当前模块采用队列化发送：

- `RefereeSend()` 只把一帧交互数据复制到内部发送队列，不直接等待串口发送完成。
- `RefereeTxProcess()` 在 UI 任务中周期运行，USART 空闲且满足发送间隔时才启动下一帧 DMA 发送。

发送间隔约为 `115ms`。裁判系统交互数据通常按约 `10Hz` 使用，理论间隔为 `100ms`；额外保留一点余量，用来覆盖任务调度抖动和时钟误差，避免实际发送频率超过协议限制。

当前发送队列深度为 `32` 帧。初始 UI 大约会连续投递 15 帧，正常空队列启动时可以容纳。若队列满：

- 初始化 UI 会保持未完成，之后按低频重试。
- 动态 UI 会保留刷新 flag，等待后续发送队列有空间后继续投递。

## 使用约束

- UI 绘制和刷新接口建议只在 UI 任务中调用。
- 其他任务需要裁判系统数据时，使用 `RefereeGet()` 获取快照。
- `Referee_Interactive_info_t` 由 application 层维护，referee 模块不会主动采集底盘、云台、发射机构状态。
- `RefereeSend()` 面向 UI 与机器人间通信共用，调用者需要检查返回值。
- 裁判系统 CRC 与 `modules/algorithm` 中的通用 CRC 实现用途不同，不能随意替换。
- `referee_protocol.h` 中协议结构体按裁判系统协议保持 1 字节对齐；运行期状态结构体不额外强制 pack。

## 当前接入点

当前工程中，`application/chassis/chassis.c` 定义 `ui_data` 并调用 `UITaskInit(&huart1, &ui_data)`。

`application/robot_task.h` 中的 `StartUITASK()` 负责运行 UI 任务：

```c
__attribute__((noreturn)) void StartUITASK(void *argument)
{
    if (MyUIInit() != 0U)
    {
        LOGINFO("[freeRTOS] UI init done, referee communication established");
    }

    for (;;)
    {
        UITask();
        osDelay(1);
    }
}
```

后续如果需要让 UI 显示真实状态，需要在 application 层把实际模式和功率数据同步到 `ui_data`。
