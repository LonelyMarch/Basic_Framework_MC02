# message_center

`message_center` 是 APP 层之间交换固定长度结构体消息的轻量发布-订阅模块。它的目标是让
`robot_cmd`、`gimbal`、`chassis`、`shoot` 等应用保持平行关系：应用之间不互相包含头文件，也不直接调用彼此函数，而是通过约定好的 topic 传递命令和反馈。

当前版本已经重构为：

- 编译期枚举 topic ID。
- Flash/只读区 topic 调试名表。
- 静态 topic 表。
- 静态 subscriber 池。
- 每个订阅者一个 FreeRTOS 静态 Queue。
- 不使用 `malloc()`。
- 运行期不修改注册表。
- Queue 满时丢弃最旧消息，保留最新消息。
- 运行期热路径不输出日志，只用返回值表示失败。

## 适用场景

适合：

- 控制命令，例如 `MESSAGE_TOPIC_GIMBAL_CMD`、`MESSAGE_TOPIC_CHASSIS_CMD`、`MESSAGE_TOPIC_SHOOT_CMD`。
- 周期反馈，例如 `MESSAGE_TOPIC_GIMBAL_FEED`、`MESSAGE_TOPIC_CHASSIS_FEED`、`MESSAGE_TOPIC_SHOOT_FEED`。
- 小型固定长度结构体。
- 最新值比历史值更重要的数据。

不适合：

- 大块数据流。
- 文件传输。
- 需要可靠重传的协议。
- 需要在 ISR 中直接发布或读取的数据。
- 0 字节空消息。即使暂时没有反馈字段，也应放置一个 `uint8_t reserved`。

## 核心概念

**topic ID**

topic 在代码中是 `MessageCenterTopic_e` 枚举值，例如 `MESSAGE_TOPIC_GIMBAL_CMD`。枚举 ID 在编译期固定，注册时不再比较字符串，也不会在每个 topic 节点里保存名称。

模块内部仍保留一张 `const` 调试名表，例如 `"gimbal_cmd"`。这张表只用于日志和调试查看，不参与发布/读取热路径。

同一个 topic ID 的发布者和订阅者必须使用完全相同的消息长度。

**publisher**

publisher 是发布者句柄。APP 初始化时调用 `MessageCenterRegisterPublisher()` 注册，运行期调用 `MessageCenterPublish()` 发布消息。

**subscriber**

subscriber 是订阅者句柄。APP 初始化时调用 `MessageCenterRegisterSubscriber()` 注册。每个 subscriber 拥有独立 FreeRTOS Queue，因此多个订阅者订阅同一个 topic 时，彼此不会抢消息。

## 初始化流程

推荐初始化顺序如下：

```c
void RobotInit(void)
{
    __disable_irq();

    BSPInit();
    MessageCenterInit();

    RobotCMDInit();
    GimbalInit();
    ShootInit();
    ChassisInit();

    MessageCenterLockRegistration();

    __enable_irq();
}
```

`MessageCenterInit()` 清空内部静态池。工程必须在任何发布者/订阅者注册之前显式调用它；如果忘记调用，注册接口会拒绝注册并返回 `NULL`。

各 APP 的 `Init()` 函数中注册发布者和订阅者。

`MessageCenterLockRegistration()` 锁定注册阶段。锁定后不能继续注册 topic 或 subscriber，只能发布和读取消息。

这样做的好处是运行期不会再修改 topic 表和订阅者链表，并发复杂度只集中在 FreeRTOS Queue 收发路径上。

## 资源上限

资源上限定义在 `message_center.h`：

```c
#define MESSAGE_CENTER_MAX_SUBSCRIBER_COUNT 24U
#define MESSAGE_CENTER_MAX_MESSAGE_BYTES 128U
#define MESSAGE_CENTER_MAX_QUEUE_DEPTH 4U
#define MESSAGE_CENTER_DEFAULT_QUEUE_DEPTH 1U

typedef enum
{
    MESSAGE_TOPIC_GIMBAL_CMD = 0,
    MESSAGE_TOPIC_GIMBAL_FEED,
    MESSAGE_TOPIC_SHOOT_CMD,
    MESSAGE_TOPIC_SHOOT_FEED,
    MESSAGE_TOPIC_CHASSIS_CMD,
    MESSAGE_TOPIC_CHASSIS_FEED,
    MESSAGE_TOPIC_COUNT,
} MessageCenterTopic_e;
```

含义：

- `MESSAGE_CENTER_MAX_SUBSCRIBER_COUNT`：最多支持多少个订阅者。
- `MESSAGE_CENTER_MAX_MESSAGE_BYTES`：单条消息最大字节数。
- `MESSAGE_CENTER_MAX_QUEUE_DEPTH`：单个订阅者 Queue 的最大深度。
- `MESSAGE_CENTER_DEFAULT_QUEUE_DEPTH`：传入 `0` 或默认使用时的 Queue 深度。
- `MESSAGE_TOPIC_COUNT`：topic 数量，必须放在枚举末尾。

新增 topic 时，需要同时修改：

1. `MessageCenterTopic_e` 枚举。
2. `message_center.c` 中的 `topic_debug_names[]` 调试名表。

当前控制类消息推荐使用深度 1。深度 1 表示“最新帧邮箱”：如果发布者连续发布多次而订阅者还没读取，订阅者最终只保留最新一帧。

为了保持 1kHz 控制路径足够轻，深度 1 的覆盖路径不会额外调用 `uxQueueMessagesWaiting()` 统计覆盖次数。`MessageCenterDroppedCount()` 主要用于深度大于 1 的 Queue 满丢弃统计。

## 对外接口

### MessageCenterInit

```c
void MessageCenterInit(void);
```

初始化消息中心，清空静态 topic 池和 subscriber 池。

应在所有 APP 注册发布者和订阅者之前调用。

### MessageCenterLockRegistration

```c
void MessageCenterLockRegistration(void);
```

锁定注册阶段。锁定之后，`MessageCenterRegisterPublisher()` 和
`MessageCenterRegisterSubscriber()` 会拒绝继续注册。

### MessageCenterTopicName

```c
const char* MessageCenterTopicName(MessageCenterTopic_e topic_id);
```

获取 topic 的调试名称。合法 topic 返回 Flash/只读区中的调试名，非法 topic 返回 `"invalid_topic"`。

该接口只用于日志和调试，不参与消息发布/读取热路径。

### MessageCenterRegisterPublisher

```c
MessageCenterPublisher_t* MessageCenterRegisterPublisher(MessageCenterTopic_e topic_id,
                                                         size_t data_len);
```

注册 topic 发布者。

参数：

- `topic_id`：topic 枚举 ID。
- `data_len`：消息结构体长度，通常使用 `sizeof(MessageType)`，必须大于 0。

返回值：

- 成功：发布者句柄。
- 失败：`NULL`。

失败原因通常包括：

- topic ID 非法。
- 消息长度为 0。
- 消息长度超过 `MESSAGE_CENTER_MAX_MESSAGE_BYTES`。
- 同一 topic 已存在但消息长度不一致。
- 注册阶段已经被锁定。

### MessageCenterRegisterSubscriber

```c
MessageCenterSubscriber_t* MessageCenterRegisterSubscriber(MessageCenterTopic_e topic_id,
                                                           size_t data_len,
                                                           uint8_t queue_depth);
```

注册 topic 订阅者。

参数：

- `topic_id`：topic 枚举 ID。
- `data_len`：消息结构体长度，必须大于 0，且必须与同一 topic ID 一致。
- `queue_depth`：该订阅者 Queue 深度。传 `0` 时使用默认深度。

返回值：

- 成功：订阅者句柄。
- 失败：`NULL`。

订阅者允许早于发布者注册。订阅者先注册时会先创建 topic 节点，发布者后续注册同一 topic ID 时会绑定到同一个 topic 节点。

### MessageCenterPublish

```c
uint8_t MessageCenterPublish(MessageCenterPublisher_t* publisher,
                             const void* data);
```

向 topic 发布一条消息。

参数：

- `publisher`：发布者句柄。
- `data`：待发布的消息地址，必须为非空指针。

返回值：

- 成功写入的订阅者 Queue 数量。
- `0` 表示没有订阅者，或参数/调用上下文非法。

队列满时的策略：

- Queue 深度为 1：使用 `xQueueOverwrite()`，直接覆盖旧消息。
- Queue 深度大于 1：先尝试入队；若满，则丢弃最旧消息，再写入最新消息。

发布接口运行期不打印日志。参数非法、ISR误调用或无订阅者时只返回 `0`，避免高频控制路径被格式化日志打扰。

### MessageCenterFetch

```c
uint8_t MessageCenterFetch(MessageCenterSubscriber_t* subscriber,
                           void* data);
```

从订阅者 Queue 读取一条消息。

参数：

- `subscriber`：订阅者句柄。
- `data`：接收消息的缓冲区，必须为非空指针。

返回值：

- `1`：读取到新消息。
- `0`：没有新消息，或参数/调用上下文非法。

### MessageCenterPendingCount

```c
uint8_t MessageCenterPendingCount(const MessageCenterSubscriber_t* subscriber);
```

返回订阅者 Queue 中等待读取的消息数量。

### MessageCenterDroppedCount

```c
uint32_t MessageCenterDroppedCount(const MessageCenterSubscriber_t* subscriber);
```

返回该订阅者由于 Queue 满而丢弃旧消息的次数。该计数主要用于调试观察。

注意：默认深度 1 使用 `xQueueOverwrite()` 保留最新帧，为了减少热路径开销，不统计每次覆盖旧消息的次数。

## 使用示例

假设有如下消息结构体：

```c
typedef struct
{
    float yaw;
    float pitch;
} GimbalCmd_t;
```

发布者：

```c
static MessageCenterPublisher_t* gimbal_cmd_pub;
static GimbalCmd_t gimbal_cmd;

void RobotCMDInit(void)
{
    gimbal_cmd_pub = MessageCenterRegisterPublisher(MESSAGE_TOPIC_GIMBAL_CMD,
                                                    sizeof(GimbalCmd_t));
}

void RobotCMDTask(void)
{
    gimbal_cmd.yaw = 10.0f;
    gimbal_cmd.pitch = 2.0f;
    (void)MessageCenterPublish(gimbal_cmd_pub, &gimbal_cmd);
}
```

订阅者：

```c
static MessageCenterSubscriber_t* gimbal_cmd_sub;
static GimbalCmd_t gimbal_cmd_recv;

void GimbalInit(void)
{
    gimbal_cmd_sub = MessageCenterRegisterSubscriber(MESSAGE_TOPIC_GIMBAL_CMD,
                                                     sizeof(GimbalCmd_t),
                                                     MESSAGE_CENTER_DEFAULT_QUEUE_DEPTH);
}

void GimbalTask(void)
{
    if (MessageCenterFetch(gimbal_cmd_sub, &gimbal_cmd_recv) != 0U)
    {
        // 收到新命令后更新控制目标
    }
}
```

## 当前工程中的 topic

单板模式下，当前主要 topic 如下：

| topic ID | 调试名 | 发布者 | 订阅者 | 消息类型 |
| --- | --- | --- | --- | --- |
| `MESSAGE_TOPIC_GIMBAL_CMD` | `gimbal_cmd` | `robot_cmd` | `gimbal` | `Gimbal_Ctrl_Cmd_s` |
| `MESSAGE_TOPIC_GIMBAL_FEED` | `gimbal_feed` | `gimbal` | `robot_cmd` | `Gimbal_Upload_Data_s` |
| `MESSAGE_TOPIC_SHOOT_CMD` | `shoot_cmd` | `robot_cmd` | `shoot` | `Shoot_Ctrl_Cmd_s` |
| `MESSAGE_TOPIC_SHOOT_FEED` | `shoot_feed` | `shoot` | `robot_cmd` | `Shoot_Upload_Data_s` |
| `MESSAGE_TOPIC_CHASSIS_CMD` | `chassis_cmd` | `robot_cmd` | `chassis` | `Chassis_Ctrl_Cmd_s` |
| `MESSAGE_TOPIC_CHASSIS_FEED` | `chassis_feed` | `chassis` | `robot_cmd` | `Chassis_Upload_Data_s` |

双板模式下，`robot_cmd` 和 `chassis` 之间的底盘命令/反馈会改用 `can_comm` 跨板传输。

## FreeRTOS 和中断约束

`message_center` 使用 FreeRTOS 静态 Queue，Queue 的收发接口本身支持任务间并发访问。

模块不提供 ISR 版本接口。`MessageCenterPublish()` 和 `MessageCenterFetch()` 会拒绝 ISR 中调用。

如果中断中产生了某类事件，应遵循当前 BSP 层约定：

1. ISR 中只保存必要数据或投递延后事件。
2. 在任务上下文中解析数据。
3. 再调用 `MessageCenterPublish()` 发布给 APP。

## 设计取舍

为什么不使用 `malloc()`：

- 避免 newlib heap 与 FreeRTOS heap 混用。
- 避免堆碎片。
- 启动后资源占用完全可预测。

为什么使用枚举 topic ID：

- 不在每个 topic 节点中保存字符串，节省 RAM。
- 注册阶段按 ID 直接索引 topic 表，不再 `strcmp()`。
- 编译器能检查枚举名，减少字符串拼写错误导致的隐性 bug。
- 调试名表放在 `const` 只读区，需要日志时仍能看到可读名称。

为什么每个 subscriber 一个 Queue：

- 多个订阅者订阅同一个 topic 时，互不影响。
- 一个订阅者读取消息不会让另一个订阅者丢消息。
- FreeRTOS Queue 负责收发同步，减少手写并发保护。

为什么 Queue 满时丢最旧消息：

- 控制命令和周期反馈通常只关心最新值。
- 旧命令滞留可能比丢弃更危险。
- 该策略能保证慢订阅者恢复读取时拿到较新的状态。

## 维护建议

- 新增 topic 时，消息结构体应放在 `application/robot_def.h` 或对应公共头文件中。
- 新增 topic 时，必须同时更新 `MessageCenterTopic_e` 和 `topic_debug_names[]`。
- 发布者和订阅者必须使用同一个消息类型的 `sizeof()`。
- 消息结构体不能是 0 字节；暂无字段时使用 `uint8_t reserved;` 占位。
- 控制类 topic 优先使用 `MESSAGE_CENTER_DEFAULT_QUEUE_DEPTH`。
- 只有确实需要保留短历史时，才提高 `queue_depth`。
- 若消息结构体超过 `MESSAGE_CENTER_MAX_MESSAGE_BYTES`，应优先重新设计消息，而不是直接扩大上限。
- 不要在 ISR 中调用发布/读取接口。
- 不要在 `MessageCenterLockRegistration()` 之后注册新的发布者或订阅者。
