# bsp_can

<p align='right'>neozng1@hnu.edu.cn</p>

# 请注意使用CAN设备的时候务必保证总线只接入了2个终端电阻！开发板一般都有一个，6020电机、c620/c610电调、LK电机也都有终端电阻，注意把多于2个的全部断开（通过拨码）



## 使用说明

若你希望新增一个基于CAN的module，首先在该模块下应该有一个包含`CANInstance*`指针的module结构体（或当功能简单的时候，可以是单独存在的`CANInstance*`，但不推荐这样做）。

## 代码结构

.h文件内包括了外部接口和类型定义,以及模块对应的宏。c文件内为私有函数和外部接口的定义。

## 类型定义

```c

#define FDCAN // G系列和H7系列使用FDCAN
// #define BXCAN // F系列使用BxCAN

#define hcan1 hfdcan1
#define hcan2 hfdcan2
#define hcan3 hfdcan3

#define CAN_MX_REGISTER_CNT 32     // 所有FDCAN外设共用的CAN实例注册上限
#define MX_CAN_FILTER_CNT (3 * 14) // 最多可以使用的CAN过滤器数量,目前远不会用到这么多
#define DEVICE_CAN_CNT 3           // H723VG有3个FDCAN

/* can instance typedef, every module registered to CAN should have this variable */
typedef struct _
{
    FDCAN_HandleTypeDef *can_handle; // can句柄
    FDCAN_TxHeaderTypeDef txconf;    // CAN报文发送配置
    uint32_t tx_id;                // 发送id
    uint32_t tx_mailbox;           // BxCAN消息填入的邮箱号,FDCAN模式下不使用
    uint8_t tx_buff[8];            // 发送缓存,发送消息长度可以通过CANSetDLC()设定,最大为8
    uint8_t rx_buff[8];            // 接收缓存,最大消息长度为8
    uint32_t rx_id;                // 接收id
    uint8_t rx_len;                // 接收长度,可能为0-8
    // 接收的回调函数,用于解析接收到的数据
    void (*can_module_callback)(struct _ *); // callback needs an instance to tell among registered ones
    void *id;                                // 使用can外设的模块指针(即id指向的模块拥有此can实例,是父子关系)
} CANInstance;

typedef struct
{
    FDCAN_HandleTypeDef *can_handle;          // can句柄
    uint32_t tx_id;                           // 发送id
    uint32_t rx_id;                           // 接收id
    void (*can_module_callback)(CANInstance *); // 处理接收数据的回调函数
    void *id;                                 // 拥有can实例的模块地址
} CAN_Init_Config_s;
```

- `CAN_MX_REGISTER_CNT`是最大的CAN实例注册数量，当前FDCAN分支下为三路FDCAN共用的全局上限。
- `MX_CAN_FILTER_CNT`是最大的CAN接收过滤器数量宏。FDCAN实际可用过滤器数量由CubeMX中每路FDCAN的`StdFiltersNbr`决定。
- `DEVICE_CAN_CNT`是MCU拥有的CAN/FDCAN硬件数量。

- `CANInstance`是一个CAN实例。注意，CAN作为一个总线设备，一条总线上可以挂载多个设备，因此多个设备可以共享同一个CAN硬件。其成员变量包括发送id，发送配置，发送buff以及接收buff，还有接收id和接收协议解析回调函数。**由于目前使用的是Classic CAN，每个数据帧最大长度为8，因此收发buff长度暂时固定为8**。定义该结构体的时候使用了一个技巧，使得在结构体内部可以用结构体自身的指针作为成员，即`can_module_callback`的定义。

- `CAN_Init_Config_s`是用于初始化CAN实例的结构，在调用CAN实例的初始化函数时传入（下面介绍函数时详细介绍）。

- `can_module_callback()`是模块提供给CAN接收中断回调函数使用的协议解析函数指针。对于每个需要CAN的模块，需要定义一个这样的函数用于解包数据。
- 每个使用CAN外设的module，都需要在其内部定义一个`CANInstance*`。


## 外部接口

```c
CANInstance *CANRegister(CAN_Init_Config_s *config);
void CANSetDLC(CANInstance *_instance, uint8_t length); // 设置发送帧的数据长度
uint8_t CANTransmit(CANInstance *_instance, float timeout);
```

`CANRegister`是用于初始化CAN实例的接口，module层的模块对象（也应当为一个结构体）内要包含一个`CANInstance*`。调用时传入用于初始化的config，函数返回注册完成后的`CANInstance*`。`CANRegister`应当在module的初始化函数内被调用，推荐config采用以下的方式定义，更加直观明了：

```c
CAN_Init_Config_s config = {
    .can_handle = &hcan1,
    .tx_id = 0x005,
    .rx_id = 0x200,
    .can_module_callback = MotorCallback,
};
```

`CANTransmit()`是模块通过其拥有的CAN实例发送数据的接口，调用时传入对应的instance。在发送之前，应当给instance内的`tx_buff`赋值。

## 私有函数和变量

在.c文件内设为static的函数和变量

```c
static CANInstance *can_instance[CAN_MX_REGISTER_CNT] = {NULL};
```

这是bsp层管理所有CAN实例的入口。当前所有FDCAN外设共用这一个实例指针数组，接收回调中根据`can_handle`和`rx_id`查找对应实例。

```c
static void CANServiceInit()
static void CANAddFilter(CANInstance *_instance)
static void CANStartFDCANService(FDCAN_HandleTypeDef *hfdcan, uint8_t can_idx, uint32_t rx_active_its)
static uint8_t FDCANDecodeDLC(uint32_t dlc)
static void FDCANFIFOxCallback(FDCAN_HandleTypeDef *_hfdcan, uint32_t fifox)
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
```

- `CANServiceInit()`会被`CANRegister()`调用，对CAN外设进行硬件初始化并开启接收中断和消息提醒。FDCAN模式下会启动FDCAN1、FDCAN2、FDCAN3。

- `CANStartFDCANService()`用于启动单路FDCAN服务，配置FIFO覆盖模式、全局过滤器、启动FDCAN并激活接收通知。若HAL配置失败，会通过日志输出错误并停止运行。

- `CANAddFilter()`在每次使用`CANRegister()`的时候被调用，用于给当前注册的实例添加过滤器规则并设定处理对应`rx_id`的接收FIFO。FDCAN分支会按当前FDCAN句柄分别分配过滤器索引，并检查是否超过CubeMX配置的`StdFiltersNbr`。过滤器按接收ID匹配，奇数`rx_id`进入FIFO0，偶数`rx_id`进入FIFO1。

- `FDCANDecodeDLC()`用于将HAL FDCAN接收头中的`DataLength`宏转换为实际接收字节数。当前Classic CAN配置下支持0到8字节。

- `HAL_FDCAN_RxFifo0Callback()`和`HAL_FDCAN_RxFifo1Callback()`都是对HAL的FDCAN回调函数的重定义，当FIFO0或FIFO1有新消息、满、水位线到达或消息丢失时，对应callback会被调用。`FDCANFIFOxCallback()`随后被前两者调用，并根据接收id和硬件中断来源调用对应的instance的回调函数进行协议解析。`FDCANFIFOxCallback()`会循环取出当前FIFO中的报文，直到FIFO为空。

- 当有一个模块注册了多个can实例时，通过`CANInstance.id`,使用强制类型转换将其转换成对应模块的实例指针，就可以对不同的模块实例进行回调处理了。

## 注意事项

由于CAN总线自带发送检测，如果总线上没有挂载目标设备（接收id和发送报文相同的设备），那么发送队列可能会被占满而无法发送。在`CANTransmit()`中会检查发送邮箱或FDCAN Tx FIFO Queue是否空闲。当超出`timeout`之后函数会返回零，说明发送失败。

由于会等待发送邮箱或FDCAN Tx FIFO Queue空闲，调用`CANTransmit()`的任务可能无法按时挂起，导致任务定时不精确。建议在没有连接CAN进行调试时，按需注释掉有关CAN发送的代码部分，或设定一个较小的`timeout`值，防止对其他需要精确定时的任务产生影响。
