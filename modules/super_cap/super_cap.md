# super_cap

`super_cap` 是超级电容模块封装，用于通过 CAN 接收超级电容反馈，并向超级电容发送控制数据。

当前工程只使用一台超级电容，因此模块内部使用单实例静态池，不依赖 `malloc()`。

## 工作流程

1. 应用层填写 `SuperCap_Init_Config_s`，指定超级电容所在的 FDCAN、发送 ID、接收 ID 和可选的在线检测计数。
2. `SuperCapInit()` 注册 CAN 实例，并把 `SuperCapInstance *` 写入 `CANInstance.id`。
3. BSP CAN 收到超级电容反馈后，由 `CANProcessTask` 调用 `SuperCapRxCallback()`。
4. 回调解析 6 字节反馈帧，并更新 `SuperCap_Msg_s`。
5. 应用层通过 `SuperCapGet()` 获取新数据快照，通过 `SuperCapIsOnline()` 判断超级电容是否在线。

## 数据格式

超级电容反馈帧长度为 6 字节：

| 字节  | 含义       |
|-----|----------|
| 0-1 | 电压，高字节在前 |
| 2-3 | 电流，高字节在前 |
| 4-5 | 功率，高字节在前 |

`SuperCap_Msg_s` 中保存的是协议原始值，具体比例系数由超级电容固件协议决定。

## 接口

```c
SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *supercap_config);
uint8_t SuperCapSend(SuperCapInstance *instance, const uint8_t *data, uint8_t len);
uint8_t SuperCapGet(SuperCapInstance *instance, SuperCap_Msg_s *msg);
uint8_t SuperCapIsOnline(SuperCapInstance *instance);
```

`SuperCapSend()` 的 `len` 最大为 8 字节，返回 `1` 表示数据已经成功提交到 CAN 发送 FIFO，返回 `0` 表示参数错误或发送失败。

`SuperCapGet()` 返回 `1` 表示本次取到了新反馈数据，返回 `0` 表示没有新数据或参数错误。该接口会在任务态短暂锁住调度器，避免读取时和
`CANProcessTask` 更新数据互相打断。

## 使用示例

```c
SuperCap_Init_Config_s cap_conf = {
    .can_config = {
        .can_handle = &hfdcan2,
        .tx_id = 0x302,
        .rx_id = 0x301,
    },
    .daemon_count = 100,
};

SuperCapInstance *cap = SuperCapInit(&cap_conf);
```

周期任务中读取反馈：

```c
SuperCap_Msg_s cap_msg;

if (SuperCapIsOnline(cap) != 0U && SuperCapGet(cap, &cap_msg) != 0U)
{
    /* 在这里使用 cap_msg.vol / cap_msg.current / cap_msg.power */
}
```

发送控制数据：

```c
uint8_t tx_data[8] = {0};

(void)SuperCapSend(cap, tx_data, sizeof(tx_data));
```

## 注意事项

- 当前工程只支持一台超级电容，重复调用 `SuperCapInit()` 会返回已有实例。
- 发送接口不会解释控制协议，只负责把调用者提供的 1-8 字节数据通过 CAN 发出。
- `SuperCapRxCallback()` 运行在 `CANProcessTask` 中，不在 FDCAN 中断中直接解析上层协议。
- 上层如果要把电容反馈用于底盘功率限制，应先判断 `SuperCapIsOnline()`，离线时不能继续信任最后一次反馈值。
