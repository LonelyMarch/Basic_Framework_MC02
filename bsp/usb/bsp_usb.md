# bsp_usb

`bsp_usb` 封装 USB CDC 虚拟串口,用于调试输出、VOFA 数据输出,以及可选的日志 USB 后端。

## 初始化

```c
USB_Init_Config_s conf = {
    .rx_callback = DebugRxCallback,
    .tx_callback = DebugTxDone,
};

USBInit(&conf);
```

`USBInit()` 只注册 BSP 回调并清理内部状态。USB 设备栈仍由 CubeMX 生成的 `MX_USB_DEVICE_Init()` 启动。

## 发送

```c
USBTransmit(data, len);
USBPrintf("ch0:%d,ch1:%d\r\n", ch0, ch1);
```

USB CDC 发送是异步完成。BSP 会先把上层数据复制到内部发送缓冲区,因此调用者可以传入局部数组。上一帧未发送完成时返回
`USBD_BUSY`。

## 接收

USB CDC 底层接收完成后由 `usbd_cdc_if.c` 调用 `USB_CDC_RxCpltCallback()`。该函数只把数据复制进 `BSPFrameQueue`
管理的接收帧缓存,然后唤醒 `BSPServiceTask`。

真正的 `rx_callback` 由 `USBProcess()` 在任务上下文调用。

## 缓冲区

- `USB_TX_BUFFER_SIZE`: 单次发送缓存大小。
- `USB_RX_BUFFER_SIZE`: 单次接收帧最大长度。
- `USB_RX_FRAME_BUFFER_CNT`: 接收帧缓存槽位数量。
- `USB_PRINTF_BUFFER_SIZE`: `USBPrintf()` 格式化临时缓存。

当前 USB PCD DMA 关闭,USB 缓冲区不强制放入 `.dma_buffer`。若后续开启 USB DMA,需要重新检查 STM32H7 内存域和 D-Cache 维护。

## FreeRTOS约束

- 接收解析不在 USB 中断中执行。
- `tx_callback` 也通过 `BSPServiceTask` 延后到任务上下文执行。
- `USBTransmit()` 可在任务中调用,返回 `USBD_BUSY` 表示本次发送未启动。

## 注意事项

- USB CDC 是否可用取决于 PC 端枚举状态,可用 `USBIsReady()` 判断。
- 默认日志仍走 RTT。若打开 `BSP_LOG_USE_USB`,需要确认 USB 没有被视觉或 VOFA 高频通道占满。
