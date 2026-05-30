# TFmini Plus

`modules/TFminiPlus` 是北醒 TFmini Plus 单点激光雷达的模块层封装。模块通过 `bsp/iic` 访问传感器,负责注册 I2C 从设备、发送测距命令、读取并校验数据帧,再向上层提供最近一次有效距离和信号强度。

当前目录尚未加入主工程 `CMakeLists.txt`,需要实际使用时再接入构建和上层任务。

## 初始化

```c
TFMiniPlus_Init_Config_s config = {
    .hi2c = &hi2c2,
    .work_mode = IIC_BLOCK_MODE,
};

TFMiniPlusInstance *tfmini = TFMiniPlusRegister(&config);
```

- `hi2c` 使用 CubeMX 生成的 I2C 句柄。
- `work_mode` 可选择 `IIC_BLOCK_MODE`、`IIC_IT_MODE` 或 `IIC_DMA_MODE`。
- 当前工程 I2C2 已开启 EV/ER 中断,可使用阻塞或中断模式。若要使用 DMA 模式,需要先在 CubeMX 中为 I2C2 配置 TX/RX DMA。
- 注册阶段会等待 `TFMINIPLUS_POWER_ON_DELAY_MS`,若 FreeRTOS 调度器已经运行则使用 `osDelay()`,否则使用 `DWT_Delay()`。

## 读取数据

```c
if (TFMiniPlusRead(tfmini) == HAL_OK)
{
    uint16_t distance = TFMiniPlusGetDistance(tfmini);
    uint16_t strength = TFMiniPlusGetStrength(tfmini);
}
```

`TFMiniPlusRead()` 会先发送获取测距数据命令,再读取 `TFMINIPLUS_FRAME_LEN` 字节原始帧。读取成功后会检查帧头和校验和,只有校验通过才更新实例中的距离、强度和在线状态。

## 状态接口

```c
uint8_t online = TFMiniPlusIsOnline(tfmini);
```

- `TFMiniPlusGetDistance()` 返回最近一次有效距离值。
- `TFMiniPlusGetStrength()` 返回最近一次有效信号强度。
- `TFMiniPlusIsOnline()` 返回最近一次读取是否成功。
- `error_count` 保存在实例中,用于调试观察读取错误次数。

## 注意事项

- TFmini Plus 的 I2C 地址宏为 `TFMINIPLUS_I2C_ADDR`,传入 BSP 时使用 7 位地址,不需要左移。
- I2C 总线互斥、IT/DMA 完成等待由 `bsp/iic` 负责。
- 不建议在中断回调里直接调用 `TFMiniPlusRead()`；应在任务中周期读取,或由中断只通知任务读取。
