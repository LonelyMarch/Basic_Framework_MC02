# bsp_adc

`bsp_adc` 是 ADC DMA 循环采样的最小 BSP 封装。CubeMX 负责 ADC、GPIO、DMA 的底层初始化,BSP 负责注册实例、启动/停止
DMA、读取原始值和换算电压。

## 职责边界

- 只管理 MCU 片上 ADC 外设。
- 不负责电阻分压、传感器标定、滤波和业务量纲换算。
- 不在中断中输出日志,只记录转换次数和错误状态。

## 注册与启动

```c
ADC_Init_Config_s adc_conf = {
    .adc_handle = &hadc1,
    .channel_count = 2,
    .vref = 3.3f,
};

ADCInstance *adc = ADCRegister(&adc_conf);
ADCStart(adc);
```

`channel_count` 必须和 CubeMX 中 ADC regular conversion 数量一致。读取时的 `channel_index` 从 0 开始,对应 CubeMX 的
regular rank 顺序。

## DMA缓冲区

STM32H7 的 DMA1/DMA2 不能访问 DTCM。ADC DMA 循环缓冲区由 BSP 内部提供,位于 `.dma_buffer` / `RAM_D2`。

若开启 D-Cache,BSP 在启动和读取时会对 DMA 缓冲区做 cache 维护。若后续通过 MPU 将 `.dma_buffer` 配置为 non-cacheable,可将
`BSP_ADC_USE_DMA_CACHE_MAINTENANCE` 设为 `0U`。

## 接口

```c
ADCInstance *ADCRegister(ADC_Init_Config_s *config);
HAL_StatusTypeDef ADCStart(ADCInstance *instance);
HAL_StatusTypeDef ADCStop(ADCInstance *instance);
uint16_t ADCGetRaw(ADCInstance *instance, uint8_t channel_index);
float ADCGetVoltage(ADCInstance *instance, uint8_t channel_index);
uint32_t ADCGetUpdateCount(ADCInstance *instance);
uint32_t ADCGetErrorCount(ADCInstance *instance);
uint32_t ADCGetLastError(ADCInstance *instance);
```

`ADCGetVoltage()` 使用注册时传入的 `vref`。如果 `vref <= 0`,使用 `BSP_ADC_DEFAULT_VREF`。

## FreeRTOS约束

ADC DMA 启动后硬件持续更新内部缓冲区,上层任务可以周期读取。HAL ADC 完成回调只递增 `update_count`,错误回调只记录
`error_count` 和 `last_error`。

## 注意事项

- 当前实现适合简单多通道循环采样。
- 如果需要精确同步触发、过采样、校准或复杂滤波,应放在模块层或单独扩展 ADC BSP。
- `ADCRegister()` 失败返回 `NULL`,上层必须检查。
