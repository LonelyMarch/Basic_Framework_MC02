GPIO BSP 对 HAL GPIO 做轻量封装,可用于普通输入输出,也可用于 EXTI 外部中断回调分发。

## 实例注册

使用 `GPIORegister()` 注册一个 GPIO 实例。注册成功返回 `GPIOInstance *`,注册失败返回 `NULL` 并通过 `LOGERROR` 输出原因。

注册失败的常见原因:

- 配置指针为空
- `GPIOx` 为空
- `GPIO_Pin` 不是有效的 `GPIO_PIN_x` 宏
- 注册实例数量超过 `GPIO_MX_DEVICE_NUM`
- 内存分配失败
- 同一 EXTI line 被重复注册

`GPIO_Pin` 使用 HAL 提供的 `GPIO_PIN_0`、`GPIO_PIN_1` 等宏。GPIO 的端口、方向、上下拉、输出速度、EXTI 边沿等硬件配置仍由 CubeMX 生成的 `MX_GPIO_Init()` 完成,这里的 `exti_mode` 只用于 BSP 层记录和重复 EXTI 注册检查,需要和 CubeMX 配置保持一致。

## EXTI回调

`HAL_GPIO_EXTI_Callback()` 会根据 `GPIO_Pin` 查找已注册的 `GPIOInstance`,并调用实例中的 `gpio_model_callback`。

STM32 的 EXTI 按 pin number 分线,例如 `PA10/PB10/PE10` 都对应 `EXTI10`,同一条 EXTI line 同一时刻只能映射到一个 GPIO 端口。因此 BSP 层禁止同一个 `GPIO_Pin` 重复注册为 EXTI 实例。

## FreeRTOS环境下的中断回调约束

`gpio_model_callback` 会在 `HAL_GPIO_EXTI_Callback()` 中被直接调用,因此它运行在 EXTI 中断上下文中。回调函数应只做很短的处理,例如置位标志、记录时间戳、或调用允许在中断中使用的 RTOS 通知接口。

不要在 `gpio_model_callback` 中执行阻塞等待、普通 `osDelay()`、普通互斥锁/信号量等待、长时间 SPI/I2C/UART 通信、动态内存申请释放或复杂日志输出。需要处理较复杂逻辑时,建议在回调中通知任务,再由任务完成实际处理。

## API

```c
GPIOInstance *GPIORegister(GPIO_Init_Config_s *GPIO_config);
void GPIOToggle(GPIOInstance *_instance);
void GPIOSet(GPIOInstance *_instance);
void GPIOReset(GPIOInstance *_instance);
GPIO_PinState GPIORead(GPIOInstance *_instance);
```

## 使用示例

```c
GPIO_Init_Config_s gpio_init = {
    .exti_mode = GPIO_EXTI_MODE_NONE,
    .GPIO_Pin = GPIO_PIN_6,
    .GPIOx = GPIOG,
    .gpio_model_callback = NULL,
};

GPIOInstance *test_gpio = GPIORegister(&gpio_init);
if (test_gpio != NULL)
{
    GPIOSet(test_gpio);
    GPIOToggle(test_gpio);
}
```
