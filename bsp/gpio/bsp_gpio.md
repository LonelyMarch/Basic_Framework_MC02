# bsp_gpio

`bsp_gpio` 对 HAL GPIO 做轻量封装,用于普通输入输出控制和 EXTI 外部中断回调分发。

## 注册

```c
GPIO_Init_Config_s conf = {
    .GPIOx = GPIOA,
    .GPIO_Pin = GPIO_PIN_0,
    .exti_mode = GPIO_EXTI_MODE_RISING,
    .gpio_model_callback = MyExtiCallback,
    .id = module,
};

GPIOInstance *gpio = GPIORegister(&conf);
```

注册表使用静态实例池,不依赖 heap。EXTI 引脚会检查重复注册,避免同一条 EXTI 线被多个实例同时认领。

## 普通GPIO接口

```c
void GPIOSet(GPIOInstance *instance);
void GPIOReset(GPIOInstance *instance);
void GPIOToggle(GPIOInstance *instance);
GPIO_PinState GPIORead(GPIOInstance *instance);
```

这些接口统一使用 `GPIOInstance *` 作为入口,上层不直接散落 `HAL_GPIO_WritePin()`。

## EXTI流程

HAL 的 `HAL_GPIO_EXTI_Callback()` 只有 `GPIO_Pin` 参数,没有 `GPIOx`。在 STM32 EXTI 映射中,同一条 EXTI 线只能连接一个端口的同号引脚,因此只要 CubeMX 配置不冲突,即可通过 `GPIO_Pin` 定位注册实例。

当前流程为:

1. EXTI 中断触发。
2. `HAL_GPIO_EXTI_Callback()` 查找对应 `GPIOInstance`。
3. ISR 只向 `BSPServiceTask` 投递延后事件。
4. `gpio_model_callback()` 在任务上下文执行。

## FreeRTOS约束

GPIO 模块回调不直接运行在中断中,因此可以做普通状态更新和轻量模块逻辑。但仍不建议在回调中执行长时间阻塞操作。

## 注意事项

- GPIO 模式、上下拉、EXTI触发沿仍由 CubeMX 配置。
- `exti_mode` 主要用于注册语义检查和文档表达,不会动态重配硬件寄存器。
- 注册失败返回 `NULL`,上层必须检查。
