# bsp pwm

`bsp_pwm` 对 STM32 HAL 的 PWM 接口做了一层注册式封装。上层模块通过 `PWMRegister()` 注册一个 `PWMInstance`，之后使用该实例设置周期、占空比、启停 PWM 或启动 PWM DMA。

## 基本流程

1. 在 CubeMX 中完成对应 TIM、PWM Channel、GPIO 复用、Prescaler、DMA 等底层配置。
2. 在模块初始化阶段填写 `PWM_Init_Config_s`。
3. 调用 `PWMRegister()` 注册实例。
4. 调用 `PWMSetPeriod()`、`PWMSetDutyRatio()`、`PWMStart()`、`PWMStop()` 或 `PWMStartDMA()` 控制 PWM。

示例：

```c
PWM_Init_Config_s config = {
    .htim = &htim12,
    .channel = TIM_CHANNEL_2,
    .period = 0.001f,
    .dutyratio = 0.0f,
    .callback = NULL,
    .id = NULL,
};

PWMInstance *pwm = PWMRegister(&config);
PWMSetDutyRatio(pwm, 0.5f);
```

## 配置结构体

`PWM_Init_Config_s` 字段含义：

- `htim`：CubeMX 生成的 TIM 句柄，例如 `&htim3`、`&htim12`。
- `channel`：PWM 通道，例如 `TIM_CHANNEL_1`、`TIM_CHANNEL_4`。
- `period`：PWM 周期，单位为秒。
- `dutyratio`：初始占空比，范围为 `0~1`。
- `callback`：PWM DMA 传输完成回调函数，不使用 DMA 时可设为 `NULL`。
- `id`：上层模块自定义指针，用于保存父对象或用户数据。

## 周期与占空比

`PWMSetPeriod()` 的 `period` 单位是秒。函数会根据 TIM 所在 APB 总线、当前 RCC 时钟、TIMPRE 规则和定时器 Prescaler 计算 ARR。

定时器实际周期关系为：

```text
PWM周期 = (ARR + 1) / 定时器计数频率
```

`PWMSetDutyRatio()` 的 `dutyratio` 使用 `0~1` 语义：

- 小于等于 `0` 时输出 `0%`。
- 大于等于 `1` 时输出 `100%`。
- 中间值按 `(ARR + 1) * dutyratio` 写入 CCR。

如果同一个 TIM 下有多个 PWM 通道，修改其中一个实例的周期会改变该 TIM 的 ARR，因此会影响同一个 TIM 下的其他 PWM 通道。

## 定时器支持

当前代码按 STM32H723 的普通 PWM TIM 进行时钟归类：

- APB1：`TIM2`、`TIM3`、`TIM4`、`TIM5`、`TIM12`、`TIM13`、`TIM14`、`TIM23`、`TIM24`
- APB2：`TIM1`、`TIM8`、`TIM15`、`TIM16`、`TIM17`

其中 `TIM2`、`TIM5`、`TIM23`、`TIM24` 按 32 位 ARR 处理，其余普通 PWM TIM 按 16 位 ARR 处理。

`TIM6/TIM7` 是 basic timer，没有 PWM 输出通道；`LPTIMx` 不属于当前 `TIM_HandleTypeDef + HAL_TIM_PWM_Start()` 封装路径。

## DMA 回调

使用 `PWMStartDMA()` 时，需要在 CubeMX 中为对应 TIM Channel 配置 DMA，并保证 DMA 传输数据宽度和 `pData` 指向的数据宽度一致。

HAL 的 `HAL_TIM_PWM_PulseFinishedCallback()` 会遍历已注册的 `PWMInstance`，根据 TIM 句柄和通道匹配来源。如果实例配置了 `callback`，则调用：

```c
pwm_instance->callback(pwm_instance);
```

注意：PWM DMA 传输中如果占空比数据为 0，可能不会产生预期的 PWM 脉冲完成行为，应结合具体 TIM/DMA 配置确认。

## FreeRTOS 注意事项

`PWMRegister()` 内部会把实例保存到全局注册表，当前注册表容量为 `PWM_DEVICE_CNT`。注册表写入时使用极短临界区保护 `idx` 和实例数组，避免多个任务同时注册 PWM 时写乱全局表。

建议仍然尽量在系统初始化阶段完成 PWM 注册；任务运行期间可以调用 `PWMSetDutyRatio()`、`PWMSetPeriod()` 等接口更新输出。

## 错误处理

当前模块会检查空指针、实例数量、内存申请、未知定时器、HAL 启停失败、非法周期、DMA 参数等异常，并通过 `LOGERROR` 输出错误信息。部分初始化失败会调用 `Error_Handler()`。
