# bsp_pwm

`bsp_pwm` 封装 TIM PWM 输出,负责实例注册、PWM 启停、周期设置、占空比设置和 PWM DMA 完成回调分发。

## 注册

```c
PWM_Init_Config_s conf = {
    .htim = &htim3,
    .channel = TIM_CHANNEL_4,
    .period = 0.001f,
    .dutyratio = 0.5f,
};

PWMInstance *pwm = PWMRegister(&conf);
```

注册成功后 BSP 会启动对应 PWM 通道,并按配置设置周期和占空比。实例控制结构体来自静态池。

## 周期与占空比

```c
void PWMSetPeriod(PWMInstance *pwm, float period);
void PWMSetDutyRatio(PWMInstance *pwm, float dutyratio);
```

`period` 单位为秒。BSP 会根据 TIM 所在 APB 时钟域、APB 分频和 TIMPRE 规则计算定时器输入时钟。

定时器实际周期为 `(ARR + 1) / counter_clk`,因此 BSP 设置 ARR 时会按目标 tick 数减 1。

`dutyratio` 范围为 `0~1`。小于等于 0 输出 0%,大于等于 1 输出 100%。

## 定时器支持

当前按 STM32H723 的定时器分布识别 APB1/APB2 PWM 定时器,并用 `#ifdef TIMx` 保护不同芯片头文件中的定时器宏。

32 位定时器会按 32 位 ARR 计算周期上限,其余定时器按 16 位 ARR 计算。

## DMA输出

```c
void PWMStartDMA(PWMInstance *pwm, uint32_t *pData, uint32_t Size);
```

PWM DMA 完成后,HAL 回调会根据 `TIM_HandleTypeDef` 和通道找到对应 `PWMInstance`,再调用注册时传入的 `callback`。

DMA 数据宽度需要和 CubeMX 中定时器 DMA 配置一致。

## FreeRTOS约束

PWM 注册通常在模块初始化阶段完成。注册表发布时使用短临界区,避免中断回调读到半初始化实例。

## 注意事项

- PWM 引脚复用、TIM 基本参数和 DMA 通道仍由 CubeMX 配置。
- 不建议在 PWM DMA 完成回调中执行耗时逻辑。
- 注册失败或硬件启动失败属于初始化阶段严重错误,当前会输出日志并进入 `Error_Handler()`。
