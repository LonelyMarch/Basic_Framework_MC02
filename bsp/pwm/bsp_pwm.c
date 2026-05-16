#include "bsp_pwm.h"
#include "bsp_log.h"
#include "stdlib.h"
#include "memory.h"

// 配合中断以及初始化
static uint8_t idx;
static PWMInstance *pwm_instance[PWM_DEVICE_CNT] = {NULL}; // 所有的pwm instance保存于此,用于callback时判断中断来源
static uint8_t PWMIsAPB1Timer(TIM_TypeDef *tim);
static uint8_t PWMIsAPB2Timer(TIM_TypeDef *tim);
static uint8_t PWMIs32BitTimer(TIM_TypeDef *tim);
static uint32_t PWMGetTimerClock(uint32_t pclk, uint32_t apb_prescaler);
static uint32_t PWMSelectTclk(TIM_HandleTypeDef *htim);
static uint8_t PWMAddInstance(PWMInstance *pwm);
/**
 * @brief pwm dma传输完成回调函数
 *
 * @param htim 发生中断的定时器句柄
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    for (uint8_t i = 0; i < idx; i++)
    { // 来自同一个定时器的中断且通道相同
        if (pwm_instance[i]->htim == htim && htim->Channel == (1<<(pwm_instance[i]->channel/4)))
        {
            if (pwm_instance[i]->callback) // 如果有回调函数
                pwm_instance[i]->callback(pwm_instance[i]);
            return; // 一次只能有一个通道的中断,所以直接返回
        }
    }
}

PWMInstance *PWMRegister(PWM_Init_Config_s *config)
{
    if (config == NULL || config->htim == NULL)
    {
        LOGERROR("[bsp_pwm] PWMRegister received null config");
        Error_Handler();
        return NULL;
    }

    if (idx >= PWM_DEVICE_CNT) // 超过最大实例数,考虑增加或查看是否有内存泄漏
    {
        LOGERROR("[bsp_pwm] pwm instance count exceeds limit");
        Error_Handler();
        return NULL;
    }

    PWMInstance *pwm = (PWMInstance *)malloc(sizeof(PWMInstance));
    if (pwm == NULL)
    {
        LOGERROR("[bsp_pwm] pwm instance malloc failed");
        Error_Handler();
        return NULL;
    }

    memset(pwm, 0, sizeof(PWMInstance));

    pwm->htim = config->htim;
    pwm->channel = config->channel;
    pwm->period = config->period;
    pwm->dutyratio = config->dutyratio;
    pwm->callback = config->callback;
    pwm->id = config->id;
    pwm->tclk = PWMSelectTclk(pwm->htim);
    if (pwm->tclk == 0U)
    {
        LOGERROR("[bsp_pwm] unsupported timer instance");
        free(pwm);
        Error_Handler();
        return NULL;
    }

    // 启动PWM
    if (HAL_TIM_PWM_Start(pwm->htim, pwm->channel) != HAL_OK)
    {
        LOGERROR("[bsp_pwm] pwm start failed");
        free(pwm);
        Error_Handler();
        return NULL;
    }

    PWMSetPeriod(pwm, pwm->period);
    PWMSetDutyRatio(pwm, pwm->dutyratio);

    if (PWMAddInstance(pwm) == 0U)
    {
        LOGERROR("[bsp_pwm] pwm instance count exceeds limit");
        (void)HAL_TIM_PWM_Stop(pwm->htim, pwm->channel);
        free(pwm);
        Error_Handler();
        return NULL;
    }

    return pwm;
}

/* 只是对HAL的函数进行了形式上的封装 */
void PWMStart(PWMInstance *pwm)
{
    if (pwm == NULL || pwm->htim == NULL)
    {
        LOGERROR("[bsp_pwm] PWMStart received null instance");
        return;
    }

    if (HAL_TIM_PWM_Start(pwm->htim, pwm->channel) != HAL_OK)
    {
        LOGERROR("[bsp_pwm] pwm start failed");
        Error_Handler();
    }
}

/* 只是对HAL的函数进行了形式上的封装 */
void PWMStop(PWMInstance *pwm)
{
    if (pwm == NULL || pwm->htim == NULL)
    {
        LOGERROR("[bsp_pwm] PWMStop received null instance");
        return;
    }

    if (HAL_TIM_PWM_Stop(pwm->htim, pwm->channel) != HAL_OK)
    {
        LOGERROR("[bsp_pwm] pwm stop failed");
        Error_Handler();
    }
}

/*
 * @brief 设置pwm周期
 *
 * @param pwm pwm实例
 * @param period 周期 单位 s
 */
void PWMSetPeriod(PWMInstance *pwm, float period)
{
    if (pwm == NULL || pwm->htim == NULL)
    {
        LOGERROR("[bsp_pwm] PWMSetPeriod received null instance");
        return;
    }

    if (period <= 0.0f)
    {
        LOGERROR("[bsp_pwm] invalid pwm period");
        return;
    }

    if (pwm->tclk == 0U)
    {
        LOGERROR("[bsp_pwm] invalid timer clock");
        return;
    }

    uint32_t counter_clk = pwm->tclk / (pwm->htim->Init.Prescaler + 1U);
    if (counter_clk == 0U)
    {
        LOGERROR("[bsp_pwm] invalid timer counter clock");
        return;
    }

    float ticks_float = period * (float)counter_clk;
    if (ticks_float < 1.0f)
    {
        ticks_float = 1.0f;
    }

    uint32_t max_arr = PWMIs32BitTimer(pwm->htim->Instance) ? UINT32_MAX : UINT16_MAX;
    float max_ticks_float = (float)max_arr + 1.0f;

    if (ticks_float > max_ticks_float)
    {
        LOGERROR("[bsp_pwm] pwm period exceeds timer ARR range");
        __HAL_TIM_SetAutoreload(pwm->htim, max_arr);
        return;
    }

    if (ticks_float >= max_ticks_float)
    {
        __HAL_TIM_SetAutoreload(pwm->htim, max_arr);
        return;
    }

    // TIM更新周期为(ARR + 1)个计数周期,所以由目标周期反推ARR时需要减1
    uint32_t ticks = (uint32_t)(ticks_float + 0.5f);
    __HAL_TIM_SetAutoreload(pwm->htim, ticks - 1U);
}
/*
    * @brief 设置pwm占空比
    *
    * @param pwm pwm实例
    * @param dutyratio 占空比 0~1
*/
void PWMSetDutyRatio(PWMInstance *pwm, float dutyratio)
{
    if (pwm == NULL || pwm->htim == NULL)
    {
        LOGERROR("[bsp_pwm] PWMSetDutyRatio received null instance");
        return;
    }

    if (dutyratio <= 0.0f)
    {
        __HAL_TIM_SetCompare(pwm->htim, pwm->channel, 0U);
        return;
    }

    // PWM一个完整周期包含(ARR + 1)个计数值,按周期计数值计算CCR才能覆盖100%占空比
    uint64_t period_ticks = (uint64_t)pwm->htim->Instance->ARR + 1ULL;

    if (dutyratio >= 1.0f)
    {
        uint32_t compare = period_ticks > UINT32_MAX ? UINT32_MAX : (uint32_t)period_ticks;
        __HAL_TIM_SetCompare(pwm->htim, pwm->channel, compare);
        return;
    }

    uint32_t compare = (uint32_t)(dutyratio * (float)period_ticks + 0.5f);
    __HAL_TIM_SetCompare(pwm->htim, pwm->channel, compare);
}

/* 只是对HAL的函数进行了形式上的封装 */
void PWMStartDMA(PWMInstance *pwm, uint32_t *pData, uint32_t Size)
{
    if (pwm == NULL || pwm->htim == NULL)
    {
        LOGERROR("[bsp_pwm] PWMStartDMA received null instance");
        return;
    }

    if (pData == NULL || Size == 0U)
    {
        LOGERROR("[bsp_pwm] invalid pwm dma buffer");
        return;
    }

    if (HAL_TIM_PWM_Start_DMA(pwm->htim, pwm->channel, pData, Size) != HAL_OK)
    {
        LOGERROR("[bsp_pwm] pwm dma start failed");
        Error_Handler();
    }
}

// 判断定时器是否挂载在 APB1 定时器时钟域
static uint8_t PWMIsAPB1Timer(TIM_TypeDef *tim)
{
    return 0
#ifdef TIM2
           || (tim == TIM2)
#endif
#ifdef TIM3
           || (tim == TIM3)
#endif
#ifdef TIM4
           || (tim == TIM4)
#endif
#ifdef TIM5
           || (tim == TIM5)
#endif
#ifdef TIM12
           || (tim == TIM12)
#endif
#ifdef TIM13
           || (tim == TIM13)
#endif
#ifdef TIM14
           || (tim == TIM14)
#endif
#ifdef TIM23
           || (tim == TIM23)
#endif
#ifdef TIM24
           || (tim == TIM24)
#endif
           ;
}

// 判断定时器是否挂载在 APB2 定时器时钟域
static uint8_t PWMIsAPB2Timer(TIM_TypeDef *tim)
{
    return 0
#ifdef TIM1
           || (tim == TIM1)
#endif
#ifdef TIM8
           || (tim == TIM8)
#endif
#ifdef TIM15
           || (tim == TIM15)
#endif
#ifdef TIM16
           || (tim == TIM16)
#endif
#ifdef TIM17
           || (tim == TIM17)
#endif
           ;
}

// 判断定时器是否为32位计数器,其余普通PWM定时器按16位ARR处理
static uint8_t PWMIs32BitTimer(TIM_TypeDef *tim)
{
    return 0
#ifdef TIM2
           || (tim == TIM2)
#endif
#ifdef TIM5
           || (tim == TIM5)
#endif
#ifdef TIM23
           || (tim == TIM23)
#endif
#ifdef TIM24
           || (tim == TIM24)
#endif
           ;
}

// 根据 STM32H7 的 TIMPRE 规则计算定时器实际输入时钟
static uint32_t PWMGetTimerClock(uint32_t pclk, uint32_t apb_prescaler)
{
    uint32_t hclk = HAL_RCC_GetHCLKFreq();

    if ((RCC->CFGR & RCC_CFGR_TIMPRE) == 0U)
    {
        // TIMPRE=0: APB分频为1或2时定时器时钟等于HCLK,否则等于2倍PCLK
        if ((apb_prescaler == RCC_D2CFGR_D2PPRE1_DIV1) ||
            (apb_prescaler == RCC_D2CFGR_D2PPRE1_DIV2) ||
            (apb_prescaler == RCC_D2CFGR_D2PPRE2_DIV1) ||
            (apb_prescaler == RCC_D2CFGR_D2PPRE2_DIV2))
        {
            return hclk;
        }

        return pclk * 2U;
    }

    // TIMPRE=1: APB分频为1/2/4时定时器时钟等于HCLK,否则等于4倍PCLK
    if ((apb_prescaler == RCC_D2CFGR_D2PPRE1_DIV1) ||
        (apb_prescaler == RCC_D2CFGR_D2PPRE1_DIV2) ||
        (apb_prescaler == RCC_D2CFGR_D2PPRE1_DIV4) ||
        (apb_prescaler == RCC_D2CFGR_D2PPRE2_DIV1) ||
        (apb_prescaler == RCC_D2CFGR_D2PPRE2_DIV2) ||
        (apb_prescaler == RCC_D2CFGR_D2PPRE2_DIV4))
    {
        return hclk;
    }

    return pclk * 4U;
}

// 设置pwm对应定时器时钟源频率
static uint32_t PWMSelectTclk(TIM_HandleTypeDef *htim)
{
    if (htim == NULL)
    {
        return 0;
    }

    if (PWMIsAPB1Timer(htim->Instance))
    {
        return PWMGetTimerClock(HAL_RCC_GetPCLK1Freq(), RCC->D2CFGR & RCC_D2CFGR_D2PPRE1);
    }

    if (PWMIsAPB2Timer(htim->Instance))
    {
        return PWMGetTimerClock(HAL_RCC_GetPCLK2Freq(), RCC->D2CFGR & RCC_D2CFGR_D2PPRE2);
    }

    return 0;
}

// 将PWM实例加入全局注册表,只在极短临界区内保护idx和实例数组
static uint8_t PWMAddInstance(PWMInstance *pwm)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (idx >= PWM_DEVICE_CNT)
    {
        __set_PRIMASK(primask);
        return 0U;
    }

    pwm_instance[idx] = pwm;
    idx++;
    __set_PRIMASK(primask);

    return 1U;
}
