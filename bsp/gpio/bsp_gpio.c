#include "bsp_gpio.h"
#include "bsp_log.h"
#include "memory.h"
#include "stdlib.h"

static uint8_t idx;
static GPIOInstance *gpio_instance[GPIO_MX_DEVICE_NUM] = {NULL};

static GPIOInstance *GPIOFindEXTIInstance(uint16_t GPIO_Pin)
{
    for (size_t i = 0; i < idx; i++)
    {
        GPIOInstance *gpio = gpio_instance[i];
        if (gpio == NULL)
        {
            continue;
        }

        if (gpio->exti_mode != GPIO_EXTI_MODE_NONE && gpio->GPIO_Pin == GPIO_Pin)
        {
            return gpio;
        }
    }

    return NULL;
}

/**
 * @brief EXTI中断回调函数,根据GPIO_Pin找到对应的GPIOInstance,并调用模块回调函数(如果有)
 * @note 如何判断具体是哪一个GPIO的引脚连接到这个EXTI中断线上?
 *       一个EXTI中断线只能连接一个GPIO引脚,因此可以通过GPIO_Pin来判断,PinX对应EXTIX
 *       一个Pin号只会对应一个EXTI,详情见gpio.md
 * @param GPIO_Pin 发生中断的GPIO_Pin
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // 如有必要,可以根据pinstate和HAL_GPIO_ReadPin来判断是上升沿还是下降沿/rise&fall等
    // 注意: gpio_model_callback运行在EXTI中断上下文,回调内不要执行阻塞或耗时操作
    GPIOInstance *gpio;
    for (size_t i = 0; i < idx; i++)
    {
        gpio = gpio_instance[i];
        if (gpio == NULL)
        {
            continue;
        }

        if (gpio->GPIO_Pin == GPIO_Pin && gpio->gpio_model_callback != NULL)
        {
            gpio->gpio_model_callback(gpio);
            return;
        }
    }
}

GPIOInstance *GPIORegister(GPIO_Init_Config_s *GPIO_config)
{
    if (GPIO_config == NULL)
    {
        LOGERROR("[bsp_gpio] GPIO注册失败: 配置指针为空");
        return NULL;
    }

    if (GPIO_config->GPIOx == NULL)
    {
        LOGERROR("[bsp_gpio] GPIO注册失败: GPIOx为空");
        return NULL;
    }

    if (!IS_GPIO_PIN(GPIO_config->GPIO_Pin))
    {
        LOGERROR("[bsp_gpio] GPIO注册失败: 无效引脚0x%x", (unsigned int)GPIO_config->GPIO_Pin);
        return NULL;
    }

    if (idx >= GPIO_MX_DEVICE_NUM)
    {
        LOGERROR("[bsp_gpio] GPIO注册失败: 实例数量超限, used:%u, limit:%u",
                 (unsigned int)idx,
                 (unsigned int)GPIO_MX_DEVICE_NUM);
        return NULL;
    }

    if (GPIO_config->exti_mode != GPIO_EXTI_MODE_NONE &&
        GPIOFindEXTIInstance(GPIO_config->GPIO_Pin) != NULL)
    {
        LOGERROR("[bsp_gpio] GPIO注册失败: EXTI引脚重复, pin:0x%x",
                 (unsigned int)GPIO_config->GPIO_Pin);
        return NULL;
    }

    GPIOInstance *ins = (GPIOInstance *)malloc(sizeof(GPIOInstance));
    if (ins == NULL)
    {
        LOGERROR("[bsp_gpio] GPIO注册失败: 内存分配失败");
        return NULL;
    }

    memset(ins, 0, sizeof(GPIOInstance));

    ins->GPIOx = GPIO_config->GPIOx;
    ins->GPIO_Pin = GPIO_config->GPIO_Pin;
    ins->pin_state = GPIO_config->pin_state;
    ins->exti_mode = GPIO_config->exti_mode;
    ins->id = GPIO_config->id;
    ins->gpio_model_callback = GPIO_config->gpio_model_callback;

    // 注册表会在EXTI中断回调中被读取,这里只保护发布实例的极短临界区
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (idx >= GPIO_MX_DEVICE_NUM)
    {
        __set_PRIMASK(primask);
        free(ins);
        LOGERROR("[bsp_gpio] GPIO注册失败: 实例数量超限, used:%u, limit:%u",
                 (unsigned int)idx,
                 (unsigned int)GPIO_MX_DEVICE_NUM);
        return NULL;
    }

    if (ins->exti_mode != GPIO_EXTI_MODE_NONE && GPIOFindEXTIInstance(ins->GPIO_Pin) != NULL)
    {
        __set_PRIMASK(primask);
        free(ins);
        LOGERROR("[bsp_gpio] GPIO注册失败: EXTI引脚重复, pin:0x%x",
                 (unsigned int)GPIO_config->GPIO_Pin);
        return NULL;
    }

    gpio_instance[idx] = ins;
    idx++;

    __set_PRIMASK(primask);
    return ins;
}

// ----------------- GPIO API -----------------
// 都是对HAL的形式上的封装,后续考虑增加GPIO state变量,可以直接读取state

void GPIOToggle(GPIOInstance *_instance)
{
    HAL_GPIO_TogglePin(_instance->GPIOx, _instance->GPIO_Pin);
}

void GPIOSet(GPIOInstance *_instance)
{
    HAL_GPIO_WritePin(_instance->GPIOx, _instance->GPIO_Pin, GPIO_PIN_SET);
}

void GPIOReset(GPIOInstance *_instance)
{
    HAL_GPIO_WritePin(_instance->GPIOx, _instance->GPIO_Pin, GPIO_PIN_RESET);
}

GPIO_PinState GPIORead(GPIOInstance *_instance)
{
    return HAL_GPIO_ReadPin(_instance->GPIOx, _instance->GPIO_Pin);
}
