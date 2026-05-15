#ifndef BSP_INIT_h
#define BSP_INIT_h

#include "bsp_log.h"
#include "bsp_dwt.h"
#include "tim.h"
//#include "bsp_usb.h"

/**
 * @brief bsp层初始化统一入口,这里仅初始化必须的bsp组件,其他组件的初始化在各自的模块中进行
 *        需在实时系统启动前调用,目前由RobotoInit()调用
 * 
 * @note 其他实例型的外设如CAN和串口会在注册实例的时候自动初始化,不注册不初始化
  */
// 
void BSPInit()
{
    // DWT按CPU内核时钟计数,这里使用SystemCoreClock换算MHz,不要使用HCLK
    DWT_Init(SystemCoreClock / 1000000U);
    BSPLogInit();
    // 启动TIM6周期中断,每1s刷新一次DWT时间轴,避免CYCCNT长时间未读取导致溢出漏计
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
    {
        LOGERROR("[bsp_init] TIM6 start failed");
        Error_Handler();
    }
}


#endif // !BSP_INIT_h
