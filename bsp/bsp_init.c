#include "bsp_init.h"

#include "bsp_dwt.h"
#include "bsp_can.h"
#include "bsp_flash.h"
#include "bsp_flash_async.h"
#include "bsp_iic.h"
#include "bsp_log.h"
#include "bsp_service.h"
#include "bsp_spi.h"
#include "cmsis_os2.h"
#include "main.h"
#include "tim.h"

static osThreadId_t can_process_task_handle;
static osThreadId_t bsp_service_task_handle;
static osThreadId_t flash_async_task_handle;

/**
 * @brief BSP层调度器启动前初始化入口,这里只初始化必须先存在的基础资源。
 *
 * @note 需在实时系统启动前调用,目前由RobotInit()调用。
 *       这里不创建RTOS线程,也不做模块层实例注册。
 *       其他实例型外设如CAN、USART、SPI、IIC等会在模块初始化时按需注册。
 */
void BSPInit(void)
{
    // DWT按CPU内核时钟计数,这里使用SystemCoreClock换算MHz,不要使用HCLK。
    DWT_Init(SystemCoreClock / 1000000U);
    BSPLogInit();
    BSPServiceInit();
    if (BSP_Flash_Init() != BSP_FLASH_OK)
    {
        LOGERROR("[bsp_init] on-chip flash BSP init failed");
        Error_Handler();
    }
    BSP_FlashAsyncInit();

    // 启动TIM6周期中断,每1s刷新一次DWT时间轴,避免CYCCNT长时间未读取导致溢出漏计。
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
    {
        LOGERROR("[bsp_init] TIM6 start failed");
        Error_Handler();
    }
}

/**
 * @brief 创建BSP运行期资源和后台任务。
 *
 * @note 该函数由application层在创建业务任务时调用一次。
 *       此时osKernelInitialize()已经完成,可以创建mutex/semaphore/thread等RTOS对象。
 *       APP 和模块管理任务由 application/robot_task.c 统一创建。
 */
void BSPTaskInit(void)
{
    const osThreadAttr_t can_process_task_attr = {
        .name = "cantask",
        .stack_size = 512 * 4,
        .priority = osPriorityHigh,
    };

    const osThreadAttr_t bsp_service_task_attr = {
        .name = "bsptask",
        .stack_size = 512 * 4,
        .priority = osPriorityNormal,
    };

    const osThreadAttr_t flash_async_task_attr = {
        .name = "flashtask",
        .stack_size = 512 * 4,
        .priority = osPriorityLow,
    };

    if (SPIBusOsInit() != HAL_OK)
    {
        LOGERROR("[bsp_init] SPI bus RTOS objects init failed");
        Error_Handler();
    }

    if (IICBusOsInit() != HAL_OK)
    {
        LOGERROR("[bsp_init] IIC bus RTOS objects init failed");
        Error_Handler();
    }

    if (can_process_task_handle == NULL)
    {
        can_process_task_handle = osThreadNew(CANProcessTask, NULL, &can_process_task_attr);
        if (can_process_task_handle == NULL)
        {
            LOGERROR("[bsp_init] CAN process task create failed");
            Error_Handler();
        }
    }

    if (bsp_service_task_handle == NULL)
    {
        bsp_service_task_handle = osThreadNew(BSPServiceTask, NULL, &bsp_service_task_attr);
        if (bsp_service_task_handle == NULL)
        {
            LOGERROR("[bsp_init] BSP service task create failed");
            Error_Handler();
        }
    }

    if (flash_async_task_handle == NULL)
    {
        flash_async_task_handle = osThreadNew(BSP_FlashAsyncTask, NULL, &flash_async_task_attr);
        if (flash_async_task_handle == NULL)
        {
            LOGERROR("[bsp_init] flash async task create failed");
            Error_Handler();
        }
    }
}
