/**
 * @file bsp_init.h
 * @brief BSP层初始化边界。
 *
 * @note BSPInit()用于调度器启动前的基础资源初始化;
 *       BSPTaskInit()用于FreeRTOS内核初始化后创建BSP后台任务和RTOS对象。
 */
#ifndef BSP_INIT_H
#define BSP_INIT_H

/**
 * @brief 初始化调度器启动前必须存在的BSP基础资源。
 */
void BSPInit(void);


/**
 * @brief 创建BSP运行期资源和后台任务。
 */
void BSPTaskInit(void);

#endif // BSP_INIT_H
