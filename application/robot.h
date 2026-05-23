#ifndef ROBOT_H
#define ROBOT_H

/* Robot利用robot_def.h中的宏对不同的机器人进行了大量的兼容,同时兼容了两个开发板(云台板和底盘板)的配置 */

/**
 * @brief 机器人硬件和模块初始化,请在开启RTOS调度器之前调用。
 * 
 */
void RobotInit(void);

/**
 * @brief 创建机器人和BSP相关RTOS任务,必须在osKernelInitialize()之后、osKernelStart()之前调用。
 *
 */
void RobotOSTaskInit(void);

/**
 * @brief 机器人任务,放入实时系统以一定频率运行,内部会调用各个应用的任务
 * 
 */
void RobotTask(void);

#endif
