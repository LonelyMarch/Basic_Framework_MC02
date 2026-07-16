/**
 * @file robot_task.h
 * @brief APP 层任务框架创建接口。
 */

#ifndef ROBOT_TASK_H
#define ROBOT_TASK_H

/**
 * @brief 创建电机管理、daemon 和 APP 控制任务。
 *
 * @note 必须在 osKernelInitialize() 之后、osKernelStart() 之前调用一次。
 */
void RobotTaskInit(void);

#endif // ROBOT_TASK_H
