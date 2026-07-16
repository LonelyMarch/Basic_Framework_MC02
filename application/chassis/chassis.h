/**
 * @file chassis.h
 * @brief 底盘 APP 空框架。
 */

#ifndef CHASSIS_H
#define CHASSIS_H

/**
 * @brief 初始化底盘使用的消息、传感器引用和执行器实例。
 */
void ChassisInit(void);


/**
 * @brief 执行一次底盘状态机、运动学与目标更新。
 */
void ChassisTask(void);

#endif // CHASSIS_H
