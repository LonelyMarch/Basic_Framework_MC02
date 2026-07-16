/**
 * @file gimbal.h
 * @brief 云台 APP 空框架。
 */

#ifndef GIMBAL_H
#define GIMBAL_H

/**
 * @brief 初始化云台使用的消息、传感器引用和执行器实例。
 */
void GimbalInit(void);


/**
 * @brief 执行一次云台状态机与控制目标更新。
 */
void GimbalTask(void);

#endif // GIMBAL_H
