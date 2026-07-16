/**
 * @file motor_task.h
 * @author neozng
 * @brief  DJI、LK、HT、DDT、DM 电机的统一周期通信入口
 * @version beta
 * @date 2022-11-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H


/**
 * @brief 在 RTOS 中按 1 kHz 调用，执行各类 CAN/RS485 电机的一次控制与通信调度。
 */
void MotorControlTask(void);

#endif // !MOTOR_TASK_H
