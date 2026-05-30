/**
 * @file remote_control.h
 * @author DJI 2016
 * @author modified by neozng
 * @brief  遥控器模块定义头文件
 * @version beta
 * @date 2022-11-01
 *
 * @copyright Copyright (c) 2016 DJI corp
 * @copyright Copyright (c) 2022 HNU YueLu EC all rights reserved
 *
 */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

// 用于遥控器数据读取,遥控器数据是一个大小为2的数组
#define RC_LAST 1
#define RC_TEMP 0

// 获取按键操作
#define RC_KEY_PRESS 0
#define RC_KEY_PRESS_WITH_CTRL 1
#define RC_KEY_PRESS_WITH_SHIFT 2

// 检查接收值是否出错
#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)
#define REMOTE_CONTROL_CHANNEL_VALID_LIMIT 660 // 摇杆/拨轮解析后超过该绝对值视为异常,直接清零

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)   // 开关向上时的值
#define RC_SW_MID ((uint16_t)3)  // 开关中间时的值
#define RC_SW_DOWN ((uint16_t)2) // 开关向下时的值
// 三个判断开关状态的宏
#define switch_is_down(s) ((s) == RC_SW_DOWN)
#define switch_is_mid(s) ((s) == RC_SW_MID)
#define switch_is_up(s) ((s) == RC_SW_UP)

/* ----------------------- PC Key Definition-------------------------------- */
// 对应key[x][0~16],获取对应的键;例如通过key[RC_KEY_PRESS][RC_KEY_W]获取W键是否按下
#define RC_KEY_W 0
#define RC_KEY_S 1
#define RC_KEY_D 2
#define RC_KEY_A 3
#define RC_KEY_SHIFT 4
#define RC_KEY_CTRL 5
#define RC_KEY_Q 6
#define RC_KEY_E 7
#define RC_KEY_R 8
#define RC_KEY_F 9
#define RC_KEY_G 10
#define RC_KEY_Z 11
#define RC_KEY_X 12
#define RC_KEY_C 13
#define RC_KEY_V 14
#define RC_KEY_B 15

/* ----------------------- Data Struct ------------------------------------- */
// 待测试的位域结构体,可以极大提升解析速度
typedef union
{
    struct // 用于访问键盘状态
    {
        uint16_t w : 1;
        uint16_t s : 1;
        uint16_t d : 1;
        uint16_t a : 1;
        uint16_t shift : 1;
        uint16_t ctrl : 1;
        uint16_t q : 1;
        uint16_t e : 1;
        uint16_t r : 1;
        uint16_t f : 1;
        uint16_t g : 1;
        uint16_t z : 1;
        uint16_t x : 1;
        uint16_t c : 1;
        uint16_t v : 1;
        uint16_t b : 1;
    };
    uint16_t keys; // 用于memcpy而不需要进行强制类型转换
} Key_t;

// @todo 当前结构体嵌套过深,需要进行优化
typedef struct
{
    struct
    {
        int16_t rocker_l_; // 左水平
        int16_t rocker_l1; // 左竖直
        int16_t rocker_r_; // 右水平
        int16_t rocker_r1; // 右竖直
        int16_t dial;      // 侧边拨轮

        uint8_t switch_left;  // 左侧开关
        uint8_t switch_right; // 右侧开关
    } rc;
    struct
    {
        int16_t x;
        int16_t y;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;

    Key_t key[3]; // 改为位域后的键盘索引,空间减少8倍,速度增加16~倍

    uint8_t key_count[3][16];
} RC_ctrl_t;

/* ------------------------- Internal Data ----------------------------------- */

/**
 * @brief 初始化遥控器,该函数会将遥控器注册到串口
 *
 * @attention 注意分配正确的串口硬件,当前达妙MC02工程中遥控器使用UART5。
 *
 */
RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle);

/**
 * @brief 获取遥控器数据快照
 *
 * @note 传入的rc_snapshot必须指向长度为2的RC_ctrl_t数组,分别保存RC_TEMP和RC_LAST。
 *       这样上层任务读取遥控器数据时不会和USART解析任务发生半更新竞争。
 *
 * @param rc_snapshot 输出快照数组
 * @return uint8_t 1:获取成功 0:尚未初始化或参数非法
 */
uint8_t RemoteControlGet(RC_ctrl_t *rc_snapshot);

/**
 * @brief 检查遥控器是否在线,若尚未初始化也视为离线
 *
 * @return uint8_t 1:在线 0:离线
 */
uint8_t RemoteControlIsOnline(void);

#endif
