/**
 * @file bsp_flash.h
 * @brief STM32H723片上Flash用户区读写BSP封装。
 */
#ifndef BSP_FLASH_H
#define BSP_FLASH_H

#include "main.h"
#include <stdint.h>

/*----------------------------------------------- 片上Flash用户区配置 ---------------------------------------*/

#define BSP_FLASH_OK                         0
#define BSP_FLASH_ERROR_INVALID_PARAM       -1
#define BSP_FLASH_ERROR_LOCK                -2
#define BSP_FLASH_ERROR_ERASE               -3
#define BSP_FLASH_ERROR_PROGRAM             -4
#define BSP_FLASH_ERROR_VERIFY              -5
#define BSP_FLASH_ERROR_NOT_ERASED          -6
#define BSP_FLASH_ERROR_IN_ISR              -7
#define BSP_FLASH_ERROR_MUTEX               -8

#define BSP_FLASH_SECTOR_SIZE                FLASH_SECTOR_SIZE
#define BSP_FLASH_PROGRAM_UNIT               (FLASH_NB_32BITWORD_IN_FLASHWORD * 4U)

extern uint8_t __user_flash_start__;
extern uint8_t __user_flash_end__;

#define BSP_FLASH_USER_START                 ((uint32_t)(uintptr_t)&__user_flash_start__)
#define BSP_FLASH_USER_END                   ((uint32_t)(uintptr_t)&__user_flash_end__)
#define BSP_FLASH_USER_SIZE                  (BSP_FLASH_USER_END - BSP_FLASH_USER_START)

/**
 * @brief 初始化片上Flash BSP
 *
 * @note 当前实现不主动擦写Flash,只创建静态递归互斥锁。
 *
 * @retval BSP_FLASH_OK 初始化成功
 */
int8_t BSP_Flash_Init(void);


/**
 * @brief 获取用户Flash区起始地址
 *
 * @retval 用户Flash区绝对地址
 */
uint32_t BSP_Flash_GetUserStart(void);


/**
 * @brief 获取用户Flash区大小
 *
 * @retval 用户Flash区大小,单位字节
 */
uint32_t BSP_Flash_GetUserSize(void);


/**
 * @brief 读取片上Flash用户区数据
 *
 * @param offset 用户区内偏移,0表示BSP_FLASH_USER_START
 * @param buffer 读出数据缓冲区
 * @param size 读取字节数
 * @retval BSP_FLASH_OK 读取成功
 */
int8_t BSP_Flash_Read(uint32_t offset, void* buffer, uint32_t size);


/**
 * @brief 擦除片上Flash用户区
 *
 * @attention offset和size必须按BSP_FLASH_SECTOR_SIZE对齐。
 *
 * @param offset 用户区内偏移
 * @param size 擦除字节数
 * @retval BSP_FLASH_OK 擦除成功
 */
int8_t BSP_Flash_Erase(uint32_t offset, uint32_t size);


/**
 * @brief 擦除整个片上Flash用户区
 *
 * @retval BSP_FLASH_OK 擦除成功
 */
int8_t BSP_Flash_EraseAll(void);


/**
 * @brief 写入片上Flash用户区
 *
 * @attention offset必须按BSP_FLASH_PROGRAM_UNIT对齐。写入前必须先擦除目标区域。
 *
 * @param offset 用户区内偏移
 * @param buffer 待写入数据缓冲区
 * @param size 写入字节数
 * @retval BSP_FLASH_OK 写入成功
 */
int8_t BSP_Flash_Write(uint32_t offset, const void* buffer, uint32_t size);


/**
 * @brief 获取最近一次HAL Flash错误码
 *
 * @retval HAL_FLASH_GetError()返回的错误码
 */
uint32_t BSP_Flash_GetLastError(void);

#endif
