#ifndef BSP_QSPI_FLASH_H
#define BSP_QSPI_FLASH_H

#include "main.h"
#include <stdint.h>

/*----------------------------------------------- W25Q64基本参数 -------------------------------------------*/

#define BSP_QSPI_FLASH_OK                         0
#define BSP_QSPI_FLASH_ERROR_INIT                -1
#define BSP_QSPI_FLASH_ERROR_WRITE_ENABLE        -2
#define BSP_QSPI_FLASH_ERROR_AUTOPOLLING         -3
#define BSP_QSPI_FLASH_ERROR_ERASE               -4
#define BSP_QSPI_FLASH_ERROR_TRANSMIT            -5
#define BSP_QSPI_FLASH_ERROR_MEMORY_MAPPED       -6
#define BSP_QSPI_FLASH_ERROR_INVALID_PARAM       -7
#define BSP_QSPI_FLASH_ERROR_MUTEX               -8
#define BSP_QSPI_FLASH_ERROR_IN_ISR              -9

#define BSP_QSPI_FLASH_CMD_ENABLE_RESET           0x66U
#define BSP_QSPI_FLASH_CMD_RESET_DEVICE           0x99U
#define BSP_QSPI_FLASH_CMD_JEDEC_ID               0x9FU
#define BSP_QSPI_FLASH_CMD_WRITE_ENABLE           0x06U

#define BSP_QSPI_FLASH_CMD_SECTOR_ERASE           0x20U  // 擦除4KB扇区
#define BSP_QSPI_FLASH_CMD_BLOCK_ERASE_32K        0x52U  // 擦除32KB块
#define BSP_QSPI_FLASH_CMD_BLOCK_ERASE_64K        0xD8U  // 擦除64KB块
#define BSP_QSPI_FLASH_CMD_CHIP_ERASE             0xC7U  // 擦除整片Flash

#define BSP_QSPI_FLASH_CMD_QUAD_INPUT_PAGE_PROG   0x32U  // 1-1-4页编程
#define BSP_QSPI_FLASH_CMD_FAST_READ_QUAD_IO      0xEBU  // 1-4-4快速读取

#define BSP_QSPI_FLASH_CMD_READ_STATUS_REG1       0x05U
#define BSP_QSPI_FLASH_STATUS_REG1_BUSY           0x01U
#define BSP_QSPI_FLASH_STATUS_REG1_WEL            0x02U

#define BSP_QSPI_FLASH_PAGE_SIZE                  256U
#define BSP_QSPI_FLASH_SECTOR_SIZE                4096U
#define BSP_QSPI_FLASH_BLOCK_32K_SIZE             (32U * 1024U)
#define BSP_QSPI_FLASH_BLOCK_64K_SIZE             (64U * 1024U)
#define BSP_QSPI_FLASH_SIZE                       0x800000U
#define BSP_QSPI_FLASH_JEDEC_ID                   0xEF4017U
#define BSP_QSPI_FLASH_MEM_MAPPED_ADDR            0x90000000U

/**
 * @brief 初始化板载外部Flash,通过读取JEDEC ID确认W25Q64通信正常
 *
 * @retval BSP_QSPI_FLASH_OK 初始化成功
 * @retval BSP_QSPI_FLASH_ERROR_INIT ID不匹配或通信失败
 */
int8_t BSP_QSPI_Flash_Init(void);

/**
 * @brief 读取W25Q64的JEDEC ID
 *
 * @retval 24位JEDEC ID,正常W25Q64应为0xEF4017
 */
uint32_t BSP_QSPI_Flash_ReadID(void);

/**
 * @brief 读取外部Flash数据
 *
 * @param read_addr Flash内部地址,范围为0 ~ BSP_QSPI_FLASH_SIZE - 1
 * @param buffer 读出数据的存放缓冲区
 * @param size 读取字节数
 * @retval BSP_QSPI_FLASH_OK 读取成功
 */
int8_t BSP_QSPI_Flash_Read(uint32_t read_addr, uint8_t *buffer, uint32_t size);

/**
 * @brief 写入外部Flash数据,函数内部会按256字节页自动拆分
 *
 * @attention 写入前必须保证目标区域已经擦除,Flash只能将bit从1写为0,不能直接从0写回1
 *
 * @param write_addr Flash内部地址,范围为0 ~ BSP_QSPI_FLASH_SIZE - 1
 * @param buffer 待写入数据缓冲区
 * @param size 写入字节数
 * @retval BSP_QSPI_FLASH_OK 写入成功
 */
int8_t BSP_QSPI_Flash_Write(uint32_t write_addr, const uint8_t *buffer, uint32_t size);

/**
 * @brief 写入单页数据,最多写入256字节,且不能跨页
 *
 * @param write_addr Flash内部地址
 * @param buffer 待写入数据缓冲区
 * @param size 写入字节数,最大为256
 * @retval BSP_QSPI_FLASH_OK 写入成功
 */
int8_t BSP_QSPI_Flash_WritePage(uint32_t write_addr, const uint8_t *buffer, uint16_t size);

/**
 * @brief 擦除一个4KB扇区
 *
 * @param sector_addr 扇区起始地址,必须4KB对齐
 * @retval BSP_QSPI_FLASH_OK 擦除成功
 */
int8_t BSP_QSPI_Flash_EraseSector(uint32_t sector_addr);

/**
 * @brief 擦除一个32KB块
 *
 * @param block_addr 块起始地址,必须32KB对齐
 * @retval BSP_QSPI_FLASH_OK 擦除成功
 */
int8_t BSP_QSPI_Flash_EraseBlock32K(uint32_t block_addr);

/**
 * @brief 擦除一个64KB块
 *
 * @param block_addr 块起始地址,必须64KB对齐
 * @retval BSP_QSPI_FLASH_OK 擦除成功
 */
int8_t BSP_QSPI_Flash_EraseBlock64K(uint32_t block_addr);

/**
 * @brief 擦除一段Flash区域,函数内部会优先使用64KB/32KB块擦除,剩余部分使用4KB扇区擦除
 *
 * @attention 为避免误擦除额外数据,erase_addr和size都必须4KB对齐
 *
 * @param erase_addr 擦除起始地址
 * @param size 擦除字节数
 * @retval BSP_QSPI_FLASH_OK 擦除成功
 */
int8_t BSP_QSPI_Flash_EraseRange(uint32_t erase_addr, uint32_t size);

/**
 * @brief 擦除整片W25Q64
 *
 * @attention 整片擦除耗时很长,任务中调用时会长期占用Flash互斥锁
 *
 * @retval BSP_QSPI_FLASH_OK 擦除成功
 */
int8_t BSP_QSPI_Flash_ChipErase(void);

/**
 * @brief 进入OSPI内存映射模式,之后可以从0x90000000地址直接读取外部Flash
 *
 * @attention 内存映射模式适合只读访问,进入后如需使用间接命令擦写Flash,本BSP会先退出内存映射模式
 *
 * @retval BSP_QSPI_FLASH_OK 配置成功
 */
int8_t BSP_QSPI_Flash_MemoryMappedMode(void);

/**
 * @brief 退出OSPI内存映射模式
 *
 * @retval BSP_QSPI_FLASH_OK 退出成功
 */
int8_t BSP_QSPI_Flash_AbortMemoryMappedMode(void);

#endif
