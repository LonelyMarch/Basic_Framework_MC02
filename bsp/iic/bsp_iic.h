/**
 * @file bsp_iic.h
 * @brief I2C/IIC总线型外设BSP封装,支持阻塞/中断/DMA同步事务和序列传输。
 */
#ifndef BSP_IIC_H
#define BSP_IIC_H

#include "i2c.h"
#include <stdint.h>

#define IIC_DEVICE_CNT 2   // MC02主控板有2个I2C接口可用
#define MX_IIC_SLAVE_CNT 8 // 最大从机数目,根据需要修改

#ifndef IIC_USE_DMA_CACHE_MAINTENANCE
#define IIC_USE_DMA_CACHE_MAINTENANCE 1U // STM32H7开启DCache后,DMA缓冲区需要做Cache维护
#endif

/* I2C工作模式枚举 */
typedef enum
{
    // 基本工作模式
    IIC_BLOCK_MODE = 0, // 阻塞模式
    IIC_IT_MODE, // 中断模式
    IIC_DMA_MODE, // DMA模式
} IIC_Work_Mode_e;

/* I2C MEM工作模式枚举,这两种方法都是阻塞 */
typedef enum
{
    IIC_READ_MEM = 0, // 读取从机内部的寄存器or内存
    IIC_WRITE_MEM, // 写入从机内部的寄存器or内存
} IIC_Mem_Mode_e;

/* 序列传输模式枚举。HOLDON只适用于IT/DMA模式,用于连续多段事务期间保持总线互斥锁。 */
// 每个HOLDON序列必须以IIC_SEQ_RELEASE结束,否则同一条I2C总线会一直被当前实例占用。
typedef enum
{
    IIC_SEQ_RELEASE = 0, // 完成传输后释放总线占有权,这是默认的传输方式
    IIC_SEQ_HOLDON = 1, // 保持BSP总线互斥锁不释放,只支持IT和DMA模式
} IIC_Seq_Mode_e;

/* I2C从设备实例 */
typedef struct iic_temp_s
{
    I2C_HandleTypeDef* handle; // I2C硬件句柄
    uint8_t dev_address; // 暂时只支持7位地址(还有一位是读写位)

    IIC_Work_Mode_e work_mode; // 工作模式
    uint8_t* rx_buffer; // 接收缓冲区指针
    uint16_t rx_len; // 接收长度,与HAL I2C的Size参数保持一致
    void (*callback)(struct iic_temp_s*); // 接收完成后的回调函数,由调用者任务上下文触发

    void* id; // 用于标识I2C instance
} IICInstance;

/* I2C 初始化结构体配置 */
typedef struct
{
    I2C_HandleTypeDef* handle; // I2C硬件句柄
    uint8_t dev_address; // 暂时只支持7位地址(还有一位是读写位),注意不需要左移
    IIC_Work_Mode_e work_mode; // 工作模式
    void (*callback)(IICInstance*); // 接收完成后的回调函数,由调用者任务上下文触发
    void* id; // 用于标识I2C instance
} IIC_Init_Config_s;

/**
 * @brief IIC初始化
 *
 * @param conf 初始化配置
 * @return IICInstance*
 */
IICInstance* IICRegister(IIC_Init_Config_s* conf);


/**
 * @brief 为已经注册的I2C硬件总线创建RTOS互斥锁和完成信号量。
 *
 * @note 由BSPTaskInit()在osKernelInitialize()之后、任务启动前统一调用。
 */
HAL_StatusTypeDef IICBusOsInit(void);


/**
 * @brief IIC发送数据
 *
 * @param iic iic实例
 * @param data 待发送的数据首地址指针
 * @param size 发送长度
 * @param mode 序列传输模式
 * @return HAL_StatusTypeDef HAL_OK表示发送完成,其他值表示失败或超时
 * @note 注意,如果发送结构体,那么该结构体在声明时务必使用#pragma pack(1)进行对齐,并在声明结束后使用#pragma pack()恢复对齐
 * @note IIC_DMA_MODE下,BSP会自动使用RAM_D2内部中转缓冲区,上层data可以位于普通栈/全局/堆内存
 *
 */
HAL_StatusTypeDef IICTransmit(IICInstance* iic, uint8_t* data, uint16_t size, IIC_Seq_Mode_e mode);


/**
 * @brief IIC接收数据
 *
 * @param iic iic实例
 * @param data 接收数据的首地址指针
 * @param size 接收长度
 * @param mode 序列传输模式
 * @return HAL_StatusTypeDef HAL_OK表示接收完成,其他值表示失败或超时
 * @note 注意,如果直接将接收数据memcpy到目标结构体或通过强制类型转换进行逐字节写入,
 *       那么该结构体在声明时务必使用#pragma pack(1)进行对齐,并在声明结束后使用#pragma pack()恢复对齐
 * @note IIC_DMA_MODE下,BSP会自动使用RAM_D2内部中转缓冲区,上层data可以位于普通栈/全局/堆内存
 */
HAL_StatusTypeDef IICReceive(IICInstance* iic, uint8_t* data, uint16_t size, IIC_Seq_Mode_e mode);


/**
 * @brief IIC访问从机寄存器(内存)
 *
 * @param iic iic实例
 * @param mem_addr 要读取的从机内存地址,目前只支持8位地址
 * @param data 要读取或写入的数据首地址指针
 * @param size 要读取或写入的数据长度
 * @param mode 写入或读取模式: IIC_READ_MEM or IIC_WRITE_MEM
 * @param mem8bit_flag 从机内存地址是否为8位
 * @return HAL_StatusTypeDef HAL_OK表示访问完成,其他值表示失败或超时
 * @note IIC_DMA_MODE下,BSP会自动使用RAM_D2内部中转缓冲区,上层data可以位于普通栈/全局/堆内存
 */
HAL_StatusTypeDef IICAccessMem(IICInstance* iic, uint16_t mem_addr, uint8_t* data, uint16_t size, IIC_Mem_Mode_e mode,
                               uint8_t mem8bit_flag);

#endif // BSP_IIC_H
