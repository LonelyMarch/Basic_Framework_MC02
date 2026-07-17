#include "bsp_qspi_flash.h"

#include "bsp_log.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "octospi.h"
#include <string.h>

extern OSPI_HandleTypeDef hospi2;

#define BSP_QSPI_FLASH_DEFAULT_TIMEOUT_MS       500U
#define BSP_QSPI_FLASH_WRITE_TIMEOUT_MS         10U
#define BSP_QSPI_FLASH_SECTOR_ERASE_TIMEOUT_MS  1000U
#define BSP_QSPI_FLASH_BLOCK_32K_TIMEOUT_MS     2000U
#define BSP_QSPI_FLASH_BLOCK_64K_TIMEOUT_MS     3000U
#define BSP_QSPI_FLASH_CHIP_ERASE_TIMEOUT_MS    100000U
#define BSP_QSPI_FLASH_MUTEX_TIMEOUT_MS         1000U

static StaticSemaphore_t qspi_flash_mutex_cb;
static SemaphoreHandle_t qspi_flash_mutex = NULL;
static uint8_t qspi_flash_memory_mapped = 0;


static int8_t BSP_QSPI_Flash_Lock(uint32_t timeout);


static void BSP_QSPI_Flash_Unlock(int8_t lock_state);


static int8_t BSP_QSPI_Flash_CheckRange(uint32_t address, uint32_t size);


static int8_t BSP_QSPI_Flash_CheckAligned(uint32_t address, uint32_t align);


static void BSP_QSPI_Flash_FillCommandDefault(OSPI_RegularCmdTypeDef * command);


static int8_t BSP_QSPI_Flash_WriteEnable(void);


static int8_t BSP_QSPI_Flash_WaitReady(uint32_t timeout);


static int8_t BSP_QSPI_Flash_EraseByCommand(uint32_t address, uint32_t command_code, uint32_t align, uint32_t timeout);


static int8_t BSP_QSPI_Flash_ExitMemoryMappedIfNeeded(void);


/**
 * @brief 初始化板载外部Flash,通过读取JEDEC ID确认W25Q64通信正常
 *
 * @note CubeMX已经在main.c中调用MX_OCTOSPI2_Init()完成OSPI外设和引脚初始化,
 *       本函数只负责确认W25Q64本体能正常响应命令。
 */
int8_t BSP_QSPI_Flash_Init(void)
{
    uint32_t device_id;

    if (qspi_flash_mutex == NULL)
    {
        qspi_flash_mutex = xSemaphoreCreateRecursiveMutexStatic(&qspi_flash_mutex_cb);
        if (qspi_flash_mutex == NULL)
        {
            LOGERROR("[qspi_flash] mutex create failed");
            return BSP_QSPI_FLASH_ERROR_MUTEX;
        }
    }

    device_id = BSP_QSPI_Flash_ReadID();

    if (device_id != BSP_QSPI_FLASH_JEDEC_ID)
    {
        LOGERROR("[qspi_flash] init failed, id = 0x%06X", device_id);
        return BSP_QSPI_FLASH_ERROR_INIT;
    }

    LOGINFO("[qspi_flash] W25Q64 init ok, id = 0x%06X", device_id);
    return BSP_QSPI_FLASH_OK;
}

/**
 * @brief 读取W25Q64的JEDEC ID
 *
 * @note JEDEC ID由厂商ID、存储类型、容量ID三个字节组成,W25Q64通常为0xEF4017。
 */
uint32_t BSP_QSPI_Flash_ReadID(void)
{
    OSPI_RegularCmdTypeDef command;
    uint8_t receive_buffer[3] = {0};
    int8_t lock_state = BSP_QSPI_Flash_Lock(BSP_QSPI_FLASH_MUTEX_TIMEOUT_MS);

    if (lock_state < 0)
    {
        return 0;
    }

    if (BSP_QSPI_Flash_ExitMemoryMappedIfNeeded() != BSP_QSPI_FLASH_OK)
    {
        BSP_QSPI_Flash_Unlock(lock_state);
        return 0;
    }

    BSP_QSPI_Flash_FillCommandDefault(&command);
    command.Instruction = BSP_QSPI_FLASH_CMD_JEDEC_ID;
    command.AddressMode = HAL_OSPI_ADDRESS_NONE;
    command.DataMode = HAL_OSPI_DATA_1_LINE;
    command.NbData = sizeof(receive_buffer);

    if (HAL_OSPI_Command(&hospi2, &command, BSP_QSPI_FLASH_DEFAULT_TIMEOUT_MS) != HAL_OK)
    {
        LOGERROR("[qspi_flash] read id command failed");
        BSP_QSPI_Flash_Unlock(lock_state);
        return 0;
    }

    if (HAL_OSPI_Receive(&hospi2, receive_buffer, BSP_QSPI_FLASH_DEFAULT_TIMEOUT_MS) != HAL_OK)
    {
        LOGERROR("[qspi_flash] read id receive failed");
        BSP_QSPI_Flash_Unlock(lock_state);
        return 0;
    }

    BSP_QSPI_Flash_Unlock(lock_state);
    return ((uint32_t)receive_buffer[0] << 16) | ((uint32_t)receive_buffer[1] << 8) | receive_buffer[2];
}

/**
 * @brief 读取外部Flash数据
 *
 * @note 采用官方例程中的0xEB Fast Read Quad I/O命令:
 *       指令阶段1线,地址阶段4线,数据阶段4线,并使用6个dummy cycle。
 */
int8_t BSP_QSPI_Flash_Read(uint32_t read_addr, uint8_t* buffer, uint32_t size)
{
    OSPI_RegularCmdTypeDef command;
    int8_t lock_state;
    int8_t status;

    if ((buffer == NULL) || (BSP_QSPI_Flash_CheckRange(read_addr, size) != BSP_QSPI_FLASH_OK))
    {
        LOGERROR("[qspi_flash] read invalid param, addr = 0x%X, size = %u", read_addr, size);
        return BSP_QSPI_FLASH_ERROR_INVALID_PARAM;
    }

    if (size == 0U)
    {
        return BSP_QSPI_FLASH_OK;
    }

    lock_state = BSP_QSPI_Flash_Lock(BSP_QSPI_FLASH_MUTEX_TIMEOUT_MS);
    if (lock_state < 0)
    {
        return lock_state;
    }

    status = BSP_QSPI_Flash_ExitMemoryMappedIfNeeded();
    if (status != BSP_QSPI_FLASH_OK)
    {
        BSP_QSPI_Flash_Unlock(lock_state);
        return status;
    }

    BSP_QSPI_Flash_FillCommandDefault(&command);
    command.Instruction = BSP_QSPI_FLASH_CMD_FAST_READ_QUAD_IO;
    command.Address = read_addr;
    command.AddressMode = HAL_OSPI_ADDRESS_4_LINES;
    command.DataMode = HAL_OSPI_DATA_4_LINES;
    command.NbData = size;
    command.DummyCycles = 6U;

    if (HAL_OSPI_Command(&hospi2, &command, BSP_QSPI_FLASH_DEFAULT_TIMEOUT_MS) != HAL_OK)
    {
        LOGERROR("[qspi_flash] read command failed, addr = 0x%X, size = %u", read_addr, size);
        BSP_QSPI_Flash_Unlock(lock_state);
        return BSP_QSPI_FLASH_ERROR_TRANSMIT;
    }

    if (HAL_OSPI_Receive(&hospi2, buffer, BSP_QSPI_FLASH_DEFAULT_TIMEOUT_MS) != HAL_OK)
    {
        LOGERROR("[qspi_flash] read receive failed, addr = 0x%X, size = %u", read_addr, size);
        BSP_QSPI_Flash_Unlock(lock_state);
        return BSP_QSPI_FLASH_ERROR_TRANSMIT;
    }

    status = BSP_QSPI_Flash_WaitReady(BSP_QSPI_FLASH_DEFAULT_TIMEOUT_MS);
    BSP_QSPI_Flash_Unlock(lock_state);
    return status;
}

/**
 * @brief 写入外部Flash数据,函数内部会自动按照页边界拆分
 *
 * @note W25Q64页大小为256字节,页编程命令不能跨页。这里先计算当前页剩余空间,
 *       再逐页调用BSP_QSPI_Flash_WritePage()完成连续数据写入。
 */
int8_t BSP_QSPI_Flash_Write(uint32_t write_addr, const uint8_t* buffer, uint32_t size)
{
    uint32_t current_addr;
    uint32_t current_size;
    uint32_t end_addr;
    const uint8_t* write_data;
    int8_t lock_state;
    int8_t status;

    if ((buffer == NULL) || (BSP_QSPI_Flash_CheckRange(write_addr, size) != BSP_QSPI_FLASH_OK))
    {
        LOGERROR("[qspi_flash] write invalid param, addr = 0x%X, size = %u", write_addr, size);
        return BSP_QSPI_FLASH_ERROR_INVALID_PARAM;
    }

    if (size == 0U)
    {
        return BSP_QSPI_FLASH_OK;
    }

    lock_state = BSP_QSPI_Flash_Lock(BSP_QSPI_FLASH_MUTEX_TIMEOUT_MS);
    if (lock_state < 0)
    {
        return lock_state;
    }

    current_addr = write_addr;
    end_addr = write_addr + size;
    write_data = buffer;
    current_size = BSP_QSPI_FLASH_PAGE_SIZE - (write_addr % BSP_QSPI_FLASH_PAGE_SIZE);
    if (current_size > size)
    {
        current_size = size;
    }

    while (current_addr < end_addr)
    {
        status = BSP_QSPI_Flash_WritePage(current_addr, write_data, (uint16_t)current_size);
        if (status != BSP_QSPI_FLASH_OK)
        {
            BSP_QSPI_Flash_Unlock(lock_state);
            return status;
        }

        current_addr += current_size;
        write_data += current_size;
        current_size = ((current_addr + BSP_QSPI_FLASH_PAGE_SIZE) > end_addr)
                           ? (end_addr - current_addr)
                           : BSP_QSPI_FLASH_PAGE_SIZE;
    }

    BSP_QSPI_Flash_Unlock(lock_state);
    return BSP_QSPI_FLASH_OK;
}

/**
 * @brief 写入单页数据
 *
 * @attention 页编程前必须先发送写使能命令,编程结束后必须轮询BUSY位等待Flash内部写入完成。
 */
int8_t BSP_QSPI_Flash_WritePage(uint32_t write_addr, const uint8_t* buffer, uint16_t size)
{
    OSPI_RegularCmdTypeDef command;
    uint32_t page_offset;
    int8_t lock_state;
    int8_t status;

    page_offset = write_addr % BSP_QSPI_FLASH_PAGE_SIZE;
    if ((buffer == NULL) || (size == 0U) || (size > BSP_QSPI_FLASH_PAGE_SIZE) ||
        ((page_offset + size) > BSP_QSPI_FLASH_PAGE_SIZE) ||
        (BSP_QSPI_Flash_CheckRange(write_addr, size) != BSP_QSPI_FLASH_OK))
    {
        LOGERROR("[qspi_flash] write page invalid param, addr = 0x%X, size = %u", write_addr, size);
        return BSP_QSPI_FLASH_ERROR_INVALID_PARAM;
    }

    lock_state = BSP_QSPI_Flash_Lock(BSP_QSPI_FLASH_MUTEX_TIMEOUT_MS);
    if (lock_state < 0)
    {
        return lock_state;
    }

    status = BSP_QSPI_Flash_ExitMemoryMappedIfNeeded();
    if (status != BSP_QSPI_FLASH_OK)
    {
        BSP_QSPI_Flash_Unlock(lock_state);
        return status;
    }

    status = BSP_QSPI_Flash_WriteEnable();
    if (status != BSP_QSPI_FLASH_OK)
    {
        BSP_QSPI_Flash_Unlock(lock_state);
        return status;
    }

    BSP_QSPI_Flash_FillCommandDefault(&command);
    command.Instruction = BSP_QSPI_FLASH_CMD_QUAD_INPUT_PAGE_PROG;
    command.Address = write_addr;
    command.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
    command.DataMode = HAL_OSPI_DATA_4_LINES;
    command.NbData = size;

    if (HAL_OSPI_Command(&hospi2, &command, BSP_QSPI_FLASH_DEFAULT_TIMEOUT_MS) != HAL_OK)
    {
        LOGERROR("[qspi_flash] write page command failed, addr = 0x%X, size = %u", write_addr, size);
        BSP_QSPI_Flash_Unlock(lock_state);
        return BSP_QSPI_FLASH_ERROR_TRANSMIT;
    }

    if (HAL_OSPI_Transmit(&hospi2, (uint8_t*)buffer, BSP_QSPI_FLASH_DEFAULT_TIMEOUT_MS) != HAL_OK)
    {
        LOGERROR("[qspi_flash] write page transmit failed, addr = 0x%X, size = %u", write_addr, size);
        BSP_QSPI_Flash_Unlock(lock_state);
        return BSP_QSPI_FLASH_ERROR_TRANSMIT;
    }

    status = BSP_QSPI_Flash_WaitReady(BSP_QSPI_FLASH_WRITE_TIMEOUT_MS);
    BSP_QSPI_Flash_Unlock(lock_state);
    return status;
}

int8_t BSP_QSPI_Flash_EraseSector(uint32_t sector_addr)
{
    return BSP_QSPI_Flash_EraseByCommand(sector_addr,
                                         BSP_QSPI_FLASH_CMD_SECTOR_ERASE,
                                         BSP_QSPI_FLASH_SECTOR_SIZE,
                                         BSP_QSPI_FLASH_SECTOR_ERASE_TIMEOUT_MS);
}

int8_t BSP_QSPI_Flash_EraseBlock32K(uint32_t block_addr)
{
    return BSP_QSPI_Flash_EraseByCommand(block_addr,
                                         BSP_QSPI_FLASH_CMD_BLOCK_ERASE_32K,
                                         BSP_QSPI_FLASH_BLOCK_32K_SIZE,
                                         BSP_QSPI_FLASH_BLOCK_32K_TIMEOUT_MS);
}

int8_t BSP_QSPI_Flash_EraseBlock64K(uint32_t block_addr)
{
    return BSP_QSPI_Flash_EraseByCommand(block_addr,
                                         BSP_QSPI_FLASH_CMD_BLOCK_ERASE_64K,
                                         BSP_QSPI_FLASH_BLOCK_64K_SIZE,
                                         BSP_QSPI_FLASH_BLOCK_64K_TIMEOUT_MS);
}

/**
 * @brief 擦除一段Flash区域
 *
 * @note 为减少擦除时间,当地址和剩余长度满足条件时优先使用64KB块擦除,
 *       其次使用32KB块擦除,最后用4KB扇区擦除补齐。
 */
int8_t BSP_QSPI_Flash_EraseRange(uint32_t erase_addr, uint32_t size)
{
    uint32_t current_addr;
    uint32_t remain_size;
    int8_t lock_state;
    int8_t status;

    if ((BSP_QSPI_Flash_CheckRange(erase_addr, size) != BSP_QSPI_FLASH_OK) ||
        (BSP_QSPI_Flash_CheckAligned(erase_addr, BSP_QSPI_FLASH_SECTOR_SIZE) != BSP_QSPI_FLASH_OK) ||
        (BSP_QSPI_Flash_CheckAligned(size, BSP_QSPI_FLASH_SECTOR_SIZE) != BSP_QSPI_FLASH_OK))
    {
        LOGERROR("[qspi_flash] erase range invalid param, addr = 0x%X, size = %u", erase_addr, size);
        return BSP_QSPI_FLASH_ERROR_INVALID_PARAM;
    }

    current_addr = erase_addr;
    remain_size = size;

    lock_state = BSP_QSPI_Flash_Lock(BSP_QSPI_FLASH_MUTEX_TIMEOUT_MS);
    if (lock_state < 0)
    {
        return lock_state;
    }

    while (remain_size > 0U)
    {
        if ((remain_size >= BSP_QSPI_FLASH_BLOCK_64K_SIZE) &&
            ((current_addr % BSP_QSPI_FLASH_BLOCK_64K_SIZE) == 0U))
        {
            status = BSP_QSPI_Flash_EraseBlock64K(current_addr);
            current_addr += BSP_QSPI_FLASH_BLOCK_64K_SIZE;
            remain_size -= BSP_QSPI_FLASH_BLOCK_64K_SIZE;
        }
        else if ((remain_size >= BSP_QSPI_FLASH_BLOCK_32K_SIZE) &&
            ((current_addr % BSP_QSPI_FLASH_BLOCK_32K_SIZE) == 0U))
        {
            status = BSP_QSPI_Flash_EraseBlock32K(current_addr);
            current_addr += BSP_QSPI_FLASH_BLOCK_32K_SIZE;
            remain_size -= BSP_QSPI_FLASH_BLOCK_32K_SIZE;
        }
        else
        {
            status = BSP_QSPI_Flash_EraseSector(current_addr);
            current_addr += BSP_QSPI_FLASH_SECTOR_SIZE;
            remain_size -= BSP_QSPI_FLASH_SECTOR_SIZE;
        }

        if (status != BSP_QSPI_FLASH_OK)
        {
            BSP_QSPI_Flash_Unlock(lock_state);
            return status;
        }
    }

    BSP_QSPI_Flash_Unlock(lock_state);
    return BSP_QSPI_FLASH_OK;
}

/**
 * @brief 擦除整片W25Q64
 *
 * @note W25Q64整片擦除典型时间约20s,最大可能到100s。任务中调用时,
 *       其他任务对外部Flash的访问会被互斥锁阻塞。
 */
int8_t BSP_QSPI_Flash_ChipErase(void)
{
    OSPI_RegularCmdTypeDef command;
    int8_t lock_state;
    int8_t status;

    lock_state = BSP_QSPI_Flash_Lock(UINT32_MAX);
    if (lock_state < 0)
    {
        return lock_state;
    }

    status = BSP_QSPI_Flash_ExitMemoryMappedIfNeeded();
    if (status != BSP_QSPI_FLASH_OK)
    {
        BSP_QSPI_Flash_Unlock(lock_state);
        return status;
    }

    status = BSP_QSPI_Flash_WriteEnable();
    if (status != BSP_QSPI_FLASH_OK)
    {
        BSP_QSPI_Flash_Unlock(lock_state);
        return status;
    }

    BSP_QSPI_Flash_FillCommandDefault(&command);
    command.Instruction = BSP_QSPI_FLASH_CMD_CHIP_ERASE;
    command.AddressMode = HAL_OSPI_ADDRESS_NONE;
    command.DataMode = HAL_OSPI_DATA_NONE;

    if (HAL_OSPI_Command(&hospi2, &command, BSP_QSPI_FLASH_DEFAULT_TIMEOUT_MS) != HAL_OK)
    {
        LOGERROR("[qspi_flash] chip erase command failed");
        BSP_QSPI_Flash_Unlock(lock_state);
        return BSP_QSPI_FLASH_ERROR_ERASE;
    }

    status = BSP_QSPI_Flash_WaitReady(BSP_QSPI_FLASH_CHIP_ERASE_TIMEOUT_MS);
    BSP_QSPI_Flash_Unlock(lock_state);
    return status;
}

/**
 * @brief 进入内存映射模式
 *
 * @note 内存映射模式下CPU可以直接从0x90000000地址读取外部Flash,适合读取固定资源或参数。
 *       但擦除/写入仍必须通过间接命令完成,因此本BSP不会在初始化时默认进入该模式。
 */
int8_t BSP_QSPI_Flash_MemoryMappedMode(void)
{
    OSPI_RegularCmdTypeDef command;
    OSPI_MemoryMappedTypeDef mem_mapped_cfg;
    int8_t lock_state;
    int8_t status;

    lock_state = BSP_QSPI_Flash_Lock(BSP_QSPI_FLASH_MUTEX_TIMEOUT_MS);
    if (lock_state < 0)
    {
        return lock_state;
    }

    status = BSP_QSPI_Flash_ExitMemoryMappedIfNeeded();
    if (status != BSP_QSPI_FLASH_OK)
    {
        BSP_QSPI_Flash_Unlock(lock_state);
        return status;
    }

    BSP_QSPI_Flash_FillCommandDefault(&command);
    command.Instruction = BSP_QSPI_FLASH_CMD_FAST_READ_QUAD_IO;
    command.AddressMode = HAL_OSPI_ADDRESS_4_LINES;
    command.DataMode = HAL_OSPI_DATA_4_LINES;
    command.NbData = 1U;
    command.DummyCycles = 6U;

    if (HAL_OSPI_Command(&hospi2, &command, BSP_QSPI_FLASH_DEFAULT_TIMEOUT_MS) != HAL_OK)
    {
        LOGERROR("[qspi_flash] memory mapped command failed");
        BSP_QSPI_Flash_Unlock(lock_state);
        return BSP_QSPI_FLASH_ERROR_TRANSMIT;
    }

    memset(&mem_mapped_cfg, 0, sizeof(mem_mapped_cfg));
    mem_mapped_cfg.TimeOutActivation = HAL_OSPI_TIMEOUT_COUNTER_DISABLE;
    mem_mapped_cfg.TimeOutPeriod = 0U;

    if (HAL_OSPI_MemoryMapped(&hospi2, &mem_mapped_cfg) != HAL_OK)
    {
        LOGERROR("[qspi_flash] memory mapped mode failed");
        BSP_QSPI_Flash_Unlock(lock_state);
        return BSP_QSPI_FLASH_ERROR_MEMORY_MAPPED;
    }

    qspi_flash_memory_mapped = 1U;
    BSP_QSPI_Flash_Unlock(lock_state);
    return BSP_QSPI_FLASH_OK;
}

int8_t BSP_QSPI_Flash_AbortMemoryMappedMode(void)
{
    int8_t lock_state = BSP_QSPI_Flash_Lock(BSP_QSPI_FLASH_MUTEX_TIMEOUT_MS);
    int8_t status;

    if (lock_state < 0)
    {
        return lock_state;
    }

    status = BSP_QSPI_Flash_ExitMemoryMappedIfNeeded();
    BSP_QSPI_Flash_Unlock(lock_state);
    return status;
}

/**
 * @brief 根据当前运行环境获取Flash互斥锁
 *
 * @note 裸机初始化阶段没有并发任务,不使用RTOS互斥锁;调度器运行后使用CMSIS-RTOS v2互斥锁。
 *       Flash接口不允许在中断中调用,因为擦写和轮询等待都可能持续较长时间。
 */
static int8_t BSP_QSPI_Flash_Lock(uint32_t timeout)
{
    TickType_t timeout_ticks;

    if (__get_IPSR() != 0U)
    {
        return BSP_QSPI_FLASH_ERROR_IN_ISR;
    }

    if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING)
    {
        return 0;
    }

    if (qspi_flash_mutex == NULL)
    {
        LOGERROR("[qspi_flash] mutex not initialized");
        return BSP_QSPI_FLASH_ERROR_MUTEX;
    }

    timeout_ticks = (timeout == UINT32_MAX) ? portMAX_DELAY : pdMS_TO_TICKS(timeout);
    if (xSemaphoreTakeRecursive(qspi_flash_mutex, timeout_ticks) != pdPASS)
    {
        LOGERROR("[qspi_flash] mutex acquire timeout");
        return BSP_QSPI_FLASH_ERROR_MUTEX;
    }

    return 1;
}

static void BSP_QSPI_Flash_Unlock(int8_t lock_state)
{
    if ((lock_state > 0) && (qspi_flash_mutex != NULL))
    {
        if (xSemaphoreGiveRecursive(qspi_flash_mutex) != pdPASS)
        {
            LOGERROR("[qspi_flash] mutex release failed");
        }
    }
}

static int8_t BSP_QSPI_Flash_CheckRange(uint32_t address, uint32_t size)
{
    if (size == 0U)
    {
        return BSP_QSPI_FLASH_OK;
    }

    if ((address >= BSP_QSPI_FLASH_SIZE) || (size > (BSP_QSPI_FLASH_SIZE - address)))
    {
        return BSP_QSPI_FLASH_ERROR_INVALID_PARAM;
    }

    return BSP_QSPI_FLASH_OK;
}

static int8_t BSP_QSPI_Flash_CheckAligned(uint32_t address, uint32_t align)
{
    if ((align == 0U) || ((address % align) != 0U))
    {
        return BSP_QSPI_FLASH_ERROR_INVALID_PARAM;
    }

    return BSP_QSPI_FLASH_OK;
}

/**
 * @brief 填充W25Q64常用OSPI命令默认配置
 *
 * @note 官方例程中每个函数都会完整配置OSPI_RegularCmdTypeDef。这里提取公共字段,
 *       每条具体命令再覆盖指令、地址模式、数据线数和数据长度等差异字段。
 */
static void BSP_QSPI_Flash_FillCommandDefault(OSPI_RegularCmdTypeDef* command)
{
    memset(command, 0, sizeof(OSPI_RegularCmdTypeDef));

    command->OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
    command->FlashId = HAL_OSPI_FLASH_ID_1;
    command->InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
    command->InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
    command->InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
    command->Address = 0U;
    command->AddressMode = HAL_OSPI_ADDRESS_1_LINE;
    command->AddressSize = HAL_OSPI_ADDRESS_24_BITS;
    command->AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
    command->AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
    command->AlternateBytesDtrMode = HAL_OSPI_ALTERNATE_BYTES_DTR_DISABLE;
    command->DataMode = HAL_OSPI_DATA_NONE;
    command->DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
    command->NbData = 0U;
    command->DummyCycles = 0U;
    command->DQSMode = HAL_OSPI_DQS_DISABLE;
    command->SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;
}

/**
 * @brief 发送写使能命令并轮询WEL位
 *
 * @note W25Q64所有擦除和页编程命令之前都必须先写使能。写使能后通过状态寄存器1的WEL位
 *       判断Flash是否已经允许执行后续写/擦命令。
 */
static int8_t BSP_QSPI_Flash_WriteEnable(void)
{
    OSPI_RegularCmdTypeDef command;
    OSPI_AutoPollingTypeDef config;

    BSP_QSPI_Flash_FillCommandDefault(&command);
    command.Instruction = BSP_QSPI_FLASH_CMD_WRITE_ENABLE;
    command.AddressMode = HAL_OSPI_ADDRESS_NONE;
    command.DataMode = HAL_OSPI_DATA_NONE;

    if (HAL_OSPI_Command(&hospi2, &command, BSP_QSPI_FLASH_DEFAULT_TIMEOUT_MS) != HAL_OK)
    {
        LOGERROR("[qspi_flash] write enable command failed");
        return BSP_QSPI_FLASH_ERROR_WRITE_ENABLE;
    }

    BSP_QSPI_Flash_FillCommandDefault(&command);
    command.Instruction = BSP_QSPI_FLASH_CMD_READ_STATUS_REG1;
    command.AddressMode = HAL_OSPI_ADDRESS_NONE;
    command.DataMode = HAL_OSPI_DATA_1_LINE;
    command.NbData = 1U;

    if (HAL_OSPI_Command(&hospi2, &command, BSP_QSPI_FLASH_DEFAULT_TIMEOUT_MS) != HAL_OK)
    {
        LOGERROR("[qspi_flash] write enable status command failed");
        return BSP_QSPI_FLASH_ERROR_WRITE_ENABLE;
    }

    memset(&config, 0, sizeof(config));
    config.Match = BSP_QSPI_FLASH_STATUS_REG1_WEL;
    config.Mask = BSP_QSPI_FLASH_STATUS_REG1_WEL;
    config.MatchMode = HAL_OSPI_MATCH_MODE_AND;
    config.Interval = 0x10U;
    config.AutomaticStop = HAL_OSPI_AUTOMATIC_STOP_ENABLE;

    if (HAL_OSPI_AutoPolling(&hospi2, &config, BSP_QSPI_FLASH_DEFAULT_TIMEOUT_MS) != HAL_OK)
    {
        LOGERROR("[qspi_flash] wait write enable failed");
        return BSP_QSPI_FLASH_ERROR_AUTOPOLLING;
    }

    return BSP_QSPI_FLASH_OK;
}

/**
 * @brief 轮询状态寄存器1的BUSY位,等待Flash内部操作完成
 *
 * @note 擦除和写入命令发送完成后,W25Q64仍会在芯片内部执行实际操作。
 *       BUSY位为1表示芯片忙,变回0才表示本次操作真正结束。
 */
static int8_t BSP_QSPI_Flash_WaitReady(uint32_t timeout)
{
    OSPI_RegularCmdTypeDef command;
    OSPI_AutoPollingTypeDef config;

    BSP_QSPI_Flash_FillCommandDefault(&command);
    command.Instruction = BSP_QSPI_FLASH_CMD_READ_STATUS_REG1;
    command.AddressMode = HAL_OSPI_ADDRESS_NONE;
    command.DataMode = HAL_OSPI_DATA_1_LINE;
    command.NbData = 1U;

    if (HAL_OSPI_Command(&hospi2, &command, BSP_QSPI_FLASH_DEFAULT_TIMEOUT_MS) != HAL_OK)
    {
        LOGERROR("[qspi_flash] wait ready command failed");
        return BSP_QSPI_FLASH_ERROR_AUTOPOLLING;
    }

    memset(&config, 0, sizeof(config));
    config.Match = 0U;
    config.Mask = BSP_QSPI_FLASH_STATUS_REG1_BUSY;
    config.MatchMode = HAL_OSPI_MATCH_MODE_AND;
    config.Interval = 0x10U;
    config.AutomaticStop = HAL_OSPI_AUTOMATIC_STOP_ENABLE;

    if (HAL_OSPI_AutoPolling(&hospi2, &config, timeout) != HAL_OK)
    {
        LOGERROR("[qspi_flash] wait ready timeout");
        return BSP_QSPI_FLASH_ERROR_AUTOPOLLING;
    }

    return BSP_QSPI_FLASH_OK;
}

static int8_t BSP_QSPI_Flash_EraseByCommand(uint32_t address, uint32_t command_code, uint32_t align, uint32_t timeout)
{
    OSPI_RegularCmdTypeDef command;
    int8_t lock_state;
    int8_t status;

    if ((BSP_QSPI_Flash_CheckRange(address, align) != BSP_QSPI_FLASH_OK) ||
        (BSP_QSPI_Flash_CheckAligned(address, align) != BSP_QSPI_FLASH_OK))
    {
        LOGERROR("[qspi_flash] erase invalid param, addr = 0x%X, align = %u", address, align);
        return BSP_QSPI_FLASH_ERROR_INVALID_PARAM;
    }

    lock_state = BSP_QSPI_Flash_Lock(BSP_QSPI_FLASH_MUTEX_TIMEOUT_MS);
    if (lock_state < 0)
    {
        return lock_state;
    }

    status = BSP_QSPI_Flash_ExitMemoryMappedIfNeeded();
    if (status != BSP_QSPI_FLASH_OK)
    {
        BSP_QSPI_Flash_Unlock(lock_state);
        return status;
    }

    status = BSP_QSPI_Flash_WriteEnable();
    if (status != BSP_QSPI_FLASH_OK)
    {
        BSP_QSPI_Flash_Unlock(lock_state);
        return status;
    }

    BSP_QSPI_Flash_FillCommandDefault(&command);
    command.Instruction = command_code;
    command.Address = address;
    command.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
    command.DataMode = HAL_OSPI_DATA_NONE;

    if (HAL_OSPI_Command(&hospi2, &command, BSP_QSPI_FLASH_DEFAULT_TIMEOUT_MS) != HAL_OK)
    {
        LOGERROR("[qspi_flash] erase command failed, addr = 0x%X, cmd = 0x%X", address, command_code);
        BSP_QSPI_Flash_Unlock(lock_state);
        return BSP_QSPI_FLASH_ERROR_ERASE;
    }

    status = BSP_QSPI_Flash_WaitReady(timeout);
    BSP_QSPI_Flash_Unlock(lock_state);
    return status;
}

static int8_t BSP_QSPI_Flash_ExitMemoryMappedIfNeeded(void)
{
    if (qspi_flash_memory_mapped == 0U)
    {
        return BSP_QSPI_FLASH_OK;
    }

    if (HAL_OSPI_Abort(&hospi2) != HAL_OK)
    {
        LOGERROR("[qspi_flash] abort memory mapped mode failed");
        return BSP_QSPI_FLASH_ERROR_MEMORY_MAPPED;
    }

    qspi_flash_memory_mapped = 0U;
    return BSP_QSPI_FLASH_OK;
}
