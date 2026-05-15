#include "bsp_flash.h"

#include "bsp_log.h"
#include "cmsis_os2.h"
#include <string.h>

static osMutexId_t bsp_flash_mutex = NULL;
static uint32_t bsp_flash_last_error = 0U;

static int8_t BSP_Flash_Lock(uint32_t timeout);
static void BSP_Flash_Unlock(int8_t lock_state);
static int8_t BSP_Flash_CheckRange(uint32_t offset, uint32_t size);
static int8_t BSP_Flash_CheckAligned(uint32_t value, uint32_t align);
static uint32_t BSP_Flash_OffsetToAddress(uint32_t offset);
static uint32_t BSP_Flash_AddressToSector(uint32_t address);
static int8_t BSP_Flash_CheckErased(uint32_t address, uint32_t size);
static void BSP_Flash_InvalidateCache(uint32_t address, uint32_t size);

/**
 * @brief 初始化片上Flash BSP
 *
 * @note 本函数不擦除、不写入Flash。调度器运行后,互斥锁会在首次访问时创建。
 */
int8_t BSP_Flash_Init(void)
{
    return BSP_FLASH_OK;
}

uint32_t BSP_Flash_GetUserStart(void)
{
    return BSP_FLASH_USER_START;
}

uint32_t BSP_Flash_GetUserSize(void)
{
    return BSP_FLASH_USER_SIZE;
}

int8_t BSP_Flash_Read(uint32_t offset, void *buffer, uint32_t size)
{
    int8_t lock_state;

    if ((buffer == NULL) || (BSP_Flash_CheckRange(offset, size) != BSP_FLASH_OK))
    {
        LOGERROR("[bsp_flash] read invalid param, offset = 0x%X, size = %u", offset, size);
        return BSP_FLASH_ERROR_INVALID_PARAM;
    }

    if (size == 0U)
    {
        return BSP_FLASH_OK;
    }

    lock_state = BSP_Flash_Lock(osWaitForever);
    if (lock_state < 0)
    {
        return lock_state;
    }

    memcpy(buffer, (const void *)BSP_Flash_OffsetToAddress(offset), size);
    BSP_Flash_Unlock(lock_state);
    return BSP_FLASH_OK;
}

int8_t BSP_Flash_Erase(uint32_t offset, uint32_t size)
{
    FLASH_EraseInitTypeDef erase_cfg;
    uint32_t sector_error = 0xFFFFFFFFU;
    uint32_t start_address;
    uint32_t start_sector;
    uint32_t sector_count;
    int8_t lock_state;
    int8_t status = BSP_FLASH_OK;

    if ((BSP_Flash_CheckRange(offset, size) != BSP_FLASH_OK) ||
        (BSP_Flash_CheckAligned(offset, BSP_FLASH_SECTOR_SIZE) != BSP_FLASH_OK) ||
        (BSP_Flash_CheckAligned(size, BSP_FLASH_SECTOR_SIZE) != BSP_FLASH_OK))
    {
        LOGERROR("[bsp_flash] erase invalid param, offset = 0x%X, size = %u", offset, size);
        return BSP_FLASH_ERROR_INVALID_PARAM;
    }

    if (size == 0U)
    {
        return BSP_FLASH_OK;
    }

    lock_state = BSP_Flash_Lock(osWaitForever);
    if (lock_state < 0)
    {
        return lock_state;
    }

    start_address = BSP_Flash_OffsetToAddress(offset);
    start_sector = BSP_Flash_AddressToSector(start_address);
    sector_count = size / BSP_FLASH_SECTOR_SIZE;

    memset(&erase_cfg, 0, sizeof(erase_cfg));
    erase_cfg.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_cfg.Banks = FLASH_BANK_1;
    erase_cfg.Sector = start_sector;
    erase_cfg.NbSectors = sector_count;
    erase_cfg.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        bsp_flash_last_error = HAL_FLASH_GetError();
        LOGERROR("[bsp_flash] unlock failed, error = 0x%X", bsp_flash_last_error);
        BSP_Flash_Unlock(lock_state);
        return BSP_FLASH_ERROR_LOCK;
    }

    if (HAL_FLASHEx_Erase(&erase_cfg, &sector_error) != HAL_OK)
    {
        bsp_flash_last_error = HAL_FLASH_GetError();
        LOGERROR("[bsp_flash] erase failed, sector = %u, error = 0x%X", sector_error, bsp_flash_last_error);
        status = BSP_FLASH_ERROR_ERASE;
    }

    if (HAL_FLASH_Lock() != HAL_OK)
    {
        bsp_flash_last_error = HAL_FLASH_GetError();
        LOGERROR("[bsp_flash] lock failed, error = 0x%X", bsp_flash_last_error);
        status = BSP_FLASH_ERROR_LOCK;
    }

    if (status == BSP_FLASH_OK)
    {
        BSP_Flash_InvalidateCache(start_address, size);
    }

    BSP_Flash_Unlock(lock_state);
    return status;
}

int8_t BSP_Flash_EraseAll(void)
{
    return BSP_Flash_Erase(0U, BSP_FLASH_USER_SIZE);
}

int8_t BSP_Flash_Write(uint32_t offset, const void *buffer, uint32_t size)
{
    uint32_t flash_word[FLASH_NB_32BITWORD_IN_FLASHWORD];
    const uint8_t *write_data = (const uint8_t *)buffer;
    uint32_t current_offset;
    uint32_t current_address;
    uint32_t remain_size;
    uint32_t chunk_size;
    int8_t lock_state;
    int8_t status = BSP_FLASH_OK;

    if ((buffer == NULL) || (BSP_Flash_CheckRange(offset, size) != BSP_FLASH_OK) ||
        (BSP_Flash_CheckAligned(offset, BSP_FLASH_PROGRAM_UNIT) != BSP_FLASH_OK))
    {
        LOGERROR("[bsp_flash] write invalid param, offset = 0x%X, size = %u", offset, size);
        return BSP_FLASH_ERROR_INVALID_PARAM;
    }

    if (size == 0U)
    {
        return BSP_FLASH_OK;
    }

    lock_state = BSP_Flash_Lock(osWaitForever);
    if (lock_state < 0)
    {
        return lock_state;
    }

    current_offset = offset;
    remain_size = size;

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        bsp_flash_last_error = HAL_FLASH_GetError();
        LOGERROR("[bsp_flash] unlock failed, error = 0x%X", bsp_flash_last_error);
        BSP_Flash_Unlock(lock_state);
        return BSP_FLASH_ERROR_LOCK;
    }

    while (remain_size > 0U)
    {
        current_address = BSP_Flash_OffsetToAddress(current_offset);
        chunk_size = (remain_size > BSP_FLASH_PROGRAM_UNIT) ? BSP_FLASH_PROGRAM_UNIT : remain_size;

        /*
         * STM32H723一次写入一个256bit flash word。即使最后一包不足32字节,
         * HAL也会写满32字节,因此这里用0xFF补齐,并要求整组目标flash word均为擦除态。
         */
        BSP_Flash_InvalidateCache(current_address, BSP_FLASH_PROGRAM_UNIT);
        if (BSP_Flash_CheckErased(current_address, BSP_FLASH_PROGRAM_UNIT) != BSP_FLASH_OK)
        {
            LOGERROR("[bsp_flash] target is not erased, address = 0x%X", current_address);
            status = BSP_FLASH_ERROR_NOT_ERASED;
            break;
        }

        memset(flash_word, 0xFF, sizeof(flash_word));
        memcpy(flash_word, write_data, chunk_size);

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                              current_address,
                              (uint32_t)(uintptr_t)flash_word) != HAL_OK)
        {
            bsp_flash_last_error = HAL_FLASH_GetError();
            LOGERROR("[bsp_flash] program failed, address = 0x%X, error = 0x%X",
                     current_address, bsp_flash_last_error);
            status = BSP_FLASH_ERROR_PROGRAM;
            break;
        }

        BSP_Flash_InvalidateCache(current_address, BSP_FLASH_PROGRAM_UNIT);
        if (memcmp((const void *)current_address, flash_word, sizeof(flash_word)) != 0)
        {
            LOGERROR("[bsp_flash] verify failed, address = 0x%X", current_address);
            status = BSP_FLASH_ERROR_VERIFY;
            break;
        }

        current_offset += chunk_size;
        write_data += chunk_size;
        remain_size -= chunk_size;
    }

    if (HAL_FLASH_Lock() != HAL_OK)
    {
        bsp_flash_last_error = HAL_FLASH_GetError();
        LOGERROR("[bsp_flash] lock failed, error = 0x%X", bsp_flash_last_error);
        status = BSP_FLASH_ERROR_LOCK;
    }

    if (status == BSP_FLASH_OK)
    {
        BSP_Flash_InvalidateCache(BSP_Flash_OffsetToAddress(offset), size);
    }

    BSP_Flash_Unlock(lock_state);
    return status;
}

uint32_t BSP_Flash_GetLastError(void)
{
    return bsp_flash_last_error;
}

static int8_t BSP_Flash_Lock(uint32_t timeout)
{
    osKernelState_t kernel_state;
    const osMutexAttr_t mutex_attr = {
        .name = "bsp_flash",
        .attr_bits = osMutexRecursive | osMutexPrioInherit,
    };

    if (__get_IPSR() != 0U)
    {
        LOGERROR("[bsp_flash] cannot access flash in ISR");
        return BSP_FLASH_ERROR_IN_ISR;
    }

    kernel_state = osKernelGetState();
    if (kernel_state != osKernelRunning)
    {
        return 0;
    }

    if (bsp_flash_mutex == NULL)
    {
        int32_t kernel_lock = osKernelLock();
        if (kernel_lock < 0)
        {
            LOGERROR("[bsp_flash] kernel lock failed");
            return BSP_FLASH_ERROR_MUTEX;
        }

        if (bsp_flash_mutex == NULL)
        {
            bsp_flash_mutex = osMutexNew(&mutex_attr);
        }

        if (osKernelRestoreLock(kernel_lock) < 0)
        {
            LOGERROR("[bsp_flash] kernel restore lock failed");
            return BSP_FLASH_ERROR_MUTEX;
        }

        if (bsp_flash_mutex == NULL)
        {
            LOGERROR("[bsp_flash] mutex create failed");
            return BSP_FLASH_ERROR_MUTEX;
        }
    }

    if (osMutexAcquire(bsp_flash_mutex, timeout) != osOK)
    {
        LOGERROR("[bsp_flash] mutex acquire timeout");
        return BSP_FLASH_ERROR_MUTEX;
    }

    return 1;
}

static void BSP_Flash_Unlock(int8_t lock_state)
{
    if ((lock_state > 0) && (bsp_flash_mutex != NULL))
    {
        if (osMutexRelease(bsp_flash_mutex) != osOK)
        {
            LOGERROR("[bsp_flash] mutex release failed");
        }
    }
}

static int8_t BSP_Flash_CheckRange(uint32_t offset, uint32_t size)
{
    if (size == 0U)
    {
        return BSP_FLASH_OK;
    }

    if ((offset >= BSP_FLASH_USER_SIZE) || (size > (BSP_FLASH_USER_SIZE - offset)))
    {
        return BSP_FLASH_ERROR_INVALID_PARAM;
    }

    return BSP_FLASH_OK;
}

static int8_t BSP_Flash_CheckAligned(uint32_t value, uint32_t align)
{
    if ((align == 0U) || ((value % align) != 0U))
    {
        return BSP_FLASH_ERROR_INVALID_PARAM;
    }

    return BSP_FLASH_OK;
}

static uint32_t BSP_Flash_OffsetToAddress(uint32_t offset)
{
    return BSP_FLASH_USER_START + offset;
}

static uint32_t BSP_Flash_AddressToSector(uint32_t address)
{
    return (address - FLASH_BANK1_BASE) / BSP_FLASH_SECTOR_SIZE;
}

static int8_t BSP_Flash_CheckErased(uint32_t address, uint32_t size)
{
    const uint8_t *flash_data = (const uint8_t *)address;
    uint32_t i;

    for (i = 0U; i < size; i++)
    {
        if (flash_data[i] != 0xFFU)
        {
            return BSP_FLASH_ERROR_NOT_ERASED;
        }
    }

    return BSP_FLASH_OK;
}

static void BSP_Flash_InvalidateCache(uint32_t address, uint32_t size)
{
    uint32_t aligned_address;
    uint32_t end_address;
    int32_t aligned_size;

    if (size == 0U)
    {
        return;
    }

    aligned_address = address & ~31UL;
    end_address = (address + size + 31UL) & ~31UL;
    aligned_size = (int32_t)(end_address - aligned_address);

    SCB_InvalidateDCache_by_Addr((uint32_t *)(uintptr_t)aligned_address, aligned_size);
    SCB_InvalidateICache();
}
