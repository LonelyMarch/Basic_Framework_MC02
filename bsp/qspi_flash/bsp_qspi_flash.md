# bsp_qspi_flash

<p align='right'>neozng1@hnu.edu.cn</p>

## 简介

`bsp_qspi_flash` 是主控板板载 W25Q64 外部 Flash 的 BSP 层封装。W25Q64 通过 STM32H723 的 `OCTOSPI2` 连接，容量为 8MB，页大小为 256 字节，最小擦除单位为 4KB 扇区。

本模块参考主控板官方 `CtrBoard-H7_W25Q64` 例程实现，使用 W25Q64 常用指令完成 ID 读取、数据读取、页编程、扇区擦除、块擦除、整片擦除和内存映射读取。

## CubeMX 配置

当前工程中 `OCTOSPI2` 由 CubeMX 初始化，相关函数为：

```c
MX_OCTOSPI2_Init();
```

W25Q64 使用的关键参数：

```c
hospi2.Instance = OCTOSPI2;
hospi2.Init.FifoThreshold = 8;
hospi2.Init.DeviceSize = 23;
hospi2.Init.ClockMode = HAL_OSPI_CLOCK_MODE_3;
hospi2.Init.ClockPrescaler = 3;
hospi2.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_HALFCYCLE;
```

相关引脚：

- `PB2`: OCTOSPIM_P1_CLK
- `PE11`: OCTOSPIM_P1_NCS
- `PD11`: OCTOSPIM_P1_IO0
- `PB0`: OCTOSPIM_P1_IO1
- `PA3`: OCTOSPIM_P1_IO2
- `PA1`: OCTOSPIM_P1_IO3

## FreeRTOS 适配

本模块运行在 CMSIS-RTOS v2 / FreeRTOS 环境下。

调度器运行后，外部 Flash 读、写、擦除和内存映射配置接口都会通过递归互斥锁保护 `hospi2`，避免多个任务同时访问 OSPI 外设。裸机初始化阶段如果调用本模块，此时没有多任务并发，函数不会创建或获取互斥锁。

中断服务函数中不允许调用本模块接口，因为 Flash 擦除、写入和自动轮询可能持续较长时间。

如果在全局中断关闭期间调用本模块，硬件异常时 HAL 超时计数可能无法正常推进，因此不建议在 `RobotInit()` 关中断阶段做外部 Flash 擦写或长时间读写。

不建议在高频控制任务中直接执行擦除或大块写入。参数保存、日志保存等操作建议放在低优先级任务中完成。

## 外部接口

```c
int8_t BSP_QSPI_Flash_Init(void);
uint32_t BSP_QSPI_Flash_ReadID(void);

int8_t BSP_QSPI_Flash_Read(uint32_t read_addr, uint8_t *buffer, uint32_t size);
int8_t BSP_QSPI_Flash_Write(uint32_t write_addr, const uint8_t *buffer, uint32_t size);
int8_t BSP_QSPI_Flash_WritePage(uint32_t write_addr, const uint8_t *buffer, uint16_t size);

int8_t BSP_QSPI_Flash_EraseSector(uint32_t sector_addr);
int8_t BSP_QSPI_Flash_EraseBlock32K(uint32_t block_addr);
int8_t BSP_QSPI_Flash_EraseBlock64K(uint32_t block_addr);
int8_t BSP_QSPI_Flash_EraseRange(uint32_t erase_addr, uint32_t size);
int8_t BSP_QSPI_Flash_ChipErase(void);

int8_t BSP_QSPI_Flash_MemoryMappedMode(void);
int8_t BSP_QSPI_Flash_AbortMemoryMappedMode(void);
```

## 地址与容量

W25Q64 的内部地址范围为：

```c
0x000000 ~ 0x7FFFFF
```

模块中定义的常用参数：

```c
#define BSP_QSPI_FLASH_PAGE_SIZE       256U
#define BSP_QSPI_FLASH_SECTOR_SIZE     4096U
#define BSP_QSPI_FLASH_BLOCK_32K_SIZE  (32U * 1024U)
#define BSP_QSPI_FLASH_BLOCK_64K_SIZE  (64U * 1024U)
#define BSP_QSPI_FLASH_SIZE            0x800000U
#define BSP_QSPI_FLASH_JEDEC_ID        0xEF4017U
#define BSP_QSPI_FLASH_MEM_MAPPED_ADDR 0x90000000U
```

## 使用示例

```c
uint8_t write_buf[256];
uint8_t read_buf[256];

if (BSP_QSPI_Flash_Init() == BSP_QSPI_FLASH_OK)
{
    BSP_QSPI_Flash_EraseSector(0x000000);
    BSP_QSPI_Flash_Write(0x000000, write_buf, sizeof(write_buf));
    BSP_QSPI_Flash_Read(0x000000, read_buf, sizeof(read_buf));
}
```

写入前必须先擦除目标区域。W25Q64 只能在编程时把 bit 从 1 写成 0，不能直接把 0 写回 1。

## 擦除接口

`BSP_QSPI_Flash_EraseSector()` 擦除 4KB，地址必须 4KB 对齐。

`BSP_QSPI_Flash_EraseBlock32K()` 擦除 32KB，地址必须 32KB 对齐。

`BSP_QSPI_Flash_EraseBlock64K()` 擦除 64KB，地址必须 64KB 对齐。

`BSP_QSPI_Flash_EraseRange()` 用于擦除连续区域。为了避免误擦除额外数据，起始地址和长度都必须 4KB 对齐。函数内部会优先使用 64KB 块擦除，其次使用 32KB 块擦除，最后使用 4KB 扇区擦除。

`BSP_QSPI_Flash_ChipErase()` 会擦除整片 W25Q64，耗时可能达到几十秒，使用时应谨慎。

## 内存映射模式

调用：

```c
BSP_QSPI_Flash_MemoryMappedMode();
```

之后可以从 `0x90000000` 地址直接读取外部 Flash：

```c
uint8_t value = *(uint8_t *)(BSP_QSPI_FLASH_MEM_MAPPED_ADDR + offset);
```

内存映射模式适合只读访问。如果之后调用擦除或写入接口，本模块会先退出内存映射模式，再发送间接命令。

## 注意事项

- 本模块驱动的是板载外部 W25Q64，不是 STM32H723 片内 Flash。
- `MX_OCTOSPI2_Init()` 已在 `main.c` 中由 CubeMX 生成代码调用，本模块不重复初始化 OSPI 外设。
- `BSP_QSPI_Flash_Init()` 只读取 JEDEC ID，用于确认 W25Q64 通信正常。
- 写入前必须擦除。
- 页编程最大 256 字节，`BSP_QSPI_Flash_Write()` 会自动按页拆分。
- 擦除和写入接口不能在中断中调用。
- 多任务环境下访问外部 Flash 会自动通过互斥锁串行化。
- 默认不进入内存映射模式，避免影响后续擦除和写入命令。
