#include "BMI088Middleware.h"

/*
 * 旧版BMI088驱动曾在本文件中逐字节直连HAL SPI。
 * 当前主流程已经改为在BMI088driver.c中整包走bsp/spi,这里保留文件以兼容既有CMake源文件列表。
 */
