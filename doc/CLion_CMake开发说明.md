# CLion + CMake 开发说明

本文档说明当前工程如何使用 CLion 和 CMake 构建。CMake 配置基本等价于原 Makefile：同一批源码、同一套宏定义、同一链接脚本和 CMSIS-DSP 静态库。

## 1. 新增文件

- `CMakeLists.txt`：主构建脚本，生成 `Basic_Framework_MC02.elf`。
- `cmake/arm-none-eabi-gcc.cmake`：ARM GCC 交叉编译工具链文件。
- `CMakePresets.json`：提供 `debug` 和 `release` 两个 CMake preset。

## 2. 工具链要求

需要安装以下工具：

- CLion
- CMake
- Ninja
- ARM GNU Toolchain，也就是 `arm-none-eabi-gcc`

如果已经安装 STM32CubeCLT，通常会自带 CMake、Ninja 和 ARM GCC。当前机器上检测到的 ARM GCC 路径类似：

```text
C:\Program Files\ST\STM32CubeCLT_1.19.0\GNU-tools-for-STM32\bin\arm-none-eabi-gcc.exe
```

工具链文件会按以下顺序查找编译器：

1. 环境变量 `GCC_PATH`
2. 环境变量 `GCC_ARM_NONE_EABI_PATH`
3. 环境变量 `ARM_NONE_EABI_TOOLCHAIN_PATH`
4. 系统 `PATH` 中的 `arm-none-eabi-gcc`

以上环境变量应指向 `bin` 目录，例如：

```text
C:\Program Files\ST\STM32CubeCLT_1.19.0\GNU-tools-for-STM32\bin
```

## 3. CLion 打开方式

1. 用 CLion 打开 `E:\电赛小车\矩形框识别\control`。
2. CLion 识别 `CMakePresets.json` 后，选择 `Debug - ARM GCC`。
3. 如果 CLion 没有自动找到工具链，在 CLion 的 Toolchains 设置中，把 C Compiler 指向 `arm-none-eabi-gcc.exe`，或配置上面的环境变量。
4. Reload CMake Project。
5. 构建目标 `Basic_Framework_MC02`。

构建完成后，产物位于：

```text
cmake-build-debug/Basic_Framework_MC02.elf
cmake-build-debug/Basic_Framework_MC02.hex
cmake-build-debug/Basic_Framework_MC02.bin
cmake-build-debug/Basic_Framework_MC02.map
```

## 4. 命令行构建

在工程根目录执行：

```powershell
cmake --preset debug
cmake --build --preset debug
```

Release 构建：

```powershell
cmake --preset release
cmake --build --preset release
```

## 5. 与原 Makefile 的对应关系

CMake 已沿用原 Makefile 中的关键配置：

- MCU：`-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard`
- 宏定义：`USE_HAL_DRIVER`、`STM32H723xx`、`ARM_MATH_CM7`、`DISABLE_LOG_SYSTEM`
- 链接脚本：`STM32H723VGTx_FLASH.ld`
- DSP 库：`Drivers/CMSIS/DSP/Lib/GCC/libarm_cortexM7lfdp_math.a`
- Debug 优化：`-Og -g -gdwarf-2`
- 输出：`elf`、`hex`、`bin`、`map`

## 6. 注意事项

- 当前 CMake 忠实保留原 Makefile 的 FreeRTOS portable 路径：`Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F`。虽然 MCU 是 Cortex-M7，但为了避免改变现有工程行为，暂未切换目录。
- `CMakeLists.txt` 中的源文件列表来自原 Makefile。后续新增 `.c` 文件时，需要同步加入 CMake 源文件列表。
- 若 CubeMX 重新生成工程并改动外设文件，也需要同步检查 `CMakeLists.txt`。
- 烧录和调试暂未写入 CMake 目标；可以继续使用 Ozone/J-Link/OpenOCD，调试文件选择 CMake 生成的 `.elf`。

