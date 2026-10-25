# CMake 构建说明 / CMake Build Guide

## 状态 / Status

- 中文：Keil 目标 `standard_robot`（STM32F407IGHx）已转换为 CMake + Ninja + GNU Arm 工具链构建，状态为 **已验证 / validated**。
- English: The Keil target `standard_robot` (STM32F407IGHx) now builds with CMake + Ninja + GNU Arm toolchain. Status: **已验证 / validated**.

## 环境检查 / Environment Check

中文：在项目根目录执行以下命令，确认必需工具可用；该脚本只检查，不安装或修改环境。

English: Run this command from the project root to confirm required tools. The script only inspects the environment; it never installs or changes anything.

```text
python check_build_env.py
```

## 编译 / Build

中文：在项目根目录依次执行以下命令。首次配置或更换工具链后使用 `--fresh` 清除旧缓存。

English: Run these commands from the project root. Use `--fresh` for the first configure or after changing the toolchain.

```text
cmake --preset vscode-debug --fresh
cmake --build --preset vscode-debug
```

中文：预设使用 Ninja、`cmake/arm-none-eabi-gcc.cmake`、`Debug` 构建类型，并输出 `compile_commands.json`。

English: The preset uses Ninja, `cmake/arm-none-eabi-gcc.cmake`, the `Debug` build type, and exports `compile_commands.json`.

## 编译成品 / Build Outputs

| 文件 / File | 简介 / Description |
| --- | --- |
| `build/vscode-debug/standard_robot.elf` | 可调试固件及符号 / Debuggable firmware and symbols |
| `build/vscode-debug/standard_robot.bin` | 原始烧录镜像 / Raw programming image |
| `build/vscode-debug/standard_robot.hex` | Intel HEX 烧录镜像 / Intel HEX programming image |
| `build/vscode-debug/standard_robot.map` | 链接映射与内存分布 / Link map and memory layout |

## 验证结果 / Validation Results

- 中文：源文件集合与 Keil 目标一致：164 个编译单元，无缺失、无重复；宏定义和包含路径完整。
- English: Source membership matches the Keil target: 164 compilation units, no missing or duplicate entries; defines and include paths are complete.
- 中文：向量表位于 `0x08000000`，共 98 项；`Reset_Handler` 为入口，ELF 使用 hard-float ABI，且不存在 RWX 段。
- English: The vector table is at `0x08000000` with 98 entries; `Reset_Handler` is the entry, the ELF uses the hard-float ABI, and no RWX segment exists.
- 中文：`validate_conversion.py` 通过，0 个错误、0 个警告。
- English: `validate_conversion.py` passes with 0 errors and 0 warnings.
- 中文：GNU 构建 Flash 使用 104744 B（9.99%），RAM 使用 66752 B（50.93%）。Keil 构建为 Flash 129624 B、RAM 66632 B。
- English: The GNU build uses 104744 B Flash (9.99%) and 66752 B RAM (50.93%). The Keil build uses 129624 B Flash and 66632 B RAM.
- 中文：Keil 对比基准来自现有 `MDK-ARM/standard_tpye_c` 产物（ARMCC 5.06），本次未重新运行 Keil。
- English: The Keil comparison baseline comes from the existing `MDK-ARM/standard_tpye_c` artifacts (ARMCC 5.06); Keil was not rebuilt in this run.

## 转换说明 / Conversion Notes

- 中文：Keil ARMCC 启动文件替换为 `gcc/startup_stm32f407xx.c`，向量表顺序与 Keil 一致。
- English: The Keil ARMCC startup file is replaced by `gcc/startup_stm32f407xx.c`; the vector-table order matches Keil.
- 中文：FreeRTOS 从 `portable/RVDS/ARM_CM4F` 切换为 `portable/GCC/ARM_CM4F`。
- English: FreeRTOS uses `portable/GCC/ARM_CM4F` instead of `portable/RVDS/ARM_CM4F`.
- 中文：`AHRS.lib` 和 `arm_cortexM4lf_math.lib` 替换为 `gcc/ahrs_compat.c` 与 `gcc/cmsis_dsp_compat.c`；当前实现只覆盖链接所需函数，数值行为未逐函数比对，后续使用更多库函数时需要补充并复核。
- English: `AHRS.lib` and `arm_cortexM4lf_math.lib` are replaced by `gcc/ahrs_compat.c` and `gcc/cmsis_dsp_compat.c`. The current implementations cover only linked functions and were not compared numerically function by function; extend and review them if more library APIs are used later.
- 中文：未向 GCC 传递 `__CC_ARM`，也未使用 RVDS 头文件路径。
- English: `__CC_ARM` and the RVDS include path are intentionally not propagated to GCC.
- 中文：GNU 启动代码先初始化 `.data`/`.bss`，再调用 `SystemInit`、`__libc_init_array` 和 `main`；Keil 启动代码在 C 运行时初始化前调用 `SystemInit`。
- English: The GNU startup initializes `.data`/`.bss` before calling `SystemInit`, `__libc_init_array`, and `main`; the Keil startup calls `SystemInit` before its C runtime initialization.
- 中文：使用 newlib-nano + nosys；活动代码没有浮点 `printf` 格式，因此未链接 `_printf_float`。标准输入输出保持丢弃行为。
- English: newlib-nano + nosys is used; active code has no floating-point `printf` formats, so `_printf_float` is not linked. Standard I/O remains intentionally discarded.
- 中文：Keil `-O1` 映射为 Debug 构建的 `-Og`；Release 构建使用 `-O2`，未添加 `NDEBUG`。
- English: Keil `-O1` maps to `-Og` in the Debug build; the Release build uses `-O2` and does not add `NDEBUG`.
- 中文：GNU 栈顶为 `0x20020000`（RAM 末端），Keil 为 `0x20010448`；两者堆/栈预留均为 8 KiB。
- English: The GNU stack top is `0x20020000` (end of RAM); Keil uses `0x20010448`. Both reserve 8 KiB each for heap and stack.
- 中文：Keil 输出名为 `standard_tpye_c`，CMake 输出名为 `standard_robot`。
- English: The Keil output name is `standard_tpye_c`; the CMake output name is `standard_robot`.
- 中文：编译期仍有应用头文件产生的 GCC 警告（如 `extern inline` 声明），不影响链接；硬件烧录和运行冒烟测试未执行。
- English: GCC still reports application-header warnings such as `extern inline` declarations; linking is unaffected. Hardware flashing and runtime smoke tests were not performed.

## 复验命令 / Re-validation Command

中文：在项目根目录执行以下命令可复现源文件、宏、包含路径、产物和 ELF 检查。该命令使用同级目录中的 `convert-keil-to-cmake` 技能脚本；日常编译不依赖该目录。

English: Run this command from the project root to reproduce the source, define, include, artifact, and ELF checks. It uses the sibling `convert-keil-to-cmake` skill scripts; normal builds do not depend on that directory.

```text
python ../convert-keil-to-cmake/scripts/validate_conversion.py MDK-ARM/standard_robot.uvprojx build/vscode-debug/compile_commands.json --target standard_robot --source-substitution "MDK-ARM/startup_stm32f407xx.s=gcc/startup_stm32f407xx.c" --source-substitution "Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F/port.c=Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c" --source-substitution "components/algorithm/AHRS.lib=gcc/ahrs_compat.c" --source-substitution "components/algorithm/arm_cortexM4lf_math.lib=gcc/cmsis_dsp_compat.c" --ignore-define __CC_ARM --ignore-include Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F --elf build/vscode-debug/standard_robot.elf
```
