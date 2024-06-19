cmake_minimum_required(VERSION 3.15)

#进行交叉编译需要指定运行平台, 否则在最后生成可执行程序时会自动加上exe后缀
set(CMAKE_SYSTEM_NAME   Generic)  #设置目标系统, windows, linux, Generic(无操作系统的裸机)
set(CMAKE_SYSTEM_VERSION  1)  #目标系统的版本, 用得不多
set(CMAKE_SYSTEM_PROCESSOR  cortex-m4)  # 目标处理器, Arm, ARM64, x86, x64, MIPS, ...

set(CMAKE_HOST_SYSTEM_NAME  Windows)  #设置主机系统, windows, linux, Generic(无操作系统的裸机)
set(CMAKE_HOST_SYSTEM_VERSION  1)  #主机系统的版本, 用得不多
set(CMAKE_HOST_SYSTEM_PROCESSOR  x64)  # 主机处理器, Arm, ARM64, x86, x64, MIPS, ...

set(CMAKE_C_COMPILER_FORCED YES)  #强制使用指定的编译器
set(CMAKE_CXX_COMPILER_FORCED YES)  #强制使用指定的编译器

#编译器

set(CMAKE_C_COMPILER  D:/gcc/arm-none-eabi-gcc/bin/arm-none-eabi-gcc.exe)
set(CMAKE_ASM_COMPILER  D:/gcc/arm-none-eabi-gcc/bin/arm-none-eabi-gcc.exe)
set(CMAKE_CXX_COMPILER  D:/gcc/arm-none-eabi-gcc/bin/arm-none-eabi-g++.exe)

#工具
set(CMAKE_OBJCOPY D:/gcc/arm-none-eabi-gcc/bin/arm-none-eabi-objcopy.exe)  #用于生成可bin文件
set(CMAKE_SIZE D:/gcc/arm-none-eabi-gcc/bin/arm-none-eabi-size.exe)  #用于显示文件小

project(led C CXX ASM)

#宏定义
add_definitions(
    -DGD32F470
)

#编译选项
set(MCU_FLAGS "-mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16")
set(CMAKE_C_FLAGS "${MCU_FLAGS} -w -Wno-unknown-pragmas")
set(CMAKE_C_FLAGS_DEBUG "-O0 -g2 -ggdb")
set(CMAKE_C_FLAGS_RELEASE "-O3")

#头文件
include_directories(
    Firmware/GD32F4xx_standard_peripheral/Include
    Firmware/CMSIS
    Firmware/CMSIS/GD/GD32F4xx/Include
    User
    Hardware/led
    #${CMAKE_CURRENT_BINARY_DIR}
)



set(LINKER_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/Firmware/CMSIS/GD/GD32F4xx/Source/GCC/gd32_rom.ld)
set(CMAKE_EXE_LINKER_FLAGS
"--specs=nano.specs -specs=nosys.specs -nostartfiles -T${LINKER_SCRIPT} -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map,--cref -Wl,--gc-sections"
)
#生成可执行文件
file(GLOB userSRC User/*.c)
file(GLOB hardSRC Hardware/led/*.c)
# message("** led.c = " ${hardSRC})
file(GLOB gdSRC Firmware/GD32F4xx_standard_peripheral/Source/*.c)
# message("** led.c = " ${gdSRC})
set(cmsisSRC Firmware/CMSIS/GD/GD32F4xx/Source/GCC/startup_gd32f4xx.S 
            Firmware/CMSIS/GD/GD32F4xx/Source/system_gd32f4xx.c)


add_executable(led.elf ${userSRC} ${hardSRC} ${gdSRC} ${cmsisSRC})


set(ELF_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.elf)
set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET "${PROJECT_NAME}.elf" POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Obinary ${ELF_FILE} ${BIN_FILE}
    COMMAND ${CMAKE_OBJCOPY} -Oihex  ${ELF_FILE} ${HEX_FILE}
    COMMENT "Building ${PROJECT_NAME}.bin and ${PROJECT_NAME}.hex"

    COMMAND ${CMAKE_COMMAND} -E copy ${HEX_FILE} "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.hex"
    COMMAND ${CMAKE_COMMAND} -E copy ${BIN_FILE} "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin"

    COMMAND ${CMAKE_SIZE} --format=berkeley ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMENT "Invoking: Cross ARM GNU Print Size"
)

