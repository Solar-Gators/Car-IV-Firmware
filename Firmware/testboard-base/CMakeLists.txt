########################################################################
# Toolchain file must go at the top
########################################################################
cmake_minimum_required(VERSION 3.12)
set(RESOURCES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../Resources)
set(CMAKE_TOOLCHAIN_FILE ${RESOURCES_DIR}/CMake/arm-none-eabi.cmake)
set(CMAKE_BUILD_TYPE Debug)

########################################################################
# MCU information
# Edit this section based on the target platform
########################################################################
set(MCU_FAMILY STM32L4xx)
set(MCU_MODEL STM32L496xx)
set(MCU_PART STM32L496RGTx)

set(CPU_PARAMETERS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard)

########################################################################
# Includes
# Edit this section based on desired include files
########################################################################
set(STM_HAL_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
    ${RESOURCES_DIR}/Drivers/${MCU_FAMILY}_HAL_Driver/Inc
    ${RESOURCES_DIR}/Drivers/${MCU_FAMILY}_HAL_Driver/Inc/Legacy
    ${RESOURCES_DIR}/Drivers/CMSIS/Include
    ${RESOURCES_DIR}/Drivers/CMSIS/Device/ST/${MCU_FAMILY}/Include)
set(THIRD_PARTY_INCLUDES
    ${RESOURCES_DIR}/Drivers/segger-rtt-master/RTT
    ${RESOURCES_DIR}/Drivers/segger-rtt-master/Config
    ${RESOURCES_DIR}/Middlewares/Third_Party/etl/include
    ${RESOURCES_DIR}/Middlewares/Third_Party/FreeRTOS/Source/include
    ${RESOURCES_DIR}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
    ${RESOURCES_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F)
set(USER_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Inc
    ${RESOURCES_DIR}/Drivers/SG_Drivers/Inc)

########################################################################
# Sources
# Edit this section based on drivers used
########################################################################
file(GLOB_RECURSE RECURSE_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/*.s
    ${CMAKE_CURRENT_SOURCE_DIR}/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
    ${RESOURCES_DIR}/Drivers/${MCU_FAMILY}_HAL_Driver/Src/*.c
    ${RESOURCES_DIR}/Drivers/segger-rtt-master/RTT/*.c)
file(GLOB GLOB_SOURCES
    ${RESOURCES_DIR}/Middlewares/Third_Party/FreeRTOS/Source/*.c
    ${RESOURCES_DIR}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c
    ${RESOURCES_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    ${RESOURCES_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c)
file(GLOB DRIVER_SOURCES
    ${RESOURCES_DIR}/Drivers/SG_Drivers/Src/logger.cpp)
    
    
######### You shouldn't have to edit anything below this line ##########
    
########################################################################
# Set project and executable name to match top-level project folder
########################################################################
get_filename_component(SOURCE_DIR_NAME ${CMAKE_SOURCE_DIR} NAME)
project(${SOURCE_DIR_NAME})
set(EXECUTABLE ${CMAKE_PROJECT_NAME})
add_executable(${EXECUTABLE} ${GLOB_SOURCES} ${RECURSE_SOURCES} ${DRIVER_SOURCES})

########################################################################
# Define startup ASM file and linker script
########################################################################
string(TOLOWER ${MCU_MODEL} MCU_MODEL_L)
set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/${MCU_PART}_FLASH.ld)

########################################################################
# Language settings
########################################################################
enable_language(C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

########################################################################
# Defines (after add_executable!)
########################################################################
target_compile_definitions(${EXECUTABLE} PRIVATE
    ${MCU_MODEL}
    USE_HAL_DRIVER)

########################################################################
# Includes (after add_executable!)
########################################################################
target_include_directories(${EXECUTABLE} PRIVATE
    ${STM_HAL_INCLUDES}
    ${USER_INCLUDES}
    ${THIRD_PARTY_INCLUDES})

########################################################################
# Compiler and linker options (after add_executable!)
########################################################################
target_compile_options(${EXECUTABLE} PRIVATE
    ${CPU_PARAMETERS}
    -Wall
    -Wextra
    -Wno-unused-parameter
    $<$<COMPILE_LANGUAGE:CXX>:
        -Wno-volatile
        -Wsuggest-override>
    $<$<CONFIG:Debug>:-Og -g3 -ggdb>
    $<$<CONFIG:Release>:-O3 -g0>)

target_link_options(${EXECUTABLE} PRIVATE
    ${CPU_PARAMETERS}
    -T${LINKER_SCRIPT}
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map,--cref
    --specs=nosys.specs
    -Wl,--undefined,_printf_float
    -Wl,--no-warn-rwx-segment
    -Wl,--start-group
    -lc
    -lm
    -lnosys
    -Wl,--end-group
    -Wl,--gc-sections,--print-memory-usage)

########################################################################
# Post build commands
# Get section sizes, convert executable to .hex and .bin
######################################################################## 
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${EXECUTABLE}>)

add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${EXECUTABLE}> ${EXECUTABLE}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${EXECUTABLE}> ${EXECUTABLE}.bin)

    