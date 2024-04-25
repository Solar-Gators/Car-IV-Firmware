cmake_minimum_required(VERSION 3.5)
########################################################################
# Toolchain file must go at the top
########################################################################
set(RESOURCES_DIR ./../../Resources)
set(CMAKE_TOOLCHAIN_FILE ${RESOURCES_DIR}/CMake/arm-cm4.cmake)
# Choose between Debug and Release builds
set(CMAKE_BUILD_TYPE Debug)

########################################################################
# MCU information
# Edit this section based on the target platform
########################################################################
set(MCU_FAMILY STM32L4xx)
set(MCU_MODEL STM32L496xx)
set(MCU_PART STM32L496RGTx)
string(TOLOWER ${MCU_FAMILY} MCU_FAMILY_LOWER)
string(TOLOWER ${MCU_MODEL} MCU_MODEL_LOWER)

########################################################################
# Includes
########################################################################
include_directories(
    Core/Inc
    User/Inc
    ${RESOURCES_DIR}/Drivers/CMSIS/Include
    ${RESOURCES_DIR}/Drivers/CMSIS/Device/ST/${MCU_FAMILY}/Include
    ${RESOURCES_DIR}/Drivers/${MCU_FAMILY}_HAL_Driver/Inc
    ${RESOURCES_DIR}/Drivers/segger-rtt-master/RTT
    ${RESOURCES_DIR}/Drivers/segger-rtt-master/Config
    ${RESOURCES_DIR}/Drivers/SG_Drivers/Inc
    ${RESOURCES_DIR}/Drivers/DataModules
    ${RESOURCES_DIR}/Middlewares/Third_Party/etl/include
    ${RESOURCES_DIR}/Middlewares/Third_Party/FreeRTOS/Source/include
    ${RESOURCES_DIR}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
    ${RESOURCES_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    ${RESOURCES_DIR}/Middlewares/Third_Party/FatFs/src
)

########################################################################
# Sources
########################################################################
file(GLOB_RECURSE RECURSE_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/*.s
    ${CMAKE_CURRENT_SOURCE_DIR}/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
    ${RESOURCES_DIR}/Drivers/${MCU_FAMILY}_HAL_Driver/Src/*.c
    ${RESOURCES_DIR}/Drivers/segger-rtt-master/RTT/*.c
    ${RESOURCES_DIR}/Middlewares/Third_Party/FatFs/src/*.c
)
file(GLOB GLOB_SOURCES
    ${RESOURCES_DIR}/Middlewares/Third_Party/FreeRTOS/Source/*.c
    ${RESOURCES_DIR}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c
    ${RESOURCES_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    ${RESOURCES_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
)
file(GLOB SG_SOURCES
    ${RESOURCES_DIR}/Drivers/SG_Drivers/Src/logger.cpp
    ${RESOURCES_DIR}/Drivers/SG_Drivers/Src/sg_can.cpp
    ${RESOURCES_DIR}/Drivers/SG_Drivers/Src/DACx311.cpp
    ${RESOURCES_DIR}/Drivers/SG_Drivers/Src/button.cpp
)
list(REMOVE_ITEM GLOB_SOURCES ${COMMON_DIR}/${MCU_FAMILY}_HAL_Driver/Src/${MCU_FAMILY_LOWER}_hal_msp_template.c)
list(REMOVE_ITEM GLOB_SOURCES ${COMMON_DIR}/${MCU_FAMILY}_HAL_Driver/Src/${MCU_FAMILY_LOWER}_hal_timebase_tim_template.c)
list(REMOVE_ITEM GLOB_SOURCES ${COMMON_DIR}/${MCU_FAMILY}_HAL_Driver/Src/${MCU_FAMILY_LOWER}_hal_timebase_rtc_wakeup_template.c)
list(REMOVE_ITEM GLOB_SOURCES ${COMMON_DIR}/${MCU_FAMILY}_HAL_Driver/Src/${MCU_FAMILY_LOWER}_hal_timebase_rtc_alarm_template.c)

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
# Executable build instructions
########################################################################
get_filename_component(SOURCE_DIR_NAME ${CMAKE_SOURCE_DIR} NAME)
project(${SOURCE_DIR_NAME})

# Add executable
add_executable(${PROJECT_NAME} ${RECURSE_SOURCES} ${GLOB_SOURCES} ${SG_SOURCES})
target_compile_definitions(${PROJECT_NAME} PRIVATE
    ${MCU_MODEL}
    USE_HAL_DRIVER
)

########################################################################
# Linker options
########################################################################
set(CMAKE_EXE_LINKER_FLAGS
    "-specs=nosys.specs \
    -T${CMAKE_CURRENT_SOURCE_DIR}/${MCU_PART}_FLASH.ld \
    -Wl,--gc-sections \
    -Wl,-Map=${PROJECT_NAME}.map,--cref \
    -Wl,--print-memory-usage"
)
target_link_libraries(${PROJECT_NAME} c m nosys)

########################################################################
# Post build commands
# Get section sizes, convert executable to .hex and .bin
######################################################################## 
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary -S $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
)