set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Some default GCC settings
set(TOOLCHAIN_PREFIX arm-none-eabi-)

set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_SIZE ${TOOLCHAIN_PREFIX}size)

set(CMAKE_EXECUTABLE_SUFFIX_ASM ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX ".elf")

# Set Ninja as the default generator if not specified
if(NOT CMAKE_GENERATOR)
    set(CMAKE_GENERATOR Ninja CACHE STRING "Default generator" FORCE)
endif()


# Compiler flags
set(CMAKE_C_FLAGS 
    "-mcpu=cortex-m4 \
    -mthumb \
    -mfpu=fpv4-sp-d16 \
    -mfloat-abi=hard \
    -Wall -fdata-sections -ffunction-sections"
)
set (CMAKE_CXX_FLAGS
    "${CMAKE_C_FLAGS} \
    -Wno-volatile"
)
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS} -Og -g -gdwarf-2")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS} -Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Og -g -gdwarf-2")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -Os -g0")
set(CMAKE_ASM_FLAGS "-x assembler-with-cpp")

# Make sure CMake doesn't get confused when cross-compiling
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)