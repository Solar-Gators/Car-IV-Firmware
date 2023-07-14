######################################
# target
######################################
TARGET = $(notdir $(patsubst %/,%,$(CURDIR)))

######################################
# building variables
######################################
DEBUG = 1
OPT = -Og

#################################
# GNU ARM Embedded Toolchain
#################################
GCC_PATH = ../../Tools/gcc-arm-none-eabi-10.3-2021.10/bin
PREFIX = arm-none-eabi-
CC = $(GCC_PATH)/$(PREFIX)gcc
CXX = $(GCC_PATH)/$(PREFIX)g++
LD = $(GCC_PATH)/$(PREFIX)ld
AR = $(GCC_PATH)/$(PREFIX)ar
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
OD = $(GCC_PATH)/$(PREFIX)objdump
NM = $(GCC_PATH)/$(PREFIX)nm
SZ = $(GCC_PATH)/$(PREFIX)size
A2L = $(GCC_PATH)/$(PREFIX)addr2line

#######################################
# paths
#######################################
# Build path
BUILD_DIR = build
RESOURCES_DIR = ../../Resources

######################################
# sources and includes
######################################
# C sources
C_SOURCES =  \
Core/Src/main.c \
Core/Src/freertos.c \
Core/Src/stm32l4xx_it.c \
Core/Src/stm32l4xx_hal_msp.c \
Core/Src/stm32l4xx_hal_timebase_tim.c \
Core/Src/system_stm32l4xx.c \
$(RESOURCES_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c \
$(RESOURCES_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.c \
$(RESOURCES_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.c \
$(RESOURCES_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.c \
$(RESOURCES_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.c \
$(RESOURCES_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.c \
$(RESOURCES_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.c \
$(RESOURCES_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.c \
$(RESOURCES_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.c \
$(RESOURCES_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.c \
$(RESOURCES_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.c \
$(RESOURCES_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.c \
$(RESOURCES_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.c \
$(RESOURCES_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.c \
$(RESOURCES_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.c \
$(RESOURCES_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.c \
$(RESOURCES_DIR)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.c \
$(RESOURCES_DIR)/Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
$(RESOURCES_DIR)/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
$(RESOURCES_DIR)/Middlewares/Third_Party/FreeRTOS/Source/list.c \
$(RESOURCES_DIR)/Middlewares/Third_Party/FreeRTOS/Source/queue.c \
$(RESOURCES_DIR)/Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
$(RESOURCES_DIR)/Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
$(RESOURCES_DIR)/Middlewares/Third_Party/FreeRTOS/Source/timers.c \
$(RESOURCES_DIR)/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c \
$(RESOURCES_DIR)/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
$(RESOURCES_DIR)/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c

# C++ sources
CXX_SOURCES = \
User/Src/user.cpp

# ASM sources
ASM_SOURCES =  \
startup_stm32l476xx.s

# AS includes
AS_INCLUDES =  \
-ICore/Inc

# C/C++ includes
INCLUDES =  \
-ICore/Inc \
-IUser/Inc \
-I$(RESOURCES_DIR)/Drivers/STM32L4xx_HAL_Driver/Inc \
-I$(RESOURCES_DIR)/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy \
-I$(RESOURCES_DIR)/Middlewares/Third_Party/FreeRTOS/Source/include \
-I$(RESOURCES_DIR)/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 \
-I$(RESOURCES_DIR)/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
-I$(RESOURCES_DIR)/Drivers/CMSIS/Device/ST/STM32L4xx/Include \
-I$(RESOURCES_DIR)/Drivers/CMSIS/Include

#######################################
# hardware definitions
#######################################
# cpu
CPU = -mcpu=cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

#######################################
# compile flags
#######################################
# AS defines
AS_DEFINES = 

# C defines
DEFINES =  \
-DUSE_HAL_DRIVER \
-DSTM32L476xx

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFINES) $(INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -fstack-usage
CFLAGS = $(MCU) $(DEFINES) $(INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -fstack-usage
CXXFLAGS = $(MCU) $(DEFINES) $(INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -fstack-usage

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
CXXFLAGS += -g -gdwarf-2
endif

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"
CXXFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32L476RGTx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys
LIBDIR = 
LDFLAGS = $(MCU) -specs=nosys.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections,--print-memory-usage

#######################################
# Objects
#######################################
# list of C program objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of C++ program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(CXX_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CXX_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

#######################################
# build the application
#######################################
# default action: build all
all: $(BUILD_DIR)/$(TARGET).hex

# Convert the target elf file to hex
$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(CP) -O ihex $< $@

# Link the object files to generate target elf executable
$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS)
	$(CXX) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

# Generate object files from C++ source files
$(BUILD_DIR)/%.o: %.cpp $(BUILD_DIR) 
	$(CXX) -c $(CXXFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

# Generate object files from C source files
$(BUILD_DIR)/%.o: %.c $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

# Generate object files from ASM source files
$(BUILD_DIR)/%.o: %.s $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

# Create the build directory if it doesn't exist
$(BUILD_DIR):
	mkdir -p $@

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)