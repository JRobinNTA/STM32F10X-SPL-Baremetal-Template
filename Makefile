# Makefile for STM32F103 "Blue Pill" project

# Project name
TARGET = stm32f10x_display

# Toolchain definitions
TOOLCHAIN = arm-none-eabi-
CC = $(TOOLCHAIN)gcc
CXX = $(TOOLCHAIN)g++
LD = $(TOOLCHAIN)ld
OBJCOPY = $(TOOLCHAIN)objcopy
OBJDUMP = $(TOOLCHAIN)objdump
SIZE = $(TOOLCHAIN)size

# Directories
INC_DIR = inc
SRC_DIR = src
STARTUP_DIR = startup
STD_PERIPH_DIR = STM32F10x_StdPeriph_Driver
STD_PERIPH_INC = $(STD_PERIPH_DIR)/inc
STD_PERIPH_SRC = $(STD_PERIPH_DIR)/src

# Source files
C_SOURCES = \
	$(SRC_DIR)/main.c \
	$(SRC_DIR)/stm32f10x_it.c \
	$(SRC_DIR)/system_stm32f10x.c

# ASM sources
ASM_SOURCES = $(STARTUP_DIR)/startup_stm32f10x_md.s

# Include paths
INCLUDES = \
	-I$(INC_DIR) \
	-I$(STD_PERIPH_INC)

# MCU flags
MCU = -mcpu=cortex-m3 -mthumb -mfloat-abi=soft

# Compiler flags
CFLAGS = $(MCU) $(INCLUDES) -O2 -Wall -fdata-sections -ffunction-sections
CFLAGS += -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER

# Linker flags
LDFLAGS = $(MCU) -T stm32f103c8_flash.ld -Wl,--gc-sections

# Build directory
BUILD_DIR = build

# List of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))

# Make sure build directory exists
$(shell mkdir -p $(BUILD_DIR))

# Default target
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

# Compile C files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) -c $(CFLAGS) $< -o $@

# Compile startup assembly file
$(BUILD_DIR)/%.o: $(STARTUP_DIR)/%.s
	$(CC) -c $(CFLAGS) -x assembler-with-cpp $< -o $@

# Link
$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SIZE) $@

# Create binary
$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

# Create hex file
$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

# Flash the board (requires st-flash tool from stlink)
flash: $(BUILD_DIR)/$(TARGET).bin
	st-flash write $(BUILD_DIR)/$(TARGET).bin 0x8000000

# Clean
clean:
	rm -rf $(BUILD_DIR)

.PHONY: all flash clean
