#!/bin/bash

# Set BOOT0 high, RESET low
gpioset gpiochip0 2=1 3=0
sleep 0.1

# Set RESET high
gpioset gpiochip0 3=1
sleep 0.5

# Attempt to communicate with the bootloader
#stm32flash -g 0x0 /dev/ttyUSB0
#stm32flash -e 100 -w STM32L010RBT6.bin -v -g 0x08000000 /dev/ttyUSB0
BIN_FILE="STM32L010RBT6.bin"
PAGE_SIZE=128  # Adjust this to your MCU's flash page size

# Get the size of the binary file in bytes
BIN_SIZE=$(stat -c%s "$BIN_FILE")

# Calculate the number of pages needed
NUM_PAGES=$(( (BIN_SIZE + PAGE_SIZE - 1) / PAGE_SIZE ))

# Run stm32flash with the calculated erase parameter
stm32flash -e $NUM_PAGES -w $BIN_FILE -v -g 0x08000000 /dev/ttyUSB0
