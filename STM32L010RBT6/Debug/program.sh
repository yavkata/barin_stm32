#!/bin/bash

# Set BOOT0 high, RESET low
gpioset gpiochip0 2=1 3=0
sleep 0.1

# Set RESET high
gpioset gpiochip0 3=1
sleep 0.5

# Attempt to communicate with the bootloader
#stm32flash -g 0x0 /dev/ttyUSB0
stm32flash -e 100 -w STM32L010RBT6.bin -v -g 0x08000000 /dev/ttyUSB0
