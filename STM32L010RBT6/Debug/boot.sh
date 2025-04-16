#!/bin/bash

# GPIO line numbers for BOOT0 and RESET
BOOT0_LINE=2
RESET_LINE=3

# Disable Bootloader Mode: Set BOOT0 low (0)
gpioset gpiochip0 $BOOT0_LINE=0

# Short delay to ensure the state is stable
sleep 0.1

# Reset the microcontroller:
# Pull RESET low (0) and then back high (1)
gpioset gpiochip0 $RESET_LINE=0
sleep 0.1
gpioset gpiochip0 $RESET_LINE=1

