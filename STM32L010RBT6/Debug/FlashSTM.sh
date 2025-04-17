#!/bin/bash

# Step 2: Convert ELF to BIN
echo "Converting ELF to binary..."
arm-none-eabi-objcopy -O binary STM32L010RBT6.elf STM32L010RBT6.bin
if [ $? -ne 0 ]; then
    echo "Error: Failed to convert ELF to binary."
    exit 1
fi
echo "Conversion successful."
sleep 1

# Step 3: Enter Bootloader mode and program the MCU
echo "Putting MCU in bootloader mode..."
gpioset gpiochip0 2=1
gpioset gpiochip0 3=0
sleep 0.1
gpioset gpiochip0 3=1
sleep 1

echo "Programming MCU using stm32flash..."
#stm32flash -e 100 -w STM32L010RBT6.bin -v -g 0x08000000 /dev/ttyUSB0
BIN_FILE="STM32L010RBT6.bin"
PAGE_SIZE=128  # Adjust this to your MCU's flash page size

# Get the size of the binary file in bytes
BIN_SIZE=$(stat -c%s "$BIN_FILE")

# Calculate the number of pages needed
NUM_PAGES=$(( (BIN_SIZE + PAGE_SIZE - 1) / PAGE_SIZE ))

# Run stm32flash with the calculated erase parameter
stm32flash -e $NUM_PAGES -w $BIN_FILE -v -g 0x08000000 /dev/ttyUSB0
if [ $? -ne 0 ]; then
    echo "Error: Failed to program the MCU."
    exit 1
fi
echo "Programming successful."
sleep 1

# Step 4: Run the code
echo "Running the new firmware..."
gpioset gpiochip0 2=0
gpioset gpiochip0 3=0
sleep 0.1
gpioset gpiochip0 3=1
sleep 1

echo "Done! The MCU should now be running the new code."
