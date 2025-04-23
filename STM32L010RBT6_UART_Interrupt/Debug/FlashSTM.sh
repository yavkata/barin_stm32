#!/bin/bash

# Automatically find ELF file with target name
ELF_FILE=$(find . -maxdepth 1 -name "*STM32L010RBT6*.elf" | head -n 1)

if [ -z "$ELF_FILE" ]; then
    echo "Error: No ELF file with 'STM32L010RBT6' in its name found."
    exit 1
fi

BIN_FILE="${ELF_FILE%.elf}.bin"

# Step 2: Convert ELF to BIN
echo "Converting $ELF_FILE to binary..."
arm-none-eabi-objcopy -O binary "$ELF_FILE" "$BIN_FILE"
if [ $? -ne 0 ]; then
    echo "Error: Failed to convert ELF to binary."
    exit 1
fi
echo "Conversion successful: $BIN_FILE"
sleep 1

# Step 3: Enter Bootloader mode and program the MCU
echo "Putting MCU in bootloader mode..."
gpioset gpiochip0 2=1
gpioset gpiochip0 3=0
sleep 0.1
gpioset gpiochip0 3=1
sleep 1

echo "Programming MCU using stm32flash..."
PAGE_SIZE=128  # Adjust this to your MCU's flash page size
BIN_SIZE=$(stat -c%s "$BIN_FILE")
NUM_PAGES=$(( (BIN_SIZE + PAGE_SIZE - 1) / PAGE_SIZE ))

stm32flash -e $NUM_PAGES -w "$BIN_FILE" -v -g 0x08000000 /dev/ttyUSB0
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

