/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MP2760_ADDR  (0x69 << 1) // 8-bit format for HAL functions


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void MP2760_WriteRegister(uint8_t reg, uint8_t value);
uint8_t MP2760_ReadRegister(uint8_t reg);
uint16_t MP2760_ReadRegister16(uint8_t reg);
float MP2760_ReadVoltage(uint8_t msb_reg);
void MP2760_PrintStatus(void);
void MP2760_PrintVoltages(void);
void MP2760_PrintThresholds(void);
void MP2760_PrintFaults(void);
void MP2760_Dashboard(void);
void MP2760_PrintAllRegisters(void);
void MP2760_Print_REG05(void);
void MP2760_Print_REG16(void);
void MP2760_Print_REG23(void);
void MP2760_Print_REG26(void);
void MP2760_Print_REG17(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void MP2760_WriteRegister(uint8_t reg, uint8_t value)
{
    HAL_I2C_Mem_Write(&hi2c1, MP2760_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
}

uint8_t MP2760_ReadRegister(uint8_t reg)
{
    uint8_t value = 0;
    HAL_I2C_Mem_Read(&hi2c1, MP2760_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
    return value;
}

uint16_t MP2760_ReadRegister16(uint8_t reg)
{
    uint8_t buf[2] = {0};
    HAL_I2C_Mem_Read(&hi2c1, MP2760_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY);
    return ((uint16_t)buf[1] << 8) | buf[0]; // LSB first
}

float MP2760_ReadVoltage(uint8_t reg)
{
    uint16_t raw = MP2760_ReadRegister16(reg);
    return raw * 0.00488f;
}

void MP2760_PrintAllRegisters(void)
{
    char msg[128];

    snprintf(msg, sizeof(msg), "REG05  Device Address Settings:  0x%04X\r\n", MP2760_ReadRegister16(0x05));
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "REG06  Input Minimum Voltage Limit Settings:  0x%04X\r\n", MP2760_ReadRegister16(0x06));
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "REG07  System Minimum Voltage Settings:  0x%04X\r\n", MP2760_ReadRegister16(0x07));
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "REG08  Input Current Limit Settings:  0x%04X\r\n", MP2760_ReadRegister16(0x08));
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "REG09  Output Voltage Settings in Source Mode:  0x%04X\r\n", MP2760_ReadRegister16(0x09));
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "REG0A  Battery Impedance Compensation and Output Current Limit Settings:  0x%04X\r\n", MP2760_ReadRegister16(0x0A));
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "REG0B  Battery Low Voltage Settings and Battery Discharge Current Regulation:  0x%04X\r\n", MP2760_ReadRegister16(0x0B));
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "REG0C  JEITA Action Settings:  0x%04X\r\n", MP2760_ReadRegister16(0x0C));
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "REG0D  Temperature Protection Settings:  0x%04X\r\n", MP2760_ReadRegister16(0x0D));
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "REG0E  Configuration Register 0:  0x%04X\r\n", MP2760_ReadRegister16(0x0E));
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "REG0F  Configuration Register 1:  0x%04X\r\n", MP2760_ReadRegister16(0x0F));
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "REG10  Configuration Register 2:  0x%04X\r\n", MP2760_ReadRegister16(0x10));
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "REG11  Configuration Register 3:  0x%04X\r\n", MP2760_ReadRegister16(0x11));
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "REG12  Configuration Register 4:  0x%04X\r\n", MP2760_ReadRegister16(0x12));
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "REG14  Charge Current Setting:  0x%04X\r\n", MP2760_ReadRegister16(0x14));
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "REG15  Battery-Full Voltage Setting:  0x%04X\r\n", MP2760_ReadRegister16(0x15));
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "REG16  Status and Fault Register 0:  0x%04X\r\n", MP2760_ReadRegister16(0x16));
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "REG17  Status and Fault Register 1:  0x%04X\r\n", MP2760_ReadRegister16(0x17));
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "REG18  INT Mask Setting Register 0:  0x%04X\r\n", MP2760_ReadRegister16(0x18));
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "REG19  INT Mask Setting Register 1:  0x%04X\r\n", MP2760_ReadRegister16(0x19));
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "REG22  Internal DAC Output of the Input Current Limit:  0x%04X\r\n", MP2760_ReadRegister16(0x22));
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "REG23  ADC Result of the Input Voltage:  0x%04X\r\n", MP2760_ReadRegister16(0x23));
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "REG24  ADC Result of the Input Current:  0x%04X\r\n", MP2760_ReadRegister16(0x24));
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "REG25  ADC Result of the Battery Voltage per Cell:  0x%04X\r\n", MP2760_ReadRegister16(0x25));
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "REG26  ADC Result of the System Voltage:  0x%04X\r\n", MP2760_ReadRegister16(0x26));
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "REG27  ADC Result of the Battery Charge Current:  0x%04X\r\n", MP2760_ReadRegister16(0x27));
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "REG28  ADC Result of the NTC-Sense Voltage Ratio:  0x%04X\r\n", MP2760_ReadRegister16(0x28));
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "REG29  ADC Result of the TS-Sense Voltage Ratio:  0x%04X\r\n", MP2760_ReadRegister16(0x29));
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "REG2A  ADC Result of the Junction Temperature:  0x%04X\r\n", MP2760_ReadRegister16(0x2A));
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "REG2B  ADC Result of the Battery Discharge Current in Source Mode:  0x%04X\r\n", MP2760_ReadRegister16(0x2B));
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "REG2C  ADC Result of the Output Voltage in Source Mode:  0x%04X\r\n", MP2760_ReadRegister16(0x2C));
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "REG2D  ADC Result of the Output Current in Source Mode :  0x%04X\r\n", MP2760_ReadRegister16(0x2D));
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void MP2760_PrintStatus(void)
{
    uint8_t input_status = MP2760_ReadRegister(0x0A);
    uint8_t charger_status = MP2760_ReadRegister(0x0B);

    char msg[128];
    snprintf(msg, sizeof(msg),
             "Input Status: ACOK:%d Input_Present:%d Adapter_Present:%d\r\n"
             "Charging Status: Charging_Enable:%d Charger_State:0x%02X\r\n",
             (input_status >> 7) & 0x01,
             (input_status >> 5) & 0x01,
             (input_status >> 3) & 0x01,
             (charger_status >> 7) & 0x01,
             charger_status & 0x7F);

    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void MP2760_PrintVoltages(void)
{
    float vin = MP2760_ReadVoltage(0x1C);
    float vbat = MP2760_ReadVoltage(0x1E);
    float vsys = MP2760_ReadVoltage(0x20);

    char msg[128];
    snprintf(msg, sizeof(msg),
             "Voltages: VIN:%.2fV VBAT:%.2fV VSYS:%.2fV\r\n",
             vin, vbat, vsys);

    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void MP2760_PrintThresholds(void)
{
    uint8_t vin_ovp = MP2760_ReadRegister(0x11) >> 6; // Bits 7:6
    uint8_t vin_uvlo = (MP2760_ReadRegister(0x11) >> 4) & 0x03; // Bits 5:4
    uint8_t sys_reg_raw = MP2760_ReadRegister(0x15);
    uint8_t vbat_reg_raw = MP2760_ReadRegister(0x14);

    float sys_reg = 3.84 + (sys_reg_raw * 0.032);
    float vbat_reg = 3.84 + (vbat_reg_raw * 0.032);

    char msg[128];
    const char* vin_ovp_str[] = { "14.4V", "18.4V", "20.0V", "22.4V" };
    const char* vin_uvlo_str[] = { "3.2V", "4.2V", "4.6V", "4.8V" };

    snprintf(msg, sizeof(msg),
             "Thresholds:\r\n"
             "VIN_OVP Setting: %d (%s)\r\n"
             "VIN_UVLO Setting: %d (%s)\r\n"
             "SYS_REG: %.2fV\r\n"
             "VBAT_REG: %.2fV\r\n",
             vin_ovp, vin_ovp_str[vin_ovp],
             vin_uvlo, vin_uvlo_str[vin_uvlo],
             sys_reg, vbat_reg);

    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void MP2760_PrintFaults(void)
{
    uint8_t fault_status = MP2760_ReadRegister(0x0C);
    char msg[256];

    snprintf(msg, sizeof(msg), "Faults: 0x%02X\r\n", fault_status);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    if (fault_status == 0) {
        snprintf(msg, sizeof(msg), "No faults detected.\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return;
    }

    if (fault_status & 0x10) HAL_UART_Transmit(&huart2, (uint8_t*)"Fault: Input UVLO detected!\r\n", 29, HAL_MAX_DELAY);
    if (fault_status & 0x08) HAL_UART_Transmit(&huart2, (uint8_t*)"Fault: Battery OVP!\r\n", 21, HAL_MAX_DELAY);
    if (fault_status & 0x04) HAL_UART_Transmit(&huart2, (uint8_t*)"Fault: Battery Short!\r\n", 23, HAL_MAX_DELAY);
    if (fault_status & 0x02) HAL_UART_Transmit(&huart2, (uint8_t*)"Fault: Battery NTC Fault!\r\n", 26, HAL_MAX_DELAY);
    if (fault_status & 0x40) HAL_UART_Transmit(&huart2, (uint8_t*)"Fault: Thermal Shutdown!\r\n", 26, HAL_MAX_DELAY);
}

void MP2760_PrintAdvancedPathStatus(void)
{
    uint8_t msb = MP2760_ReadRegister(0x17);
    uint8_t lsb = MP2760_ReadRegister(0x18);
    uint16_t reg17 = (msb << 8) | lsb;

    char msg[256];
    snprintf(msg, sizeof(msg), "Advanced Path Status REG17h: 0x%04X\r\n", reg17);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    if (reg17 & (1 << 15)) HAL_UART_Transmit(&huart2, (uint8_t*)"Bit 15: Reserved (set)\r\n", 24, HAL_MAX_DELAY);
        if (reg17 & (1 << 14)) HAL_UART_Transmit(&huart2, (uint8_t*)"Bit 14: SYS LDO Fault\r\n", 23, HAL_MAX_DELAY);
        if (reg17 & (1 << 13)) HAL_UART_Transmit(&huart2, (uint8_t*)"Bit 13: ACFET Fault\r\n", 21, HAL_MAX_DELAY);
        if (reg17 & (1 << 12)) HAL_UART_Transmit(&huart2, (uint8_t*)"Bit 12: BATFET Fault\r\n", 23, HAL_MAX_DELAY);
        if (reg17 & (1 << 11)) HAL_UART_Transmit(&huart2, (uint8_t*)"Bit 11: Reverse Blocking Enabled\r\n", 34, HAL_MAX_DELAY);
        if (reg17 & (1 << 10)) HAL_UART_Transmit(&huart2, (uint8_t*)"Bit 10: SYS Short Fault\r\n", 26, HAL_MAX_DELAY);
        if (reg17 & (1 << 9))  HAL_UART_Transmit(&huart2, (uint8_t*)"Bit  9: SYS LDO Path Active\r\n", 29, HAL_MAX_DELAY);
        if (reg17 & (1 << 8))  HAL_UART_Transmit(&huart2, (uint8_t*)"Bit  8: VBUS Source Path Active (SRC_EN)\r\n", 42, HAL_MAX_DELAY);
        if (reg17 & (1 << 7))  HAL_UART_Transmit(&huart2, (uint8_t*)"Bit  7: Reserved\r\n", 19, HAL_MAX_DELAY);
        if (reg17 & (1 << 6))  HAL_UART_Transmit(&huart2, (uint8_t*)"Bit  6: Reserved\r\n", 19, HAL_MAX_DELAY);
        if (reg17 & (1 << 5))  HAL_UART_Transmit(&huart2, (uint8_t*)"Bit  5: NTC Fault (Temp Out of Range)\r\n", 39, HAL_MAX_DELAY);
        if (reg17 & (1 << 4))  HAL_UART_Transmit(&huart2, (uint8_t*)"Bit  4: Battery Insert Detected\r\n", 33, HAL_MAX_DELAY);
        if (reg17 & (1 << 3))  HAL_UART_Transmit(&huart2, (uint8_t*)"Bit  3: Charging Terminated\r\n", 30, HAL_MAX_DELAY);
        if (reg17 & (1 << 2))  HAL_UART_Transmit(&huart2, (uint8_t*)"Bit  2: Watchdog Timeout\r\n", 26, HAL_MAX_DELAY);
        if (reg17 & (1 << 1))  HAL_UART_Transmit(&huart2, (uint8_t*)"Bit  1: VBUS Present\r\n", 23, HAL_MAX_DELAY);
        if (reg17 & (1 << 0))  HAL_UART_Transmit(&huart2, (uint8_t*)"Bit  0: CHG_EN Pin State = HIGH\r\n", 33, HAL_MAX_DELAY);
}

void MP2760_Print_REG05(void)
{
    char msg[128];
    uint16_t val = MP2760_ReadRegister16(0x05);

    snprintf(msg, sizeof(msg), "\r\nREG05 (0x05) - Device Address Settings = 0x%04X\r\n", val);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // WD_SET [9]
    snprintf(msg, sizeof(msg), "  WD_SET [9] = %d // %s\r\n",
             (val >> 9) & 1, ((val >> 9) & 1) ? "Watchdog Enabled" : "Watchdog Disabled");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // TLOW_EN [8]
    snprintf(msg, sizeof(msg), "  TLOW_EN [8] = %d // %s\r\n",
             (val >> 8) & 1, ((val >> 8) & 1) ? "TLOW Enabled (Alert on INT)" : "TLOW Disabled (I2C only)");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // ADDR[3:0] [6:3]
    uint8_t addr_bits = (val >> 3) & 0x0F;
    snprintf(msg, sizeof(msg), "  ADDR[3:0] [6:3] = 0x%X // I2C Device Address Offset\r\n", addr_bits);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    char footer[] = "*******************************************************\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)footer, strlen(footer), HAL_MAX_DELAY);
}

void MP2760_Print_REG23(void)
{
    char msg[128];
    uint16_t val = MP2760_ReadRegister16(0x23);

    snprintf(msg, sizeof(msg), "\r\nREG23 (0x23) - ADC Result of the Input Voltage = 0x%04X\r\n", val);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // VIN_ADC[9:0] [9:0] — Input voltage, 20mV/LSB (typical)
    uint16_t vin_adc = val & 0x03FF;
    float vin_mv = (vin_adc * 20.0f)/1000;

    snprintf(msg, sizeof(msg), "  VIN_ADC[9:0] [9:0] = %d // %.2f V input voltage\r\n",
             vin_adc, vin_mv);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    char footer[] = "*******************************************************\r\n";
     HAL_UART_Transmit(&huart2, (uint8_t*)footer, strlen(footer), HAL_MAX_DELAY);
}

void MP2760_Print_REG26(void)
{
    char msg[128];
    uint16_t val = MP2760_ReadRegister16(0x26);

    snprintf(msg, sizeof(msg), "\r\nREG26 (0x26) - ADC Result of System Voltage = 0x%04X\r\n", val);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // VSYS_ADC[9:0] — System voltage, 20 mV/LSB (typical)
    uint16_t vsys_adc = val & 0x03FF;
    float vsys_mv = (vsys_adc * 20.0f)/1000;

    snprintf(msg, sizeof(msg), "  VSYS_ADC[9:0] [9:0] = %d // %.2f V system voltage\r\n",
             vsys_adc, vsys_mv);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    char footer[] = "*******************************************************\r\n";
         HAL_UART_Transmit(&huart2, (uint8_t*)footer, strlen(footer), HAL_MAX_DELAY);
}

void MP2760_Print_REG16(void)
{
    char msg[128];
    uint16_t val = MP2760_ReadRegister16(0x16);
    snprintf(msg, sizeof(msg), "\r\nREG16 (0x16) - Status and Fault Register 0 = 0x%04X\r\n", val);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // MD_STAT[1:0] (15:14)
    uint8_t md_stat = (val >> 14) & 0x03;
    const char* md_modes[] = {"Standby", "Operation", "Standby", "Operation"};
    snprintf(msg, sizeof(msg), "  MD_STAT[15:14] = %d // %s\r\n", md_stat, md_modes[md_stat]);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // PG_STAT [13]
    snprintf(msg, sizeof(msg), "  PG_STAT[13] = %d // %s\r\n", (val >> 13) & 0x01,
             (val & (1 << 13)) ? "VIN power good" : "VIN not power good");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // SWITCH_STAT[12:11]
    uint8_t sw_mode = (val >> 11) & 0x03;
    const char* sw_modes[] = {"Idle", "Buck", "Buck-Boost", "Boost"};
    snprintf(msg, sizeof(msg), "  SWITCH_STAT[12:11] = %d // %s mode\r\n", sw_mode, sw_modes[sw_mode]);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // BATT_MISS_STAT [10]
    snprintf(msg, sizeof(msg), "  BATT_MISS_STAT[10] = %d // %s\r\n", (val >> 10) & 0x01,
             (val & (1 << 10)) ? "Battery missing" : "Battery present");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // CHG_STAT[8:6]
    uint8_t chg_stat = (val >> 6) & 0x07;
    const char* chg_phases[] = {
        "No charging", "Trickle", "Pre-charge", "CC", "CV", "Termination", "Reserved", "Reserved"
    };
    snprintf(msg, sizeof(msg), "  CHG_STAT[8:6] = %d // %s\r\n", chg_stat, chg_phases[chg_stat]);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // VIN_MIN_STAT [5]
    snprintf(msg, sizeof(msg), "  VIN_MIN_STAT[5] = %d // %s\r\n", (val >> 5) & 0x01,
             (val & (1 << 5)) ? "VIN limit loop active" : "VIN OK");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // IIN_LIM_STAT [4]
    snprintf(msg, sizeof(msg), "  IIN_LIM_STAT[4] = %d // %s\r\n", (val >> 4) & 0x01,
             (val & (1 << 4)) ? "Input current limited" : "IIN OK");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // BATFET_OC [2]
    snprintf(msg, sizeof(msg), "  BATFET_OC[2] = %d // %s\r\n", (val >> 2) & 0x01,
             (val & (1 << 2)) ? "BATFET Overcurrent" : "Normal");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // TS_FAULT [1]
    snprintf(msg, sizeof(msg), "  TS_FAULT[1] = %d // %s\r\n", (val >> 1) & 0x01,
             (val & (1 << 1)) ? "TS fault (hot condition)" : "TS normal");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    char footer[] = "*******************************************************\r\n";
                 HAL_UART_Transmit(&huart2, (uint8_t*)footer, strlen(footer), HAL_MAX_DELAY);
}
void MP2760_Print_REG17(void)
{
    char msg[128];
    uint16_t val = MP2760_ReadRegister16(0x17);
    snprintf(msg, sizeof(msg), "\r\nREG17 (0x17) - Status and Fault Register 1 = 0x%04X\r\n", val);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "  VIN_SRC_OV [15] = %d\r\n", (val >> 15) & 0x01);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "  VIN_SRC_UV [14] = %d\r\n", (val >> 14) & 0x01);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "  VIN_CHG_OV [13] = %d\r\n", (val >> 13) & 0x01);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "  VADP_OV [12] = %d\r\n", (val >> 12) & 0x01);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "  VSYS_OV [11] = %d\r\n", (val >> 11) & 0x01);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "  VSYS_UV [10] = %d\r\n", (val >> 10) & 0x01);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "  VBATT_OV [8] = %d\r\n", (val >> 8) & 0x01);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "  VBATT_LOW [7] = %d\r\n", (val >> 7) & 0x01);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "  WTD_EXP [6] = %d // %s\r\n", (val >> 6) & 0x01,
             (val & (1 << 6)) ? "Watchdog expired" : "OK");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "  CHG_TMR_EXP [5] = %d\r\n", (val >> 5) & 0x01);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "  THERM_SHDN [4] = %d\r\n", (val >> 4) & 0x01);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // NTC_FAULT[2:0]
    uint8_t ntc = val & 0x07;
    const char* ntc_states[] = {
        "Normal", "Cold", "Cool", "Warm",
        "Hot", "Reserved", "Reserved", "Float"
    };
    snprintf(msg, sizeof(msg), "  NTC_FAULT[2:0] = %d // %s\r\n", ntc, ntc_states[ntc]);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    char footer[] = "*******************************************************\r\n";
                 HAL_UART_Transmit(&huart2, (uint8_t*)footer, strlen(footer), HAL_MAX_DELAY);
}

void MP2760_Dashboard(void)
{
	const char* clear_screen = "\033[2J\033[H";  // ANSI: clear + move cursor to home
	    HAL_UART_Transmit(&huart2, (uint8_t*)clear_screen, strlen(clear_screen), HAL_MAX_DELAY);

    char header[] = "\r\n================ MP2760 DASHBOARD ==================\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)header, strlen(header), HAL_MAX_DELAY);

   // MP2760_PrintStatus();
    //MP2760_PrintVoltages();
 //   MP2760_PrintThresholds();
  //  MP2760_PrintFaults();
  //  MP2760_PrintAdvancedPathStatus();
 //  MP2760_PrintAllRegisters();
    MP2760_Print_REG05();
    MP2760_Print_REG16();
    MP2760_Print_REG17();
    MP2760_Print_REG23();
    MP2760_Print_REG26();

    char footer[] = "====================================================\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)footer, strlen(footer), HAL_MAX_DELAY);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  char hello[] = "Starting MP2760 Live Monitor...\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)hello, strlen(hello), HAL_MAX_DELAY);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    MP2760_Dashboard();
	    HAL_Delay(1000); // Every 1 second
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000608;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
