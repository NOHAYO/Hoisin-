/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    max20303.h
  * @brief   MAX20303 PMIC Driver Header
  ******************************************************************************
  * @attention
  *
  * MAX20303 is a Power Management IC with battery charger, fuel gauge, etc.
  * Connected via I2C (I2C1) - should be on PC0 (SCL), PC1 (SDA)
  * Default I2C address: 0x50 (7-bit) or 0xA0 (8-bit write), 0xA1 (8-bit read)
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MAX20303_H
#define __MAX20303_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "i2c.h"

/* MAX20303 I2C Address */
#define MAX20303_I2C_ADDR               0x50  // 7-bit address

/* MAX20303 Register Addresses */
#define MAX20303_REG_HARDWARE_ID        0x00
#define MAX20303_REG_FIRMWARE_REV       0x01
#define MAX20303_REG_INT0               0x02
#define MAX20303_REG_INT1               0x03
#define MAX20303_REG_INT2               0x04
#define MAX20303_REG_STATUS0            0x05
#define MAX20303_REG_STATUS1            0x06
#define MAX20303_REG_STATUS2            0x07
#define MAX20303_REG_STATUS3            0x08
#define MAX20303_REG_SYSTEM_ERROR       0x09
#define MAX20303_REG_INT_MASK0          0x0A
#define MAX20303_REG_INT_MASK1          0x0B
#define MAX20303_REG_INT_MASK2          0x0C
#define MAX20303_REG_AP_DATOUT0         0x0D
#define MAX20303_REG_AP_DATOUT1         0x0E
#define MAX20303_REG_AP_DATOUT2         0x0F
#define MAX20303_REG_AP_DATOUT3         0x10
#define MAX20303_REG_AP_DATOUT4         0x11
#define MAX20303_REG_AP_DATOUT5         0x12
#define MAX20303_REG_AP_DATOUT6         0x13
#define MAX20303_REG_AP_CMDOUT          0x14
#define MAX20303_REG_AP_RESPONSE        0x15
#define MAX20303_REG_AP_DATAIN0         0x16
#define MAX20303_REG_AP_DATAIN1         0x17
#define MAX20303_REG_AP_DATAIN2         0x18
#define MAX20303_REG_AP_DATAIN3         0x19
#define MAX20303_REG_AP_DATAIN4         0x1A
#define MAX20303_REG_AP_DATAIN5         0x1B
#define MAX20303_REG_LDO_DIRECT         0x1C
#define MAX20303_REG_MPC_DIRECTWRITE    0x1D
#define MAX20303_REG_MPC_DIRECTRED      0x1E

/* LDO Control Register */
#define MAX20303_REG_LDO1_DIRECT        0x1F
#define MAX20303_REG_LDO2_DIRECT        0x20

/* Buck-Boost Converter Control */
#define MAX20303_REG_BB_DIRECT          0x21
#define MAX20303_REG_BB_EXTRA           0x22

/* LED Control */
#define MAX20303_REG_LED0_DIRECT        0x23
#define MAX20303_REG_LED1_DIRECT        0x24
#define MAX20303_REG_LED2_DIRECT        0x25
#define MAX20303_REG_LED_STEP_DIRECT    0x26

/* Power Configuration */
#define MAX20303_REG_PWR_CFG            0x27
#define MAX20303_REG_PWR_OFF            0x28

/* Charger Configuration */
#define MAX20303_REG_CHG_CFG            0x29
#define MAX20303_REG_CHG_I              0x2A
#define MAX20303_REG_CHG_V              0x2B

/* Hardware ID */
#define MAX20303_HARDWARE_ID            0x02

/* LDO voltage settings (in mV) */
#define MAX20303_LDO_VOLTAGE_MIN        800
#define MAX20303_LDO_VOLTAGE_MAX        3600
#define MAX20303_LDO_VOLTAGE_STEP       100

/* Charger current settings (in mA) */
#define MAX20303_CHG_CURRENT_MIN        100
#define MAX20303_CHG_CURRENT_MAX        300
#define MAX20303_CHG_CURRENT_STEP       50

/* Status register bits */
#define MAX20303_STATUS0_PWR_OK         (1 << 0)
#define MAX20303_STATUS0_LDO0_OK        (1 << 1)
#define MAX20303_STATUS0_LDO1_OK        (1 << 2)
#define MAX20303_STATUS1_CHG_STAT0      (1 << 0)
#define MAX20303_STATUS1_CHG_STAT1      (1 << 1)

/* Function Prototypes */
HAL_StatusTypeDef MAX20303_Init(void);
HAL_StatusTypeDef MAX20303_ReadRegister(uint8_t reg, uint8_t *data);
HAL_StatusTypeDef MAX20303_WriteRegister(uint8_t reg, uint8_t data);
HAL_StatusTypeDef MAX20303_ReadMultipleRegisters(uint8_t reg, uint8_t *data, uint16_t length);
uint8_t MAX20303_GetHardwareID(void);
HAL_StatusTypeDef MAX20303_SetLDOVoltage(uint8_t ldo_num, uint16_t voltage_mv);
HAL_StatusTypeDef MAX20303_EnableLDO(uint8_t ldo_num, uint8_t enable);
HAL_StatusTypeDef MAX20303_GetStatus(uint8_t *status0, uint8_t *status1, uint8_t *status2);
HAL_StatusTypeDef MAX20303_SetChargerCurrent(uint16_t current_ma);
HAL_StatusTypeDef MAX20303_EnableCharger(uint8_t enable);
HAL_StatusTypeDef MAX20303_GetChargerStatus(uint8_t *charging);

#ifdef __cplusplus
}
#endif

#endif /* __MAX20303_H */
