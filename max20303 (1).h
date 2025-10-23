/**
 * @file max20303.h
 * @brief MAX20303 PMIC Driver for STM32
 * @author Your Name
 * @date 2025
 */

#ifndef MAX20303_H
#define MAX20303_H

#include "stm32xxxx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* I2C Address */
#define MAX20303_I2C_ADDR           (0x50 << 1)  // 7-bit addr shifted for HAL

/* Register Addresses - Key Registers */
#define MAX20303_REG_HARDWARE_ID    0x00
#define MAX20303_REG_FIRMWARE_REV   0x01
#define MAX20303_REG_INT0           0x02
#define MAX20303_REG_INT1           0x03
#define MAX20303_REG_INT2           0x04
#define MAX20303_REG_STATUS0        0x05
#define MAX20303_REG_STATUS1        0x06
#define MAX20303_REG_STATUS2        0x07
#define MAX20303_REG_STATUS3        0x08

/* LDO Configuration Registers */
#define MAX20303_REG_AP_DATOUT0     0x09
#define MAX20303_REG_AP_DATOUT1     0x0A
#define MAX20303_REG_AP_CMDOUT      0x0B

/* LDO Control Registers */
#define MAX20303_REG_LDO_DIRECT     0x13

/* Buck-Boost Registers */
#define MAX20303_REG_BOOST_VSET     0x14
#define MAX20303_REG_BOOST_CFG      0x15

/* Power-Good Threshold Registers */
#define MAX20303_REG_PWR_CFG        0x16

/* ADC Registers */
#define MAX20303_REG_ADC_DATA_H     0x32
#define MAX20303_REG_ADC_DATA_L     0x33
#define MAX20303_REG_ADC_CONTROL    0x34

/* Expected Device ID */
#define MAX20303_DEVICE_ID          0x02

/* Power Rail Definitions */
typedef enum {
    PMIC_LDO1 = 0,
    PMIC_LDO2,
    PMIC_LDO3,
    PMIC_BUCK_BOOST
} PMIC_PowerRail_t;

/* LDO Voltage Settings (in mV) */
typedef enum {
    LDO_VOLTAGE_800MV = 0,
    LDO_VOLTAGE_900MV,
    LDO_VOLTAGE_1000MV,
    LDO_VOLTAGE_1100MV,
    LDO_VOLTAGE_1200MV,
    LDO_VOLTAGE_1500MV,
    LDO_VOLTAGE_1800MV,
    LDO_VOLTAGE_2500MV,
    LDO_VOLTAGE_2800MV,
    LDO_VOLTAGE_3000MV,
    LDO_VOLTAGE_3300MV
} LDO_Voltage_t;

/* ADC Channel Selection */
typedef enum {
    ADC_CHANNEL_VSYS = 0,
    ADC_CHANNEL_VBAT,
    ADC_CHANNEL_VBUS,
    ADC_CHANNEL_THM
} ADC_Channel_t;

/* PMIC Handle Structure */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t device_addr;
    bool initialized;
    uint8_t device_id;
    uint8_t firmware_rev;
} MAX20303_Handle_t;

/* Function Prototypes */

/**
 * @brief Initialize MAX20303 PMIC
 * @param hpmic Pointer to PMIC handle
 * @return HAL_OK on success
 */
HAL_StatusTypeDef MAX20303_Init(MAX20303_Handle_t *hpmic);

/**
 * @brief Read device ID
 * @param hpmic Pointer to PMIC handle
 * @param id Pointer to store device ID
 * @return HAL_OK on success
 */
HAL_StatusTypeDef MAX20303_ReadDeviceID(MAX20303_Handle_t *hpmic, uint8_t *id);

/**
 * @brief Enable LDO with specified voltage
 * @param hpmic Pointer to PMIC handle
 * @param ldo LDO number (LDO1, LDO2, LDO3)
 * @param voltage Voltage in mV
 * @return HAL_OK on success
 */
HAL_StatusTypeDef MAX20303_EnableLDO(MAX20303_Handle_t *hpmic, 
                                     PMIC_PowerRail_t ldo, 
                                     uint16_t voltage_mv);

/**
 * @brief Disable LDO
 * @param hpmic Pointer to PMIC handle
 * @param ldo LDO number
 * @return HAL_OK on success
 */
HAL_StatusTypeDef MAX20303_DisableLDO(MAX20303_Handle_t *hpmic, 
                                      PMIC_PowerRail_t ldo);

/**
 * @brief Configure Buck-Boost converter
 * @param hpmic Pointer to PMIC handle
 * @param voltage_mv Output voltage in mV
 * @param enable Enable/disable buck-boost
 * @return HAL_OK on success
 */
HAL_StatusTypeDef MAX20303_ConfigureBuckBoost(MAX20303_Handle_t *hpmic, 
                                               uint16_t voltage_mv, 
                                               bool enable);

/**
 * @brief Read ADC channel
 * @param hpmic Pointer to PMIC handle
 * @param channel ADC channel to read
 * @param value_mv Pointer to store voltage in mV
 * @return HAL_OK on success
 */
HAL_StatusTypeDef MAX20303_ReadADC(MAX20303_Handle_t *hpmic, 
                                   ADC_Channel_t channel, 
                                   uint16_t *value_mv);

/**
 * @brief Read battery voltage
 * @param hpmic Pointer to PMIC handle
 * @param voltage_mv Pointer to store voltage in mV
 * @return HAL_OK on success
 */
HAL_StatusTypeDef MAX20303_ReadBatteryVoltage(MAX20303_Handle_t *hpmic, 
                                               uint16_t *voltage_mv);

/**
 * @brief Read battery current
 * @param hpmic Pointer to PMIC handle
 * @param current_ma Pointer to store current in mA
 * @return HAL_OK on success
 */
HAL_StatusTypeDef MAX20303_ReadBatteryCurrent(MAX20303_Handle_t *hpmic, 
                                               int16_t *current_ma);

/**
 * @brief Set battery charge current
 * @param hpmic Pointer to PMIC handle
 * @param current_ma Charge current in mA
 * @return HAL_OK on success
 */
HAL_StatusTypeDef MAX20303_SetChargeCurrent(MAX20303_Handle_t *hpmic, 
                                            uint16_t current_ma);

/**
 * @brief Read temperature
 * @param hpmic Pointer to PMIC handle
 * @param temp_c Pointer to store temperature in Celsius
 * @return HAL_OK on success
 */
HAL_StatusTypeDef MAX20303_ReadTemperature(MAX20303_Handle_t *hpmic, 
                                           float *temp_c);

/**
 * @brief Read interrupt status
 * @param hpmic Pointer to PMIC handle
 * @param int_status Pointer to store interrupt status
 * @return HAL_OK on success
 */
HAL_StatusTypeDef MAX20303_ReadInterruptStatus(MAX20303_Handle_t *hpmic, 
                                                uint8_t *int_status);

/**
 * @brief Clear interrupts
 * @param hpmic Pointer to PMIC handle
 * @return HAL_OK on success
 */
HAL_StatusTypeDef MAX20303_ClearInterrupts(MAX20303_Handle_t *hpmic);

/* Low-level I2C functions */
HAL_StatusTypeDef MAX20303_ReadRegister(MAX20303_Handle_t *hpmic, 
                                        uint8_t reg_addr, 
                                        uint8_t *data);

HAL_StatusTypeDef MAX20303_WriteRegister(MAX20303_Handle_t *hpmic, 
                                         uint8_t reg_addr, 
                                         uint8_t data);

HAL_StatusTypeDef MAX20303_ReadRegisters(MAX20303_Handle_t *hpmic, 
                                         uint8_t reg_addr, 
                                         uint8_t *data, 
                                         uint16_t length);

#endif /* MAX20303_H */
