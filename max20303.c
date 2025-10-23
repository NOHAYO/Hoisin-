/**
 * @file max20303.c
 * @brief MAX20303 PMIC Driver Implementation
 */

#include "max20303.h"
#include <string.h>

/* I2C Timeout */
#define I2C_TIMEOUT_MS 100

/* Private Functions */
static uint8_t voltage_to_ldo_code(uint16_t voltage_mv);

/**
 * @brief Initialize MAX20303 PMIC
 */
HAL_StatusTypeDef MAX20303_Init(MAX20303_Handle_t *hpmic)
{
    HAL_StatusTypeDef status;
    uint8_t device_id;
    
    if (hpmic == NULL || hpmic->hi2c == NULL) {
        return HAL_ERROR;
    }
    
    /* Test I2C communication */
    status = HAL_I2C_IsDeviceReady(hpmic->hi2c, hpmic->device_addr, 3, I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        return status;
    }
    
    /* Read device ID */
    status = MAX20303_ReadDeviceID(hpmic, &device_id);
    if (status != HAL_OK) {
        return status;
    }
    
    if (device_id != MAX20303_DEVICE_ID) {
        return HAL_ERROR;  // Wrong device
    }
    
    hpmic->device_id = device_id;
    
    /* Read firmware revision */
    status = MAX20303_ReadRegister(hpmic, MAX20303_REG_FIRMWARE_REV, &hpmic->firmware_rev);
    if (status != HAL_OK) {
        return status;
    }
    
    /* Clear any pending interrupts */
    status = MAX20303_ClearInterrupts(hpmic);
    if (status != HAL_OK) {
        return status;
    }
    
    hpmic->initialized = true;
    
    return HAL_OK;
}

/**
 * @brief Read device ID
 */
HAL_StatusTypeDef MAX20303_ReadDeviceID(MAX20303_Handle_t *hpmic, uint8_t *id)
{
    if (hpmic == NULL || id == NULL) {
        return HAL_ERROR;
    }
    
    return MAX20303_ReadRegister(hpmic, MAX20303_REG_HARDWARE_ID, id);
}

/**
 * @brief Enable LDO with specified voltage
 */
HAL_StatusTypeDef MAX20303_EnableLDO(MAX20303_Handle_t *hpmic, 
                                     PMIC_PowerRail_t ldo, 
                                     uint16_t voltage_mv)
{
    HAL_StatusTypeDef status;
    uint8_t ldo_code;
    uint8_t reg_val;
    
    if (hpmic == NULL || !hpmic->initialized) {
        return HAL_ERROR;
    }
    
    /* Convert voltage to LDO code */
    ldo_code = voltage_to_ldo_code(voltage_mv);
    
    /* Configure LDO voltage and enable */
    switch (ldo) {
        case PMIC_LDO1:
            /* Write to AP_CMDOUT register with LDO1 command */
            reg_val = 0x01;  // LDO1 command
            status = MAX20303_WriteRegister(hpmic, MAX20303_REG_AP_CMDOUT, reg_val);
            if (status != HAL_OK) return status;
            
            /* Write voltage setting to AP_DATOUT0 */
            reg_val = ldo_code | 0x80;  // Enable bit
            status = MAX20303_WriteRegister(hpmic, MAX20303_REG_AP_DATOUT0, reg_val);
            break;
            
        case PMIC_LDO2:
            reg_val = 0x02;  // LDO2 command
            status = MAX20303_WriteRegister(hpmic, MAX20303_REG_AP_CMDOUT, reg_val);
            if (status != HAL_OK) return status;
            
            reg_val = ldo_code | 0x80;
            status = MAX20303_WriteRegister(hpmic, MAX20303_REG_AP_DATOUT0, reg_val);
            break;
            
        case PMIC_LDO3:
            reg_val = 0x03;  // LDO3 command
            status = MAX20303_WriteRegister(hpmic, MAX20303_REG_AP_CMDOUT, reg_val);
            if (status != HAL_OK) return status;
            
            reg_val = ldo_code | 0x80;
            status = MAX20303_WriteRegister(hpmic, MAX20303_REG_AP_DATOUT0, reg_val);
            break;
            
        default:
            return HAL_ERROR;
    }
    
    /* Wait for LDO to stabilize */
    HAL_Delay(10);
    
    return status;
}

/**
 * @brief Disable LDO
 */
HAL_StatusTypeDef MAX20303_DisableLDO(MAX20303_Handle_t *hpmic, PMIC_PowerRail_t ldo)
{
    HAL_StatusTypeDef status;
    uint8_t reg_val;
    
    if (hpmic == NULL || !hpmic->initialized) {
        return HAL_ERROR;
    }
    
    switch (ldo) {
        case PMIC_LDO1:
            reg_val = 0x01;
            status = MAX20303_WriteRegister(hpmic, MAX20303_REG_AP_CMDOUT, reg_val);
            if (status != HAL_OK) return status;
            
            reg_val = 0x00;  // Disable
            status = MAX20303_WriteRegister(hpmic, MAX20303_REG_AP_DATOUT0, reg_val);
            break;
            
        case PMIC_LDO2:
            reg_val = 0x02;
            status = MAX20303_WriteRegister(hpmic, MAX20303_REG_AP_CMDOUT, reg_val);
            if (status != HAL_OK) return status;
            
            reg_val = 0x00;
            status = MAX20303_WriteRegister(hpmic, MAX20303_REG_AP_DATOUT0, reg_val);
            break;
            
        case PMIC_LDO3:
            reg_val = 0x03;
            status = MAX20303_WriteRegister(hpmic, MAX20303_REG_AP_CMDOUT, reg_val);
            if (status != HAL_OK) return status;
            
            reg_val = 0x00;
            status = MAX20303_WriteRegister(hpmic, MAX20303_REG_AP_DATOUT0, reg_val);
            break;
            
        default:
            return HAL_ERROR;
    }
    
    return status;
}

/**
 * @brief Configure Buck-Boost converter
 */
HAL_StatusTypeDef MAX20303_ConfigureBuckBoost(MAX20303_Handle_t *hpmic, 
                                               uint16_t voltage_mv, 
                                               bool enable)
{
    HAL_StatusTypeDef status;
    uint8_t vset_code;
    uint8_t cfg_val;
    
    if (hpmic == NULL || !hpmic->initialized) {
        return HAL_ERROR;
    }
    
    /* Calculate voltage setting (example: 2.5V to 5.0V in 50mV steps) */
    if (voltage_mv < 2500) voltage_mv = 2500;
    if (voltage_mv > 5000) voltage_mv = 5000;
    
    vset_code = (voltage_mv - 2500) / 50;
    
    /* Write voltage setting */
    status = MAX20303_WriteRegister(hpmic, MAX20303_REG_BOOST_VSET, vset_code);
    if (status != HAL_OK) return status;
    
    /* Configure and enable/disable */
    cfg_val = enable ? 0x01 : 0x00;
    status = MAX20303_WriteRegister(hpmic, MAX20303_REG_BOOST_CFG, cfg_val);
    
    if (enable) {
        HAL_Delay(10);  // Wait for boost to stabilize
    }
    
    return status;
}

/**
 * @brief Read ADC channel
 */
HAL_StatusTypeDef MAX20303_ReadADC(MAX20303_Handle_t *hpmic, 
                                   ADC_Channel_t channel, 
                                   uint16_t *value_mv)
{
    HAL_StatusTypeDef status;
    uint8_t adc_ctrl;
    uint8_t adc_data[2];
    uint16_t adc_raw;
    
    if (hpmic == NULL || !hpmic->initialized || value_mv == NULL) {
        return HAL_ERROR;
    }
    
    /* Configure ADC channel */
    adc_ctrl = (channel & 0x03) | 0x80;  // Start conversion
    status = MAX20303_WriteRegister(hpmic, MAX20303_REG_ADC_CONTROL, adc_ctrl);
    if (status != HAL_OK) return status;
    
    /* Wait for conversion (typical 1ms) */
    HAL_Delay(2);
    
    /* Read ADC result */
    status = MAX20303_ReadRegisters(hpmic, MAX20303_REG_ADC_DATA_H, adc_data, 2);
    if (status != HAL_OK) return status;
    
    /* Combine 12-bit result */
    adc_raw = ((uint16_t)adc_data[0] << 4) | ((adc_data[1] & 0xF0) >> 4);
    
    /* Convert to mV (assuming 0-5000mV range, adjust based on datasheet) */
    *value_mv = (adc_raw * 5000) / 4096;
    
    return HAL_OK;
}

/**
 * @brief Read battery voltage
 */
HAL_StatusTypeDef MAX20303_ReadBatteryVoltage(MAX20303_Handle_t *hpmic, 
                                               uint16_t *voltage_mv)
{
    return MAX20303_ReadADC(hpmic, ADC_CHANNEL_VBAT, voltage_mv);
}

/**
 * @brief Read battery current (placeholder implementation)
 */
HAL_StatusTypeDef MAX20303_ReadBatteryCurrent(MAX20303_Handle_t *hpmic, 
                                               int16_t *current_ma)
{
    /* Implementation depends on external current sense circuit */
    /* This is a placeholder - implement based on your hardware */
    if (hpmic == NULL || !hpmic->initialized || current_ma == NULL) {
        return HAL_ERROR;
    }
    
    *current_ma = 0;  // Placeholder
    return HAL_OK;
}

/**
 * @brief Set battery charge current
 */
HAL_StatusTypeDef MAX20303_SetChargeCurrent(MAX20303_Handle_t *hpmic, 
                                            uint16_t current_ma)
{
    /* Implementation depends on charger configuration */
    /* This is a placeholder - implement based on datasheet */
    if (hpmic == NULL || !hpmic->initialized) {
        return HAL_ERROR;
    }
    
    /* Configure charge current register */
    /* TODO: Implement based on MAX20303 datasheet */
    
    return HAL_OK;
}

/**
 * @brief Read temperature
 */
HAL_StatusTypeDef MAX20303_ReadTemperature(MAX20303_Handle_t *hpmic, float *temp_c)
{
    HAL_StatusTypeDef status;
    uint16_t thm_mv;
    
    if (temp_c == NULL) {
        return HAL_ERROR;
    }
    
    /* Read thermistor ADC channel */
    status = MAX20303_ReadADC(hpmic, ADC_CHANNEL_THM, &thm_mv);
    if (status != HAL_OK) return status;
    
    /* Convert to temperature (depends on thermistor characteristics) */
    /* This is a simplified placeholder - implement proper NTC conversion */
    *temp_c = 25.0f;  // Placeholder
    
    return HAL_OK;
}

/**
 * @brief Read interrupt status
 */
HAL_StatusTypeDef MAX20303_ReadInterruptStatus(MAX20303_Handle_t *hpmic, 
                                                uint8_t *int_status)
{
    if (hpmic == NULL || !hpmic->initialized || int_status == NULL) {
        return HAL_ERROR;
    }
    
    return MAX20303_ReadRegister(hpmic, MAX20303_REG_INT0, int_status);
}

/**
 * @brief Clear interrupts
 */
HAL_StatusTypeDef MAX20303_ClearInterrupts(MAX20303_Handle_t *hpmic)
{
    HAL_StatusTypeDef status;
    uint8_t dummy;
    
    if (hpmic == NULL) {
        return HAL_ERROR;
    }
    
    /* Reading interrupt registers clears them */
    status = MAX20303_ReadRegister(hpmic, MAX20303_REG_INT0, &dummy);
    if (status != HAL_OK) return status;
    
    status = MAX20303_ReadRegister(hpmic, MAX20303_REG_INT1, &dummy);
    if (status != HAL_OK) return status;
    
    status = MAX20303_ReadRegister(hpmic, MAX20303_REG_INT2, &dummy);
    
    return status;
}

/**
 * @brief Read single register
 */
HAL_StatusTypeDef MAX20303_ReadRegister(MAX20303_Handle_t *hpmic, 
                                        uint8_t reg_addr, 
                                        uint8_t *data)
{
    if (hpmic == NULL || hpmic->hi2c == NULL || data == NULL) {
        return HAL_ERROR;
    }
    
    return HAL_I2C_Mem_Read(hpmic->hi2c, hpmic->device_addr, reg_addr, 
                           I2C_MEMADD_SIZE_8BIT, data, 1, I2C_TIMEOUT_MS);
}

/**
 * @brief Write single register
 */
HAL_StatusTypeDef MAX20303_WriteRegister(MAX20303_Handle_t *hpmic, 
                                         uint8_t reg_addr, 
                                         uint8_t data)
{
    if (hpmic == NULL || hpmic->hi2c == NULL) {
        return HAL_ERROR;
    }
    
    return HAL_I2C_Mem_Write(hpmic->hi2c, hpmic->device_addr, reg_addr, 
                            I2C_MEMADD_SIZE_8BIT, &data, 1, I2C_TIMEOUT_MS);
}

/**
 * @brief Read multiple registers
 */
HAL_StatusTypeDef MAX20303_ReadRegisters(MAX20303_Handle_t *hpmic, 
                                         uint8_t reg_addr, 
                                         uint8_t *data, 
                                         uint16_t length)
{
    if (hpmic == NULL || hpmic->hi2c == NULL || data == NULL || length == 0) {
        return HAL_ERROR;
    }
    
    return HAL_I2C_Mem_Read(hpmic->hi2c, hpmic->device_addr, reg_addr, 
                           I2C_MEMADD_SIZE_8BIT, data, length, I2C_TIMEOUT_MS);
}

/**
 * @brief Convert voltage in mV to LDO code
 */
static uint8_t voltage_to_ldo_code(uint16_t voltage_mv)
{
    /* LDO voltage codes (adjust based on datasheet) */
    if (voltage_mv <= 800) return 0x00;
    else if (voltage_mv <= 900) return 0x01;
    else if (voltage_mv <= 1000) return 0x02;
    else if (voltage_mv <= 1100) return 0x03;
    else if (voltage_mv <= 1200) return 0x04;
    else if (voltage_mv <= 1500) return 0x05;
    else if (voltage_mv <= 1800) return 0x06;
    else if (voltage_mv <= 2500) return 0x07;
    else if (voltage_mv <= 2800) return 0x08;
    else if (voltage_mv <= 3000) return 0x09;
    else return 0x0A;  // 3.3V
}
