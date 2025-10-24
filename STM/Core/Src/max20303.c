/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    max20303.c
  * @brief   MAX20303 PMIC Driver Implementation
  ******************************************************************************
  */
/* USER CODE END Header */

#include "max20303.h"

/**
  * @brief  Read a register from MAX20303
  * @param  reg: Register address
  * @param  data: Pointer to store read data
  * @retval HAL status
  */
HAL_StatusTypeDef MAX20303_ReadRegister(uint8_t reg, uint8_t *data)
{
    return HAL_I2C_Mem_Read(&hi2c1, (MAX20303_I2C_ADDR << 1), reg, 
                            I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

/**
  * @brief  Write a register to MAX20303
  * @param  reg: Register address
  * @param  data: Data to write
  * @retval HAL status
  */
HAL_StatusTypeDef MAX20303_WriteRegister(uint8_t reg, uint8_t data)
{
    return HAL_I2C_Mem_Write(&hi2c1, (MAX20303_I2C_ADDR << 1), reg, 
                             I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

/**
  * @brief  Read multiple registers from MAX20303
  * @param  reg: Starting register address
  * @param  data: Buffer to store data
  * @param  length: Number of bytes to read
  * @retval HAL status
  */
HAL_StatusTypeDef MAX20303_ReadMultipleRegisters(uint8_t reg, uint8_t *data, uint16_t length)
{
    return HAL_I2C_Mem_Read(&hi2c1, (MAX20303_I2C_ADDR << 1), reg, 
                            I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

/**
  * @brief  Get Hardware ID
  * @retval Hardware ID value
  */
uint8_t MAX20303_GetHardwareID(void)
{
    uint8_t hw_id = 0;
    MAX20303_ReadRegister(MAX20303_REG_HARDWARE_ID, &hw_id);
    return hw_id;
}

/**
  * @brief  Set LDO output voltage
  * @param  ldo_num: LDO number (1 or 2)
  * @param  voltage_mv: Voltage in millivolts (800-3600 mV)
  * @retval HAL status
  */
HAL_StatusTypeDef MAX20303_SetLDOVoltage(uint8_t ldo_num, uint16_t voltage_mv)
{
    uint8_t reg_addr;
    uint8_t voltage_code;
    
    // Validate voltage range
    if (voltage_mv < MAX20303_LDO_VOLTAGE_MIN || voltage_mv > MAX20303_LDO_VOLTAGE_MAX) {
        return HAL_ERROR;
    }
    
    // Calculate voltage code
    voltage_code = (voltage_mv - MAX20303_LDO_VOLTAGE_MIN) / MAX20303_LDO_VOLTAGE_STEP;
    
    // Select register based on LDO number
    if (ldo_num == 1) {
        reg_addr = MAX20303_REG_LDO1_DIRECT;
    } else if (ldo_num == 2) {
        reg_addr = MAX20303_REG_LDO2_DIRECT;
    } else {
        return HAL_ERROR;
    }
    
    return MAX20303_WriteRegister(reg_addr, voltage_code);
}

/**
  * @brief  Enable/Disable LDO
  * @param  ldo_num: LDO number (1 or 2)
  * @param  enable: 1 to enable, 0 to disable
  * @retval HAL status
  */
HAL_StatusTypeDef MAX20303_EnableLDO(uint8_t ldo_num, uint8_t enable)
{
    uint8_t reg_addr;
    uint8_t reg_value;
    HAL_StatusTypeDef status;
    
    // Select register based on LDO number
    if (ldo_num == 1) {
        reg_addr = MAX20303_REG_LDO1_DIRECT;
    } else if (ldo_num == 2) {
        reg_addr = MAX20303_REG_LDO2_DIRECT;
    } else {
        return HAL_ERROR;
    }
    
    // Read current value
    status = MAX20303_ReadRegister(reg_addr, &reg_value);
    if (status != HAL_OK) return status;
    
    // Set or clear enable bit (bit 7)
    if (enable) {
        reg_value |= (1 << 7);
    } else {
        reg_value &= ~(1 << 7);
    }
    
    return MAX20303_WriteRegister(reg_addr, reg_value);
}

/**
  * @brief  Get status registers
  * @param  status0: Pointer to store STATUS0 register
  * @param  status1: Pointer to store STATUS1 register
  * @param  status2: Pointer to store STATUS2 register
  * @retval HAL status
  */
HAL_StatusTypeDef MAX20303_GetStatus(uint8_t *status0, uint8_t *status1, uint8_t *status2)
{
    uint8_t buffer[3];
    HAL_StatusTypeDef status;
    
    status = MAX20303_ReadMultipleRegisters(MAX20303_REG_STATUS0, buffer, 3);
    if (status == HAL_OK) {
        *status0 = buffer[0];
        *status1 = buffer[1];
        *status2 = buffer[2];
    }
    
    return status;
}

/**
  * @brief  Set charger current
  * @param  current_ma: Charging current in mA (100-300 mA)
  * @retval HAL status
  */
HAL_StatusTypeDef MAX20303_SetChargerCurrent(uint16_t current_ma)
{
    uint8_t current_code;
    
    // Validate current range
    if (current_ma < MAX20303_CHG_CURRENT_MIN || current_ma > MAX20303_CHG_CURRENT_MAX) {
        return HAL_ERROR;
    }
    
    // Calculate current code
    current_code = (current_ma - MAX20303_CHG_CURRENT_MIN) / MAX20303_CHG_CURRENT_STEP;
    
    return MAX20303_WriteRegister(MAX20303_REG_CHG_I, current_code);
}

/**
  * @brief  Enable/Disable battery charger
  * @param  enable: 1 to enable, 0 to disable
  * @retval HAL status
  */
HAL_StatusTypeDef MAX20303_EnableCharger(uint8_t enable)
{
    uint8_t reg_value;
    HAL_StatusTypeDef status;
    
    status = MAX20303_ReadRegister(MAX20303_REG_CHG_CFG, &reg_value);
    if (status != HAL_OK) return status;
    
    if (enable) {
        reg_value |= (1 << 0);  // Enable charger
    } else {
        reg_value &= ~(1 << 0); // Disable charger
    }
    
    return MAX20303_WriteRegister(MAX20303_REG_CHG_CFG, reg_value);
}

/**
  * @brief  Get charger status
  * @param  charging: Pointer to store charging status (1 = charging, 0 = not charging)
  * @retval HAL status
  */
HAL_StatusTypeDef MAX20303_GetChargerStatus(uint8_t *charging)
{
    uint8_t status1;
    HAL_StatusTypeDef status;
    
    status = MAX20303_ReadRegister(MAX20303_REG_STATUS1, &status1);
    if (status != HAL_OK) return status;
    
    // Check charging status bits
    uint8_t chg_stat = (status1 & 0x03);
    *charging = (chg_stat == 0x01) ? 1 : 0;  // 0x01 = charging
    
    return HAL_OK;
}

/**
  * @brief  Initialize MAX20303 PMIC
  * @retval HAL status
  */
HAL_StatusTypeDef MAX20303_Init(void)
{
    HAL_StatusTypeDef status;
    uint8_t hw_id;
    
    HAL_Delay(10);
    
    // Verify Hardware ID
    hw_id = MAX20303_GetHardwareID();
    if (hw_id != MAX20303_HARDWARE_ID) {
        return HAL_ERROR;  // Wrong device ID
    }
    
    // Configure default settings
    // Set LDO1 to 3.3V and enable
    status = MAX20303_SetLDOVoltage(1, 3300);
    if (status != HAL_OK) return status;
    
    status = MAX20303_EnableLDO(1, 1);
    if (status != HAL_OK) return status;
    
    // Set LDO2 to 1.8V and enable
    status = MAX20303_SetLDOVoltage(2, 1800);
    if (status != HAL_OK) return status;
    
    status = MAX20303_EnableLDO(2, 1);
    if (status != HAL_OK) return status;
    
    // Configure charger with 100mA current
    status = MAX20303_SetChargerCurrent(100);
    if (status != HAL_OK) return status;
    
    // Enable charger
    status = MAX20303_EnableCharger(1);
    
    return status;
}
