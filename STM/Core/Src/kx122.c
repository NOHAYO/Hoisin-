/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    kx122.c
  * @brief   KX122 Accelerometer Driver Implementation
  ******************************************************************************
  */
/* USER CODE END Header */

#include "kx122.h"
#include <math.h>

/* Private variables */
static uint8_t tx_buffer[256];
static uint8_t rx_buffer[256];
static KX122_Range_t current_range = KX122_RANGE_2G;

/**
  * @brief  Read a register from KX122
  * @param  reg: Register address
  * @param  data: Pointer to store read data
  * @retval HAL status
  */
HAL_StatusTypeDef KX122_ReadRegister(uint8_t reg, uint8_t *data)
{
    HAL_StatusTypeDef status;
    
    tx_buffer[0] = reg | 0x80;  // Set MSB for read operation
    tx_buffer[1] = 0x00;        // Dummy byte
    
    KX122_CS_LOW();
    status = HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 2, HAL_MAX_DELAY);
    KX122_CS_HIGH();
    
    if (status == HAL_OK) {
        *data = rx_buffer[1];
    }
    
    return status;
}

/**
  * @brief  Write a register to KX122
  * @param  reg: Register address
  * @param  data: Data to write
  * @retval HAL status
  */
HAL_StatusTypeDef KX122_WriteRegister(uint8_t reg, uint8_t data)
{
    HAL_StatusTypeDef status;
    
    tx_buffer[0] = reg & 0x7F;  // Clear MSB for write operation
    tx_buffer[1] = data;
    
    KX122_CS_LOW();
    status = HAL_SPI_Transmit(&hspi1, tx_buffer, 2, HAL_MAX_DELAY);
    KX122_CS_HIGH();
    
    return status;
}

/**
  * @brief  Read multiple registers from KX122
  * @param  reg: Starting register address
  * @param  data: Buffer to store data
  * @param  length: Number of bytes to read
  * @retval HAL status
  */
HAL_StatusTypeDef KX122_ReadMultipleRegisters(uint8_t reg, uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef status;
    
    tx_buffer[0] = reg | 0x80;  // Set MSB for read operation
    
    KX122_CS_LOW();
    status = HAL_SPI_Transmit(&hspi1, tx_buffer, 1, HAL_MAX_DELAY);
    if (status == HAL_OK) {
        status = HAL_SPI_Receive(&hspi1, data, length, HAL_MAX_DELAY);
    }
    KX122_CS_HIGH();
    
    return status;
}

/**
  * @brief  Get Who Am I value
  * @retval Who Am I value
  */
uint8_t KX122_GetWhoAmI(void)
{
    uint8_t who_am_i = 0;
    KX122_ReadRegister(KX122_REG_WHO_AM_I, &who_am_i);
    return who_am_i;
}

/**
  * @brief  Software reset
  * @retval HAL status
  */
HAL_StatusTypeDef KX122_SoftReset(void)
{
    HAL_StatusTypeDef status;
    
    // Write to CNTL2 register bit 7 to initiate software reset
    status = KX122_WriteRegister(KX122_REG_CNTL2, 0x80);
    HAL_Delay(50);  // Wait for reset to complete
    
    return status;
}

/**
  * @brief  Enable or disable operating mode
  * @param  enable: 1 to enable, 0 to disable
  * @retval HAL status
  */
HAL_StatusTypeDef KX122_EnableOperatingMode(uint8_t enable)
{
    uint8_t cntl1_value;
    HAL_StatusTypeDef status;
    
    status = KX122_ReadRegister(KX122_REG_CNTL1, &cntl1_value);
    if (status != HAL_OK) return status;
    
    if (enable) {
        cntl1_value |= KX122_CNTL1_PC1;
    } else {
        cntl1_value &= ~KX122_CNTL1_PC1;
    }
    
    return KX122_WriteRegister(KX122_REG_CNTL1, cntl1_value);
}

/**
  * @brief  Set accelerometer range
  * @param  range: Range setting (2g, 4g, 8g)
  * @retval HAL status
  */
HAL_StatusTypeDef KX122_SetRange(KX122_Range_t range)
{
    uint8_t cntl1_value;
    HAL_StatusTypeDef status;
    
    // Disable operating mode before changing settings
    status = KX122_EnableOperatingMode(0);
    if (status != HAL_OK) return status;
    
    status = KX122_ReadRegister(KX122_REG_CNTL1, &cntl1_value);
    if (status != HAL_OK) return status;
    
    // Clear range bits and set new range
    cntl1_value &= ~(0x03 << 3);
    cntl1_value |= (range << 3);
    
    status = KX122_WriteRegister(KX122_REG_CNTL1, cntl1_value);
    if (status != HAL_OK) return status;
    
    current_range = range;
    
    // Re-enable operating mode
    return KX122_EnableOperatingMode(1);
}

/**
  * @brief  Set output data rate
  * @param  odr: Output data rate setting
  * @retval HAL status
  */
HAL_StatusTypeDef KX122_SetOutputDataRate(uint8_t odr)
{
    HAL_StatusTypeDef status;
    
    // Disable operating mode before changing settings
    status = KX122_EnableOperatingMode(0);
    if (status != HAL_OK) return status;
    
    status = KX122_WriteRegister(KX122_REG_ODCNTL, odr & 0x0F);
    if (status != HAL_OK) return status;
    
    // Re-enable operating mode
    return KX122_EnableOperatingMode(1);
}

/**
  * @brief  Read accelerometer data
  * @param  accel_data: Pointer to store acceleration data
  * @retval HAL status
  */
HAL_StatusTypeDef KX122_ReadAccelData(KX122_AccelData_t *accel_data)
{
    HAL_StatusTypeDef status;
    uint8_t raw_data[6];
    float sensitivity;
    
    // Read 6 bytes starting from XOUT_L
    status = KX122_ReadMultipleRegisters(KX122_REG_XOUT_L, raw_data, 6);
    if (status != HAL_OK) return status;
    
    // Combine high and low bytes for each axis
    accel_data->x = (int16_t)((raw_data[1] << 8) | raw_data[0]);
    accel_data->y = (int16_t)((raw_data[3] << 8) | raw_data[2]);
    accel_data->z = (int16_t)((raw_data[5] << 8) | raw_data[4]);
    
    // Calculate sensitivity based on range
    switch (current_range) {
        case KX122_RANGE_2G:
            sensitivity = 16384.0f;  // LSB/g for ±2g
            break;
        case KX122_RANGE_4G:
            sensitivity = 8192.0f;   // LSB/g for ±4g
            break;
        case KX122_RANGE_8G:
            sensitivity = 4096.0f;   // LSB/g for ±8g
            break;
        default:
            sensitivity = 16384.0f;
    }
    
    // Convert to g
    accel_data->x_g = (float)accel_data->x / sensitivity;
    accel_data->y_g = (float)accel_data->y / sensitivity;
    accel_data->z_g = (float)accel_data->z / sensitivity;
    
    return HAL_OK;
}

/**
  * @brief  Initialize KX122 accelerometer
  * @retval HAL status
  */
HAL_StatusTypeDef KX122_Init(void)
{
    HAL_StatusTypeDef status;
    uint8_t who_am_i;
    
    // Ensure CS is high initially
    KX122_CS_HIGH();
    HAL_Delay(10);
    
    // Verify Who Am I
    who_am_i = KX122_GetWhoAmI();
    if (who_am_i != KX122_WHO_AM_I_VALUE) {
        return HAL_ERROR;  // Wrong device ID
    }
    
    // Software reset
    status = KX122_SoftReset();
    if (status != HAL_OK) return status;
    
    HAL_Delay(50);
    
    // Disable operating mode
    status = KX122_EnableOperatingMode(0);
    if (status != HAL_OK) return status;
    
    // Configure CNTL1: High resolution, ±2g range
    status = KX122_WriteRegister(KX122_REG_CNTL1, 
                                  KX122_CNTL1_RES | KX122_CNTL1_GSEL_2G);
    if (status != HAL_OK) return status;
    
    // Set output data rate to 100 Hz
    status = KX122_WriteRegister(KX122_REG_ODCNTL, KX122_ODR_100_HZ);
    if (status != HAL_OK) return status;
    
    // Enable operating mode
    status = KX122_EnableOperatingMode(1);
    if (status != HAL_OK) return status;
    
    HAL_Delay(20);  // Wait for first sample
    
    return HAL_OK;
}
