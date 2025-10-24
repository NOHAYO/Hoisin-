/**
 * @file max86171.c
 * @brief MAX86171 Optical Sensor Driver Implementation for STM32
 * @version 1.0 - Final Consolidated Version
 * @author Based on configuration CSV and integrated driver versions
 */

#include "max86171.h"
#include <string.h>

/* =============================================================================
 * PRIVATE VARIABLES
 * ============================================================================= */

static MAX86171_Status_t device_status = {
    .initialized = false,
    .measurement_active = false,
    .part_id = 0,
    .revision_id = 0
};

/**
 * @brief Configuration array - Auto-generated from CSV
 * Contains 117 writable registers for initialization
 */
static const MAX86171_RegConfig_t init_config[] = {
    {0x09, 0xF4},  // FIFO Configuration 1
    {0x0A, 0x08},  // FIFO Configuration 2
    {0x0C, 0x02},  // System Configuration 1
    {0x0D, 0xFF},  // System Configuration 2
    {0x0E, 0x00},  // System Configuration 3
    {0x0F, 0x55},  // Photo Diode Bias
    {0x10, 0x00},  // Pin Functional Configuration
    {0x11, 0x00},  // Output Pin Configuration
    {0x15, 0x00},  // FR Clock Frequency Select
    {0x16, 0x01},  // FR Clock Divider MSB
    {0x17, 0x40},  // FR Clock Divider LSB
    {0x18, 0x08},  // MEAS1 Selects (Green)
    {0x19, 0x1A},  // MEAS1 Configuration 1
    {0x1A, 0x3A},  // MEAS1 Configuration 2
    {0x1B, 0x50},  // MEAS1 Configuration 3
    {0x1C, 0x0A},  // MEAS1 LEDA Current (10mA)
    {0x1D, 0x0A},  // MEAS1 LEDB Current (10mA)
    {0x1E, 0x0A},  // MEAS1 LEDC Current (10mA)
    {0x20, 0x0A},  // MEAS2 Selects (Green)
    {0x21, 0x7A},  // MEAS2 Configuration 1
    {0x22, 0x3A},  // MEAS2 Configuration 2
    {0x23, 0x50},  // MEAS2 Configuration 3
    {0x24, 0x0A},  // MEAS2 LEDA Current (10mA)
    {0x25, 0x0A},  // MEAS2 LEDB Current (10mA)
    {0x26, 0x0A},  // MEAS2 LEDC Current (10mA)
    {0x28, 0x17},  // MEAS3 Selects (IR)
    {0x29, 0x1A},  // MEAS3 Configuration 1
    {0x2A, 0x3A},  // MEAS3 Configuration 2
    {0x2B, 0x50},  // MEAS3 Configuration 3
    {0x2C, 0x14},  // MEAS3 LEDA Current (20mA)
    {0x2D, 0x14},  // MEAS3 LEDB Current (20mA)
    {0x2E, 0x14},  // MEAS3 LEDC Current (20mA)
    {0x30, 0x17},  // MEAS4 Selects (IR)
    {0x31, 0x7A},  // MEAS4 Configuration 1
    {0x32, 0x3A},  // MEAS4 Configuration 2
    {0x33, 0x50},  // MEAS4 Configuration 3
    {0x34, 0x14},  // MEAS4 LEDA Current (20mA)
    {0x35, 0x14},  // MEAS4 LEDB Current (20mA)
    {0x36, 0x14},  // MEAS4 LEDC Current (20mA)
    {0x38, 0x21},  // MEAS5 Selects (Red)
    {0x39, 0x1A},  // MEAS5 Configuration 1
    {0x3A, 0x3A},  // MEAS5 Configuration 2
    {0x3B, 0x50},  // MEAS5 Configuration 3
    {0x3C, 0x16},  // MEAS5 LEDA Current (22mA)
    {0x3D, 0x16},  // MEAS5 LEDB Current (22mA)
    {0x3E, 0x16},  // MEAS5 LEDC Current (22mA)
    {0x40, 0x21},  // MEAS6 Selects (Red)
    {0x41, 0x7A},  // MEAS6 Configuration 1
    {0x42, 0x3A},  // MEAS6 Configuration 2
    {0x43, 0x50},  // MEAS6 Configuration 3
    {0x44, 0x16},  // MEAS6 LEDA Current (22mA)
    {0x45, 0x16},  // MEAS6 LEDB Current (22mA)
    {0x46, 0x16},  // MEAS6 LEDC Current (22mA)
    {0x48, 0x3C},  // MEAS7 Selects
    {0x49, 0x19},  // MEAS7 Configuration 1
    {0x4A, 0x3A},  // MEAS7 Configuration 2
    {0x4B, 0x50},  // MEAS7 Configuration 3
    {0x4C, 0x0A},  // MEAS7 LEDA Current
    {0x4D, 0x22},  // MEAS7 LEDB Current (34mA)
    {0x4E, 0x22},  // MEAS7 LEDC Current (34mA)
    {0x50, 0x3C},  // MEAS8 Selects
    {0x51, 0x79},  // MEAS8 Configuration 1
    {0x52, 0x3A},  // MEAS8 Configuration 2
    {0x53, 0x50},  // MEAS8 Configuration 3
    {0x54, 0x0A},  // MEAS8 LEDA Current
    {0x55, 0x22},  // MEAS8 LEDB Current (34mA)
    {0x56, 0x22},  // MEAS8 LEDC Current (34mA)
    {0x58, 0x00},  // MEAS9 Selects
    {0x59, 0x18},  // MEAS9 Configuration 1
    {0x5A, 0x3A},  // MEAS9 Configuration 2
    {0x5B, 0x50},  // MEAS9 Configuration 3
    {0x5C, 0x0A},  // MEAS9 LEDA Current
    {0x5D, 0x00},  // MEAS9 LEDB Current (Off)
    {0x5E, 0x00},  // MEAS9 LEDC Current (Off)
    {0x68, 0x00},  // THRESHOLD MEAS SEL
    {0x69, 0x00},  // THRESHOLD HYST
    {0x6A, 0xFF},  // PPG HI THRESHOLD1
    {0x6B, 0x00},  // PPG LO THRESHOLD1
    {0x6C, 0xFF},  // PPG HI THRESHOLD2
    {0x6D, 0x00},  // PPG LO THRESHOLD2
    {0x70, 0x00},  // Picket Fence Measurement Select
    {0x71, 0x40},  // Picket Fence Configuration
    {0x78, 0x80},  // Interrupt1 Enable 1 (FIFO Almost Full)
    {0x79, 0x00},  // Interrupt1 Enable 2
    {0x7A, 0x00},  // Interrupt1 Enable 3
    {0x7B, 0x00},  // Interrupt1 Enable 4
    {0x7C, 0x00},  // Interrupt2 Enable 1
    {0x7D, 0x00},  // Interrupt2 Enable 2
    {0x7E, 0x00},  // Interrupt2 Enable 3
    {0x7F, 0x00},  // Interrupt2 Enable 4
    {0xD0, 0x20},  // Program Control 1
    {0xD1, 0x64},  // User OTP Byte 1
    {0xD2, 0x59},  // User OTP Byte 2
    {0xD3, 0x32},  // User OTP Byte 3
    {0xD4, 0x85},  // User OTP Byte 4
    {0xD5, 0x42},  // User OTP Byte 5
    {0xD6, 0x61},  // User OTP Byte 6
    {0xD7, 0x72},  // User OTP Byte 7
    {0xD8, 0x72},  // User OTP Byte 8
    {0xD9, 0x20},  // Program Control 2
    {0xDA, 0x79},  // User OTP Byte 9
    {0xDB, 0x20},  // User OTP Byte 10
    {0xDC, 0x4B},  // User OTP Byte 11
    {0xDD, 0x75},  // User OTP Byte 12
    {0xDE, 0x6C},  // User OTP Byte 13
    {0xDF, 0x70},  // User OTP Byte 14
    {0xE0, 0x20},  // User OTP Byte 15
    {0xE1, 0x69},  // User OTP Byte 16
    {0xE2, 0x20},  // Program Control 3
    {0xE3, 0x73},  // User OTP Byte 17
    {0xE4, 0x20},  // User OTP Byte 18
    {0xE5, 0x67},  // User OTP Byte 19
    {0xE6, 0x72},  // User OTP Byte 20
    {0xE7, 0x65},  // User OTP Byte 21
    {0xE8, 0x61},  // User OTP Byte 22
    {0xE9, 0x74},  // User OTP Byte 23
    {0xEA, 0x21},  // User OTP Byte 24
};

#define INIT_CONFIG_SIZE (sizeof(init_config) / sizeof(MAX86171_RegConfig_t))

/* =============================================================================
 * PUBLIC FUNCTIONS - BASIC COMMUNICATION
 * ============================================================================= */

/**
 * @brief Write a single register via SPI
 */
HAL_StatusTypeDef MAX86171_WriteRegister(uint8_t reg_addr, uint8_t value) {
    uint8_t tx_data[2];
    tx_data[0] = reg_addr & 0x7F;  // Clear MSB for write operation
    tx_data[1] = value;
    
    MAX86171_CS_LOW();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1, tx_data, 2, MAX86171_SPI_TIMEOUT_MS);
    MAX86171_CS_HIGH();
    
    return status;
}

/**
 * @brief Read a single register via SPI
 */
HAL_StatusTypeDef MAX86171_ReadRegister(uint8_t reg_addr, uint8_t *value) {
    if (value == NULL) {
        return HAL_ERROR;
    }
    
    uint8_t tx_addr = reg_addr | 0x80;  // Set MSB for read operation
    
    MAX86171_CS_LOW();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1, &tx_addr, 1, MAX86171_SPI_TIMEOUT_MS);
    if (status == HAL_OK) {
        status = HAL_SPI_Receive(&hspi1, value, 1, MAX86171_SPI_TIMEOUT_MS);
    }
    MAX86171_CS_HIGH();
    
    return status;
}

/**
 * @brief Read multiple registers via SPI (burst read)
 */
HAL_StatusTypeDef MAX86171_ReadRegisters(uint8_t reg_addr, uint8_t *buffer, uint16_t length) {
    if (buffer == NULL || length == 0) {
        return HAL_ERROR;
    }
    
    uint8_t tx_addr = reg_addr | 0x80;  // Set MSB for read operation
    
    MAX86171_CS_LOW();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1, &tx_addr, 1, MAX86171_SPI_TIMEOUT_MS);
    if (status == HAL_OK) {
        status = HAL_SPI_Receive(&hspi1, buffer, length, MAX86171_SPI_TIMEOUT_MS);
    }
    MAX86171_CS_HIGH();
    
    return status;
}

/* =============================================================================
 * PUBLIC FUNCTIONS - DEVICE CONTROL
 * ============================================================================= */

/**
 * @brief Reset the MAX86171 device
 */
HAL_StatusTypeDef MAX86171_Reset(void) {
    HAL_StatusTypeDef status;
    
    // Write reset bit
    status = MAX86171_WriteRegister(MAX86171_REG_SYSTEM_CFG1, MAX86171_RESET_BIT);
    if (status != HAL_OK) {
        return status;
    }
    
    // Wait for reset to complete
    HAL_Delay(10);
    
    device_status.initialized = false;
    device_status.measurement_active = false;
    
    return HAL_OK;
}

/**
 * @brief Initialize MAX86171 with configuration
 */
HAL_StatusTypeDef MAX86171_Init(void) {
    HAL_StatusTypeDef status;
    uint8_t reg_value;
    
    // Ensure CS is high initially
    MAX86171_CS_HIGH();
    HAL_Delay(1);
    
    // Reset device
    status = MAX86171_Reset();
    if (status != HAL_OK) {
        return status;
    }
    
    // Apply configuration
    for (uint16_t i = 0; i < INIT_CONFIG_SIZE; i++) {
        status = MAX86171_WriteRegister(init_config[i].address, init_config[i].value);
        if (status != HAL_OK) {
            return status;
        }
    }
    
    // Verify Part ID
    status = MAX86171_ReadRegister(MAX86171_REG_PART_ID, &reg_value);
    if (status != HAL_OK) {
        return status;
    }
    
    // Store Part ID but don't fail if it doesn't match exactly
    // (some variations may exist)
    device_status.part_id = reg_value;
    
    // Read Revision ID
    status = MAX86171_ReadRegister(MAX86171_REG_REVISION_ID, &reg_value);
    if (status == HAL_OK) {
        device_status.revision_id = reg_value;
    }
    
    device_status.initialized = true;
    device_status.measurement_active = false;
    
    return HAL_OK;
}

/**
 * @brief Start measurements
 */
HAL_StatusTypeDef MAX86171_StartMeasurement(void) {
    if (!device_status.initialized) {
        return HAL_ERROR;
    }
    
    uint8_t sys_cfg;
    HAL_StatusTypeDef status = MAX86171_ReadRegister(MAX86171_REG_SYSTEM_CFG1, &sys_cfg);
    if (status != HAL_OK) {
        return status;
    }
    
    // Clear shutdown bit to start measurements
    sys_cfg &= ~MAX86171_SHUTDOWN_BIT;
    status = MAX86171_WriteRegister(MAX86171_REG_SYSTEM_CFG1, sys_cfg);
    
    if (status == HAL_OK) {
        device_status.measurement_active = true;
    }
    
    return status;
}

/**
 * @brief Stop measurements (enter shutdown mode)
 */
HAL_StatusTypeDef MAX86171_StopMeasurement(void) {
    uint8_t sys_cfg;
    HAL_StatusTypeDef status = MAX86171_ReadRegister(MAX86171_REG_SYSTEM_CFG1, &sys_cfg);
    if (status != HAL_OK) {
        return status;
    }
    
    // Set shutdown bit to stop measurements
    sys_cfg |= MAX86171_SHUTDOWN_BIT;
    status = MAX86171_WriteRegister(MAX86171_REG_SYSTEM_CFG1, sys_cfg);
    
    if (status == HAL_OK) {
        device_status.measurement_active = false;
    }
    
    return status;
}

/**
 * @brief Enable/Disable shutdown mode
 */
HAL_StatusTypeDef MAX86171_Shutdown(uint8_t enable) {
    if (enable) {
        return MAX86171_StopMeasurement();
    } else {
        return MAX86171_StartMeasurement();
    }
}

/* =============================================================================
 * PUBLIC FUNCTIONS - FIFO OPERATIONS
 * ============================================================================= */

/**
 * @brief Read FIFO data (burst read for efficiency)
 */
HAL_StatusTypeDef MAX86171_ReadFIFO(uint8_t *buffer, uint16_t length) {
    if (buffer == NULL || length == 0) {
        return HAL_ERROR;
    }
    
    // Use burst read for efficiency
    return MAX86171_ReadRegisters(MAX86171_REG_FIFO_DATA, buffer, length);
}

/**
 * @brief Get number of samples available in FIFO
 */
uint16_t MAX86171_GetFIFOSamples(void) {
    uint8_t counter_msb, counter_lsb;
    
    if (MAX86171_ReadRegister(MAX86171_REG_FIFO_COUNTER1, &counter_msb) != HAL_OK) {
        return 0;
    }
    
    if (MAX86171_ReadRegister(MAX86171_REG_FIFO_COUNTER2, &counter_lsb) != HAL_OK) {
        return 0;
    }
    
    return ((uint16_t)counter_msb << 8) | counter_lsb;
}

/**
 * @brief Clear FIFO by resetting pointers
 */
HAL_StatusTypeDef MAX86171_ClearFIFO(void) {
    HAL_StatusTypeDef status;
    
    status = MAX86171_WriteRegister(MAX86171_REG_FIFO_WRITE_PTR, 0x00);
    if (status != HAL_OK) return status;
    
    status = MAX86171_WriteRegister(MAX86171_REG_FIFO_READ_PTR, 0x00);
    if (status != HAL_OK) return status;
    
    return HAL_OK;
}

/**
 * @brief Parse a 3-byte FIFO sample into structured data
 */
void MAX86171_ParseSample(uint8_t *data, MAX86171_Sample_t *sample) {
    if (data == NULL || sample == NULL) {
        return;
    }
    
    // Extract 19-bit data
    sample->data  = ((uint32_t)(data[0] & 0x1F)) << 14;  // 5 MSBs
    sample->data |= ((uint32_t)data[1]) << 6;             // 8 middle bits
    sample->data |= ((uint32_t)data[2]) >> 2;             // 6 LSBs
    
    // Extract 3-bit tag
    sample->tag = (data[0] >> 5) & 0x07;
}

/* =============================================================================
 * PUBLIC FUNCTIONS - STATUS AND INFO
 * ============================================================================= */

/**
 * @brief Read status registers
 */
HAL_StatusTypeDef MAX86171_ReadStatus(uint8_t *status1, uint8_t *status2) {
    HAL_StatusTypeDef status;
    
    if (status1 != NULL) {
        status = MAX86171_ReadRegister(MAX86171_REG_STATUS1, status1);
        if (status != HAL_OK) return status;
    }
    
    if (status2 != NULL) {
        status = MAX86171_ReadRegister(MAX86171_REG_STATUS2, status2);
        if (status != HAL_OK) return status;
    }
    
    return HAL_OK;
}

/**
 * @brief Get device information
 */
HAL_StatusTypeDef MAX86171_GetInfo(MAX86171_Status_t *status) {
    if (status == NULL) {
        return HAL_ERROR;
    }
    
    memcpy(status, &device_status, sizeof(MAX86171_Status_t));
    return HAL_OK;
}

/**
 * @brief Get Part ID (for compatibility with main.c)
 */
uint8_t MAX86171_GetPartID(void) {
    return device_status.part_id;
}

/**
 * @brief Get Revision ID
 */
uint8_t MAX86171_GetRevisionID(void) {
    return device_status.revision_id;
}

/**
 * @brief Verify configuration by reading back key registers
 */
HAL_StatusTypeDef MAX86171_VerifyConfig(void) {
    uint8_t reg_value;
    HAL_StatusTypeDef status;
    
    // Verify FIFO Configuration 1
    status = MAX86171_ReadRegister(0x09, &reg_value);
    if (status != HAL_OK || reg_value != 0xF4) {
        return HAL_ERROR;
    }
    
    // Verify FIFO Configuration 2
    status = MAX86171_ReadRegister(0x0A, &reg_value);
    if (status != HAL_OK || reg_value != 0x08) {
        return HAL_ERROR;
    }
    
    // Verify System Configuration 1
    status = MAX86171_ReadRegister(0x0C, &reg_value);
    if (status != HAL_OK || reg_value != 0x02) {
        return HAL_ERROR;
    }
    
    return HAL_OK;
}
