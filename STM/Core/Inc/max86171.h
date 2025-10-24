/**
 * @file max86171.h
 * @brief MAX86171 Optical Sensor Driver for STM32
 * @version 1.0 - Final Consolidated Version
 * 
 * Hardware: MAX86171 connected via SPI1
 * Platform: STM32L4 with HAL library
 * CS Pin: SPI1_NSS (from main.c GPIO configuration)
 */

#ifndef MAX86171_H
#define MAX86171_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "spi.h"
#include <stdint.h>
#include <stdbool.h>

/* =============================================================================
 * HARDWARE CONFIGURATION
 * ============================================================================= */

// SPI handle - defined in spi.c
extern SPI_HandleTypeDef hspi1;

// CS (Chip Select) Pin Configuration - from main.c GPIO setup
#define MAX86171_CS_GPIO_PORT       SPI1_NSS_GPIO_Port
#define MAX86171_CS_PIN             SPI1_NSS_Pin

// CS Pin Control Macros
#define MAX86171_CS_LOW()           HAL_GPIO_WritePin(MAX86171_CS_GPIO_PORT, MAX86171_CS_PIN, GPIO_PIN_RESET)
#define MAX86171_CS_HIGH()          HAL_GPIO_WritePin(MAX86171_CS_GPIO_PORT, MAX86171_CS_PIN, GPIO_PIN_SET)

/* =============================================================================
 * SPI PARAMETERS
 * ============================================================================= */

#define MAX86171_SPI_TIMEOUT_MS     100
#define MAX86171_SPI_MAX_FREQ_HZ    8000000  // 8 MHz max

/* =============================================================================
 * DEVICE IDs
 * ============================================================================= */

#define MAX86171_PART_ID            0x36     // Expected Part ID
#define MAX86171_REVISION_ID        0x03     // Expected Revision ID

/* =============================================================================
 * REGISTER ADDRESSES
 * ============================================================================= */

// Status and Interrupt Registers
#define MAX86171_REG_STATUS1        0x00
#define MAX86171_REG_STATUS2        0x01
#define MAX86171_REG_INT_ENABLE1    0x02
#define MAX86171_REG_INT_ENABLE2    0x03

// FIFO Registers
#define MAX86171_REG_FIFO_WRITE_PTR 0x04
#define MAX86171_REG_FIFO_READ_PTR  0x05
#define MAX86171_REG_FIFO_COUNTER1  0x06
#define MAX86171_REG_FIFO_COUNTER2  0x07
#define MAX86171_REG_FIFO_DATA      0x08
#define MAX86171_REG_FIFO_CONFIG1   0x09
#define MAX86171_REG_FIFO_CONFIG2   0x0A

// System Configuration Registers
#define MAX86171_REG_SYSTEM_CFG1    0x0C
#define MAX86171_REG_SYSTEM_CFG2    0x0D
#define MAX86171_REG_SYSTEM_CFG3    0x0E

// LED and PPG Configuration Registers
#define MAX86171_REG_PD_BIAS        0x0F
#define MAX86171_REG_PIN_FUNC_CFG   0x10
#define MAX86171_REG_OUTPUT_PIN_CFG 0x11

// Device ID Registers
#define MAX86171_REG_REVISION_ID    0xFE
#define MAX86171_REG_PART_ID        0xFF

/* =============================================================================
 * SYSTEM CONFIGURATION BITS
 * ============================================================================= */

#define MAX86171_RESET_BIT          0x01
#define MAX86171_SHUTDOWN_BIT       0x02
#define MAX86171_LOW_POWER_BIT      0x04

/* =============================================================================
 * STATUS BITS
 * ============================================================================= */

#define MAX86171_STATUS_FIFO_A_FULL 0x80
#define MAX86171_STATUS_PPG_RDY     0x40
#define MAX86171_STATUS_ALC_OVF     0x20
#define MAX86171_STATUS_PROXY_INT   0x10
#define MAX86171_STATUS_LED_COMPLIANT 0x08
#define MAX86171_STATUS_DIE_TEMP_RDY  0x04

/* =============================================================================
 * DATA STRUCTURES
 * ============================================================================= */

/**
 * @brief Register configuration structure
 */
typedef struct {
    uint8_t address;
    uint8_t value;
} MAX86171_RegConfig_t;

/**
 * @brief FIFO sample structure (3 bytes per sample)
 * Format: [TAG(3) | DATA(19)] = 22 bits total in 3 bytes
 */
typedef struct {
    uint32_t data;      // 19-bit PPG data
    uint8_t tag;        // 3-bit measurement tag
} MAX86171_Sample_t;

/**
 * @brief Device status structure
 */
typedef struct {
    bool initialized;
    bool measurement_active;
    uint8_t part_id;
    uint8_t revision_id;
} MAX86171_Status_t;

/* =============================================================================
 * FUNCTION PROTOTYPES - INITIALIZATION & CONTROL
 * ============================================================================= */

/**
 * @brief Initialize MAX86171 with full configuration
 * @retval HAL_OK if successful, HAL_ERROR otherwise
 */
HAL_StatusTypeDef MAX86171_Init(void);

/**
 * @brief Reset the MAX86171 device
 * @retval HAL_OK if successful
 */
HAL_StatusTypeDef MAX86171_Reset(void);

/**
 * @brief Start measurements
 * @retval HAL_OK if successful
 */
HAL_StatusTypeDef MAX86171_StartMeasurement(void);

/**
 * @brief Stop measurements (enter shutdown mode)
 * @retval HAL_OK if successful
 */
HAL_StatusTypeDef MAX86171_StopMeasurement(void);

/**
 * @brief Enable/Disable shutdown mode
 * @param enable 1 to shutdown, 0 to wake up
 * @retval HAL_OK if successful
 */
HAL_StatusTypeDef MAX86171_Shutdown(uint8_t enable);

/* =============================================================================
 * FUNCTION PROTOTYPES - REGISTER ACCESS
 * ============================================================================= */

/**
 * @brief Write a single register
 * @param reg_addr Register address
 * @param value Value to write
 * @retval HAL_OK if successful
 */
HAL_StatusTypeDef MAX86171_WriteRegister(uint8_t reg_addr, uint8_t value);

/**
 * @brief Read a single register
 * @param reg_addr Register address
 * @param value Pointer to store read value
 * @retval HAL_OK if successful
 */
HAL_StatusTypeDef MAX86171_ReadRegister(uint8_t reg_addr, uint8_t *value);

/**
 * @brief Read multiple registers (burst read)
 * @param reg_addr Starting register address
 * @param buffer Buffer to store data
 * @param length Number of bytes to read
 * @retval HAL_OK if successful
 */
HAL_StatusTypeDef MAX86171_ReadRegisters(uint8_t reg_addr, uint8_t *buffer, uint16_t length);

/* =============================================================================
 * FUNCTION PROTOTYPES - FIFO OPERATIONS
 * ============================================================================= */

/**
 * @brief Read FIFO data
 * @param buffer Buffer to store FIFO data
 * @param length Number of bytes to read
 * @retval HAL_OK if successful
 */
HAL_StatusTypeDef MAX86171_ReadFIFO(uint8_t *buffer, uint16_t length);

/**
 * @brief Parse a single 3-byte sample from FIFO data
 * @param data Pointer to 3-byte sample data
 * @param sample Pointer to sample structure to fill
 */
void MAX86171_ParseSample(uint8_t *data, MAX86171_Sample_t *sample);

/**
 * @brief Get number of samples available in FIFO
 * @retval Number of samples (0-8191)
 */
uint16_t MAX86171_GetFIFOSamples(void);

/**
 * @brief Clear FIFO by resetting pointers
 * @retval HAL_OK if successful
 */
HAL_StatusTypeDef MAX86171_ClearFIFO(void);

/* =============================================================================
 * FUNCTION PROTOTYPES - STATUS & INFO
 * ============================================================================= */

/**
 * @brief Read status registers
 * @param status1 Pointer to store STATUS1
 * @param status2 Pointer to store STATUS2
 * @retval HAL_OK if successful
 */
HAL_StatusTypeDef MAX86171_ReadStatus(uint8_t *status1, uint8_t *status2);

/**
 * @brief Get device information
 * @param status Pointer to status structure to fill
 * @retval HAL_OK if successful
 */
HAL_StatusTypeDef MAX86171_GetInfo(MAX86171_Status_t *status);

/**
 * @brief Get Part ID (for verification)
 * @retval Part ID value
 */
uint8_t MAX86171_GetPartID(void);

/**
 * @brief Get Revision ID
 * @retval Revision ID value
 */
uint8_t MAX86171_GetRevisionID(void);

/**
 * @brief Verify configuration by reading back key registers
 * @retval HAL_OK if configuration matches, HAL_ERROR otherwise
 */
HAL_StatusTypeDef MAX86171_VerifyConfig(void);

#ifdef __cplusplus
}
#endif

#endif /* MAX86171_H */
