/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    kx122.h
  * @brief   KX122 Accelerometer Driver Header
  ******************************************************************************
  * @attention
  *
  * KX122 is a 3-axis accelerometer with integrated motion detection
  * Connected via SPI (SPI1) with CS on PA3
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __KX122_H
#define __KX122_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "spi.h"

/* KX122 Register Addresses */
#define KX122_REG_XOUT_L                0x06
#define KX122_REG_XOUT_H                0x07
#define KX122_REG_YOUT_L                0x08
#define KX122_REG_YOUT_H                0x09
#define KX122_REG_ZOUT_L                0x0A
#define KX122_REG_ZOUT_H                0x0B
#define KX122_REG_COTR                  0x0C
#define KX122_REG_WHO_AM_I              0x0F
#define KX122_REG_TSCP                  0x10
#define KX122_REG_TSPP                  0x11
#define KX122_REG_INS1                  0x12
#define KX122_REG_INS2                  0x13
#define KX122_REG_INS3                  0x14
#define KX122_REG_STAT                  0x15
#define KX122_REG_INT_REL               0x17
#define KX122_REG_CNTL1                 0x18
#define KX122_REG_CNTL2                 0x19
#define KX122_REG_CNTL3                 0x1A
#define KX122_REG_ODCNTL                0x1B
#define KX122_REG_INC1                  0x1C
#define KX122_REG_INC2                  0x1D
#define KX122_REG_INC3                  0x1E
#define KX122_REG_INC4                  0x1F
#define KX122_REG_INC5                  0x20
#define KX122_REG_INC6                  0x21
#define KX122_REG_TILT_TIMER            0x22
#define KX122_REG_WUFC                  0x23
#define KX122_REG_TDTRC                 0x24
#define KX122_REG_TDTC                  0x25
#define KX122_REG_TTH                   0x26
#define KX122_REG_TTL                   0x27
#define KX122_REG_FTD                   0x28
#define KX122_REG_STD                   0x29
#define KX122_REG_TLT                   0x2A
#define KX122_REG_TWS                   0x2B
#define KX122_REG_ATH                   0x30
#define KX122_REG_TILT_ANGLE_LL         0x32
#define KX122_REG_TILT_ANGLE_HL         0x33
#define KX122_REG_HYST_SET              0x34
#define KX122_REG_LP_CNTL               0x35
#define KX122_REG_BUF_CNTL1             0x3A
#define KX122_REG_BUF_CNTL2             0x3B
#define KX122_REG_BUF_STATUS_1          0x3C
#define KX122_REG_BUF_STATUS_2          0x3D
#define KX122_REG_BUF_CLEAR             0x3E
#define KX122_REG_BUF_READ              0x3F
#define KX122_REG_SELF_TEST             0x60

/* CNTL1 Register Bits */
#define KX122_CNTL1_PC1                 (1 << 7)  // Operating mode
#define KX122_CNTL1_RES                 (1 << 6)  // Performance mode
#define KX122_CNTL1_DRDYE               (1 << 5)  // Data ready enable
#define KX122_CNTL1_GSEL_2G             (0 << 3)  // ±2g range
#define KX122_CNTL1_GSEL_4G             (1 << 3)  // ±4g range
#define KX122_CNTL1_GSEL_8G             (2 << 3)  // ±8g range

/* ODCNTL Register - Output Data Rate */
#define KX122_ODR_0_781_HZ              0x00
#define KX122_ODR_1_563_HZ              0x01
#define KX122_ODR_3_125_HZ              0x02
#define KX122_ODR_6_25_HZ               0x03
#define KX122_ODR_12_5_HZ               0x04
#define KX122_ODR_25_HZ                 0x05
#define KX122_ODR_50_HZ                 0x06
#define KX122_ODR_100_HZ                0x07
#define KX122_ODR_200_HZ                0x08
#define KX122_ODR_400_HZ                0x09
#define KX122_ODR_800_HZ                0x0A
#define KX122_ODR_1600_HZ               0x0B

/* Who Am I value */
#define KX122_WHO_AM_I_VALUE            0x1B

/* G-range options */
typedef enum {
    KX122_RANGE_2G = 0,
    KX122_RANGE_4G = 1,
    KX122_RANGE_8G = 2
} KX122_Range_t;

/* Data structure for accelerometer data */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
    float x_g;
    float y_g;
    float z_g;
} KX122_AccelData_t;

/* CS Pin Control */
#define KX122_CS_LOW()   HAL_GPIO_WritePin(A_SPI0_CSACC_N_GPIO_Port, A_SPI0_CSACC_N_Pin, GPIO_PIN_RESET)
#define KX122_CS_HIGH()  HAL_GPIO_WritePin(A_SPI0_CSACC_N_GPIO_Port, A_SPI0_CSACC_N_Pin, GPIO_PIN_SET)

/* Function Prototypes */
HAL_StatusTypeDef KX122_Init(void);
HAL_StatusTypeDef KX122_ReadRegister(uint8_t reg, uint8_t *data);
HAL_StatusTypeDef KX122_WriteRegister(uint8_t reg, uint8_t data);
HAL_StatusTypeDef KX122_ReadAccelData(KX122_AccelData_t *accel_data);
HAL_StatusTypeDef KX122_SetRange(KX122_Range_t range);
HAL_StatusTypeDef KX122_SetOutputDataRate(uint8_t odr);
HAL_StatusTypeDef KX122_EnableOperatingMode(uint8_t enable);
uint8_t KX122_GetWhoAmI(void);
HAL_StatusTypeDef KX122_SoftReset(void);

#ifdef __cplusplus
}
#endif

#endif /* __KX122_H */
