/*****************************************************************************
  kx122.h

 Copyright (c) 2018 ROHM Co.,Ltd.
 Modified for standard C implementation

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
******************************************************************************/

#ifndef _KX122_H_
#define _KX122_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Device I2C Addresses (7-bit) */
#define KX122_DEVICE_ADDRESS_1E   (0x1E)
#define KX122_DEVICE_ADDRESS_1F   (0x1F)
#define KX122_WAI_VAL             (0x1B)

/* Register Addresses */
#define KX122_XOUT_L              (0x06)
#define KX122_XOUT_H              (0x07)
#define KX122_YOUT_L              (0x08)
#define KX122_YOUT_H              (0x09)
#define KX122_ZOUT_L              (0x0A)
#define KX122_ZOUT_H              (0x0B)
#define KX122_WHO_AM_I            (0x0F)
#define KX122_CNTL1               (0x18)
#define KX122_ODCNTL              (0x1B)

/* CNTL1 Register Bits */
#define KX122_CNTL1_TPE           (1 << 0)
#define KX122_CNTL1_WUFE          (1 << 1)
#define KX122_CNTL1_TDTE          (1 << 2)
#define KX122_CNTL1_GSELMASK      (0x18)
#define KX122_CNTL1_GSEL_2G       (0x00)
#define KX122_CNTL1_GSEL_4G       (0x08)
#define KX122_CNTL1_GSEL_8G       (0x10)
#define KX122_CNTL1_DRDYE         (1 << 5)
#define KX122_CNTL1_RES           (1 << 6)
#define KX122_CNTL1_PC1           (1 << 7)

/* ODCNTL Register Bits */
#define KX122_ODCNTL_OSA_50HZ     (2)
#define KX122_ODCNTL_LPRO         (1 << 6)
#define KX122_IIR_BYPASS          (1 << 7)

/* Default Configuration Values */
#define KX122_CNTL1_VAL           (KX122_CNTL1_RES | KX122_CNTL1_GSEL_2G)
#define KX122_ODCNTL_VAL          (KX122_ODCNTL_OSA_50HZ)

/* Return Codes */
#define KX122_SUCCESS             (0)
#define KX122_ERROR               (-1)

/* Structure to hold KX122 device context */
typedef struct {
    uint8_t device_address;
    uint16_t g_sens;
    
    /* I2C function pointers for platform abstraction */
    int (*i2c_write)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len);
    int (*i2c_read)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len);
} kx122_t;

/* Acceleration data structure */
typedef struct {
    float x;
    float y;
    float z;
} kx122_accel_t;

/* Raw acceleration data structure */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} kx122_accel_raw_t;

/**
 * @brief Initialize the KX122 device structure
 * 
 * @param dev Pointer to KX122 device structure
 * @param slave_address I2C address of the device
 * @param i2c_write_func Platform-specific I2C write function
 * @param i2c_read_func Platform-specific I2C read function
 * @return KX122_SUCCESS on success, KX122_ERROR on failure
 */
int kx122_init_device(kx122_t *dev, 
                      uint8_t slave_address,
                      int (*i2c_write_func)(uint8_t, uint8_t, uint8_t*, uint8_t),
                      int (*i2c_read_func)(uint8_t, uint8_t, uint8_t*, uint8_t));

/**
 * @brief Initialize and configure the KX122 sensor
 * 
 * @param dev Pointer to KX122 device structure
 * @return KX122_SUCCESS on success, KX122_ERROR on failure
 */
int kx122_init(kx122_t *dev);

/**
 * @brief Get raw acceleration values
 * 
 * @param dev Pointer to KX122 device structure
 * @param data Pointer to buffer for 6 bytes of raw data
 * @return KX122_SUCCESS on success, KX122_ERROR on failure
 */
int kx122_get_rawval(kx122_t *dev, uint8_t *data);

/**
 * @brief Get acceleration values in g units
 * 
 * @param dev Pointer to KX122 device structure
 * @param accel Pointer to acceleration data structure
 * @return KX122_SUCCESS on success, KX122_ERROR on failure
 */
int kx122_get_val(kx122_t *dev, kx122_accel_t *accel);

/**
 * @brief Get raw acceleration values as signed 16-bit integers
 * 
 * @param dev Pointer to KX122 device structure
 * @param accel Pointer to raw acceleration data structure
 * @return KX122_SUCCESS on success, KX122_ERROR on failure
 */
int kx122_get_raw_accel(kx122_t *dev, kx122_accel_raw_t *accel);

/**
 * @brief Write data to KX122 register
 * 
 * @param dev Pointer to KX122 device structure
 * @param reg_address Register address
 * @param data Pointer to data to write
 * @param size Number of bytes to write
 * @return KX122_SUCCESS on success, KX122_ERROR on failure
 */
int kx122_write(kx122_t *dev, uint8_t reg_address, uint8_t *data, uint8_t size);

/**
 * @brief Read data from KX122 register
 * 
 * @param dev Pointer to KX122 device structure
 * @param reg_address Register address
 * @param data Pointer to buffer for read data
 * @param size Number of bytes to read
 * @return KX122_SUCCESS on success, KX122_ERROR on failure
 */
int kx122_read(kx122_t *dev, uint8_t reg_address, uint8_t *data, uint8_t size);

/**
 * @brief Software reset the KX122 sensor
 * 
 * @param dev Pointer to KX122 device structure
 * @return KX122_SUCCESS on success, KX122_ERROR on failure
 */
int kx122_soft_reset(kx122_t *dev);

/**
 * @brief Set the g-range of the accelerometer
 * 
 * @param dev Pointer to KX122 device structure
 * @param gsel G-range selection (KX122_CNTL1_GSEL_2G, _4G, or _8G)
 * @return KX122_SUCCESS on success, KX122_ERROR on failure
 */
int kx122_set_range(kx122_t *dev, uint8_t gsel);

#ifdef __cplusplus
}
#endif

#endif /* _KX122_H_ */