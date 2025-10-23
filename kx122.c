/*****************************************************************************
  kx122.c

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

#include "kx122.h"
#include <stddef.h>

int kx122_init_device(kx122_t *dev, 
                      uint8_t slave_address,
                      int (*i2c_write_func)(uint8_t, uint8_t, uint8_t*, uint8_t),
                      int (*i2c_read_func)(uint8_t, uint8_t, uint8_t*, uint8_t))
{
    if (dev == NULL || i2c_write_func == NULL || i2c_read_func == NULL) {
        return KX122_ERROR;
    }
    
    dev->device_address = slave_address;
    dev->i2c_write = i2c_write_func;
    dev->i2c_read = i2c_read_func;
    dev->g_sens = 16384; /* Default for 2G range */
    
    return KX122_SUCCESS;
}

int kx122_init(kx122_t *dev)
{
    int rc;
    uint8_t reg;
    uint8_t gsel;
    
    if (dev == NULL) {
        return KX122_ERROR;
    }
    
    /* Read WHO_AM_I register to verify device */
    rc = kx122_read(dev, KX122_WHO_AM_I, &reg, 1);
    if (rc != KX122_SUCCESS) {
        return rc;
    }
    
    if (reg != KX122_WAI_VAL) {
        return KX122_ERROR;
    }
    
    /* Set CNTL1 register (standby mode first) */
    reg = KX122_CNTL1_VAL;
    rc = kx122_write(dev, KX122_CNTL1, &reg, 1);
    if (rc != KX122_SUCCESS) {
        return rc;
    }
    
    /* Set output data rate */
    reg = KX122_ODCNTL_VAL;
    rc = kx122_write(dev, KX122_ODCNTL, &reg, 1);
    if (rc != KX122_SUCCESS) {
        return rc;
    }
    
    /* Read back CNTL1 to get g-range setting */
    rc = kx122_read(dev, KX122_CNTL1, &reg, 1);
    if (rc != KX122_SUCCESS) {
        return rc;
    }
    
    gsel = reg & KX122_CNTL1_GSELMASK;
    
    /* Enable operating mode (PC1 bit) */
    reg |= KX122_CNTL1_PC1;
    rc = kx122_write(dev, KX122_CNTL1, &reg, 1);
    if (rc != KX122_SUCCESS) {
        return rc;
    }
    
    /* Set sensitivity based on g-range */
    switch(gsel) {
        case KX122_CNTL1_GSEL_2G:
            dev->g_sens = 16384;
            break;
        case KX122_CNTL1_GSEL_4G:
            dev->g_sens = 8192;
            break;
        case KX122_CNTL1_GSEL_8G:
            dev->g_sens = 4096;
            break;
        default:
            dev->g_sens = 16384;
            break;
    }
    
    return KX122_SUCCESS;
}

int kx122_get_rawval(kx122_t *dev, uint8_t *data)
{
    if (dev == NULL || data == NULL) {
        return KX122_ERROR;
    }
    
    return kx122_read(dev, KX122_XOUT_L, data, 6);
}

int kx122_get_val(kx122_t *dev, kx122_accel_t *accel)
{
    int rc;
    uint8_t val[6];
    int16_t acc[3];
    
    if (dev == NULL || accel == NULL) {
        return KX122_ERROR;
    }
    
    rc = kx122_get_rawval(dev, val);
    if (rc != KX122_SUCCESS) {
        return rc;
    }
    
    /* Combine bytes into signed 16-bit values (little-endian) */
    acc[0] = (int16_t)((val[1] << 8) | val[0]);
    acc[1] = (int16_t)((val[3] << 8) | val[2]);
    acc[2] = (int16_t)((val[5] << 8) | val[4]);
    
    /* Convert LSB to g */
    accel->x = (float)acc[0] / (float)dev->g_sens;
    accel->y = (float)acc[1] / (float)dev->g_sens;
    accel->z = (float)acc[2] / (float)dev->g_sens;
    
    return KX122_SUCCESS;
}

int kx122_get_raw_accel(kx122_t *dev, kx122_accel_raw_t *accel)
{
    int rc;
    uint8_t val[6];
    
    if (dev == NULL || accel == NULL) {
        return KX122_ERROR;
    }
    
    rc = kx122_get_rawval(dev, val);
    if (rc != KX122_SUCCESS) {
        return rc;
    }
    
    /* Combine bytes into signed 16-bit values (little-endian) */
    accel->x = (int16_t)((val[1] << 8) | val[0]);
    accel->y = (int16_t)((val[3] << 8) | val[2]);
    accel->z = (int16_t)((val[5] << 8) | val[4]);
    
    return KX122_SUCCESS;
}

int kx122_write(kx122_t *dev, uint8_t reg_address, uint8_t *data, uint8_t size)
{
    if (dev == NULL || data == NULL || dev->i2c_write == NULL) {
        return KX122_ERROR;
    }
    
    return dev->i2c_write(dev->device_address, reg_address, data, size);
}

int kx122_read(kx122_t *dev, uint8_t reg_address, uint8_t *data, uint8_t size)
{
    if (dev == NULL || data == NULL || dev->i2c_read == NULL) {
        return KX122_ERROR;
    }
    
    return dev->i2c_read(dev->device_address, reg_address, data, size);
}

int kx122_soft_reset(kx122_t *dev)
{
    int rc;
    uint8_t reg;
    
    if (dev == NULL) {
        return KX122_ERROR;
    }
    
    /* Read current CNTL1 value */
    rc = kx122_read(dev, KX122_CNTL1, &reg, 1);
    if (rc != KX122_SUCCESS) {
        return rc;
    }
    
    /* Clear PC1 bit to put device in standby */
    reg &= ~KX122_CNTL1_PC1;
    rc = kx122_write(dev, KX122_CNTL1, &reg, 1);
    if (rc != KX122_SUCCESS) {
        return rc;
    }
    
    return KX122_SUCCESS;
}

int kx122_set_range(kx122_t *dev, uint8_t gsel)
{
    int rc;
    uint8_t reg;
    
    if (dev == NULL) {
        return KX122_ERROR;
    }
    
    /* Validate g-range selection */
    if (gsel != KX122_CNTL1_GSEL_2G && 
        gsel != KX122_CNTL1_GSEL_4G && 
        gsel != KX122_CNTL1_GSEL_8G) {
        return KX122_ERROR;
    }
    
    /* Read current CNTL1 value */
    rc = kx122_read(dev, KX122_CNTL1, &reg, 1);
    if (rc != KX122_SUCCESS) {
        return rc;
    }
    
    /* Device must be in standby mode to change range */
    uint8_t was_active = reg & KX122_CNTL1_PC1;
    if (was_active) {
        reg &= ~KX122_CNTL1_PC1;
        rc = kx122_write(dev, KX122_CNTL1, &reg, 1);
        if (rc != KX122_SUCCESS) {
            return rc;
        }
    }
    
    /* Clear existing range bits and set new range */
    reg &= ~KX122_CNTL1_GSELMASK;
    reg |= gsel;
    rc = kx122_write(dev, KX122_CNTL1, &reg, 1);
    if (rc != KX122_SUCCESS) {
        return rc;
    }
    
    /* Restore active mode if it was active */
    if (was_active) {
        reg |= KX122_CNTL1_PC1;
        rc = kx122_write(dev, KX122_CNTL1, &reg, 1);
        if (rc != KX122_SUCCESS) {
            return rc;
        }
    }
    
    /* Update sensitivity value */
    switch(gsel) {
        case KX122_CNTL1_GSEL_2G:
            dev->g_sens = 16384;
            break;
        case KX122_CNTL1_GSEL_4G:
            dev->g_sens = 8192;
            break;
        case KX122_CNTL1_GSEL_8G:
            dev->g_sens = 4096;
            break;
    }
    
    return KX122_SUCCESS;
}