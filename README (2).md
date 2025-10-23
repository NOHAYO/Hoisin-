# KX122 Accelerometer Driver - Standard C Implementation

This is a platform-independent C driver for the ROHM KX122 3-axis accelerometer sensor.

## Overview

The KX122 is a 3-axis accelerometer with configurable ranges (±2g, ±4g, ±8g) and high resolution output. This driver provides a clean API for interfacing with the sensor using I2C communication.

## Files

- **kx122.h** - Header file with register definitions, structures, and function prototypes
- **kx122.c** - Implementation of the driver functions
- **kx122_example.c** - Example code showing how to use the driver

## Features

- Platform-independent design using function pointers for I2C operations
- Support for all three accelerometer ranges (2g, 4g, 8g)
- Read acceleration in both raw values and floating-point g units
- Direct register access functions
- Soft reset capability
- Runtime range configuration

## Hardware Setup

### I2C Address
The KX122 can have one of two I2C addresses:
- `0x1E` (KX122_DEVICE_ADDRESS_1E)
- `0x1F` (KX122_DEVICE_ADDRESS_1F)

Select the appropriate address based on your hardware configuration.

### Pin Connections
- **VDD** - Power supply (1.71V to 3.6V)
- **GND** - Ground
- **SCL** - I2C Clock
- **SDA** - I2C Data
- **SDO** - Address select pin (affects I2C address)
- **INT1/INT2** - Interrupt outputs (optional)

## Integration Guide

### Step 1: Implement Platform-Specific I2C Functions

You need to provide two I2C functions for your platform:

```c
/**
 * Write data to I2C device
 * @param dev_addr: 7-bit I2C device address
 * @param reg_addr: Register address to write to
 * @param data: Pointer to data buffer
 * @param len: Number of bytes to write
 * @return: 0 on success, negative on error
 */
int platform_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len);

/**
 * Read data from I2C device
 * @param dev_addr: 7-bit I2C device address
 * @param reg_addr: Register address to read from
 * @param data: Pointer to data buffer
 * @param len: Number of bytes to read
 * @return: 0 on success, negative on error
 */
int platform_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len);
```

### Step 2: Initialize the Driver

```c
#include "kx122.h"

kx122_t sensor;
int result;

/* Initialize device structure with I2C functions */
result = kx122_init_device(&sensor, 
                           KX122_DEVICE_ADDRESS_1E,
                           platform_i2c_write,
                           platform_i2c_read);

/* Configure and start the sensor */
result = kx122_init(&sensor);
if (result != KX122_SUCCESS) {
    // Handle error
}
```

### Step 3: Read Acceleration Data

```c
kx122_accel_t accel;

/* Read acceleration in g units */
result = kx122_get_val(&sensor, &accel);
if (result == KX122_SUCCESS) {
    printf("X: %.3f g\n", accel.x);
    printf("Y: %.3f g\n", accel.y);
    printf("Z: %.3f g\n", accel.z);
}
```

## API Reference

### Initialization Functions

#### `kx122_init_device()`
```c
int kx122_init_device(kx122_t *dev, 
                      uint8_t slave_address,
                      int (*i2c_write_func)(uint8_t, uint8_t, uint8_t*, uint8_t),
                      int (*i2c_read_func)(uint8_t, uint8_t, uint8_t*, uint8_t));
```
Initialize the device structure with I2C functions and address.

#### `kx122_init()`
```c
int kx122_init(kx122_t *dev);
```
Configure and start the KX122 sensor with default settings.

### Data Reading Functions

#### `kx122_get_val()`
```c
int kx122_get_val(kx122_t *dev, kx122_accel_t *accel);
```
Read acceleration values in g units (floating-point).

#### `kx122_get_raw_accel()`
```c
int kx122_get_raw_accel(kx122_t *dev, kx122_accel_raw_t *accel);
```
Read raw acceleration values as 16-bit signed integers.

#### `kx122_get_rawval()`
```c
int kx122_get_rawval(kx122_t *dev, uint8_t *data);
```
Read 6 bytes of raw register data (X_L, X_H, Y_L, Y_H, Z_L, Z_H).

### Configuration Functions

#### `kx122_set_range()`
```c
int kx122_set_range(kx122_t *dev, uint8_t gsel);
```
Set accelerometer range. Valid values:
- `KX122_CNTL1_GSEL_2G` - ±2g range
- `KX122_CNTL1_GSEL_4G` - ±4g range
- `KX122_CNTL1_GSEL_8G` - ±8g range

#### `kx122_soft_reset()`
```c
int kx122_soft_reset(kx122_t *dev);
```
Put sensor into standby mode (soft reset).

### Low-Level Register Access

#### `kx122_write()`
```c
int kx122_write(kx122_t *dev, uint8_t reg_address, uint8_t *data, uint8_t size);
```
Write data to sensor registers.

#### `kx122_read()`
```c
int kx122_read(kx122_t *dev, uint8_t reg_address, uint8_t *data, uint8_t size);
```
Read data from sensor registers.

## Data Structures

### `kx122_t`
Device context structure containing:
- Device I2C address
- Current sensitivity setting
- Function pointers to I2C functions

### `kx122_accel_t`
Acceleration data in g units:
```c
typedef struct {
    float x;
    float y;
    float z;
} kx122_accel_t;
```

### `kx122_accel_raw_t`
Raw acceleration data (16-bit signed):
```c
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} kx122_accel_raw_t;
```

## Configuration Details

### Default Configuration
- **Range**: ±2g
- **Resolution**: High resolution mode (16-bit)
- **Output Data Rate**: 50 Hz
- **Operating Mode**: Active (PC1 enabled)

### Sensitivity Values
- **2g range**: 16384 LSB/g
- **4g range**: 8192 LSB/g
- **8g range**: 4096 LSB/g

## Example Code

### Basic Usage
```c
kx122_t sensor;
kx122_accel_t accel;

// Initialize
kx122_init_device(&sensor, KX122_DEVICE_ADDRESS_1E, 
                  my_i2c_write, my_i2c_read);
kx122_init(&sensor);

// Read data
while(1) {
    if (kx122_get_val(&sensor, &accel) == KX122_SUCCESS) {
        printf("Accel: X=%.2f Y=%.2f Z=%.2f g\n", 
               accel.x, accel.y, accel.z);
    }
    delay_ms(100);
}
```

### Changing Range
```c
// Change to 4g range for higher acceleration measurement
kx122_set_range(&sensor, KX122_CNTL1_GSEL_4G);

// Change to 8g range for even higher acceleration
kx122_set_range(&sensor, KX122_CNTL1_GSEL_8G);
```

### Reading Raw Values
```c
kx122_accel_raw_t raw;

if (kx122_get_raw_accel(&sensor, &raw) == KX122_SUCCESS) {
    printf("Raw: X=%d Y=%d Z=%d\n", raw.x, raw.y, raw.z);
}
```

## Platform Examples

### STM32 HAL
```c
int stm32_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len) {
    uint8_t buf[32];
    buf[0] = reg_addr;
    memcpy(&buf[1], data, len);
    
    if (HAL_I2C_Master_Transmit(&hi2c1, dev_addr << 1, buf, len + 1, 100) == HAL_OK)
        return 0;
    return -1;
}

int stm32_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len) {
    if (HAL_I2C_Mem_Read(&hi2c1, dev_addr << 1, reg_addr, 
                         I2C_MEMADD_SIZE_8BIT, data, len, 100) == HAL_OK)
        return 0;
    return -1;
}
```

### ESP-IDF
```c
int esp_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return (ret == ESP_OK) ? 0 : -1;
}

int esp_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return (ret == ESP_OK) ? 0 : -1;
}
```

## Error Handling

All functions return:
- `KX122_SUCCESS` (0) on success
- `KX122_ERROR` (-1) on failure

Always check return values:
```c
if (kx122_init(&sensor) != KX122_SUCCESS) {
    printf("Initialization failed!\n");
    // Handle error
}
```

## License

This software is provided under the MIT License. See the source files for full license text.

Copyright (c) 2018 ROHM Co.,Ltd.

## Additional Resources

- [KX122 Datasheet](https://www.rohm.com/products/sensors-mems/accelerometer-ics/kx122-product)
- ROHM Sensor Shield Arduino Examples
- Application Notes from ROHM

## Troubleshooting

### Sensor not detected (WHO_AM_I mismatch)
- Check I2C wiring and pull-up resistors
- Verify correct I2C address (0x1E or 0x1F)
- Ensure proper power supply voltage
- Check SDO pin configuration

### Reading all zeros
- Verify sensor is in active mode (PC1 bit set)
- Check if sensor needs time to stabilize after power-on
- Ensure proper I2C read implementation

### Inconsistent readings
- Check for I2C bus noise or interference
- Verify proper decoupling capacitors near sensor
- Consider lowering I2C clock speed
- Check sensor mounting and mechanical stability
