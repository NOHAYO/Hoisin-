/*****************************************************************************
  kx122_example.c
  
  Example implementation showing how to use the KX122 driver with your
  platform-specific I2C functions.
  
******************************************************************************/

#include "kx122.h"
#include <stdio.h>
#include <stdint.h>

/* 
 * Platform-specific I2C functions that you need to implement 
 * These are just templates - replace with your actual I2C implementation
 */

/**
 * @brief Platform-specific I2C write function
 * 
 * @param dev_addr 7-bit I2C device address
 * @param reg_addr Register address to write to
 * @param data Pointer to data buffer
 * @param len Number of bytes to write
 * @return 0 on success, negative error code on failure
 */
int platform_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    /* TODO: Implement your platform-specific I2C write here */
    /* Example pseudocode:
     * 
     * i2c_start();
     * i2c_write_byte(dev_addr << 1);  // Write address
     * i2c_write_byte(reg_addr);       // Register address
     * for (int i = 0; i < len; i++) {
     *     i2c_write_byte(data[i]);    // Write data
     * }
     * i2c_stop();
     * 
     * return 0 on success, -1 on error
     */
    
    return 0;  // Replace with actual implementation
}

/**
 * @brief Platform-specific I2C read function
 * 
 * @param dev_addr 7-bit I2C device address
 * @param reg_addr Register address to read from
 * @param data Pointer to data buffer
 * @param len Number of bytes to read
 * @return 0 on success, negative error code on failure
 */
int platform_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    /* TODO: Implement your platform-specific I2C read here */
    /* Example pseudocode:
     * 
     * i2c_start();
     * i2c_write_byte(dev_addr << 1);      // Write address
     * i2c_write_byte(reg_addr);           // Register address
     * i2c_repeated_start();
     * i2c_write_byte((dev_addr << 1) | 1); // Read address
     * for (int i = 0; i < len; i++) {
     *     data[i] = i2c_read_byte(i < len-1); // Send ACK except for last byte
     * }
     * i2c_stop();
     * 
     * return 0 on success, -1 on error
     */
    
    return 0;  // Replace with actual implementation
}

int main(void)
{
    kx122_t kx122_sensor;
    kx122_accel_t accel_data;
    kx122_accel_raw_t raw_data;
    int result;
    
    /* Initialize the KX122 device structure */
    result = kx122_init_device(&kx122_sensor, 
                               KX122_DEVICE_ADDRESS_1E,  // or KX122_DEVICE_ADDRESS_1F
                               platform_i2c_write,
                               platform_i2c_read);
    
    if (result != KX122_SUCCESS) {
        printf("Failed to initialize KX122 device structure\n");
        return -1;
    }
    
    /* Initialize and configure the sensor */
    result = kx122_init(&kx122_sensor);
    if (result != KX122_SUCCESS) {
        printf("Failed to initialize KX122 sensor\n");
        return -1;
    }
    
    printf("KX122 initialized successfully!\n");
    
    /* Main loop - read acceleration data */
    while (1) {
        /* Read acceleration in g units */
        result = kx122_get_val(&kx122_sensor, &accel_data);
        if (result == KX122_SUCCESS) {
            printf("Acceleration (g): X=%.3f, Y=%.3f, Z=%.3f\n",
                   accel_data.x, accel_data.y, accel_data.z);
        }
        
        /* Or read raw acceleration values */
        result = kx122_get_raw_accel(&kx122_sensor, &raw_data);
        if (result == KX122_SUCCESS) {
            printf("Raw values: X=%d, Y=%d, Z=%d\n",
                   raw_data.x, raw_data.y, raw_data.z);
        }
        
        /* Add delay here (platform specific) */
        // delay_ms(100);
    }
    
    return 0;
}

/* Example: Changing accelerometer range */
void example_change_range(kx122_t *dev)
{
    int result;
    
    /* Change to 4G range */
    result = kx122_set_range(dev, KX122_CNTL1_GSEL_4G);
    if (result == KX122_SUCCESS) {
        printf("Range changed to 4G\n");
    }
    
    /* Change to 8G range */
    result = kx122_set_range(dev, KX122_CNTL1_GSEL_8G);
    if (result == KX122_SUCCESS) {
        printf("Range changed to 8G\n");
    }
    
    /* Change back to 2G range */
    result = kx122_set_range(dev, KX122_CNTL1_GSEL_2G);
    if (result == KX122_SUCCESS) {
        printf("Range changed to 2G\n");
    }
}

/* Example: Direct register access */
void example_direct_register_access(kx122_t *dev)
{
    uint8_t who_am_i;
    int result;
    
    /* Read WHO_AM_I register */
    result = kx122_read(dev, KX122_WHO_AM_I, &who_am_i, 1);
    if (result == KX122_SUCCESS) {
        printf("WHO_AM_I: 0x%02X (expected 0x%02X)\n", who_am_i, KX122_WAI_VAL);
    }
    
    /* Write to a configuration register */
    uint8_t config_value = KX122_ODCNTL_OSA_50HZ;
    result = kx122_write(dev, KX122_ODCNTL, &config_value, 1);
    if (result == KX122_SUCCESS) {
        printf("Configuration written successfully\n");
    }
}