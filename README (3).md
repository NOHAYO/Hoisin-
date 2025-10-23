# MAX20303 PMIC Driver

Comprehensive C driver for the Maxim Integrated MAX20303 PMIC with Ultra Low IQ Voltage Regulators, Battery Charger, and Fuel Gauge for Small Lithium Ion Systems.

## Features

The MAX20303 is a highly integrated power management IC optimized for ultra-low-power wearable applications. This driver provides complete access to all device features:

### Power Regulators
- **2x Buck Regulators** (<1µA quiescent current each)
  - Buck1: 0.8V to 2.375V in 25mV steps, 220mA output
  - Buck2: 0.8V to 3.95V in 50mV steps, 220mA output
- **Micro-IQ LDO** (1µA quiescent current)
  - 0.9V to 4.0V in 100mV steps, 100mA output
- **Micro-IQ LV-LDO** (1µA quiescent current)
  - 0.5V to 1.95V in 25mV steps, 50mA output
- **Buck-Boost Regulator** (1.3µA quiescent current)
  - 2.5V to 5.0V in 100mV steps, 250mW output
- **Boost Regulator** (2.4µA quiescent current)
  - 5V to 20V in 250mV steps, 300mW output

### Battery Management
- **Li+ Battery Charger**
  - Fast charge current: 5mA to 500mA
  - Programmable JEITA current/voltage profiles
  - 28V/-5.5V tolerant input
  - Smart power selector
- **Fuel Gauge**
  - Battery voltage monitoring
  - State of charge (SOC) estimation
  - Current measurement

### Additional Features
- **3x LED Current Sinks** (20V tolerant, 0.6mA to 30mA)
- **Haptic Driver** (ERM/LRA with automatic resonance tracking)
- **I2C Interface** for configuration and monitoring
- **Interrupt System** for event notification
- **Integrated ADC** for voltage monitoring

## Hardware Requirements

- MAX20303 PMIC IC
- I2C interface (master mode)
- Pull-up resistors on I2C lines (typically 4.7kΩ)
- Appropriate power supply and battery connections

## I2C Addresses

The MAX20303 uses two I2C addresses:
- **Primary PMIC Address**: 0x28 (7-bit) / 0x50 (8-bit write) / 0x51 (8-bit read)
- **Fuel Gauge Address**: 0x36 (7-bit) / 0x6C (8-bit write) / 0x6D (8-bit read)

## Files

- `max20303.h` - Header file with register definitions and function prototypes
- `max20303.c` - Implementation of all driver functions
- `max20303_example.c` - Example usage code demonstrating all features
- `README.md` - This documentation file

## Integration Guide

### 1. Platform-Specific Implementation

You must implement three platform-specific functions for I2C communication:

```c
/**
 * I2C Write Function
 * @param handle: Your I2C peripheral handle
 * @param addr: 7-bit I2C device address
 * @param reg: Register address to write
 * @param data: Pointer to data buffer
 * @param len: Number of bytes to write
 * @return 0 on success, negative on error
 */
int platform_i2c_write(void *handle, uint8_t addr, uint8_t reg, 
                       uint8_t *data, uint16_t len);

/**
 * I2C Read Function
 * @param handle: Your I2C peripheral handle
 * @param addr: 7-bit I2C device address
 * @param reg: Register address to read from
 * @param data: Pointer to data buffer
 * @param len: Number of bytes to read
 * @return 0 on success, negative on error
 */
int platform_i2c_read(void *handle, uint8_t addr, uint8_t reg, 
                      uint8_t *data, uint16_t len);

/**
 * Delay Function
 * @param ms: Delay time in milliseconds
 * @return 0 on success
 */
int platform_delay_ms(uint32_t ms);
```

### 2. Basic Initialization

```c
#include "max20303.h"

max20303_handle_t pmic;

// Initialize the device handle
pmic.i2c_addr = MAX20303_I2C_ADDR_7BIT;
pmic.fg_i2c_addr = MAX20303_FG_I2C_ADDR_7BIT;
pmic.i2c_handle = &hi2c1;  // Your I2C peripheral handle
pmic.i2c_write = platform_i2c_write;
pmic.i2c_read = platform_i2c_read;
pmic.delay_ms = platform_delay_ms;

// Initialize the PMIC
if (max20303_init(&pmic) != 0) {
    // Handle error
}
```

### 3. Common Usage Examples

#### Configure Buck Regulator

```c
// Set Buck1 to 1.8V and enable
max20303_buck_set_voltage(&pmic, 1, 1800);
max20303_buck_enable(&pmic, 1, true);

// Or use the configuration structure
max20303_buck_config_t buck_cfg = {
    .regulator_id = 1,
    .voltage_mv = 1800,
    .enabled = true
};
max20303_buck_config(&pmic, &buck_cfg);
```

#### Configure LDO

```c
// Regular LDO at 2.8V
max20303_ldo_set_voltage(&pmic, false, 2800);
max20303_ldo_enable(&pmic, false, true);

// Low-voltage LDO at 1.2V
max20303_ldo_set_voltage(&pmic, true, 1200);
max20303_ldo_enable(&pmic, true, true);
```

#### Control LED

```c
// Set LED0 to 10mA
max20303_led_set_current(&pmic, 0, 10000);

// Turn off LED
max20303_led_set_current(&pmic, 0, 0);
```

#### Configure Battery Charger

```c
max20303_charger_config_t chg_cfg = {
    .fast_charge_ma = 100,
    .voltage_mv = 4200,
    .input_current_limit_ma = 500,
    .enabled = true
};
max20303_charger_config(&pmic, &chg_cfg);
```

#### Read Battery Status

```c
max20303_battery_status_t status;
max20303_fg_read_battery_status(&pmic, &status);

printf("Battery: %u mV, %u%%, %s\n", 
       status.voltage_mv, 
       status.soc_percent,
       status.charging ? "Charging" : "Discharging");
```

#### Configure Haptic Driver

```c
max20303_haptic_config_t haptic = {
    .type = MAX20303_HAPTIC_LRA,
    .drive_strength = 75,
    .frequency_hz = 175,
    .enabled = true
};
max20303_haptic_config(&pmic, &haptic);
max20303_haptic_play(&pmic, 1);  // Play pattern 1
```

## API Reference

### Initialization Functions

- `max20303_init()` - Initialize the PMIC
- `max20303_reset()` - Reset the device
- `max20303_get_version()` - Read device version

### Buck Regulator Functions

- `max20303_buck_config()` - Configure buck regulator
- `max20303_buck_enable()` - Enable/disable buck regulator
- `max20303_buck_set_voltage()` - Set buck output voltage

### LDO Functions

- `max20303_ldo_config()` - Configure LDO
- `max20303_ldo_enable()` - Enable/disable LDO
- `max20303_ldo_set_voltage()` - Set LDO output voltage

### Boost Functions

- `max20303_boost_config()` - Configure boost regulator
- `max20303_boost_enable()` - Enable/disable boost
- `max20303_boost_set_voltage()` - Set boost output voltage

### Buck-Boost Functions

- `max20303_buckboost_config()` - Configure buck-boost regulator
- `max20303_buckboost_enable()` - Enable/disable buck-boost

### LED Functions

- `max20303_led_config()` - Configure LED driver
- `max20303_led_set_current()` - Set LED current

### Charger Functions

- `max20303_charger_config()` - Configure battery charger
- `max20303_charger_enable()` - Enable/disable charger
- `max20303_charger_set_current()` - Set charge current

### Fuel Gauge Functions

- `max20303_fg_read_battery_status()` - Read complete battery status
- `max20303_fg_read_voltage()` - Read battery voltage
- `max20303_fg_read_soc()` - Read state of charge

### Haptic Functions

- `max20303_haptic_config()` - Configure haptic driver
- `max20303_haptic_play()` - Play haptic pattern
- `max20303_haptic_stop()` - Stop haptic playback

### Interrupt Functions

- `max20303_read_interrupts()` - Read interrupt status
- `max20303_clear_interrupts()` - Clear interrupt flags
- `max20303_set_interrupt_mask()` - Configure interrupt masks

### Low-Level I2C Functions

- `max20303_write_reg()` - Write single register
- `max20303_read_reg()` - Read single register
- `max20303_write_regs()` - Write multiple registers
- `max20303_read_regs()` - Read multiple registers
- `max20303_send_ap_command()` - Send AP command

## Register Map Summary

### Direct Access Registers (Primary I2C Address)

| Address | Name | Description |
|---------|------|-------------|
| 0x00 | INT0 | Interrupt 0 register |
| 0x01 | INT1 | Interrupt 1 register |
| 0x02 | INT2 | Interrupt 2 register |
| 0x03-0x07 | STATUS | Status registers |
| 0x08-0x0A | INT_MASK | Interrupt mask registers |
| 0x0B-0x11 | AP_DATOUT | AP data out registers |
| 0x17 | AP_CMDOUT | AP command out |
| 0x18 | AP_RESPONSE | AP response |
| 0x19-0x1E | AP_DATAIN | AP data in registers |
| 0x20 | LDO_DIRECT | LDO direct control |
| 0x21-0x22 | MPC_DIRECT | MPC direct registers |
| 0x28-0x2A | HPT_RAM | Haptic RAM registers |
| 0x2C-0x2F | LED_DIRECT | LED direct registers |
| 0x30-0x33 | HPT_DIRECT | Haptic direct registers |

### Fuel Gauge Registers (Fuel Gauge I2C Address)

| Address | Name | Description |
|---------|------|-------------|
| 0x02 | VCELL | Battery voltage |
| 0x04 | SOC | State of charge |
| 0x06 | MODE | Mode register |
| 0x08 | VERSION | Version |
| 0x0A | HIBRT | Hibernate |
| 0x0C | CONFIG | Configuration |
| 0x14 | VALRT | Voltage alert |
| 0x16 | CRATE | C-rate |
| 0x1A | STATUS | Status |

## Error Codes

All functions return:
- `0` on success
- `-1` for invalid parameters
- `-2` for timeout
- `-3` for command error
- Other negative values for I2C communication errors

## Notes and Best Practices

1. **Power Sequencing**: Ensure proper power-up sequence for your application
2. **Interrupt Handling**: Read interrupt registers to clear flags
3. **Battery Monitoring**: Regularly check battery status in battery-powered applications
4. **Thermal Management**: Monitor junction temperature in high-power applications
5. **I2C Speed**: The device supports up to 400kHz I2C (Fast Mode)
6. **Pull-ups**: Ensure proper I2C pull-up resistors (typically 4.7kΩ)

## License

This driver is provided as-is for use with the MAX20303 PMIC. Please refer to your project's license for distribution terms.

## Support

For MAX20303 datasheet and additional information:
- Visit Analog Devices website: https://www.analog.com
- Search for MAX20303 datasheet

## Version History

- v1.0 (2025) - Initial release
  - Complete register definitions
  - All power management functions
  - Battery charger and fuel gauge support
  - LED and haptic driver support
  - Interrupt handling
  - Comprehensive examples

## Author

Generated from MAX20303 datasheet specifications.
