/**
 * @file max20303_example.c
 * @brief MAX20303 Driver Usage Examples
 * 
 * This file demonstrates how to use the MAX20303 PMIC driver.
 * You'll need to implement the platform-specific I2C functions.
 * 
 * @author Example Code
 * @date 2025
 */

#include "max20303.h"
#include <stdio.h>

/* ===================================================================== */
/* PLATFORM-SPECIFIC I2C FUNCTIONS (USER MUST IMPLEMENT)                */
/* ===================================================================== */

/**
 * Platform-specific I2C write function
 * This is just a template - implement for your platform
 */
int platform_i2c_write(void *handle, uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len)
{
    // Example implementation for your platform:
    // 1. Start I2C transaction with device address (addr)
    // 2. Write register address (reg)
    // 3. Write data bytes (data, len)
    // 4. Stop I2C transaction
    // Return 0 on success, negative on error
    
    printf("I2C Write: Addr=0x%02X, Reg=0x%02X, Len=%d\n", addr, reg, len);
    return 0; // Success
}

/**
 * Platform-specific I2C read function
 * This is just a template - implement for your platform
 */
int platform_i2c_read(void *handle, uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len)
{
    // Example implementation for your platform:
    // 1. Start I2C transaction with device address (addr)
    // 2. Write register address (reg)
    // 3. Repeated start
    // 4. Read data bytes (data, len)
    // 5. Stop I2C transaction
    // Return 0 on success, negative on error
    
    printf("I2C Read: Addr=0x%02X, Reg=0x%02X, Len=%d\n", addr, reg, len);
    return 0; // Success
}

/**
 * Platform-specific delay function
 */
int platform_delay_ms(uint32_t ms)
{
    // Implement platform-specific delay
    // For example: HAL_Delay(ms) on STM32
    printf("Delay: %ums\n", ms);
    return 0;
}

/* ===================================================================== */
/* EXAMPLE 1: BASIC INITIALIZATION                                       */
/* ===================================================================== */

void example_basic_init(void)
{
    max20303_handle_t pmic;
    int ret;
    uint16_t version;
    
    printf("\n=== Example 1: Basic Initialization ===\n");
    
    // Initialize device handle
    pmic.i2c_addr = MAX20303_I2C_ADDR_7BIT;
    pmic.fg_i2c_addr = MAX20303_FG_I2C_ADDR_7BIT;
    pmic.i2c_handle = NULL; // Set to your I2C peripheral handle
    pmic.i2c_write = platform_i2c_write;
    pmic.i2c_read = platform_i2c_read;
    pmic.delay_ms = platform_delay_ms;
    
    // Initialize the PMIC
    ret = max20303_init(&pmic);
    if (ret != 0) {
        printf("ERROR: Failed to initialize MAX20303 (error %d)\n", ret);
        return;
    }
    
    printf("MAX20303 initialized successfully!\n");
    
    // Read device version
    ret = max20303_get_version(&pmic, &version);
    if (ret == 0) {
        printf("Device version: 0x%04X\n", version);
    }
}

/* ===================================================================== */
/* EXAMPLE 2: CONFIGURE BUCK REGULATORS                                  */
/* ===================================================================== */

void example_buck_regulators(void)
{
    max20303_handle_t pmic;
    max20303_buck_config_t buck1_cfg, buck2_cfg;
    int ret;
    
    printf("\n=== Example 2: Buck Regulators ===\n");
    
    // Initialize (setup as in example 1)
    pmic.i2c_addr = MAX20303_I2C_ADDR_7BIT;
    pmic.fg_i2c_addr = MAX20303_FG_I2C_ADDR_7BIT;
    pmic.i2c_write = platform_i2c_write;
    pmic.i2c_read = platform_i2c_read;
    pmic.delay_ms = platform_delay_ms;
    max20303_init(&pmic);
    
    // Configure Buck1 for 1.8V output (for MCU core)
    buck1_cfg.regulator_id = 1;
    buck1_cfg.voltage_mv = 1800;
    buck1_cfg.enabled = true;
    
    ret = max20303_buck_config(&pmic, &buck1_cfg);
    if (ret == 0) {
        printf("Buck1 configured: 1.8V, enabled\n");
    }
    
    // Configure Buck2 for 3.3V output (for peripherals)
    buck2_cfg.regulator_id = 2;
    buck2_cfg.voltage_mv = 3300;
    buck2_cfg.enabled = true;
    
    ret = max20303_buck_config(&pmic, &buck2_cfg);
    if (ret == 0) {
        printf("Buck2 configured: 3.3V, enabled\n");
    }
    
    // Alternative: Set voltage and enable separately
    ret = max20303_buck_set_voltage(&pmic, 1, 1500);
    ret |= max20303_buck_enable(&pmic, 1, true);
    if (ret == 0) {
        printf("Buck1 reconfigured: 1.5V\n");
    }
}

/* ===================================================================== */
/* EXAMPLE 3: CONFIGURE LDO REGULATORS                                   */
/* ===================================================================== */

void example_ldo_regulators(void)
{
    max20303_handle_t pmic;
    max20303_ldo_config_t ldo_cfg, lvldo_cfg;
    int ret;
    
    printf("\n=== Example 3: LDO Regulators ===\n");
    
    // Initialize
    pmic.i2c_addr = MAX20303_I2C_ADDR_7BIT;
    pmic.fg_i2c_addr = MAX20303_FG_I2C_ADDR_7BIT;
    pmic.i2c_write = platform_i2c_write;
    pmic.i2c_read = platform_i2c_read;
    pmic.delay_ms = platform_delay_ms;
    max20303_init(&pmic);
    
    // Configure regular LDO for 2.8V (for sensor)
    ldo_cfg.voltage_mv = 2800;
    ldo_cfg.enabled = true;
    ldo_cfg.is_lv_ldo = false;
    
    ret = max20303_ldo_config(&pmic, &ldo_cfg);
    if (ret == 0) {
        printf("LDO configured: 2.8V, enabled\n");
    }
    
    // Configure low-voltage LDO for 1.2V (for memory)
    lvldo_cfg.voltage_mv = 1200;
    lvldo_cfg.enabled = true;
    lvldo_cfg.is_lv_ldo = true;
    
    ret = max20303_ldo_config(&pmic, &lvldo_cfg);
    if (ret == 0) {
        printf("LV-LDO configured: 1.2V, enabled\n");
    }
}

/* ===================================================================== */
/* EXAMPLE 4: CONFIGURE BOOST REGULATOR                                  */
/* ===================================================================== */

void example_boost_regulator(void)
{
    max20303_handle_t pmic;
    max20303_boost_config_t boost_cfg;
    int ret;
    
    printf("\n=== Example 4: Boost Regulator ===\n");
    
    // Initialize
    pmic.i2c_addr = MAX20303_I2C_ADDR_7BIT;
    pmic.fg_i2c_addr = MAX20303_FG_I2C_ADDR_7BIT;
    pmic.i2c_write = platform_i2c_write;
    pmic.i2c_read = platform_i2c_read;
    pmic.delay_ms = platform_delay_ms;
    max20303_init(&pmic);
    
    // Configure boost for 12V output (for display backlight)
    boost_cfg.voltage_mv = 12000;
    boost_cfg.enabled = true;
    
    ret = max20303_boost_config(&pmic, &boost_cfg);
    if (ret == 0) {
        printf("Boost configured: 12V, enabled\n");
    }
    
    // Disable boost when not needed
    ret = max20303_boost_enable(&pmic, false);
    if (ret == 0) {
        printf("Boost disabled\n");
    }
}

/* ===================================================================== */
/* EXAMPLE 5: LED DRIVERS                                                */
/* ===================================================================== */

void example_led_drivers(void)
{
    max20303_handle_t pmic;
    max20303_led_config_t led_cfg;
    int ret;
    
    printf("\n=== Example 5: LED Drivers ===\n");
    
    // Initialize
    pmic.i2c_addr = MAX20303_I2C_ADDR_7BIT;
    pmic.fg_i2c_addr = MAX20303_FG_I2C_ADDR_7BIT;
    pmic.i2c_write = platform_i2c_write;
    pmic.i2c_read = platform_i2c_read;
    pmic.delay_ms = platform_delay_ms;
    max20303_init(&pmic);
    
    // Configure LED0 with 10mA for indicator LED
    led_cfg.led_id = 0;
    led_cfg.current_ua = 10000; // 10mA
    led_cfg.enabled = true;
    
    ret = max20303_led_config(&pmic, &led_cfg);
    if (ret == 0) {
        printf("LED0 configured: 10mA\n");
    }
    
    // Configure LED1 with 20mA for backlight
    led_cfg.led_id = 1;
    led_cfg.current_ua = 20000; // 20mA
    
    ret = max20303_led_config(&pmic, &led_cfg);
    if (ret == 0) {
        printf("LED1 configured: 20mA\n");
    }
    
    // Adjust LED brightness by changing current
    ret = max20303_led_set_current(&pmic, 0, 5000); // Dim to 5mA
    if (ret == 0) {
        printf("LED0 dimmed to 5mA\n");
    }
    
    // Turn off LED by setting current to 0
    ret = max20303_led_set_current(&pmic, 0, 0);
    if (ret == 0) {
        printf("LED0 turned off\n");
    }
}

/* ===================================================================== */
/* EXAMPLE 6: BATTERY CHARGER                                            */
/* ===================================================================== */

void example_battery_charger(void)
{
    max20303_handle_t pmic;
    max20303_charger_config_t chg_cfg;
    int ret;
    
    printf("\n=== Example 6: Battery Charger ===\n");
    
    // Initialize
    pmic.i2c_addr = MAX20303_I2C_ADDR_7BIT;
    pmic.fg_i2c_addr = MAX20303_FG_I2C_ADDR_7BIT;
    pmic.i2c_write = platform_i2c_write;
    pmic.i2c_read = platform_i2c_read;
    pmic.delay_ms = platform_delay_ms;
    max20303_init(&pmic);
    
    // Configure charger for 100mA fast charge
    chg_cfg.fast_charge_ma = 100;
    chg_cfg.voltage_mv = 4200; // Li-ion full charge voltage
    chg_cfg.input_current_limit_ma = 500;
    chg_cfg.enabled = true;
    
    ret = max20303_charger_config(&pmic, &chg_cfg);
    if (ret == 0) {
        printf("Charger configured: 100mA, 4.2V, enabled\n");
    }
    
    // Adjust charge current based on battery temperature
    ret = max20303_charger_set_current(&pmic, 50); // Reduce to 50mA
    if (ret == 0) {
        printf("Charge current reduced to 50mA\n");
    }
    
    // Disable charger
    ret = max20303_charger_enable(&pmic, false);
    if (ret == 0) {
        printf("Charger disabled\n");
    }
}

/* ===================================================================== */
/* EXAMPLE 7: FUEL GAUGE - READ BATTERY STATUS                           */
/* ===================================================================== */

void example_fuel_gauge(void)
{
    max20303_handle_t pmic;
    max20303_battery_status_t batt_status;
    uint16_t voltage_mv;
    uint8_t soc_percent;
    int ret;
    
    printf("\n=== Example 7: Fuel Gauge ===\n");
    
    // Initialize
    pmic.i2c_addr = MAX20303_I2C_ADDR_7BIT;
    pmic.fg_i2c_addr = MAX20303_FG_I2C_ADDR_7BIT;
    pmic.i2c_write = platform_i2c_write;
    pmic.i2c_read = platform_i2c_read;
    pmic.delay_ms = platform_delay_ms;
    max20303_init(&pmic);
    
    // Read complete battery status
    ret = max20303_fg_read_battery_status(&pmic, &batt_status);
    if (ret == 0) {
        printf("Battery Status:\n");
        printf("  Voltage: %u mV\n", batt_status.voltage_mv);
        printf("  SOC: %u%%\n", batt_status.soc_percent);
        printf("  Charging: %s\n", batt_status.charging ? "Yes" : "No");
    }
    
    // Read individual parameters
    ret = max20303_fg_read_voltage(&pmic, &voltage_mv);
    if (ret == 0) {
        printf("Battery voltage: %u mV\n", voltage_mv);
    }
    
    ret = max20303_fg_read_soc(&pmic, &soc_percent);
    if (ret == 0) {
        printf("Battery SOC: %u%%\n", soc_percent);
    }
    
    // Check if battery is low
    if (soc_percent < 20) {
        printf("WARNING: Battery low! (%u%%)\n", soc_percent);
    }
}

/* ===================================================================== */
/* EXAMPLE 8: HAPTIC DRIVER                                              */
/* ===================================================================== */

void example_haptic_driver(void)
{
    max20303_handle_t pmic;
    max20303_haptic_config_t haptic_cfg;
    int ret;
    
    printf("\n=== Example 8: Haptic Driver ===\n");
    
    // Initialize
    pmic.i2c_addr = MAX20303_I2C_ADDR_7BIT;
    pmic.fg_i2c_addr = MAX20303_FG_I2C_ADDR_7BIT;
    pmic.i2c_write = platform_i2c_write;
    pmic.i2c_read = platform_i2c_read;
    pmic.delay_ms = platform_delay_ms;
    max20303_init(&pmic);
    
    // Configure for ERM motor
    haptic_cfg.type = MAX20303_HAPTIC_ERM;
    haptic_cfg.drive_strength = 80; // 80% strength
    haptic_cfg.frequency_hz = 0; // Not used for ERM
    haptic_cfg.enabled = true;
    
    ret = max20303_haptic_config(&pmic, &haptic_cfg);
    if (ret == 0) {
        printf("Haptic configured for ERM motor\n");
    }
    
    // Play haptic pattern 1
    ret = max20303_haptic_play(&pmic, 1);
    if (ret == 0) {
        printf("Playing haptic pattern 1\n");
    }
    
    // Wait for haptic to complete
    platform_delay_ms(500);
    
    // Stop haptic
    ret = max20303_haptic_stop(&pmic);
    if (ret == 0) {
        printf("Haptic stopped\n");
    }
    
    // Configure for LRA motor with specific frequency
    haptic_cfg.type = MAX20303_HAPTIC_LRA;
    haptic_cfg.drive_strength = 70;
    haptic_cfg.frequency_hz = 175; // 175Hz resonant frequency
    
    ret = max20303_haptic_config(&pmic, &haptic_cfg);
    if (ret == 0) {
        printf("Haptic reconfigured for LRA motor at 175Hz\n");
    }
}

/* ===================================================================== */
/* EXAMPLE 9: INTERRUPT HANDLING                                         */
/* ===================================================================== */

void example_interrupts(void)
{
    max20303_handle_t pmic;
    uint8_t int0, int1, int2;
    int ret;
    
    printf("\n=== Example 9: Interrupt Handling ===\n");
    
    // Initialize
    pmic.i2c_addr = MAX20303_I2C_ADDR_7BIT;
    pmic.fg_i2c_addr = MAX20303_FG_I2C_ADDR_7BIT;
    pmic.i2c_write = platform_i2c_write;
    pmic.i2c_read = platform_i2c_read;
    pmic.delay_ms = platform_delay_ms;
    max20303_init(&pmic);
    
    // Configure interrupt masks (0 = interrupt enabled)
    ret = max20303_set_interrupt_mask(&pmic, 0xFF, 0x00, 0xFF);
    if (ret == 0) {
        printf("Interrupt masks configured\n");
        printf("  INT0: All disabled\n");
        printf("  INT1: All enabled (charger, input monitoring)\n");
        printf("  INT2: All disabled\n");
    }
    
    // Read interrupt status
    ret = max20303_read_interrupts(&pmic, &int0, &int1, &int2);
    if (ret == 0) {
        printf("Interrupt status read:\n");
        printf("  INT0: 0x%02X\n", int0);
        printf("  INT1: 0x%02X\n", int1);
        printf("  INT2: 0x%02X\n", int2);
        
        // Check specific interrupts
        if (int1 & MAX20303_INT1_CHG) {
            printf("  Charge status changed\n");
        }
        if (int1 & MAX20303_INT1_CHGIN_OK) {
            printf("  CHGIN status changed\n");
        }
        if (int2 & MAX20303_INT2_BUCK1_DROPOUT) {
            printf("  WARNING: Buck1 dropout!\n");
        }
    }
    
    // Clear interrupts (done automatically by reading)
    ret = max20303_clear_interrupts(&pmic);
    if (ret == 0) {
        printf("Interrupts cleared\n");
    }
}

/* ===================================================================== */
/* EXAMPLE 10: COMPLETE POWER MANAGEMENT SYSTEM                          */
/* ===================================================================== */

void example_complete_system(void)
{
    max20303_handle_t pmic;
    max20303_battery_status_t batt_status;
    int ret;
    
    printf("\n=== Example 10: Complete Power Management System ===\n");
    
    // Initialize device
    pmic.i2c_addr = MAX20303_I2C_ADDR_7BIT;
    pmic.fg_i2c_addr = MAX20303_FG_I2C_ADDR_7BIT;
    pmic.i2c_write = platform_i2c_write;
    pmic.i2c_read = platform_i2c_read;
    pmic.delay_ms = platform_delay_ms;
    
    ret = max20303_init(&pmic);
    if (ret != 0) {
        printf("ERROR: Initialization failed!\n");
        return;
    }
    printf("System initialized\n");
    
    // Configure all power rails
    printf("Configuring power rails...\n");
    
    // Buck1: 1.8V for MCU core
    max20303_buck_set_voltage(&pmic, 1, 1800);
    max20303_buck_enable(&pmic, 1, true);
    
    // Buck2: 3.3V for peripherals
    max20303_buck_set_voltage(&pmic, 2, 3300);
    max20303_buck_enable(&pmic, 2, true);
    
    // LDO: 2.8V for sensors
    max20303_ldo_set_voltage(&pmic, false, 2800);
    max20303_ldo_enable(&pmic, false, true);
    
    // Boost: 12V for display (initially off)
    max20303_boost_set_voltage(&pmic, 12000);
    max20303_boost_enable(&pmic, false);
    
    printf("Power rails configured\n");
    
    // Configure charger
    printf("Configuring charger...\n");
    max20303_charger_set_current(&pmic, 100);
    max20303_charger_enable(&pmic, true);
    printf("Charger enabled: 100mA\n");
    
    // Configure LED indicator
    max20303_led_set_current(&pmic, 0, 5000); // 5mA status LED
    printf("Status LED configured\n");
    
    // Monitor battery
    ret = max20303_fg_read_battery_status(&pmic, &batt_status);
    if (ret == 0) {
        printf("\nBattery Monitor:\n");
        printf("  Voltage: %u mV\n", batt_status.voltage_mv);
        printf("  Charge: %u%%\n", batt_status.soc_percent);
        printf("  Status: %s\n", batt_status.charging ? "Charging" : "Discharging");
        
        // Take action based on battery level
        if (batt_status.soc_percent < 10) {
            printf("  ACTION: Low battery - entering power save mode\n");
            // Disable boost
            max20303_boost_enable(&pmic, false);
            // Reduce LED current
            max20303_led_set_current(&pmic, 0, 1000);
        } else if (batt_status.soc_percent > 90) {
            printf("  STATUS: Battery high\n");
        }
    }
    
    printf("\nSystem configured successfully!\n");
}

/* ===================================================================== */
/* MAIN FUNCTION                                                          */
/* ===================================================================== */

int main(void)
{
    printf("\n");
    printf("========================================\n");
    printf("   MAX20303 PMIC Driver Examples\n");
    printf("========================================\n");
    
    // Run all examples
    example_basic_init();
    example_buck_regulators();
    example_ldo_regulators();
    example_boost_regulator();
    example_led_drivers();
    example_battery_charger();
    example_fuel_gauge();
    example_haptic_driver();
    example_interrupts();
    example_complete_system();
    
    printf("\n");
    printf("========================================\n");
    printf("   All examples completed\n");
    printf("========================================\n");
    printf("\n");
    
    return 0;
}
