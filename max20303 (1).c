/**
 * @file max20303.c
 * @brief MAX20303 PMIC Driver Implementation
 * 
 * Implementation of driver functions for MAX20303 PMIC with Ultra Low IQ
 * Voltage Regulators, Battery Charger and Fuel Gauge
 * 
 * @author Generated from MAX20303 Datasheet
 * @date 2025
 */

#include "max20303.h"
#include <string.h>

/* ===================================================================== */
/* PRIVATE MACROS                                                        */
/* ===================================================================== */
#define MAX20303_TIMEOUT_MS     1000
#define MAX20303_AP_TIMEOUT_MS  100

/* AP Response codes */
#define AP_RESPONSE_SUCCESS     0x00
#define AP_RESPONSE_ERROR       0xFF

/* ===================================================================== */
/* PRIVATE FUNCTION PROTOTYPES                                           */
/* ===================================================================== */
static int max20303_wait_ap_ready(max20303_handle_t *dev);
static uint8_t calculate_buck1_reg_value(uint16_t voltage_mv);
static uint8_t calculate_buck2_reg_value(uint16_t voltage_mv);
static uint8_t calculate_ldo_reg_value(uint16_t voltage_mv);
static uint8_t calculate_lvldo_reg_value(uint16_t voltage_mv);
static uint8_t calculate_boost_reg_value(uint16_t voltage_mv);
static uint8_t calculate_buckboost_reg_value(uint16_t voltage_mv);

/* ===================================================================== */
/* LOW-LEVEL I2C FUNCTIONS                                               */
/* ===================================================================== */

int max20303_write_reg(max20303_handle_t *dev, uint8_t reg, uint8_t value)
{
    if (dev == NULL || dev->i2c_write == NULL) {
        return -1;
    }
    
    return dev->i2c_write(dev->i2c_handle, dev->i2c_addr, reg, &value, 1);
}

int max20303_read_reg(max20303_handle_t *dev, uint8_t reg, uint8_t *value)
{
    if (dev == NULL || dev->i2c_read == NULL || value == NULL) {
        return -1;
    }
    
    return dev->i2c_read(dev->i2c_handle, dev->i2c_addr, reg, value, 1);
}

int max20303_write_regs(max20303_handle_t *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (dev == NULL || dev->i2c_write == NULL || data == NULL) {
        return -1;
    }
    
    return dev->i2c_write(dev->i2c_handle, dev->i2c_addr, reg, data, len);
}

int max20303_read_regs(max20303_handle_t *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (dev == NULL || dev->i2c_read == NULL || data == NULL) {
        return -1;
    }
    
    return dev->i2c_read(dev->i2c_handle, dev->i2c_addr, reg, data, len);
}

/* ===================================================================== */
/* FUEL GAUGE I2C FUNCTIONS                                              */
/* ===================================================================== */

static int max20303_fg_write_reg(max20303_handle_t *dev, uint8_t reg, uint16_t value)
{
    uint8_t data[2];
    
    if (dev == NULL || dev->i2c_write == NULL) {
        return -1;
    }
    
    // MAX20303 fuel gauge uses MSB first
    data[0] = (value >> 8) & 0xFF;
    data[1] = value & 0xFF;
    
    return dev->i2c_write(dev->i2c_handle, dev->fg_i2c_addr, reg, data, 2);
}

static int max20303_fg_read_reg(max20303_handle_t *dev, uint8_t reg, uint16_t *value)
{
    uint8_t data[2];
    int ret;
    
    if (dev == NULL || dev->i2c_read == NULL || value == NULL) {
        return -1;
    }
    
    ret = dev->i2c_read(dev->i2c_handle, dev->fg_i2c_addr, reg, data, 2);
    if (ret != 0) {
        return ret;
    }
    
    // MAX20303 fuel gauge uses MSB first
    *value = ((uint16_t)data[0] << 8) | data[1];
    
    return 0;
}

/* ===================================================================== */
/* AP COMMAND INTERFACE                                                  */
/* ===================================================================== */

static int max20303_wait_ap_ready(max20303_handle_t *dev)
{
    uint8_t response;
    int timeout = MAX20303_AP_TIMEOUT_MS;
    int ret;
    
    // Wait for AP to be ready (response register cleared)
    while (timeout > 0) {
        ret = max20303_read_reg(dev, MAX20303_REG_AP_RESPONSE, &response);
        if (ret != 0) {
            return ret;
        }
        
        if (response == AP_RESPONSE_SUCCESS) {
            return 0;
        }
        
        if (dev->delay_ms) {
            dev->delay_ms(1);
        }
        timeout--;
    }
    
    return -2; // Timeout
}

int max20303_send_ap_command(max20303_handle_t *dev, uint8_t cmd, uint8_t *data, uint8_t data_len)
{
    int ret;
    uint8_t response;
    
    if (dev == NULL) {
        return -1;
    }
    
    // Wait for AP to be ready
    ret = max20303_wait_ap_ready(dev);
    if (ret != 0) {
        return ret;
    }
    
    // Write command data if provided
    if (data != NULL && data_len > 0) {
        ret = max20303_write_regs(dev, MAX20303_REG_AP_DATAIN0, data, data_len);
        if (ret != 0) {
            return ret;
        }
    }
    
    // Write command
    ret = max20303_write_reg(dev, MAX20303_REG_AP_CMDOUT, cmd);
    if (ret != 0) {
        return ret;
    }
    
    // Wait for command completion
    ret = max20303_wait_ap_ready(dev);
    if (ret != 0) {
        return ret;
    }
    
    // Check response
    ret = max20303_read_reg(dev, MAX20303_REG_AP_RESPONSE, &response);
    if (ret != 0) {
        return ret;
    }
    
    if (response != AP_RESPONSE_SUCCESS) {
        return -3; // Command error
    }
    
    return 0;
}

/* ===================================================================== */
/* INITIALIZATION FUNCTIONS                                              */
/* ===================================================================== */

int max20303_init(max20303_handle_t *dev)
{
    uint8_t status;
    int ret;
    
    if (dev == NULL) {
        return -1;
    }
    
    // Set default I2C addresses if not set
    if (dev->i2c_addr == 0) {
        dev->i2c_addr = MAX20303_I2C_ADDR_7BIT;
    }
    if (dev->fg_i2c_addr == 0) {
        dev->fg_i2c_addr = MAX20303_FG_I2C_ADDR_7BIT;
    }
    
    // Verify device communication by reading status register
    ret = max20303_read_reg(dev, MAX20303_REG_STATUS0, &status);
    if (ret != 0) {
        return ret;
    }
    
    // Clear any pending interrupts
    ret = max20303_clear_interrupts(dev);
    if (ret != 0) {
        return ret;
    }
    
    return 0;
}

int max20303_reset(max20303_handle_t *dev)
{
    // Implement soft reset if available in AP commands
    // This would require specific command from datasheet
    return 0;
}

int max20303_get_version(max20303_handle_t *dev, uint16_t *version)
{
    if (dev == NULL || version == NULL) {
        return -1;
    }
    
    // Read fuel gauge version register
    return max20303_fg_read_reg(dev, MAX20303_FG_REG_VERSION, version);
}

/* ===================================================================== */
/* BUCK REGULATOR FUNCTIONS                                              */
/* ===================================================================== */

static uint8_t calculate_buck1_reg_value(uint16_t voltage_mv)
{
    // Buck1: 0.8V to 2.375V in 25mV steps
    if (voltage_mv < MAX20303_BUCK1_VOUT_MIN_MV) {
        voltage_mv = MAX20303_BUCK1_VOUT_MIN_MV;
    }
    if (voltage_mv > MAX20303_BUCK1_VOUT_MAX_MV) {
        voltage_mv = MAX20303_BUCK1_VOUT_MAX_MV;
    }
    
    return (uint8_t)((voltage_mv - MAX20303_BUCK1_VOUT_MIN_MV) / MAX20303_BUCK1_VOUT_STEP_MV);
}

static uint8_t calculate_buck2_reg_value(uint16_t voltage_mv)
{
    // Buck2: 0.8V to 3.95V in 50mV steps
    if (voltage_mv < MAX20303_BUCK2_VOUT_MIN_MV) {
        voltage_mv = MAX20303_BUCK2_VOUT_MIN_MV;
    }
    if (voltage_mv > MAX20303_BUCK2_VOUT_MAX_MV) {
        voltage_mv = MAX20303_BUCK2_VOUT_MAX_MV;
    }
    
    return (uint8_t)((voltage_mv - MAX20303_BUCK2_VOUT_MIN_MV) / MAX20303_BUCK2_VOUT_STEP_MV);
}

int max20303_buck_config(max20303_handle_t *dev, const max20303_buck_config_t *config)
{
    int ret;
    
    if (dev == NULL || config == NULL) {
        return -1;
    }
    
    // Set voltage
    ret = max20303_buck_set_voltage(dev, config->regulator_id, config->voltage_mv);
    if (ret != 0) {
        return ret;
    }
    
    // Enable/disable
    return max20303_buck_enable(dev, config->regulator_id, config->enabled);
}

int max20303_buck_enable(max20303_handle_t *dev, uint8_t buck_id, bool enable)
{
    uint8_t cmd;
    uint8_t data;
    
    if (dev == NULL || (buck_id != 1 && buck_id != 2)) {
        return -1;
    }
    
    cmd = (buck_id == 1) ? MAX20303_CMD_BUCK1EN : MAX20303_CMD_BUCK2EN;
    data = enable ? 0x01 : 0x00;
    
    return max20303_send_ap_command(dev, cmd, &data, 1);
}

int max20303_buck_set_voltage(max20303_handle_t *dev, uint8_t buck_id, uint16_t voltage_mv)
{
    uint8_t cmd;
    uint8_t reg_val;
    
    if (dev == NULL || (buck_id != 1 && buck_id != 2)) {
        return -1;
    }
    
    if (buck_id == 1) {
        cmd = MAX20303_CMD_BUCK1SET;
        reg_val = calculate_buck1_reg_value(voltage_mv);
    } else {
        cmd = MAX20303_CMD_BUCK2SET;
        reg_val = calculate_buck2_reg_value(voltage_mv);
    }
    
    return max20303_send_ap_command(dev, cmd, &reg_val, 1);
}

/* ===================================================================== */
/* LDO REGULATOR FUNCTIONS                                               */
/* ===================================================================== */

static uint8_t calculate_ldo_reg_value(uint16_t voltage_mv)
{
    // LDO: 0.9V to 4.0V in 100mV steps
    if (voltage_mv < MAX20303_LDO_VOUT_MIN_MV) {
        voltage_mv = MAX20303_LDO_VOUT_MIN_MV;
    }
    if (voltage_mv > MAX20303_LDO_VOUT_MAX_MV) {
        voltage_mv = MAX20303_LDO_VOUT_MAX_MV;
    }
    
    return (uint8_t)((voltage_mv - MAX20303_LDO_VOUT_MIN_MV) / MAX20303_LDO_VOUT_STEP_MV);
}

static uint8_t calculate_lvldo_reg_value(uint16_t voltage_mv)
{
    // LV-LDO: 0.5V to 1.95V in 25mV steps
    if (voltage_mv < MAX20303_LVLDO_VOUT_MIN_MV) {
        voltage_mv = MAX20303_LVLDO_VOUT_MIN_MV;
    }
    if (voltage_mv > MAX20303_LVLDO_VOUT_MAX_MV) {
        voltage_mv = MAX20303_LVLDO_VOUT_MAX_MV;
    }
    
    return (uint8_t)((voltage_mv - MAX20303_LVLDO_VOUT_MIN_MV) / MAX20303_LVLDO_VOUT_STEP_MV);
}

int max20303_ldo_config(max20303_handle_t *dev, const max20303_ldo_config_t *config)
{
    int ret;
    
    if (dev == NULL || config == NULL) {
        return -1;
    }
    
    // Set voltage
    ret = max20303_ldo_set_voltage(dev, config->is_lv_ldo, config->voltage_mv);
    if (ret != 0) {
        return ret;
    }
    
    // Enable/disable
    return max20303_ldo_enable(dev, config->is_lv_ldo, config->enabled);
}

int max20303_ldo_enable(max20303_handle_t *dev, bool is_lv_ldo, bool enable)
{
    uint8_t cmd;
    uint8_t data;
    
    if (dev == NULL) {
        return -1;
    }
    
    cmd = is_lv_ldo ? MAX20303_CMD_LVLDOEN : MAX20303_CMD_LDOEN;
    data = enable ? 0x01 : 0x00;
    
    return max20303_send_ap_command(dev, cmd, &data, 1);
}

int max20303_ldo_set_voltage(max20303_handle_t *dev, bool is_lv_ldo, uint16_t voltage_mv)
{
    uint8_t cmd;
    uint8_t reg_val;
    
    if (dev == NULL) {
        return -1;
    }
    
    if (is_lv_ldo) {
        cmd = MAX20303_CMD_LVLDOSET;
        reg_val = calculate_lvldo_reg_value(voltage_mv);
    } else {
        cmd = MAX20303_CMD_LDOSET;
        reg_val = calculate_ldo_reg_value(voltage_mv);
    }
    
    return max20303_send_ap_command(dev, cmd, &reg_val, 1);
}

/* ===================================================================== */
/* BOOST REGULATOR FUNCTIONS                                             */
/* ===================================================================== */

static uint8_t calculate_boost_reg_value(uint16_t voltage_mv)
{
    // Boost: 5V to 20V in 250mV steps
    if (voltage_mv < MAX20303_BOOST_VOUT_MIN_MV) {
        voltage_mv = MAX20303_BOOST_VOUT_MIN_MV;
    }
    if (voltage_mv > MAX20303_BOOST_VOUT_MAX_MV) {
        voltage_mv = MAX20303_BOOST_VOUT_MAX_MV;
    }
    
    return (uint8_t)((voltage_mv - MAX20303_BOOST_VOUT_MIN_MV) / MAX20303_BOOST_VOUT_STEP_MV);
}

int max20303_boost_config(max20303_handle_t *dev, const max20303_boost_config_t *config)
{
    int ret;
    
    if (dev == NULL || config == NULL) {
        return -1;
    }
    
    // Set voltage
    ret = max20303_boost_set_voltage(dev, config->voltage_mv);
    if (ret != 0) {
        return ret;
    }
    
    // Enable/disable
    return max20303_boost_enable(dev, config->enabled);
}

int max20303_boost_enable(max20303_handle_t *dev, bool enable)
{
    uint8_t data;
    
    if (dev == NULL) {
        return -1;
    }
    
    data = enable ? 0x01 : 0x00;
    return max20303_send_ap_command(dev, MAX20303_CMD_BOOSTEN, &data, 1);
}

int max20303_boost_set_voltage(max20303_handle_t *dev, uint16_t voltage_mv)
{
    uint8_t reg_val;
    
    if (dev == NULL) {
        return -1;
    }
    
    reg_val = calculate_boost_reg_value(voltage_mv);
    return max20303_send_ap_command(dev, MAX20303_CMD_BOOSTSET, &reg_val, 1);
}

/* ===================================================================== */
/* BUCK-BOOST REGULATOR FUNCTIONS                                        */
/* ===================================================================== */

static uint8_t calculate_buckboost_reg_value(uint16_t voltage_mv)
{
    // Buck-Boost: 2.5V to 5.0V in 100mV steps
    if (voltage_mv < MAX20303_BUCKBOOST_VOUT_MIN_MV) {
        voltage_mv = MAX20303_BUCKBOOST_VOUT_MIN_MV;
    }
    if (voltage_mv > MAX20303_BUCKBOOST_VOUT_MAX_MV) {
        voltage_mv = MAX20303_BUCKBOOST_VOUT_MAX_MV;
    }
    
    return (uint8_t)((voltage_mv - MAX20303_BUCKBOOST_VOUT_MIN_MV) / MAX20303_BUCKBOOST_VOUT_STEP_MV);
}

int max20303_buckboost_config(max20303_handle_t *dev, const max20303_buckboost_config_t *config)
{
    int ret;
    uint8_t reg_val;
    
    if (dev == NULL || config == NULL) {
        return -1;
    }
    
    // Set voltage
    reg_val = calculate_buckboost_reg_value(config->voltage_mv);
    ret = max20303_send_ap_command(dev, MAX20303_CMD_BUCKBOOSTSET, &reg_val, 1);
    if (ret != 0) {
        return ret;
    }
    
    // Enable/disable
    return max20303_buckboost_enable(dev, config->enabled);
}

int max20303_buckboost_enable(max20303_handle_t *dev, bool enable)
{
    uint8_t data;
    
    if (dev == NULL) {
        return -1;
    }
    
    data = enable ? 0x01 : 0x00;
    return max20303_send_ap_command(dev, MAX20303_CMD_BUCKBOOSTEN, &data, 1);
}

/* ===================================================================== */
/* LED DRIVER FUNCTIONS                                                  */
/* ===================================================================== */

int max20303_led_config(max20303_handle_t *dev, const max20303_led_config_t *config)
{
    int ret;
    
    if (dev == NULL || config == NULL || config->led_id > 2) {
        return -1;
    }
    
    // Set LED current
    ret = max20303_led_set_current(dev, config->led_id, config->current_ua);
    if (ret != 0) {
        return ret;
    }
    
    // Note: Enable is typically controlled through the current setting
    // Setting current to 0 effectively disables the LED
    
    return 0;
}

int max20303_led_set_current(max20303_handle_t *dev, uint8_t led_id, uint16_t current_ua)
{
    uint8_t cmd;
    uint8_t reg_val;
    
    if (dev == NULL || led_id > 2) {
        return -1;
    }
    
    // Clamp current to valid range
    if (current_ua < MAX20303_LED_CURRENT_MIN_UA) {
        current_ua = 0; // Disable
    }
    if (current_ua > MAX20303_LED_CURRENT_MAX_UA) {
        current_ua = MAX20303_LED_CURRENT_MAX_UA;
    }
    
    // Calculate register value (implementation depends on datasheet details)
    // This is a simplified calculation - adjust based on actual register mapping
    reg_val = (uint8_t)(current_ua / 100); // Example: 100uA steps
    
    // Select command based on LED ID
    switch (led_id) {
        case 0:
            cmd = MAX20303_CMD_LED0SET;
            break;
        case 1:
            cmd = MAX20303_CMD_LED1SET;
            break;
        case 2:
            cmd = MAX20303_CMD_LED2SET;
            break;
        default:
            return -1;
    }
    
    return max20303_send_ap_command(dev, cmd, &reg_val, 1);
}

/* ===================================================================== */
/* BATTERY CHARGER FUNCTIONS                                             */
/* ===================================================================== */

int max20303_charger_config(max20303_handle_t *dev, const max20303_charger_config_t *config)
{
    int ret;
    uint8_t data[4];
    
    if (dev == NULL || config == NULL) {
        return -1;
    }
    
    // Set fast charge current
    ret = max20303_charger_set_current(dev, config->fast_charge_ma);
    if (ret != 0) {
        return ret;
    }
    
    // Set input current limit
    if (config->input_current_limit_ma > 0) {
        uint8_t ilim = (uint8_t)(config->input_current_limit_ma / 10); // Example scale
        ret = max20303_send_ap_command(dev, MAX20303_CMD_ILIMSET, &ilim, 1);
        if (ret != 0) {
            return ret;
        }
    }
    
    // Enable/disable charger
    return max20303_charger_enable(dev, config->enabled);
}

int max20303_charger_enable(max20303_handle_t *dev, bool enable)
{
    uint8_t data;
    
    if (dev == NULL) {
        return -1;
    }
    
    data = enable ? 0x01 : 0x00;
    return max20303_send_ap_command(dev, MAX20303_CMD_CHGEN, &data, 1);
}

int max20303_charger_set_current(max20303_handle_t *dev, uint16_t current_ma)
{
    uint8_t reg_val;
    
    if (dev == NULL) {
        return -1;
    }
    
    // Clamp current to valid range
    if (current_ma < MAX20303_CHG_CURRENT_MIN_MA) {
        current_ma = MAX20303_CHG_CURRENT_MIN_MA;
    }
    if (current_ma > MAX20303_CHG_CURRENT_MAX_MA) {
        current_ma = MAX20303_CHG_CURRENT_MAX_MA;
    }
    
    // Calculate register value (simplified - adjust based on datasheet)
    reg_val = (uint8_t)(current_ma / 5); // Example: 5mA steps
    
    return max20303_send_ap_command(dev, MAX20303_CMD_CHGCFG, &reg_val, 1);
}

/* ===================================================================== */
/* FUEL GAUGE FUNCTIONS                                                  */
/* ===================================================================== */

int max20303_fg_read_battery_status(max20303_handle_t *dev, max20303_battery_status_t *status)
{
    int ret;
    uint16_t vcell, soc_raw;
    
    if (dev == NULL || status == NULL) {
        return -1;
    }
    
    memset(status, 0, sizeof(max20303_battery_status_t));
    
    // Read battery voltage
    ret = max20303_fg_read_reg(dev, MAX20303_FG_REG_VCELL, &vcell);
    if (ret != 0) {
        return ret;
    }
    
    // Convert VCELL register to voltage in mV
    // VCELL LSB = 1.25mV/cell (for single cell)
    status->voltage_mv = (uint16_t)((vcell * 125) / 100);
    
    // Read state of charge
    ret = max20303_fg_read_reg(dev, MAX20303_FG_REG_SOC, &soc_raw);
    if (ret != 0) {
        return ret;
    }
    
    // Convert SOC register to percentage
    // SOC LSB = 1/256 %
    status->soc_percent = (uint8_t)(soc_raw >> 8);
    
    // Read status to determine if charging
    ret = max20303_read_reg(dev, MAX20303_REG_STATUS1, &soc_raw);
    if (ret == 0) {
        status->charging = (soc_raw & MAX20303_INT1_CHG) != 0;
    }
    
    return 0;
}

int max20303_fg_read_voltage(max20303_handle_t *dev, uint16_t *voltage_mv)
{
    uint16_t vcell;
    int ret;
    
    if (dev == NULL || voltage_mv == NULL) {
        return -1;
    }
    
    ret = max20303_fg_read_reg(dev, MAX20303_FG_REG_VCELL, &vcell);
    if (ret != 0) {
        return ret;
    }
    
    // Convert VCELL register to voltage in mV
    // VCELL LSB = 1.25mV/cell
    *voltage_mv = (uint16_t)((vcell * 125) / 100);
    
    return 0;
}

int max20303_fg_read_soc(max20303_handle_t *dev, uint8_t *soc_percent)
{
    uint16_t soc_raw;
    int ret;
    
    if (dev == NULL || soc_percent == NULL) {
        return -1;
    }
    
    ret = max20303_fg_read_reg(dev, MAX20303_FG_REG_SOC, &soc_raw);
    if (ret != 0) {
        return ret;
    }
    
    // Convert SOC register to percentage
    // SOC LSB = 1/256 %
    *soc_percent = (uint8_t)(soc_raw >> 8);
    
    return 0;
}

/* ===================================================================== */
/* HAPTIC DRIVER FUNCTIONS                                               */
/* ===================================================================== */

int max20303_haptic_config(max20303_handle_t *dev, const max20303_haptic_config_t *config)
{
    uint8_t data[4];
    
    if (dev == NULL || config == NULL) {
        return -1;
    }
    
    // Configure haptic parameters
    data[0] = (uint8_t)config->type;
    data[1] = config->drive_strength;
    data[2] = (config->frequency_hz >> 8) & 0xFF;
    data[3] = config->frequency_hz & 0xFF;
    
    return max20303_send_ap_command(dev, MAX20303_CMD_HPTSET, data, 4);
}

int max20303_haptic_play(max20303_handle_t *dev, uint8_t pattern_id)
{
    if (dev == NULL) {
        return -1;
    }
    
    return max20303_send_ap_command(dev, MAX20303_CMD_HPTPLAY, &pattern_id, 1);
}

int max20303_haptic_stop(max20303_handle_t *dev)
{
    uint8_t data = 0;
    
    if (dev == NULL) {
        return -1;
    }
    
    return max20303_send_ap_command(dev, MAX20303_CMD_HPTEN, &data, 1);
}

/* ===================================================================== */
/* INTERRUPT FUNCTIONS                                                   */
/* ===================================================================== */

int max20303_read_interrupts(max20303_handle_t *dev, uint8_t *int0, uint8_t *int1, uint8_t *int2)
{
    int ret;
    
    if (dev == NULL) {
        return -1;
    }
    
    if (int0 != NULL) {
        ret = max20303_read_reg(dev, MAX20303_REG_INT0, int0);
        if (ret != 0) {
            return ret;
        }
    }
    
    if (int1 != NULL) {
        ret = max20303_read_reg(dev, MAX20303_REG_INT1, int1);
        if (ret != 0) {
            return ret;
        }
    }
    
    if (int2 != NULL) {
        ret = max20303_read_reg(dev, MAX20303_REG_INT2, int2);
        if (ret != 0) {
            return ret;
        }
    }
    
    return 0;
}

int max20303_clear_interrupts(max20303_handle_t *dev)
{
    uint8_t dummy;
    
    if (dev == NULL) {
        return -1;
    }
    
    // Reading interrupt registers clears them
    return max20303_read_interrupts(dev, &dummy, &dummy, &dummy);
}

int max20303_set_interrupt_mask(max20303_handle_t *dev, uint8_t mask0, uint8_t mask1, uint8_t mask2)
{
    int ret;
    
    if (dev == NULL) {
        return -1;
    }
    
    ret = max20303_write_reg(dev, MAX20303_REG_INT_MASK0, mask0);
    if (ret != 0) {
        return ret;
    }
    
    ret = max20303_write_reg(dev, MAX20303_REG_INT_MASK1, mask1);
    if (ret != 0) {
        return ret;
    }
    
    ret = max20303_write_reg(dev, MAX20303_REG_INT_MASK2, mask2);
    if (ret != 0) {
        return ret;
    }
    
    return 0;
}
