/**
 * @file max20303.h
 * @brief MAX20303 PMIC Driver Header File
 * 
 * MAX20303 PMIC with Ultra Low IQ Voltage Regulators,
 * Battery Charger and Fuel Gauge for Small Lithium Ion Systems
 * 
 * Features:
 * - 2x Micro-IQ Buck Regulators (<1µA IQ)
 * - Micro-IQ LDOs and Load Switches
 * - Buck-Boost Regulator
 * - Li+ Battery Charger (5mA to 500mA)
 * - Fuel Gauge
 * - Haptic Driver (ERM/LRA)
 * - 3 Channel LED Current Sinks
 * - Boost Regulator
 * - I2C Interface
 * 
 * @author Generated from MAX20303 Datasheet
 * @date 2025
 */

#ifndef MAX20303_H
#define MAX20303_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===================================================================== */
/* I2C ADDRESSES                                                         */
/* ===================================================================== */
#define MAX20303_I2C_ADDR_WRITE         0x50    // 7-bit address 0b0101000
#define MAX20303_I2C_ADDR_READ          0x51
#define MAX20303_I2C_ADDR_7BIT          0x28    // 7-bit format (0x50 >> 1)

#define MAX20303_FG_I2C_ADDR_WRITE      0x6C    // Fuel Gauge 7-bit: 0b0110110
#define MAX20303_FG_I2C_ADDR_READ       0x6D
#define MAX20303_FG_I2C_ADDR_7BIT       0x36    // 7-bit format (0x6C >> 1)

/* ===================================================================== */
/* DIRECT ACCESS I2C REGISTERS - INTERRUPT & STATUS                      */
/* ===================================================================== */
#define MAX20303_REG_INT0               0x00    // Interrupt 0
#define MAX20303_REG_INT1               0x01    // Interrupt 1
#define MAX20303_REG_INT2               0x02    // Interrupt 2
#define MAX20303_REG_STATUS0            0x03    // Status 0
#define MAX20303_REG_STATUS1            0x04    // Status 1
#define MAX20303_REG_STATUS2            0x05    // Status 2
#define MAX20303_REG_STATUS3            0x06    // Status 3
#define MAX20303_REG_SYSTEM_ERROR       0x07    // System Error Status
#define MAX20303_REG_INT_MASK0          0x08    // Interrupt Mask 0
#define MAX20303_REG_INT_MASK1          0x09    // Interrupt Mask 1
#define MAX20303_REG_INT_MASK2          0x0A    // Interrupt Mask 2
#define MAX20303_REG_AP_DATOUT0         0x0B    // AP Data Out 0
#define MAX20303_REG_AP_DATOUT1         0x0C    // AP Data Out 1
#define MAX20303_REG_AP_DATOUT2         0x0D    // AP Data Out 2
#define MAX20303_REG_AP_DATOUT3         0x0E    // AP Data Out 3
#define MAX20303_REG_AP_DATOUT4         0x0F    // AP Data Out 4
#define MAX20303_REG_AP_DATOUT5         0x10    // AP Data Out 5
#define MAX20303_REG_AP_DATOUT6         0x11    // AP Data Out 6

/* ===================================================================== */
/* DIRECT ACCESS I2C REGISTERS - AP INTERFACE                            */
/* ===================================================================== */
#define MAX20303_REG_AP_CMDOUT          0x17    // AP Command Out
#define MAX20303_REG_AP_RESPONSE        0x18    // AP Response
#define MAX20303_REG_AP_DATAIN0         0x19    // AP Data In 0
#define MAX20303_REG_AP_DATAIN1         0x1A    // AP Data In 1
#define MAX20303_REG_AP_DATAIN2         0x1B    // AP Data In 2
#define MAX20303_REG_AP_DATAIN3         0x1C    // AP Data In 3
#define MAX20303_REG_AP_DATAIN4         0x1D    // AP Data In 4
#define MAX20303_REG_AP_DATAIN5         0x1E    // AP Data In 5

/* ===================================================================== */
/* DIRECT ACCESS I2C REGISTERS - POWER CONTROL                           */
/* ===================================================================== */
#define MAX20303_REG_LDO_DIRECT         0x20    // LDO Direct Control
#define MAX20303_REG_MPC_DIRECTWRITE    0x21    // MPC Direct Write
#define MAX20303_REG_MPC_DIRECTRED      0x22    // MPC Direct Read

/* ===================================================================== */
/* DIRECT ACCESS I2C REGISTERS - HAPTIC                                  */
/* ===================================================================== */
#define MAX20303_REG_HPT_RAM_ADDR       0x28    // Haptic RAM Address
#define MAX20303_REG_HPT_RAM_DATA       0x29    // Haptic RAM Data
#define MAX20303_REG_HPT_RAM_WRITE      0x2A    // Haptic RAM Write

/* ===================================================================== */
/* DIRECT ACCESS I2C REGISTERS - LED                                     */
/* ===================================================================== */
#define MAX20303_REG_LED_STEP_DIRECT    0x2C    // LED Step Direct
#define MAX20303_REG_LED0_DIRECT        0x2D    // LED 0 Direct
#define MAX20303_REG_LED1_DIRECT        0x2E    // LED 1 Direct
#define MAX20303_REG_LED2_DIRECT        0x2F    // LED 2 Direct

/* ===================================================================== */
/* DIRECT ACCESS I2C REGISTERS - HAPTIC CONTROL                          */
/* ===================================================================== */
#define MAX20303_REG_HPT_DIRECT0        0x30    // Haptic Direct 0
#define MAX20303_REG_HPT_DIRECT1        0x31    // Haptic Direct 1
#define MAX20303_REG_HPT_DIRECT2        0x32    // Haptic Direct 2
#define MAX20303_REG_HPT_DIRECT3        0x33    // Haptic Direct 3

/* ===================================================================== */
/* FUEL GAUGE REGISTERS                                                  */
/* ===================================================================== */
#define MAX20303_FG_REG_VCELL           0x02    // Battery voltage
#define MAX20303_FG_REG_SOC             0x04    // State of charge
#define MAX20303_FG_REG_MODE            0x06    // Mode register
#define MAX20303_FG_REG_VERSION         0x08    // Version
#define MAX20303_FG_REG_HIBRT           0x0A    // Hibernate
#define MAX20303_FG_REG_CONFIG          0x0C    // Configuration
#define MAX20303_FG_REG_VALRT           0x14    // Voltage alert threshold
#define MAX20303_FG_REG_CRATE           0x16    // C-rate
#define MAX20303_FG_REG_VRESET_ID       0x18    // Reset voltage
#define MAX20303_FG_REG_STATUS          0x1A    // Status
#define MAX20303_FG_REG_TABLE           0x40    // Table access

/* ===================================================================== */
/* AP COMMAND OPCODES                                                    */
/* ===================================================================== */
// Power Management Commands
#define MAX20303_CMD_BOOSTSET           0x00    // Set boost voltage
#define MAX20303_CMD_BOOSTEN            0x01    // Enable boost
#define MAX20303_CMD_BUCK1SET           0x02    // Set buck1 voltage
#define MAX20303_CMD_BUCK1EN            0x03    // Enable buck1
#define MAX20303_CMD_BUCK2SET           0x04    // Set buck2 voltage
#define MAX20303_CMD_BUCK2EN            0x05    // Enable buck2
#define MAX20303_CMD_LDOSET             0x06    // Set LDO voltage
#define MAX20303_CMD_LDOEN              0x07    // Enable LDO
#define MAX20303_CMD_LVLDOSET           0x08    // Set LV LDO voltage
#define MAX20303_CMD_LVLDOEN            0x09    // Enable LV LDO
#define MAX20303_CMD_BUCKBOOSTSET       0x0A    // Set buck-boost voltage
#define MAX20303_CMD_BUCKBOOSTEN        0x0B    // Enable buck-boost
#define MAX20303_CMD_SBOUTEN            0x0C    // Enable safe output

// LED Commands
#define MAX20303_CMD_LED0SET            0x10    // Set LED0 current
#define MAX20303_CMD_LED1SET            0x11    // Set LED1 current
#define MAX20303_CMD_LED2SET            0x12    // Set LED2 current
#define MAX20303_CMD_LEDSTEP            0x13    // Set LED step/ramp

// Charger Commands
#define MAX20303_CMD_CHGCFG             0x20    // Charger configuration
#define MAX20303_CMD_CHGEN              0x21    // Enable charger
#define MAX20303_CMD_ILIMSET            0x22    // Set input current limit

// ADC/Monitor Commands
#define MAX20303_CMD_MONSET             0x30    // Set monitor mux
#define MAX20303_CMD_ADCSTART           0x31    // Start ADC conversion
#define MAX20303_CMD_ADCREAD            0x32    // Read ADC value

// Haptic Commands
#define MAX20303_CMD_HPTEN              0x40    // Enable haptic
#define MAX20303_CMD_HPTSET             0x41    // Set haptic parameters
#define MAX20303_CMD_HPTPLAY            0x42    // Play haptic pattern

/* ===================================================================== */
/* INTERRUPT BITS - INT0                                                 */
/* ===================================================================== */
#define MAX20303_INT0_GPI0_F            (1 << 0)    // GPI0 falling edge
#define MAX20303_INT0_GPI0_R            (1 << 1)    // GPI0 rising edge
#define MAX20303_INT0_GPI1_F            (1 << 2)    // GPI1 falling edge
#define MAX20303_INT0_GPI1_R            (1 << 3)    // GPI1 rising edge
#define MAX20303_INT0_GPI2_F            (1 << 4)    // GPI2 falling edge
#define MAX20303_INT0_GPI2_R            (1 << 5)    // GPI2 rising edge
#define MAX20303_INT0_NRST              (1 << 6)    // Reset interrupt
#define MAX20303_INT0_THM               (1 << 7)    // Thermal interrupt

/* ===================================================================== */
/* INTERRUPT BITS - INT1                                                 */
/* ===================================================================== */
#define MAX20303_INT1_CHGIN_I           (1 << 0)    // CHGIN interrupt
#define MAX20303_INT1_CHG               (1 << 1)    // Charge status
#define MAX20303_INT1_TJ_REG            (1 << 2)    // Junction temp regulation
#define MAX20303_INT1_CHGIN_OK          (1 << 3)    // CHGIN OK
#define MAX20303_INT1_SYS_CTRL          (1 << 4)    // System control
#define MAX20303_INT1_SYS_CNFG          (1 << 5)    // System config
#define MAX20303_INT1_I_LIMIT           (1 << 6)    // Current limit

/* ===================================================================== */
/* INTERRUPT BITS - INT2                                                 */
/* ===================================================================== */
#define MAX20303_INT2_LDO_DROPOUT       (1 << 0)    // LDO dropout
#define MAX20303_INT2_BUCK1_DROPOUT     (1 << 1)    // Buck1 dropout
#define MAX20303_INT2_BUCK2_DROPOUT     (1 << 2)    // Buck2 dropout

/* ===================================================================== */
/* BUCK VOLTAGE SETTINGS                                                 */
/* ===================================================================== */
// Buck1: 0.8V to 2.375V in 25mV steps
#define MAX20303_BUCK1_VOUT_MIN_MV      800
#define MAX20303_BUCK1_VOUT_MAX_MV      2375
#define MAX20303_BUCK1_VOUT_STEP_MV     25

// Buck2: 0.8V to 3.95V in 50mV steps  
#define MAX20303_BUCK2_VOUT_MIN_MV      800
#define MAX20303_BUCK2_VOUT_MAX_MV      3950
#define MAX20303_BUCK2_VOUT_STEP_MV     50

/* ===================================================================== */
/* LDO VOLTAGE SETTINGS                                                  */
/* ===================================================================== */
// LDO: 0.9V to 4.0V in 100mV steps
#define MAX20303_LDO_VOUT_MIN_MV        900
#define MAX20303_LDO_VOUT_MAX_MV        4000
#define MAX20303_LDO_VOUT_STEP_MV       100

// LV-LDO: 0.5V to 1.95V in 25mV steps
#define MAX20303_LVLDO_VOUT_MIN_MV      500
#define MAX20303_LVLDO_VOUT_MAX_MV      1950
#define MAX20303_LVLDO_VOUT_STEP_MV     25

/* ===================================================================== */
/* BOOST VOLTAGE SETTINGS                                                */
/* ===================================================================== */
// Boost: 5V to 20V in 250mV steps
#define MAX20303_BOOST_VOUT_MIN_MV      5000
#define MAX20303_BOOST_VOUT_MAX_MV      20000
#define MAX20303_BOOST_VOUT_STEP_MV     250

/* ===================================================================== */
/* BUCK-BOOST VOLTAGE SETTINGS                                           */
/* ===================================================================== */
// Buck-Boost: 2.5V to 5.0V in 100mV steps
#define MAX20303_BUCKBOOST_VOUT_MIN_MV  2500
#define MAX20303_BUCKBOOST_VOUT_MAX_MV  5000
#define MAX20303_BUCKBOOST_VOUT_STEP_MV 100

/* ===================================================================== */
/* LED CURRENT SETTINGS                                                  */
/* ===================================================================== */
// LED: 0.6mA to 30mA programmable
#define MAX20303_LED_CURRENT_MIN_UA     600
#define MAX20303_LED_CURRENT_MAX_UA     30000

/* ===================================================================== */
/* CHARGER SETTINGS                                                      */
/* ===================================================================== */
// Fast charge current: 5mA to 500mA
#define MAX20303_CHG_CURRENT_MIN_MA     5
#define MAX20303_CHG_CURRENT_MAX_MA     500

/* ===================================================================== */
/* DATA STRUCTURES                                                       */
/* ===================================================================== */

/**
 * @brief MAX20303 device handle structure
 */
typedef struct {
    uint8_t i2c_addr;           // Primary I2C address (7-bit)
    uint8_t fg_i2c_addr;        // Fuel gauge I2C address (7-bit)
    void *i2c_handle;           // Platform-specific I2C handle
    
    // Function pointers for I2C operations (platform-specific)
    int (*i2c_write)(void *handle, uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len);
    int (*i2c_read)(void *handle, uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len);
    int (*delay_ms)(uint32_t ms);
} max20303_handle_t;

/**
 * @brief Buck regulator configuration
 */
typedef struct {
    uint16_t voltage_mv;        // Output voltage in millivolts
    bool enabled;               // Enable state
    uint8_t regulator_id;       // 1 or 2
} max20303_buck_config_t;

/**
 * @brief LDO configuration
 */
typedef struct {
    uint16_t voltage_mv;        // Output voltage in millivolts
    bool enabled;               // Enable state
    bool is_lv_ldo;             // True for LV-LDO, false for regular LDO
} max20303_ldo_config_t;

/**
 * @brief Boost regulator configuration
 */
typedef struct {
    uint16_t voltage_mv;        // Output voltage in millivolts
    bool enabled;               // Enable state
} max20303_boost_config_t;

/**
 * @brief Buck-boost regulator configuration
 */
typedef struct {
    uint16_t voltage_mv;        // Output voltage in millivolts
    bool enabled;               // Enable state
} max20303_buckboost_config_t;

/**
 * @brief LED configuration
 */
typedef struct {
    uint8_t led_id;             // LED channel (0, 1, or 2)
    uint16_t current_ua;        // Current in microamperes
    bool enabled;               // Enable state
} max20303_led_config_t;

/**
 * @brief Charger configuration
 */
typedef struct {
    uint16_t fast_charge_ma;    // Fast charge current in mA
    uint16_t voltage_mv;        // Charge voltage in mV
    bool enabled;               // Enable state
    uint16_t input_current_limit_ma; // Input current limit
} max20303_charger_config_t;

/**
 * @brief Battery status
 */
typedef struct {
    uint16_t voltage_mv;        // Battery voltage in mV
    uint8_t soc_percent;        // State of charge in %
    int16_t current_ma;         // Battery current in mA (+ charging, - discharging)
    bool charging;              // True if charging
    uint16_t capacity_mah;      // Battery capacity in mAh
} max20303_battery_status_t;

/**
 * @brief Haptic motor type
 */
typedef enum {
    MAX20303_HAPTIC_ERM = 0,    // Eccentric Rotating Mass
    MAX20303_HAPTIC_LRA = 1     // Linear Resonant Actuator
} max20303_haptic_type_t;

/**
 * @brief Haptic configuration
 */
typedef struct {
    max20303_haptic_type_t type; // Motor type
    uint8_t drive_strength;      // Drive strength (0-100)
    uint16_t frequency_hz;       // Frequency for LRA
    bool enabled;                // Enable state
} max20303_haptic_config_t;

/* ===================================================================== */
/* FUNCTION PROTOTYPES - INITIALIZATION                                  */
/* ===================================================================== */

/**
 * @brief Initialize MAX20303 device
 * @param dev Pointer to device handle
 * @return 0 on success, negative error code on failure
 */
int max20303_init(max20303_handle_t *dev);

/**
 * @brief Reset MAX20303 device
 * @param dev Pointer to device handle
 * @return 0 on success, negative error code on failure
 */
int max20303_reset(max20303_handle_t *dev);

/**
 * @brief Read device ID/version
 * @param dev Pointer to device handle
 * @param version Pointer to store version
 * @return 0 on success, negative error code on failure
 */
int max20303_get_version(max20303_handle_t *dev, uint16_t *version);

/* ===================================================================== */
/* FUNCTION PROTOTYPES - BUCK REGULATORS                                 */
/* ===================================================================== */

/**
 * @brief Configure buck regulator
 * @param dev Pointer to device handle
 * @param config Pointer to buck configuration
 * @return 0 on success, negative error code on failure
 */
int max20303_buck_config(max20303_handle_t *dev, const max20303_buck_config_t *config);

/**
 * @brief Enable/disable buck regulator
 * @param dev Pointer to device handle
 * @param buck_id Buck regulator ID (1 or 2)
 * @param enable True to enable, false to disable
 * @return 0 on success, negative error code on failure
 */
int max20303_buck_enable(max20303_handle_t *dev, uint8_t buck_id, bool enable);

/**
 * @brief Set buck regulator voltage
 * @param dev Pointer to device handle
 * @param buck_id Buck regulator ID (1 or 2)
 * @param voltage_mv Output voltage in millivolts
 * @return 0 on success, negative error code on failure
 */
int max20303_buck_set_voltage(max20303_handle_t *dev, uint8_t buck_id, uint16_t voltage_mv);

/* ===================================================================== */
/* FUNCTION PROTOTYPES - LDO REGULATORS                                  */
/* ===================================================================== */

/**
 * @brief Configure LDO regulator
 * @param dev Pointer to device handle
 * @param config Pointer to LDO configuration
 * @return 0 on success, negative error code on failure
 */
int max20303_ldo_config(max20303_handle_t *dev, const max20303_ldo_config_t *config);

/**
 * @brief Enable/disable LDO regulator
 * @param dev Pointer to device handle
 * @param is_lv_ldo True for LV-LDO, false for regular LDO
 * @param enable True to enable, false to disable
 * @return 0 on success, negative error code on failure
 */
int max20303_ldo_enable(max20303_handle_t *dev, bool is_lv_ldo, bool enable);

/**
 * @brief Set LDO voltage
 * @param dev Pointer to device handle
 * @param is_lv_ldo True for LV-LDO, false for regular LDO
 * @param voltage_mv Output voltage in millivolts
 * @return 0 on success, negative error code on failure
 */
int max20303_ldo_set_voltage(max20303_handle_t *dev, bool is_lv_ldo, uint16_t voltage_mv);

/* ===================================================================== */
/* FUNCTION PROTOTYPES - BOOST REGULATOR                                 */
/* ===================================================================== */

/**
 * @brief Configure boost regulator
 * @param dev Pointer to device handle
 * @param config Pointer to boost configuration
 * @return 0 on success, negative error code on failure
 */
int max20303_boost_config(max20303_handle_t *dev, const max20303_boost_config_t *config);

/**
 * @brief Enable/disable boost regulator
 * @param dev Pointer to device handle
 * @param enable True to enable, false to disable
 * @return 0 on success, negative error code on failure
 */
int max20303_boost_enable(max20303_handle_t *dev, bool enable);

/**
 * @brief Set boost voltage
 * @param dev Pointer to device handle
 * @param voltage_mv Output voltage in millivolts
 * @return 0 on success, negative error code on failure
 */
int max20303_boost_set_voltage(max20303_handle_t *dev, uint16_t voltage_mv);

/* ===================================================================== */
/* FUNCTION PROTOTYPES - BUCK-BOOST REGULATOR                            */
/* ===================================================================== */

/**
 * @brief Configure buck-boost regulator
 * @param dev Pointer to device handle
 * @param config Pointer to buck-boost configuration
 * @return 0 on success, negative error code on failure
 */
int max20303_buckboost_config(max20303_handle_t *dev, const max20303_buckboost_config_t *config);

/**
 * @brief Enable/disable buck-boost regulator
 * @param dev Pointer to device handle
 * @param enable True to enable, false to disable
 * @return 0 on success, negative error code on failure
 */
int max20303_buckboost_enable(max20303_handle_t *dev, bool enable);

/* ===================================================================== */
/* FUNCTION PROTOTYPES - LED DRIVERS                                     */
/* ===================================================================== */

/**
 * @brief Configure LED driver
 * @param dev Pointer to device handle
 * @param config Pointer to LED configuration
 * @return 0 on success, negative error code on failure
 */
int max20303_led_config(max20303_handle_t *dev, const max20303_led_config_t *config);

/**
 * @brief Set LED current
 * @param dev Pointer to device handle
 * @param led_id LED channel (0, 1, or 2)
 * @param current_ua Current in microamperes
 * @return 0 on success, negative error code on failure
 */
int max20303_led_set_current(max20303_handle_t *dev, uint8_t led_id, uint16_t current_ua);

/* ===================================================================== */
/* FUNCTION PROTOTYPES - BATTERY CHARGER                                 */
/* ===================================================================== */

/**
 * @brief Configure battery charger
 * @param dev Pointer to device handle
 * @param config Pointer to charger configuration
 * @return 0 on success, negative error code on failure
 */
int max20303_charger_config(max20303_handle_t *dev, const max20303_charger_config_t *config);

/**
 * @brief Enable/disable charger
 * @param dev Pointer to device handle
 * @param enable True to enable, false to disable
 * @return 0 on success, negative error code on failure
 */
int max20303_charger_enable(max20303_handle_t *dev, bool enable);

/**
 * @brief Set fast charge current
 * @param dev Pointer to device handle
 * @param current_ma Charge current in milliamperes
 * @return 0 on success, negative error code on failure
 */
int max20303_charger_set_current(max20303_handle_t *dev, uint16_t current_ma);

/* ===================================================================== */
/* FUNCTION PROTOTYPES - FUEL GAUGE                                      */
/* ===================================================================== */

/**
 * @brief Read battery status from fuel gauge
 * @param dev Pointer to device handle
 * @param status Pointer to store battery status
 * @return 0 on success, negative error code on failure
 */
int max20303_fg_read_battery_status(max20303_handle_t *dev, max20303_battery_status_t *status);

/**
 * @brief Read battery voltage
 * @param dev Pointer to device handle
 * @param voltage_mv Pointer to store voltage in millivolts
 * @return 0 on success, negative error code on failure
 */
int max20303_fg_read_voltage(max20303_handle_t *dev, uint16_t *voltage_mv);

/**
 * @brief Read state of charge
 * @param dev Pointer to device handle
 * @param soc_percent Pointer to store SOC percentage
 * @return 0 on success, negative error code on failure
 */
int max20303_fg_read_soc(max20303_handle_t *dev, uint8_t *soc_percent);

/* ===================================================================== */
/* FUNCTION PROTOTYPES - HAPTIC DRIVER                                   */
/* ===================================================================== */

/**
 * @brief Configure haptic driver
 * @param dev Pointer to device handle
 * @param config Pointer to haptic configuration
 * @return 0 on success, negative error code on failure
 */
int max20303_haptic_config(max20303_handle_t *dev, const max20303_haptic_config_t *config);

/**
 * @brief Play haptic pattern
 * @param dev Pointer to device handle
 * @param pattern_id Pattern ID to play
 * @return 0 on success, negative error code on failure
 */
int max20303_haptic_play(max20303_handle_t *dev, uint8_t pattern_id);

/**
 * @brief Stop haptic playback
 * @param dev Pointer to device handle
 * @return 0 on success, negative error code on failure
 */
int max20303_haptic_stop(max20303_handle_t *dev);

/* ===================================================================== */
/* FUNCTION PROTOTYPES - INTERRUPT MANAGEMENT                            */
/* ===================================================================== */

/**
 * @brief Read interrupt status
 * @param dev Pointer to device handle
 * @param int0 Pointer to store INT0 register value
 * @param int1 Pointer to store INT1 register value
 * @param int2 Pointer to store INT2 register value
 * @return 0 on success, negative error code on failure
 */
int max20303_read_interrupts(max20303_handle_t *dev, uint8_t *int0, uint8_t *int1, uint8_t *int2);

/**
 * @brief Clear interrupt flags
 * @param dev Pointer to device handle
 * @return 0 on success, negative error code on failure
 */
int max20303_clear_interrupts(max20303_handle_t *dev);

/**
 * @brief Configure interrupt masks
 * @param dev Pointer to device handle
 * @param mask0 Mask for INT0
 * @param mask1 Mask for INT1
 * @param mask2 Mask for INT2
 * @return 0 on success, negative error code on failure
 */
int max20303_set_interrupt_mask(max20303_handle_t *dev, uint8_t mask0, uint8_t mask1, uint8_t mask2);

/* ===================================================================== */
/* FUNCTION PROTOTYPES - LOW-LEVEL I2C                                   */
/* ===================================================================== */

/**
 * @brief Write register
 * @param dev Pointer to device handle
 * @param reg Register address
 * @param value Value to write
 * @return 0 on success, negative error code on failure
 */
int max20303_write_reg(max20303_handle_t *dev, uint8_t reg, uint8_t value);

/**
 * @brief Read register
 * @param dev Pointer to device handle
 * @param reg Register address
 * @param value Pointer to store read value
 * @return 0 on success, negative error code on failure
 */
int max20303_read_reg(max20303_handle_t *dev, uint8_t reg, uint8_t *value);

/**
 * @brief Write multiple registers
 * @param dev Pointer to device handle
 * @param reg Starting register address
 * @param data Pointer to data buffer
 * @param len Number of bytes to write
 * @return 0 on success, negative error code on failure
 */
int max20303_write_regs(max20303_handle_t *dev, uint8_t reg, uint8_t *data, uint16_t len);

/**
 * @brief Read multiple registers
 * @param dev Pointer to device handle
 * @param reg Starting register address
 * @param data Pointer to data buffer
 * @param len Number of bytes to read
 * @return 0 on success, negative error code on failure
 */
int max20303_read_regs(max20303_handle_t *dev, uint8_t reg, uint8_t *data, uint16_t len);

/**
 * @brief Send AP command
 * @param dev Pointer to device handle
 * @param cmd Command opcode
 * @param data Pointer to command data (can be NULL)
 * @param data_len Length of command data
 * @return 0 on success, negative error code on failure
 */
int max20303_send_ap_command(max20303_handle_t *dev, uint8_t cmd, uint8_t *data, uint8_t data_len);

#ifdef __cplusplus
}
#endif

#endif /* MAX20303_H */
