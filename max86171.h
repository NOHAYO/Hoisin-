/**
 * @file max86171.h
 * @brief MAX86171 Low-Noise AFE for Pulse Oximeter and Heart Rate Monitor Driver
 * @author Generated from datasheet
 * @date 2025
 * 
 * This driver supports I2C and SPI communication with the MAX86171
 */

#ifndef MAX86171_H
#define MAX86171_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * DEVICE INFORMATION
 * ======================================================================== */
#define MAX86171_PART_ID                0x2C
#define MAX86171_I2C_ADDR_LOW           0xC8  // When ADDR pin is LOW (write)
#define MAX86171_I2C_ADDR_LOW_READ      0xC9  // When ADDR pin is LOW (read)
#define MAX86171_I2C_ADDR_HIGH          0xCA  // When ADDR pin is HIGH (write)
#define MAX86171_I2C_ADDR_HIGH_READ     0xCB  // When ADDR pin is HIGH (read)

/* ========================================================================
 * REGISTER ADDRESSES
 * ======================================================================== */

/* Status Registers */
#define MAX86171_REG_STATUS_1           0x00
#define MAX86171_REG_STATUS_2           0x01
#define MAX86171_REG_STATUS_3           0x02

/* FIFO Registers */
#define MAX86171_REG_FIFO_WR_PTR        0x04
#define MAX86171_REG_FIFO_RD_PTR        0x05
#define MAX86171_REG_FIFO_COUNTER_1     0x06
#define MAX86171_REG_FIFO_COUNTER_2     0x07
#define MAX86171_REG_FIFO_DATA          0x08
#define MAX86171_REG_FIFO_CONFIG_1      0x09
#define MAX86171_REG_FIFO_CONFIG_2      0x0A

/* System Control Registers */
#define MAX86171_REG_SYS_CONFIG_1       0x0C
#define MAX86171_REG_SYS_CONFIG_2       0x0D
#define MAX86171_REG_SYS_CONFIG_3       0x0E
#define MAX86171_REG_PD_BIAS            0x0F
#define MAX86171_REG_PIN_FUNC_CFG       0x10
#define MAX86171_REG_OUTPUT_PIN_CFG     0x11

/* Frame Rate Clock Registers */
#define MAX86171_REG_FR_CLK_FREQ_SEL    0x15
#define MAX86171_REG_FR_CLK_DIV_MSB     0x16
#define MAX86171_REG_FR_CLK_DIV_LSB     0x17

/* Measurement Setup Registers (MEAS1-MEAS9) */
#define MAX86171_REG_MEAS1_SELECT       0x18
#define MAX86171_REG_MEAS1_CONFIG_1     0x19
#define MAX86171_REG_MEAS1_CONFIG_2     0x1A
#define MAX86171_REG_MEAS1_CONFIG_3     0x1B
#define MAX86171_REG_MEAS1_DRVA_CURR    0x1C
#define MAX86171_REG_MEAS1_DRVB_CURR    0x1D
#define MAX86171_REG_MEAS1_DRVC_CURR    0x1E

#define MAX86171_REG_MEAS2_SELECT       0x20
#define MAX86171_REG_MEAS3_SELECT       0x28
#define MAX86171_REG_MEAS4_SELECT       0x30
#define MAX86171_REG_MEAS5_SELECT       0x38
#define MAX86171_REG_MEAS6_SELECT       0x40
#define MAX86171_REG_MEAS7_SELECT       0x48
#define MAX86171_REG_MEAS8_SELECT       0x50
#define MAX86171_REG_MEAS9_SELECT       0x58

/* Threshold Registers */
#define MAX86171_REG_THRESH_MEAS_SEL    0x68
#define MAX86171_REG_THRESH_HYST        0x69
#define MAX86171_REG_THRESH1_HI         0x6A
#define MAX86171_REG_THRESH1_LO         0x6B
#define MAX86171_REG_THRESH2_HI         0x6C
#define MAX86171_REG_THRESH2_LO         0x6D

/* Picket Fence Registers */
#define MAX86171_REG_PF_MEAS_SEL        0x70
#define MAX86171_REG_PF_CONFIG          0x71

/* Interrupt Enable Registers */
#define MAX86171_REG_INT1_ENABLE_1      0x78
#define MAX86171_REG_INT1_ENABLE_2      0x79
#define MAX86171_REG_INT1_ENABLE_3      0x7A
#define MAX86171_REG_INT2_ENABLE_1      0x7C
#define MAX86171_REG_INT2_ENABLE_2      0x7D
#define MAX86171_REG_INT2_ENABLE_3      0x7E

/* Part ID Register */
#define MAX86171_REG_PART_ID            0xFF

/* ========================================================================
 * REGISTER BIT MASKS - STATUS 1 (0x00)
 * ======================================================================== */
#define MAX86171_STATUS1_A_FULL         0x80
#define MAX86171_STATUS1_FRAME_RDY      0x40
#define MAX86171_STATUS1_FIFO_DATA_RDY  0x20
#define MAX86171_STATUS1_ALC_OVF        0x10
#define MAX86171_STATUS1_EXP_OVF        0x08
#define MAX86171_STATUS1_THRESH2_HILO   0x04
#define MAX86171_STATUS1_THRESH1_HILO   0x02
#define MAX86171_STATUS1_PWR_RDY        0x01

/* ========================================================================
 * REGISTER BIT MASKS - STATUS 2 (0x01)
 * ======================================================================== */
#define MAX86171_STATUS2_LED9_COMPB     0x80
#define MAX86171_STATUS2_VDD_OOR        0x40
#define MAX86171_STATUS2_INVALID_CFG    0x04

/* ========================================================================
 * REGISTER BIT MASKS - FIFO CONFIG 2 (0x0A)
 * ======================================================================== */
#define MAX86171_FIFO_FLUSH             0x10
#define MAX86171_FIFO_STAT_CLR          0x08
#define MAX86171_FIFO_A_FULL_TYPE       0x04
#define MAX86171_FIFO_RO                0x02

/* ========================================================================
 * REGISTER BIT MASKS - SYSTEM CONFIG 1 (0x0C)
 * ======================================================================== */
#define MAX86171_MEAS9_EN               0x80
#define MAX86171_SW_FORCE_SYNC          0x40
#define MAX86171_SYNC_MODE_MASK         0x30
#define MAX86171_SYNC_MODE_SHIFT        4
#define MAX86171_PPG2_PWRDN             0x08
#define MAX86171_PPG1_PWRDN             0x04
#define MAX86171_SHDN                   0x02
#define MAX86171_RESET                  0x01

/* ========================================================================
 * REGISTER BIT MASKS - SYSTEM CONFIG 3 (0x0E)
 * ======================================================================== */
#define MAX86171_ALC_DISABLE            0x10
#define MAX86171_COLLECT_RAW_DATA       0x02
#define MAX86171_MEAS1_CONFIG_SEL       0x01

/* ========================================================================
 * FIFO DATA FORMAT
 * ======================================================================== */
#define MAX86171_FIFO_TAG_MASK          0xF0
#define MAX86171_FIFO_TAG_SHIFT         4
#define MAX86171_FIFO_DATA_SIGN_BIT     0x08  // Bit 19 in 20-bit value

/* FIFO Tags */
#define MAX86171_FIFO_TAG_MEAS1         0x01
#define MAX86171_FIFO_TAG_MEAS2         0x02
#define MAX86171_FIFO_TAG_MEAS3         0x03
#define MAX86171_FIFO_TAG_MEAS4         0x04
#define MAX86171_FIFO_TAG_MEAS5         0x05
#define MAX86171_FIFO_TAG_MEAS6         0x06
#define MAX86171_FIFO_TAG_MEAS7         0x07
#define MAX86171_FIFO_TAG_MEAS8         0x08
#define MAX86171_FIFO_TAG_MEAS9         0x09
#define MAX86171_FIFO_TAG_DARK          0x0A
#define MAX86171_FIFO_TAG_ALC_OVF       0x0B
#define MAX86171_FIFO_TAG_EXP_OVF       0x0C
#define MAX86171_FIFO_TAG_PF            0x0D
#define MAX86171_FIFO_TAG_INVALID       0x0E

/* ========================================================================
 * ENUMERATIONS
 * ======================================================================== */

/**
 * @brief Communication interface type
 */
typedef enum {
    MAX86171_INTERFACE_I2C = 0,
    MAX86171_INTERFACE_SPI = 1
} max86171_interface_t;

/**
 * @brief Synchronization mode
 */
typedef enum {
    MAX86171_SYNC_INTERNAL = 0,     /**< Internal frame sync */
    MAX86171_SYNC_EXTERNAL_TRIG = 1,/**< External frame trigger */
    MAX86171_SYNC_EXTERNAL_CLK = 2  /**< External clock input */
} max86171_sync_mode_t;

/**
 * @brief ADC integration time
 */
typedef enum {
    MAX86171_TINT_14_6_US = 0,      /**< 14.6 microseconds */
    MAX86171_TINT_29_2_US = 1,      /**< 29.2 microseconds */
    MAX86171_TINT_58_6_US = 2,      /**< 58.6 microseconds */
    MAX86171_TINT_117_1_US = 3      /**< 117.1 microseconds */
} max86171_integration_time_t;

/**
 * @brief ADC full-scale range
 */
typedef enum {
    MAX86171_ADC_RANGE_4UA = 0,     /**< 4μA (7.6pA/LSB) */
    MAX86171_ADC_RANGE_8UA = 1,     /**< 8μA (15.3pA/LSB) */
    MAX86171_ADC_RANGE_16UA = 2,    /**< 16μA (30.5pA/LSB) */
    MAX86171_ADC_RANGE_32UA = 3     /**< 32μA (61.0pA/LSB) */
} max86171_adc_range_t;

/**
 * @brief LED current full-scale range
 */
typedef enum {
    MAX86171_LED_RANGE_32MA = 0,    /**< 32mA (0.125mA/LSB) */
    MAX86171_LED_RANGE_64MA = 1,    /**< 64mA (0.250mA/LSB) */
    MAX86171_LED_RANGE_96MA = 2,    /**< 96mA (0.375mA/LSB) */
    MAX86171_LED_RANGE_128MA = 3    /**< 128mA (0.500mA/LSB) */
} max86171_led_range_t;

/**
 * @brief Number of samples to average
 */
typedef enum {
    MAX86171_AVERAGE_1 = 0,         /**< 1 sample */
    MAX86171_AVERAGE_2 = 1,         /**< 2 samples */
    MAX86171_AVERAGE_4 = 2,         /**< 4 samples */
    MAX86171_AVERAGE_8 = 3,         /**< 8 samples */
    MAX86171_AVERAGE_16 = 4,        /**< 16 samples */
    MAX86171_AVERAGE_32 = 5,        /**< 32 samples */
    MAX86171_AVERAGE_64 = 6,        /**< 64 samples */
    MAX86171_AVERAGE_128 = 7        /**< 128 samples */
} max86171_average_t;

/**
 * @brief Photodiode bias setting
 */
typedef enum {
    MAX86171_PD_BIAS_INVALID = 0,   /**< Not recommended */
    MAX86171_PD_BIAS_0_125PF = 1,   /**< 0-125pF */
    MAX86171_PD_BIAS_125_250PF = 2, /**< 125-250pF */
    MAX86171_PD_BIAS_250_500PF = 3  /**< 250-500pF */
} max86171_pd_bias_t;

/**
 * @brief LED driver selection
 */
typedef enum {
    MAX86171_LED_DRV_PIN1 = 0,
    MAX86171_LED_DRV_PIN2 = 1,
    MAX86171_LED_DRV_PIN3 = 2,
    MAX86171_LED_DRV_PIN4 = 3
} max86171_led_drv_pin_t;

/**
 * @brief Photodiode input selection
 */
typedef enum {
    MAX86171_PD_INPUT_1 = 0,        /**< PD1 for PPG1, PD2 for PPG2 */
    MAX86171_PD_INPUT_2 = 1         /**< PD3 for PPG1, PD4 for PPG2 */
} max86171_pd_input_t;

/**
 * @brief Ambient light cancellation filter
 */
typedef enum {
    MAX86171_FILTER_CDM = 0,        /**< Central Difference Method */
    MAX86171_FILTER_FDM = 1         /**< Forward Difference Method */
} max86171_filter_t;

/**
 * @brief Frame rate clock source
 */
typedef enum {
    MAX86171_CLK_32000HZ = 0,       /**< 32.0 kHz */
    MAX86171_CLK_32768HZ = 1        /**< 32.768 kHz */
} max86171_clock_source_t;

/**
 * @brief Interrupt output configuration
 */
typedef enum {
    MAX86171_INT_OPEN_DRAIN = 0,    /**< Open drain, active low */
    MAX86171_INT_PUSH_PULL_HIGH = 1,/**< Push-pull, active high */
    MAX86171_INT_PUSH_PULL_LOW = 2  /**< Push-pull, active low */
} max86171_int_output_t;

/**
 * @brief Measurement number
 */
typedef enum {
    MAX86171_MEAS_1 = 1,
    MAX86171_MEAS_2 = 2,
    MAX86171_MEAS_3 = 3,
    MAX86171_MEAS_4 = 4,
    MAX86171_MEAS_5 = 5,
    MAX86171_MEAS_6 = 6,
    MAX86171_MEAS_7 = 7,
    MAX86171_MEAS_8 = 8,
    MAX86171_MEAS_9 = 9
} max86171_measurement_num_t;

/* ========================================================================
 * STRUCTURES
 * ======================================================================== */

/**
 * @brief FIFO sample structure
 */
typedef struct {
    uint8_t tag;                    /**< FIFO tag indicating data type */
    int32_t data;                   /**< 20-bit signed ADC value */
    bool valid;                     /**< Data validity flag */
} max86171_fifo_sample_t;

/**
 * @brief Measurement configuration structure
 */
typedef struct {
    bool enabled;                           /**< Enable this measurement */
    bool ambient_mode;                      /**< Direct ambient measurement */
    max86171_led_drv_pin_t drva_pin;        /**< LED Driver A pin selection */
    max86171_led_drv_pin_t drvb_pin;        /**< LED Driver B pin selection */
    max86171_led_drv_pin_t drvc_pin;        /**< LED Driver C pin selection */
    max86171_pd_input_t ppg1_pd_sel;        /**< PPG1 photodiode selection */
    max86171_pd_input_t ppg2_pd_sel;        /**< PPG2 photodiode selection */
    max86171_integration_time_t int_time;   /**< Integration time */
    max86171_average_t average;             /**< Number of averages */
    bool sinc3_enable;                      /**< Enable SINC3 filter */
    max86171_filter_t filter_sel;           /**< CDM or FDM */
    max86171_led_range_t led_range;         /**< LED current range */
    max86171_adc_range_t ppg1_adc_range;    /**< PPG1 ADC range */
    max86171_adc_range_t ppg2_adc_range;    /**< PPG2 ADC range */
    uint8_t drva_current;                   /**< LED Driver A current (0-255) */
    uint8_t drvb_current;                   /**< LED Driver B current (0-255) */
    uint8_t drvc_current;                   /**< LED Driver C current (0-255) */
    uint8_t ppg1_dac_offset;                /**< PPG1 DAC offset (0-3) */
    uint8_t ppg2_dac_offset;                /**< PPG2 DAC offset (0-3) */
    uint8_t pd_settling;                    /**< PD settling time (0-3) */
    uint8_t led_settling;                   /**< LED settling time (0-3) */
} max86171_meas_config_t;

/**
 * @brief Device configuration structure
 */
typedef struct {
    max86171_interface_t interface;         /**< I2C or SPI */
    uint8_t i2c_address;                    /**< I2C address (if I2C mode) */
    max86171_sync_mode_t sync_mode;         /**< Synchronization mode */
    max86171_clock_source_t clock_source;   /**< Frame rate clock source */
    uint16_t clock_divider;                 /**< Frame rate clock divider */
    bool ppg1_power_down;                   /**< Power down PPG1 channel */
    bool ppg2_power_down;                   /**< Power down PPG2 channel */
    bool alc_disable;                       /**< Disable analog ALC */
    bool collect_raw_data;                  /**< Collect raw ADC data */
    uint8_t fifo_almost_full;               /**< FIFO almost full threshold */
    bool fifo_rollover;                     /**< Enable FIFO rollover */
    max86171_pd_bias_t pd1_bias;            /**< PD1 bias setting */
    max86171_pd_bias_t pd2_bias;            /**< PD2 bias setting */
    max86171_pd_bias_t pd3_bias;            /**< PD3 bias setting */
    max86171_pd_bias_t pd4_bias;            /**< PD4 bias setting */
} max86171_config_t;

/**
 * @brief Device handle structure
 */
typedef struct {
    max86171_config_t config;               /**< Device configuration */
    void *user_data;                        /**< User data pointer */
    
    /* Function pointers for platform-specific I/O */
    int (*write_reg)(void *handle, uint8_t reg, uint8_t *data, uint16_t len);
    int (*read_reg)(void *handle, uint8_t reg, uint8_t *data, uint16_t len);
    void (*delay_ms)(uint32_t ms);
} max86171_handle_t;

/* ========================================================================
 * FUNCTION PROTOTYPES
 * ======================================================================== */

/* Initialization and Configuration */

/**
 * @brief Initialize the MAX86171 device
 * @param handle Pointer to device handle
 * @param config Pointer to configuration structure
 * @return 0 on success, negative error code on failure
 */
int max86171_init(max86171_handle_t *handle, const max86171_config_t *config);

/**
 * @brief Reset the device to power-on state
 * @param handle Pointer to device handle
 * @return 0 on success, negative error code on failure
 */
int max86171_reset(max86171_handle_t *handle);

/**
 * @brief Enter shutdown mode
 * @param handle Pointer to device handle
 * @param shutdown true to enter shutdown, false to exit
 * @return 0 on success, negative error code on failure
 */
int max86171_shutdown(max86171_handle_t *handle, bool shutdown);

/**
 * @brief Read the part ID
 * @param handle Pointer to device handle
 * @param part_id Pointer to store part ID
 * @return 0 on success, negative error code on failure
 */
int max86171_get_part_id(max86171_handle_t *handle, uint8_t *part_id);

/* Register Access */

/**
 * @brief Write a single register
 * @param handle Pointer to device handle
 * @param reg Register address
 * @param value Value to write
 * @return 0 on success, negative error code on failure
 */
int max86171_write_register(max86171_handle_t *handle, uint8_t reg, uint8_t value);

/**
 * @brief Read a single register
 * @param handle Pointer to device handle
 * @param reg Register address
 * @param value Pointer to store read value
 * @return 0 on success, negative error code on failure
 */
int max86171_read_register(max86171_handle_t *handle, uint8_t reg, uint8_t *value);

/**
 * @brief Write multiple registers
 * @param handle Pointer to device handle
 * @param reg Starting register address
 * @param data Pointer to data buffer
 * @param len Number of bytes to write
 * @return 0 on success, negative error code on failure
 */
int max86171_write_registers(max86171_handle_t *handle, uint8_t reg, 
                             const uint8_t *data, uint16_t len);

/**
 * @brief Read multiple registers
 * @param handle Pointer to device handle
 * @param reg Starting register address
 * @param data Pointer to data buffer
 * @param len Number of bytes to read
 * @return 0 on success, negative error code on failure
 */
int max86171_read_registers(max86171_handle_t *handle, uint8_t reg, 
                            uint8_t *data, uint16_t len);

/* Status and Interrupts */

/**
 * @brief Read status register 1
 * @param handle Pointer to device handle
 * @param status Pointer to store status value
 * @return 0 on success, negative error code on failure
 */
int max86171_get_status1(max86171_handle_t *handle, uint8_t *status);

/**
 * @brief Read status register 2
 * @param handle Pointer to device handle
 * @param status Pointer to store status value
 * @return 0 on success, negative error code on failure
 */
int max86171_get_status2(max86171_handle_t *handle, uint8_t *status);

/**
 * @brief Read status register 3
 * @param handle Pointer to device handle
 * @param status Pointer to store status value
 * @return 0 on success, negative error code on failure
 */
int max86171_get_status3(max86171_handle_t *handle, uint8_t *status);

/**
 * @brief Enable INT1 interrupts
 * @param handle Pointer to device handle
 * @param mask Interrupt mask (OR of MAX86171_STATUS1_* bits)
 * @return 0 on success, negative error code on failure
 */
int max86171_enable_int1(max86171_handle_t *handle, uint8_t mask);

/**
 * @brief Enable INT2 interrupts
 * @param handle Pointer to device handle
 * @param mask Interrupt mask (OR of MAX86171_STATUS1_* bits)
 * @return 0 on success, negative error code on failure
 */
int max86171_enable_int2(max86171_handle_t *handle, uint8_t mask);

/**
 * @brief Configure interrupt output type
 * @param handle Pointer to device handle
 * @param int_num Interrupt number (1 or 2)
 * @param output_type Output configuration
 * @return 0 on success, negative error code on failure
 */
int max86171_config_interrupt_output(max86171_handle_t *handle, uint8_t int_num,
                                     max86171_int_output_t output_type);

/* FIFO Operations */

/**
 * @brief Flush the FIFO
 * @param handle Pointer to device handle
 * @return 0 on success, negative error code on failure
 */
int max86171_fifo_flush(max86171_handle_t *handle);

/**
 * @brief Get the number of samples available in FIFO
 * @param handle Pointer to device handle
 * @param count Pointer to store sample count
 * @return 0 on success, negative error code on failure
 */
int max86171_fifo_get_count(max86171_handle_t *handle, uint16_t *count);

/**
 * @brief Read a single sample from FIFO
 * @param handle Pointer to device handle
 * @param sample Pointer to store sample data
 * @return 0 on success, negative error code on failure
 */
int max86171_fifo_read_sample(max86171_handle_t *handle, 
                              max86171_fifo_sample_t *sample);

/**
 * @brief Read multiple samples from FIFO
 * @param handle Pointer to device handle
 * @param samples Pointer to sample array
 * @param num_samples Number of samples to read
 * @param samples_read Pointer to store actual number of samples read
 * @return 0 on success, negative error code on failure
 */
int max86171_fifo_read_samples(max86171_handle_t *handle, 
                               max86171_fifo_sample_t *samples,
                               uint16_t num_samples, 
                               uint16_t *samples_read);

/**
 * @brief Set FIFO almost full threshold
 * @param handle Pointer to device handle
 * @param threshold Number of free spaces before interrupt (0-255)
 * @return 0 on success, negative error code on failure
 */
int max86171_fifo_set_almost_full(max86171_handle_t *handle, uint8_t threshold);

/* Measurement Configuration */

/**
 * @brief Configure a measurement
 * @param handle Pointer to device handle
 * @param meas_num Measurement number (1-9)
 * @param config Pointer to measurement configuration
 * @return 0 on success, negative error code on failure
 */
int max86171_config_measurement(max86171_handle_t *handle, 
                                max86171_measurement_num_t meas_num,
                                const max86171_meas_config_t *config);

/**
 * @brief Enable/disable a measurement
 * @param handle Pointer to device handle
 * @param meas_num Measurement number (1-9)
 * @param enable true to enable, false to disable
 * @return 0 on success, negative error code on failure
 */
int max86171_enable_measurement(max86171_handle_t *handle, 
                               max86171_measurement_num_t meas_num,
                               bool enable);

/**
 * @brief Set LED current for a measurement
 * @param handle Pointer to device handle
 * @param meas_num Measurement number (1-9)
 * @param drva_current Driver A current (0-255)
 * @param drvb_current Driver B current (0-255)
 * @param drvc_current Driver C current (0-255)
 * @return 0 on success, negative error code on failure
 */
int max86171_set_led_current(max86171_handle_t *handle,
                             max86171_measurement_num_t meas_num,
                             uint8_t drva_current,
                             uint8_t drvb_current,
                             uint8_t drvc_current);

/* Frame Rate Control */

/**
 * @brief Set frame rate clock divider
 * @param handle Pointer to device handle
 * @param divider Clock divider value (11-32768)
 * @return 0 on success, negative error code on failure
 */
int max86171_set_frame_rate_divider(max86171_handle_t *handle, uint16_t divider);

/**
 * @brief Set frame rate clock source
 * @param handle Pointer to device handle
 * @param source Clock source selection
 * @return 0 on success, negative error code on failure
 */
int max86171_set_clock_source(max86171_handle_t *handle, 
                              max86171_clock_source_t source);

/**
 * @brief Calculate frame rate
 * @param clock_source Clock source (32000Hz or 32768Hz)
 * @param divider Clock divider value
 * @return Frame rate in Hz
 */
float max86171_calculate_frame_rate(max86171_clock_source_t clock_source,
                                    uint16_t divider);

/* Power Management */

/**
 * @brief Power down PPG channel
 * @param handle Pointer to device handle
 * @param ppg_num PPG channel number (1 or 2)
 * @param power_down true to power down, false to power up
 * @return 0 on success, negative error code on failure
 */
int max86171_ppg_power_down(max86171_handle_t *handle, uint8_t ppg_num, 
                            bool power_down);

/* Threshold Configuration */

/**
 * @brief Configure threshold detection
 * @param handle Pointer to device handle
 * @param thresh_num Threshold number (1 or 2)
 * @param meas_num Measurement to monitor (0 to disable)
 * @param ppg_channel PPG channel to monitor (1 or 2)
 * @param upper_threshold Upper threshold value
 * @param lower_threshold Lower threshold value
 * @return 0 on success, negative error code on failure
 */
int max86171_config_threshold(max86171_handle_t *handle,
                              uint8_t thresh_num,
                              uint8_t meas_num,
                              uint8_t ppg_channel,
                              uint8_t upper_threshold,
                              uint8_t lower_threshold);

/* Utility Functions */

/**
 * @brief Convert LED current code to milliamps
 * @param range LED range setting
 * @param code LED current code (0-255)
 * @return Current in milliamps
 */
float max86171_led_code_to_ma(max86171_led_range_t range, uint8_t code);

/**
 * @brief Convert milliamps to LED current code
 * @param range LED range setting
 * @param current_ma Current in milliamps
 * @return LED current code (0-255)
 */
uint8_t max86171_ma_to_led_code(max86171_led_range_t range, float current_ma);

/**
 * @brief Convert ADC code to microamps
 * @param range ADC range setting
 * @param code ADC code (signed 20-bit)
 * @return Current in microamps
 */
float max86171_adc_code_to_ua(max86171_adc_range_t range, int32_t code);

/**
 * @brief Get register address for measurement
 * @param meas_num Measurement number (1-9)
 * @param reg_offset Register offset (0-6)
 * @return Register address
 */
uint8_t max86171_get_meas_reg_addr(max86171_measurement_num_t meas_num, 
                                   uint8_t reg_offset);

#ifdef __cplusplus
}
#endif

#endif /* MAX86171_H */