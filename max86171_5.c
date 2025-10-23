/**
 * @file max86171.c
 * @brief MAX86171 Low-Noise AFE Driver Implementation
 */

#include "max86171.h"
#include <string.h>

/* ========================================================================
 * PRIVATE MACROS
 * ======================================================================== */
#define CHECK_NULL(ptr) if ((ptr) == NULL) return -1
#define CHECK_HANDLE(h) if ((h) == NULL || (h)->write_reg == NULL || (h)->read_reg == NULL) return -1

/* FIFO bytes per sample */
#define FIFO_BYTES_PER_SAMPLE   3

/* Sign extension for 20-bit value */
#define SIGN_EXTEND_20BIT(val) (((val) & 0x80000) ? ((val) | 0xFFF00000) : (val))

/* ========================================================================
 * PRIVATE FUNCTION PROTOTYPES
 * ======================================================================== */
static int max86171_modify_register(max86171_handle_t *handle, uint8_t reg,
                                    uint8_t mask, uint8_t value);
static int max86171_set_bit(max86171_handle_t *handle, uint8_t reg, uint8_t bit);
static int max86171_clear_bit(max86171_handle_t *handle, uint8_t reg, uint8_t bit);

/* ========================================================================
 * PUBLIC FUNCTION IMPLEMENTATIONS
 * ======================================================================== */

/**
 * @brief Initialize the MAX86171 device
 */
int max86171_init(max86171_handle_t *handle, const max86171_config_t *config)
{
    int ret;
    uint8_t part_id;
    
    CHECK_NULL(handle);
    CHECK_NULL(config);
    CHECK_HANDLE(handle);
    
    /* Copy configuration */
    memcpy(&handle->config, config, sizeof(max86171_config_t));
    
    /* Reset device */
    ret = max86171_reset(handle);
    if (ret != 0) return ret;
    
    /* Wait for device to be ready */
    if (handle->delay_ms != NULL) {
        handle->delay_ms(10);
    }
    
    /* Verify part ID */
    ret = max86171_get_part_id(handle, &part_id);
    if (ret != 0) return ret;
    
    if (part_id != MAX86171_PART_ID) {
        return -2; /* Invalid part ID */
    }
    
    /* Configure photodiode bias */
    uint8_t pd_bias = ((config->pd4_bias & 0x03) << 6) |
                      ((config->pd3_bias & 0x03) << 4) |
                      ((config->pd2_bias & 0x03) << 2) |
                      (config->pd1_bias & 0x03);
    ret = max86171_write_register(handle, MAX86171_REG_PD_BIAS, pd_bias);
    if (ret != 0) return ret;
    
    /* Configure system settings */
    uint8_t sys_config1 = 0;
    sys_config1 |= (config->sync_mode << MAX86171_SYNC_MODE_SHIFT) & MAX86171_SYNC_MODE_MASK;
    if (config->ppg2_power_down) sys_config1 |= MAX86171_PPG2_PWRDN;
    if (config->ppg1_power_down) sys_config1 |= MAX86171_PPG1_PWRDN;
    ret = max86171_write_register(handle, MAX86171_REG_SYS_CONFIG_1, sys_config1);
    if (ret != 0) return ret;
    
    /* Configure system config 3 */
    uint8_t sys_config3 = 0;
    if (config->alc_disable) sys_config3 |= MAX86171_ALC_DISABLE;
    if (config->collect_raw_data) sys_config3 |= MAX86171_COLLECT_RAW_DATA;
    ret = max86171_write_register(handle, MAX86171_REG_SYS_CONFIG_3, sys_config3);
    if (ret != 0) return ret;
    
    /* Configure frame rate clock */
    ret = max86171_set_clock_source(handle, config->clock_source);
    if (ret != 0) return ret;
    
    ret = max86171_set_frame_rate_divider(handle, config->clock_divider);
    if (ret != 0) return ret;
    
    /* Configure FIFO */
    ret = max86171_fifo_set_almost_full(handle, config->fifo_almost_full);
    if (ret != 0) return ret;
    
    uint8_t fifo_config2 = 0;
    if (config->fifo_rollover) fifo_config2 |= MAX86171_FIFO_RO;
    fifo_config2 |= MAX86171_FIFO_STAT_CLR; /* Enable status clear on FIFO read */
    ret = max86171_write_register(handle, MAX86171_REG_FIFO_CONFIG_2, fifo_config2);
    if (ret != 0) return ret;
    
    /* Flush FIFO */
    ret = max86171_fifo_flush(handle);
    if (ret != 0) return ret;
    
    return 0;
}

/**
 * @brief Reset the device
 */
int max86171_reset(max86171_handle_t *handle)
{
    CHECK_HANDLE(handle);
    
    int ret = max86171_set_bit(handle, MAX86171_REG_SYS_CONFIG_1, MAX86171_RESET);
    
    /* Wait for reset to complete */
    if (handle->delay_ms != NULL) {
        handle->delay_ms(10);
    }
    
    return ret;
}

/**
 * @brief Enter/exit shutdown mode
 */
int max86171_shutdown(max86171_handle_t *handle, bool shutdown)
{
    CHECK_HANDLE(handle);
    
    if (shutdown) {
        return max86171_set_bit(handle, MAX86171_REG_SYS_CONFIG_1, MAX86171_SHDN);
    } else {
        return max86171_clear_bit(handle, MAX86171_REG_SYS_CONFIG_1, MAX86171_SHDN);
    }
}

/**
 * @brief Read part ID
 */
int max86171_get_part_id(max86171_handle_t *handle, uint8_t *part_id)
{
    CHECK_HANDLE(handle);
    CHECK_NULL(part_id);
    
    return max86171_read_register(handle, MAX86171_REG_PART_ID, part_id);
}

/**
 * @brief Write a single register
 */
int max86171_write_register(max86171_handle_t *handle, uint8_t reg, uint8_t value)
{
    CHECK_HANDLE(handle);
    
    return handle->write_reg(handle->user_data, reg, &value, 1);
}

/**
 * @brief Read a single register
 */
int max86171_read_register(max86171_handle_t *handle, uint8_t reg, uint8_t *value)
{
    CHECK_HANDLE(handle);
    CHECK_NULL(value);
    
    return handle->read_reg(handle->user_data, reg, value, 1);
}

/**
 * @brief Write multiple registers
 */
int max86171_write_registers(max86171_handle_t *handle, uint8_t reg, 
                             const uint8_t *data, uint16_t len)
{
    CHECK_HANDLE(handle);
    CHECK_NULL(data);
    
    return handle->write_reg(handle->user_data, reg, (uint8_t*)data, len);
}

/**
 * @brief Read multiple registers
 */
int max86171_read_registers(max86171_handle_t *handle, uint8_t reg, 
                            uint8_t *data, uint16_t len)
{
    CHECK_HANDLE(handle);
    CHECK_NULL(data);
    
    return handle->read_reg(handle->user_data, reg, data, len);
}

/**
 * @brief Read status register 1
 */
int max86171_get_status1(max86171_handle_t *handle, uint8_t *status)
{
    return max86171_read_register(handle, MAX86171_REG_STATUS_1, status);
}

/**
 * @brief Read status register 2
 */
int max86171_get_status2(max86171_handle_t *handle, uint8_t *status)
{
    return max86171_read_register(handle, MAX86171_REG_STATUS_2, status);
}

/**
 * @brief Read status register 3
 */
int max86171_get_status3(max86171_handle_t *handle, uint8_t *status)
{
    return max86171_read_register(handle, MAX86171_REG_STATUS_3, status);
}

/**
 * @brief Enable INT1 interrupts
 */
int max86171_enable_int1(max86171_handle_t *handle, uint8_t mask)
{
    CHECK_HANDLE(handle);
    
    return max86171_write_register(handle, MAX86171_REG_INT1_ENABLE_1, mask);
}

/**
 * @brief Enable INT2 interrupts
 */
int max86171_enable_int2(max86171_handle_t *handle, uint8_t mask)
{
    CHECK_HANDLE(handle);
    
    return max86171_write_register(handle, MAX86171_REG_INT2_ENABLE_1, mask);
}

/**
 * @brief Configure interrupt output type
 */
int max86171_config_interrupt_output(max86171_handle_t *handle, uint8_t int_num,
                                     max86171_int_output_t output_type)
{
    CHECK_HANDLE(handle);
    
    uint8_t shift = (int_num == 1) ? 2 : 4;
    uint8_t mask = 0x03 << shift;
    uint8_t value = ((uint8_t)output_type << shift) & mask;
    
    return max86171_modify_register(handle, MAX86171_REG_OUTPUT_PIN_CFG, mask, value);
}

/**
 * @brief Flush the FIFO
 */
int max86171_fifo_flush(max86171_handle_t *handle)
{
    CHECK_HANDLE(handle);
    
    return max86171_set_bit(handle, MAX86171_REG_FIFO_CONFIG_2, MAX86171_FIFO_FLUSH);
}

/**
 * @brief Get number of samples in FIFO
 */
int max86171_fifo_get_count(max86171_handle_t *handle, uint16_t *count)
{
    int ret;
    uint8_t data[2];
    
    CHECK_HANDLE(handle);
    CHECK_NULL(count);
    
    /* Read FIFO counter registers */
    ret = max86171_read_register(handle, MAX86171_REG_FIFO_COUNTER_1, &data[0]);
    if (ret != 0) return ret;
    
    ret = max86171_read_register(handle, MAX86171_REG_FIFO_COUNTER_2, &data[1]);
    if (ret != 0) return ret;
    
    /* Combine MSB and LSB */
    *count = ((uint16_t)(data[0] & 0x01) << 8) | data[1];
    
    return 0;
}

/**
 * @brief Read a single sample from FIFO
 */
int max86171_fifo_read_sample(max86171_handle_t *handle, 
                              max86171_fifo_sample_t *sample)
{
    int ret;
    uint8_t fifo_data[FIFO_BYTES_PER_SAMPLE];
    
    CHECK_HANDLE(handle);
    CHECK_NULL(sample);
    
    /* Read 3 bytes from FIFO */
    ret = max86171_read_registers(handle, MAX86171_REG_FIFO_DATA, 
                                  fifo_data, FIFO_BYTES_PER_SAMPLE);
    if (ret != 0) {
        sample->valid = false;
        return ret;
    }
    
    /* Extract tag (upper 4 bits of first byte) */
    sample->tag = (fifo_data[0] & MAX86171_FIFO_TAG_MASK) >> MAX86171_FIFO_TAG_SHIFT;
    
    /* Extract 20-bit data value */
    int32_t raw_value = ((uint32_t)(fifo_data[0] & 0x0F) << 16) |
                        ((uint32_t)fifo_data[1] << 8) |
                        fifo_data[2];
    
    /* Sign extend from 20-bit to 32-bit */
    sample->data = SIGN_EXTEND_20BIT(raw_value);
    
    /* Mark as valid if not an invalid tag */
    sample->valid = (sample->tag != MAX86171_FIFO_TAG_INVALID);
    
    return 0;
}

/**
 * @brief Read multiple samples from FIFO
 */
int max86171_fifo_read_samples(max86171_handle_t *handle, 
                               max86171_fifo_sample_t *samples,
                               uint16_t num_samples, 
                               uint16_t *samples_read)
{
    int ret;
    uint16_t available;
    uint16_t to_read;
    
    CHECK_HANDLE(handle);
    CHECK_NULL(samples);
    CHECK_NULL(samples_read);
    
    *samples_read = 0;
    
    /* Get number of samples available */
    ret = max86171_fifo_get_count(handle, &available);
    if (ret != 0) return ret;
    
    /* Limit to requested number */
    to_read = (available < num_samples) ? available : num_samples;
    
    /* Read samples one by one */
    for (uint16_t i = 0; i < to_read; i++) {
        ret = max86171_fifo_read_sample(handle, &samples[i]);
        if (ret != 0) return ret;
        
        if (samples[i].valid) {
            (*samples_read)++;
        }
    }
    
    return 0;
}

/**
 * @brief Set FIFO almost full threshold
 */
int max86171_fifo_set_almost_full(max86171_handle_t *handle, uint8_t threshold)
{
    CHECK_HANDLE(handle);
    
    return max86171_write_register(handle, MAX86171_REG_FIFO_CONFIG_1, threshold);
}

/**
 * @brief Configure a measurement
 */
int max86171_config_measurement(max86171_handle_t *handle, 
                                max86171_measurement_num_t meas_num,
                                const max86171_meas_config_t *config)
{
    int ret;
    uint8_t base_addr;
    uint8_t reg_data[7];
    
    CHECK_HANDLE(handle);
    CHECK_NULL(config);
    
    if (meas_num < 1 || meas_num > 9) return -1;
    
    /* Calculate base register address */
    base_addr = max86171_get_meas_reg_addr(meas_num, 0);
    
    /* MEAS Select register */
    reg_data[0] = 0;
    if (config->ambient_mode) reg_data[0] |= 0x40;
    reg_data[0] |= (config->drvc_pin & 0x03) << 4;
    reg_data[0] |= (config->drvb_pin & 0x03) << 2;
    reg_data[0] |= (config->drva_pin & 0x03);
    
    /* MEAS Config 1 register */
    reg_data[1] = 0;
    if (config->ppg2_pd_sel) reg_data[1] |= 0x40;
    if (config->ppg1_pd_sel) reg_data[1] |= 0x20;
    reg_data[1] |= (config->int_time & 0x03) << 3;
    reg_data[1] |= (config->average & 0x07);
    
    /* MEAS Config 2 register */
    reg_data[2] = 0;
    if (config->sinc3_enable) reg_data[2] |= 0x80;
    if (config->filter_sel) reg_data[2] |= 0x40;
    reg_data[2] |= (config->led_range & 0x03) << 4;
    reg_data[2] |= (config->ppg2_adc_range & 0x03) << 2;
    reg_data[2] |= (config->ppg1_adc_range & 0x03);
    
    /* MEAS Config 3 register */
    reg_data[3] = 0;
    reg_data[3] |= (config->pd_settling & 0x03) << 6;
    reg_data[3] |= (config->led_settling & 0x03) << 4;
    reg_data[3] |= (config->ppg2_dac_offset & 0x03) << 2;
    reg_data[3] |= (config->ppg1_dac_offset & 0x03);
    
    /* LED current registers */
    reg_data[4] = config->drva_current;
    reg_data[5] = config->drvb_current;
    reg_data[6] = config->drvc_current;
    
    /* Write all registers */
    ret = max86171_write_registers(handle, base_addr, reg_data, 7);
    if (ret != 0) return ret;
    
    /* Enable/disable measurement */
    return max86171_enable_measurement(handle, meas_num, config->enabled);
}

/**
 * @brief Enable/disable a measurement
 */
int max86171_enable_measurement(max86171_handle_t *handle, 
                               max86171_measurement_num_t meas_num,
                               bool enable)
{
    uint8_t reg;
    uint8_t bit;
    
    CHECK_HANDLE(handle);
    
    if (meas_num < 1 || meas_num > 9) return -1;
    
    /* Determine register and bit */
    if (meas_num == 9) {
        reg = MAX86171_REG_SYS_CONFIG_1;
        bit = MAX86171_MEAS9_EN;
    } else {
        reg = MAX86171_REG_SYS_CONFIG_2;
        bit = (1 << (meas_num - 1));
    }
    
    /* Set or clear bit */
    if (enable) {
        return max86171_set_bit(handle, reg, bit);
    } else {
        return max86171_clear_bit(handle, reg, bit);
    }
}

/**
 * @brief Set LED current for a measurement
 */
int max86171_set_led_current(max86171_handle_t *handle,
                             max86171_measurement_num_t meas_num,
                             uint8_t drva_current,
                             uint8_t drvb_current,
                             uint8_t drvc_current)
{
    int ret;
    uint8_t base_addr;
    uint8_t reg_data[3];
    
    CHECK_HANDLE(handle);
    
    if (meas_num < 1 || meas_num > 9) return -1;
    
    /* Calculate base address for current registers */
    base_addr = max86171_get_meas_reg_addr(meas_num, 4);
    
    reg_data[0] = drva_current;
    reg_data[1] = drvb_current;
    reg_data[2] = drvc_current;
    
    return max86171_write_registers(handle, base_addr, reg_data, 3);
}

/**
 * @brief Set frame rate clock divider
 */
int max86171_set_frame_rate_divider(max86171_handle_t *handle, uint16_t divider)
{
    int ret;
    uint8_t msb, lsb;
    
    CHECK_HANDLE(handle);
    
    /* Clamp divider to valid range */
    if (divider < 11) divider = 11;
    if (divider > 32768) divider = 32768;
    
    /* Split into MSB and LSB */
    msb = (divider >> 8) & 0x7F;
    lsb = divider & 0xFF;
    
    /* Write MSB */
    ret = max86171_write_register(handle, MAX86171_REG_FR_CLK_DIV_MSB, msb);
    if (ret != 0) return ret;
    
    /* Write LSB */
    ret = max86171_write_register(handle, MAX86171_REG_FR_CLK_DIV_LSB, lsb);
    if (ret != 0) return ret;
    
    /* Update cached value */
    handle->config.clock_divider = divider;
    
    return 0;
}

/**
 * @brief Set frame rate clock source
 */
int max86171_set_clock_source(max86171_handle_t *handle, 
                              max86171_clock_source_t source)
{
    int ret;
    uint8_t value;
    
    CHECK_HANDLE(handle);
    
    /* Read current value */
    ret = max86171_read_register(handle, MAX86171_REG_FR_CLK_FREQ_SEL, &value);
    if (ret != 0) return ret;
    
    /* Modify clock select bit */
    if (source == MAX86171_CLK_32768HZ) {
        value |= 0x20;
    } else {
        value &= ~0x20;
    }
    
    /* Write back */
    ret = max86171_write_register(handle, MAX86171_REG_FR_CLK_FREQ_SEL, value);
    if (ret != 0) return ret;
    
    /* Update cached value */
    handle->config.clock_source = source;
    
    return 0;
}

/**
 * @brief Calculate frame rate
 */
float max86171_calculate_frame_rate(max86171_clock_source_t clock_source,
                                    uint16_t divider)
{
    float clock_freq = (clock_source == MAX86171_CLK_32768HZ) ? 32768.0f : 32000.0f;
    
    if (divider < 11) divider = 11;
    
    return clock_freq / (float)divider;
}

/**
 * @brief Power down PPG channel
 */
int max86171_ppg_power_down(max86171_handle_t *handle, uint8_t ppg_num, 
                            bool power_down)
{
    uint8_t bit;
    
    CHECK_HANDLE(handle);
    
    if (ppg_num == 1) {
        bit = MAX86171_PPG1_PWRDN;
    } else if (ppg_num == 2) {
        bit = MAX86171_PPG2_PWRDN;
    } else {
        return -1;
    }
    
    if (power_down) {
        return max86171_set_bit(handle, MAX86171_REG_SYS_CONFIG_1, bit);
    } else {
        return max86171_clear_bit(handle, MAX86171_REG_SYS_CONFIG_1, bit);
    }
}

/**
 * @brief Configure threshold detection
 */
int max86171_config_threshold(max86171_handle_t *handle,
                              uint8_t thresh_num,
                              uint8_t meas_num,
                              uint8_t ppg_channel,
                              uint8_t upper_threshold,
                              uint8_t lower_threshold)
{
    int ret;
    uint8_t meas_sel;
    uint8_t ppg_sel;
    
    CHECK_HANDLE(handle);
    
    if (thresh_num < 1 || thresh_num > 2) return -1;
    if (meas_num > 9) return -1;
    if (ppg_channel < 1 || ppg_channel > 2) return -1;
    
    /* Read current measurement select */
    ret = max86171_read_register(handle, MAX86171_REG_THRESH_MEAS_SEL, &meas_sel);
    if (ret != 0) return ret;
    
    /* Update measurement selection */
    if (thresh_num == 1) {
        meas_sel = (meas_sel & 0xF0) | (meas_num & 0x0F);
    } else {
        meas_sel = (meas_sel & 0x0F) | ((meas_num & 0x0F) << 4);
    }
    
    ret = max86171_write_register(handle, MAX86171_REG_THRESH_MEAS_SEL, meas_sel);
    if (ret != 0) return ret;
    
    /* Configure PPG channel selection */
    ret = max86171_read_register(handle, MAX86171_REG_THRESH_HYST, &ppg_sel);
    if (ret != 0) return ret;
    
    if (thresh_num == 1) {
        if (ppg_channel == 1) {
            ppg_sel &= ~0x40;
        } else {
            ppg_sel |= 0x40;
        }
    } else {
        if (ppg_channel == 1) {
            ppg_sel &= ~0x80;
        } else {
            ppg_sel |= 0x80;
        }
    }
    
    ret = max86171_write_register(handle, MAX86171_REG_THRESH_HYST, ppg_sel);
    if (ret != 0) return ret;
    
    /* Write threshold values */
    if (thresh_num == 1) {
        ret = max86171_write_register(handle, MAX86171_REG_THRESH1_HI, upper_threshold);
        if (ret != 0) return ret;
        ret = max86171_write_register(handle, MAX86171_REG_THRESH1_LO, lower_threshold);
    } else {
        ret = max86171_write_register(handle, MAX86171_REG_THRESH2_HI, upper_threshold);
        if (ret != 0) return ret;
        ret = max86171_write_register(handle, MAX86171_REG_THRESH2_LO, lower_threshold);
    }
    
    return ret;
}

/**
 * @brief Convert LED current code to milliamps
 */
float max86171_led_code_to_ma(max86171_led_range_t range, uint8_t code)
{
    const float lsb_ma[] = {0.125f, 0.250f, 0.375f, 0.500f};
    
    if (range > MAX86171_LED_RANGE_128MA) {
        range = MAX86171_LED_RANGE_128MA;
    }
    
    return (float)code * lsb_ma[range];
}

/**
 * @brief Convert milliamps to LED current code
 */
uint8_t max86171_ma_to_led_code(max86171_led_range_t range, float current_ma)
{
    const float lsb_ma[] = {0.125f, 0.250f, 0.375f, 0.500f};
    uint16_t code;
    
    if (range > MAX86171_LED_RANGE_128MA) {
        range = MAX86171_LED_RANGE_128MA;
    }
    
    code = (uint16_t)(current_ma / lsb_ma[range] + 0.5f);
    
    if (code > 255) code = 255;
    
    return (uint8_t)code;
}

/**
 * @brief Convert ADC code to microamps
 */
float max86171_adc_code_to_ua(max86171_adc_range_t range, int32_t code)
{
    const float lsb_pa[] = {7.6f, 15.3f, 30.5f, 61.0f};
    
    if (range > MAX86171_ADC_RANGE_32UA) {
        range = MAX86171_ADC_RANGE_32UA;
    }
    
    return (float)code * lsb_pa[range] / 1000.0f;
}

/**
 * @brief Get register address for measurement
 */
uint8_t max86171_get_meas_reg_addr(max86171_measurement_num_t meas_num, 
                                   uint8_t reg_offset)
{
    if (meas_num < 1 || meas_num > 9) return 0;
    if (reg_offset > 6) return 0;
    
    /* Base addresses for each measurement */
    const uint8_t base_addrs[] = {
        0x18, 0x20, 0x28, 0x30, 0x38, 0x40, 0x48, 0x50, 0x58
    };
    
    return base_addrs[meas_num - 1] + reg_offset;
}

/* ========================================================================
 * PRIVATE FUNCTION IMPLEMENTATIONS
 * ======================================================================== */

/**
 * @brief Modify specific bits in a register
 */
static int max86171_modify_register(max86171_handle_t *handle, uint8_t reg,
                                    uint8_t mask, uint8_t value)
{
    int ret;
    uint8_t reg_value;
    
    /* Read current value */
    ret = max86171_read_register(handle, reg, &reg_value);
    if (ret != 0) return ret;
    
    /* Modify bits */
    reg_value = (reg_value & ~mask) | (value & mask);
    
    /* Write back */
    return max86171_write_register(handle, reg, reg_value);
}

/**
 * @brief Set a specific bit in a register
 */
static int max86171_set_bit(max86171_handle_t *handle, uint8_t reg, uint8_t bit)
{
    return max86171_modify_register(handle, reg, bit, bit);
}

/**
 * @brief Clear a specific bit in a register
 */
static int max86171_clear_bit(max86171_handle_t *handle, uint8_t reg, uint8_t bit)
{
    return max86171_modify_register(handle, reg, bit, 0);
}