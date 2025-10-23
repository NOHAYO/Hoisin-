# Sensor Register Map Quick Reference

## MAX86171 PPG/ECG Sensor Registers

### Configuration & Control
| Address | Register Name | Description | Key Bits |
|---------|--------------|-------------|----------|
| 0x00 | INT_ENABLE_1 | Interrupt Enable 1 | A_FULL_EN[7], PPG_RDY_EN[6], ALC_OVF_EN[5] |
| 0x01 | INT_ENABLE_2 | Interrupt Enable 2 | DIE_TEMP_RDY_EN[1] |
| 0x02 | INT_STATUS_1 | Interrupt Status 1 | A_FULL[7], PPG_RDY[6], ALC_OVF[5] |
| 0x03 | INT_STATUS_2 | Interrupt Status 2 | DIE_TEMP_RDY[1], PWR_RDY[0] |

### FIFO Configuration
| Address | Register Name | Description | Config for 100 SPS |
|---------|--------------|-------------|-------------------|
| 0x04 | FIFO_WR_PTR | FIFO Write Pointer | Read only |
| 0x05 | OVF_COUNTER | FIFO Overflow Counter | Read only |
| 0x06 | FIFO_RD_PTR | FIFO Read Pointer | Read only |
| 0x07 | FIFO_DATA | FIFO Data Register | Read data here |
| 0x08 | FIFO_CONFIG_1 | FIFO Configuration 1 | FIFO_A_FULL[4:0]=10 |
| 0x09 | FIFO_CONFIG_2 | FIFO Configuration 2 | FLUSH_FIFO[4], FIFO_RO[2] |
| 0x0A | FIFO_CONFIG_3 | FIFO Configuration 3 | FIFO_STAT_CLR[0] |

### PPG Configuration
| Address | Register Name | Description | Config for 100 SPS |
|---------|--------------|-------------|-------------------|
| 0x0D | PPG_CONFIG_1 | PPG Configuration 1 | PPG_ADC_RGE[1:0], PPG_TINT[4:2] |
| 0x0E | PPG_CONFIG_2 | PPG Configuration 2 | SMP_AVG[2:0], SMP_FREQ[6:4] for 100Hz |
| 0x0F | PPG_CONFIG_3 | PPG Configuration 3 | LED_RANGE[6:5] |
| 0x10 | PD_BIAS | Photodiode Bias | Default: 0x00 |

### LED Configuration (Slot 1)
| Address | Register Name | Description | Value for 50mA |
|---------|--------------|-------------|----------------|
| 0x21 | LED1_PA | LED1 Pulse Amplitude | ~0x32 (50mA) |
| 0x22 | LED2_PA | LED2 Pulse Amplitude | ~0x32 (50mA) |
| 0x23 | LED3_PA | LED3 Pulse Amplitude | ~0x32 (50mA) |
| 0x24 | LED_RANGE | LED Range | LED_RANGE[1:0] |
| 0x25 | LED_PILOT_PA | Pilot LED Amplitude | For proximity |

### System Configuration
| Address | Register Name | Description | Value |
|---------|--------------|-------------|-------|
| 0x0C | SYSTEM_CONTROL | System Control | RESET[0]=1, SHDN[1] |
| 0x11 | PROX_INT_THRESH | Proximity Interrupt Threshold | Configure if using proximity |
| 0xFF | PART_ID | Part ID | Should read 0x22 or 0x23 |

### Sample Rate Settings (Register 0x0E: PPG_CONFIG_2)
```
SMP_FREQ[6:4] bits for sample rate:
000 = 10 Hz
001 = 20 Hz
010 = 50 Hz
011 = 84 Hz
100 = 100 Hz  ← Use this
101 = 200 Hz
110 = 400 Hz
111 = Reserved

SMP_AVG[2:0] bits for averaging:
000 = No averaging
001 = Average 2 samples
010 = Average 4 samples
011 = Average 8 samples
100 = Average 16 samples
101 = Average 32 samples
```

### LED Current Settings
```
LED current = LED_PA × 0.2mA (when LED_RANGE = 0)
LED current = LED_PA × 0.4mA (when LED_RANGE = 1)
LED current = LED_PA × 0.8mA (when LED_RANGE = 2)
LED current = LED_PA × 1.6mA (when LED_RANGE = 3)

For 50mA with LED_RANGE=1:
LED_PA = 50mA / 0.4mA = 125 (0x7D)

For 50mA with LED_RANGE=2:
LED_PA = 50mA / 0.8mA = 62.5 ≈ 63 (0x3F)
```

---

## KX122-1037 Accelerometer Registers

### Device Identification
| Address | Register Name | Description | Expected Value |
|---------|--------------|-------------|----------------|
| 0x13 | WHO_AM_I | Device ID | 0x1A |
| 0x14 | TSCP | Current Tilt Position | Read only |
| 0x15 | TSPP | Previous Tilt Position | Read only |

### Accelerometer Data Registers
| Address | Register Name | Description | Format |
|---------|--------------|-------------|---------|
| 0x06 | XOUT_L | X-axis output LSB | 8-bit LSB |
| 0x07 | XOUT_H | X-axis output MSB | 8-bit MSB |
| 0x08 | YOUT_L | Y-axis output LSB | 8-bit LSB |
| 0x09 | YOUT_H | Y-axis output MSB | 8-bit MSB |
| 0x0A | ZOUT_L | Z-axis output LSB | 8-bit LSB |
| 0x0B | ZOUT_H | Z-axis output MSB | 8-bit MSB |

### Control Registers
| Address | Register Name | Description | Config for 100 Hz |
|---------|--------------|-------------|-------------------|
| 0x18 | CNTL1 | Control 1 | PC1[7]=1, RES[6]=1, GSEL[4:3] |
| 0x19 | CNTL2 | Control 2 | SRST[7] for soft reset |
| 0x1A | CNTL3 | Control 3 | OTP[6:5], OTDT[4:2], OWUF[1:0] |
| 0x1B | ODCNTL | Output Data Control | OSA[3:0]=0010 for 100Hz |

### Interrupt Configuration
| Address | Register Name | Description | Value |
|---------|--------------|-------------|-------|
| 0x1C | INC1 | Interrupt Control 1 | IEN1[5]=1 (enable INT1) |
| 0x1D | INC2 | Interrupt Control 2 | IEL1[3]=0 (latched) |
| 0x1E | INC3 | Interrupt Control 3 | IEL2[3], IEA2[4] |
| 0x1F | INC4 | Interrupt Control 4 | DRDYI1[4]=1 (DRDY on INT1) |
| 0x20 | INC5 | Interrupt Control 5 | Pin routing |
| 0x21 | INC6 | Interrupt Control 6 | Pin routing |

### Buffer Control
| Address | Register Name | Description | Config |
|---------|--------------|-------------|--------|
| 0x3A | BUF_CNTL1 | Buffer Control 1 | SMP_TH[7:0] threshold |
| 0x3B | BUF_CNTL2 | Buffer Control 2 | BUFE[7]=1, BRES[6], BM[1:0] |
| 0x3C | BUF_STATUS_1 | Buffer Status 1 | SMP_LEV[7:0] level |
| 0x3D | BUF_STATUS_2 | Buffer Status 2 | BUF_TRIG[7] |
| 0x3E | BUF_CLEAR | Buffer Clear | Clear buffer |
| 0x3F-0x5E | BUF_READ | Buffer Read | Read buffer data |

### ODR (Output Data Rate) Settings (Register 0x1B: ODCNTL)
```
OSA[3:0] bits for ODR:
0000 = 12.5 Hz
0001 = 25 Hz
0010 = 50 Hz
0011 = 100 Hz  ← Use this for 100 Hz
0100 = 200 Hz
0101 = 400 Hz
0110 = 800 Hz
0111 = 1600 Hz
1000 = 0.781 Hz
1001 = 1.563 Hz
1010 = 3.125 Hz
1011 = 6.25 Hz
1100-1111 = Reserved
```

### G-Range Settings (Register 0x18: CNTL1)
```
GSEL[4:3] bits:
00 = ±2g  ← Recommended for most applications
01 = ±4g
10 = ±8g
11 = Reserved

Resolution (RES bit[6]):
0 = 8-bit mode
1 = 16-bit high resolution mode ← Use this
```

### Data Conversion
```c
// For ±2g range with 16-bit resolution:
float accel_x_g = (int16_t)(XOUT_H << 8 | XOUT_L) / 32768.0f * 2.0f;
float accel_y_g = (int16_t)(YOUT_H << 8 | YOUT_L) / 32768.0f * 2.0f;
float accel_z_g = (int16_t)(ZOUT_H << 8 | ZOUT_L) / 32768.0f * 2.0f;

// For ±4g range:
// ... / 32768.0f * 4.0f;

// For ±8g range:
// ... / 32768.0f * 8.0f;
```

---

## MAX20303 PMIC Registers

### Identification
| Address | Register Name | Description | Expected Value |
|---------|--------------|-------------|----------------|
| 0x00 | HARDWARE_ID | Hardware ID | 0x02 |
| 0x01 | FIRMWARE_REV | Firmware Revision | Variable |

### Interrupt Registers
| Address | Register Name | Description |
|---------|--------------|-------------|
| 0x02 | INT0 | Interrupt 0 Status |
| 0x03 | INT1 | Interrupt 1 Status |
| 0x04 | INT2 | Interrupt 2 Status |

### Status Registers
| Address | Register Name | Description |
|---------|--------------|-------------|
| 0x05 | STATUS0 | Status Register 0 |
| 0x06 | STATUS1 | Status Register 1 |
| 0x07 | STATUS2 | Status Register 2 |
| 0x08 | STATUS3 | Status Register 3 |

### LDO Command Interface
| Address | Register Name | Description | Usage |
|---------|--------------|-------------|-------|
| 0x09 | AP_DATOUT0 | Application Data Output 0 | LDO voltage/enable data |
| 0x0A | AP_DATOUT1 | Application Data Output 1 | Additional data |
| 0x0B | AP_CMDOUT | Application Command Output | Command selection |

### Power Rail Control
| Address | Register Name | Description |
|---------|--------------|-------------|
| 0x13 | LDO_DIRECT | LDO Direct Control |
| 0x14 | BOOST_VSET | Boost Voltage Setting |
| 0x15 | BOOST_CFG | Boost Configuration |
| 0x16 | PWR_CFG | Power Configuration |

### ADC Registers
| Address | Register Name | Description |
|---------|--------------|-------------|
| 0x32 | ADC_DATA_H | ADC Data High Byte |
| 0x33 | ADC_DATA_L | ADC Data Low Byte |
| 0x34 | ADC_CONTROL | ADC Control Register |

### LDO Command Codes (Register 0x0B: AP_CMDOUT)
```
0x01 = LDO1 command
0x02 = LDO2 command
0x03 = LDO3 command
0x04 = Buck-Boost command
```

### LDO Voltage Codes (in AP_DATOUT0)
```
Bit 7: Enable (1=Enable, 0=Disable)
Bits 6:0: Voltage setting

Common voltages:
0x06 = 1.8V
0x0A = 3.3V

Full range: 0.8V to 3.3V in various steps
(Refer to datasheet for complete table)
```

### ADC Channel Selection (Register 0x34: ADC_CONTROL)
```
Bit 7: Start conversion (write 1)
Bits 1:0: Channel selection
  00 = VSYS
  01 = VBAT
  10 = VBUS
  11 = THM (Thermistor)
```

---

## Initialization Sequences

### MAX86171 Initialization for 100 SPS PPG Mode

```c
// 1. Soft reset
Write 0x0C = 0x01  // SYSTEM_CONTROL: RESET

// 2. Wait 10ms for reset

// 3. Verify Part ID
Read 0xFF  // Should be 0x22 or 0x23

// 4. Configure FIFO
Write 0x08 = 0x0A  // FIFO_CONFIG_1: Interrupt at 10 samples
Write 0x09 = 0x00  // FIFO_CONFIG_2: Normal mode

// 5. Configure PPG for 100 Hz
Write 0x0D = 0x03  // PPG_CONFIG_1: ADC range and integration time
Write 0x0E = 0x40  // PPG_CONFIG_2: SMP_FREQ=100Hz, no averaging
Write 0x0F = 0x00  // PPG_CONFIG_3: LED range

// 6. Set LED current (50mA with LED_RANGE=2)
Write 0x21 = 0x3F  // LED1_PA = 63
Write 0x24 = 0x02  // LED_RANGE = 2 (0.8mA per step)

// 7. Enable interrupts
Write 0x00 = 0x80  // INT_ENABLE_1: A_FULL_EN

// 8. Clear FIFO
Write 0x09 |= 0x10 // FIFO_CONFIG_2: FLUSH_FIFO

// 9. Exit shutdown mode
Write 0x0C = 0x00  // SYSTEM_CONTROL: Normal operation
```

### KX122 Initialization for 100 Hz

```c
// 1. Soft reset
Write 0x19 = 0x80  // CNTL2: SRST=1

// 2. Wait 2ms for reset

// 3. Verify WHO_AM_I
Read 0x13  // Should be 0x1A

// 4. Set to stand-by mode (PC1=0)
Write 0x18 = 0x00  // CNTL1: PC1=0

// 5. Configure accelerometer
Write 0x1B = 0x03  // ODCNTL: OSA=0011 (100 Hz)
Write 0x18 = 0x40  // CNTL1: RES=1 (high res), GSEL=00 (±2g), PC1=0

// 6. Configure interrupt
Write 0x1C = 0x20  // INC1: IEN1=1 (enable INT1 pin)
Write 0x1F = 0x10  // INC4: DRDYI1=1 (data ready on INT1)
Write 0x1D = 0x00  // INC2: Latched interrupt

// 7. Enter operating mode
Write 0x18 = 0xC0  // CNTL1: PC1=1, RES=1, GSEL=00
```

### MAX20303 Initialization

```c
// 1. Verify communication
Read 0x00  // Should be 0x02

// 2. Clear interrupts
Read 0x02, 0x03, 0x04

// 3. Enable LDO1 at 1.8V for sensors
Write 0x0B = 0x01  // AP_CMDOUT: LDO1 command
Write 0x09 = 0x86  // AP_DATOUT0: Enable + 1.8V

// 4. Enable LDO2 at 1.8V
Write 0x0B = 0x02  // AP_CMDOUT: LDO2 command
Write 0x09 = 0x86  // AP_DATOUT0: Enable + 1.8V

// 5. Wait 10ms for power stabilization

// 6. Configure Buck-Boost if needed
Write 0x14 = 0x1E  // BOOST_VSET: 3.3V
Write 0x15 = 0x01  // BOOST_CFG: Enable
```

---

## Common Register Access Patterns

### Reading Sensor Data (Burst Read)

```c
// MAX86171 FIFO Read
uint8_t fifo_data[3];  // 19-bit data + 5-bit LED number
HAL_I2C_Mem_Read(hi2c, MAX86171_ADDR, 0x07, I2C_MEMADD_SIZE_8BIT, 
                 fifo_data, 3, timeout);

// Parse 19-bit PPG value
uint32_t ppg_value = ((uint32_t)fifo_data[0] << 11) | 
                     ((uint32_t)fifo_data[1] << 3) | 
                     ((fifo_data[2] & 0xE0) >> 5);
```

```c
// KX122 XYZ Read
uint8_t accel_data[6];
HAL_I2C_Mem_Read(hi2c, KX122_ADDR, 0x06, I2C_MEMADD_SIZE_8BIT, 
                 accel_data, 6, timeout);

// Parse 16-bit values
int16_t x = (int16_t)((accel_data[1] << 8) | accel_data[0]);
int16_t y = (int16_t)((accel_data[3] << 8) | accel_data[2]);
int16_t z = (int16_t)((accel_data[5] << 8) | accel_data[4]);
```

---

## Debug Tips

### Verify I2C Communication
```c
// Scan for devices
for (uint8_t addr = 0x10; addr < 0x78; addr++) {
    if (HAL_I2C_IsDeviceReady(hi2c, addr << 1, 1, 10) == HAL_OK) {
        printf("Found device at 0x%02X\r\n", addr);
    }
}
```

### Check Register Values
```c
// Read and verify key registers
uint8_t value;

// MAX86171 Part ID (should be 0x22 or 0x23)
HAL_I2C_Mem_Read(hi2c, MAX86171_ADDR, 0xFF, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
printf("MAX86171 Part ID: 0x%02X\r\n", value);

// KX122 WHO_AM_I (should be 0x1A)
HAL_I2C_Mem_Read(hi2c, KX122_ADDR, 0x13, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
printf("KX122 WHO_AM_I: 0x%02X\r\n", value);

// MAX20303 Hardware ID (should be 0x02)
HAL_I2C_Mem_Read(hi2c, MAX20303_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
printf("MAX20303 HW ID: 0x%02X\r\n", value);
```

---

**Note**: This is a quick reference guide. For complete register descriptions and all configuration options, refer to the official datasheets.

**Last Updated**: October 2025
