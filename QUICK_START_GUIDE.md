# Wearable Sensor System - Quick Start Guide

## 📋 Project Summary

This project implements a complete firmware solution for a wearable sensor system that:
- Collects PPG (photoplethysmography) data from MAX86171 at 100 SPS
- Collects 3-axis accelerometer data from KX122-1037 at 100 SPS
- Manages power via MAX20303 PMIC
- Transmits synchronized data via UART to NRF52 for BLE transmission

## 🗂️ Generated Files

### Documentation
1. **firmware_architecture.md** - Comprehensive architecture document with:
   - System overview
   - Peripheral operating modes
   - Driver architecture
   - Data flow diagrams
   - Configuration details
   - Development recommendations

2. **system_flowcharts.mermaid** - Visual flowcharts including:
   - System initialization flow
   - Main data acquisition loop
   - Driver state machines
   - UART protocol handling
   - Error recovery procedures
   - Power state management

### Driver Code Templates
3. **max20303.h** - PMIC driver header
4. **max20303.c** - PMIC driver implementation
5. **example_main.c** - Complete example application integrating all drivers

## 🚀 Getting Started

### Step 1: Set Up STM32CubeIDE Project

1. **Create New Project**
   ```
   File → New → STM32 Project
   Select your STM32 MCU (e.g., STM32F103, STM32L4, etc.)
   ```

2. **Configure Peripherals in CubeMX**
   - **I2C1**: 400 kHz Fast Mode
     - SCL: PB6 (or your pin)
     - SDA: PB7 (or your pin)
   
   - **USART1**: 115200 baud, 8N1
     - TX: PA9
     - RX: PA10
   
   - **TIM2**: 100 Hz timer interrupt
     - Clock source: Internal
     - Prescaler & Period: Configure for 100 Hz
   
   - **GPIO**: Configure interrupt pins
     - PA0: MAX86171 interrupt (EXTI0)
     - PA1: KX122 interrupt (EXTI1)

3. **Project Structure**
   ```
   YourProject/
   ├── Core/
   │   ├── Inc/
   │   │   └── main.h
   │   └── Src/
   │       ├── main.c          (use example_main.c as template)
   │       └── stm32xxxx_it.c
   ├── Drivers/
   │   ├── PMIC/
   │   │   ├── max20303.h
   │   │   └── max20303.c
   │   ├── PPG_Sensor/
   │   │   ├── max86171.h      (you need to create)
   │   │   ├── max86171.c      (you need to create)
   │   │   └── max86171_regs.h (you need to create)
   │   └── Accelerometer/
   │       ├── kx122.h         (you need to create)
   │       ├── kx122.c         (you need to create)
   │       └── kx122_regs.h    (you need to create)
   └── Middlewares/
       ├── sensor_manager.c/h  (optional, for advanced integration)
       └── uart_protocol.c/h   (optional, for complex protocols)
   ```

### Step 2: Implement Remaining Drivers

Based on the MAX20303 template provided, you need to create:

1. **MAX86171 Driver** (PPG Sensor)
   - Follow same pattern as max20303.c
   - Key registers to implement:
     - FIFO configuration (0x08-0x0A)
     - PPG configuration (0x0D-0x10)
     - LED configuration (0x21-0x2F)
     - Interrupt enable (0x00-0x01)
   - Sample rate configuration for 100 Hz
   - FIFO read functions
   - LED current control

2. **KX122 Driver** (Accelerometer)
   - Follow same pattern as max20303.c
   - Key registers:
     - XOUT_L/H, YOUT_L/H, ZOUT_L/H (0x06-0x0B)
     - CNTL1 (0x18): Control register
     - ODCNTL (0x1B): Output data rate control
     - INC1-4 (0x1C-0x1F): Interrupt control
     - BUF_CNTL1/2 (0x3A-0x3B): Buffer control
   - ODR configuration for 100 Hz
   - Range setting (±2g/±4g/±8g)
   - High resolution mode
   - Data ready interrupt

### Step 3: Verify I2C Addresses

Check your hardware schematic for the actual I2C addresses:

```c
// In your code, verify these addresses match your hardware
#define MAX20303_I2C_ADDR   0x50  // PMIC - usually fixed
#define MAX86171_I2C_ADDR   0x62  // or 0x63, check AD0/AD1 pins
#define KX122_I2C_ADDR      0x1E  // or 0x1F, check SDO/SA0 pin
```

### Step 4: Build and Test Incrementally

#### Phase 1: PMIC Test
```c
// Test code in main()
MAX20303_Handle_t hpmic;
hpmic.hi2c = &hi2c1;
hpmic.device_addr = MAX20303_I2C_ADDR;

if (MAX20303_Init(&hpmic) == HAL_OK) {
    printf("PMIC OK\r\n");
    MAX20303_EnableLDO(&hpmic, PMIC_LDO1, 1800);
    MAX20303_EnableLDO(&hpmic, PMIC_LDO2, 1800);
}
```

#### Phase 2: Sensor Communication Test
```c
// After PMIC powers up sensors
MAX86171_Handle_t hppg;
hppg.hi2c = &hi2c1;
hppg.device_addr = MAX86171_I2C_ADDR;

if (MAX86171_Init(&hppg) == HAL_OK) {
    printf("PPG Sensor OK\r\n");
}

KX122_Handle_t haccel;
haccel.hi2c = &hi2c1;
haccel.device_addr = KX122_I2C_ADDR;

if (KX122_Init(&haccel) == HAL_OK) {
    printf("Accelerometer OK\r\n");
}
```

#### Phase 3: Data Acquisition Test
```c
// Start sampling
MAX86171_StartSampling(&hppg);
KX122_StartSampling(&haccel);

// In main loop
PPG_Sample_t ppg_sample;
Accel_Sample_t accel_sample;

if (MAX86171_ReadFIFO(&hppg, &ppg_sample, 1, &num_read) == HAL_OK) {
    printf("PPG: %lu\r\n", ppg_sample.ppg_data);
}

if (KX122_ReadSample(&haccel, &accel_sample) == HAL_OK) {
    printf("Accel: X=%d Y=%d Z=%d\r\n", 
           accel_sample.x, accel_sample.y, accel_sample.z);
}
```

### Step 5: Set Up NRF52 (SEGGER Embedded Studio)

1. **Create New Project**
   ```
   File → New Project
   Nordic Semiconductor → nRF5 SDK Project
   Select: nRF52 DK/Device
   ```

2. **Configure BLE Service**
   ```c
   // Create custom BLE service for sensor data
   // Follow Nordic SDK examples (ble_app_uart, ble_app_template)
   
   // Define UUIDs
   #define BLE_UUID_SENSOR_SERVICE    0x1800
   #define BLE_UUID_PPG_CHAR          0x2A01
   #define BLE_UUID_ACCEL_CHAR        0x2A02
   ```

3. **Implement UART Receiver**
   ```c
   // Initialize UART
   // Set up RX interrupt/DMA
   // Parse incoming packets
   // Update BLE characteristics
   // Send notifications to connected devices
   ```

## 📊 Expected Data Flow

```
[MAX86171] --\                    
              |-- I2C --> [STM32] -- UART --> [NRF52] -- BLE --> [Mobile App]
[KX122]    --/            ^
                          |
[MAX20303] -- I2C --------/
   (Power)
```

**Data Rate Calculation:**
- Each sample: ~20 bytes (PPG: 4 bytes, Accel: 6 bytes, Timestamp: 4 bytes, Overhead: 6 bytes)
- 100 SPS × 20 bytes = 2000 bytes/second = 16 kbps
- UART @ 115200 baud: ~11 kB/s capacity (sufficient)
- BLE throughput: ~5-10 kbps typical (may need buffering)

## 🔧 Troubleshooting

### I2C Communication Issues
```c
// Add these debug checks
HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(&hi2c1, device_addr, 3, 100);
if (status != HAL_OK) {
    printf("I2C Error: 0x%02X\r\n", status);
    // Check: Pull-up resistors, clock speed, pin configuration
}
```

### No Data from Sensors
1. Verify PMIC powers up LDOs (use multimeter)
2. Check I2C addresses with logic analyzer
3. Verify GPIO interrupts are triggering
4. Check sensor registers are configured correctly

### UART Issues
```c
// Test UART loopback first
uint8_t test_data[] = "Hello\r\n";
HAL_UART_Transmit(&huart1, test_data, sizeof(test_data), 100);
// Should see data in terminal
```

### Timing Issues
```c
// Verify timer is running at correct frequency
// Add counter in timer ISR
volatile uint32_t timer_count = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        timer_count++;  // Should increment at 100 Hz
    }
}
```

## 📈 Performance Optimization

### 1. Use DMA for UART
```c
HAL_UART_Transmit_DMA(&huart1, packet_bytes, sizeof(UARTPacket_t));
```

### 2. Use DMA for I2C (if available)
```c
HAL_I2C_Mem_Read_DMA(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT, data, len);
```

### 3. Optimize FIFO Usage
```c
// Read larger batches from sensor FIFOs
PPG_Sample_t ppg_samples[16];
MAX86171_ReadFIFO(&hppg, ppg_samples, 16, &num_read);
```

### 4. Power Optimization
```c
// Enter low power mode when idle
__WFI();  // Wait for interrupt
```

## 🧪 Testing Checklist

- [ ] PMIC initialization successful
- [ ] LDO outputs at correct voltage (measure with multimeter)
- [ ] MAX86171 responds to I2C commands
- [ ] KX122 responds to I2C commands
- [ ] PPG data streaming at 100 SPS
- [ ] Accelerometer data streaming at 100 SPS
- [ ] UART packets transmitting correctly
- [ ] NRF52 receiving UART data
- [ ] BLE notifications working
- [ ] Mobile app receiving data
- [ ] System runs for extended period without errors
- [ ] Battery monitoring working
- [ ] Error recovery mechanisms functioning

## 📚 Additional Resources

1. **Datasheets** (refer to uploaded PDFs)
   - MAX20303 PMIC
   - MAX86171 PPG/ECG Sensor
   - KX122-1037 Accelerometer

2. **Reference Manuals**
   - STM32 HAL documentation
   - Nordic nRF5 SDK documentation

3. **Tools**
   - Logic analyzer (for I2C debugging)
   - Oscilloscope (for signal verification)
   - UART terminal (e.g., PuTTY, Tera Term)
   - nRF Connect app (for BLE testing)

## ⚠️ Important Notes

1. **Power Sequencing**: Always enable PMIC and wait for stabilization before accessing sensors
2. **I2C Speed**: Use 400 kHz for optimal performance
3. **Interrupt Priority**: Set timer interrupt lower priority than I2C/UART
4. **Watchdog**: Implement watchdog timer for production use
5. **Error Handling**: Always check return values and implement retry logic
6. **CRC Validation**: Implement CRC checking on UART packets
7. **Buffer Management**: Monitor buffer levels to prevent overflow

## 🎯 Next Steps

1. Complete MAX86171 and KX122 drivers following the MAX20303 template
2. Test each driver independently
3. Integrate all drivers using the example_main.c template
4. Implement NRF52 UART receiver and BLE service
5. Test end-to-end data flow
6. Optimize for power consumption
7. Add error handling and recovery mechanisms
8. Perform long-duration testing

---

**Project Status**: Firmware architecture complete, driver templates provided  
**Created**: October 2025  
**Version**: 1.0
