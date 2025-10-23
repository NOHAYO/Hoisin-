# Wearable Sensor System Firmware Package

## 📦 Package Contents

This package contains a complete firmware architecture and code templates for a wearable sensor system that collects physiological data (PPG and accelerometer) at 100 samples per second and transmits it via BLE.

### Documentation Files

1. **firmware_architecture.md** (Comprehensive 14-section guide)
   - Complete system architecture
   - Peripheral operating modes
   - Driver architecture with APIs
   - Middleware layer design
   - Data flow diagrams
   - Timing considerations
   - Power management
   - Error handling strategies
   - Development recommendations

2. **system_flowcharts.mermaid** (Visual flowcharts)
   - System initialization flow
   - Main data acquisition loop
   - Driver state machines (MAX86171, KX122, UART)
   - Error recovery procedures
   - Power state management
   - Can be rendered in GitHub, VS Code, or online Mermaid editors

3. **QUICK_START_GUIDE.md** (Step-by-step implementation guide)
   - Project setup instructions for STM32CubeIDE and SEGGER Studio
   - Incremental testing procedures
   - Troubleshooting tips
   - Performance optimization recommendations
   - Testing checklist

4. **REGISTER_MAP_REFERENCE.md** (Register-level details)
   - Complete register maps for all three peripherals
   - Configuration tables for 100 SPS
   - Initialization sequences
   - Data parsing examples
   - Debug tips

### Code Files

5. **max20303.h** - PMIC driver header
   - Complete API definitions
   - Data structure definitions
   - Register address definitions

6. **max20303.c** - PMIC driver implementation
   - Full working driver with all functions implemented
   - I2C communication layer
   - LDO control functions
   - ADC reading functions

7. **example_main.c** - Complete application example
   - System initialization
   - Sensor integration
   - Data acquisition loop
   - UART packet transmission
   - Error handling
   - Timer interrupt handling

## 🎯 Project Overview

### Hardware Components
- **MCU**: STM32 (Any series compatible with STM32CubeIDE)
- **BLE Module**: nRF52 Series
- **PMIC**: MAX20303 (Power Management)
- **Optical Sensor**: MAX86171 (PPG/ECG)
- **Accelerometer**: KX122-1037 (3-axis)
- **Communication**: I2C (sensors to STM32), UART (STM32 to nRF52)

### Key Features
- **Sample Rate**: 100 samples per second (both sensors)
- **Data Synchronization**: Timestamp-aligned PPG and accelerometer data
- **Buffering**: Ring buffer with overflow detection
- **UART Protocol**: Packetized data with CRC16 error detection
- **Power Management**: Intelligent LDO control via PMIC
- **Error Recovery**: Comprehensive error handling and retry logic

### System Architecture

```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│  MAX86171    │     │    KX122     │     │  MAX20303    │
│  (PPG)       │────▶│  (Accel)     │────▶│  (PMIC)      │
│  100 SPS     │ I2C │  100 SPS     │ I2C │  Power Mgmt  │
└──────────────┘     └──────────────┘     └──────┬───────┘
       │                    │                     │
       └────────────────────┴─────────────────────┘
                            │ I2C
                            ▼
                   ┌─────────────────┐
                   │   STM32 MCU     │
                   │  - Sync data    │
                   │  - Buffer       │
                   │  - Protocol     │
                   └────────┬────────┘
                            │ UART 115200
                            ▼
                   ┌─────────────────┐
                   │   NRF52 BLE     │
                   │  - Parse UART   │
                   │  - BLE Service  │
                   └────────┬────────┘
                            │ BLE
                            ▼
                   ┌─────────────────┐
                   │  Mobile App     │
                   └─────────────────┘
```

## 🚀 Quick Start

### Prerequisites
- **STM32CubeIDE** installed
- **SEGGER Embedded Studio** installed
- **Hardware**: Development boards with STM32 and nRF52
- **Tools**: Logic analyzer (recommended), UART terminal

### Implementation Steps

1. **Read the Documentation**
   ```
   Start with: QUICK_START_GUIDE.md
   Reference: firmware_architecture.md
   Lookup: REGISTER_MAP_REFERENCE.md
   ```

2. **Set Up STM32CubeIDE Project**
   - Create new project for your STM32 MCU
   - Configure peripherals using CubeMX:
     - I2C1: 400 kHz
     - USART1: 115200 baud
     - TIM2: 100 Hz interrupt
     - GPIO: Interrupt pins
   - Copy driver files to project

3. **Implement Missing Drivers**
   - Use `max20303.c` as a template
   - Create `max86171.c` and `max86171.h`
   - Create `kx122.c` and `kx122.h`
   - Reference REGISTER_MAP_REFERENCE.md for register details

4. **Integrate Application**
   - Use `example_main.c` as your starting point
   - Modify for your specific pin assignments
   - Add error handling as needed

5. **Set Up nRF52 Project**
   - Create BLE service for sensor data
   - Implement UART receiver
   - Set up BLE notifications

6. **Test Incrementally**
   - Phase 1: PMIC and power (use provided driver)
   - Phase 2: Individual sensor communication
   - Phase 3: Data acquisition
   - Phase 4: UART transmission
   - Phase 5: BLE end-to-end

## 📋 Implementation Checklist

### STM32 Firmware
- [ ] Copy `max20303.h` and `max20303.c` to project
- [ ] Implement `max86171.c` driver (follow max20303 pattern)
- [ ] Implement `kx122.c` driver (follow max20303 pattern)
- [ ] Adapt `example_main.c` for your project
- [ ] Configure STM32CubeMX peripherals
- [ ] Verify I2C addresses for your hardware
- [ ] Test PMIC initialization
- [ ] Test sensor initialization
- [ ] Verify 100 Hz sampling timing
- [ ] Test UART packet transmission

### nRF52 Firmware
- [ ] Create BLE service with custom UUIDs
- [ ] Implement UART receive handler
- [ ] Create packet parser
- [ ] Set up BLE characteristics (PPG, Accel, Status)
- [ ] Implement BLE notifications
- [ ] Test UART-to-BLE data flow

### Integration Testing
- [ ] End-to-end data flow working
- [ ] Data rate stable at 100 SPS
- [ ] BLE connection stable
- [ ] Error recovery working
- [ ] Battery monitoring functional
- [ ] Long-duration stability test (>1 hour)

## 🔧 Key Configuration Parameters

### I2C Addresses (Verify with your hardware)
```c
#define MAX20303_I2C_ADDR   0x50  // Fixed
#define MAX86171_I2C_ADDR   0x62  // or 0x63 (check AD pin)
#define KX122_I2C_ADDR      0x1E  // or 0x1F (check SDO pin)
```

### Sample Rate Configuration
```c
#define SAMPLE_RATE_HZ      100
#define SAMPLE_PERIOD_MS    10
```

### UART Configuration
```c
#define UART_BAUD_RATE      115200
#define PACKET_HEADER       0xAA55
```

### Power Configuration
```c
// LDO voltages for sensors
#define SENSOR_LDO_VOLTAGE  1800  // 1.8V
```

## 📊 Expected Performance

### Data Throughput
- **Raw data rate**: 100 samples/sec × ~20 bytes = 2 kB/s
- **UART capacity**: 115200 baud ≈ 11 kB/s (sufficient)
- **BLE throughput**: 5-10 kbps typical (may need buffering)

### Timing
- **Sample period**: 10 ms
- **I2C transaction time**: < 1 ms per cycle
- **UART packet time**: ~4 ms
- **Total cycle time**: ~6 ms (60% utilization)

### Power Consumption
- **Active mode**: 10-20 mA
- **Idle mode**: 1-5 mA
- **Deep sleep**: < 10 µA

## 📚 Additional Resources

### Datasheets
- Refer to uploaded PDF files for complete specifications
- MAX20303 PMIC
- MAX86171 Optical Sensor
- KX122-1037 Accelerometer

### Development Tools
- **STM32CubeIDE**: https://www.st.com/en/development-tools/stm32cubeide.html
- **SEGGER Embedded Studio**: https://www.segger.com/products/development-tools/embedded-studio/
- **nRF5 SDK**: https://www.nordicsemi.com/Products/Development-software/nRF5-SDK

### Debug Tools
- Logic analyzer for I2C/UART debugging
- Oscilloscope for signal verification
- UART terminal (PuTTY, Tera Term)
- nRF Connect mobile app for BLE testing

## 🐛 Troubleshooting

### Common Issues

**I2C Communication Fails**
- Check pull-up resistors (2.2kΩ - 4.7kΩ)
- Verify I2C clock speed (400 kHz)
- Check pin assignments
- Verify device addresses with logic analyzer

**Sensors Not Responding**
- Verify PMIC LDOs are enabled
- Measure voltage at sensor power pins (should be 1.8V)
- Check I2C addresses
- Try power cycle

**No UART Data**
- Test UART loopback first
- Verify baud rate on both ends
- Check TX/RX pin assignments
- Use terminal to verify data

**Timing Issues**
- Verify timer configuration
- Check interrupt priorities
- Monitor actual sample rate
- Look for I2C blocking issues

## ⚠️ Important Notes

1. **Power Sequencing**: Always initialize PMIC first and wait for power stabilization before accessing sensors
2. **I2C Speed**: Use 400 kHz for best performance
3. **Error Handling**: Always check return values and implement retry logic
4. **Watchdog**: Implement watchdog timer for production
5. **Testing**: Test each component independently before integration

## 📝 What You Need to Implement

This package provides:
- ✅ Complete architecture and design
- ✅ MAX20303 PMIC driver (fully implemented)
- ✅ Example main application
- ✅ UART protocol structure
- ✅ Detailed register maps
- ✅ Initialization sequences

You need to implement:
- ⬜ MAX86171 driver (follow max20303.c pattern)
- ⬜ KX122 driver (follow max20303.c pattern)
- ⬜ nRF52 UART receiver
- ⬜ nRF52 BLE service
- ⬜ Hardware-specific pin assignments

## 🎓 Learning Path

1. **Understand the Architecture** → Read firmware_architecture.md
2. **Study the Example** → Review example_main.c
3. **Learn the Registers** → Study REGISTER_MAP_REFERENCE.md
4. **Follow the Guide** → Work through QUICK_START_GUIDE.md
5. **Implement Drivers** → Use max20303.c as template
6. **Test Incrementally** → Follow testing checklist

## 📞 Support

For questions about:
- **STM32**: ST Community forums
- **nRF52**: Nordic DevZone
- **General embedded**: Stack Overflow

## 📄 License

This code is provided as templates for educational and development purposes.
Review and adapt for your specific requirements.

---

**Version**: 1.0  
**Created**: October 2025  
**Status**: Architecture Complete, Ready for Implementation
