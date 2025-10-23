# Firmware Architecture for Wearable Sensor System

## System Overview

**Target Hardware:**
- **MCU**: STM32 (STM32CubeIDE)
- **BLE Module**: NRF52 (SEGGER Embedded Studio)
- **Communication**: UART between STM32 and NRF52

**Peripherals:**
1. **MAX20303** - PMIC (I2C)
2. **MAX86171** - Optical PPG/ECG Sensor (I2C)
3. **KX122-1037** - 3-Axis Accelerometer (I2C)

**System Requirements:**
- Sample Rate: 100 SPS (Samples Per Second)
- Data Flow: Sensors → STM32 → UART → NRF52 → BLE

---

## 1. Peripheral Modes of Operation

### 1.1 MAX20303 PMIC

**Key Features:**
- I2C Interface (address: 0x50)
- Multiple LDO outputs (LDO1, LDO2, LDO3)
- Buck-Boost regulator
- Battery charger
- Power path management
- Integrated ADC for monitoring

**Operating Modes:**

1. **Initialization Mode**
   - Configure power rails
   - Set LDO voltages
   - Enable required outputs
   - Configure battery charging parameters

2. **Active Mode**
   - All required power rails enabled
   - Battery charging active (if connected)
   - Monitoring via ADC

3. **Low Power Mode**
   - Disable unused LDOs
   - Reduce quiescent current
   - Keep critical rails active

**Recommended Power Distribution:**
- **LDO1**: MAX86171 (1.8V or 3.3V)
- **LDO2**: KX122 (1.8V or 3.3V)
- **LDO3**: STM32 peripherals
- **Buck-Boost**: Main system power

---

### 1.2 MAX86171 Optical Sensor

**Key Features:**
- I2C Interface (address: 0x62 or 0x63)
- Integrated LED drivers
- 19-bit ADC
- 256-sample FIFO
- Programmable sample rate: 10-4096 Hz
- Interrupts for FIFO management

**Operating Modes:**

1. **Shutdown Mode**
   - Minimal power consumption (~10 µA)
   - I2C interface active
   - Fast wake-up

2. **PPG Mode (Single LED)**
   - One LED active
   - Continuous sampling
   - FIFO buffering
   - Typical for heart rate monitoring

3. **Multi-LED PPG Mode**
   - Multiple LEDs time-multiplexed
   - Enhanced signal quality
   - SpO2 measurement capable

4. **ECG Mode**
   - External electrode inputs
   - Differential measurement
   - Lower sample rates typical

**Configuration for 100 SPS:**
- PPG Configuration 1 Register (0x0D): Set PPG_ADC_RGE, PPG_TINT
- PPG Configuration 2 Register (0x0E): Set SMP_AVG, SMP_FREQ for 100 Hz
- FIFO Configuration: Enable FIFO_A_FULL interrupt at appropriate threshold
- LED Configuration: Set LED current and pulse width

---

### 1.3 KX122-1037 Accelerometer

**Key Features:**
- I2C Interface (address: 0x1E or 0x1F)
- Selectable ranges: ±2g, ±4g, ±8g
- Output Data Rates: 0.781 Hz to 25.6 kHz
- 1024-sample buffer
- Interrupts for data ready and buffer watermark
- Low power modes

**Operating Modes:**

1. **Stand-by Mode**
   - Low power state
   - Configuration changes allowed
   - Fast transition to active

2. **Low Power Mode**
   - Reduced ODR (< 100 Hz)
   - Lower current consumption
   - Suitable for motion detection

3. **High Resolution Mode**
   - Full 16-bit data
   - Higher current consumption
   - Best performance

4. **Buffer Mode**
   - FIFO operation
   - Watermark interrupts
   - Batch data collection

**Configuration for 100 SPS:**
- ODCNTL Register: Set OSA[3:0] = 0b0010 for 100 Hz ODR
- CNTL1 Register: Set GSEL for ±2g/±4g/±8g, set RES=1 for high resolution
- CNTL1 Register: Set PC1=1 to enter operating mode
- BUF_CNTL2 Register: Set watermark for buffered operation
- INC1/INC4 Registers: Enable interrupt routing

---

## 2. System Initialization Flowchart

```
┌─────────────────────────────────────┐
│      SYSTEM POWER ON / RESET        │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│    STM32 HAL & Clock Init           │
│    - System Clock Config            │
│    - GPIO Init                      │
│    - I2C Init (100/400 kHz)         │
│    - UART Init (115200 baud)        │
│    - Timer Init (for timing)        │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│    Initialize MAX20303 PMIC         │
│    1. Verify I2C communication      │
│    2. Read Device ID                │
│    3. Configure LDO1 (Sensor 1.8V)  │
│    4. Configure LDO2 (Sensor 1.8V)  │
│    5. Configure Buck-Boost          │
│    6. Enable power outputs          │
│    7. Wait for power stabilization  │
└──────────────┬──────────────────────┘
               │
               ▼ (Wait 10-50ms)
               │
┌─────────────────────────────────────┐
│    Initialize MAX86171 PPG Sensor   │
│    1. Verify I2C communication      │
│    2. Software reset                │
│    3. Read Part ID                  │
│    4. Configure FIFO (128 samples)  │
│    5. Set sample rate to 100 Hz     │
│    6. Configure PPG mode            │
│    7. Set LED current               │
│    8. Enable interrupts             │
│    9. Clear FIFO                    │
│   10. Start sampling                │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│    Initialize KX122 Accelerometer   │
│    1. Verify I2C communication      │
│    2. Software reset                │
│    3. Read WHO_AM_I (0x1A)          │
│    4. Set to stand-by mode          │
│    5. Configure ODR = 100 Hz        │
│    6. Set range (±2g/±4g/±8g)       │
│    7. Enable high resolution        │
│    8. Configure buffer/FIFO         │
│    9. Enable data ready interrupt   │
│   10. Set to operating mode         │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│    Synchronization & Calibration    │
│    - Wait for first valid samples   │
│    - Verify sample timing           │
│    - Start main data acquisition    │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│      ENTER MAIN LOOP                │
└─────────────────────────────────────┘
```

---

## 3. Main Data Acquisition Loop Flowchart

```
┌─────────────────────────────────────┐
│         MAIN LOOP START             │
└──────────────┬──────────────────────┘
               │
               ▼
        ┌──────────────┐
        │  Wait for     │◄────────────┐
        │  Timer/Event  │             │
        └──────┬────────┘             │
               │                      │
               ▼                      │
        ┌─────────────────┐           │
        │ Check MAX86171  │           │
        │ FIFO Status     │           │
        └──────┬──────────┘           │
               │                      │
               ▼                      │
        ┌─────────────────┐           │
     ┌──┤ FIFO Ready?     │           │
     │  │ (≥10 samples)   │           │
     │  └──────┬──────────┘           │
     │         │ Yes                  │
     │ No      ▼                      │
     │  ┌─────────────────────────┐   │
     │  │ Read MAX86171 FIFO      │   │
     │  │ - Read n samples        │   │
     │  │ - Parse 19-bit data     │   │
     │  │ - Store in buffer       │   │
     │  └──────┬──────────────────┘   │
     │         │                      │
     │         ▼                      │
     │  ┌─────────────────────────┐   │
     │  │ Check KX122 Status      │   │
     │  └──────┬──────────────────┘   │
     │         │                      │
     │         ▼                      │
     │  ┌─────────────────────────┐   │
     └─►│ Data Ready?             │   │
        │ (DRDY interrupt or poll)│   │
        └──────┬──────────────────┘   │
               │ Yes                  │
               ▼                      │
        ┌─────────────────────────┐   │
        │ Read KX122 Data         │   │
        │ - Read XOUT/YOUT/ZOUT   │   │
        │ - Convert to g values   │   │
        │ - Store in buffer       │   │
        └──────┬──────────────────┘   │
               │                      │
               ▼                      │
        ┌─────────────────────────┐   │
        │ Synchronize Data        │   │
        │ - Timestamp samples     │   │
        │ - Align PPG and Accel   │   │
        └──────┬──────────────────┘   │
               │                      │
               ▼                      │
        ┌─────────────────────────┐   │
        │ Package Data            │   │
        │ - Create data packet    │   │
        │ - Add header/CRC        │   │
        │ - Prepare for UART TX   │   │
        └──────┬──────────────────┘   │
               │                      │
               ▼                      │
        ┌─────────────────────────┐   │
        │ Transmit via UART       │   │
        │ - Send to NRF52         │   │
        │ - Wait for completion   │   │
        └──────┬──────────────────┘   │
               │                      │
               ▼                      │
        ┌─────────────────────────┐   │
        │ Check System Status     │   │
        │ - Battery level (PMIC)  │   │
        │ - Error conditions      │   │
        │ - Power management      │   │
        └──────┬──────────────────┘   │
               │                      │
               └──────────────────────┘
```

---

## 4. Driver Architecture

### 4.1 Layered Architecture

```
┌─────────────────────────────────────────────────────┐
│              APPLICATION LAYER                       │
│  - Main loop                                         │
│  - Data synchronization                              │
│  - Communication protocol                            │
└────────────┬────────────────────────────────────────┘
             │
┌────────────┴────────────────────────────────────────┐
│            MIDDLEWARE LAYER                          │
│  - Sensor Manager                                    │
│  - Data Buffer Manager                               │
│  - Power Manager                                     │
│  - Communication Manager (UART Protocol)             │
└────────────┬────────────────────────────────────────┘
             │
┌────────────┴────────────────────────────────────────┐
│              DRIVER LAYER                            │
│  ┌──────────────┐ ┌──────────────┐ ┌─────────────┐ │
│  │  MAX20303    │ │  MAX86171    │ │   KX122     │ │
│  │  Driver      │ │  Driver      │ │   Driver    │ │
│  └──────────────┘ └──────────────┘ └─────────────┘ │
└────────────┬────────────────────────────────────────┘
             │
┌────────────┴────────────────────────────────────────┐
│              HAL LAYER (STM32)                       │
│  - I2C HAL                                           │
│  - UART HAL                                          │
│  - GPIO HAL                                          │
│  - Timer HAL                                         │
│  - Interrupt HAL                                     │
└──────────────────────────────────────────────────────┘
```

---

## 5. STM32 Firmware Structure

### 5.1 Project Organization

```
STM32_Firmware/
├── Core/
│   ├── Inc/
│   │   ├── main.h
│   │   ├── stm32xxxx_it.h
│   │   └── stm32xxxx_hal_conf.h
│   └── Src/
│       ├── main.c
│       ├── stm32xxxx_it.c
│       └── system_stm32xxxx.c
├── Drivers/
│   ├── PMIC/
│   │   ├── max20303.h
│   │   └── max20303.c
│   ├── PPG_Sensor/
│   │   ├── max86171.h
│   │   ├── max86171.c
│   │   └── max86171_regs.h
│   └── Accelerometer/
│       ├── kx122.h
│       ├── kx122.c
│       └── kx122_regs.h
├── Middleware/
│   ├── sensor_manager.h
│   ├── sensor_manager.c
│   ├── data_buffer.h
│   ├── data_buffer.c
│   ├── power_manager.h
│   ├── power_manager.c
│   ├── uart_protocol.h
│   └── uart_protocol.c
├── Application/
│   ├── app_main.h
│   └── app_main.c
└── STM32CubeIDE Project Files
```

### 5.2 Key Driver APIs

#### MAX20303 PMIC Driver

```c
// max20303.h
typedef enum {
    PMIC_LDO1 = 0,
    PMIC_LDO2,
    PMIC_LDO3,
    PMIC_BUCK_BOOST
} PMIC_PowerRail_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t device_addr;
    bool initialized;
} MAX20303_Handle_t;

// Public APIs
HAL_StatusTypeDef MAX20303_Init(MAX20303_Handle_t *hpmic);
HAL_StatusTypeDef MAX20303_EnableLDO(MAX20303_Handle_t *hpmic, PMIC_PowerRail_t ldo, uint16_t voltage_mv);
HAL_StatusTypeDef MAX20303_DisableLDO(MAX20303_Handle_t *hpmic, PMIC_PowerRail_t ldo);
HAL_StatusTypeDef MAX20303_ReadBatteryVoltage(MAX20303_Handle_t *hpmic, uint16_t *voltage_mv);
HAL_StatusTypeDef MAX20303_ReadBatteryCurrent(MAX20303_Handle_t *hpmic, int16_t *current_ma);
HAL_StatusTypeDef MAX20303_SetChargeCurrent(MAX20303_Handle_t *hpmic, uint16_t current_ma);
```

#### MAX86171 PPG Sensor Driver

```c
// max86171.h
typedef enum {
    PPG_MODE_NONE = 0,
    PPG_MODE_SINGLE_LED,
    PPG_MODE_MULTI_LED,
    PPG_MODE_ECG
} PPG_Mode_t;

typedef struct {
    uint32_t timestamp;
    uint32_t ppg_data;  // 19-bit data
    uint8_t led_num;
} PPG_Sample_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t device_addr;
    PPG_Mode_t mode;
    uint16_t sample_rate;
    bool fifo_enabled;
    uint8_t fifo_threshold;
    GPIO_TypeDef *int_port;
    uint16_t int_pin;
    bool initialized;
} MAX86171_Handle_t;

// Public APIs
HAL_StatusTypeDef MAX86171_Init(MAX86171_Handle_t *hppg);
HAL_StatusTypeDef MAX86171_Reset(MAX86171_Handle_t *hppg);
HAL_StatusTypeDef MAX86171_SetMode(MAX86171_Handle_t *hppg, PPG_Mode_t mode);
HAL_StatusTypeDef MAX86171_SetSampleRate(MAX86171_Handle_t *hppg, uint16_t rate_hz);
HAL_StatusTypeDef MAX86171_ConfigureFIFO(MAX86171_Handle_t *hppg, uint8_t threshold);
HAL_StatusTypeDef MAX86171_SetLEDCurrent(MAX86171_Handle_t *hppg, uint8_t led_num, uint8_t current_ma);
HAL_StatusTypeDef MAX86171_StartSampling(MAX86171_Handle_t *hppg);
HAL_StatusTypeDef MAX86171_StopSampling(MAX86171_Handle_t *hppg);
HAL_StatusTypeDef MAX86171_ReadFIFO(MAX86171_Handle_t *hppg, PPG_Sample_t *samples, uint8_t max_samples, uint8_t *num_read);
HAL_StatusTypeDef MAX86171_GetFIFOLevel(MAX86171_Handle_t *hppg, uint8_t *level);
```

#### KX122 Accelerometer Driver

```c
// kx122.h
typedef enum {
    ACCEL_RANGE_2G = 0,
    ACCEL_RANGE_4G,
    ACCEL_RANGE_8G
} Accel_Range_t;

typedef struct {
    uint32_t timestamp;
    int16_t x;  // Raw 16-bit data
    int16_t y;
    int16_t z;
} Accel_Sample_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t device_addr;
    Accel_Range_t range;
    uint16_t odr_hz;
    bool high_resolution;
    bool buffer_enabled;
    GPIO_TypeDef *int_port;
    uint16_t int_pin;
    bool initialized;
} KX122_Handle_t;

// Public APIs
HAL_StatusTypeDef KX122_Init(KX122_Handle_t *haccel);
HAL_StatusTypeDef KX122_Reset(KX122_Handle_t *haccel);
HAL_StatusTypeDef KX122_SetRange(KX122_Handle_t *haccel, Accel_Range_t range);
HAL_StatusTypeDef KX122_SetODR(KX122_Handle_t *haccel, uint16_t odr_hz);
HAL_StatusTypeDef KX122_EnableHighResolution(KX122_Handle_t *haccel, bool enable);
HAL_StatusTypeDef KX122_ConfigureBuffer(KX122_Handle_t *haccel, uint16_t watermark);
HAL_StatusTypeDef KX122_StartSampling(KX122_Handle_t *haccel);
HAL_StatusTypeDef KX122_StopSampling(KX122_Handle_t *haccel);
HAL_StatusTypeDef KX122_ReadSample(KX122_Handle_t *haccel, Accel_Sample_t *sample);
HAL_StatusTypeDef KX122_ReadBuffer(KX122_Handle_t *haccel, Accel_Sample_t *samples, uint16_t max_samples, uint16_t *num_read);
HAL_StatusTypeDef KX122_DataReady(KX122_Handle_t *haccel, bool *ready);
```

---

## 6. Middleware Layer

### 6.1 Sensor Manager

```c
// sensor_manager.h
typedef struct {
    MAX20303_Handle_t pmic;
    MAX86171_Handle_t ppg;
    KX122_Handle_t accel;
    bool all_initialized;
} SensorManager_t;

typedef struct {
    uint32_t timestamp;
    PPG_Sample_t ppg;
    Accel_Sample_t accel;
    bool ppg_valid;
    bool accel_valid;
} SyncedSample_t;

// APIs
HAL_StatusTypeDef SensorManager_Init(SensorManager_t *mgr, I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef SensorManager_StartAcquisition(SensorManager_t *mgr);
HAL_StatusTypeDef SensorManager_StopAcquisition(SensorManager_t *mgr);
HAL_StatusTypeDef SensorManager_GetSyncedSample(SensorManager_t *mgr, SyncedSample_t *sample);
HAL_StatusTypeDef SensorManager_CheckHealth(SensorManager_t *mgr);
```

### 6.2 Data Buffer Manager

```c
// data_buffer.h
#define DATA_BUFFER_SIZE 256

typedef struct {
    SyncedSample_t buffer[DATA_BUFFER_SIZE];
    uint16_t write_idx;
    uint16_t read_idx;
    uint16_t count;
    bool overflow;
} DataBuffer_t;

// APIs
void DataBuffer_Init(DataBuffer_t *buf);
bool DataBuffer_Push(DataBuffer_t *buf, SyncedSample_t *sample);
bool DataBuffer_Pop(DataBuffer_t *buf, SyncedSample_t *sample);
uint16_t DataBuffer_GetCount(DataBuffer_t *buf);
bool DataBuffer_IsFull(DataBuffer_t *buf);
bool DataBuffer_IsEmpty(DataBuffer_t *buf);
void DataBuffer_Clear(DataBuffer_t *buf);
```

### 6.3 UART Protocol

```c
// uart_protocol.h
#define PACKET_HEADER 0xAA55
#define MAX_PACKET_SIZE 64

typedef enum {
    PKT_TYPE_SYNCED_DATA = 0x01,
    PKT_TYPE_STATUS = 0x02,
    PKT_TYPE_ERROR = 0x03,
    PKT_TYPE_ACK = 0x04
} PacketType_t;

typedef struct __attribute__((packed)) {
    uint16_t header;        // 0xAA55
    uint8_t packet_type;
    uint8_t length;
    uint32_t sequence;
    uint8_t data[MAX_PACKET_SIZE];
    uint16_t crc16;
} UARTPacket_t;

// APIs
HAL_StatusTypeDef UART_SendSyncedData(UART_HandleTypeDef *huart, SyncedSample_t *sample);
HAL_StatusTypeDef UART_SendStatus(UART_HandleTypeDef *huart, uint8_t *status_data, uint8_t len);
HAL_StatusTypeDef UART_PacketBuilder(PacketType_t type, uint8_t *data, uint8_t len, UARTPacket_t *packet);
uint16_t UART_CalculateCRC(uint8_t *data, uint16_t length);
```

---

## 7. NRF52 Firmware Structure

### 7.1 Project Organization

```
NRF52_Firmware/
├── main.c
├── ble_service/
│   ├── ble_sensor_service.h
│   ├── ble_sensor_service.c
│   └── ble_sensor_service_uuid.h
├── uart_handler/
│   ├── uart_handler.h
│   └── uart_handler.c
├── data_parser/
│   ├── data_parser.h
│   └── data_parser.c
├── config/
│   └── sdk_config.h
└── SEGGER Embedded Studio Project Files
```

### 7.2 BLE Service Architecture

```c
// ble_sensor_service.h
#define BLE_SENSOR_SERVICE_UUID_BASE {0x12, 0x34, 0x56, 0x78, ...}
#define BLE_SENSOR_SERVICE_UUID 0x1800
#define BLE_SENSOR_PPG_CHAR_UUID 0x2A01
#define BLE_SENSOR_ACCEL_CHAR_UUID 0x2A02
#define BLE_SENSOR_STATUS_CHAR_UUID 0x2A03

typedef struct {
    uint16_t service_handle;
    ble_gatts_char_handles_t ppg_char_handles;
    ble_gatts_char_handles_t accel_char_handles;
    ble_gatts_char_handles_t status_char_handles;
    uint16_t conn_handle;
    bool notifications_enabled;
} ble_sensor_service_t;

// APIs
uint32_t ble_sensor_service_init(ble_sensor_service_t *p_service);
uint32_t ble_sensor_service_ppg_update(ble_sensor_service_t *p_service, PPG_Sample_t *ppg_data);
uint32_t ble_sensor_service_accel_update(ble_sensor_service_t *p_service, Accel_Sample_t *accel_data);
uint32_t ble_sensor_service_status_update(ble_sensor_service_t *p_service, uint8_t *status, uint8_t len);
```

---

## 8. Data Flow Summary

```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│  MAX86171    │     │    KX122     │     │  MAX20303    │
│  (PPG)       │     │  (Accel)     │     │  (PMIC)      │
└──────┬───────┘     └──────┬───────┘     └──────┬───────┘
       │ I2C                │ I2C                 │ I2C
       │ 100 SPS            │ 100 SPS             │ Monitoring
       │                    │                     │
       └────────┬───────────┴─────────────────────┘
                │
                ▼
       ┌─────────────────┐
       │   STM32 MCU     │
       │  - Read sensors │
       │  - Sync data    │
       │  - Buffer       │
       └────────┬────────┘
                │ UART (115200 baud)
                │ Packetized data
                ▼
       ┌─────────────────┐
       │   NRF52 BLE     │
       │  - Parse UART   │
       │  - BLE notify   │
       └────────┬────────┘
                │ BLE
                │ Notifications
                ▼
       ┌─────────────────┐
       │  Mobile App /   │
       │  BLE Central    │
       └─────────────────┘
```

---

## 9. Timing Considerations

### 9.1 Sample Timing (100 SPS)

- **Period**: 10 ms between samples
- **Jitter tolerance**: ±1 ms
- **Synchronization**: Use STM32 hardware timer for precise timing

### 9.2 I2C Transaction Times

- **I2C Clock**: 400 kHz (Fast Mode recommended)
- **Single register read**: ~50 µs
- **FIFO read (10 samples)**: ~500 µs
- **Accel XYZ read**: ~100 µs
- **Total I2C overhead per cycle**: < 1 ms

### 9.3 UART Transmission

- **Baud Rate**: 115200 bps
- **Packet size**: ~50 bytes
- **Transmission time**: ~4.3 ms per packet
- **Buffering**: Required to handle burst data

### 9.4 BLE Transmission

- **Connection interval**: 7.5-20 ms (configurable)
- **Notification size**: 20 bytes (can use long writes for more)
- **Throughput**: ~5-10 kbps typical

---

## 10. Power Management Strategy

### 10.1 Power States

1. **Active State**
   - All sensors sampling at 100 SPS
   - UART active
   - BLE connected and notifying
   - **Current**: 10-20 mA

2. **Idle State**
   - Sensors in low power mode
   - UART standby
   - BLE advertising
   - **Current**: 1-5 mA

3. **Deep Sleep State**
   - All sensors off via PMIC
   - STM32 in STOP mode
   - NRF52 in system-off
   - **Current**: < 10 µA

### 10.2 Power Optimization

- Use MAX86171 FIFO to reduce I2C transactions
- Use KX122 buffer mode
- Batch UART transmissions
- Use BLE connection intervals wisely
- Monitor battery via PMIC ADC
- Implement low battery warnings

---

## 11. Error Handling

### 11.1 Error Types

1. **I2C Communication Errors**
   - Timeout: Retry 3 times
   - NACK: Reinitialize sensor
   - Bus lockup: Software I2C reset

2. **Sensor Errors**
   - Invalid data: Discard sample
   - FIFO overflow: Clear and restart
   - Sensor not responding: Full reinit

3. **UART Errors**
   - Overrun: Clear and continue
   - Framing: Reset UART peripheral
   - Packet corruption: Request retransmit

4. **BLE Errors**
   - Disconnection: Return to advertising
   - Notification failure: Buffer and retry

### 11.2 Watchdog Implementation

- Enable IWDG on STM32
- Refresh every main loop iteration
- Timeout: 1-2 seconds
- Reset on hang or critical error

---

## 12. Development Recommendations

### 12.1 Development Phases

**Phase 1: Hardware Validation**
1. Test PMIC power-up sequence
2. Verify I2C communication with each sensor
3. Test interrupt lines

**Phase 2: Individual Driver Development**
1. Develop and test MAX20303 driver
2. Develop and test MAX86171 driver
3. Develop and test KX122 driver

**Phase 3: Integration**
1. Integrate all drivers with sensor manager
2. Test synchronized data acquisition
3. Implement UART protocol

**Phase 4: NRF52 Development**
1. UART receive and parsing
2. BLE service implementation
3. End-to-end testing

**Phase 5: Optimization**
1. Power optimization
2. Performance tuning
3. Error handling robustness

### 12.2 Debug Tools

- **STM32**: STLink debugger, UART debug output
- **NRF52**: JLink debugger, RTT (Real-Time Transfer)
- **Logic Analyzer**: Monitor I2C, UART, interrupts
- **Oscilloscope**: Verify power rails, timing

### 12.3 Testing Strategy

1. **Unit Tests**: Test each driver independently
2. **Integration Tests**: Test sensor manager with all drivers
3. **Stress Tests**: Long-duration runs, error injection
4. **Power Tests**: Measure current in all states
5. **BLE Tests**: Range, throughput, disconnection recovery

---

## 13. Configuration Summary

### 13.1 I2C Addresses

```c
#define MAX20303_I2C_ADDR   0x50  // 7-bit address
#define MAX86171_I2C_ADDR   0x62  // or 0x63, check hardware
#define KX122_I2C_ADDR      0x1E  // or 0x1F, check hardware
```

### 13.2 GPIO Pin Assignments (Example)

```c
// STM32 GPIO Configuration
#define MAX86171_INT_PIN    GPIO_PIN_0
#define MAX86171_INT_PORT   GPIOA

#define KX122_INT_PIN       GPIO_PIN_1
#define KX122_INT_PORT      GPIOA

#define I2C_SCL_PIN         GPIO_PIN_6
#define I2C_SCL_PORT        GPIOB

#define I2C_SDA_PIN         GPIO_PIN_7
#define I2C_SDA_PORT        GPIOB

#define UART_TX_PIN         GPIO_PIN_9
#define UART_TX_PORT        GPIOA

#define UART_RX_PIN         GPIO_PIN_10
#define UART_RX_PORT        GPIOA
```

### 13.3 Recommended Settings

```c
// MAX86171 Configuration
#define PPG_SAMPLE_RATE     100     // Hz
#define PPG_LED_CURRENT     50      // mA
#define PPG_FIFO_THRESHOLD  10      // samples

// KX122 Configuration
#define ACCEL_ODR           100     // Hz
#define ACCEL_RANGE         ACCEL_RANGE_2G
#define ACCEL_HIGH_RES      true

// UART Configuration
#define UART_BAUD_RATE      115200
#define UART_WORD_LENGTH    UART_WORDLENGTH_8B
#define UART_STOP_BITS      UART_STOPBITS_1
#define UART_PARITY         UART_PARITY_NONE

// Timing
#define SAMPLE_PERIOD_MS    10      // 100 SPS = 10ms period
```

---

## 14. Next Steps

1. **Set up STM32CubeIDE project**
   - Configure clocks
   - Initialize peripherals with CubeMX
   - Generate base code

2. **Set up SEGGER Embedded Studio for NRF52**
   - Import Nordic SDK
   - Configure BLE stack
   - Set up UART peripheral

3. **Begin driver development**
   - Start with PMIC (MAX20303)
   - Then sensors (MAX86171, KX122)
   - Test each independently

4. **Develop middleware layer**
   - Sensor manager
   - Data buffers
   - UART protocol

5. **Integration and testing**
   - End-to-end data flow
   - BLE transmission
   - Power optimization

---

**Document Version**: 1.0  
**Last Updated**: October 2025
