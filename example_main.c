/**
 * @file main.c
 * @brief Main Application for Wearable Sensor System
 * @description Collects data from MAX86171 PPG and KX122 Accelerometer at 100 SPS
 *              Sends data via UART to NRF52 for BLE transmission
 */

#include "main.h"
#include "max20303.h"
#include "max86171.h"
#include "kx122.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* Private defines */
#define SAMPLE_RATE_HZ          100
#define SAMPLE_PERIOD_MS        (1000 / SAMPLE_RATE_HZ)
#define DATA_BUFFER_SIZE        32

#define UART_PACKET_HEADER      0xAA55
#define PACKET_TYPE_DATA        0x01
#define PACKET_TYPE_STATUS      0x02

/* Private typedef */
typedef struct {
    uint32_t timestamp;
    uint32_t ppg_value;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
} SensorData_t;

typedef struct __attribute__((packed)) {
    uint16_t header;
    uint8_t type;
    uint8_t length;
    uint32_t sequence;
    SensorData_t data;
    uint16_t crc;
} UARTPacket_t;

/* Private variables */
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim2;

MAX20303_Handle_t hpmic;
MAX86171_Handle_t hppg;
KX122_Handle_t haccel;

SensorData_t data_buffer[DATA_BUFFER_SIZE];
volatile uint16_t buffer_write_idx = 0;
volatile uint16_t buffer_read_idx = 0;
volatile bool buffer_overflow = false;

uint32_t packet_sequence = 0;
volatile bool sample_ready = false;

/* Private function prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART1_Init(void);
static void MX_TIM2_Init(void);
void Error_Handler(void);

/* Application functions */
static HAL_StatusTypeDef Initialize_Peripherals(void);
static HAL_StatusTypeDef Start_Data_Acquisition(void);
static void Process_Sensor_Data(void);
static void Send_UART_Packet(SensorData_t *data);
static uint16_t Calculate_CRC16(uint8_t *data, uint16_t length);
static void Check_System_Health(void);

/**
 * @brief Main application entry point
 */
int main(void)
{
    HAL_StatusTypeDef status;
    
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();
    
    /* Configure the system clock */
    SystemClock_Config();
    
    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_UART1_Init();
    MX_TIM2_Init();
    
    /* Small delay for power stabilization */
    HAL_Delay(100);
    
    /* Initialize sensor peripherals */
    status = Initialize_Peripherals();
    if (status != HAL_OK) {
        Error_Handler();
    }
    
    /* Start data acquisition */
    status = Start_Data_Acquisition();
    if (status != HAL_OK) {
        Error_Handler();
    }
    
    /* Start sampling timer */
    HAL_TIM_Base_Start_IT(&htim2);
    
    printf("System initialized successfully\r\n");
    printf("Starting data acquisition at %d SPS\r\n", SAMPLE_RATE_HZ);
    
    /* Infinite loop */
    while (1)
    {
        /* Process sensor data when ready */
        if (sample_ready) {
            sample_ready = false;
            Process_Sensor_Data();
        }
        
        /* Periodically check system health */
        static uint32_t last_health_check = 0;
        if (HAL_GetTick() - last_health_check > 1000) {
            Check_System_Health();
            last_health_check = HAL_GetTick();
        }
        
        /* Check for buffer overflow */
        if (buffer_overflow) {
            printf("WARNING: Data buffer overflow!\r\n");
            buffer_overflow = false;
            // Optionally send error packet via UART
        }
    }
}

/**
 * @brief Initialize all sensor peripherals
 */
static HAL_StatusTypeDef Initialize_Peripherals(void)
{
    HAL_StatusTypeDef status;
    
    printf("Initializing peripherals...\r\n");
    
    /* Initialize MAX20303 PMIC */
    printf("  Initializing PMIC...\r\n");
    hpmic.hi2c = &hi2c1;
    hpmic.device_addr = MAX20303_I2C_ADDR;
    
    status = MAX20303_Init(&hpmic);
    if (status != HAL_OK) {
        printf("  ERROR: PMIC initialization failed!\r\n");
        return status;
    }
    
    /* Enable LDO1 for sensors (1.8V) */
    status = MAX20303_EnableLDO(&hpmic, PMIC_LDO1, 1800);
    if (status != HAL_OK) {
        printf("  ERROR: Failed to enable LDO1!\r\n");
        return status;
    }
    
    /* Enable LDO2 for sensors (1.8V) */
    status = MAX20303_EnableLDO(&hpmic, PMIC_LDO2, 1800);
    if (status != HAL_OK) {
        printf("  ERROR: Failed to enable LDO2!\r\n");
        return status;
    }
    
    printf("  PMIC initialized successfully\r\n");
    
    /* Wait for power to stabilize */
    HAL_Delay(50);
    
    /* Initialize MAX86171 PPG Sensor */
    printf("  Initializing PPG sensor...\r\n");
    hppg.hi2c = &hi2c1;
    hppg.device_addr = MAX86171_I2C_ADDR;
    hppg.int_port = GPIOA;  // Example
    hppg.int_pin = GPIO_PIN_0;
    
    status = MAX86171_Init(&hppg);
    if (status != HAL_OK) {
        printf("  ERROR: PPG initialization failed!\r\n");
        return status;
    }
    
    /* Configure PPG for 100 SPS */
    status = MAX86171_SetSampleRate(&hppg, SAMPLE_RATE_HZ);
    if (status != HAL_OK) {
        printf("  ERROR: Failed to set PPG sample rate!\r\n");
        return status;
    }
    
    /* Configure FIFO */
    status = MAX86171_ConfigureFIFO(&hppg, 10);  // Interrupt at 10 samples
    if (status != HAL_OK) {
        printf("  ERROR: Failed to configure PPG FIFO!\r\n");
        return status;
    }
    
    /* Set LED current */
    status = MAX86171_SetLEDCurrent(&hppg, 1, 50);  // LED1, 50mA
    if (status != HAL_OK) {
        printf("  ERROR: Failed to set LED current!\r\n");
        return status;
    }
    
    printf("  PPG sensor initialized successfully\r\n");
    
    /* Initialize KX122 Accelerometer */
    printf("  Initializing accelerometer...\r\n");
    haccel.hi2c = &hi2c1;
    haccel.device_addr = KX122_I2C_ADDR;
    haccel.int_port = GPIOA;  // Example
    haccel.int_pin = GPIO_PIN_1;
    
    status = KX122_Init(&haccel);
    if (status != HAL_OK) {
        printf("  ERROR: Accelerometer initialization failed!\r\n");
        return status;
    }
    
    /* Configure accelerometer for 100 Hz ODR */
    status = KX122_SetODR(&haccel, SAMPLE_RATE_HZ);
    if (status != HAL_OK) {
        printf("  ERROR: Failed to set accelerometer ODR!\r\n");
        return status;
    }
    
    /* Set range to ±2g */
    status = KX122_SetRange(&haccel, ACCEL_RANGE_2G);
    if (status != HAL_OK) {
        printf("  ERROR: Failed to set accelerometer range!\r\n");
        return status;
    }
    
    /* Enable high resolution */
    status = KX122_EnableHighResolution(&haccel, true);
    if (status != HAL_OK) {
        printf("  ERROR: Failed to enable high resolution mode!\r\n");
        return status;
    }
    
    printf("  Accelerometer initialized successfully\r\n");
    
    return HAL_OK;
}

/**
 * @brief Start data acquisition
 */
static HAL_StatusTypeDef Start_Data_Acquisition(void)
{
    HAL_StatusTypeDef status;
    
    /* Start PPG sampling */
    status = MAX86171_StartSampling(&hppg);
    if (status != HAL_OK) {
        return status;
    }
    
    /* Start accelerometer sampling */
    status = KX122_StartSampling(&haccel);
    if (status != HAL_OK) {
        return status;
    }
    
    return HAL_OK;
}

/**
 * @brief Process sensor data
 */
static void Process_Sensor_Data(void)
{
    HAL_StatusTypeDef status;
    SensorData_t sensor_data;
    PPG_Sample_t ppg_sample;
    Accel_Sample_t accel_sample;
    bool ppg_valid = false;
    bool accel_valid = false;
    
    /* Read PPG data if available */
    uint8_t fifo_level;
    status = MAX86171_GetFIFOLevel(&hppg, &fifo_level);
    if (status == HAL_OK && fifo_level > 0) {
        uint8_t samples_read;
        status = MAX86171_ReadFIFO(&hppg, &ppg_sample, 1, &samples_read);
        if (status == HAL_OK && samples_read > 0) {
            ppg_valid = true;
        }
    }
    
    /* Read accelerometer data if available */
    bool data_ready;
    status = KX122_DataReady(&haccel, &data_ready);
    if (status == HAL_OK && data_ready) {
        status = KX122_ReadSample(&haccel, &accel_sample);
        if (status == HAL_OK) {
            accel_valid = true;
        }
    }
    
    /* If both sensors have data, create synchronized sample */
    if (ppg_valid && accel_valid) {
        sensor_data.timestamp = HAL_GetTick();
        sensor_data.ppg_value = ppg_sample.ppg_data;
        sensor_data.accel_x = accel_sample.x;
        sensor_data.accel_y = accel_sample.y;
        sensor_data.accel_z = accel_sample.z;
        
        /* Store in buffer */
        data_buffer[buffer_write_idx] = sensor_data;
        buffer_write_idx = (buffer_write_idx + 1) % DATA_BUFFER_SIZE;
        
        /* Check for overflow */
        if (buffer_write_idx == buffer_read_idx) {
            buffer_overflow = true;
        }
        
        /* Send data via UART */
        Send_UART_Packet(&sensor_data);
    }
}

/**
 * @brief Send data packet via UART
 */
static void Send_UART_Packet(SensorData_t *data)
{
    UARTPacket_t packet;
    uint8_t *packet_bytes = (uint8_t *)&packet;
    
    /* Build packet */
    packet.header = UART_PACKET_HEADER;
    packet.type = PACKET_TYPE_DATA;
    packet.length = sizeof(SensorData_t);
    packet.sequence = packet_sequence++;
    memcpy(&packet.data, data, sizeof(SensorData_t));
    
    /* Calculate CRC (exclude CRC field itself) */
    packet.crc = Calculate_CRC16(packet_bytes, sizeof(UARTPacket_t) - sizeof(uint16_t));
    
    /* Transmit packet */
    HAL_UART_Transmit(&huart1, packet_bytes, sizeof(UARTPacket_t), 100);
}

/**
 * @brief Calculate CRC16 for data integrity
 */
static uint16_t Calculate_CRC16(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    
    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}

/**
 * @brief Check system health
 */
static void Check_System_Health(void)
{
    uint16_t battery_mv;
    
    /* Check battery voltage */
    if (MAX20303_ReadBatteryVoltage(&hpmic, &battery_mv) == HAL_OK) {
        if (battery_mv < 3300) {  // Low battery threshold
            printf("WARNING: Low battery: %d mV\r\n", battery_mv);
        }
    }
    
    /* Could add more health checks here:
     * - Verify sensors are still responding
     * - Check for I2C errors
     * - Monitor data rates
     * etc.
     */
}

/**
 * @brief Timer interrupt callback - triggers at 100 Hz
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        sample_ready = true;
    }
}

/**
 * @brief GPIO EXTI interrupt callback
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    /* Handle PPG interrupt */
    if (GPIO_Pin == hppg.int_pin) {
        /* PPG FIFO ready - flag will be checked in main loop */
    }
    
    /* Handle Accelerometer interrupt */
    if (GPIO_Pin == haccel.int_pin) {
        /* Accelerometer data ready */
    }
}

/**
 * @brief System Clock Configuration
 */
void SystemClock_Config(void)
{
    /* Configure system clock to 72 MHz (adjust for your STM32) */
    /* This is typically generated by CubeMX */
    /* Example configuration - adjust based on your specific MCU */
    
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief I2C1 Initialization
 */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;  // 400 kHz Fast Mode
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief UART1 Initialization
 */
static void MX_UART1_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief TIM2 Initialization - 100 Hz timer
 */
static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 7200 - 1;  // Assuming 72 MHz clock
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 100 - 1;  // 100 Hz = 10 ms period
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }
    
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief GPIO Initialization
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure GPIO pins for interrupts */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;  // PPG and Accel INT pins
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* EXTI interrupt init */
    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

/**
 * @brief Error Handler
 */
void Error_Handler(void)
{
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
        /* Toggle LED or other error indication */
        HAL_Delay(200);
    }
}

/* Printf redirection for debugging via UART */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add implementation to report the file name and line number */
    printf("Assert failed: file %s line %lu\r\n", file, line);
}
#endif
