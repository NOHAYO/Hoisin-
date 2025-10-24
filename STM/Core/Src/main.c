/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "max86171.h"
#include "kx122.h"
#include "max20303.h"
#include "nrf52_comm.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Sensor status flags
uint8_t max86171_initialized = 0;
uint8_t kx122_initialized = 0;
uint8_t max20303_initialized = 0;
uint8_t nrf52_initialized = 0;

// Sensor data buffers
KX122_AccelData_t accel_data;
uint8_t ppg_data[32];
uint8_t max20303_status[3];

// Timing variables
uint32_t last_sensor_read_time = 0;
uint32_t sensor_read_interval = 100;  // Read sensors every 100ms

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
void InitializeSensors(void);
void ReadSensors(void);
void SendDataToBLE(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  Initialize all sensors
  * @retval None
  */
void InitializeSensors(void)
{
    HAL_StatusTypeDef status;

    // Initialize MAX20303 PMIC (I2C)
    printf("Initializing MAX20303 PMIC...\r\n");
    status = MAX20303_Init();
    if (status == HAL_OK) {
        max20303_initialized = 1;
        printf("MAX20303 initialized successfully!\r\n");
    } else {
        printf("MAX20303 initialization failed!\r\n");
    }

    HAL_Delay(100);

    // Initialize MAX86171 Optical Sensor (SPI)
    printf("Initializing MAX86171 Optical Sensor...\r\n");
    status = MAX86171_Init();
    if (status == HAL_OK) {
        max86171_initialized = 1;
        printf("MAX86171 initialized successfully! Part ID: 0x%02X\r\n", MAX86171_GetPartID());

        // Start measurement
        MAX86171_StartMeasurement();
    } else {
        printf("MAX86171 initialization failed!\r\n");
    }

    HAL_Delay(100);

    // Initialize KX122 Accelerometer (SPI)
    printf("Initializing KX122 Accelerometer...\r\n");
    status = KX122_Init();
    if (status == HAL_OK) {
        kx122_initialized = 1;
        printf("KX122 initialized successfully! WHO_AM_I: 0x%02X\r\n", KX122_GetWhoAmI());
    } else {
        printf("KX122 initialization failed!\r\n");
    }

    HAL_Delay(100);

    // Initialize NRF52 Communication (UART)
    printf("Initializing NRF52 Communication...\r\n");
    status = NRF52_Init();
    if (status == HAL_OK) {
        nrf52_initialized = 1;
        printf("NRF52 communication initialized successfully!\r\n");

        // Send ping to NRF52
        NRF52_Ping();
    } else {
        printf("NRF52 initialization failed!\r\n");
    }

    printf("\r\n=== Sensor Initialization Complete ===\r\n");
    printf("MAX20303 PMIC: %s\r\n", max20303_initialized ? "OK" : "FAILED");
    printf("MAX86171 Optical: %s\r\n", max86171_initialized ? "OK" : "FAILED");
    printf("KX122 Accel: %s\r\n", kx122_initialized ? "OK" : "FAILED");
    printf("NRF52 BLE: %s\r\n", nrf52_initialized ? "OK" : "FAILED");
    printf("=====================================\r\n\r\n");
}

/**
  * @brief  Read data from all sensors
  * @retval None
  */
void ReadSensors(void)
{
    HAL_StatusTypeDef status;

    // Read KX122 Accelerometer
    if (kx122_initialized) {
        status = KX122_ReadAccelData(&accel_data);
        if (status == HAL_OK) {
//            printf("Accel: X=%.2fg, Y=%.2fg, Z=%.2fg\r\n",
//                   accel_data.x_g, accel_data.y_g, accel_data.z_g);
        }
    }

    // Read MAX86171 FIFO (example - read 32 bytes)
    if (max86171_initialized) {
        status = MAX86171_ReadFIFO(ppg_data, 32);
        if (status == HAL_OK) {
            printf("PPG data read successfully\r\n");
        }
    }

    // Read MAX20303 Status
    if (max20303_initialized) {
        status = MAX20303_GetStatus(&max20303_status[0], &max20303_status[1], &max20303_status[2]);
        if (status == HAL_OK) {
            uint8_t charging = 0;
            MAX20303_GetChargerStatus(&charging);
            printf("PMIC Status: 0x%02X 0x%02X 0x%02X, Charging: %s\r\n",
                   max20303_status[0], max20303_status[1], max20303_status[2],
                   charging ? "Yes" : "No");
        }
    }
}

/**
  * @brief  Send sensor data to BLE module
  * @retval None
  */
void SendDataToBLE(void)
{
    if (nrf52_initialized) {
        // Create a simple data packet
        uint8_t data_packet[32];
        uint16_t index = 0;

        // Add accelerometer data
        data_packet[index++] = (uint8_t)(accel_data.x >> 8);
        data_packet[index++] = (uint8_t)(accel_data.x & 0xFF);
        data_packet[index++] = (uint8_t)(accel_data.y >> 8);
        data_packet[index++] = (uint8_t)(accel_data.y & 0xFF);
        data_packet[index++] = (uint8_t)(accel_data.z >> 8);
        data_packet[index++] = (uint8_t)(accel_data.z & 0xFF);

        // Add PPG data (first 4 bytes as example)
        data_packet[index++] = ppg_data[0];
        data_packet[index++] = ppg_data[1];
        data_packet[index++] = ppg_data[2];
        data_packet[index++] = ppg_data[3];

        // Send to NRF52
        NRF52_SendSensorData(data_packet, index);
        printf("Data sent to BLE module\r\n");
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_CAN1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // Toggle LED to indicate startup
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
    HAL_Delay(500);
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);

    printf("\r\n\r\n=== STM32 Sensor System Starting ===\r\n");

    // Initialize all sensors
    InitializeSensors();

    // Record start time
    last_sensor_read_time = HAL_GetTick();

    printf("Entering main loop...\r\n\r\n");


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // Check if it's time to read sensors
	      uint32_t current_time = HAL_GetTick();
	      if (current_time - last_sensor_read_time >= sensor_read_interval) {
	          last_sensor_read_time = current_time;

	          // Toggle LED to indicate activity
	          HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

	          // Read all sensors
	          ReadSensors();

	          // Send data to BLE module
	          SendDataToBLE();

	          printf("\r\n");
	      }

	      // Add other application code here
	      HAL_Delay(10);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch){
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
