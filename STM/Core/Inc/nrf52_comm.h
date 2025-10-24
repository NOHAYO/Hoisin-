/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    nrf52_comm.h
  * @brief   NRF52840 UART Communication Driver Header
  ******************************************************************************
  * @attention
  *
  * Communication with NRF52840 BLE module via UART (USART1)
  * UART1: PA9 (TX), PA10 (RX) - 115200 baud
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __NRF52_COMM_H
#define __NRF52_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "usart.h"
#include <string.h>

/* Configuration */
#define NRF52_UART_HANDLE           huart1
#define NRF52_UART_TIMEOUT          1000
#define NRF52_RX_BUFFER_SIZE        256
#define NRF52_TX_BUFFER_SIZE        256

/* Message frame structure (customize as needed) */
typedef struct {
    uint8_t start_byte;     // Frame start marker (e.g., 0xAA)
    uint8_t command;        // Command byte
    uint8_t length;         // Data length
    uint8_t data[252];      // Data payload
    uint8_t checksum;       // Simple checksum
} NRF52_Message_t;

/* Command definitions (example - customize for your protocol) */
typedef enum {
    NRF52_CMD_PING          = 0x01,
    NRF52_CMD_SEND_DATA     = 0x02,
    NRF52_CMD_GET_STATUS    = 0x03,
    NRF52_CMD_SET_CONFIG    = 0x04,
    NRF52_CMD_BLE_ADVERTISE = 0x05,
    NRF52_CMD_BLE_CONNECT   = 0x06,
    NRF52_CMD_BLE_DISCONNECT= 0x07,
    NRF52_CMD_SENSOR_DATA   = 0x10,
    NRF52_CMD_ACK           = 0xAA,
    NRF52_CMD_NACK          = 0xBB
} NRF52_Command_t;

/* Frame start marker */
#define NRF52_FRAME_START       0xAA

/* Function Prototypes */
HAL_StatusTypeDef NRF52_Init(void);
HAL_StatusTypeDef NRF52_SendRaw(uint8_t *data, uint16_t length);
HAL_StatusTypeDef NRF52_ReceiveRaw(uint8_t *data, uint16_t length, uint32_t timeout);
HAL_StatusTypeDef NRF52_SendMessage(NRF52_Command_t command, uint8_t *data, uint8_t length);
HAL_StatusTypeDef NRF52_ReceiveMessage(NRF52_Message_t *message, uint32_t timeout);
HAL_StatusTypeDef NRF52_SendSensorData(uint8_t *sensor_data, uint8_t length);
HAL_StatusTypeDef NRF52_Ping(void);
uint8_t NRF52_CalculateChecksum(uint8_t *data, uint16_t length);
void NRF52_EnableRxInterrupt(void);
void NRF52_DisableRxInterrupt(void);

/* Callback function (user should implement) */
void NRF52_RxCallback(uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* __NRF52_COMM_H */
