/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    nrf52_comm.c
  * @brief   NRF52840 UART Communication Driver Implementation
  ******************************************************************************
  */
/* USER CODE END Header */

#include "nrf52_comm.h"

/* Private variables */
static uint8_t rx_buffer[NRF52_RX_BUFFER_SIZE];
static uint8_t tx_buffer[NRF52_TX_BUFFER_SIZE];
static volatile uint16_t rx_index = 0;

/**
  * @brief  Calculate simple checksum
  * @param  data: Pointer to data
  * @param  length: Data length
  * @retval Checksum value
  */
uint8_t NRF52_CalculateChecksum(uint8_t *data, uint16_t length)
{
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < length; i++) {
        checksum += data[i];
    }
    return checksum;
}

/**
  * @brief  Send raw data via UART
  * @param  data: Pointer to data
  * @param  length: Data length
  * @retval HAL status
  */
HAL_StatusTypeDef NRF52_SendRaw(uint8_t *data, uint16_t length)
{
    return HAL_UART_Transmit(&NRF52_UART_HANDLE, data, length, NRF52_UART_TIMEOUT);
}

/**
  * @brief  Receive raw data via UART
  * @param  data: Pointer to buffer
  * @param  length: Expected data length
  * @param  timeout: Timeout in milliseconds
  * @retval HAL status
  */
HAL_StatusTypeDef NRF52_ReceiveRaw(uint8_t *data, uint16_t length, uint32_t timeout)
{
    return HAL_UART_Receive(&NRF52_UART_HANDLE, data, length, timeout);
}

/**
  * @brief  Send a structured message to NRF52
  * @param  command: Command byte
  * @param  data: Pointer to data payload
  * @param  length: Data length
  * @retval HAL status
  */
HAL_StatusTypeDef NRF52_SendMessage(NRF52_Command_t command, uint8_t *data, uint8_t length)
{
    uint16_t msg_length = 0;
    
    // Build message frame
    tx_buffer[msg_length++] = NRF52_FRAME_START;  // Start byte
    tx_buffer[msg_length++] = command;            // Command
    tx_buffer[msg_length++] = length;             // Data length
    
    // Copy data
    if (length > 0 && data != NULL) {
        memcpy(&tx_buffer[msg_length], data, length);
        msg_length += length;
    }
    
    // Calculate and append checksum
    uint8_t checksum = NRF52_CalculateChecksum(&tx_buffer[1], msg_length - 1);
    tx_buffer[msg_length++] = checksum;
    
    // Send message
    return HAL_UART_Transmit(&NRF52_UART_HANDLE, tx_buffer, msg_length, NRF52_UART_TIMEOUT);
}

/**
  * @brief  Receive a structured message from NRF52
  * @param  message: Pointer to message structure
  * @param  timeout: Timeout in milliseconds
  * @retval HAL status
  */
HAL_StatusTypeDef NRF52_ReceiveMessage(NRF52_Message_t *message, uint32_t timeout)
{
    HAL_StatusTypeDef status;
    uint8_t header[3];
    uint8_t checksum_rx;
    
    // Wait for start byte
    status = HAL_UART_Receive(&NRF52_UART_HANDLE, header, 3, timeout);
    if (status != HAL_OK) return status;
    
    // Verify start byte
    if (header[0] != NRF52_FRAME_START) {
        return HAL_ERROR;
    }
    
    message->start_byte = header[0];
    message->command = header[1];
    message->length = header[2];
    
    // Receive data payload if length > 0
    if (message->length > 0) {
        if (message->length > 252) {
            return HAL_ERROR;  // Invalid length
        }
        status = HAL_UART_Receive(&NRF52_UART_HANDLE, message->data, 
                                   message->length, timeout);
        if (status != HAL_OK) return status;
    }
    
    // Receive checksum
    status = HAL_UART_Receive(&NRF52_UART_HANDLE, &checksum_rx, 1, timeout);
    if (status != HAL_OK) return status;
    
    // Verify checksum
    uint8_t checksum_calc = message->command + message->length;
    for (uint8_t i = 0; i < message->length; i++) {
        checksum_calc += message->data[i];
    }
    
    if (checksum_calc != checksum_rx) {
        return HAL_ERROR;  // Checksum mismatch
    }
    
    message->checksum = checksum_rx;
    
    return HAL_OK;
}

/**
  * @brief  Send sensor data to NRF52
  * @param  sensor_data: Pointer to sensor data
  * @param  length: Data length
  * @retval HAL status
  */
HAL_StatusTypeDef NRF52_SendSensorData(uint8_t *sensor_data, uint8_t length)
{
    return NRF52_SendMessage(NRF52_CMD_SENSOR_DATA, sensor_data, length);
}

/**
  * @brief  Send ping command to NRF52
  * @retval HAL status
  */
HAL_StatusTypeDef NRF52_Ping(void)
{
    return NRF52_SendMessage(NRF52_CMD_PING, NULL, 0);
}

/**
  * @brief  Enable UART receive interrupt
  */
void NRF52_EnableRxInterrupt(void)
{
    HAL_UART_Receive_IT(&NRF52_UART_HANDLE, rx_buffer, 1);
}

/**
  * @brief  Disable UART receive interrupt
  */
void NRF52_DisableRxInterrupt(void)
{
    HAL_UART_AbortReceive_IT(&NRF52_UART_HANDLE);
}

/**
  * @brief  UART receive complete callback (should be called from HAL_UART_RxCpltCallback)
  * @param  huart: UART handle
  */
void NRF52_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == NRF52_UART_HANDLE.Instance) {
        // Store received byte
        if (rx_index < NRF52_RX_BUFFER_SIZE) {
            rx_index++;
        }
        
        // Re-enable interrupt for next byte
        HAL_UART_Receive_IT(&NRF52_UART_HANDLE, &rx_buffer[rx_index], 1);
    }
}

/**
  * @brief  Initialize NRF52 communication
  * @retval HAL status
  */
HAL_StatusTypeDef NRF52_Init(void)
{
    // UART is already initialized by MX_USART1_UART_Init()
    // Clear buffers
    memset(rx_buffer, 0, NRF52_RX_BUFFER_SIZE);
    memset(tx_buffer, 0, NRF52_TX_BUFFER_SIZE);
    rx_index = 0;
    
    // Small delay for NRF52 to be ready
    HAL_Delay(100);
    
    // Optional: Send ping to verify communication
    // HAL_StatusTypeDef status = NRF52_Ping();
    
    return HAL_OK;
}

/* Weak callback function - user can override in main.c */
__weak void NRF52_RxCallback(uint8_t *data, uint16_t length)
{
    /* User should implement this function in main.c */
    UNUSED(data);
    UNUSED(length);
}
