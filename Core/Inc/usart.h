/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define RX_TIMEOUT_MS 50 // Receive timeout in milliseconds

extern uint8_t rx_buf[128];
extern uint16_t rx_len;
extern uint32_t rx_timeout ;
extern uint8_t rx_flag;

#define byte0(dw_temp)     (*(char*)(&dw_temp))
#define byte1(dw_temp)     (*((char*)(&dw_temp) + 1))
#define byte2(dw_temp)     (*((char*)(&dw_temp) + 2))
#define byte3(dw_temp)     (*((char*)(&dw_temp) + 3))
/* USER CODE END Private defines */

void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void USART_SendArray(uint8_t *pData, uint16_t len);
void vofa_send_data(float data);
void vofa_sendframetail(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

