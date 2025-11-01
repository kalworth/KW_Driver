/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */
#include "can_rcc.h"
#include "usr_config.h"
#include "util.h"
#include "foc_encoder.h"
#include  "pll.h"
#include "foc_handle.h"
#include "fsm.h"
#include "controller.h"
/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;

uint32_t can_rx_id = 0;
uint32_t can_rx_len = 0;
uint8_t rx_data[8] = {0};
uint8_t can_rx_flag = 0;

static uint8_t can_motor_enable = 0;
/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 10;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 8;
  hfdcan1.Init.NominalTimeSeg2 = 8;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 8;
  hfdcan1.Init.ExtFiltersNbr = 8;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
	FDCAN_FilterTypeDef FDCAN1_RXFilter;
	FDCAN1_RXFilter.IdType=FDCAN_STANDARD_ID;                       //标准ID
	FDCAN1_RXFilter.FilterIndex=0;                                  //滤波器索引                   
	FDCAN1_RXFilter.FilterType=FDCAN_FILTER_RANGE;                   //滤波器类型
	FDCAN1_RXFilter.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;           //过滤器0关联到FIFO0  
	FDCAN1_RXFilter.FilterID1=0x0000;                               //32位ID
	FDCAN1_RXFilter.FilterID2=0x0000;                               //如果FDCAN配置为传统模式的话，这里是32位掩码
	if(HAL_FDCAN_ConfigFilter(&hfdcan1,&FDCAN1_RXFilter)!=HAL_OK) //滤波器初始化
	{
		Error_Handler();
	}

	HAL_FDCAN_Start(&hfdcan1);
	
	HAL_FDCAN_ConfigInterruptLines(&hfdcan1, FDCAN_IT_GROUP_RX_FIFO0, FDCAN_INTERRUPT_LINE0);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  /* USER CODE END FDCAN1_Init 2 */

}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN1 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
}

void print_can_data(void) {
    // 打印 CAN ID
    DEBUG("CAN ID: 0x%08X\n", can_rx_id);

    // 打印数据长度
    DEBUG("Data Length: 0x%02X (%d bytes)\n", can_rx_len, can_rx_len);

    // 打印数据帧内容
    DEBUG("Data: ");
    for (uint32_t i = 0; i < can_rx_len; i++) {
        DEBUG("0x%02X ", rx_data[i]);
    }
    DEBUG("\n");
}

/* USER CODE BEGIN 1 */
void can_send(uint32_t frameID, uint8_t* pData)
{
	FDCAN_TxHeaderTypeDef header;
	header.BitRateSwitch = FDCAN_BRS_OFF;
	header.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
	header.MessageMarker = 0;
	header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	header.IdType = FDCAN_STANDARD_ID;
	header.FDFormat = FDCAN_CLASSIC_CAN;
	header.TxFrameType = FDCAN_DATA_FRAME;
	
	header.Identifier = frameID;
	header.DataLength = FDCAN_DLC_BYTES_8;
	
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, pData);
}

extern uint8_t err_flag;
static void can_send_motor_info(void)
{
  uint8_t can_tx_data[8];
  uint16_t pos_tmp = float_to_uint(Encoder.position_output,  -UsrConfig.pos_max,  UsrConfig.pos_max,  16);
  uint16_t vel_tmp = float_to_uint(Encoder.velocity_output,  -UsrConfig.vel_max,  UsrConfig.vel_max,  16);
  uint16_t tor_tmp = float_to_uint(Foc.i_q_filt,  -UsrConfig.iq_max,  UsrConfig.iq_max,  16);
//  uint16_t pos_tmp = float_to_uint(Encoder.position_output,  -12.5,  12.5,  16);
//  uint16_t vel_tmp = float_to_uint(Pll.vel_estimate,  -45,  45,  16);
//  uint16_t tor_tmp = float_to_uint(Foc.i_q_filt,  -2,  2,  16);

  can_tx_data[0] = UsrConfig.can_id;
  can_tx_data[1] = err_flag;
  can_tx_data[2] = (pos_tmp >> 8);
  can_tx_data[3] = pos_tmp;
  can_tx_data[4] = (vel_tmp >> 8);
  can_tx_data[5] = vel_tmp;
  can_tx_data[6] = (tor_tmp >> 8);
  can_tx_data[7] = tor_tmp;

  can_send(0x10|UsrConfig.can_id,can_tx_data);
}

static void can_rx_mit_command(uint8_t data[])
{
  uint16_t pos_tmp = ((uint16_t)data[0])<<8 | data[1];
  uint16_t vel_tmp = ((uint16_t)data[2])<<4 | (data[3]>>4);
  uint16_t kp_tmp = ((uint16_t)(data[3]& 0x0F))<<8 | data[4];
  uint16_t kd_tmp = ((uint16_t)data[5])<<4 | (data[6]>>4);
  uint16_t tf_tmp = ((uint16_t)(data[6]& 0x0F))<<8 | data[7];

  Controller.input_position = uint_to_float(pos_tmp,-UsrConfig.pos_max,UsrConfig.pos_max,16);
  Controller.input_velocity = uint_to_float(vel_tmp,-UsrConfig.vel_max,UsrConfig.vel_max,12);
  Controller.mit_kp = uint_to_float(kp_tmp,0,10.0f,12);
  Controller.mit_kd = uint_to_float(kd_tmp,0,1.0f,12);
  Controller.input_current = uint_to_float(tf_tmp,-UsrConfig.iq_max,UsrConfig.iq_max,12);
}

static void can_motor_task(uint32_t rx_id)
{
  // 正常ID 电机使能
  if(rx_id == UsrConfig.can_id){

    //使能指令查询
    uint8_t enable_command = 0;
    for(uint8_t i = 0;i < 7; i ++){
      if(rx_data[i] == 0xFF){enable_command ++;}
    }
    if(enable_command==7){
      if(rx_data[7] == 0xFD){can_motor_enable = 0;FSM_input(0x1B);}
      else if(rx_data[7] == 0xFC){can_motor_enable = 1; FSM_input(CMD_MOTOR);}
    }
		
  }
  // MIT模式，ID高8位为7
  else{
    // 只保留最后一位
    uint32_t mit_id = rx_id & 0x0000000F;
    if(mit_id == UsrConfig.can_id && can_motor_enable && (rx_id & 0x000000F0) == 0x70){
        can_rx_mit_command(rx_data);
        can_send_motor_info();
    }  
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
	
	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0) {
    FDCAN_RxHeaderTypeDef rxHeader;
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader, rx_data) == HAL_OK)
    {
      can_rx_id = rxHeader.Identifier;
      can_rx_len = rxHeader.DataLength>>16; 
      can_motor_task(can_rx_id);
    }
	}	

}
/* USER CODE END 1 */
