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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "systick.h"
#include "drv8323.h"
#include "usr_config.h"
#include "foc_encoder.h"
#include "foc_handle.h"
#include "led.h"
#include "fsm.h"
#include <string.h>
#include "pll.h"
#include "can.h"
#include "controller.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /** Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
  */
  LL_PWR_DisableUCPDDeadBattery();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	SYSTICK_init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

	// ADC1 calibration
	LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED );
	while(LL_ADC_IsCalibrationOnGoing(ADC1));

	// ADC2 calibration
	LL_ADC_StartCalibration(ADC2, LL_ADC_SINGLE_ENDED );
	while(LL_ADC_IsCalibrationOnGoing(ADC2));
		
	LL_ADC_Enable(ADC1);
	LL_ADC_Enable(ADC2);
		
	LL_ADC_INJ_StartConversion(ADC1);
	LL_ADC_INJ_StartConversion(ADC2);
	
	LL_TIM_EnableAllOutputs(TIM1);
	
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
		
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH4);
		
	LL_TIM_EnableCounter(TIM1);
	
	SYSTICK_delay_ms(200);
	
	LL_ADC_EnableIT_JEOS(ADC1);
	LL_ADC_EnableIT_JEOS(ADC2);
	
	LL_TIM_EnableIT_UPDATE(TIM6);
	LL_TIM_EnableCounter(TIM6);
	
  if (0 != DRV8323_reset())
  {
    DEBUG("\n\rDRV8323 init fail!\n\r");
    while (1);
  }
	else{
		DEBUG("\n\rDRV8323 init ok!\n\r");
	}
	
  if (0 == USR_CONFIG_read_config())
  {
    DEBUG("\n\rConfig loaded ok\n\r");
  }
  else
  {
    USR_CONFIG_set_default_config();
    DEBUG("\n\rConfig loaded faile set to default\n\r");
  }
	
	pll_speed_estimator_rad_init(&Pll, UsrConfig.current_ctrl_bandwidth/20);
	
  UsrConfig.control_mode = CONTROL_MODE_CURRENT;
	UsrConfig.can_timeout_ms=1000;
	SYSTICK_delay_ms(200);
	FOC_zero_current(&Foc);
	FSM_input(CMD_MENU);
  
	LL_USART_EnableIT_RXNE_RXFNE(USART2);

	MX_FDCAN1_Init();
	//uint8_t data[8] = {0x10,0x20,0x30,0x40,0x50,0x60,0x70,0x80};
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		LED_loop();

		if(rx_flag){

			for(uint8_t i = 0; i <rx_len; i ++)
			{
				FSM_input(rx_buf[i]);
			}
			rx_flag = 0;
			rx_len = 0;
			memset(rx_buf, 0, sizeof(rx_buf));
		}
//		DEBUG("%f,%f\r\n",Encoder.velocity_output,Pll.vel_estimate);
//		DEBUG("%f,%f,%f\r\n",Foc.i_q_tar,Foc.i_d_filt,Foc.i_q_filt);
//		DEBUG("%f\r\n",Encoder.position_output);
//		DEBUG("%d,%d\r\n",Foc.adc_phase_a,Foc.adc_phase_b);
//      DEBUG("%f,%f,%f\r\n",Encoder.position_output,Pll.vel_estimate,Foc.i_q_filt);
////		DEBUG("%f\r\n",loop_freq);		
//      DEBUG("%f,%f,%f,%f,%f\r\n",Controller.input_position,Controller.input_velocity,Controller.mit_kp,Controller.mit_kd,Controller.input_current);
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4)
  {
  }
  LL_PWR_EnableRange1BoostMode();
  LL_RCC_HSI_Enable();
   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  LL_RCC_HSI_SetCalibTrimming(64);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_4, 85, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();
   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Insure 1us transition state at intermediate medium speed clock*/
  for (__IO uint32_t i = (170 >> 1); i !=0; i--);

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  LL_Init1msTick(170000000);

  LL_SetSystemCoreClock(170000000);
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
