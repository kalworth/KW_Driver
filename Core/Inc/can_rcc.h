#ifndef __CAN_RCC_H__
#define __CAN_RCC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_hal_fdcan.h"
/** @defgroup GPIO_Exported_Types GPIO Exported Types
  * @{
  */
/**
  * @brief   GPIO Init structure definition
  */
typedef struct
{
  uint32_t Pin;        /*!< Specifies the GPIO pins to be configured.
                           This parameter can be any value of @ref GPIO_pins */

  uint32_t Mode;       /*!< Specifies the operating mode for the selected pins.
                           This parameter can be a value of @ref GPIO_mode */

  uint32_t Pull;       /*!< Specifies the Pull-up or Pull-Down activation for the selected pins.
                           This parameter can be a value of @ref GPIO_pull */

  uint32_t Speed;      /*!< Specifies the speed for the selected pins.
                           This parameter can be a value of @ref GPIO_speed */

  uint32_t Alternate;  /*!< Peripheral to be connected to the selected pins
                            This parameter can be a value of @ref GPIOEx_Alternate_function_selection */
} GPIO_InitTypeDef;

/**
  * @brief  RCC extended clocks structure definition
  */
typedef struct
{
  uint32_t PeriphClockSelection;   /*!< The Extended Clock to be configured.
                                        This parameter can be a value of @ref RCCEx_Periph_Clock_Selection */

  uint32_t Usart1ClockSelection;   /*!< Specifies USART1 clock source.
                                        This parameter can be a value of @ref RCCEx_USART1_Clock_Source */

  uint32_t Usart2ClockSelection;   /*!< Specifies USART2 clock source.
                                        This parameter can be a value of @ref RCCEx_USART2_Clock_Source */
#if defined(USART3)
  uint32_t Usart3ClockSelection;   /*!< Specifies USART3 clock source.
                                        This parameter can be a value of @ref RCCEx_USART3_Clock_Source */
#endif /* UART3 */

#if defined(UART4)
  uint32_t Uart4ClockSelection;    /*!< Specifies UART4 clock source.
                                        This parameter can be a value of @ref RCCEx_UART4_Clock_Source */
#endif /* UART4 */

#if defined(UART5)
  uint32_t Uart5ClockSelection;    /*!< Specifies UART5 clock source.
                                        This parameter can be a value of @ref RCCEx_UART5_Clock_Source */

#endif /* UART5 */

  uint32_t Lpuart1ClockSelection;  /*!< Specifies LPUART1 clock source.
                                        This parameter can be a value of @ref RCCEx_LPUART1_Clock_Source */

  uint32_t I2c1ClockSelection;     /*!< Specifies I2C1 clock source.
                                        This parameter can be a value of @ref RCCEx_I2C1_Clock_Source */

  uint32_t I2c2ClockSelection;     /*!< Specifies I2C2 clock source.
                                        This parameter can be a value of @ref RCCEx_I2C2_Clock_Source */

  uint32_t I2c3ClockSelection;     /*!< Specifies I2C3 clock source.
                                        This parameter can be a value of @ref RCCEx_I2C3_Clock_Source */

#if defined(I2C4)

  uint32_t I2c4ClockSelection;     /*!< Specifies I2C4 clock source.
                                        This parameter can be a value of @ref RCCEx_I2C4_Clock_Source */
#endif /* I2C4 */

  uint32_t Lptim1ClockSelection;   /*!< Specifies LPTIM1 clock source.
                                        This parameter can be a value of @ref RCCEx_LPTIM1_Clock_Source */

  uint32_t Sai1ClockSelection;     /*!< Specifies SAI1 clock source.
                                        This parameter can be a value of @ref RCCEx_SAI1_Clock_Source */

  uint32_t I2sClockSelection;     /*!< Specifies I2S clock source.
                                        This parameter can be a value of @ref RCCEx_I2S_Clock_Source */
#if defined(FDCAN1)

  uint32_t FdcanClockSelection;     /*!< Specifies FDCAN clock source.
                                        This parameter can be a value of @ref RCCEx_FDCAN_Clock_Source */
#endif /* FDCAN1 */
#if defined(USB)

  uint32_t UsbClockSelection;      /*!< Specifies USB clock source (warning: same source for RNG).
                                        This parameter can be a value of @ref RCCEx_USB_Clock_Source */
#endif /* USB */

  uint32_t RngClockSelection;      /*!< Specifies RNG clock source (warning: same source for USB).
                                        This parameter can be a value of @ref RCCEx_RNG_Clock_Source */

  uint32_t Adc12ClockSelection;    /*!< Specifies ADC12 interface clock source.
                                        This parameter can be a value of @ref RCCEx_ADC12_Clock_Source */

#if defined(ADC345_COMMON)
  uint32_t Adc345ClockSelection;   /*!< Specifies ADC345 interface clock source.
                                        This parameter can be a value of @ref RCCEx_ADC345_Clock_Source */
#endif /* ADC345_COMMON */

#if defined(QUADSPI)
  uint32_t QspiClockSelection;     /*!< Specifies QuadSPI clock source.
                                        This parameter can be a value of @ref RCCEx_QSPI_Clock_Source */
#endif

  uint32_t RTCClockSelection;      /*!< Specifies RTC clock source.
                                        This parameter can be a value of @ref RCC_RTC_Clock_Source */
}RCC_PeriphCLKInitTypeDef;

//typedef enum
//{
//  HAL_OK       = 0x00U,
//  HAL_ERROR    = 0x01U,
//  HAL_BUSY     = 0x02U,
//  HAL_TIMEOUT  = 0x03U
//} HAL_StatusTypeDef;

#define RCC_PERIPHCLK_FDCAN            0x00001000U
#define RCC_FDCANCLKSOURCE_PCLK1        RCC_CCIPR_FDCANSEL_1
#define RCC_PERIPHCLK_RTC              0x00080000U

#define __HAL_RCC_FDCAN_CLK_ENABLE()          do { \
                                                 __IO uint32_t tmpreg; \
                                                 SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_FDCANEN); \
                                                 /* Delay after an RCC peripheral clock enabling */ \
                                                 tmpreg = READ_BIT(RCC->APB1ENR1, RCC_APB1ENR1_FDCANEN); \
                                                 UNUSED(tmpreg); \
                                               } while(0)
#define __HAL_RCC_GPIOA_CLK_ENABLE()           do { \
                                                 __IO uint32_t tmpreg; \
                                                 SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOAEN); \
                                                 /* Delay after an RCC peripheral clock enabling */ \
                                                 tmpreg = READ_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOAEN); \
                                                 UNUSED(tmpreg); \
                                               } while(0)

#define GPIO_GET_INDEX(__GPIOx__)    (((__GPIOx__) == (GPIOA))? 0UL :\
                                      ((__GPIOx__) == (GPIOB))? 1UL :\
                                      ((__GPIOx__) == (GPIOC))? 2UL :\
                                      ((__GPIOx__) == (GPIOD))? 3UL :\
                                      ((__GPIOx__) == (GPIOE))? 4UL :\
                                      ((__GPIOx__) == (GPIOF))? 5UL : 6UL)
																							 
#define GPIO_PIN_11                ((uint16_t)0x0800)  /* Pin 11 selected   */
#define GPIO_PIN_12                ((uint16_t)0x1000)  /* Pin 12 selected   */

#define GPIO_MODE_Pos                           0U																							 
#define OUTPUT_TYPE_Pos                         4U																							 
#define MODE_AF                                 (0x2UL << GPIO_MODE_Pos)
#define OUTPUT_PP                               (0x0UL << OUTPUT_TYPE_Pos)																							 
#define  GPIO_MODE_AF_PP                        (MODE_AF | OUTPUT_PP)                                       /*!< Alternate Function Push Pull Mode     */
																							 
#define GPIO_MODE                               (0x3UL << GPIO_MODE_Pos)
#define MODE_INPUT                              (0x0UL << GPIO_MODE_Pos)
#define MODE_OUTPUT                             (0x1UL << GPIO_MODE_Pos)
#define MODE_ANALOG                             (0x3UL << GPIO_MODE_Pos)
#define OUTPUT_TYPE                             (0x1UL << OUTPUT_TYPE_Pos)
#define OUTPUT_OD                               (0x1UL << OUTPUT_TYPE_Pos)
																							 
#define  GPIO_NOPULL        (0x00000000U)   /*!< No Pull-up or Pull-down activation  */
#define  GPIO_SPEED_FREQ_LOW        (0x00000000U)   /*!< range up to 5 MHz, please refer to the product datasheet */

#define GPIO_AF9_FDCAN1        ((uint8_t)0x09)  /* FDCAN1 Alternate Function mapping  */

#define __HAL_RCC_FDCAN_CLK_DISABLE()          CLEAR_BIT(RCC->APB1ENR1, RCC_APB1ENR1_FDCANEN)
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);
void HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init);		
void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority);
void HAL_NVIC_EnableIRQ(IRQn_Type IRQn);
void HAL_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin);
void HAL_NVIC_DisableIRQ(IRQn_Type IRQn);

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

