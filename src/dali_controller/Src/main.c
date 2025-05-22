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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// #define DALI_SEND_ON 1
// #define DALI_SEND_OFF 0
#define DALI_TX_INACTIVE 2
#define DALI_TX_ACTIVE 3

#define DALI_INTERNAL_IDLE 0
#define DALI_INTERNAL_START 1
#define DALI_INTERNAL_STOP 2
#define DALI_INTERNAL_DATA 3

// Direct DALI Broadcast command that sets the Lamp power to 100%
#define DALI_COMMAND_ON 	0b1111111011111110
// Direct DALI Broadcast command that sets the Lamp power to 0%
#define DALI_COMMAND_OFF	0b1111111000000000

#define DALI_DELAY 5000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void debugf(const uint8_t*, uint16_t);
char uart_buf[256];
#define LOGF(format, ...) debugf((uint8_t *)uart_buf, sprintf(uart_buf, format, ##__VA_ARGS__))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */
volatile uint8_t daliTxState = DALI_TX_INACTIVE;
volatile uint8_t daliInternalState = DALI_INTERNAL_IDLE;
volatile uint16_t internalDaliCounter = 0;
volatile uint32_t currentDaliCommand = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
extern void digitalWrite(GPIO_TypeDef*, uint16_t, GPIO_PinState);
void daliFunction(void);
void gpioTimerTest(void);
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
  setvbuf(stdin, NULL, _IONBF, 0);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  
  /* USER CODE END 2 */

  /* Initialize led */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  BSP_COM_SelectLogPort(COM1);
  while (1)
  {
    while(daliTxState == DALI_TX_ACTIVE) HAL_Delay(10);

		currentDaliCommand = DALI_COMMAND_ON;
		daliTxState = DALI_TX_ACTIVE;
		BSP_LED_On(LED2);
		HAL_Delay(DALI_DELAY);

		while(daliTxState == DALI_TX_ACTIVE) HAL_Delay(10);

		currentDaliCommand = DALI_COMMAND_OFF;
		daliTxState = DALI_TX_ACTIVE;
		BSP_LED_Off(LED2);
		HAL_Delay(DALI_DELAY);
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 50-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1000-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */
  HAL_TIM_Base_Start_IT(&htim7);
  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DALI_Output_GPIO_Port, DALI_Output_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DALI_Input_Pin */
  GPIO_InitStruct.Pin = DALI_Input_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DALI_Input_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DALI_Output_Pin */
  GPIO_InitStruct.Pin = DALI_Output_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DALI_Output_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void debugf(const uint8_t *pData, uint16_t Size){
	HAL_UART_Transmit(&hcom_uart[0], pData, Size, 100);
}

void digitalWrite(GPIO_TypeDef *Port, uint16_t Pin, GPIO_PinState State){
	HAL_GPIO_WritePin(Port, Pin, State);
}

GPIO_PinState daliOutputPinState(uint16_t currentSubBit, uint16_t bitForSend){
	if(bitForSend == 0){
    if(currentSubBit == 1){
      return GPIO_PIN_SET;
    }else{
      return GPIO_PIN_RESET;
    }
	}else{
    if(currentSubBit == 0){
      	return GPIO_PIN_SET;
	  }else{
      	return GPIO_PIN_RESET;
    }
  }
}

void daliFunction(){
	if(daliTxState == DALI_TX_INACTIVE) return;

	if(daliInternalState == DALI_INTERNAL_IDLE){
		daliInternalState = DALI_INTERNAL_START;
		internalDaliCounter = 0;
	}

	uint16_t currentBit = internalDaliCounter >> 1;
	uint16_t currentSubBit = internalDaliCounter % 2;

	uint32_t bitForSend;

	if(daliInternalState == DALI_INTERNAL_DATA){

		bitForSend = (currentDaliCommand & (0x8000 >> (currentBit)));
		// LOGF("ON(%d): 0b%d\r\n", currentBit, (bitForSend)?(1):(0));

		digitalWrite(DALI_Output_GPIO_Port, DALI_Output_Pin, daliOutputPinState(currentSubBit, bitForSend));
		internalDaliCounter++;

		if(internalDaliCounter > 16*2){
			daliInternalState = DALI_INTERNAL_STOP;
			internalDaliCounter = 0;
		}
		return;

		}else if(daliInternalState == DALI_INTERNAL_START){
		// Start bit vrijednosti 1
//		LOGF("Start\r\n");

		digitalWrite(DALI_Output_GPIO_Port, DALI_Output_Pin, daliOutputPinState(currentSubBit, 1U));
		internalDaliCounter++;

		if(internalDaliCounter >= 2){
			daliInternalState = DALI_INTERNAL_DATA;
			internalDaliCounter = 0;
		}
		return;

	}else if(daliInternalState == DALI_INTERNAL_STOP){
//		LOGF("End\r\n");

		digitalWrite(DALI_Output_GPIO_Port, DALI_Output_Pin, GPIO_PIN_RESET);
		internalDaliCounter++;

		if(internalDaliCounter >= 4){
			daliInternalState = DALI_INTERNAL_IDLE;
			internalDaliCounter = 0;
			daliTxState = DALI_TX_INACTIVE;
		}
		return;
	}
}

void gpioTimerTest(){
	volatile uint16_t currentBit = internalDaliCounter >> 1;
	volatile uint16_t currentSubBit = internalDaliCounter % 2;
	// static volatile uint8_t ledStatus = 0;

	digitalWrite(DALI_Output_GPIO_Port, DALI_Output_Pin, daliOutputPinState(currentSubBit, 1U));
	internalDaliCounter++;

	if(internalDaliCounter%1000 == 0){
		BSP_LED_Toggle(LED2);
	}
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
