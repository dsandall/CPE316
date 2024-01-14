
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>
#include "gpio.h"

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  /* Configure the system clock */
  /* Initialize all configured peripherals */
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  // 4 row pins (output)
  PIN_MODE(GPIOC, 0, 1);
  PIN_MODE(GPIOC, 1, 1);
  PIN_MODE(GPIOC, 2, 1);
  PIN_MODE(GPIOC, 3, 1);

  // Pins for LEDs (output)
  PIN_MODE(GPIOA, 5, 1);
  PIN_MODE(GPIOA, 6, 1);
  PIN_MODE(GPIOA, 7, 1);
  PIN_MODE(GPIOA, 8, 1);

  // 4 column pins (input)
  PIN_MODE(GPIOC, 5, 0);
  PIN_MODE(GPIOC, 6, 0);
  PIN_MODE(GPIOC, 7, 0);
  PIN_MODE(GPIOC, 8, 0);

  // set pins to pull down
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD5);
  GPIOC->PUPDR |= (GPIO_PUPDR_PUPD5_1); // pull low

  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD6);
  GPIOC->PUPDR |= (GPIO_PUPDR_PUPD6_1); // pull low

  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD7);
  GPIOC->PUPDR |= (GPIO_PUPDR_PUPD7_1); // pull low

  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD8);
  GPIOC->PUPDR |= (GPIO_PUPDR_PUPD8_1); // pull low


  while (1)
  {
	  // set all rows high and wait for press

	  PIN_W(GPIOC,0,1);
	  PIN_W(GPIOC,1,1);
	  PIN_W(GPIOC,2,1);
	  PIN_W(GPIOC,3,1);

	  bool press = false;

	  while(!press){
		  //read if any pins are high
		  if ((GPIOC->IDR & GPIO_PIN_5) || (GPIOC->IDR & GPIO_PIN_6) || (GPIOC->IDR & GPIO_PIN_7) || (GPIOC->IDR & GPIO_PIN_8)){
			  press = true;
			  break;
		  }
	  }


	  PIN_W(GPIOC, 0, 0);
	  PIN_W(GPIOC, 1, 0);
	  PIN_W(GPIOC, 2, 0);
	  PIN_W(GPIOC, 3, 0);

	  // go through each row
	  for (int i = 0; i < 4; i++){
		  PIN_W(GPIOC, i, 1);
		  // check if any column is high
		  for (int j = 0; j < 4; j++){	// column pin = j + 5
			  if (GPIOC->IDR & (1 << (j + 5))){
				  // button pressed = i, j
				  uint8_t num = ((i * 4) + j);
				  if (num & 1){
					  PIN_W(GPIOA, 5, 1);
				  } else {
					  PIN_W(GPIOA, 5, 0);
				  }
				  if (num & 2){
					  PIN_W(GPIOA, 6, 1);
				  } else {
					  PIN_W(GPIOA, 6, 0);
				  }
				  if (num & 4){
					  PIN_W(GPIOA, 7, 1);
				  } else {
					  PIN_W(GPIOA, 7, 0);
				  }
				  if (num & 8){
					  PIN_W(GPIOA, 8, 1);
				  } else {
					  PIN_W(GPIOA, 8, 0);
				  }
			  }
		  }
		  PIN_W(GPIOC, 0, 0);
		  PIN_W(GPIOC, 1, 0);
		  PIN_W(GPIOC, 2, 0);
		  PIN_W(GPIOC, 3, 0);
	  }

  }
}




///////////////////////////////////////////////////////////////////////////////////////////////////////////
// OTHER STM32 BOILERPLATE STUFF
///////////////////////////////////////////////////////////////////////////////////////////////////////////




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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
