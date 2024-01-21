
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
  ******************************************************************************/


#include "main.h"
#include <stdbool.h>
#include "gpio.h"
#include "keypad.h"
#include "lcd.h"

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

void dispLocked (){
	LCD_command(0x01,0); //clear display
	LCD_write_line("LOCKED");
	LCD_write_string("Enter PIN: ");
	disp_LED(0b0001);
}

void dispNewPIN (){
	LCD_command(0x01,0); //clear display
	LCD_write_line("Hello, Admin");
	LCD_write_string("Enter PIN: ");
	disp_LED(0b1110);
}

void dispUnlocked(){
	LCD_command(0x01,0); //clear display
	LCD_write_line("UNLOCKED");
	LCD_write_string("*:lock   #:reset");
	disp_LED(0b0011);
}

void dispBoom(){
	LCD_command(0x01,0); //clear display and attempts
	LCD_write_string("BOOM");
	disp_LED(0b1111);
}

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



  setup_keypad();
  setup_LCD();

  enum KeypadStates {
      LOCKED,
      UNLOCKED,
      SELF_DESTRUCT,
      NEW_PIN
  };

  enum KeypadStates currentState = LOCKED;


  char PIN[] = {'1','2','3','4'};
  char attempt[] = {'f','f','f','f'};

  uint8_t curr_attempt = 0;




  while (1)
  {





	    // Simulate keypad behavior based on the current state

	  switch (currentState) {

	  	  case LOCKED:
	        	//initialize Locked State
	        	dispLocked();

	        	while(1){
				  if (keypad_pressed()){
					  char charpressed = id2char(scan_keypad());

					  if (charpressed == '*'){
						  dispLocked();
						  for (int i = 0; i < 4; i++) {attempt[i] = 'f';}
						  curr_attempt = 0;
					  }

					  else{
						  //record press and send to display
						  LCD_command(charpressed, 1);
						  attempt[curr_attempt++] = charpressed;

						  //if they are done, goto next state
						  if (curr_attempt >=4){
							  //compare PINs and goto next state
							  if (attempt[0] == PIN[0] && attempt[1] == PIN[1] && attempt[2] == PIN[2] && attempt[3] == PIN[3]){
								  currentState = UNLOCKED;}
							  else{currentState = SELF_DESTRUCT;}

							  //clear attempts
							  for (int i = 0; i < 4; i++) {attempt[i] = 'f';}
							  curr_attempt = 0;

							  break;
						  }

					  }
				  }
				  HAL_Delay(300); //wait a sec before registering another press
	        	}
	        break;


	        //SELF_DESTRUCT
	        case SELF_DESTRUCT:

	        	dispBoom();
	        	HAL_Delay(3000);

	        	currentState = LOCKED;
			break;


			//UNLOCKED
	        case UNLOCKED:

	        	dispUnlocked();

	        	while (1){
	        		if(keypad_pressed()){
	        			char charpressed = id2char(scan_keypad());

	        			if (charpressed == '*'){
	        				//lock display
	        	        	currentState = LOCKED;
	        	        	break;
	        			}
	        			else if (charpressed == '#'){
	        				//change PIN
	        				currentState = NEW_PIN;
	        				break;
	        			}
	        		}
	        		HAL_Delay(300); //wait a sec before registering another press
	        	}


			break;


			//NEW_PIN
	        case NEW_PIN:

	        	dispNewPIN();

	        	while(1){
	        		if(keypad_pressed()){
						  //record press and send to display
						  char charpressed = id2char(scan_keypad());
						  LCD_command(charpressed, 1);
						  PIN[curr_attempt++] = charpressed;

						  //if they are done, exit state
						  if (curr_attempt >=4){
							  curr_attempt = 0;
							  currentState = UNLOCKED;
							  break;
							  //also turn off LED
						  }
	        		}
	        		HAL_Delay(300); //wait a sec before registering another press
	        	}
			break;

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
