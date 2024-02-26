
#include "main.h"
#include "dac.h"
#include "gpio.h"
#include "keypad.h"
#include "timer.h"
#include <math.h>

#define SPS 10000
#define SIN_ACC 100

SPI_HandleTypeDef hspi2;
UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);

char val = 0;
char wave = '7';
char com = 0;
uint16_t voltage = 0;
int delay = 650;
int freq = 100;
int duty_cycle = 50; // percent
int samp_per_cycle = SPS / 100;
float sin_increment = 360 / (SPS / 100);
float ramp_increment = 3000 / (SPS / 100);
float triangle_increment = 3000 / (SPS / 100) * 2;

int samp_val = 0;
volatile int inc_samp_val = 0;
int angle = 0;

double sin_values[SIN_ACC];
void build_sin_table(){
	// Precompute sin values for each degree
	for (int degree = 0; degree < SIN_ACC; degree++) {
	  sin_values[degree] = sin(degree * M_PI / (float)(SIN_ACC/2)); // Convert degree to radian
	}
}

void TIM2_IRQHandler(void);


int main(void)
{

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SPI2_Init();


  TimerInit();
  TimerInterrupt();
  // Enable GPIOA clock
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
  //enable pin A5 output
  GPIOA->MODER &= ~(0b11 << 15*2);
  GPIOA->MODER |= (0b01 << 15*2);






	DAC_init();
  	setup_keypad();

  	build_sin_table();


  samp_per_cycle = SPS / freq;
  sin_increment = SIN_ACC / samp_per_cycle;
  ramp_increment = 3000 / samp_per_cycle;
  triangle_increment = 3000 / samp_per_cycle * 2;




  /* Infinite loop */
  while (1)
  {

	  if(keypad_pressed()){
		  val = id2char(scan_keypad());
		  if(val == '6' || val == '7' || val == '8' || val == '9'){
			  wave = val;
		  }
		  else if(val == '1' || val == '2' || val == '3' || val == '4' || val == '5' || val == '*' || val == '0' || val == '#'){
			  while(keypad_pressed());
			  com = val;
			  switch(com){
			  	  	  case '1': // 100Hz
			  	  		  	  freq = 100;
			  	              break;
			  	  	  case '2': // 200Hz
			  	  		  	  freq = 200;
			  	  		  	  break;
			  	  	  case '3': // 300Hz
			  	  		  	  freq = 300;
			   	  		  	  break;
			  	      case '4': // 400Hz
			  	    	  	  freq = 400;
			  	  	  	  	  break;
			  	      case '5': // 500Hz
			  	    	  	  freq = 500;
			  		  	  	  break;
			          case '*': // decrease duty cycle
			        	  	  if(duty_cycle > 10){duty_cycle = duty_cycle - 10;}
							  break;
			          case '0': // reset duty cycle
			        	  	  duty_cycle = 50;
							  break;
			          case '#': // increase duty cycle
			        	      if(duty_cycle < 90){duty_cycle = duty_cycle + 10;}
							  break;
			          default:
							  freq = 100;
							  duty_cycle = 50;
			  }

			  samp_per_cycle = SPS / freq;
			  sin_increment = SIN_ACC / samp_per_cycle;
			  ramp_increment = 3000 / samp_per_cycle;
			  triangle_increment = 3000 / samp_per_cycle * 2;
		  }
	  }

	  if (inc_samp_val) {samp_val++; inc_samp_val = 0;}

      do_DAC();



//	  switch(wave){
//			case '6': // square
//				if (samp_val < (samp_per_cycle * (0.01 * duty_cycle))){
//					voltage = 3000;
//				}
//				else {
//					voltage = 0;
//				}
//				break;
//			case '7': // sin
//				angle = round(sin_increment * samp_val);
//				if(angle >= 360){angle = 359;}
//				voltage = (1500 * sin_values[angle]) + 1500;
//				break;
//			case '8': // ramp
//				voltage = (ramp_increment * samp_val);
//				break;
//			case '9': // triangle
//				if (samp_val < (samp_per_cycle / 2)){
//					voltage = (triangle_increment * samp_val);
//				}
//				else {
//					voltage = 3000 - (triangle_increment * (samp_val - (samp_per_cycle / 2)));
//				}
//				break;
//	        default:
//	       	    voltage = 0;
//	  }
//
//
//	  //Relocated logic with other logic
//	  if (samp_val >= samp_per_cycle){
//		  samp_val = 0;
//	  }
//
//
//	  DAC_write(DAC_volt_conv(voltage));
//	  samp_val++;
//
//
//	  for(int i = 0; i < delay; i++);


	  // if(keypad_pressed()){
		 //  val = scan_keypad();
		 //  disp_LED(val);
	  // }





//	  DAC_write(voltage);
//	  disp_LED(val);
//
//	  DAC_write(voltage);

//	  DAC_write(0x000);
//	  HAL_Delay(1);
//	  for(int i = 0; i < delay; i++);
//	  DAC_write(0xFFF);
//	  HAL_Delay(1);
//	  for(int i = 0; i < delay; i++);





  }
}


void do_DAC(){
	  switch(wave){
			case '6': // square
				if (samp_val < (samp_per_cycle * (0.01 * duty_cycle))){
					voltage = 3000;
				}
				else {
					voltage = 0;
				}
				break;
			case '7': // sin
				angle = round(sin_increment * samp_val);
				if(angle >= SIN_ACC){angle = (SIN_ACC-1);}
				voltage = (1500 * sin_values[angle]) + 1500;
				break;
			case '8': // ramp
				voltage = (ramp_increment * samp_val);
				break;
			case '9': // triangle
				if (samp_val < (samp_per_cycle / 2)){
					voltage = (triangle_increment * samp_val);
				}
				else {
					voltage = 3000 - (triangle_increment * (samp_val - (samp_per_cycle / 2)));
				}
				break;
	        default:
	       	    voltage = 0;
	  }


	  //Relocated logic with other logic
	  if (samp_val >= samp_per_cycle){
		  samp_val = 0;
	  }


}




volatile int toggle = 1; //volatile for global variables used by interrupts!
void TIM2_IRQHandler(void) {
	if (TIM2->SR & TIM_SR_UIF) {TIM2->SR &= ~TIM_SR_UIF;}
	// Clear update interrupt flag (receive interrupt, prevents multiple triggers)

	if (toggle){GPIOA->BSRR = 1<<15; toggle = 0;}
	else {GPIOA->BRR = 1<<15; toggle = 1;}

//	PIN_W(GPIOA, 5, toggle);
//	PIN_W(GPIOA, 6, toggle);
//	PIN_W(GPIOA, 7, toggle);
//	PIN_W(GPIOA, 8, toggle);


	DAC_write(DAC_volt_conv(voltage));
	inc_samp_val = 1;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

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
