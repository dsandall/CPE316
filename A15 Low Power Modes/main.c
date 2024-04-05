
#include "main.h"

void MSI_Overclock(void);
void enter_ISR_Sleep(void);
void enter_Stop0( void );

int main(void)
{
  HAL_Init();

  MSI_Overclock();
  enter_Stop0();

  while(1){
	  __asm__ __volatile__("nop");
  }


}


void enter_ISR_Sleep(void)
{
	//in this mode, the CPU sleeps whenever not handling ISRs
	//this is the simplest LP mode, basically only disabling the CPU, but leaving all periphs on
	//this mode's power consumption also depends on the current clock and clock speed, as it stays on
	//https://forum.digikey.com/t/low-power-modes-on-the-stm32l0-series/13306

	//TODO:check if this works?
    SCB->SCR &= ~( SCB_SCR_SLEEPDEEP_Msk );  // disable SLEEPDEEP bit (for more aggresive LP modes)
    SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;     // reenter low-power mode after ISR

    FLASH->ACR &= ~FLASH_ACR_SLEEP_PD;	//keep the flash memory in idle mode
    __WFI();  // enter low-power mode until interrupt recieved
}

void MSI_Overclock(void){

	//
	//Change MSI speed
	//
	uint8_t MSIRANGE_400khz =0b0010;
	uint8_t MSIRANGE_4Mhz =0b0110;
	uint8_t MSIRANGE_32Mhz = 0b1010;
	  if (RCC->CR & 0b1) { //if MSI clock is in RDY state
		  RCC->CR &= ~(0b1111 << 4); //clear MSI freq select register
		  RCC->CR |= (MSIRANGE_32Mhz << 4); //set it to desired Frequency
		  RCC->CR |= (0b1 << 3); //enable MSI frequency selection
	  }


	//Output MSI on MCO (Pin A8)
	  RCC->CFGR |= 0b100<<28; //prescale MCO by 16
	  RCC->CFGR |= 0b0001<<24; //SYSCLOCK on MCO
	  //RCC->CFGR & RCC_CFGR_SWS //system clock status register (read which clock is sysclock)
	  RCC->CFGR &= ~(0b11); //system clock switch (select which clock is sysclock)

	  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	  GPIOA->MODER &= ~(0b11<<(8*2)); //set port A8 to AF mode
	  GPIOA->MODER |= 0b10<<(8*2); //set port A8 to AF mode
	  GPIOA->AFR[0] &= ~(0b1111 << 0); //Clear AF select reg
	  GPIOA->AFR[0] |= (0b0000 << 0); //Port A8 to AFR 0

}


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
