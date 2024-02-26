#include "main.h"


void I2C1_EV_IRQHandler(void){
	//page 1297, figure 406 (assumes N<=255)

	  while (1)
	  {
		  GPIOA->BSRR = (1<<5);
		  HAL_Delay(100);
		  GPIOA->BRR = (1<<5);
		  HAL_Delay(100);
	  }


	if(I2C1->ISR & I2C_ISR_NACKF){
		//NACK condition

//		  while (1)
//		  {
//			  GPIOA->BSRR = (1<<5);
//			  HAL_Delay(200);
//			  GPIOA->BRR = (1<<5);
//			  HAL_Delay(200);
//		  }

	}
	else if (I2C1->ISR & I2C_ISR_TXIS){
		//Byte successfully transferred condition


		//write more bytes


	}
}


int main(void)
{
  HAL_Init();

  // Enable MCO, select MSI (4 MHz source)
  //RCC->CFGR = ((RCC->CFGR & ~(RCC_CFGR_MCOSEL)) | (RCC_CFGR_MCOSEL_0));

//  //Change MSI to 48MHz mode (blazing fast overclock!!!)
//  RCC->CR &= ~(RCC_CR_MSIRANGE);
//  RCC->CR |= (0b1011 << 4);


  // Enable pin A5 for debug
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
  GPIOA->MODER &= ~(0x3 << 5*2);
  GPIOA->MODER |= (0x1 << 5*2);



////////I2C Stuffs:
  //enable I2C peripheral clock
  RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;

  // Enable GPIOB clock, set PB6 and 7 to alternate func
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
  GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL6_Pos | GPIO_AFRL_AFSEL7_Pos); //Clear AF select reg
  GPIOB->AFR[0] |= (4 << GPIO_AFRL_AFSEL6_Pos)|(4 << GPIO_AFRL_AFSEL7_Pos); //AF select to AF4 for pins B6 and B7
  GPIOB->MODER &= ~((0x3 << 6*2)|(0x3 << 7*2)); //set to AF mode
  GPIOB->MODER |= (0x2 << 6*2)|(0x2 << 7*2); //set to AF mode


//I2C->CR1
  I2C1->CR1 &= ~I2C_CR1_PE; //disable I2C for configuration

  //noise filter configuration cannot be done after enabling I2C
  I2C1->CR1 &= ~I2C_CR1_ANFOFF; //disable the analog filter disabler (enable analog filter)
  I2C1->CR1 &= ~I2C_CR1_DNF; //disable the digital filter

  I2C1->CR1 &= ~I2C_CR1_NOSTRETCH; //enable clock stretching (required for master mode)

  I2C1->CR1 |= I2C_CR1_TXIE | I2C_CR1_NACKIE; //enable interrupt on TXIS and NACK events

  NVIC->ISER[I2C1_EV_IRQn / 32] = (1 << (I2C1_EV_IRQn % 32)); //enable NVIC register

  //other CR1 options include numerous interrupts, noise filters, DMA options, automatic NACK for slave mode, and packet error check bit


//I2C->OAR1
//  I2C1->OAR1 |= I2C_OAR1_OA1EN; //enable Master ADDR 1
//
//  I2C1->OAR1 &= ~(I2C_OAR1_OA1MODE); //set to 7 bit mode
//
//  const uint8_t masterAddress = 0x1F; // 7 or 10 bit address
//  I2C1->OAR1 &= ~(I2C_OAR1_OA1); //clear master address
//  I2C1->OAR1 |=  (masterAddress); //set  Master ADDR 1

  //I2C_OAR2 is like OAR1, but with some limitations and strengths
  //it only supports 7 bit addresses, but you can choose to only compare the upper bits
  //this allows for many master addresses, where each slave device writes to it's own master address
  //effectively giving you virtual addresses that identify where the data came from
  //neat!


//tweak I2C->TIMINGR register
  /////Register MUST be changed before enabling I2C module
  //PRESC, SCLDEL, SDADEL (data setup and hold timing) (page 1277)
  //SCLH, SCLL (master clock config)(page 1292) (also has formula for master clock period)

  //values taken from page 1304 of the reference manual, with configs for common clock speeds
//  //these assume a 48MHz master clock speed, and operate in standard mode (100khz)
//
//  I2C1->TIMINGR |= (0xB << 28); //set PRESC
//  I2C1->TIMINGR |= (0x13 << 0); //set SCLL
//  I2C1->TIMINGR |= (0xF << 8); //set SCLH
//  I2C1->TIMINGR |= (0x2 << 16); //set SDADEL
//  I2C1->TIMINGR |= (0x4 << 20); //SCLDEL

  I2C1->TIMINGR &= ~(0xFFFFFFFF);
  I2C1->TIMINGR |= 0x00000E14; //(values from configuration tool)


//I2C->TIMEOUTR (timeout config)




//I2C->CR1 (again)
  I2C1->CR1 |= I2C_CR1_PE; //enable I2C (disabling performs software reset, section 39.4.6)

//I2C->PECR (Error checking)
//I2C->ISR, I2C->ICR (interrupts)


//I2C->CR2
  //I2C->CR2 &= ~(ADD10); // disable 10 bit mode (this is default)

//  const uint32_t slaveAddress = 0b0001010001; // 7 or 10 bit address to send data from STM to slave
  const uint32_t slaveAddress = 0b1010001; // 7 or 10 bit address to send data from STM to slave

  I2C1->CR2 &= ~(I2C_CR2_SADD);
  I2C1->CR2 |= (slaveAddress << 1);  //set slave address in I2C_CR2, shifted by 1 bit because B[0] isn't used

  //CR2 also includes START, STOP, NACK bits (cleared by hardware)
  //CR2 includes AUTOEND, RELOAD, and NBYTES (automatic stop/reload after NBYTES)
  //CR2 includes transfer direction RD_WRN




//I2C->RXDR, I2C->TXDR (ODR,IDR)
  //39.4.7 : Data Transfer

  //write to I2C operation
  I2C1->CR2 &= ~(I2C_CR2_RD_WRN); //set to 0 for write to I2C slave
  //in case of 10 bit address mode: need HEAD10R bit

  uint8_t nbytes = 1; // number of actual data bytes planned to send
  I2C1->CR2 &= ~(I2C_CR2_NBYTES);
  I2C1->CR2 |= (nbytes << 16);

  uint8_t TXData = 0xAE;
  I2C1->TXDR = TXData;


  GPIOA->BRR = (1<<5); //tst

  I2C1->CR2 |= I2C_CR2_START; //initialize start condition





  while (1)
  {
//	  GPIOA->BSRR = (1<<5);
//	  HAL_Delay(1000);
//	  GPIOA->BRR = (1<<5);
//	  HAL_Delay(1000);
	  HAL_Delay(10000);
	  I2C1->CR2 |= I2C_CR2_START; //initialize start condition

  }
}
