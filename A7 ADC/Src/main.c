#include "main.h"

void MSI_Overclock(void);
void ADC_Init(void);
void ADC_UART(void);

volatile uint16_t ADC_value = 0;	// global for interrupt access
volatile uint8_t  ADC_flag = 0;		// global for interrupt access

int main(void)
{
  HAL_Init();
  MSI_Overclock();
  ADC_Init();

  uint16_t ADC_Array[32];
  uint8_t sampleNumber = 0;
  int16_t minimumValue;
  int16_t maximumValue;
  int16_t averageValue;

  ADC1->SMPR1 = (0b000<<15); //2.5 clocks per sample
//  ADC1->SMPR1 = (0b100<<15); //47.5 clocks per sample
//  ADC1->SMPR1 = (0b111<<15); //640.5 clocks per sample

  ADC1->CR |= ADC_CR_ADSTART;		// start conversion

  while (1)
  {
	  if (ADC_flag == 1) {				// check global flag
		  ADC_flag = 0;					// clear flag

		  if (sampleNumber < 20){
			  ADC_Array[sampleNumber] = ADC_value;
			  sampleNumber++;
			  ADC1->CR |= ADC_CR_ADSTART;	// start a new conversion
		  }
	  }

	  if (sampleNumber == 20) {
		 //process the array to calculate the minimum, maximum, and average values

	     int32_t sum = 0;  // Use a larger type for accumulation to avoid overflow
	     minimumValue = ADC_Array[0];
	     maximumValue = ADC_Array[0];

	     for (int i = 1; i < 20; i++) {
	         if (ADC_Array[i] < minimumValue) {minimumValue = ADC_Array[i];}
	         if (ADC_Array[i] > maximumValue) {maximumValue = ADC_Array[i];}
	         sum += ADC_Array[i]; // Accumulate values for average calculation
	     }

	     averageValue = (int16_t)(sum / 20);  // Explicit cast to 16 bits

	     ADC_UART();

	     //start another 20 samples
		 ADC1->CR |= ADC_CR_ADSTART;	// start a new conversion
	  }


  }
}

void  ADC_UART(void){

    //UART Transmission goes here
	//
	//
	//
	//

}

void ADC1_2_IRQHandler(void){
	if (ADC1->ISR & ADC_ISR_EOC){	// check for ADC1 conversion
		ADC_value = ADC1->DR;
		ADC_flag = 1;
	}
}

void ADC_Init(void){
	RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;		// turn on clock for ADC
	// ADC will run at the same speed as CPU (HCLK / 1) since AHB prescaler is 1
	ADC123_COMMON->CCR = ((ADC123_COMMON->CCR & ~(ADC_CCR_CKMODE)) | ADC_CCR_CKMODE_0 );

	// power up ADC and voltage regulator
	ADC1->CR &= ~(ADC_CR_DEEPPWD);
	ADC1->CR |= (ADC_CR_ADVREGEN);
	for(uint16_t i = 0; i<1000; i++)				// delay at least 20us for ADC voltage regulator to power up
	  for(uint16_t j = 0; j<100; j++);

	// Calibrate ADC
	ADC1->CR &= ~(ADC_CR_ADEN | ADC_CR_ADCALDIF);	// ensure ADC is not enabled, single ended calibration
	ADC1->CR |= ADC_CR_ADCAL;						// start calibration
	while (ADC1->CR & ADC_CR_ADCAL);				// wait for calibration to finish

	// configure single ended mode before enabling ADC
	ADC1->DIFSEL &= ~(ADC_DIFSEL_DIFSEL_5);		// PA0 is ADC1_IN5, single ended mode

	// enable ADC
	ADC1->ISR |= (ADC_ISR_ADRDY);			// clear ADC Ready flag by writing a 1
	ADC1->CR |= ADC_CR_ADEN;				// enable ADC
	while(!(ADC1->ISR & ADC_ISR_ADRDY));	// wait for ADC Ready flag
	ADC1->ISR |= (ADC_ISR_ADRDY);			// clear ADC Ready flag by writing a 1

	// configure ADC
	ADC1->SQR1 = (ADC1->SQR1 & ~(ADC_SQR1_SQ1_Msk | ADC_SQR1_L_Msk)) 	// set sequence to 1 conversion on channel 5
				| (5 << ADC_SQR1_SQ1_Pos);

	// enable interrupts for ADC
	ADC1->IER |= (ADC_IER_EOC);		// interrupt after conversion
	ADC1->ISR &= ~(ADC_ISR_EOC);		// clear EOC flag
	NVIC->ISER[0] = (1 << (ADC1_2_IRQn & 0x1F));
	__enable_irq();

	// configure GPIO pin PA0
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);	// turn on clock for GPIOA (PA0 = ADC1_IN5)
	GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(GPIO_AFRL_AFSEL0)) | (7 << GPIO_AFRL_AFSEL0_Pos);
	GPIOA->MODER |= (GPIO_MODER_MODE0);    	// analog mode for PA0
	GPIOA->ASCR |= GPIO_ASCR_ASC0;			// set PA0 to analog
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


