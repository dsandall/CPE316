/*
 * dac.c
 *
 *  Created on: Feb 12, 2024
 *      Author: chris
 */

#include "dac.h"
#include "gpio.h"
#include "main.h"
#include <math.h>
#include <stdint.h>


void DAC_init(){	// ~CS: B12  ~LDAC: B1  SCLK: B10  MOSI: C3
	pinMode('B', 1, 1); // set ~LDAC to output
	digitalWrite('B', 1, 1); // set ~LDAC to high
	RCC->APB1ENR1 |= RCC_APB1ENR1_SPI2EN;

	// Configure SCK pin (PB10) as alternate function mode
	GPIOB->MODER &= ~(GPIO_MODER_MODE10_Msk);
	GPIOB->MODER |= (0b10 << GPIO_MODER_MODE10_Pos); // Alternate function mode
	GPIOB->AFR[1] &= ~(0xF << (2 * 4)); // Clear bits for PB10
	GPIOB->AFR[1] |= (5 << (2 * 4)); // AF5 for SPI2

	// Configure MOSI pin (PB15) as alternate function mode
	GPIOB->MODER &= ~(GPIO_MODER_MODE15_Msk);
	GPIOB->MODER |= (0b10 << GPIO_MODER_MODE15_Pos); // Alternate function mode
	GPIOB->AFR[1] &= ~(0xF << (7 * 4)); // Clear bits for PB15
	GPIOB->AFR[1] |= (5 << (7 * 4)); // AF5 for SPI2

	// Configure CS pin (PB12)
	GPIOB->MODER &= ~(GPIO_MODER_MODE12_Msk);
	GPIOB->MODER |= (0b10 << GPIO_MODER_MODE12_Pos); // Alternate function mode
	GPIOB->AFR[1] &= ~(0xF << (4 * 4)); // Clear bits for PB12
	GPIOB->AFR[1] |= (5 << (4 * 4)); // AF5 for SPI2

	// Configure SPI2 peripheral
	SPI2->CR1 = 0; // Disable SPI2
	SPI2->CR1 = (SPI_CR1_CRCL | SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI); // Master mode, Software slave management, Internal slave select  | SPI_CR1_SSM | SPI_CR1_SSI
	SPI2->CR2 = 0; // Default configuration
	SPI2->CR2 = (SPI_CR2_NSSP | SPI_CR2_SSOE);

	SPI2->CR2 &= ~SPI_CR2_DS_Msk; // Clear data frame size bits
	SPI2->CR2 |= (0b1111 << SPI_CR2_DS_Pos); // 16-bit data frame size
}

void DAC_write(uint16_t data){	// outputs 2 bytes to DAC
	uint16_t transmitData = (0x7000 | data);

    SPI2->CR1 |= SPI_CR1_SPE; // Enable SPI2

    // Write data to the SPI data register
    SPI2->DR = transmitData;

    // Wait until SPI transmission is complete
    while (SPI2->SR & SPI_SR_BSY);

    SPI2->CR1 &= ~SPI_CR1_SPE; // Disable SPI2

	digitalWrite('B', 1, 0); // ~LDAC goes low
	digitalWrite('B', 1, 1); // ~LDAC returns to high
}

uint16_t DAC_volt_conv(uint16_t voltage){	// voltage is in mV
	  return (uint16_t)round((voltage * 4095) / 3300);
}

