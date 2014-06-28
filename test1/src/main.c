//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

//#include <stdio.h>
//#include "diag/Trace.h"
#include "stm32f30x.h"
#include <stdlib.h>
#include <inttypes.h>
// ----------------------------------------------------------------------------
//
// Standalone STM32F3 empty sample (trace via NONE).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the NONE output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
/*
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"
*/

void sendChar(char c)
{
	while( !((USART1->ISR)&(1<<6)) );
	USART1->TDR = c;
}
void sendString(char *str)
{
	while(*str)
	{
		sendChar(*(str++));
	}
}

void sendULInt(unsigned long int x)
{
	char integer[15];
	char place=14;
	if(x==0)
	{
		sendChar('0');
	}
	while(x!=0)
	{
		integer[place]=(x%10)+48;
		x/=10;
		place--;
	}
	place++;
	while(place<=14)
	{
		sendChar(integer[place]);
		place++;
	}
}


int
main(/*int argc, char* argv[]*/)
{
  // At this stage the system clock should have already been configured
  // at high speed.

  // Infinite loop

	RCC->AHBENR |= RCC_AHBENR_GPIOEEN;          //Enable GPIOE clock


	GPIOE->MODER |= GPIO_MODER_MODER9_0;        //set MODER10[1:0] = [0 1] : general purpose output mode
	GPIOE->ODR |= GPIO_ODR_9;          			//set GPIOE_9 HIGH

	volatile int time = 0;

	//######### UART init ###########
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;          //Enable GPIOE clock
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;		//Enable USART1 peripheral clock
	GPIOC->MODER |= GPIO_MODER_MODER4_1;        //set MODER4[1:0] = [1 0] : alternate function mode
	GPIOC->MODER |= GPIO_MODER_MODER5_1;
	//set MODER5[1:0] = [1 0] : alternate function mode
	GPIOC->AFR[0] = (7<<16) | (7<<20); 			//set AFRL to AF7 at PC4 and PC5


	USART1->CR1 |= USART_CR1_RE | USART_CR1_TE;	//Enable Rx and Tx

	USART1->BRR = SystemCoreClock/9600;			//9600 baud at SystemCoreClock = 72 MHz
	USART1->CR1 |= USART_CR1_UE;				//Enable USART
	//###############################
	sendString("\n\r START \n\r");



	//######## GYRO init ############
	/*
	 * Pinout as follows:  USING SPI1
	 * SCK on PA5
	 * CS on PE3
	 * MOSI on PA7
	 * MISO on PA6
	 */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;          //Enable GPIOA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOEEN;          //Enable GPIOE clock

	GPIOA->MODER |= GPIO_MODER_MODER5_1;		//set PA5 as alternate function mode
	GPIOA->AFR[0] |= (5<<20);					//set PA5 as SPI1 SCK

	GPIOE->MODER |= GPIO_MODER_MODER3_0;		//set PE3 as GPIO for CS

	GPIOA->MODER |= GPIO_MODER_MODER7_1; 		//set PA7 as alternate function mode
	GPIOA->AFR[0] |= (5<<28);					//set PA7 as SPI1 MOSI

	GPIOA->MODER |= GPIO_MODER_MODER6_1;		//set PA6 as alternate function mode
	GPIOA->AFR[0] |= (5<<24);					//set PA6 as SPI1 MISO

	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;			//Enable SPI1 clock

	/*
	 * set SPI prescalar to 256
	 * set Master Mode
	 * set CPOL = 1 : clock idle high
	 * set CPHA = 1 : data driving on leading edge and sampling on trailing edge
	 */
	SPI1->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0 | SPI_CR1_MSTR | SPI_CR1_CPOL | SPI_CR1_CPHA;

	// set SPI data length of 8 bits
	SPI1->CR2 |= SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0;

	SPI1->CR1 |= SPI_CR1_SPE;					//Enable SPI1
	//###############################

	uint8_t spi_rx;
	GPIOE->ODR &= ~GPIO_ODR_3;

	for(time=0;time<=10000;time++);

	SPI1->DR = 0x0F | (1<<7) ;
	while( (SPI1->SR)&SPI_SR_BSY );
	spi_rx = SPI1->DR;

	SPI1->DR = 0;
	while( (SPI1->SR)&SPI_SR_BSY );
	//while( !((SPI1->SR)&SPI_SR_TXE) );

	GPIOE->ODR |= GPIO_ODR_3;
	spi_rx = SPI1->DR;

	while(1)
	{

		GPIOE->ODR &= ~GPIO_ODR_3;

			//for(time=0;time<=10000;time++);

			SPI1->DR = 0x0F | (1<<7) ;
			while( (SPI1->SR)&SPI_SR_BSY );
			spi_rx = SPI1->DR;

			SPI1->DR = 0;
			while( (SPI1->SR)&SPI_SR_BSY );
			//while( !((SPI1->SR)&SPI_SR_TXE) );

			GPIOE->ODR |= GPIO_ODR_3;
			spi_rx = SPI1->DR;






		sendULInt(spi_rx);
		sendString(" Hello, ARM STM32F3!\n\r");
	//	for(time=0;time<=1000000;time++);
	//	GPIOE->ODR ^= GPIO_ODR_9;

	}
	return 0;
}

//#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
