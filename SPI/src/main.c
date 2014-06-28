//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include "stm32f30x.h"
#include "stm32f30x_spi.h"
#include "myGYRO.h"

#define BUFF0_SIZE 100
#define SCALING_FOR_INTEGRATION 7000

#define BUFF1_SIZE 10

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
	int place=14;
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

void sendLInt(long int x)
{
	char integer[15];
	int place=14;
	if(x==0)
	{
		sendChar('0');
	}
	if(x<0)
	{
		sendChar('-');
		x = -x;
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

void send16Hex(uint16_t data)
{
	char disp[4];
	int index=0;
	if(data == 0)
	{
		sendChar('0');
	}
	while(data)
	{
		disp[index] = data&0x0F;
		disp[index] += disp[index]<=9 ? '0' : 'A'-10;

		data = data>>4;
		index++;
	}
	while(index)
	{
		sendChar(disp[--index]);
	}
}



int main()
{
	// At this stage the system clock should have already been configured
	// at high speed.

	// Infinite loop
	spi1_init();

	RCC->AHBENR |= RCC_AHBENR_GPIOEEN;          //Enable GPIOE clock
	GPIOE->MODER |= GPIO_MODER_MODER3_0;		//set PE3 as GPIO for CS
	GPIOE->MODER |= GPIO_MODER_MODER9_0;        //set MODER10[1:0] = [0 1] : general purpose output mode

	volatile long int time;


	//######### UART init ###########
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;          //Enable GPIOE clock
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;		//Enable USART1 peripheral clock
	GPIOC->MODER |= GPIO_MODER_MODER4_1;        //set MODER4[1:0] = [1 0] : alternate function mode
	GPIOC->MODER |= GPIO_MODER_MODER5_1;
	//set MODER5[1:0] = [1 0] : alternate function mode
	GPIOC->AFR[0] = (7<<16) | (7<<20); 			//set AFRL to AF7 at PC4 and PC5


	USART1->CR1 |= USART_CR1_RE | USART_CR1_TE;	//Enable Rx and Tx

	//USART1->BRR = SystemCoreClock/9600;			//9600 baud at SystemCoreClock = 72 MHz
	USART1->BRR = SystemCoreClock/115200;			//115200 baud at SystemCoreClock = 72 MHz
	USART1->CR1 |= USART_CR1_UE;				//Enable USART
	sendString("\n\r#count sum integral\n\r");
	//###############################

	uint8_t spi_rxl, spi_rxh;
	//GYROwrite_byte(CTRL_REG1, 0xFC);

	//#### Initialize GYRO chip #####

	GYROwrite_byte(CTRL_REG1, 0xBF);
	GYROwrite_byte(CTRL_REG5, (1<<4));

	uint16_t Gyro_Xout_u;
	int16_t Gyro_Xout;
	int Gyro_integral=0;

	//int buffer[BUFF_SIZE];
	int index;
	int i;
	int sum0=0, avg0, index0=0;
	int16_t buff0[BUFF0_SIZE];

	int sum1=0, avg1=0, index1=0;
	int16_t buff1[BUFF1_SIZE];
	int GYRO_NULL = 213;
	int count = 0;
	for(i=0;i<BUFF0_SIZE;i++)
	{
		buff0[i] = 0;                 //initialise buff0 to 0
	}
	for(i=0;i<BUFF1_SIZE;i++)
	{
		buff1[i] = 0;                 //initialise buff0 to 0
	}
	while (1)
	{
		// Add your code here.

		//sum0 = 0;
		//for(i=0;i<BUFF0_SIZE;i++)
		//{

			while( !(GYROread_byte(STATUS_REG)&1) );

			spi_rxh = GYROread_byte(OUT_X_H);
			spi_rxl = GYROread_byte(OUT_X_L);

			Gyro_Xout_u = (spi_rxh<<8) | spi_rxl;
			Gyro_Xout = Gyro_Xout_u;

			//sum0 += (Gyro_Xout /*- GYRO_NULL*/);
			buff0[index0] = Gyro_Xout;       //load new value in buffer
			sum0 += (Gyro_Xout-avg1);

			index0++;
			if(index0 >= BUFF0_SIZE)
			{
				index0 = 0;
			}
			sum0 -= buff0[index0];            //remove the oldest value from sum

		//}


			avg0 = sum0/BUFF0_SIZE;

		//Gyro_integral += ((avg0*BUFF_SIZE)/SCALING_FOR_INTEGRATION);
		Gyro_integral += ((sum0/*-(BUFF0_SIZE*avg1)*/)/SCALING_FOR_INTEGRATION);

		buff1[index1] = avg0;
		sum1 += avg0;

		index1++;
		if(index1 >= BUFF1_SIZE)
		{
			index1 = 0;
		}
		sum1 -= buff1[index1];

		avg1 = sum1/BUFF1_SIZE;          //avg1 is not the dynamically calculated GYRO NULL;

	/*	BIAS_calc__counts++;
		if(BIAS_calc__counts>=BIAS_CALC_COUNTS)
		{
			BIAS_calc__counts = 1;
			BIAS = BIAS_sum/BIAS_CALC_COUNTS;
			BIAS_sum = 0;
		}
*/
		//for(time=0;time<=100000;time++);




		sendLInt(count++);

		sendChar(' ');
		//sendLInt(sample_sum);
		sendLInt(avg0);
		sendChar(' ');


		sendLInt(Gyro_integral);
		sendChar(' ');
		sendLInt(avg1);
		sendString("\n\r");

		GPIOE->ODR ^= GPIO_ODR_9;
	}
}



// ----------------------------------------------------------------------------
