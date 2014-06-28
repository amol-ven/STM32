/*
 * myGYRO.c
 *
 *  Created on: May 27, 2014
 *      Author: amol
 */

#include "stm32f30x.h"
#include "stm32f30x_spi.h"



void spi1_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOE, ENABLE);
        /* Configure IO for l3dg20 SPI SCK, MISO, MOSI *********************/

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_5); // SPI SCK
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_5); // SPI MISO
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_5); // SPI MOSI

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;//GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

    SPI_I2S_DeInit(SPI1);
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    //SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    //SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_Init(SPI1, &SPI_InitStructure);

    SPI_RxFIFOThresholdConfig(SPI1, SPI_RxFIFOThreshold_QF);

    SPI_Cmd(SPI1, ENABLE);
}

uint8_t spi1_transfer_byte(uint8_t byte)
{
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  /* Send a Byte through the SPI peripheral */
  SPI_SendData8(SPI1, byte);
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  return (uint8_t)SPI_ReceiveData8(SPI1);

}



void GYROwrite_byte(uint8_t address, uint8_t data)
{
	GPIOE->ODR &= ~GPIO_ODR_3;
	spi1_transfer_byte(address & 0b00111111);
	spi1_transfer_byte(data);
	GPIOE->ODR |= GPIO_ODR_3;
}

uint8_t GYROread_byte(uint8_t address)
{
	GPIOE->ODR &= ~GPIO_ODR_3;
	spi1_transfer_byte( (address|(1<<7))&~(1<<6) );
	uint8_t val = spi1_transfer_byte( 0 );
	GPIOE->ODR |= GPIO_ODR_3;
	return val;
}
