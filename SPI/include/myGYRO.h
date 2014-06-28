/*
 * myGYRO.h
 *
 *  Created on: May 27, 2014
 *      Author: amol
 */


#define WHO_AM_I 		0x0F
#define CTRL_REG1		0x20
#define CTRL_REG2		0x21
#define CTRL_REG3		0x22
#define CTRL_REG4		0x23
#define CTRL_REG5		0x24
#define REFERENCE		0x25
#define OUT_TEMP		0x26
#define STATUS_REG		0x27
#define OUT_X_L			0x28
#define OUT_X_H			0x29

void spi1_init(void);

uint8_t spi1_transfer_byte(uint8_t byte);

void GYROwrite_byte(uint8_t address, uint8_t data);


uint8_t GYROread_byte(uint8_t address);

