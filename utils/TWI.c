/*
 * TWI.cpp
 *
 *  Created on: May 4, 2014
 *      Author: wowas
 */

#include "TWI.h"
#include <avr/io.h>


void TWIInit(void) {
	//set SCL to 400kHz
	TWSR = 0x00;
	TWBR = 0x0B;
	//enable TWI
	TWCR = (1 << TWEN);
}

void TWIStart(void) {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	while ((TWCR & (1 << TWINT)) == 0)
		;
}

void TWIStop(void) {
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

void TWIWrite(uint8_t u8data)
{
    TWDR = u8data;
    TWCR = (1<<TWINT)|(1<<TWEN);
    while ((TWCR & (1<<TWINT)) == 0);
}

uint8_t TWIReadACK(void)
{
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
    while ((TWCR & (1<<TWINT)) == 0);
    return TWDR;
}
//read byte with NACK
uint8_t TWIReadNACK(void)
{
    TWCR = (1<<TWINT)|(1<<TWEN);
    while ((TWCR & (1<<TWINT)) == 0);
    return TWDR;
}

uint8_t TWIGetStatus(void)
{
    uint8_t status;
    //mask status
    status = TWSR & 0xF8;
    return status;
}

/**
 * Reads one byte from a slave device.
 */
//uint8_t readByteGyro(uint8_t address) {
//	TWIStart(); // ST
//
//	TWIWrite(0xD6); // SAD + W
//
//	TWIWrite(address); // SUB
//
//	TWIStart(); // SR
//
//	TWIWrite(0xD7); // SAD + R
//
//	uint8_t data = TWIReadNACK(); // NMAK
//
//	TWIStop(); // SP
//
//	return data;
//}

//void writeByteGyro(uint8_t address, uint8_t subAddress, uint8_t data) {
//	TWIStart();
//
//	TWIWrite(address);
//
//	TWIWrite(subAddress);
//
//	TWIWrite(data);
//
//	TWIStop();
//}

