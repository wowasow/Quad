/*
 * TWI.h
 *
 *  Created on: May 4, 2014
 *      Author: wowas
 */

#ifndef TWI_H_
#define TWI_H_

#include<stdlib.h>
#include<stdio.h>


void TWIInit(void);
void TWIStart(void);
void TWIStop(void);
void TWIWrite(uint8_t u8data);
uint8_t TWIReadACK(void);
uint8_t TWIReadNACK(void);
uint8_t TWIGetStatus(void);

/**
 * Reads one byte from a slave device.
 * The register which the byte is read from
 * is specified by the address parameter.
 */
//uint8_t readByteGyro(uint8_t address);

/**
 * Writes a byte of data to a specific register.
 */
//void writeByteGyro(uint8_t address, uint8_t data);


#endif /* TWI_H_ */
