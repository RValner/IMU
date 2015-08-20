/*
 * XmegaTWI.h
 *
 * Created: 17.07.2015 16:44:01
 *  Author: Robert
 */ 


#ifndef XMEGATWI_H_
#define XMEGATWI_H_

/* BAUDRATE 100kHz and Baudrate Register Settings */
#define BAUDRATE 100000
#define TWI_BAUDSETTING TWI_BAUD(F_CPU, BAUDRATE)

extern "C" 
{
	#include "twi_master_driver.h"
}

void regRead(uint8_t slaveAddress, uint8_t *reg, uint8_t *buf, uint8_t count);

void regWrite(uint8_t slaveAddress, uint8_t reg, uint8_t val);

void initTWI();

void TWIInterrtuptHandler();

#endif /* XMEGATWI_H_ */