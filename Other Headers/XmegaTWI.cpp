/*
 * XmegaTWI.cpp
 *
 * Created: 17.07.2015 16:47:33
 *  Author: Robert
 */ 
#include "XmegaTWI.h"
//#include "twi_master_driver.h"

TWI_Master_t twiMaster;    /*!< TWI master module. */

void regRead(uint8_t slaveAddress, uint8_t *reg, uint8_t *buf, uint8_t count)
{
	//Writes which register to read from
	TWI_MasterWriteRead(&twiMaster, slaveAddress, reg, 1, count);
	
	/* Wait until transaction is complete. */
	while (twiMaster.status != TWIM_STATUS_READY) {}

	//Read in the received data
	for (int i = 0; i < count; i++)
	{
		*(buf+i) = twiMaster.readData[i];
	}
}

void regWrite(uint8_t slaveAddress, uint8_t reg, uint8_t val)
{
	uint8_t sendBuffer[2] = {reg, val};
	
	/* Wait until transaction is complete !!!!!! */
	while (twiMaster.status != TWIM_STATUS_READY) {}
	
	TWI_MasterWriteRead(&twiMaster, slaveAddress, sendBuffer, 2, 0);
	
	/* Wait until transaction is complete !!!!!! */
	while (twiMaster.status != TWIM_STATUS_READY) {}
}

void initTWI()
{
	TWI_MasterInit(&twiMaster, &TWIC, TWI_MASTER_INTLVL_LO_gc, TWI_BAUDSETTING);
}

void TWIInterrtuptHandler()
{
	TWI_MasterInterruptHandler(&twiMaster);
}